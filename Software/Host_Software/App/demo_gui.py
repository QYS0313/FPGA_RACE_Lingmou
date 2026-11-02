"""Initial integrated video / gimbal / laser desktop console."""
import csv
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog

from demo_link import DemoLink, ManualMotion, ROOT
from shape_session import ShapeSession
from vision import wait_for_hold
from serial.tools import list_ports
from gimbal_view import GimbalDashboard, apply_dark_theme, BG, TEXT, MUTED, LINE, PAN

# This embedded Python ships Tk scripts under a user profile that Tcl cannot
# always traverse in a restricted session. Use the matching local script copy.
_tk_scripts = ROOT/'Runtime/tcl'
if tk.TclVersion == 8.6 and (_tk_scripts/'tcl8.6/init.tcl').exists():
    os.environ['TCL_LIBRARY'] = str(_tk_scripts/'tcl8.6')
    os.environ['TK_LIBRARY'] = str(_tk_scripts/'tk8.6')


class Worker(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.tasks, self.events = queue.Queue(), queue.Queue()
        self.link = self.motion = None
        self.shapes = None
        self.requested = (0, 0)
        self.ui_tick = time.monotonic()
        self.poll_at = self.target_at = 0
        self.alive = True
        self.record = self.writer = None
        self.sequences = [None, None]
        self.pose_sequences = [None, None]
        self.pose_poll_at = 0
        folder = ROOT/'Logs'
        folder.mkdir(exist_ok=True)
        self.log = (folder/f'demo_{datetime.now():%Y%m%d_%H%M%S}.jsonl').open('a', encoding='utf8', buffering=1)

    def emit(self, kind, data):
        if kind == 'pose':
            data = dict(data, received_at=time.monotonic())
        self.events.put((kind, data))
        self.log.write(json.dumps(dict(time=datetime.now().isoformat(), kind=kind, data=data), ensure_ascii=False)+'\n')

    def stop_record(self):
        if self.record:
            self.record.close()
            self.record = self.writer = None
            self.emit('log', '记录已保存')

    def stop(self):
        if getattr(self, 'shapes', None):
            self.shapes.cancel()
        self.requested = (0, 0)
        if not self.link:
            return
        if self.link.image == 0xD1:
            if self.link.command(0x89) & 32:
                self.motion.disable()
                self.emit('log', '已中断回位并请求双轴失能')
            else:
                self.motion.stop()
        else:
            self.link.stop_tracking()
            if self.link.motion_status()['homing']:
                self.link.command(0x74, 2)
                self.emit('log', '已中断回位并请求双轴失能')

    def disconnect(self):
        self.stop_record()
        if self.link:
            try:
                self.stop()
            finally:
                self.link.ser.close()
                self.link = self.motion = None
                self.emit('connection', None)

    def emergency(self):
        if getattr(self, 'shapes', None):
            self.shapes.cancel()
        self.requested = (0, 0)
        if self.link:
            if self.link.image == 0xD1:
                self.motion.disable()
            else:
                self.link.stop_tracking()
                self.link.command(0x74, 2)

    def handle(self, action, value):
        if action == 'connect':
            if self.link:
                raise ValueError('请先断开当前连接')
            self.link = DemoLink(value)
            self.pose_sequences = [None, None]
            self.pose_poll_at = 0
            self.motion = ManualMotion(self.link, self.emit)
            self.shapes = ShapeSession(self.link, self.emit)
            self.emit('connection', self.link.image)
            self.emit('shape_capability', self.link.has_shapes)
            if self.link.image == 0xD1:
                self.emit('video_options', self.link.video_options())
            self.emit('log', f'已连接 {value}，镜像 {self.link.image:02X}，算法版本 {self.link.version:02X}')
            return
        if action == 'disconnect':
            self.disconnect()
            return
        if action == 'quit':
            try:
                self.disconnect()
            finally:
                self.alive = False
            return
        if not self.link:
            raise ValueError('请先连接板卡串口')
        if action == 'stop':
            self.stop()
        elif action == 'disable':
            self.emergency()
            self.emit('log', '已请求双轴失能')
        elif action == 'home':
            self.stop()
            if self.link.image == 0xD1:
                if self.link.command(0x87, 0xA5):
                    raise ValueError('回位请求被拒绝')
            else:
                state = self.link.motion_status()
                if state['homing']:
                    raise ValueError('正在回位，请等待；需要中断时点击双轴失能')
                if state['pan_homed'] and state['tilt_homed'] and not any(state[k] for k in ('disabled','home_fault','tracking_fault')):
                    wait_for_hold(self.link)
                self.link.command(0x74, 0)
                self.link.command(0x77, 0)
            self.emit('log', '双轴同时回位：下轴 116.924°，上轴 180°；到位后保持')
        elif action == 'video':
            self.link.need(0xD1)
            cmd, data = value
            if cmd==0x11 and data==5 and not getattr(self.link,'has_video_motion',False):
                raise ValueError('当前固件不支持帧差运动检测，请下载 D1/A8')
            reply = self.link.command(cmd, data)
            if cmd in (0x10, 0x11):
                # D1 acknowledges the actual base mode in response bits [6:4].
                self.emit('video_mode', (reply >> 4) & 7)
            self.emit('log', f'视频设置 {cmd:02X}={data}，返回 {reply}')
        elif action == 'video_motion':
            self.emit('video_motion', self.link.set_video_motion(*value))
            self.emit('log', '帧差参数已读回，正在重建参考帧')
        elif action == 'motion_reset':
            self.link.need(0xD1)
            if not getattr(self.link,'has_video_motion',False):
                raise ValueError('当前固件不支持帧差运动检测，请下载 D1/A8')
            self.link.command(0x7B)
            self.emit('log','已请求重建运动检测参考帧')
        elif action in ('video_options', 'read_video_options'):
            self.link.need(0xD1)
            if self.motion.moving or self.link.command(0x89) & 0x30:
                raise ValueError('请先松开方向键并停稳，再修改或读回整组视频参数')
            actual=self.link.set_video_options(*value) if action=='video_options' else self.link.video_options()
            self.emit('video_options',actual)
            if actual is not None:
                self.emit('log','视频参数已读回；新算法开关和选区在帧间生效')
        elif action == 'track':
            self.stop_record()
            self.stop()
            aim, residual = self.link.start_tracking(*value)
            self.emit('log', f'已启动跟踪，标定中心 {aim}，拟合误差 {residual:.1f} DAC')
        elif action == 'laser':
            self.link.need(0xD2)
            if getattr(self.link, 'shape_active', False):
                raise ValueError('当前为形状模式，请用静态描边按钮或先切回绿色光点跟踪')
            if value:
                self.link.require_homed()
                if not self.link.command(0x75) & 16:
                    raise ValueError('请先启动跟踪，再打开激光')
            self.link.command(0x72, 1 if value else 3)
            self.emit('log', '激光已打开（有效目标下）' if value else '激光已关闭，云台继续跟踪')
        elif action == 'detect':
            self.link.need(0xD2)
            self.stop()
            self.link.command(0x21, 1)
            self.link.command(0x52, value[1])
            self.link.command(0x53, value[0])
            self.emit('log', '只检测绿色光点；电机停止、激光关闭')
        elif action in ('shape_detect', 'shape_draw'):
            self.link.need(0xD2)
            self.stop_record()
            self.stop()
            self.shapes.begin(action=='shape_draw', *value)
        elif action == 'switch':
            if self.link.image == 0xE2:
                raise ValueError('E2 实验版不支持 Flash 切换，请先下载正式 D2 工程二')
            self.stop_record()
            self.stop()
            deadline = time.monotonic()+5
            if self.link.image == 0xD1:
                # Fresh reads allow switching even if both motors are disabled.
                for axis in (0, 1):
                    row = self.link.motor(axis)
                    if row['enabled']:
                        # Only capture an encoder position after actual settling.
                        samples = []
                        while time.monotonic() < deadline:
                            row = self.link.motor(axis)
                            now = time.monotonic()
                            if abs(row['speed_rpm']) > .3 or (samples and abs(row['angle_deg']-samples[0][1]) > .1):
                                samples.clear()
                            samples.append((now, row['angle_deg']))
                            if len(samples) >= 4 and now-samples[0][0] >= .4:
                                self.link.hold(axis, row['angle_deg'])
                                break
                            time.sleep(.04)
                        else:
                            raise TimeoutError('电机未停稳，未执行镜像切换')
            while not self.link.command(0x91) & 8:
                if time.monotonic() > deadline:
                    raise TimeoutError('尚未满足停稳联锁；若刚进入跟踪工程，请先双轴回位')
                time.sleep(.05)
            self.link.switch(value, report=lambda message: self.emit('log', message))
            self.motion = ManualMotion(self.link, self.emit)
            self.shapes = ShapeSession(self.link, self.emit)
            self.emit('connection', self.link.image)
            self.emit('shape_capability', self.link.has_shapes)
            if self.link.image == 0xD1:
                self.emit('video_options', self.link.video_options())
            self.emit('log', '镜像切换完成；需要运动时请点击双轴回位，跟踪需重新启动')
        elif action == 'record':
            self.link.need(0xD2)
            if self.record:
                self.stop_record()
            else:
                path = ROOT/'Logs'/f'tracking_{datetime.now():%Y%m%d_%H%M%S}.csv'
                self.record = path.open('x', newline='', encoding='utf8')
                self.writer = None
                self.sequences = [None, None]
                self.emit('log', f'开始记录：{path}')
        else:
            raise ValueError(f'Unknown action {action}')

    def poll_tracking_poses(self):
        # Read-only cached diagnostics: do not replay one FPGA sample as fresh.
        now = time.monotonic()
        if now < self.pose_poll_at:
            return
        self.pose_poll_at = now+(.02 if self.record else .1)
        for axis in (0, 1):
            row = self.link.diagnostic(axis)
            if not row:
                continue
            token = (row['sequence'], row['fpga_tick'])
            previous = self.pose_sequences[axis]
            if previous is not None and (token[0] == previous[0] or token[1] == previous[1]):
                continue
            self.pose_sequences[axis] = token
            self.emit('pose', dict(axis=axis, enabled=not row['disabled'], angle_deg=row['angle_deg'],
                                  speed_rpm=row['feedback_rpm'], current_a=row['current_a']))
            if self.record and self.sequences[axis] != row['sequence']:
                if not self.writer:
                    self.writer = csv.DictWriter(self.record, fieldnames=list(row))
                    self.writer.writeheader()
                self.writer.writerow(row)
                self.record.flush()
                self.sequences[axis] = row['sequence']

    def poll(self):
        if not self.link:
            return
        now = time.monotonic()
        if self.shapes and self.shapes.active:
            self.shapes.tick()
            self.poll_tracking_poses()
            return
        if self.link.image == 0xD1:
            self.link.command(0x88, 0x5A)
            wanted = self.requested if now-self.ui_tick < .4 else (0, 0)
            self.motion.tick(wanted)
            if now > self.poll_at:
                self.poll_at = now+.6
                status = self.link.command(0x89)
                self.emit('status', f'回位 下/上={status&1}/{(status>>1)&1}  回位中={bool(status&32)}  故障={bool(status&192)}')
                if not status & 0x30 and not self.motion.moving:
                    self.emit('video_stats', self.link.video_stats())
                    if getattr(self.link,'has_video_motion',False):
                        self.emit('video_motion', self.link.video_motion())
                    for axis in (0, 1):
                        self.emit('pose', self.link.motor(axis))
                self.poll_at = now+.6
        else:
            self.poll_tracking_poses()
            if now > self.poll_at:
                self.emit('status', str(self.link.motion_status()))
                self.emit('target', self.link.target())
                self.poll_at = now+.5

    def run(self):
        while self.alive:
            try:
                try:
                    action, value = self.tasks.get(timeout=.02)
                except queue.Empty:
                    action = None
                if action:
                    self.handle(action, value)
                self.poll()
            except Exception as exc:
                self.emit('error', str(exc))
                self.requested = (0, 0)
                if self.shapes and self.shapes.active:
                    self.shapes.cancel()
                    try:
                        self.link.stop_tracking()
                    except Exception as stop_error:
                        self.emit('error', f'形状停止指令未确认：{stop_error}')
                # Any failed motion transaction stops feeding the old request.
                if self.motion and self.motion.moving:
                    try:
                        self.emergency()
                    except Exception as stop_error:
                        self.emit('error', f'停止反馈不可用：{stop_error}；FPGA 断连停车兜底')
                # Close broken transports rather than repeatedly sending commands.
                if isinstance(exc, OSError) and self.link:
                    self.stop_record()
                    self.link.ser.close()
                    self.link = self.motion = None
                    self.emit('connection', None)
                time.sleep(.1)
        self.log.close()


class App:
    def __init__(self, root, worker=None):
        self.root = root
        self.worker = worker or Worker()
        self.image = None
        self.keys = set()
        self.closing = False
        root.title('灵眸 FPGA · 双轴云台与实时视觉控制台')
        width=min(1540,root.winfo_screenwidth()-60)
        height=min(970,root.winfo_screenheight()-100)
        root.geometry(f'{width}x{height}+30+20')
        root.minsize(1120,780)
        apply_dark_theme(root)
        outer = ttk.Frame(root, padding=(18,12))
        outer.pack(fill='both', expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(3, weight=1)
        heading=ttk.Frame(outer)
        heading.grid(row=0,column=0,sticky='ew',pady=(0,10))
        ttk.Label(heading,text='灵眸 / FPGA 视觉云台',font=('Microsoft YaHei UI',18,'bold')).pack(side='left')
        ttk.Label(heading,text='DUAL AXIS  /  VISION CONSOLE',foreground=MUTED,font=('Consolas',11)).pack(side='right')
        bar = ttk.Frame(outer)
        bar.grid(row=1, column=0, sticky='ew')
        self.port = tk.StringVar(value='COM5')
        self.ports = ttk.Combobox(bar, textvariable=self.port, width=12, values=[p.device for p in list_ports.comports()])
        self.ports.pack(side='left')
        for label, command in [('刷新', self.refresh_ports), ('连接', lambda:self.send('connect', self.port.get())),
                               ('断开', lambda:self.send('disconnect'))]:
            ttk.Button(bar, text=label, command=command).pack(side='left', padx=3)
        self.identity = tk.StringVar(value='未连接 · 设备视频从 HDMI 输出')
        ttk.Label(bar, textvariable=self.identity,foreground=PAN).pack(side='left', padx=12)
        switch = ttk.Frame(outer)
        switch.grid(row=2, column=0, sticky='ew', pady=8)
        self.flash_ready = tk.BooleanVar(value=False)
        ttk.Checkbutton(switch, text='已按说明烧入双镜像 Flash', variable=self.flash_ready).pack(side='left')
        for label, dest in [('切换到视频 / 手动云台', 0), ('切换到视觉 / 振镜跟踪', 1)]:
            ttk.Button(switch, text=label, command=lambda d=dest:self.switch(d)).pack(side='left', padx=5)
        self.tabs = ttk.Notebook(outer)
        self.tabs.grid(row=3, column=0, sticky='nsew')
        self.page_canvases = []
        video = self.scroll_page('① 视频算法与手动云台')
        tracking_layout=ttk.Frame(self.tabs)
        self.tabs.add(tracking_layout,text='② 光点跟踪 / 形状识别与描边')
        tracking_layout.columnconfigure(0,weight=0,minsize=480)
        tracking_layout.columnconfigure(1,weight=1)
        tracking_layout.rowconfigure(0,weight=1)
        track_controls=ttk.Frame(tracking_layout,width=480)
        track_controls.grid(row=0,column=0,sticky='nsew')
        track = self.scroll_page(None,track_controls)
        self.video_page(video)
        self.track_page(track)
        self.gimbal = GimbalDashboard(tracking_layout,ROOT/'Config/gimbal_model.json')
        self.gimbal.grid(row=0,column=1,sticky='nsew')
        self.tabs.select(tracking_layout)
        feedback = ttk.LabelFrame(outer, text='电机反馈（编码器绝对角度）', padding=7)
        feedback.grid(row=4, column=0, sticky='ew', pady=8)
        self.pose = [tk.StringVar(value=f'{name}：等待反馈') for name in ('下轴 PAN', '上轴 TILT')]
        for i,var in enumerate(self.pose):
            ttk.Label(feedback, textvariable=var).grid(row=0,column=i,sticky='w',padx=(0,24))
        self.status = tk.StringVar(value='上电及连接均不自动回位，不自动打开激光')
        ttk.Label(feedback, textvariable=self.status, wraplength=1300,foreground=MUTED).grid(row=1,column=0,columnspan=2,sticky='w')
        self.messages = tk.Text(outer, height=3, state='disabled', font=('Consolas', 9),
                                bg='#0d1726',fg=MUTED,insertbackground=TEXT,
                                highlightbackground=LINE,highlightthickness=1,relief='flat',padx=10,pady=5)
        self.messages.grid(row=5, column=0, sticky='ew')
        root.bind('<KeyPress>', self.key_down)
        root.bind('<KeyRelease>', self.key_up)
        root.bind('<FocusOut>', lambda e:root.after(50, self.check_focus))
        self.tabs.bind('<<NotebookTabChanged>>', lambda e:self.release())
        root.protocol('WM_DELETE_WINDOW', self.close)
        if worker is None:
            self.worker.start()
        root.after(50, self.update)

    def scroll_page(self, title, parent=None):
        page = parent if parent is not None else ttk.Frame(self.tabs)
        if parent is None:
            self.tabs.add(page, text=title)
        canvas = tk.Canvas(page, height=400, highlightthickness=0, background=BG)
        scroll = ttk.Scrollbar(page, orient='vertical', command=canvas.yview)
        scroll.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)
        canvas.configure(yscrollcommand=scroll.set)
        inner = ttk.Frame(canvas, padding=12)
        window = canvas.create_window((0, 0), window=inner, anchor='nw')
        inner.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda e: canvas.itemconfigure(window, width=e.width))
        self.page_canvases.append(canvas)
        canvas.bind('<MouseWheel>',lambda e,c=canvas:c.yview_scroll(-int(e.delta/120)*3,'units'))
        inner.bind('<MouseWheel>',lambda e,c=canvas:c.yview_scroll(-int(e.delta/120)*3,'units'))
        return inner

    def send(self, action, value=None):
        if action in ('home', 'stop', 'disable', 'switch', 'disconnect', 'quit'):
            self.release()
        self.worker.tasks.put((action, value))

    def refresh_ports(self):
        self.ports['values'] = [p.device for p in list_ports.comports()]

    def log(self, text):
        self.messages.configure(state='normal')
        self.messages.insert('end', f'{datetime.now():%H:%M:%S}  {text}\n')
        self.messages.see('end')
        self.messages.configure(state='disabled')

    def video_page(self, frame):
        modes = ttk.LabelFrame(frame, text='视频算法（画面从 HDMI 输出）', padding=8)
        modes.pack(fill='x')
        self.video_mode=tk.IntVar(value=0)
        for mode, label in enumerate(('原图', '灰度', '二值化', 'Sobel 边缘', '颜色目标跟踪', '帧差运动检测')):
            ttk.Radiobutton(modes, text=label, variable=self.video_mode, value=mode,
                            command=lambda m=mode:self.send('video', (0x11, m))).grid(row=0, column=mode, padx=4)
        motion=ttk.LabelFrame(frame,text='帧差检测 · 固定视角 / 最多3个运动区域',padding=6)
        motion.pack(fill='x',pady=4)
        self.motion_threshold=tk.IntVar(value=18)
        self.motion_minimum=tk.IntVar(value=3)
        ttk.Label(motion,text='差分阈值').grid(row=0,column=0)
        ttk.Spinbox(motion,from_=1,to=255,width=5,textvariable=self.motion_threshold).grid(row=0,column=1)
        ttk.Label(motion,text='最小区域块数').grid(row=0,column=2,padx=6)
        ttk.Spinbox(motion,from_=1,to=32,width=5,textvariable=self.motion_minimum).grid(row=0,column=3)
        ttk.Button(motion,text='应用参数',command=self.apply_video_motion).grid(row=0,column=4,padx=6)
        ttk.Button(motion,text='重建参考帧',command=lambda:self.send('motion_reset')).grid(row=0,column=5)
        self.motion_info=tk.StringVar(value='选择“帧差运动检测”启用。噪点多可提高阈值，漏检可降低。')
        ttk.Label(motion,textvariable=self.motion_info,wraplength=820).grid(row=1,column=0,columnspan=6,sticky='w')
        ttk.Label(motion,text='青色框标出运动区域；停止后短暂保持再消失。云台移动时暂停；支持下方基础算法的全屏/矩形选区。',wraplength=820).grid(row=2,column=0,columnspan=6,sticky='w')
        colors = ttk.Frame(frame)
        colors.pack(fill='x', pady=8)
        ttk.Label(colors, text='目标颜色').pack(side='left')
        for i, label in enumerate(('红', '绿', '蓝')):
            ttk.Button(colors, text=label, command=lambda c=i:self.send('video', (0x21, c))).pack(side='left', padx=3)
        for label, val in [('OSD 开', 1), ('OSD 关', 0)]:
            ttk.Button(colors, text=label, command=lambda v=val:self.send('video', (0x10, v))).pack(side='left', padx=3)
        fx = ttk.LabelFrame(frame, text='附加特效（可与基础算法叠加）', padding=8)
        fx.pack(fill='x')
        self.video_effect=tk.IntVar(value=0)
        for i, label in enumerate(('关闭', '仅轮廓', '马赛克', '卡通', '贴图', '蓝色反相')):
            ttk.Radiobutton(fx, text=label, variable=self.video_effect, value=i,
                            command=lambda f=i:self.send('video', (0x64, f))).pack(side='left', padx=3)
        advanced=ttk.LabelFrame(frame,text='高级处理开关（修改后点击“应用并读回”）',padding=8)
        advanced.pack(fill='x',pady=6)
        self.algorithm_flags=[tk.BooleanVar(value=bool(0x2F&(1<<i))) for i in range(6)]
        for i,label in enumerate(('自动白平衡','色调曲线 / 高光压缩','降噪','饱和度增强','亮度直方图均衡','明暗均匀校正')):
            ttk.Checkbutton(advanced,text=label,variable=self.algorithm_flags[i]).grid(row=i//3,column=i%3,sticky='w',padx=8,pady=3)
        ttk.Label(advanced,text='白平衡、色调、降噪、饱和度与明暗校正作用于全图；直方图均衡可选作用范围。',wraplength=820).grid(row=2,column=0,columnspan=3,sticky='w')
        area=ttk.LabelFrame(frame,text='算法作用范围与色标选区',padding=8)
        area.pack(fill='x')
        self.scope_vars=[tk.StringVar(value=v) for v in ('全屏','选定区域','全屏')]
        for i,label in enumerate(('基础算法','附加特效','直方图均衡')):
            ttk.Label(area,text=label).grid(row=0,column=i*2,sticky='w')
            ttk.Combobox(area,textvariable=self.scope_vars[i],values=('全屏','选定区域'),state='readonly',width=10).grid(row=0,column=i*2+1,padx=5)
        self.region_source=tk.StringVar(value='手动矩形')
        ttk.Label(area,text='选区来源').grid(row=1,column=0,sticky='w',pady=6)
        ttk.Combobox(area,textvariable=self.region_source,values=('手动矩形','目标检测框'),state='readonly',width=12).grid(row=1,column=1,sticky='w')
        self.region_union=tk.BooleanVar(value=True);self.region_locked=tk.BooleanVar(value=False)
        ttk.Checkbutton(area,text='多点选区（2点矩形 / 4点四边形）',variable=self.region_union).grid(row=1,column=2,columnspan=2,sticky='w')
        ttk.Checkbutton(area,text='锁定色标选区',variable=self.region_locked).grid(row=1,column=4,columnspan=2,sticky='w')
        rect=ttk.Frame(area);rect.grid(row=2,column=0,columnspan=6,sticky='w')
        self.rect_vars=[tk.IntVar(value=v) for v in (320,180,959,539)]
        for i,label in enumerate(('X0','Y0','X1','Y1')):
            ttk.Label(rect,text=label).pack(side='left',padx=4)
            ttk.Spinbox(rect,from_=0,to=1279 if i%2==0 else 719,textvariable=self.rect_vars[i],width=6).pack(side='left')
        ttk.Button(area,text='应用并读回',command=self.apply_video_options).grid(row=3,column=0,columnspan=2,sticky='ew',pady=6)
        ttk.Button(area,text='读取当前参数',command=lambda:self.send('read_video_options')).grid(row=3,column=2,columnspan=2,sticky='ew')
        self.video_options_status=tk.StringVar(value='等待读取板卡参数；手动矩形坐标含两端边界')
        ttk.Label(area,textvariable=self.video_options_status,wraplength=820).grid(row=4,column=0,columnspan=6,sticky='w')
        ttk.Label(area,text='选择“目标检测框”并启用多点选区：四个同色标记围成四边形，可锁定后移走标记。\n四边形作用于附加特效；基础算法和直方图均衡仍使用外接矩形。',wraplength=820).grid(row=5,column=0,columnspan=6,sticky='w')
        jog = ttk.LabelFrame(frame, text='手动云台 · 按住 WASD / 方向键或按钮，松开减速停车', padding=8)
        jog.pack(fill='x', pady=10)
        self.speed = tk.DoubleVar(value=2)
        ttk.Label(jog, text='速度 rpm').grid(row=0, column=0)
        ttk.Spinbox(jog, from_=.2, to=10, increment=.2, textvariable=self.speed, width=7).grid(row=0, column=1)
        for label, key, col, row in [('上 ↑', 'w', 3, 0), ('左 ←', 'a', 2, 1), ('右 →', 'd', 4, 1), ('下 ↓', 's', 3, 2)]:
            b = ttk.Button(jog, text=label)
            b.grid(row=row, column=col, padx=5, pady=2)
            b.bind('<ButtonPress-1>', lambda e,k=key:self.keys.add(k))
            b.bind('<ButtonRelease-1>', lambda e:self.release())
            b.bind('<Leave>', lambda e:self.release())
        ttk.Button(jog, text='双轴回位 / 使能', command=lambda:self.send('home')).grid(row=1, column=0, columnspan=2, sticky='ew')
        ttk.Button(jog, text='停止并保持', command=lambda:self.send('stop')).grid(row=2, column=0, columnspan=2, sticky='ew')
        ttk.Button(jog, text='双轴失能', command=lambda:self.send('disable')).grid(row=1, column=5, padx=20)
        ttk.Label(frame, text='逻辑零位：下轴 116.924°、上轴 180°。回位后：下轴 ±60°，上轴 150°～300°。\n本工程光源保持关闭。失能后上轴可能因重心偏移下落。').pack(anchor='w')
        self.video_numbers = tk.StringVar(value='视频统计：等待连接新版视频工程')
        ttk.Label(frame, textvariable=self.video_numbers, wraplength=820).pack(anchor='w', pady=6)

    def apply_video_motion(self):
        try:
            threshold,minimum=self.motion_threshold.get(),self.motion_minimum.get()
            if not (1<=threshold<=255 and 1<=minimum<=32):
                raise ValueError('差分阈值需为1～255，最小区域块数需为1～32')
            self.send('video_motion',(threshold,minimum))
        except (ValueError,tk.TclError) as exc:
            self.log('参数错误：'+str(exc))

    def apply_video_options(self):
        try:
            flags=sum(int(v.get())<<i for i,v in enumerate(self.algorithm_flags))
            scopes=sum(int(v.get()=='选定区域')<<i for i,v in enumerate(self.scope_vars))
            scopes |= int(self.region_source.get()=='目标检测框')<<3
            rect=tuple(v.get() for v in self.rect_vars)
            if not(0<=rect[0]<=rect[2]<1280 and 0<=rect[1]<=rect[3]<720):
                raise ValueError('矩形需满足 0≤X0≤X1≤1279、0≤Y0≤Y1≤719')
            self.release()
            self.send('video_options',(flags,scopes,rect,self.region_union.get(),self.region_locked.get()))
        except (ValueError,tk.TclError) as exc:
            self.log(f'参数错误：{exc}')

    def track_page(self, frame):
        ttk.Label(frame,text='光点追踪控制',font=('Microsoft YaHei UI',15,'bold')).pack(anchor='w',pady=(0,3))
        ttk.Label(frame,text='跟踪操作与双轴实际姿态同步显示',foreground=MUTED).pack(anchor='w',pady=(0,12))
        self.calibration = tk.StringVar(value=str(ROOT/'Config/calibration.json'))
        ttk.Label(frame,text='视觉标定文件').pack(anchor='w')
        row = ttk.Frame(frame)
        row.pack(fill='x',pady=(4,12))
        ttk.Entry(row, textvariable=self.calibration,width=22).pack(side='left', fill='x', expand=True, padx=(0,5))
        ttk.Button(row, text='选择…', command=self.pick_calibration).pack(side='right')
        params = ttk.LabelFrame(frame,text='光点检测参数',padding=10)
        params.pack(fill='x', pady=(0,12))
        self.brightness, self.chroma, self.radius = tk.IntVar(value=160), tk.IntVar(value=40), tk.IntVar(value=10)
        for i,(label, var, low, high) in enumerate([('亮度', self.brightness, 0, 255), ('色差', self.chroma, 1, 255), ('框半径 px', self.radius, 4, 64)]):
            params.columnconfigure(i,weight=1)
            ttk.Label(params, text=label).grid(row=0,column=i,sticky='w')
            ttk.Spinbox(params, from_=low, to=high, textvariable=var, width=7).grid(row=1,column=i,sticky='ew',padx=(0,6),pady=5)
        self.laser = tk.BooleanVar(value=True)
        ttk.Checkbutton(params, text='启动时打开红色激光框', variable=self.laser).grid(row=2,column=0,columnspan=3,sticky='w',pady=(4,0))
        controls = ttk.Frame(frame)
        controls.pack(fill='x')
        controls.columnconfigure((0,1),weight=1)
        actions = [('双轴回位 / 使能', lambda:self.send('home')), ('开始跟踪', self.start_track),
                   ('停止跟踪 / 关激光', lambda:self.send('stop')), ('双轴失能', lambda:self.send('disable'))]
        for i,(label, command) in enumerate(actions):
            ttk.Button(controls, text=label, command=command).grid(row=i//2,column=i%2,sticky='ew',padx=(0,6),pady=3)
        extra = ttk.Frame(frame)
        extra.pack(fill='x', pady=6)
        extra.columnconfigure((0,1),weight=1)
        for i,(label, action, val) in enumerate([('激光开', 'laser', True), ('仅关激光', 'laser', False), ('开始 / 停止记录', 'record', None)]):
            ttk.Button(extra, text=label, command=lambda a=action,v=val:self.send(a,v)).grid(row=i//2,column=i%2,sticky='ew',padx=(0,6),pady=3)
        ttk.Button(extra, text='只检测光点', command=self.detect).grid(row=1,column=1,sticky='ew',padx=(0,6),pady=3)
        self.target = tk.StringVar(value='目标：等待检测')
        ttk.Label(frame, textvariable=self.target,wraplength=420,foreground=PAN).pack(anchor='w', pady=8)
        self.shape_page(frame)
        ttk.Label(frame, text='云台对准标定中心，振镜补偿并画框。移动中保持红框；目标丢失、故障或停止时关闭光源。\n更换屏幕距离或相机 / 振镜位置后，需重新视觉标定。', wraplength=420,foreground=MUTED).pack(anchor='w',pady=8)

    def shape_page(self, frame):
        panel = ttk.LabelFrame(frame, text='颜色形状识别 / 静态轮廓描边 · E7', padding=10)
        panel.pack(fill='x', pady=(8,0))
        self.shape_color = tk.StringVar(value='自动')
        self.shape_seconds = tk.IntVar(value=20)
        row = ttk.Frame(panel);row.pack(fill='x',pady=(0,8))
        ttk.Label(row, text='目标颜色').pack(side='left')
        ttk.Combobox(row, state='readonly', textvariable=self.shape_color,
                     values=('自动','红色','绿色','蓝色'), width=5).pack(side='left', padx=6)
        ttk.Label(row, text='时长 / 秒').pack(side='left')
        ttk.Spinbox(row, from_=1, to=3600, textvariable=self.shape_seconds, width=5).pack(side='left', padx=6)
        buttons=ttk.Frame(panel);buttons.pack(fill='x')
        buttons.columnconfigure((0,1),weight=1)
        self.shape_buttons=[]
        for i,(label,action) in enumerate([('只检测形状','shape_detect'),('开始静态描边','shape_draw')]):
            button=ttk.Button(buttons,text=label,command=lambda a=action:self.start_shape(a),state='disabled')
            button.grid(row=0,column=i,sticky='ew',padx=(0,4),pady=3);self.shape_buttons.append(button)
        ttk.Button(buttons,text='停止 / 关光',command=lambda:self.send('stop')).grid(row=1,column=0,columnspan=2,sticky='ew',pady=3)
        self.shape_phase=tk.StringVar(value='连接新版工程二后启用')
        self.shape_result=tk.StringVar(value='形状：等待检测')
        self.shape_diagnostic=tk.StringVar(value='实时诊断：等待采样')
        for var in (self.shape_phase,self.shape_result,self.shape_diagnostic):
            ttk.Label(panel,textvariable=var,wraplength=400).pack(anchor='w',pady=3)
        ttk.Label(panel,text='识别后锁定轮廓；移走图案仍画原位置，换目标前先停止。实时识别失败不取消描边。',
                  wraplength=400,foreground=MUTED).pack(anchor='w',pady=(4,0))

    def start_shape(self, action):
        try:
            color={'自动':'auto','红色':'red','绿色':'green','蓝色':'blue'}[self.shape_color.get()]
            seconds=self.shape_seconds.get()
            if not 1<=seconds<=3600:raise ValueError('描边时长须为 1～3600 秒')
            self.send(action,(self.calibration.get(),color,seconds))
        except (ValueError,KeyError,tk.TclError) as exc:
            self.log(f'形状参数错误：{exc}')

    def pick_calibration(self):
        path = filedialog.askopenfilename(filetypes=[('标定 JSON', '*.json')])
        if path:
            self.calibration.set(path)

    def start_track(self):
        try:
            self.send('track', (self.calibration.get(), self.brightness.get(), self.chroma.get(), self.radius.get(), self.laser.get()))
        except (ValueError, tk.TclError):
            self.log('请输入有效参数')

    def detect(self):
        try:
            self.send('detect', (self.brightness.get(), self.chroma.get()))
        except (ValueError, tk.TclError):
            self.log('请输入有效参数')

    def switch(self, destination):
        if not self.flash_ready.get():
            self.log('先按 demo_system/使用说明.txt 写入双镜像 Flash，再勾选上方选项；首次也可直接用 JTAG 下载单个镜像')
            return
        self.send('switch', destination)

    def key_down(self, event):
        if event.keysym == 'Escape':
            self.send('stop')
            return
        if isinstance(event.widget, (ttk.Entry, ttk.Spinbox, ttk.Combobox, tk.Text)):
            return
        key = dict(Up='w', Down='s', Left='a', Right='d').get(event.keysym, event.keysym.lower())
        if key in 'wasd' and len(key) == 1:
            self.keys.add(key)

    def key_up(self, event):
        self.keys.discard(dict(Up='w', Down='s', Left='a', Right='d').get(event.keysym, event.keysym.lower()))

    def release(self):
        self.keys.clear()
        self.worker.requested = (0, 0)

    def check_focus(self):
        if self.root.focus_displayof() is None:
            self.release()

    def update(self):
        if self.closing and not self.worker.is_alive():
            self.root.destroy()
            return
        self.worker.ui_tick = time.monotonic()
        try:
            v = max(.2, min(10, self.speed.get()))
        except (ValueError, tk.TclError):
            v = 0
        if self.image == 0xD1 and self.tabs.index('current') == 0 and not self.closing:
            self.worker.requested = (v*(('a' in self.keys)-('d' in self.keys)), v*(('w' in self.keys)-('s' in self.keys)))
        else:
            self.worker.requested = (0, 0)
        while True:
            try:
                kind, data = self.worker.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'connection':
                self.image = data
                self.gimbal.set_connection(data)
                for axis,name in enumerate(('下轴 PAN','上轴 TILT')):
                    self.pose[axis].set(f'{name}：等待反馈')
                self.video_options_status.set('等待读取板卡参数' if data==0xD1 else '未连接视频工程')
                self.video_numbers.set('视频统计：等待采样（手动移动、回位时暂停串口刷新，HDMI 持续显示）' if data == 0xD1 else '视频统计：未连接视频工程')
                self.release()
                self.identity.set({None:'未连接', 0xD1:'D1 · 视频 + 手动云台', 0xD2:'D2 · 光点跟踪 + 形状描边', 0xE2:'E2 · 形状实验版（无 Flash 切换）'}[data])
                for button in self.shape_buttons:button.configure(state='disabled')
                self.shape_phase.set('等待能力查询' if data else '未连接')
            elif kind == 'shape_capability':
                for button in self.shape_buttons:button.configure(state='normal' if data else 'disabled')
                self.shape_phase.set('E7 已就绪' if data else '当前镜像无 E7 形状功能')
            elif kind == 'shape_phase':
                self.shape_phase.set(data)
            elif kind == 'shape':
                t=data['target']
                self.shape_result.set(f'{data["color"]} · {data["shape"]}  有效 {data["valid"]}  '
                                      f'中心 ({t["x"]}, {t["y"]})  尺寸 {t["width"]} × {t["height"]}')
                self.shape_diagnostic.set(f'锁定 {data["locked"]} / 请求描边 {data["requested"]} / 出光采样 {data["laser_now"]}\n'
                    f'实时：{data["raw_color"]} {data["raw_shape"]}，有效 {data["raw_valid"]}；'
                    f'完整坐标 {data["full_range"]}，超出旧窗口 {data["range_fault"]}，'
                    f'溢出 {data["overflow"]} / 超期 {data["late"]}')
            elif kind == 'pose':
                axis = data['axis']
                self.gimbal.feed(data)
                self.pose[axis].set(f'{("下轴 PAN", "上轴 TILT")[axis]}  {data["angle_deg"]:.2f}°   {data["speed_rpm"]:.2f} rpm   {data["current_a"]:.3f} A   使能 {data["enabled"]}')
            elif kind == 'status':
                self.status.set(data)
            elif kind == 'target':
                self.target.set(f'目标有效 {data["valid"]}  中心 ({data["x"]}, {data["y"]})  尺寸 {data["width"]} × {data["height"]}')
            elif kind == 'video_options':
                if data is None:
                    self.video_options_status.set('此视频固件不支持高级开关和范围选择，请下载新版位流')
                else:
                    for i,v in enumerate(self.algorithm_flags):v.set(bool(data['flags']&(1<<i)))
                    for i,v in enumerate(self.scope_vars):v.set('选定区域' if data['scopes']&(1<<i) else '全屏')
                    self.region_source.set('目标检测框' if data['scopes']&8 else '手动矩形')
                    for v,n in zip(self.rect_vars,data['rect']):v.set(n)
                    self.region_union.set(data['union']);self.region_locked.set(data['locked'])
                    self.video_mode.set(data['mode']);self.video_effect.set(data['effect'])
                    self.video_options_status.set('已读回。'+('支持四点四边形特效选区。' if data.get('quad_supported') else '当前固件不含四点四边形，需下载 A2/C2 新版。')+'未锁定时丢失目标会在短暂保持后清除选区。')
            elif kind == 'video_motion':
                if data is None:
                    self.motion_info.set('当前固件不支持帧差检测，请下载 D1/A8')
                else:
                    state='云台移动，暂停' if data['paused'] else '建立参考帧' if data['warming'] else '全局变化抑制' if data['scene_change'] else '检测中'
                    self.motion_info.set(f"{state} | 运动区域 {data['count']} 个 | 变化采样点 {data['changed_cells']} | 已生效阈值 {data['threshold']} / 最小块数 {data['min_tiles']}")
            elif kind == 'video_stats':
                if data is None:
                    self.video_numbers.set('当前视频固件未提供数值统计，请下载新版 demo_video.bit')
                else:
                    target = f'中心 ({data["x"]}, {data["y"]})  尺寸 {data["width"]} × {data["height"]} px' if data['valid'] else '未检测到目标'
                    self.video_numbers.set(
                        f'{target}  |  采集显示 {data["camera_fps"]+15} FPS（实测 {data["camera_fps"]}+15） / 输出 {data["display_fps"]} FPS\n'
                        f'采集帧计数 {data["camera_frames"]} / 显示帧计数 {data["display_frames"]}（65536 回绕）'
                        f'  |  二值阈值 {data["binary_threshold"]} / 边缘底噪 {data["edge_floor"]}')
            else:
                if kind == 'error':
                    self.release()
                self.log(('错误：' if kind == 'error' else '')+str(data))
        self.gimbal.refresh()
        self.root.after(50, self.update)

    def close(self):
        if not self.closing:
            self.closing = True
            self.log('正在停止运动并断开串口…')
            self.send('quit')


if __name__ == '__main__':
    root = tk.Tk()
    App(root)
    root.mainloop()
