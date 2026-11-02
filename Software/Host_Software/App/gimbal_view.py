"""Local, dependency-free two-axis gimbal viewport. Never sends motor commands."""
from collections import deque
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk
from gimbal_gl import DetailedGimbalCanvas

BG = '#080f1d'
PANEL = '#101c2d'
LINE = '#26384d'
TEXT = '#e4edf7'
MUTED = '#8396ae'
PAN = '#4bd9d2'
TILT = '#ae9bff'
AMBER = '#f3be6b'
HOME = (21285*360/65536, 180.0)


def angle_delta(angle, reference):
    """Shortest encoder displacement, including the 359 -> 0 wrap."""
    return (angle-reference+180.0) % 360.0-180.0


@dataclass
class AxisFeedback:
    angle: float = 0.0
    speed: float = 0.0
    current: float = 0.0
    enabled: bool = False
    received_at: float | None = None
    history: deque = field(default_factory=lambda: deque(maxlen=240))


class PoseState:
    """Encoder samples are the only live source; missing samples never move the model."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.axes = [AxisFeedback(), AxisFeedback()]
        self.connected = False
        self.reference = list(HOME)
        self.direction = [1, -1]
        self.demo = False
        self.demo_started = 0.0

    def connect(self, connected):
        self.connected = bool(connected)
        self.demo = False
        self.axes = [AxisFeedback(), AxisFeedback()]

    def feed(self, row):
        axis = row.get('axis')
        if axis not in (0, 1):
            return False
        try:
            angle, speed, current = (float(row[k]) for k in ('angle_deg', 'speed_rpm', 'current_a'))
            stamp = float(row.get('received_at', self.clock()))
        except (KeyError, TypeError, ValueError):
            return False
        if not all(math.isfinite(v) for v in (angle, speed, current, stamp)):
            return False
        item = self.axes[axis]
        if item.received_at is not None and stamp <= item.received_at:
            return False
        item.angle, item.speed, item.current = angle % 360, speed, current
        item.enabled = bool(row.get('enabled', False))
        item.received_at = stamp
        item.history.append((stamp, item.angle))
        return True

    def status(self, axis):
        if self.demo:
            return 'demo'
        if not self.connected:
            return 'offline'
        stamp = self.axes[axis].received_at
        if stamp is None:
            return 'waiting'
        return 'live' if self.clock()-stamp <= 1.5 else 'stale'

    def offsets(self):
        return tuple(self.direction[i]*angle_delta(a.angle, self.reference[i])
                     if a.received_at is not None else 0.0 for i, a in enumerate(self.axes))

    def start_demo(self):
        if self.connected:
            return False
        self.axes = [AxisFeedback(), AxisFeedback()]
        self.demo, self.demo_started = True, self.clock()
        return True

    def step_demo(self):
        if not self.demo:
            return
        t = self.clock()-self.demo_started
        positions = (28*math.sin(t*.46), 22*math.sin(t*.61+.5))
        velocities = (28*.46*math.cos(t*.46)/6, 22*.61*math.cos(t*.61+.5)/6)
        for i in (0, 1):
            self.feed(dict(axis=i, angle_deg=self.reference[i]+positions[i]/self.direction[i],
                           speed_rpm=velocities[i]/self.direction[i],
                           current_a=.18+.07*abs(math.sin(t*.8+i)), enabled=False))


def rotate_z(p, degrees):
    a = math.radians(degrees)
    c, s = math.cos(a), math.sin(a)
    x, y, z = p
    return (c*x-s*y, s*x+c*y, z)


def transform(p, group, pan, tilt):
    if group == 'body':
        x, y, z = p
        z -= 235
        a = math.radians(tilt)
        p = (x, math.cos(a)*y-math.sin(a)*z,
             math.sin(a)*y+math.cos(a)*z+235)
    return rotate_z(p, pan) if group != 'base' else p


def box(x0, y0, z0, x1, y1, z1):
    return [
        [(x0,y0,z0),(x0,y1,z0),(x1,y1,z0),(x1,y0,z0)],
        [(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)],
        [(x0,y0,z0),(x1,y0,z0),(x1,y0,z1),(x0,y0,z1)],
        [(x1,y0,z0),(x1,y1,z0),(x1,y1,z1),(x1,y0,z1)],
        [(x1,y1,z0),(x0,y1,z0),(x0,y1,z1),(x1,y1,z1)],
        [(x0,y1,z0),(x0,y0,z0),(x0,y0,z1),(x0,y1,z1)],
    ]


def cylinder(center, radius, length, axis='z', segments=28):
    cx, cy, cz = center
    rings = []
    for sign in (-1, 1):
        ring = []
        for i in range(segments):
            a = 2*math.pi*i/segments
            u, v = radius*math.cos(a), radius*math.sin(a)
            ring.append((cx+sign*length/2, cy+u, cz+v) if axis == 'x' else
                        (cx+u, cy+sign*length/2, cz+v) if axis == 'y' else
                        (cx+u, cy+v, cz+sign*length/2))
        rings.append(ring)
    faces = [list(reversed(rings[0])), rings[1]]
    faces.extend([rings[0][i], rings[0][(i+1)%segments],
                   rings[1][(i+1)%segments], rings[1][i]] for i in range(segments))
    return faces


class GimbalCanvas(tk.Canvas):
    def __init__(self, parent, **kw):
        super().__init__(parent, bg=BG, highlightthickness=0, **kw)
        self.azimuth, self.elevation, self.zoom = 29.0, 19.0, 1.0
        self.pan = self.tilt = 0.0
        self.drag_origin = None
        self.mesh = []
        self._build_mesh()
        self.bind('<Configure>', lambda e:self.render())
        self.bind('<ButtonPress-1>', self._begin_drag)
        self.bind('<B1-Motion>', self._drag)
        self.bind('<ButtonRelease-1>', lambda e:setattr(self, 'drag_origin', None))
        self.bind('<MouseWheel>', self._wheel)

    def _begin_drag(self, e):
        self.drag_origin = (e.x, e.y, self.azimuth, self.elevation)

    def _drag(self, e):
        if self.drag_origin:
            x, y, az, el = self.drag_origin
            self.azimuth = az+(e.x-x)*.4
            self.elevation = max(-8, min(65, el+(e.y-y)*.25))
            self.render()

    def _wheel(self, e):
        self.zoom = max(.65, min(1.45, self.zoom*(1.08 if e.delta>0 else 1/1.08)))
        self.render()
        return 'break'

    def reset_camera(self):
        self.azimuth, self.elevation, self.zoom = 29, 19, 1
        self.render()

    def add(self, faces, color, group, edge='#65768a'):
        for face in faces:
            self.mesh.append((face, color, group, edge))

    def _build_mesh(self):
        # Four fixed feet, yaw motor, rotating L support, pitch axle and payload.
        self.add(box(-108,-25,0,108,25,9),'#566477','base')
        self.add(box(-25,-95,1,25,95,10),'#536274','base')
        self.add(cylinder((0,0,25),36,40),'#5b6c81','base')
        self.add(cylinder((0,0,47),41,10),'#8090a1','base')
        self.add(cylinder((0,0,51),42,3),'#319f9e','pan',PAN)
        self.add(box(-40,-38,56,42,38,100),'#69798c','pan')
        self.add(box(36,-28,65,126,28,96),'#718094','pan')
        self.add(box(96,-28,89,126,28,245),'#647488','pan')
        self.add(box(93,-33,235,130,33,250),'#8291a3','pan')
        self.add(cylinder((87,0,235),29,25,'x'),'#bac3ce','pan')
        self.add(cylinder((70,0,235),32,13,'x'),'#46566b','pan')
        self.add(cylinder((62,0,235),32.5,3,'x'),'#8475b6','body',TILT)
        self.add(box(-112,-65,174,59,70,333),'#576578','body')
        self.add(box(-115,-68,330,63,73,340),'#8793a2','body')
        self.add(box(-112,71,178,58,76,325),'#627185','body')
        self.add(box(-103,77,189,50,79,273),'#526176','body')
        self.add(box(-110,74,278,-56,84,302),'#333f51','body')
        self.add(box(-103,85,285,-65,86,294),'#b6c4d4','body')
        self.add(box(-101,86,287,-67,87,290),'#354d68','body')
        # Front optical aperture and mounting ring; attached to the tilting body.
        self.add(cylinder((-13,82,266),23,10,'y'),'#7e8fa3','body')
        self.add(cylinder((-13,89,266),17,4,'y'),'#172938','body','#8d9dae')
        self.add(cylinder((-13,92,266),10,2,'y'),'#216b83','body','#74c9df')
        self.add(cylinder((-15,93.5,269),4,1,'y'),'#69afcc','body','#69afcc')
        # Bolts, ventilation ribs, arm seams.
        for x in (-100,45):
            for z in (190,313):
                self.add(cylinder((x,80,z),2,1,'y',8),'#a1b0c1','body')
        for z in range(203,260,10):
            self.add(box(-99,80,z,-69,82,z+2),'#8390a0','body','#65768a')
        for z in (112,149,186,223):
            self.add(box(99,29,z,122,30,z+1),'#9ba7b5','pan')
        for x in (-25,25,105):
            for y in (-20,20):
                self.add(cylinder((x,y,101 if x<40 else 97),2,1,'z',8),'#b8c6d5','pan')

    def project(self, p):
        x,y,z = p
        az,el = math.radians(self.azimuth),math.radians(self.elevation)
        sx = math.cos(az)*x-math.sin(az)*y
        d = math.sin(az)*x+math.cos(az)*y
        sy = math.sin(el)*d-math.cos(el)*(z-177)
        depth = math.cos(el)*d+math.sin(el)*(z-177)
        perspective = 1100/(1100-depth)
        return (self.winfo_width()*.48+sx*self.scale*perspective,
                self.winfo_height()*.48+sy*self.scale*perspective, depth)

    @staticmethod
    def shade(color, points):
        u = [points[1][i]-points[0][i] for i in range(3)]
        v = [points[2][i]-points[0][i] for i in range(3)]
        n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        length = math.sqrt(sum(x*x for x in n)) or 1
        light = .60+.40*max(0, sum(a*b for a,b in zip(n,(-.3,.45,.84)))/length)
        return '#'+''.join(f'{min(255,int(int(color[i:i+2],16)*light)):02x}' for i in (1,3,5))

    def line3(self, points, color, width=1, dash=None, arrow=None):
        projected = [self.project(p) for p in points]
        coords = [v for p in projected for v in p[:2]]
        return self.create_line(*coords,fill=color,width=width,dash=dash,arrow=arrow)

    def render(self):
        w,h=self.winfo_width(),self.winfo_height()
        if w<20 or h<20:
            return
        self.delete('all')
        self.scale = min(w/540,h/410)*self.zoom
        # Subtle floor reference, fixed in world coordinates.
        for k in range(-240,241,40):
            self.line3([(k,-200,-5),(k,200,-5)],'#152338')
            self.line3([(-240,k,-5),(240,k,-5)],'#152338')
        for r,col in ((150,'#14283b'),(147,'#112335')):
            self.line3([(r*math.cos(i*math.pi/40),r*math.sin(i*math.pi/40),-3) for i in range(81)],col)
        # Mechanical travel references at each real joint.
        pan_arc=[(64*math.cos(math.radians(a)),64*math.sin(math.radians(a)),54) for a in range(-30,272,4)]
        self.line3(pan_arc,'#205b64',2)
        self.line3([transform(p,'pan',self.pan,0) for p in [(0,0,0),(0,0,122)]],PAN,1,dash=(3,5))
        faces=[]
        for vertices,color,group,edge in self.mesh:
            points=[transform(p,group,self.pan,self.tilt) for p in vertices]
            projected=[self.project(p) for p in points]
            faces.append((sum(p[2] for p in projected)/len(projected),projected,self.shade(color,points),edge))
        for _,points,color,edge in sorted(faces,key=lambda f:f[0]):
            self.create_polygon(*[v for p in points for v in p[:2]],fill=color,outline=edge,width=.65)
        # Joint rings and optical direction show the two different rotations.
        pitch_ring=[(132,39*math.cos(i*math.pi/32),235+39*math.sin(i*math.pi/32)) for i in range(65)]
        self.line3([transform(p,'pan',self.pan,0) for p in pitch_ring],TILT,1.5)
        self.line3([transform(p,'body',self.pan,self.tilt) for p in [(-13,100,266),(-13,185,266)]], '#42869d',1,dash=(4,5),arrow='last')
        self._callout(transform((0,0,50),'pan',self.pan,0),PAN,'01  PAN / 水平转轴',(-135,35))
        self._callout(transform((130,0,235),'pan',self.pan,0),TILT,'02  TILT / 俯仰转轴',(85,-30))
        self.create_text(24,24,text='KINEMATIC VIEW',fill=MUTED,anchor='nw',font=('Consolas',10))
        self.create_text(24,45,text='双轴云台 · 机械结构示意',fill=TEXT,anchor='nw',font=('Microsoft YaHei UI',13,'bold'))
        self.create_text(24,h-25,text='拖动旋转视角   /   滚轮缩放',fill=MUTED,anchor='sw',font=('Microsoft YaHei UI',9))
        # World orientation triad.
        origin=(w-67,h-50)
        for dx,dy,label,color in ((27,10,'X','#c78a89'),(-21,14,'Y',PAN),(0,-28,'Z','#8d9ec2')):
            self.create_line(*origin,origin[0]+dx,origin[1]+dy,fill=color,arrow='last',width=1.5)
            self.create_text(origin[0]+dx*1.4,origin[1]+dy*1.4,text=label,fill=color,font=('Consolas',9))

    def _callout(self, point, color, label, offset):
        x,y,_=self.project(point)
        tx=max(100,min(self.winfo_width()-105,x+offset[0]))
        ty=max(90,min(self.winfo_height()-72,y+offset[1]))
        self.create_oval(x-4,y-4,x+4,y+4,outline=color,width=2)
        self.create_line(x,y,tx,ty,fill=color,dash=(2,4))
        self.create_rectangle(tx-84,ty-14,tx+84,ty+14,fill=PANEL,outline=color)
        self.create_text(tx,ty,text=label,fill=color,font=('Microsoft YaHei UI',9,'bold'))


class GimbalDashboard(ttk.Frame):
    def __init__(self,parent,config_path):
        super().__init__(parent,padding=14)
        self.state=PoseState()
        self.config_path=Path(config_path)
        self.load_config()
        self._last_angles=None
        self._last_chart=0
        self.columnconfigure(0,weight=1)
        self.columnconfigure(1,weight=0,minsize=238)
        self.rowconfigure(1,weight=1)
        heading=ttk.Frame(self)
        heading.grid(row=0,column=0,columnspan=2,sticky='ew',pady=(0,12))
        title=ttk.Frame(heading)
        title.pack(side='left')
        ttk.Label(title,text='双轴实际姿态',font=('Microsoft YaHei UI',15,'bold')).pack(anchor='w')
        ttk.Label(title,text='机械结构  /  双轴编码器  /  运动趋势',foreground=MUTED).pack(anchor='w',pady=(2,0))
        self.badge=tk.Label(heading,text='●  未连接 · 等待实际反馈',bg='#1d2a3b',fg=MUTED,
                            padx=10,pady=6,font=('Microsoft YaHei UI',9))
        self.badge.pack(side='right',padx=(12,0))
        viewport=tk.Frame(self,bg=BG,highlightbackground=LINE,highlightthickness=1)
        viewport.grid(row=1,column=0,sticky='nsew',padx=(0,14))
        self.canvas=DetailedGimbalCanvas(viewport,fallback_factory=GimbalCanvas,width=470,height=470)
        self.canvas.pack(fill='both',expand=True)
        toolbar=tk.Frame(viewport,bg=BG)
        toolbar.pack(side='bottom',fill='x',padx=10,pady=(0,10),before=self.canvas)
        self.demo_button=ttk.Button(toolbar,text='离线动作演示',command=self.toggle_demo)
        self.demo_button.pack(side='left')
        ttk.Button(toolbar,text='复位视角',command=self.canvas.reset_camera).pack(side='left',padx=6)
        ttk.Button(toolbar,text='模型标定',command=self.calibrate).pack(side='left')
        tk.Label(toolbar,text='拖动旋转 / 滚轮缩放',bg=BG,fg=MUTED,font=('Microsoft YaHei UI',8)).pack(side='right',padx=4)
        self.offset_text=tk.StringVar(value='PAN  —     TILT  —')
        # Numerical offsets are shown beside the axis status; keep the toolbar compact.
        side=ttk.Frame(self)
        side.grid(row=1,column=1,sticky='nsew')
        self.cards=[]
        for axis,(name,desc,color) in enumerate((('01 / PAN','下轴 · 水平旋转',PAN),('02 / TILT','上轴 · 俯仰旋转',TILT))):
            card=tk.Frame(side,bg=PANEL,highlightbackground=LINE,highlightthickness=1)
            card.pack(fill='x',pady=(0,12))
            head=tk.Frame(card,bg=PANEL);head.pack(fill='x',padx=12,pady=(9,0))
            tk.Label(head,text=name,bg=PANEL,fg=color,font=('Consolas',10,'bold')).pack(side='left')
            tk.Label(head,text=desc.split(' · ')[0],bg=PANEL,fg=MUTED,font=('Microsoft YaHei UI',9)).pack(side='right')
            angle=tk.Label(card,text='— °',bg=PANEL,fg=TEXT,font=('Consolas',24,'bold'))
            angle.pack(anchor='w',padx=12,pady=(3,0))
            row=tk.Frame(card,bg=PANEL)
            row.pack(fill='x',padx=12,pady=(0,4))
            speed=tk.Label(row,text='— rpm',bg=PANEL,fg=TEXT,font=('Consolas',11))
            speed.pack(side='left')
            current=tk.Label(row,text='— A',bg=PANEL,fg=MUTED,font=('Consolas',11))
            current.pack(side='right')
            meter=tk.Canvas(card,height=16,bg=PANEL,highlightthickness=0)
            meter.pack(fill='x',padx=12)
            status=tk.Label(card,text='等待编码器反馈',bg=PANEL,fg=MUTED,font=('Microsoft YaHei UI',9))
            status.pack(anchor='w',padx=12,pady=(0,7))
            self.cards.append((angle,speed,current,meter,status))
        info=tk.Frame(side,bg=PANEL,highlightbackground=LINE,highlightthickness=1)
        info.pack(fill='both',expand=True)
        self.freshness=tk.Label(info,text='尚无设备数据',bg=PANEL,fg=MUTED,font=('Microsoft YaHei UI',9),justify='left',anchor='nw')
        self.freshness.pack(anchor='nw',padx=12,pady=8)
        lower=tk.Frame(self,bg=PANEL,highlightbackground=LINE,highlightthickness=1)
        lower.grid(row=2,column=0,columnspan=2,sticky='ew',pady=(12,0))
        chart_head=tk.Frame(lower,bg=PANEL)
        chart_head.pack(fill='x',padx=16,pady=(9,0))
        tk.Label(chart_head,text='双轴姿态趋势',bg=PANEL,fg=TEXT,font=('Microsoft YaHei UI',10,'bold')).pack(side='left')
        tk.Label(chart_head,text='● PAN   ● TILT    /    最近 20 s',bg=PANEL,fg=MUTED,font=('Consolas',10)).pack(side='right')
        self.chart=tk.Canvas(lower,bg=PANEL,height=86,highlightthickness=0)
        self.chart.pack(fill='x',padx=10,pady=(0,7))
        self.chart.bind('<Configure>',lambda e:self.draw_chart())
        self.refresh()

    def load_config(self):
        try:
            data=json.loads(self.config_path.read_text(encoding='utf-8'))
            refs=data['reference_deg'];directions=data['direction']
            if len(refs)!=2 or len(directions)!=2 or not all(isinstance(v,(int,float)) and math.isfinite(v) and 0<=v<360 for v in refs) or any(v not in (-1,1) for v in directions):
                return
            self.state.reference=list(refs);self.state.direction=list(directions)
        except (OSError,ValueError,KeyError,TypeError):
            pass

    def set_connection(self,image):
        self.state.connect(image is not None)
        self._last_angles=None
        self.demo_button.configure(state='disabled' if image else 'normal',text='离线动作演示')
        self.refresh()
        self.draw_chart()

    def feed(self,row):
        if self.state.connected and not self.state.demo:
            return self.state.feed(row)
        return False

    def toggle_demo(self):
        if self.state.connected:
            return
        if self.state.demo:
            self.state.connect(False)
            self.demo_button.configure(text='离线动作演示')
        else:
            self.state.start_demo()
            self.demo_button.configure(text='停止离线演示')
        self._last_angles=None
        self.draw_chart()

    def refresh(self):
        now=self.state.clock()
        self.state.step_demo()
        offsets=self.state.offsets()
        if offsets!=self._last_angles:
            self.canvas.pan,self.canvas.tilt=offsets
            self.canvas.render()
            self._last_angles=offsets
        statuses=[self.state.status(i) for i in (0,1)]
        if self.state.demo:
            text,color,bg='●  离线演示 · 非设备反馈',AMBER,'#352b1b'
        elif statuses==['live','live']:
            text,color,bg='●  实际编码器反馈',PAN,'#133035'
        elif self.state.connected:
            text,color,bg='●  反馈等待 / 超时，保持最后姿态',AMBER,'#352b1b'
        else:
            text,color,bg='●  未连接 · 等待实际反馈',MUTED,'#1d2a3b'
        self.badge.configure(text=text,fg=color,bg=bg)
        self.offset_text.set(f'PAN {offsets[0]:+6.1f}°   TILT {offsets[1]:+6.1f}°' if any(a.received_at is not None for a in self.state.axes) else 'PAN  —     TILT  —')
        ages=[]
        for i,(angle,speed,current,meter,status) in enumerate(self.cards):
            a=self.state.axes[i];mode=statuses[i];color=(PAN,TILT)[i]
            has=a.received_at is not None
            angle.configure(text=f'{a.angle:06.2f}°' if has else '— °')
            speed.configure(text=f'{a.speed:+.2f} rpm' if has else '— rpm')
            current.configure(text=f'{a.current:.3f} A' if has else '— A')
            label=('演示数据 · 不发送控制指令' if mode=='demo' else
                   ('已使能' if a.enabled else '未使能')+' · 实际反馈' if mode=='live' else
                   '反馈超时 · 保持最后姿态' if mode=='stale' else
                   '已连接 · 等待该轴反馈' if mode=='waiting' else '离线 · 参考零位示意')
            status.configure(text=label,fg=AMBER if mode in ('demo','stale') else color if mode=='live' else MUTED)
            meter.delete('all');mw=max(200,meter.winfo_width())
            meter.create_line(0,8,mw,8,fill=LINE,width=3)
            meter.create_line(mw/2,2,mw/2,14,fill=MUTED)
            px=mw/2+max(-1,min(1,offsets[i]/(60 if i==0 else 120)))*(mw/2-4)
            if has:
                meter.create_line(mw/2,8,px,8,fill=color,width=3)
                meter.create_oval(px-3,5,px+3,11,fill=color,outline='')
            ages.append(f'{("PAN","TILT")[i]} '+(f'{max(0,now-a.received_at):.1f}s' if has else '等待'))
        self.freshness.configure(text='离线演示 · 非设备反馈' if self.state.demo else '  /  '.join(ages))
        if now-self._last_chart>.2:
            self.draw_chart();self._last_chart=now

    def draw_chart(self):
        c=self.chart;c.delete('all')
        w,h=c.winfo_width(),c.winfo_height()
        if w<80:return
        now=self.state.clock();left,right,top,bottom=52,w-16,12,h-22
        for value in (-60,0,60,120):
            y=bottom-(value+60)/180*(bottom-top)
            c.create_line(left,y,right,y,fill=LINE,dash=(2,5))
            c.create_text(left-8,y,text=f'{value}°',fill=MUTED,anchor='e',font=('Consolas',8))
        for seconds in (20,15,10,5,0):
            x=right-seconds/20*(right-left)
            c.create_text(x,h-9,text=f'-{seconds}s' if seconds else '现在',fill=MUTED,font=('Microsoft YaHei UI',8))
        plotted=False
        for i,a in enumerate(self.state.axes):
            segments=[];points=[];previous=None;previous_value=None
            for stamp,angle in a.history:
                if not now-20<=stamp<=now:continue
                value=self.state.direction[i]*angle_delta(angle,self.state.reference[i])
                if previous is not None and (stamp-previous>1.5 or abs(value-previous_value)>180):
                    if len(points)>=4:segments.append(points)
                    points=[]
                previous=stamp;previous_value=value
                x=right-(now-stamp)/20*(right-left)
                y=bottom-(max(-60,min(120,value))+60)/180*(bottom-top)
                points.extend((x,y))
            if len(points)>=4:segments.append(points)
            for points in segments:
                c.create_line(*points,fill=(PAN,TILT)[i],width=2)
                plotted=True
        if not plotted:
            c.create_text((left+right)/2,(top+bottom)/2-8,text='等待双轴实际反馈，或开启离线动作演示',fill=MUTED,font=('Microsoft YaHei UI',10))

    def calibrate(self):
        dialog=tk.Toplevel(self)
        dialog.title('模型姿态标定');dialog.resizable(False,False);dialog.configure(bg=BG)
        dialog.transient(self.winfo_toplevel())
        body=ttk.Frame(dialog,padding=20);body.pack(fill='both',expand=True)
        ttk.Label(body,text='编码器零位与显示方向',font=('Microsoft YaHei UI',13,'bold')).grid(row=0,column=0,columnspan=3,sticky='w',pady=(0,12))
        refs=[tk.DoubleVar(value=v) for v in self.state.reference]
        dirs=[tk.StringVar(value='正向' if v==1 else '反向') for v in self.state.direction]
        for i,name in enumerate(('下轴 PAN','上轴 TILT')):
            ttk.Label(body,text=name).grid(row=i+1,column=0,padx=(0,14),pady=8)
            ttk.Spinbox(body,from_=0,to=359.999,increment=.1,textvariable=refs[i],width=12).grid(row=i+1,column=1,padx=6)
            ttk.Combobox(body,textvariable=dirs[i],values=('正向','反向'),state='readonly',width=8).grid(row=i+1,column=2,padx=6)
        notice=tk.StringVar(value='只调整模型映射，不改变电机零位或控制参数。')
        ttk.Label(body,textvariable=notice,wraplength=390).grid(row=3,column=0,columnspan=3,sticky='w',pady=12)
        def save():
            try:
                values=[v.get() for v in refs]
                if any(not math.isfinite(v) or not 0<=v<360 for v in values):raise ValueError('零位角度需在 0～360° 之间')
                signs=[1 if v.get()=='正向' else -1 for v in dirs]
                self.config_path.write_text(json.dumps(dict(reference_deg=values,direction=signs),indent=2)+'\n',encoding='utf-8')
                self.state.reference=values;self.state.direction=signs;self._last_angles=None
                dialog.destroy()
            except (ValueError,tk.TclError,OSError) as exc:
                notice.set(str(exc))
        ttk.Button(body,text='保存显示标定',command=save).grid(row=4,column=1,columnspan=2,sticky='e')


def apply_dark_theme(root):
    style=ttk.Style(root);style.theme_use('clam')
    root.configure(bg=BG)
    root.option_add('*TCombobox*Listbox.background',PANEL)
    root.option_add('*TCombobox*Listbox.foreground',TEXT)
    style.configure('.',background=BG,foreground=TEXT,font=('Microsoft YaHei UI',9),bordercolor=LINE,
                    lightcolor=LINE,darkcolor=LINE,troughcolor=BG,selectbackground='#244965',selectforeground=TEXT)
    style.configure('TFrame',background=BG)
    style.configure('TLabel',background=BG,foreground=TEXT)
    style.configure('TLabelframe',background=BG,bordercolor=LINE)
    style.configure('TLabelframe.Label',background=BG,foreground='#b8cbe0',font=('Microsoft YaHei UI',9,'bold'))
    style.configure('TButton',padding=(10,6),background='#192a40',foreground=TEXT,bordercolor='#33465d',focuscolor=PAN)
    style.map('TButton',background=[('active','#284663'),('pressed','#1e5360')],foreground=[('disabled','#52657c')])
    for name in ('TEntry','TSpinbox','TCombobox'):
        style.configure(name,fieldbackground=PANEL,background='#1a2b40',foreground=TEXT,arrowcolor=MUTED,
                        bordercolor=LINE,insertcolor=TEXT,padding=4)
        style.map(name,fieldbackground=[('readonly',PANEL)],foreground=[('readonly',TEXT),('disabled',MUTED)])
    style.configure('TNotebook',background=BG,borderwidth=0)
    style.configure('TNotebook.Tab',padding=(17,9),background='#111d2e',foreground=MUTED)
    style.map('TNotebook.Tab',background=[('selected','#20334b')],foreground=[('selected',PAN)])
    for name in ('TCheckbutton','TRadiobutton'):
        style.configure(name,background=BG,foreground=TEXT,indicatorbackground='#15263c',indicatorforeground=PAN)
        style.map(name,background=[('active','#142238')])
    style.configure('Vertical.TScrollbar',background='#24374f',arrowcolor=MUTED,troughcolor=BG,borderwidth=0)
    return style
