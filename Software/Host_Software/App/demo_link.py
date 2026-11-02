"""One UART owner for both demo images; never uploads or flashes a device."""
from pathlib import Path
import json
import math
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'App'))
from vision import Link, fit, calibration_centre

HOME = (21285*360/65536, 180.0)
LIMITS = ((-60, 60), (-30, 120))


def ramp(current, target, dt, accel=5.0, decel=8.0):
    if current*target < 0:
        target = 0
    rate = decel if abs(target) < abs(current) else accel
    change = rate*min(max(dt, 0), .15)
    return current+max(-change, min(change, target-current))


def limited(axis, angle, requested):
    offset = (angle-HOME[axis]+180) % 360-180
    low, high = LIMITS[axis]
    if not low-.5 <= offset <= high+.5:
        raise ValueError('当前位置超出工作范围，请重新回位')
    distance = max(0, (high-offset if requested > 0 else offset-low)-2)
    # 250 ms transport reserve, 8 rpm/s braking, two-degree margin.
    cap = math.sqrt(4+8*distance/3)-2
    return math.copysign(min(abs(requested), cap), requested)


class DemoLink(Link):
    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=.25, write_timeout=.25)
        try:
            self.ser.reset_input_buffer()
            self.identify()
        except Exception:
            self.ser.close()
            raise

    def identify(self):
        self.image = self.command(0x90)
        if self.image not in (0xD1, 0xD2, 0xE2):
            raise ValueError('当前不是集成版 D1/D2 或 E2 实验版，请先下载 demo_system/bits 中的镜像')
        self.version = self.command(0x26)
        self.shape_version = self.command(0xA4) if self.image in (0xD2,0xE2) else 0
        self.has_shapes = self.shape_version == 0xE7
        self.shape_active = bool(self.command(0xA5)&2) if self.has_shapes else False
        self.has_video_stats = self.image == 0xD1 and self.command(0x58) == 0xB1
        options_version = self.command(0x74) if self.image == 0xD1 else 0
        self.has_video_options = options_version in (0xC1, 0xC2)
        self.has_video_quad = options_version == 0xC2
        self.has_video_motion = self.image == 0xD1 and self.command(0x7C) == 0xD8
        return self.image

    def video_motion(self):
        self.need(0xD1)
        if not getattr(self, 'has_video_motion', False):
            return None
        raw=bytes(self.command(0x7A,i) for i in range(4))
        flags,count,cells=struct.unpack('<BBH',raw)
        return dict(valid=bool(flags&1),warming=bool(flags&2),paused=bool(flags&4),
                    scene_change=bool(flags&8),count=count,changed_cells=cells,
                    threshold=self.command(0x78),min_tiles=self.command(0x79))

    def set_video_motion(self, threshold, min_tiles):
        self.need(0xD1)
        if not getattr(self, 'has_video_motion', False):
            raise ValueError('请下载支持帧差运动检测的 D1/A8 位流')
        if not (isinstance(threshold,int) and isinstance(min_tiles,int) and
                1<=threshold<=255 and 1<=min_tiles<=32):
            raise ValueError('帧差阈值范围1～255，最小区域块数1～32')
        if self.command(0x76,threshold)!=threshold or self.command(0x77,min_tiles)!=min_tiles:
            raise IOError('运动参数写入失败')
        self.command(0x7B)
        actual=self.video_motion()
        if actual['threshold']!=threshold or actual['min_tiles']!=min_tiles:
            raise IOError('运动参数读回不一致')
        return actual

    def video_options(self):
        self.need(0xD1)
        if not getattr(self, 'has_video_options', False):
            return None
        raw=[]
        for i in range(10):
            if i % 4 == 0:
                self.command(0x88, 0x5A)
            raw.append(self.command(0x73, i))
        flags, scopes, x0, y0, x1, y1=struct.unpack('<BB4H', bytes(raw))
        return dict(flags=flags, scopes=scopes, rect=(x0,y0,x1,y1),
                    quad_supported=getattr(self, 'has_video_quad', False),
                    union=bool(self.command(0x67)), locked=bool(self.command(0x6B)),
                    mode=(self.command(0x00)>>4)&7, effect=self.command(0x66))

    def set_video_options(self, flags, scopes, rect, union, locked):
        self.need(0xD1)
        if not getattr(self, 'has_video_options', False):
            raise ValueError('请先下载支持算法开关和选区的新版视频位流')
        x0,y0,x1,y1=rect
        if not (0 <= flags < 64 and 0 <= scopes < 16 and
                all(isinstance(v,int) for v in rect) and 0 <= x0 <= x1 < 1280 and 0 <= y0 <= y1 < 720):
            raise ValueError('矩形需满足 0≤X0≤X1≤1279、0≤Y0≤Y1≤719')
        raw=struct.pack('<BB4H',flags,scopes,*rect)
        for i,v in enumerate(raw):
            if i % 3 == 0:
                self.command(0x88,0x5A)
            if self.command(0x70,i)!=i or self.command(0x71,v)!=v:
                raise IOError('设置暂存失败，未提交新参数')
        # These two controls describe the existing detector's region source.
        self.command(0x65,int(union));self.command(0x6A,int(locked))
        result=self.command(0x72,0xA5)
        if result:
            raise IOError(f'参数提交失败，返回 {result}；1=跨时钟传递忙，2=参数越界')
        actual=self.video_options()
        if (actual['flags'],actual['scopes'],actual['rect'])!=(flags,scopes,tuple(rect)):
            raise IOError('参数读回不一致')
        return actual

    def video_stats(self):
        self.need(0xD1)
        if not getattr(self, 'has_video_stats', False):
            return None
        raw = []
        for i in range(17):
            if i % 6 == 0:
                self.command(0x88, 0x5A)
            raw.append(self.command(0x59, i))
        flags, x, y, w, h, cam, out, cf, df, binary, edge = struct.unpack('<B4H2B2H2B', bytes(raw))
        return dict(valid=bool(flags & 1), x=x, y=y, width=w, height=h,
                    camera_fps=cam, display_fps=out, camera_frames=cf,
                    display_frames=df, binary_threshold=binary, edge_floor=edge)

    def need(self, image):
        if self.image != image and not (image==0xD2 and self.image==0xE2):
            raise ValueError('当前工程不支持该操作，请先切换镜像')

    def motor(self, axis, command=0, value=0):
        self.need(0xD1)
        if axis not in (0, 1) or command not in (0, 2, 4, 5):
            raise ValueError('Unsupported manual command')
        self.command(0x88, 0x5A)
        for cmd, data in ((0x80, axis), (0x81, command), (0x82, value & 255), (0x83, (value >> 8) & 255)):
            if self.command(cmd, data) != data:
                raise IOError('电机指令提交忙，请停止后重试')
        if self.command(0x84, 0xA5):
            raise IOError('电机指令被拒绝')
        deadline = time.monotonic()+.6
        while self.command(0x85) & 1:
            if time.monotonic() >= deadline:
                raise TimeoutError('电机通道超时')
            time.sleep(.002)
        self.command(0x88, 0x5A)
        raw = bytes(self.command(0x86, i) for i in range(8))
        result, flags, angle, speed, current = struct.unpack('<BBHhh', raw)
        if result:
            raise IOError(f'电机响应错误 {result}（4 表示未回位、忙或超限）')
        if (flags >> 1) & 1 != axis:
            raise IOError('电机响应轴不匹配')
        return dict(axis=axis, enabled=bool(flags & 1), angle_deg=angle*360/65536,
                    speed_rpm=speed*1000/32767, current_a=current*10/32767)

    def speed(self, axis, rpm):
        return self.motor(axis, 4, round(max(-10, min(10, rpm))*32767/1000))

    def hold(self, axis, angle):
        return self.motor(axis, 5, round(angle*65536/360) & 65535)

    def stop_tracking(self):
        super().stop_tracking()
        if getattr(self, 'has_shapes', False):
            self.command(0xA0, 0)
            self.shape_active = False

    def start_tracking(self, path, brightness, chroma, radius, laser):
        self.need(0xD2)
        if not 0 <= brightness <= 255 or not 1 <= chroma <= 255 or not 4 <= radius <= 64:
            raise ValueError('亮度 0～255，色差 1～255，框半径 4～64')
        data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
        coeff, error = fit(data['samples'])
        aim = calibration_centre(coeff)
        self.require_homed()
        self.stop_tracking()
        self.command(0x21, 1)
        self.command(0x52, chroma)
        self.command(0x53, brightness)
        self.command(0x74, 1)
        try:
            self.configure(coeff=coeff, radius=radius, mode=1 if laser else 3, aim=aim)
        except Exception:
            self.stop_tracking()
            raise
        return aim, error

    def switch(self, destination, report=None):
        report = report or (lambda message: None)
        if destination not in (0, 1):
            raise ValueError('Invalid image')
        if self.image == 0xD1+destination:
            return
        if self.image == 0xE2:
            raise ValueError('E2 实验版不支持 Flash 跳转，请下载正式 D2 工程二')
        source = self.image
        initial = self.command(0x91)
        report(f'切换诊断：{source:02X} → {0xD1+destination:02X}，提交前状态=0x{initial:02X}')
        if not initial & 8:
            raise ValueError('FPGA 尚未确认双轴停稳保持；请先回位或停止，再重试')
        if self.command(0x92, destination) or self.command(0x93, 0xA5):
            self.command(0x94)
            raise ValueError('镜像切换联锁未通过')
        report('切换诊断：准备、提交均已收到 FPGA 确认，等待重新配置')
        time.sleep(.3)
        deadline = time.monotonic()+12
        last_observation = None
        last_image = last_status = None
        failures = 0
        while time.monotonic() < deadline:
            try:
                self.ser.reset_input_buffer()
                observed = self.command(0x90)
                last_image = observed
                if observed == 0xD1+destination:
                    if self.identify() == observed:
                        report(f'切换诊断：目标镜像 {observed:02X} 已确认')
                        return
                elif observed == source:
                    last_status = self.command(0x91)
                    observation = (observed, last_status)
                    if observation != last_observation:
                        stage = ('空闲/跳转已取消', '已准备', '等待触发', '已置位启动信号')[last_status & 3]
                        report(f'切换诊断：仍收到 {observed:02X}，状态=0x{last_status:02X}（{stage}，'
                               f'联锁={bool(last_status & 8)}，忙={bool(last_status & 4)}）')
                        last_observation = observation
                else:
                    if (observed, None) != last_observation:
                        report(f'切换诊断：收到其它身份 0x{observed:02X}')
                        last_observation = (observed, None)
            except (OSError, ValueError):
                failures += 1
            time.sleep(.2)
        detail = '期间没有有效身份回复'
        if last_image == source and last_status is not None:
            if last_status & 3 == 0:
                detail = '旧镜像仍回复、状态为空闲：跳转可能被联锁取消，或重新启动后又回到旧镜像'
            elif last_status & 3 == 3:
                detail = '旧镜像仍回复且启动信号已置位：需检查 Multi Boot 原语及配置路径'
            else:
                detail = f'旧镜像仍回复，切换状态=0x{last_status:02X}'
        elif last_image is not None:
            detail = f'最后收到身份 0x{last_image:02X}，目标身份或后续版本确认未完成'
        report(f'切换诊断结束：{detail}；通信/识别失败 {failures} 次')
        raise TimeoutError(f'未收到目标镜像完整确认；{detail}。请保留诊断日志，暂勿重复擦写 Flash')


class ManualMotion:
    """Host velocity ramp; FPGA heartbeat fallback stops on host loss."""
    def __init__(self, link, emit):
        self.link, self.emit = link, emit
        self.moving = {}

    def tick(self, requests):
        for axis in (0, 1):
            requested = requests[axis]
            if requested and axis not in self.moving:
                s = self.link.command(0x89)
                if s & 0xE0 or s & 3 != 3:
                    raise ValueError('请先完成双轴回位')
                row = self.link.motor(axis)
                if not row['enabled']:
                    raise ValueError('电机未使能，请回位')
                self.moving[axis] = dict(velocity=row['speed_rpm'], last=time.monotonic(), samples=[], stopping=None, row=row)
            if axis not in self.moving:
                continue
            m = self.moving[axis]
            target = limited(axis, m['row']['angle_deg'], requested)
            now = time.monotonic()
            velocity = ramp(m['velocity'], target, now-m['last'])
            command_at = now
            row = self.link.speed(axis, velocity)
            now = time.monotonic()
            m.update(velocity=velocity, last=command_at, row=row)
            self.emit('pose', row)
            if abs(velocity) > .001 or requested:
                m['samples'].clear()
                m['stopping'] = None
                continue
            if m['stopping'] is None:
                m['stopping'] = now
            if abs(row['speed_rpm']) > .3:
                m['samples'].clear()
            else:
                samples = m['samples']
                if samples and abs((row['angle_deg']-samples[0][1]+180) % 360-180) > .1:
                    samples.clear()
                samples.append((now, row['angle_deg']))
                if len(samples) >= 4 and now-samples[0][0] >= .4:
                    self.emit('pose', self.link.hold(axis, row['angle_deg']))
                    self.emit('log', f'{("下轴", "上轴")[axis]} 已停稳，保持 {row["angle_deg"]:.2f}°')
                    del self.moving[axis]
            if axis in self.moving and now-m['stopping'] > 4:
                raise TimeoutError('停车超时，已请求失能；请检查电机参数和负载')

    def stop(self):
        deadline = time.monotonic()+6
        while self.moving:
            self.tick((0, 0))
            if time.monotonic() >= deadline:
                raise TimeoutError('停车超时')
            time.sleep(.02)

    def disable(self):
        self.moving.clear()
        errors = []
        for axis in (0, 1):
            try:
                self.link.motor(axis, 2)
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise IOError('; '.join(errors))
