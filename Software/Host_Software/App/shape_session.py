"""Nonblocking E7 shape workflow; uses the GUI worker's single UART owner."""
import json
import time
from pathlib import Path

COLORS = ('dark', 'red', 'green', 'blue')
COLOR_LABELS = ('深色', '红色', '绿色', '蓝色')
SHAPE_LABELS = ('未知', '三角形', '矩形', '圆形')


def check_mapping(coeff, target):
    x0, x1 = target['x']-target['width']//2, target['x']+(target['width']+1)//2
    y0, y1 = target['y']-target['height']//2, target['y']+(target['height']+1)//2
    a, b, c, d, e, f = coeff
    points = [((a*x+b*y+c)>>8, (d*x+e*y+f)>>8) for x in (x0,x1) for y in (y0,y1)]
    if any(v < 0 or v > 65535 for point in points for v in point):
        raise ValueError('轮廓超出 DAC 可表示的 0～65535 数值范围；未开光')
    return any(v < 28768 or v > 36768 for point in points for v in point)


class ShapeSession:
    def __init__(self, link, emit, clock=time.monotonic):
        self.link, self.emit, self.clock = link, emit, clock
        self.phase = 'existing' if getattr(link, 'shape_active', False) else 'idle'
        self.next_poll = 0
        self.deadline = self.draw_until = None
        self.previous = None
        self.confirmations = 0

    @property
    def active(self):
        return self.phase != 'idle'

    def cancel(self):
        """Cancel scheduling; caller sends stop through the shared link."""
        self.phase = 'idle'
        self.deadline = self.draw_until = None
        self.emit('shape_phase', '已停止')

    def begin(self, drawing, path, color, seconds):
        if not getattr(self.link, 'has_shapes', False):
            raise ValueError('请下载支持 E7 形状功能的新版工程二')
        if color not in ('auto', 'red', 'green', 'blue') or not 1 <= seconds <= 3600:
            raise ValueError('请选择自动/红/绿/蓝，描边时间须为 1～3600 秒')
        self.link.stop_tracking()
        self.cancel()
        self.seconds = seconds
        if drawing:
            data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
            self.coeff = data['coeff_q8']
            if len(self.coeff) != 6 or any(type(v) is not int for v in self.coeff):
                raise ValueError('标定文件需要六个整数 coeff_q8 系数')
            for i,v in enumerate(self.coeff):
                bits = 32 if i in (2,5) else 24
                if not -(1<<(bits-1)) <= v < (1<<(bits-1)):
                    raise ValueError('标定系数超出 FPGA 数值范围')
        control = 9 if color == 'auto' else 1 | (COLORS.index(color)<<1)
        try:
            self.link.command(0xA1, 120)
            self.link.command(0xA0, control | (16 if drawing else 0))
            self.link.shape_active = True
            if drawing:
                self.link.configure(coeff=self.coeff, mode=0)
            self.phase = 'acquiring' if drawing else 'detecting'
            self.previous = None
            self.confirmations = 0
            self.next_poll = 0
            self.deadline = self.clock()+5 if drawing else None
            self.emit('shape_phase', '关光采集，等待稳定目标…' if drawing else '实时检测中，激光关闭')
            self.emit('log', '形状模式已启动；不自动回位或移动云台')
        except Exception:
            self.cancel()
            self.link.stop_tracking()
            raise

    def snapshot(self):
        status = self.link.command(0xA2)
        target = self.link.target()
        raw = sum(self.link.command(0xA6+i) << (8*i) for i in range(4))
        flags = self.link.command(0xA5)
        return dict(valid=bool(status&1), color=COLOR_LABELS[(status>>3)&3],
                    shape=SHAPE_LABELS[(status>>1)&3], identity=status&31, target=target,
                    raw_valid=bool(raw&(1<<24)), raw_shape=SHAPE_LABELS[(raw>>22)&3],
                    raw_color=COLOR_LABELS[(raw>>20)&3], raw_area=raw&65535,
                    vertices=(raw>>16)&15, locked=bool(raw&(1<<27)),
                    overflow=bool(raw&(1<<25)), late=bool(raw&(1<<26)),
                    range_fault=bool(flags&1), full_range=bool(flags&32),
                    requested=bool(flags&64), laser_now=bool(flags&128))

    def tick(self):
        if not self.active:
            return
        now = self.clock()
        if self.draw_until is not None and now >= self.draw_until:
            self.cancel()
            self.link.stop_tracking()
            self.emit('log', '静态描边时间到，已关光并退出形状模式')
            return
        if self.deadline is not None and now >= self.deadline:
            self.cancel()
            self.link.stop_tracking()
            raise ValueError('5 秒内未获得稳定形状；保持关光，请查看实时诊断')
        if now < self.next_poll:
            return
        self.next_poll = now+.2
        row = self.snapshot()
        self.emit('shape', row)
        if self.phase == 'acquiring':
            t = row['target']
            current = (row['identity'], t['x'], t['y'], t['width'], t['height'])
            if row['valid'] and t['valid']:
                same = self.previous is not None and current[0] == self.previous[0] and all(
                    abs(a-b)<=8 for a,b in zip(current[1:],self.previous[1:]))
                self.confirmations = self.confirmations+1 if same else 1
                self.previous = current
            else:
                self.previous = None
                self.confirmations = 0
            if self.confirmations >= 3:
                outside = check_mapping(self.coeff, t)
                self.link.command(0x72, 1)
                self.phase = 'drawing'
                self.deadline = None
                self.draw_until = self.clock()+self.seconds
                self.emit('log', '已请求锁定轮廓描边；'+('超出旧窗口仍保留真实坐标。' if outside else '')+
                          '实时识别丢失不取消描边；换图案前先停止。')
        if self.phase == 'drawing':
            self.emit('shape_phase', f'静态描边 · 剩余 {max(0,self.draw_until-self.clock()):.1f} 秒')
