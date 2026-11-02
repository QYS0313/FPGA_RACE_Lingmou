"""Green-spot tracking, B20 faster pursuit with B19 bounded laser chase."""
import argparse
from collections import deque
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'Vendor'))
import serial

DEFAULT_CALIBRATION = ROOT/'Config/calibration.json'


def decode_diagnostic(response, axis, version=0xBE):
    checksum = 0
    for value in response:
        checksum ^= value
    if len(response) != 28 or response[:2] != b'\x5a\xf8' or checksum:
        raise IOError(f'Invalid/timeout diagnostic reply: {response.hex()}')
    status = response[2]
    if status & ~3 or (status >> 1) != axis:
        raise IOError(f'Diagnostic axis/status mismatch: {status:02x}')
    if not status & 1:
        return None
    payload = response[3:27]
    tick, seq, got_axis, flags, error, desired, planned, angle, speed, current = struct.unpack('<IHBBhhhHhh', payload[:20])
    if got_axis & (0x08 if version >= 0xBE else 0x0e) or (got_axis & 1) != axis:
        raise IOError('Diagnostic payload axis mismatch')
    xy = int.from_bytes(payload[20:23], 'little')
    row = dict(fpga_tick=tick, sequence=seq, axis=('pan', 'tilt')[axis],
               error_px=error, desired_raw=desired, planned_raw=planned,
               desired_rpm=desired*1000/32767, planned_rpm=planned*1000/32767,
               angle_deg=angle*360/65536, feedback_rpm=speed*1000/32767,
               current_a=current*10/32767, x=xy & 4095, y=xy >> 12,
               planned_command=payload[23], test_active=(got_axis >> 7) & 1,
               test_stage=((got_axis >> 4) & 7) | ((got_axis & 2) << 2 if version >= 0xBE else 0),
               test_profile=int(version >= 0xBE and bool(got_axis & 4)))
    for bit, name in enumerate(('spot_valid', 'run', 'ready', 'tracking', 'velocity_mode', 'stop_pending', 'fault', 'disabled')):
        row[name] = (flags >> bit) & 1
    return row


class TraceClock:
    """Unwrap the 50 MHz clock, including slightly out-of-order axis snapshots."""
    def __init__(self):
        self.previous = None
        self.elapsed = 0

    def seconds(self, tick):
        if self.previous is not None:
            delta = ((tick-self.previous+0x80000000) & 0xffffffff)-0x80000000
            if delta < -25000000:
                raise ValueError('FPGA time reset/stale snapshot; restart recording after homing')
            self.elapsed += delta
        self.previous = tick
        return self.elapsed/50000000


class EncoderRate:
    """Causal ~120 ms encoder slope for diagnosis, independent of reported rpm.

    Not instantaneous speed: encoder_rpm averages the preceding window.
    Reset on a long sample gap and unwrap the 0/360 degree boundary.
    """
    def __init__(self):
        self.points = deque()
        self.last_angle = self.unwrapped = None

    def sample(self, timestamp, angle):
        if self.points and not 0 < timestamp-self.points[-1][0] <= .16:
            self.points.clear()
            self.last_angle = self.unwrapped = None
        self.unwrapped = angle if self.last_angle is None else self.unwrapped + (angle-self.last_angle+180) % 360-180
        self.last_angle = angle
        self.points.append((timestamp, self.unwrapped))
        while len(self.points) > 2 and self.points[1][0] <= timestamp-.12:
            self.points.popleft()
        start, position = self.points[0]
        span = timestamp-start
        return ((self.unwrapped-position)/(6*span), span) if span >= .079 else (None, None)


def record_diagnostics(link, seconds, output=None, *, speed_sequence=False, test_request=None):
    if link.version < 0xBB:
        raise ValueError('Diagnostics requires B11 (BB); download the new green_spot_gimbal_laser.bit first')
    path = output or ROOT/'Logs'/f'vision_trace_{datetime.now():%Y%m%d_%H%M%S_%f}.csv'
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    clock = TraceClock()
    previous = [None, None]
    encoders = [EncoderRate(), EncoderRate()]
    counts, skipped = [0, 0], [0, 0]
    started = time.monotonic()
    status = link.command(0x75)  # Read-only global status, also valid after a fault.
    last_status = last_flush = last_report = started
    note = ('Speed test ends autonomously; Ctrl+C requests an early stop.' if speed_sequence else
            'Read-only capture. Tracking continues after recording; use off to stop.')
    print(f'Recording {seconds:g}s to {path}\n{note}', flush=True)
    try:
        with path.open('x', newline='', encoding='utf-8') as stream:
            writer = None
            try:
                while time.monotonic()-started < seconds:
                    for axis in (0, 1):
                        row = link.diagnostic(axis)
                        now = time.monotonic()
                        if row is None or previous[axis] == row['sequence']:
                            continue
                        gap = 0 if previous[axis] is None else ((row['sequence']-previous[axis]) & 65535)-1
                        previous[axis] = row['sequence']
                        skipped[axis] += gap
                        elapsed = clock.seconds(row['fpga_tick'])
                        encoder_rpm, encoder_window = encoders[axis].sample(elapsed, row['angle_deg'])
                        row = dict(host_elapsed_s=now-started, fpga_elapsed_s=elapsed,
                                   encoder_rpm=encoder_rpm, encoder_window_s=encoder_window,
                                   firmware=f'{link.version:02X}', global_status=status, skipped_samples=gap, **row)
                        if test_request is not None:
                            row['test_requested_axis'], row['test_requested_profile'] = test_request
                        if writer is None:
                            writer = csv.DictWriter(stream, fieldnames=list(row))
                            writer.writeheader()
                        writer.writerow(row)
                        counts[axis] += 1
                    now = time.monotonic()
                    if now-last_status >= 1:
                        status = link.command(0x75)
                        last_status = now
                    if now-last_flush >= 1:
                        stream.flush()
                        last_flush = now
                    if now-last_report >= 5:
                        print(f'{now-started:.0f}s: pan {counts[0]}, tilt {counts[1]} samples; status 0x{status:02X}', flush=True)
                        last_report = now
                    time.sleep(.002)
            except KeyboardInterrupt:
                print('\nRecording interrupted; keeping collected rows.')
    finally:
        print(f'CSV: {path}\nSamples pan/tilt: {counts}; polling skipped: {skipped} (not motor communication errors).', flush=True)
    if not sum(counts):
        raise ValueError('No diagnostic samples. Check motion status / finish homing first')
    return path


def wait_for_hold(link, timeout=3):
    """Wait for fresh post-off snapshots of both motors in angle hold."""
    baseline = [link.diagnostic(axis) for axis in (0, 1)]
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        link.require_homed()
        rows = [link.diagnostic(axis) for axis in (0, 1)]
        if all(row is not None and (old is None or row['sequence'] != old['sequence'])
               and row['ready'] and not any(row[k] for k in ('run', 'velocity_mode', 'fault', 'disabled', 'test_active'))
               for row, old in zip(rows, baseline)):
            return
        time.sleep(.04)
    raise ValueError('Motors did not confirm angle hold within 3 s; check motion status')


def read_pose(link, axis, timeout=1.5):
    """Read a fresh encoder angle without changing any motor/laser controls."""
    if link.version < 0xBB:
        raise ValueError('pose requires B11 (BB) or newer firmware')
    if axis not in (0, 1):
        raise ValueError('Axis must be 0 or 1')

    def check_status():
        status = link.motion_status()
        if status['disabled']:
            raise ValueError('Motors disabled: this firmware stops updating encoder snapshots. '
                             'A cached angle cannot measure a manually repositioned axis. '
                             'No enable/home command was sent.')
        if status['homing'] or status['home_fault'] or status['tracking_fault'] or not (
                status['pan_homed'] and status['tilt_homed']):
            raise ValueError(f'Live encoder snapshots unavailable: {status}. No motor command was sent.')

    check_status()
    baseline = link.diagnostic(axis)
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        row = link.diagnostic(axis)
        # The FPGA can keep returning an old valid snapshot after disable/fault.
        # Require both a new sequence and tick; never label the cache as live.
        if row is not None and (baseline is None or (
                row['sequence'] != baseline['sequence'] and row['fpga_tick'] != baseline['fpga_tick'])):
            check_status()
            if row['ready'] and not row['fault'] and not row['disabled']:
                return row
        time.sleep(.02)
    raise ValueError('No fresh encoder feedback; cached angle rejected. No motor command was sent.')


def verify_speed_test_start(link, axis, profile, baseline, timeout=.6):
    """Check actual latched selection during the two-second zero-speed stage.

    Never infer success from echoed host arguments or a cached diagnostic row.
    Caller always cancels/holds on mismatch, timeout or late verification.
    """
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        status = link.command(0x79)
        if not status & 0x80 or status & 0x40:
            raise ValueError(f'{profile} speed test rejected: status 0x{status:02X}')
        selected_axis = (status >> 4) & 1
        if selected_axis != axis:
            raise ValueError(f'Test axis mismatch: requested {("PAN", "TILT")[axis]}, '
                             f'FPGA selected {("PAN", "TILT")[selected_axis]}; stopping')
        stage_mask = 15 if link.version >= 0xBE else 7
        if status & stage_mask:
            raise ValueError('Test verification missed the initial zero-speed stage; stopping')
        row = link.diagnostic(axis)
        if row is not None and (baseline is None or (
                row['sequence'] != baseline['sequence'] and row['fpga_tick'] != baseline['fpga_tick'])) and row['test_active']:
            if row['test_profile'] != int(profile == 'dynamic'):
                raise ValueError(f'Test profile mismatch: requested {profile}, '
                                 f'FPGA selected {("fixed", "dynamic")[row["test_profile"]]}; stopping')
            if row['test_stage'] != 0 or row['planned_raw'] != 0 or row['velocity_mode']:
                raise ValueError('Test already moving before selection verification; stopping')
            if not row['ready'] or row['fault'] or row['disabled'] or row['run']:
                raise ValueError('Motor not ready for verified speed test; stopping')
            return
        time.sleep(.02)
    raise ValueError('No fresh speed-test selection confirmation; stopping')


def speed_test(link, axis, output=None, *, profile='fixed'):
    if profile not in ('fixed', 'dynamic') or axis not in (0, 1):
        raise ValueError('Invalid speed-test profile or axis')
    if link.version < 0xBC:
        raise ValueError('speed-test requires B12 (BC) firmware')
    dynamic = profile == 'dynamic'
    if dynamic and link.version < 0xBE:
        raise ValueError('Dynamic speed-test requires B14 (BE); download the new green_spot_gimbal_laser.bit first')
    if output is not None and Path(output).exists():
        raise FileExistsError(f'Output already exists: {output}')
    # Explicit operator invocation only. Other axis stays in angle hold.
    # FPGA bounds execution even if this process or USB connection disappears.
    try:
        link.stop_tracking()
        wait_for_hold(link)
        # Set the selectors while request is LOW, then trigger separately.
        # This also prevents a torn start byte on older B12..B14 images.
        selectors = (axis << 3) | (64 if dynamic else 0)
        echo = link.command(0x74, selectors)
        if link.version >= 0xBF and echo != selectors:
            raise ValueError('Test selector write was not acknowledged; stopping')
        time.sleep(.02)
        baseline = link.diagnostic(axis)
        echo = link.command(0x74, selectors | 4)
        if link.version >= 0xBF and echo != (selectors | 4):
            raise ValueError('Test start write was not acknowledged; stopping')
        time.sleep(.06)
        verify_speed_test_start(link, axis, profile, baseline)
        print(f'FPGA verified: {("PAN", "TILT")[axis]} {profile}, initial zero-speed stage.', flush=True)
        sequence = ('0, +0.31, +0.61, +0.31, 0, -0.31, -0.61, -0.31, 0' if dynamic else
                    '0, +0.49, +1.01, 0, -0.49, -1.01, 0')
        print(f'{("PAN", "TILT")[axis]} {profile} speed test: {sequence} rpm; '
              '2 s per stage. Laser OFF, other axis holds. Ctrl+C stops early.', flush=True)
        path = record_diagnostics(link, 22 if dynamic else 18, output,
                                  speed_sequence=True, test_request=(axis, int(dynamic)))
        status = link.command(0x79)
        if status & 0x80 and not status & 0x40:
            print('Recording stopped early; stopping speed test and confirming hold.')
            return path
        if status & 0xc0 or not status & 0x20:
            raise ValueError(f'Test interrupted/failed: status 0x{status:02X}; collected CSV retained')
        print(f'{profile.capitalize()} speed sequence complete; both motors holding their final angles.')
        return path
    finally:
        link.stop_tracking()
        wait_for_hold(link)


def calibration_centre(coeff):
    a,b,c,d,e,f=coeff
    det=a*e-b*d
    if not det: raise ValueError('Singular calibration')
    u,v=32768*256-c,32768*256-f
    x,y=round((u*e-b*v)/det),round((a*v-u*d)/det)
    if not (0<=x<1280 and 0<=y<720): raise ValueError('Calibrated centre is outside camera image')
    return x,y


def solve(matrix, values):
    rows = [list(map(float, row)) + [float(v)] for row, v in zip(matrix, values)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda i: abs(rows[i][col]))
        rows[col], rows[pivot] = rows[pivot], rows[col]
        if abs(rows[col][col]) < 1e-9:
            raise ValueError('Calibration points are collinear or too close')
        scale = rows[col][col]
        rows[col] = [v / scale for v in rows[col]]
        for i in range(3):
            if i != col:
                scale = rows[i][col]
                rows[i] = [a-scale*b for a,b in zip(rows[i], rows[col])]
    return [row[3] for row in rows]


def fit(samples):
    # Centre coordinates before solving to avoid poor conditioning at 1280x720.
    ox = statistics.mean(s['pixel'][0] for s in samples)
    oy = statistics.mean(s['pixel'][1] for s in samples)
    rows = [[s['pixel'][0]-ox, s['pixel'][1]-oy, 1] for s in samples]
    normal = [[sum(r[i]*r[j] for r in rows) for j in range(3)] for i in range(3)]
    coeff = []
    for axis in range(2):
        rhs = [sum(r[i]*s['dac'][axis] for r,s in zip(rows,samples)) for i in range(3)]
        a,b,c = solve(normal, rhs)
        coeff.extend([a,b,c-a*ox-b*oy])
    quantized = [round(v*256) for v in coeff]
    for i,v in enumerate(quantized):
        bits = 32 if i in (2,5) else 24
        if not -(1 << (bits-1)) <= v < 1 << (bits-1):
            raise ValueError('Calibration coefficient out of FPGA range')
    residual = max(math.hypot(*[
        sum(quantized[j*3+i]*([*s['pixel'],1][i]) for i in range(3))/256-s['dac'][j]
        for j in range(2)]) for s in samples)
    if residual > 150:
        raise ValueError(f'Calibration residual {residual:.1f} DAC exceeds 150; check spot detection / use smaller span')
    return quantized, residual


class Link:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.5, write_timeout=0.5)
        self.ser.reset_input_buffer()
        self.version = self.command(0x26, 0)
        if self.version not in (0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4):
            self.ser.close()
            raise ValueError('Wrong FPGA image; download green_spot_rectangle.bit first')

    def command(self, cmd, data=0):
        self.ser.write(bytes([0xA5, cmd, data, 0xA5 ^ cmd ^ data]))
        response = self.ser.read(4)
        if len(response) != 4 or response[0] != 0x5A or response[1] != (cmd | 0x80) or response[3] != (response[0]^response[1]^response[2]):
            raise IOError(f'Invalid/timeout FPGA reply: {response.hex()}')
        return response[2]

    def target(self):
        data = [self.command(0x20, i) for i in range(9)]
        return {'valid':bool(data[0]), 'x':data[1]+256*data[2], 'y':data[3]+256*data[4],
                'width':data[5]+256*data[6], 'height':data[7]+256*data[8]}

    def diagnostic(self, axis):
        if self.version < 0xBB:
            raise ValueError('Diagnostics requires B11 (BB) firmware')
        if axis not in (0, 1):
            raise ValueError('Axis must be 0 or 1')
        self.ser.write(bytes([0xA5, 0x78, axis, 0xA5 ^ 0x78 ^ axis]))
        return decode_diagnostic(self.ser.read(28), axis, self.version)

    def configure(self, coeff=None, manual=(32768,32768), radius=20, mode=0, aim=(546,252)):
        if not 4<=radius<=64: raise ValueError('radius must be 4..64 pixels')
        self.command(0x72, 0)
        packet = struct.pack('<6i2HB', *(coeff or [0]*6), *manual, radius)
        packet += (aim[0] | (aim[1]<<12)).to_bytes(3,'little')
        for i,value in enumerate(packet):
            self.command(0x70, i)
            self.command(0x71, value)
            if self.command(0x73) != value: raise IOError(f'Config readback failed at byte {i}')
        self.command(0x72, mode)

    def motion_status(self):
        if self.version<0xB3: raise ValueError('Download green_spot_gimbal.bit (B3) first')
        s=self.command(0x75)
        return dict(pan_homed=bool(s&1),tilt_homed=bool(s&2),homing=bool(s&4),
                    home_fault=bool(s&8),tracking=bool(s&16),disabled=bool(s&32),tracking_fault=bool(s&64),speed_test=bool(s&128))

    def stop_tracking(self):
        # Blank before releasing the gimbal's bounded projection path.
        self.command(0x72,0)
        # Stopping must not release the disable latch and trigger boot homing.
        if self.version>=0xB3: self.command(0x74,2 if self.command(0x75)&32 else 0)

    def require_homed(self):
        s=self.motion_status()
        if s['homing'] or s['home_fault'] or s['tracking_fault'] or not s['pan_homed'] or not s['tilt_homed']:
            raise ValueError(f'Motors not ready: {s}; use motion to monitor, or home to retry')

    def sample(self):
        samples=[]
        for _ in range(20):
            s=self.target()
            if not s['valid']: raise ValueError('No compact calibration spot detected; inspect HDMI image / thresholds')
            samples.append(s)
            time.sleep(0.06)
        for axis in ('x','y'):
            if max(s[axis] for s in samples)-min(s[axis] for s in samples)>4:
                raise ValueError('Calibration spot is moving; fix the camera and screen')
        return [statistics.median(s[k] for s in samples) for k in ('x','y')]

    def verify_dark(self, pixel):
        self.command(0x72,0)
        time.sleep(0.3)  # Drain camera/DDR frames after blanking the light.
        for _ in range(3):
            s=self.target()
            if s['valid'] and math.hypot(s['x']-pixel[0],s['y']-pixel[1])<10:
                raise ValueError('Bright spot remains with laser OFF; likely background/reflection, calibration rejected')
            time.sleep(0.06)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['read','watch','detect','calibrate','track','off','motion','home','disable','diagnose','speed-test','pose'])
    p.add_argument('--axis', choices=['pan','tilt'], default='tilt', help='pose/speed-test: selected motor (default tilt)')
    p.add_argument('--profile', choices=['fixed','dynamic'], default='fixed', help='speed-test: fixed stages or continuous low-speed changes (B14)')
    p.add_argument('--seconds',type=float,default=20,help='diagnose: capture duration, 1..60 seconds')
    p.add_argument('--output',type=Path,help='diagnose/speed-test: new CSV path (default vision_lab/logs/vision_trace_TIMESTAMP.csv)')
    p.add_argument('--port',default='COM5')
    p.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION)
    p.add_argument('--radius',type=int,default=20,help='rectangle half-size in camera pixels, 4..64')
    p.add_argument('--span',type=int,default=1800,help='calibration DAC half-span, 500..3000')
    p.add_argument('--brightness',type=int,default=160)
    p.add_argument('--chroma',type=int,default=40)
    p.add_argument('--white-calibration',action='store_true',help='B2 firmware: acquire a compact bright white core for calibration only')
    p.add_argument('--gimbal',action='store_true',help='B3: move both motors toward calibrated optical centre')
    p.add_argument('--no-laser',action='store_true',help='track with gimbal only, keep laser dark')
    p.add_argument('--invert-pan',action='store_true')
    p.add_argument('--invert-tilt',action='store_true')
    args=p.parse_args()
    if args.profile != 'fixed' and args.action != 'speed-test':
        p.error('--profile dynamic requires speed-test')
    if not 1<=args.seconds<=60:
        p.error('seconds must be 1..60')
    if not 4<=args.radius<=64 or not 500<=args.span<=3000:
        p.error('radius must be 4..64; span must be 500..3000')
    if not 0<=args.brightness<=255 or not 1<=args.chroma<=255:
        p.error('brightness 0..255; chroma 1..255')
    link=Link(args.port)
    try:
        if args.action=='diagnose':
            record_diagnostics(link,args.seconds,args.output)
        elif args.action=='pose':
            row=read_pose(link, ('pan','tilt').index(args.axis))
            print(f"{row['axis']} angle_deg={row['angle_deg']:.6f} "
                  f"speed_rpm={row['feedback_rpm']:.3f} current_a={row['current_a']:.3f}")
            print('Fresh encoder angle (not relative to logical home). Read-only; controls unchanged.')
            if any(row[k] for k in ('run','test_active','velocity_mode','stop_pending')):
                print('Motion/stop sequence active. Use off and wait for the axis to settle before choosing a home angle.')
        elif args.action=='speed-test':
            speed_test(link, ('pan','tilt').index(args.axis), args.output, profile=args.profile)
        elif args.action=='off':
            link.stop_tracking()
            print('Tracking stopped; motors decelerate and hold final angle. Laser OFF.')
        elif args.action=='motion': print(link.motion_status())
        elif args.action in ('home','disable'):
            link.motion_status()  # Require B3 before issuing motor commands.
            link.stop_tracking()
            if args.action=='home':
                link.command(0x77,0);print('Both axes returning to fixed home; check motion status.')
            else:
                link.command(0x74,2);print('Both axes disabling; support the unbalanced upper assembly.')
        elif args.action in ('read','watch','detect'):
            if args.action=='detect':
                link.stop_tracking()
                link.command(0x21,1)
                link.command(0x52,args.chroma); link.command(0x53,args.brightness)
                print('Green/white-core detection only; laser OFF. Ctrl+C exits monitoring.')
            while True:
                print(link.target(),flush=True)
                if args.action=='read': break
                time.sleep(.2)
        elif args.action=='calibrate':
            if args.white_calibration and link.version < 0xB2:
                raise ValueError('White calibration needs green_spot_rectangle_white_calibration.bit (B2)')
            link.stop_tracking()
            if link.version>=0xB3:
                link.require_homed()
                time.sleep(1)  # Let any in-progress tracking ramp stop before sampling.
            link.command(0x21,2 if args.white_calibration else 0)
            link.command(0x52,args.chroma); link.command(0x53,args.brightness)
            samples=[]
            try:
                print('Fix the camera/gimbal and screen. Turn off the external green spot.\nThe FPGA will draw four small RED calibration squares. Check them on HDMI.')
                if args.white_calibration:
                    print('WHITE-core acquisition enabled. Confirm the cross is on the laser spot, not a reflection. Use --brightness 224 initially.')
                input('Press Enter to start (Ctrl+C cancels): ')
                for sx,sy in [(-1,-1),(1,-1),(-1,1),(1,1)]:
                    dac=[32768+sx*args.span,32768+sy*args.span]
                    link.configure(manual=dac,mode=2)
                    time.sleep(.5)
                    input(f'DAC {dac}: confirm the small red square is visible and still; press Enter to sample: ')
                    pixel=link.sample()
                    if args.white_calibration:
                        link.verify_dark(pixel)
                    samples.append({'pixel':pixel,'dac':dac})
                    print('camera centre:',pixel)
                coeff,residual=fit(samples)
                data={'format':1,'coeff_q8':coeff,'samples':samples,'residual_dac':residual,'radius':args.radius}
                args.calibration.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
                print(f'Saved {args.calibration}; max fit error {residual:.1f} DAC. Run track next.')
            finally:
                link.command(0x72,0); link.command(0x21,1)
        else:
            data=json.loads(args.calibration.read_text(encoding='utf-8'))
            # Recompute instead of trusting a stale or edited coefficient array.
            coeff,residual=fit(data['samples'])
            link.stop_tracking(); link.command(0x21,1)
            link.command(0x52,args.chroma); link.command(0x53,args.brightness)
            centre=calibration_centre(coeff)
            if args.gimbal: link.require_homed()
            if args.no_laser and not args.gimbal: raise ValueError('--no-laser requires --gimbal')
            if args.gimbal:
                flags=1 | (16 if args.invert_pan else 0) | (32 if args.invert_tilt else 0)
                # Select bounded projection while mode is still dark. The motor
                # run condition also requires the committed mode to be 1 or 3.
                link.command(0x74,flags)
            link.configure(coeff=coeff,radius=args.radius,mode=3 if args.no_laser else 1,aim=centre)
            if args.gimbal:
                laser_note = 'OFF' if args.no_laser else ('ON during chase, rectangle centre clamped to +/-24px' if link.version>=0xC3 else 'near-centre only')
                print(f'Gimbal tracking (firmware {link.version:02X}) toward calibration centre {centre}; up to 3 rpm, stop/resume 12/20px; continuous velocity, delayed stationary hold. Laser {laser_note}.')
                if link.version>=0xC3 and not args.no_laser:
                    print('B19: outside the optical window, the red box follows the gimbal at the window edge; it encloses the green spot after catching up. Loss/fault/off still blanks light.')
                if link.version>=0xC4:
                    print('B20: up to 25% faster pursuit beyond 40px; 60/100px now 0.79/1.37 rpm. Fine motion, acceleration/deceleration and 3 rpm cap retained.')
                elif link.version>=0xC2:
                    print('B18: faster TILT fine motion (20/30 px: 0.24/0.43 rpm); near braking only above base+0.12 rpm; 3 rpm cap.')
                elif link.version>=0xC1:
                    print('B17: TILT startup boost fades over 10 updates (~400 ms); encoder braking only within 40 px; 3 rpm cap.')
                elif link.version>=0xC0:
                    print('B16: TILT encoder-based lead compensation; faster startup and earlier braking; 3 rpm cap.')
                elif link.version>=0xBC: print('B12: 40 ms start confirmation; stronger near/mid gain; encoder-confirmed reversal braking.')
            if not args.no_laser:
                print(f'Autonomous GREEN tracking enabled, box {2*args.radius} x {2*args.radius} pixels. Target loss blanks laser.')
            print('Use off to stop; reload track after FPGA power cycle. Optical alignment varies with screen distance and tilt.')
    finally:
        link.ser.close()


if __name__=='__main__':
    try: main()
    except KeyboardInterrupt: print('\nCancelled.')
    except (OSError,ValueError,serial.SerialException) as exc:
        print(f'FAIL: {exc}',file=sys.stderr); sys.exit(1)
