"""Loopback-only HTML/CSS desktop console, backed by the existing single UART worker."""
from collections import deque
import argparse
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
import os
from pathlib import Path
import queue
import secrets
import subprocess
import threading
import time
import traceback
from urllib.parse import unquote,urlsplit

from demo_gui import Worker
from demo_link import ROOT
from serial.tools import list_ports

HOME=(21285*360/65536,180.0)


class ConsoleService:
    def __init__(self,preview=False,demo=False,worker=None,root=ROOT):
        self.root=Path(root);self.preview=preview
        self.worker=worker if worker is not None else (None if preview else Worker())
        self.lock=threading.RLock();self.logs=deque(maxlen=160);self.event_id=0
        self.state=dict(image=None,shape_capability=False,poses=[None,None],target=None,
                        video_mode=None,video_options=None,video_stats=None,video_motion=None,shape=None,
                        shape_phase='等待连接',status='未连接设备',recording=False,video_options_revision=0)
        self.started=time.monotonic();self.last_client=self.started;self.seen_client=False
        self.demo=bool(demo and preview);self.demo_started=self.started
        self.calibration=self.root/'Config/calibration.json'
        self.model_config=self._load_model_config()
        self._closed=False
        if self.worker is not None and isinstance(self.worker,threading.Thread):self.worker.start()
        self.log('info','控制台已就绪，等待连接设备')

    def _load_model_config(self):
        try:
            data=json.loads((self.root/'Config/gimbal_model.json').read_text(encoding='utf-8'))
            self._validate_model_config(data);return data
        except (OSError,ValueError,TypeError,KeyError):
            return dict(reference_deg=list(HOME),direction=[1,-1])

    @staticmethod
    def _validate_model_config(data):
        refs=data['reference_deg'];signs=data['direction']
        if len(refs)!=2 or len(signs)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<360 for v in refs) or any(type(v) is not int or v not in (-1,1) for v in signs):
            raise ValueError('模型零位需在0～360°之间，显示方向为正向或反向')

    def log(self,level,message):
        self.event_id+=1
        self.logs.append(dict(id=self.event_id,time=time.strftime('%H:%M:%S'),level=level,message=str(message)))

    def _drain(self):
        if not self.worker:return
        while True:
            try:kind,data=self.worker.events.get_nowait()
            except queue.Empty:break
            if kind=='connection':
                self.state['image']=data;self.state['poses']=[None,None]
                self.state['video_mode']=None;self.state['video_options']=None
                self.state['shape_capability']=False;self.demo=False
                self.state['target']=None;self.state['shape']=None
            elif kind=='pose':
                row=dict(data);axis=row.get('axis')
                if axis in (0,1):
                    row.setdefault('received_at',time.monotonic());self.state['poses'][axis]=row
            elif kind=='shape_capability':self.state['shape_capability']=bool(data)
            elif kind in self.state:
                self.state[kind]=data
                if kind=='video_options':
                    self.state['video_options_revision']+=1
                    if data is not None:self.state['video_mode']=data['mode']
            elif kind in ('log','error'):
                self.log('error' if kind=='error' else 'info',data)
                if '开始记录' in str(data):self.state['recording']=True
                if '记录已保存' in str(data):self.state['recording']=False

    def snapshot(self):
        with self.lock:
            self._drain();now=time.monotonic();state=copy.deepcopy(self.state)
            if self.demo:
                t=now-self.demo_started
                offsets=(26*math.sin(t*.35),18*math.sin(t*.47+.5))
                velocities=(26*.35*math.cos(t*.35)/6,18*.47*math.cos(t*.47+.5)/6)
                state['poses']=[dict(axis=i,angle_deg=(self.model_config['reference_deg'][i]+offsets[i]/self.model_config['direction'][i])%360,
                    speed_rpm=velocities[i]/self.model_config['direction'][i],current_a=.18+.06*abs(math.sin(t*.6+i)),enabled=False,
                    received_at=now,age_s=0,sample_id=f'demo-{now:.3f}') for i in (0,1)]
            for row in state['poses']:
                if row:
                    row['age_s']=max(0,now-row['received_at'])
                    row.setdefault('sample_id',row['received_at'])
            state.update(demo=self.demo,preview=self.preview,logs=list(self.logs),model_config=self.model_config,
                         calibration_name=self.calibration.name,ports=[p.device for p in list_ports.comports()],
                         server_time=now,demo_elapsed=now-self.demo_started if self.demo else 0)
            return state

    def heartbeat(self,data):
        with self.lock:
            self.last_client=time.monotonic();self.seen_client=True
            if self.worker:
                self.worker.ui_tick=self.last_client
                desired=data.get('requested',[0,0])
                if not isinstance(desired,list) or len(desired)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>10 for v in desired):
                    raise ValueError('无效运动请求')
                self.worker.requested=tuple(desired) if data.get('view')=='video' and self.state['image']==0xD1 else (0,0)

    def command(self,action,value=None,flash_ready=False):
        allowed={'connect','disconnect','stop','disable','home','track','laser','detect','shape_detect','shape_draw',
                 'record','switch','video','video_motion','motion_reset','video_options','read_video_options'}
        if action not in allowed:raise ValueError('不支持的操作')
        with self.lock:
            self._drain()
            if self.preview:raise ValueError('离线预览未打开设备通道')
            if action=='connect':
                if not isinstance(value,str) or not value.upper().startswith('COM') or not value[3:].isdigit():raise ValueError('请选择有效的串口')
                value=value.upper()
                if self.state['image'] is not None:raise ValueError('请先断开当前设备')
            elif self.state['image'] is None:raise ValueError('请先连接板卡串口')
            if action=='switch' and (flash_ready is not True or type(value) is not int or value not in (0,1)):
                raise ValueError('切换前请确认已按说明配置双镜像Flash')
            if action=='video':
                if not isinstance(value,list) or len(value)!=2 or type(value[0]) is not int or value[0] not in (0x10,0x11,0x21,0x64) or type(value[1]) is not int or not 0<=value[1]<=5:
                    raise ValueError('无效视频设置')
                if (value[0]==0x10 and value[1]>1) or (value[0]==0x21 and value[1]>2):raise ValueError('无效视频设置')
            if action=='track':
                if not isinstance(value,dict):raise ValueError('跟踪参数无效')
                value=(str(self.calibration),int(value['brightness']),int(value['chroma']),int(value['radius']),bool(value['laser']))
            if action in ('shape_detect','shape_draw'):
                if not isinstance(value,dict):raise ValueError('形状参数无效')
                value=(str(self.calibration),value['color'],int(value['seconds']))
            if action in ('stop','disable','disconnect'):
                self.worker.requested=(0,0)
                while True:
                    try:self.worker.tasks.get_nowait()
                    except queue.Empty:break
            elif self.worker.tasks.qsize()>12:raise ValueError('设备正在处理请求，请稍候')
            self.demo=False
            self.worker.tasks.put((action,value))
            self.log('info',dict(connect='正在连接设备',disconnect='正在断开设备',home='已提交双轴回位请求',
                                 stop='已提交停止 / 关光请求',disable='已提交双轴失能请求',track='正在启动光点跟踪',
                                 record='已提交记录操作').get(action,'已提交操作，等待设备反馈'))

    def set_demo(self,enabled):
        with self.lock:
            self._drain()
            if self.state['image'] is not None:raise ValueError('连接设备时使用实际反馈，不能开启离线演示')
            self.demo=bool(enabled);self.demo_started=time.monotonic()
            self.log('info','离线动作演示已开启，数值为演示数据' if self.demo else '离线动作演示已关闭')

    def save_model_config(self,data):
        self._validate_model_config(data)
        with self.lock:
            clean=dict(reference_deg=data['reference_deg'],direction=data['direction'])
            (self.root/'Config/gimbal_model.json').write_text(json.dumps(clean,indent=2)+'\n',encoding='utf-8')
            self.model_config=clean;self.log('info','模型显示标定已保存')

    def import_calibration(self,data):
        if not isinstance(data,dict) or not isinstance(data.get('samples'),list) or len(data['samples'])<4:
            raise ValueError('标定文件需包含至少四个samples')
        coeff=data.get('coeff_q8')
        if not isinstance(coeff,list) or len(coeff)!=6 or any(type(v) is not int for v in coeff):
            raise ValueError('标定文件需包含六个整数coeff_q8')
        with self.lock:
            path=self.root/'Config/calibration_imported.json'
            path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            self.calibration=path;self.log('info','已加载新的视觉标定文件')

    def release_client(self):
        # Release the device immediately; keep the local page alive for a reload.
        with self.lock:
            if self.worker:
                self.worker.requested=(0,0)
                while True:
                    try:self.worker.tasks.get_nowait()
                    except queue.Empty:break
                self.worker.tasks.put(('disconnect',None))

    def close(self):
        with self.lock:
            if self._closed:return
            self._closed=True
            if self.worker:
                self.worker.requested=(0,0)
                while True:
                    try:self.worker.tasks.get_nowait()
                    except queue.Empty:break
                self.worker.tasks.put(('quit',None))
        if self.worker and isinstance(self.worker,threading.Thread):self.worker.join(timeout=9)


class ConsoleHTTPServer(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,service,port=0):
        self.service=service;self.token=secrets.token_urlsafe(24)
        super().__init__(('127.0.0.1',port),ConsoleHandler)
        self.origin=f'http://127.0.0.1:{self.server_port}'
        self.url=f'{self.origin}/{self.token}/'


class ConsoleHandler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass

    def _path(self):
        prefix='/'+self.server.token+'/'
        path=unquote(urlsplit(self.path).path)
        if not path.startswith(prefix):raise PermissionError('Invalid local session')
        return path[len(prefix):]

    def _send(self,status,body,ctype='application/json; charset=utf-8',extra=None):
        if isinstance(body,(dict,list)):body=json.dumps(body,ensure_ascii=False,allow_nan=False).encode('utf-8')
        self.send_response(status);self.send_header('Content-Type',ctype);self.send_header('Content-Length',str(len(body)))
        self.send_header('X-Content-Type-Options','nosniff');self.send_header('Referrer-Policy','no-referrer')
        self.send_header('Cache-Control','no-store' if ctype.startswith('application/json') else 'private, max-age=300')
        self.send_header('Content-Security-Policy',"default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
        for key,value in (extra or {}).items():self.send_header(key,value)
        self.end_headers();self.wfile.write(body)

    def do_GET(self):
        try:
            path=self._path()
            if path=='api/state':self._send(200,self.server.service.snapshot());return
            root=(self.server.service.root/'Web').resolve();target=(root/(path or 'index.html')).resolve()
            if not target.is_relative_to(root) or not target.is_file():self._send(404,dict(error='Not found'));return
            ctype='text/javascript' if target.suffix=='.js' else mimetypes.guess_type(str(target))[0] or 'application/octet-stream'
            compressed=target.with_name(target.name+'.gz')
            if 'gzip' in self.headers.get('Accept-Encoding','') and compressed.is_file():
                self._send(200,compressed.read_bytes(),ctype,{'Content-Encoding':'gzip','Vary':'Accept-Encoding'})
            else:self._send(200,target.read_bytes(),ctype)
        except PermissionError:self._send(403,dict(error='Invalid local session'))
        except (BrokenPipeError,ConnectionResetError):pass

    def do_POST(self):
        try:
            path=self._path()
            if self.headers.get('Origin')!=self.server.origin:raise PermissionError('Origin not allowed')
            length=int(self.headers.get('Content-Length','0'))
            if not 0<length<=131072:raise ValueError('无效请求长度')
            data=json.loads(self.rfile.read(length));service=self.server.service
            if path=='api/heartbeat':service.heartbeat(data)
            elif path=='api/command':service.command(data['action'],data.get('value'),data.get('flash_ready',False))
            elif path=='api/demo':service.set_demo(data.get('enabled',False))
            elif path=='api/model-config':service.save_model_config(data)
            elif path=='api/calibration':service.import_calibration(data)
            elif path=='api/release':service.release_client()
            elif path=='api/quit':
                threading.Thread(target=self._shutdown,daemon=True).start()
            else:raise ValueError('不支持的接口')
            self._send(200,dict(ok=True))
        except PermissionError:self._send(403,dict(error='Origin or local session not allowed'))
        except (ValueError,KeyError,TypeError,OSError) as exc:self._send(400,dict(error=str(exc)))
        except (BrokenPipeError,ConnectionResetError):pass

    def _shutdown(self):
        self.server.service.close();self.server.shutdown()


def launch_browser(url):
    options=[Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft/Edge/Application/msedge.exe',
             Path(os.environ.get('ProgramFiles','C:/Program Files'))/'Microsoft/Edge/Application/msedge.exe',
             Path(os.environ.get('ProgramFiles','C:/Program Files'))/'Google/Chrome/Application/chrome.exe']
    for browser in options:
        if browser.is_file():
            subprocess.Popen([str(browser),'--app='+url,'--new-window','--window-size=1580,980','--window-position=30,20'],
                             stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            return
    import webbrowser
    webbrowser.open(url,new=1)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preview',action='store_true')
    parser.add_argument('--demo',action='store_true');parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--port',type=int,default=0);parser.add_argument('--ready-file',type=Path)
    args=parser.parse_args()
    for name in ('index.html','console.css','console.js','model_view.js','assets/assembly.json','assets/three.module.js'):
        if not (ROOT/'Web'/name).is_file():raise FileNotFoundError('请保留完整的Host_Software目录：缺少Web/'+name)
    service=ConsoleService(preview=args.preview,demo=args.demo)
    server=ConsoleHTTPServer(service,args.port)
    if args.ready_file:args.ready_file.write_text(json.dumps(dict(url=server.url,pid=os.getpid())),encoding='utf-8')
    if not args.no_browser:launch_browser(server.url)
    def watchdog():
        while not service._closed:
            time.sleep(1)
            if (service.seen_client and time.monotonic()-service.last_client>15) or (not service.seen_client and time.monotonic()-service.started>60):
                service.close();server.shutdown();break
    threading.Thread(target=watchdog,daemon=True).start()
    try:server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:service.close();server.server_close()

if __name__=='__main__':
    try:main()
    except Exception as exc:
        folder=ROOT/'Logs';folder.mkdir(exist_ok=True)
        (folder/'desktop_startup_error.txt').write_text(traceback.format_exc(),encoding='utf-8')
        if os.name=='nt':
            import ctypes
            ctypes.windll.user32.MessageBoxW(None,'上位机无法启动：'+str(exc)+'\n详细信息见 Logs/desktop_startup_error.txt','灵眸控制台',16)
        raise
