"""Exercise the real Windows graphics context without opening a UART."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import tkinter as tk
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'App'))
from gimbal_gl import DetailedGimbalCanvas

parser=argparse.ArgumentParser()
parser.add_argument('output',type=Path)
args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
root=tk.Tk();root.title('Gimbal renderer verification');root.geometry('1100x1000+0+0')
c=DetailedGimbalCanvas(root,width=1100,height=1000);c.pack(fill='both',expand=True)
result={}
def verify():
    try:
        assert c._ready and c.error is None,c.error
        assert c.last_gl_error==0,c.last_gl_error
        c.export_frame(args.output/'model_detail_home.png')
        c.pan=-18;c.tilt=24;c.render()
        c.export_frame(args.output/'model_detail_pose.png')
        first=hashlib.sha256((args.output/'model_detail_home.png').read_bytes()).digest()
        second=hashlib.sha256((args.output/'model_detail_pose.png').read_bytes()).digest()
        assert first!=second,'Joint movement did not change rendered pixels'
        start=time.perf_counter()
        for i in range(30):
            c.pan=-18+i*.8;c.tilt=24-i*.6;c._render_now()
            assert c.last_gl_error==0,c.last_gl_error
        result.update(passed=True,renderer=c.renderer_name,parts=c.part_count,triangles=c.triangle_count,
                      samples=c.samples,frames=30,seconds=time.perf_counter()-start,serial_opened=False)
    except Exception as exc:
        result.update(passed=False,error=str(exc))
    finally:
        root.destroy()
        result['native_context_released']=c.hrc is None and c.hdc is None and c.hwnd is None
        (args.output/'gpu_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
root.after(1500,verify)
root.mainloop()
print(json.dumps(result,ensure_ascii=False))
if not result.get('passed') or not result.get('native_context_released'):raise SystemExit(1)
