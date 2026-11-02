"""Run the real tracking layout with no UART worker; optional renderer QA output."""
import argparse
import json
from pathlib import Path
import queue
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'App'))
import demo_gui

class OfflineWorker:
    def __init__(self):
        self.tasks=queue.Queue();self.events=queue.Queue();self.requested=(0,0);self.ui_tick=0
    def is_alive(self):return False

parser=argparse.ArgumentParser()
parser.add_argument('--report')
parser.add_argument('--export-model')
parser.add_argument('--close-after',type=int,default=0)
parser.add_argument('--demo',action='store_true')
args=parser.parse_args()
root=demo_gui.tk.Tk();app=demo_gui.App(root,OfflineWorker())
root.title('灵眸 FPGA · 精细模型预览（离线）')
if args.demo:app.gimbal.toggle_demo()
def report():
    c=app.gimbal.canvas
    data=dict(renderer=c.renderer_name,error=c.error,triangles=c.triangle_count,parts=c.part_count,
              gl_error=c.last_gl_error,samples=c.samples,viewport=[c.winfo_width(),c.winfo_height()],
              tabs=len(app.tabs.tabs()),current_tab=app.tabs.index('current'),serial_opened=False)
    if args.export_model:
        try:data['export_size']=c.export_frame(args.export_model)
        except Exception as exc:data['export_error']=str(exc)
    if args.report:Path(args.report).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
root.after(1800,report)
if args.close_after:root.after(args.close_after,root.destroy)
root.mainloop()
