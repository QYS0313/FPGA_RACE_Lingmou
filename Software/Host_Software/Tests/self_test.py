import sys, queue
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'App'))
import demo_gui
from demo_link import ROOT as app_root
import serial
assert app_root==ROOT
class Worker:
    def __init__(self):
        self.tasks=queue.Queue();self.events=queue.Queue();self.requested=(0,0);self.ui_tick=0
    def is_alive(self):return False
root=demo_gui.tk.Tk();root.withdraw()
worker=Worker();app=demo_gui.App(root,worker);root.update_idletasks()
assert len(app.tabs.tabs())==2
assert Path(app.calibration.get())==ROOT/'Config/calibration.json'
assert Path(app.calibration.get()).is_file()
app.apply_video_motion()
assert worker.tasks.get_nowait()==('video_motion',(18,3))
assert worker.tasks.empty()
assert str(ROOT) in serial.__file__
root.destroy()
print('PASS: bundled Python/Tk/serial, two GUI pages, integrated tracking model, local calibration, motion controls. No serial connection.')
