"""Offline checks of joint geometry, telemetry freshness and GUI integration."""
import io
import math
from pathlib import Path
import queue
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'App'))
import demo_gui
from gimbal_view import HOME, PoseState, angle_delta, transform


class Clock:
    def __init__(self):self.now=10.0
    def __call__(self):return self.now


def sample(axis,angle,**kw):
    return dict(axis=axis,angle_deg=angle,speed_rpm=.5,current_a=.2,enabled=True,**kw)


class PoseTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.state=PoseState(self.clock);self.state.connect(True)

    def test_encoder_wrap_and_reference(self):
        self.assertAlmostEqual(angle_delta(.5,359.5),1)
        self.assertAlmostEqual(angle_delta(359.5,.5),-1)
        self.state.feed(sample(0,HOME[0]+25))
        self.state.feed(sample(1,HOME[1]-15))
        self.assertEqual(self.state.offsets(),(25,15))
        self.state.direction[0]=-1
        self.assertEqual(self.state.offsets(),(-25,15))

    def test_stale_feedback_freezes_and_reconnect_clears(self):
        self.assertEqual(self.state.status(0),'waiting')
        self.state.feed(sample(0,HOME[0]+20))
        self.assertEqual(self.state.status(0),'live')
        self.clock.now+=2
        self.assertEqual(self.state.status(0),'stale')
        self.assertEqual(self.state.offsets()[0],20)
        self.state.connect(False)
        self.assertEqual(self.state.status(0),'offline')
        self.assertEqual(self.state.offsets(),(0,0))

    def test_invalid_and_out_of_order_feedback(self):
        self.assertFalse(self.state.feed(sample(0,float('nan'))))
        self.assertFalse(self.state.feed(sample(2,10)))
        self.assertTrue(self.state.feed(sample(0,20,received_at=9)))
        self.assertFalse(self.state.feed(sample(0,30,received_at=8)))
        self.assertFalse(self.state.feed(sample(0,30,received_at=9)))
        self.assertEqual(self.state.axes[0].angle,20)

    def test_demo_cannot_override_a_connected_device(self):
        self.assertFalse(self.state.start_demo())
        self.state.connect(False)
        self.assertTrue(self.state.start_demo())
        self.state.step_demo()
        before=self.state.offsets()
        self.clock.now+=1;self.state.step_demo()
        self.assertNotEqual(before,self.state.offsets())
        self.assertEqual(self.state.status(0),'demo')
        self.state.connect(True)
        self.assertFalse(self.state.demo)
        self.assertEqual(len(self.state.axes[0].history),0)

    def test_history_is_bounded(self):
        for i in range(300):
            self.clock.now+=.05;self.state.feed(sample(0,i))
        self.assertEqual(len(self.state.axes[0].history),240)


class GeometryTests(unittest.TestCase):
    def test_pitch_does_not_move_support_or_base(self):
        self.assertEqual(transform((100,0,235),'base',90,45),(100,0,235))
        a=transform((100,0,235),'pan',90,0)
        b=transform((100,0,235),'pan',90,45)
        self.assertEqual(a,b)
        self.assertAlmostEqual(a[0],0,places=8);self.assertAlmostEqual(a[1],100)

    def test_pitch_pivot_stays_fixed_and_nose_tilts_up(self):
        self.assertEqual(transform((60,0,235),'body',0,45),(60,0,235))
        p=transform((0,100,235),'body',0,90)
        self.assertAlmostEqual(p[1],0,places=8);self.assertAlmostEqual(p[2],335)


class FakeWorker:
    def __init__(self):
        self.tasks=queue.Queue();self.events=queue.Queue();self.requested=(0,0);self.ui_tick=0
    def is_alive(self):return False


class GuiTests(unittest.TestCase):
    def test_existing_tracking_page_receives_both_axes_without_motor_commands(self):
        root=demo_gui.tk.Tk();root.withdraw()
        try:
            worker=FakeWorker();app=demo_gui.App(root,worker)
            self.assertEqual(len(app.tabs.tabs()),2)
            self.assertEqual(app.tabs.index('current'),1)
            worker.events.put(('connection',0xD2))
            worker.events.put(('pose',sample(0,HOME[0]+22)))
            worker.events.put(('pose',sample(1,HOME[1]+35)))
            app.update();root.update_idletasks()
            self.assertEqual(app.gimbal.state.offsets(),(22,-35))
            self.assertEqual((app.gimbal.canvas.pan,app.gimbal.canvas.tilt),(22,-35))
            self.assertIn('实际编码器',app.gimbal.badge.cget('text'))
            self.assertTrue(worker.tasks.empty())
            app.gimbal.toggle_demo()
            self.assertFalse(app.gimbal.state.demo)
            worker.events.put(('connection',None));app.update()
            app.gimbal.toggle_demo();app.gimbal.refresh()
            self.assertIn('非设备反馈',app.gimbal.badge.cget('text'))
            self.assertTrue(worker.tasks.empty())
        finally:root.destroy()

    def test_duplicate_fpga_snapshot_does_not_refresh_pose(self):
        class Link:
            sequence=1
            tick=200
            def diagnostic(self,axis):
                return dict(axis=axis,sequence=self.sequence,fpga_tick=self.tick,disabled=False,
                            angle_deg=HOME[axis],feedback_rpm=0,current_a=.1)
        worker=demo_gui.Worker.__new__(demo_gui.Worker)
        worker.link=Link();worker.pose_poll_at=0;worker.pose_sequences=[None,None]
        worker.record=None;worker.events=queue.Queue();worker.log=io.StringIO()
        worker.poll_tracking_poses()
        self.assertEqual(worker.events.qsize(),2)
        worker.pose_poll_at=0;worker.poll_tracking_poses()
        self.assertEqual(worker.events.qsize(),2)
        worker.link.tick+=1
        worker.pose_poll_at=0;worker.poll_tracking_poses()
        self.assertEqual(worker.events.qsize(),2)
        worker.link.sequence+=1
        worker.pose_poll_at=0;worker.poll_tracking_poses()
        self.assertEqual(worker.events.qsize(),4)
        self.assertIn('received_at',worker.events.get_nowait()[1])


if __name__=='__main__':unittest.main(verbosity=2)
