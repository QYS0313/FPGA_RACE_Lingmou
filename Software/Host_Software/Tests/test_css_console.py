import json
import io
import hashlib
import re
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request,urlopen
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'App'))
from desktop_server import ConsoleService,ConsoleHTTPServer
from demo_gui import Worker
from demo_link import ManualMotion, HOME

class FakeWorker:
    def __init__(self):self.tasks=queue.Queue();self.events=queue.Queue();self.requested=(0,0);self.ui_tick=0

class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.worker=FakeWorker();self.service=ConsoleService(worker=self.worker)
    def tearDown(self):self.service.close()
    def test_preview_never_creates_a_uart_owner(self):
        s=ConsoleService(preview=True,demo=True)
        try:
            self.assertIsNone(s.worker);self.assertTrue(s.snapshot()['demo'])
            with self.assertRaises(ValueError):s.command('connect','COM5')
        finally:s.close()
    def test_actual_encoder_sample_and_age_are_preserved(self):
        self.worker.events.put(('connection',0xD2))
        row=dict(axis=0,angle_deg=143.2,speed_rpm=.7,current_a=.2,enabled=True,received_at=1)
        self.worker.events.put(('pose',row));state=self.service.snapshot()
        self.assertEqual(state['poses'][0]['angle_deg'],143.2)
        self.assertGreater(state['poses'][0]['age_s'],1.5)
        self.assertIsNone(state['poses'][1])
    def test_stop_discards_queued_motion_and_releases_jog(self):
        self.worker.events.put(('connection',0xD2));self.service.snapshot()
        self.worker.tasks.put(('track',None));self.worker.requested=(1,1)
        self.service.command('stop')
        self.assertEqual(self.worker.requested,(0,0));self.assertEqual(self.worker.tasks.get_nowait(),('stop',None));self.assertTrue(self.worker.tasks.empty())
        self.worker.tasks.put(('track',None));self.worker.requested=(1,1)
        self.service.release_client()
        self.assertEqual(self.worker.requested,(0,0));self.assertEqual(self.worker.tasks.get_nowait(),('disconnect',None));self.assertTrue(self.worker.tasks.empty())
    def test_demo_is_rejected_while_connected(self):
        self.worker.events.put(('connection',0xD2));self.service.snapshot()
        with self.assertRaises(ValueError):self.service.set_demo(True)
    def test_video_mode_follows_ack_and_readback_and_clears_on_reconnect(self):
        self.worker.events.put(('connection',0xD1))
        self.worker.events.put(('video_options',dict(mode=1)))
        self.assertEqual(self.service.snapshot()['video_mode'],1)
        self.worker.events.put(('video_mode',4))
        state=self.service.snapshot()
        self.assertEqual(state['video_mode'],4)
        self.assertEqual(state['video_options']['mode'],1)  # Older full readback must not win.
        self.worker.events.put(('video_options',dict(mode=2)))
        self.assertEqual(self.service.snapshot()['video_mode'],2)
        self.worker.events.put(('connection',None))
        state=self.service.snapshot()
        self.assertIsNone(state['video_mode']);self.assertIsNone(state['video_options'])
    def test_tracking_uses_the_current_local_calibration(self):
        self.worker.events.put(('connection',0xD2));self.service.snapshot()
        self.service.command('track',dict(brightness=160,chroma=40,radius=10,laser=True))
        action,value=self.worker.tasks.get_nowait()
        self.assertEqual(action,'track');self.assertEqual(Path(value[0]),ROOT/'Config/calibration.json');self.assertEqual(value[1:],(160,40,10,True))
    def test_manual_motion_only_in_video_workspace(self):
        self.worker.events.put(('connection',0xD1));self.service.snapshot()
        self.service.heartbeat(dict(view='video',requested=[2,-1]));self.assertEqual(self.worker.requested,(2,-1))
        self.service.heartbeat(dict(view='tracking',requested=[2,-1]));self.assertEqual(self.worker.requested,(0,0))
        with self.assertRaises(ValueError):self.service.heartbeat(dict(view='video',requested=[float('nan'),0]))
    def test_switch_requires_flash_configuration_and_valid_destination(self):
        self.worker.events.put(('connection',0xD2));self.service.snapshot()
        with self.assertRaises(ValueError):self.service.command('switch',0)
        with self.assertRaises(ValueError):self.service.command('switch',3,True)
        self.service.command('switch',0,True);self.assertEqual(self.worker.tasks.get_nowait(),('switch',0))
    def test_arbitrary_serial_commands_are_not_exposed(self):
        self.worker.events.put(('connection',0xD1));self.service.snapshot()
        with self.assertRaises(ValueError):self.service.command('video',[0x87,0xa5])
        with self.assertRaises(ValueError):self.service.command('anything',None)
    def test_model_calibration_writes_only_display_configuration(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'Config').mkdir();s=ConsoleService(preview=True,root=root)
            try:
                s.save_model_config(dict(reference_deg=[120,180],direction=[-1,1]))
                self.assertEqual(json.loads((root/'Config/gimbal_model.json').read_text())['direction'],[-1,1])
                with self.assertRaises(ValueError):s.save_model_config(dict(reference_deg=[float('inf'),180],direction=[1,1]))
            finally:s.close()

class VideoWorkerTests(unittest.TestCase):
    def setUp(self):
        self.worker=Worker.__new__(Worker)
        self.worker.events=queue.Queue();self.worker.log=io.StringIO()
    def test_video_mode_is_decoded_from_device_response(self):
        class Link:
            has_video_motion=True
            def need(self,image):pass
            def command(self,cmd,data):return 0x31  # Device confirms Sobel, even if 4 was requested.
        self.worker.link=Link()
        self.worker.handle('video',(0x11,4))
        self.assertEqual(self.worker.events.get_nowait(),('video_mode',3))
        self.worker.events=queue.Queue()
        with patch.object(self.worker.link,'command',side_effect=IOError('timeout')):
            with self.assertRaises(IOError):self.worker.handle('video',(0x11,2))
        self.assertTrue(self.worker.events.empty())
    def test_manual_motion_response_reaches_model_state_while_moving(self):
        class Link:
            image=0xD1
            def command(self,cmd,data=0):return 3
            def motor(self,axis):return dict(axis=axis,enabled=True,angle_deg=HOME[axis],speed_rpm=0,current_a=.1)
            def speed(self,axis,rpm):return dict(axis=axis,enabled=True,angle_deg=HOME[axis]+(12 if rpm>0 else -12),speed_rpm=rpm,current_a=.2)
        for key,request,axis,sign in [('w',[0,2],1,1),('a',[2,0],0,1),('s',[0,-2],1,-1),('d',[-2,0],0,-1)]:
            with self.subTest(key=key):
                self.worker.events=queue.Queue();self.worker.link=Link();self.worker.shapes=None
                self.worker.motion=ManualMotion(self.worker.link,self.worker.emit)
                self.worker.poll_at=float('inf')
                fake=FakeWorker();fake.events=self.worker.events
                service=ConsoleService(worker=fake)
                try:
                    fake.events.put(('connection',0xD1));service.snapshot()
                    service.heartbeat(dict(view='video',requested=request))
                    self.worker.requested=fake.requested;self.worker.ui_tick=fake.ui_tick
                    self.worker.poll()
                    self.assertIn(axis,self.worker.motion.moving)
                    state=service.snapshot();row=state['poses'][axis]
                    self.assertEqual(row['angle_deg'],HOME[axis]+12*sign)
                    self.assertLess(row['age_s'],1.5)
                    self.assertEqual(state['model_config']['direction'],[1,-1])
                    # The display reverses pitch only; keyboard motor requests retain their signs.
                    self.assertEqual(fake.requested,tuple(request))
                    self.assertEqual(state['model_config']['direction'][axis]*(row['angle_deg']-HOME[axis]),12*sign*(1 if axis==0 else -1))
                finally:service.close()

class HttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.service=ConsoleService(preview=True);cls.server=ConsoleHTTPServer(cls.service)
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True);cls.thread.start()
    @classmethod
    def tearDownClass(cls):cls.service.close();cls.server.shutdown();cls.server.server_close()
    def test_local_page_serves_and_geometry_manifest_matches(self):
        with urlopen(self.server.url) as r:self.assertIn(b'console.css',r.read())
        with urlopen(self.server.url+'assets/assembly_manifest.json') as r:data=json.load(r)
        self.assertEqual(data['mesh_count'],96);self.assertEqual(data['triangle_count'],170130)
        ids=data['joints']['base']+data['joints']['pan']+data['joints']['tilt']
        self.assertEqual(len(ids),len(set(ids)));self.assertEqual(set(map(str,ids)),set(data['groups']))
    def test_missing_session_and_cross_origin_posts_are_rejected(self):
        with self.assertRaises(HTTPError) as e:urlopen(self.server.origin+'/api/state')
        self.assertEqual(e.exception.code,403)
        e.exception.close()
        request=Request(self.server.url+'api/demo',data=b'{"enabled":true}',headers={'Origin':'http://example.com','Content-Type':'application/json'})
        with self.assertRaises(HTTPError) as e:urlopen(request)
        self.assertEqual(e.exception.code,403)
        e.exception.close()
    def test_state_and_demo_endpoint_never_open_serial(self):
        request=Request(self.server.url+'api/demo',data=b'{"enabled":true}',headers={'Origin':self.server.origin,'Content-Type':'application/json'})
        with urlopen(request) as r:self.assertEqual(r.status,200)
        with urlopen(self.server.url+'api/state') as r:data=json.load(r)
        self.assertTrue(data['demo']);self.assertTrue(data['preview']);self.assertIsNone(self.service.worker)
    def test_path_traversal_cannot_read_configuration(self):
        with self.assertRaises(HTTPError) as e:urlopen(self.server.url+'../Config/calibration.json')
        self.assertEqual(e.exception.code,404)
        e.exception.close()

class SourceModelTests(unittest.TestCase):
    def test_supplied_html_geometry_is_preserved_exactly(self):
        source=(ROOT/'Models/assembly2_original.html').read_bytes()
        manifest=json.loads((ROOT/'Web/assets/assembly_manifest.json').read_text(encoding='utf-8'))
        self.assertEqual(hashlib.sha256(source).hexdigest(),manifest['source_sha256'])
        original=json.loads(re.search(r'const data=(.*?);\s*\nconst scene=',source.decode('utf-8-sig'),re.S)[1])
        imported=json.loads((ROOT/'Web/assets/assembly.json').read_text(encoding='utf-8'))
        self.assertEqual(imported,original)

if __name__=='__main__':unittest.main(verbosity=2)
