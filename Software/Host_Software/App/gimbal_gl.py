"""Windows OpenGL viewport embedded in Tk, using only the bundled Python runtime.

This is application rendering code. It never opens a serial port or sends controls.
"""
from array import array
import ctypes as C
from ctypes import wintypes as W
import math
import sys
import tkinter as tk
from pathlib import Path
import struct
import zlib

from gimbal_mesh import MATERIALS,build_assembly


class PFD(C.Structure):
    _fields_=[('nSize',W.WORD),('nVersion',W.WORD),('dwFlags',W.DWORD),
              *[(name,C.c_ubyte) for name in ('iPixelType','cColorBits','cRedBits','cRedShift',
                'cGreenBits','cGreenShift','cBlueBits','cBlueShift','cAlphaBits','cAlphaShift',
                'cAccumBits','cAccumRedBits','cAccumGreenBits','cAccumBlueBits','cAccumAlphaBits',
                'cDepthBits','cStencilBits','cAuxBuffers','iLayerType','bReserved')],
              ('dwLayerMask',W.DWORD),('dwVisibleMask',W.DWORD),('dwDamageMask',W.DWORD)]


class GLApi:
    def __init__(self):
        self.user=C.WinDLL('user32',use_last_error=True)
        self.gdi=C.WinDLL('gdi32',use_last_error=True)
        self.gl=C.WinDLL('opengl32',use_last_error=True)
        self.glu=C.WinDLL('glu32',use_last_error=True)
        self.kernel=C.WinDLL('kernel32',use_last_error=True)
        def bind(lib,name,result,*args):
            f=getattr(lib,name);f.restype=result;f.argtypes=args;setattr(self,name,f)
        V=C.c_void_p;I=C.c_int;U=C.c_uint;F=C.c_float;D=C.c_double
        bind(self.kernel,'GetModuleHandleW',V,W.LPCWSTR)
        bind(self.user,'CreateWindowExW',V,W.DWORD,W.LPCWSTR,W.LPCWSTR,W.DWORD,I,I,I,I,V,V,V,V)
        bind(self.user,'DefWindowProcW',C.c_ssize_t,V,U,C.c_size_t,C.c_ssize_t)
        bind(self.user,'GetDC',V,V);bind(self.user,'ReleaseDC',I,V,V)
        bind(self.user,'DestroyWindow',W.BOOL,V)
        bind(self.user,'ShowWindow',W.BOOL,V,I)
        bind(self.user,'MoveWindow',W.BOOL,V,I,I,I,I,W.BOOL)
        bind(self.user,'GetClientRect',W.BOOL,V,C.POINTER(W.RECT))
        bind(self.user,'ValidateRect',W.BOOL,V,V)
        bind(self.user,'SetCapture',V,V);bind(self.user,'ReleaseCapture',W.BOOL)
        bind(self.user,'RegisterClassExW',W.ATOM,V)
        bind(self.user,'UnregisterClassW',W.BOOL,W.LPCWSTR,V)
        bind(self.gdi,'ChoosePixelFormat',I,V,C.POINTER(PFD))
        bind(self.gdi,'SetPixelFormat',W.BOOL,V,I,C.POINTER(PFD))
        bind(self.gdi,'SwapBuffers',W.BOOL,V)
        bind(self.gl,'wglCreateContext',V,V);bind(self.gl,'wglDeleteContext',W.BOOL,V)
        bind(self.gl,'wglMakeCurrent',W.BOOL,V,V)
        bind(self.gl,'wglGetProcAddress',V,C.c_char_p)
        bind(self.gl,'glGetString',C.c_char_p,U)
        bind(self.gl,'glGetError',U)
        for name in ('glEnable','glDisable','glMatrixMode','glShadeModel','glBegin','glDepthFunc','glEnableClientState','glCallList','glDeleteLists'):
            if name=='glDeleteLists':bind(self.gl,name,None,U,I)
            else:bind(self.gl,name,None,U)
        for name in ('glLoadIdentity','glPushMatrix','glPopMatrix','glEnd','glFlush','glPopAttrib'):
            bind(self.gl,name,None)
        bind(self.gl,'glPushAttrib',None,U)
        bind(self.gl,'glViewport',None,I,I,I,I)
        bind(self.gl,'glClear',None,U);bind(self.gl,'glClearColor',None,F,F,F,F)
        bind(self.gl,'glColor3f',None,F,F,F);bind(self.gl,'glColor4f',None,F,F,F,F)
        bind(self.gl,'glNormal3f',None,F,F,F);bind(self.gl,'glVertex3f',None,F,F,F)
        bind(self.gl,'glVertex2f',None,F,F)
        bind(self.gl,'glTranslatef',None,F,F,F);bind(self.gl,'glRotatef',None,F,F,F,F)
        bind(self.gl,'glScalef',None,F,F,F)
        bind(self.gl,'glOrtho',None,D,D,D,D,D,D)
        bind(self.gl,'glMaterialfv',None,U,U,C.POINTER(F));bind(self.gl,'glMaterialf',None,U,U,F)
        bind(self.gl,'glLightfv',None,U,U,C.POINTER(F));bind(self.gl,'glLightModelfv',None,U,C.POINTER(F))
        bind(self.gl,'glLightModeli',None,U,I)
        bind(self.gl,'glBlendFunc',None,U,U);bind(self.gl,'glLineWidth',None,F)
        bind(self.gl,'glInterleavedArrays',None,U,I,V);bind(self.gl,'glDrawArrays',None,U,I,I)
        bind(self.gl,'glGenLists',U,I);bind(self.gl,'glNewList',None,U,U);bind(self.gl,'glEndList',None)
        bind(self.gl,'glDepthMask',None,C.c_ubyte)
        bind(self.gl,'glPixelStorei',None,U,I);bind(self.gl,'glReadBuffer',None,U)
        bind(self.gl,'glReadPixels',None,I,I,I,I,U,U,V)
        bind(self.glu,'gluPerspective',None,D,D,D,D)
        bind(self.glu,'gluLookAt',None,D,D,D,D,D,D,D,D,D)


class DetailedGimbalCanvas(tk.Frame):
    """True depth-tested, lit mechanical assembly; orbit controls affect the camera only."""
    def __init__(self,parent,fallback_factory=None,**kwargs):
        super().__init__(parent,bg='#091322',highlightthickness=0,**kwargs)
        self.pan=self.tilt=0.0
        self.azimuth,self.elevation,self.zoom=31.0,20.0,1.0
        self._fallback_factory=fallback_factory;self._fallback=None
        self._ready=False;self._dirty=True;self._closed=False;self._timer=None
        self._drag_origin=None;self._lists=[];self._buffers=[]
        self.hwnd=self.hdc=self.hrc=None;self.api=None
        self.renderer_name='等待三维渲染初始化';self.error=None;self.last_gl_error=0
        self.triangle_count=0;self.part_count=0
        self.samples=0
        self.bind('<Map>',self._map,add='+')
        self.bind('<Configure>',self._resize,add='+')
        self.bind('<Destroy>',self._destroy,add='+')

    def _map(self,event):
        if event.widget is self and not self._ready and not self.error and not self._closed:
            self.after_idle(self._initialize)
        self._dirty=True

    def _initialize(self):
        if self._closed or self._ready or not self.winfo_ismapped():return
        try:
            if sys.platform!='win32':raise OSError('Native OpenGL requires Windows')
            self.api=GLApi();g=self.api
            self._wndproc_type=C.WINFUNCTYPE(C.c_ssize_t,C.c_void_p,C.c_uint,C.c_size_t,C.c_ssize_t)
            self._wndproc=self._wndproc_type(self._message)
            class WC(C.Structure):
                _fields_=[('cbSize',W.UINT),('style',W.UINT),('lpfnWndProc',C.c_void_p),
                          ('cbClsExtra',C.c_int),('cbWndExtra',C.c_int),('hInstance',C.c_void_p),
                          ('hIcon',C.c_void_p),('hCursor',C.c_void_p),('hbrBackground',C.c_void_p),
                          ('lpszMenuName',W.LPCWSTR),('lpszClassName',W.LPCWSTR),('hIconSm',C.c_void_p)]
            self._class_name=f'LingMouGimbal_{id(self)}'
            self._instance=g.GetModuleHandleW(None)
            wc=WC();wc.cbSize=C.sizeof(WC);wc.style=0x23
            wc.lpfnWndProc=C.cast(self._wndproc,C.c_void_p).value
            wc.hInstance=self._instance;wc.lpszClassName=self._class_name
            if not g.RegisterClassExW(C.byref(wc)):raise C.WinError(C.get_last_error())
            self.hwnd=g.CreateWindowExW(0,self._class_name,'',0x44000000,0,0,
                        max(1,self.winfo_width()),max(1,self.winfo_height()),self.winfo_id(),None,self._instance,None)
            if not self.hwnd:raise C.WinError(C.get_last_error())
            self.hdc=g.GetDC(self.hwnd)
            pfd=PFD();pfd.nSize=C.sizeof(PFD);pfd.nVersion=1;pfd.dwFlags=0x25
            pfd.cColorBits=32;pfd.cDepthBits=24;pfd.cStencilBits=8
            fmt=g.ChoosePixelFormat(self.hdc,C.byref(pfd))
            if not fmt or not g.SetPixelFormat(self.hdc,fmt,C.byref(pfd)):raise C.WinError(C.get_last_error())
            self.hrc=g.wglCreateContext(self.hdc)
            if not self.hrc or not g.wglMakeCurrent(self.hdc,self.hrc):raise C.WinError(C.get_last_error())
            self._enable_multisampling(pfd)
            g.ShowWindow(self.hwnd,5)
            self.renderer_name=(g.glGetString(0x1F01) or b'OpenGL').decode('ascii','replace')
            self._compile()
            self._ready=True
            self._render_now()
            self._tick()
        except Exception as exc:
            self.error=str(exc)
            self._release_native()
            if self._fallback_factory:
                self._fallback=self._fallback_factory(self)
                self._fallback.pack(fill='both',expand=True)
                self.renderer_name='兼容渲染'
                self.render()
            else:
                tk.Label(self,text='三维渲染初始化失败\n'+self.error,fg='#c3cfde',bg='#091322').pack(expand=True)

    def _enable_multisampling(self,pfd):
        """Bootstrap WGL, then create a fresh window with a four-sample pixel format."""
        g=self.api;address=g.wglGetProcAddress(b'wglChoosePixelFormatARB')
        if not address or address in (1,2,3,C.c_void_p(-1).value):return
        choose=C.WINFUNCTYPE(W.BOOL,C.c_void_p,C.POINTER(C.c_int),C.POINTER(C.c_float),
                            C.c_uint,C.POINTER(C.c_int),C.POINTER(C.c_uint))(address)
        attrs=(C.c_int*23)(0x2001,1,0x2010,1,0x2011,1,0x2013,0x202b,0x2014,32,
                           0x2022,24,0x2023,8,0x2041,1,0x2042,4,0,0,0,0,0)
        fmt=C.c_int();count=C.c_uint()
        if not choose(self.hdc,attrs,None,1,C.byref(fmt),C.byref(count)) or not count.value:return
        old_window,old_dc,old_rc=self.hwnd,self.hdc,self.hrc
        new_window=g.CreateWindowExW(0,self._class_name,'',0x44000000,0,0,
                    max(1,self.winfo_width()),max(1,self.winfo_height()),self.winfo_id(),None,self._instance,None)
        if not new_window:return
        new_dc=g.GetDC(new_window);new_rc=None
        if g.SetPixelFormat(new_dc,fmt.value,C.byref(pfd)):
            new_rc=g.wglCreateContext(new_dc)
        if not new_rc or not g.wglMakeCurrent(new_dc,new_rc):
            if new_rc:g.wglDeleteContext(new_rc)
            g.ReleaseDC(new_window,new_dc);g.DestroyWindow(new_window)
            g.wglMakeCurrent(old_dc,old_rc);return
        self.hwnd,self.hdc,self.hrc=new_window,new_dc,new_rc
        self.samples=4
        g.wglDeleteContext(old_rc);g.ReleaseDC(old_window,old_dc);g.DestroyWindow(old_window)

    def _compile(self):
        g=self.api;assembly=build_assembly()
        self.part_count=len(assembly.parts)
        self.triangle_count=sum(len(p.vertices)//18 for p in assembly.parts)
        self._materials={}
        for key,(color,spec,shine) in MATERIALS.items():
            self._materials[key]=((C.c_float*4)(*color,1),(C.c_float*4)(spec,spec,spec,1),shine)
        for group in ('base','pan','body'):
            display=g.glGenLists(1)
            if not display:raise RuntimeError('OpenGL display list allocation failed')
            self._lists.append(display)
            g.glNewList(display,0x1300)
            for material in MATERIALS:
                data=array('f')
                for part in assembly.parts:
                    if part.group==group and part.material==material:data.extend(part.vertices)
                if not data:continue
                buffer=(C.c_float*len(data)).from_buffer(data)
                self._buffers.append((data,buffer))
                diffuse,specular,shine=self._materials[material]
                g.glMaterialfv(0x0408,0x1602,diffuse)
                g.glMaterialfv(0x0408,0x1202,specular)
                g.glMaterialf(0x0408,0x1601,shine)
                g.glInterleavedArrays(0x2A25,0,C.cast(buffer,C.c_void_p))
                g.glDrawArrays(0x0004,0,len(data)//6)
            g.glEndList()
        error=g.glGetError()
        if error:raise RuntimeError(f'OpenGL mesh compilation error 0x{error:04x}')

    @staticmethod
    def _signed_word(value):return C.c_short(value&65535).value

    def _message(self,hwnd,msg,wparam,lparam):
        try:
            if msg==0x000f:  # WM_PAINT: draw from the Tk timer, not inside a Win32 callback.
                self._dirty=True;self.api.ValidateRect(hwnd,None);return 0
            if msg==0x0014:return 1
            if msg==0x0201:
                self._drag_origin=(self._signed_word(lparam),self._signed_word(lparam>>16),self.azimuth,self.elevation)
                self.api.SetCapture(hwnd);return 0
            if msg==0x0202:
                self._drag_origin=None;self.api.ReleaseCapture();return 0
            if msg==0x0200 and self._drag_origin and wparam&1:
                x,y,az,el=self._drag_origin
                self.azimuth=az+(self._signed_word(lparam)-x)*.38
                self.elevation=max(-8,min(70,el+(self._signed_word(lparam>>16)-y)*.25))
                self._dirty=True;return 0
            if msg==0x020a:
                self.zoom=max(.65,min(1.6,self.zoom*(1.08 if self._signed_word(wparam>>16)>0 else 1/1.08)))
                self._dirty=True;return 0
        except Exception:
            pass
        return self.api.DefWindowProcW(hwnd,msg,wparam,lparam) if self.api else 0

    def _resize(self,event):
        if event.widget is self and self.hwnd:
            self.api.MoveWindow(self.hwnd,0,0,max(1,event.width),max(1,event.height),True)
        self._dirty=True

    def _tick(self):
        if self._closed:return
        if self._dirty and self.winfo_ismapped():self._render_now()
        self._timer=self.after(33,self._tick)

    def render(self):
        self._dirty=True
        if self._fallback:
            self._fallback.pan,self._fallback.tilt=self.pan,self.tilt
            self._fallback.render()

    def reset_camera(self):
        self.azimuth,self.elevation,self.zoom=31,20,1
        if self._fallback:self._fallback.reset_camera()
        self.render()

    def _background(self):
        g=self.api
        g.glDisable(0x0b50);g.glDisable(0x0b71)
        g.glMatrixMode(0x1701);g.glLoadIdentity();g.glOrtho(-1,1,-1,1,-1,1)
        g.glMatrixMode(0x1700);g.glLoadIdentity()
        g.glBegin(0x0007)
        for color,x,y in (((.035,.065,.11),-1,1),((.035,.065,.11),1,1),
                          ((.10,.15,.21),1,-1),((.10,.15,.21),-1,-1)):
            g.glColor3f(*color);g.glVertex2f(x,y)
        g.glEnd()

    def _floor(self):
        g=self.api
        g.glDisable(0x0b50);g.glEnable(0x0be2);g.glBlendFunc(0x0302,0x0303)
        g.glDepthMask(0)
        # A broad contact shadow gives the base weight without obscuring the mesh.
        for radius,alpha in ((143,.03),(123,.035),(104,.04),(85,.055),(64,.09)):
            g.glColor4f(0,0,0,alpha);g.glBegin(0x0006);g.glVertex3f(0,0,-1)
            for i in range(81):
                a=i*math.pi/40;g.glVertex3f(radius*math.cos(a),radius*.75*math.sin(a),-1)
            g.glEnd()
        g.glColor4f(.27,.43,.55,.18);g.glLineWidth(1)
        g.glBegin(1)
        for i in range(-240,241,30):
            for p in ((i,-210,-1.8),(i,210,-1.8),(-240,i,-1.8),(240,i,-1.8)):g.glVertex3f(*p)
        g.glEnd()
        g.glColor4f(.25,.55,.61,.3);g.glBegin(2)
        for i in range(120):g.glVertex3f(144*math.cos(i*math.pi/60),144*math.sin(i*math.pi/60),-1)
        g.glEnd();g.glDepthMask(1);g.glDisable(0x0be2)

    def _render_now(self,swap=True):
        if not self._ready:return
        g=self.api
        if not g.wglMakeCurrent(self.hdc,self.hrc):return
        rect=W.RECT();g.GetClientRect(self.hwnd,C.byref(rect))
        w,h=max(1,rect.right),max(1,rect.bottom)
        g.glViewport(0,0,w,h)
        g.glClearColor(.035,.065,.11,1);g.glClear(0x4000|0x0100|0x0400)
        g.glShadeModel(0x1d01)
        self._background()
        g.glMatrixMode(0x1701);g.glLoadIdentity()
        g.gluPerspective(32,w/h,10,2500)
        # Match the front/side presentation of the supplied mechanical reference.
        g.glScalef(-1,1,1)
        g.glMatrixMode(0x1700);g.glLoadIdentity()
        az,el=math.radians(self.azimuth),math.radians(self.elevation)
        distance=max(745,745/max(.65,w/h))/self.zoom
        g.gluLookAt(distance*math.sin(az)*math.cos(el)-10,distance*math.cos(az)*math.cos(el),
                    174+distance*math.sin(el),-10,0,174,0,0,1)
        g.glEnable(0x0b71);g.glDepthFunc(0x0203)
        if self.samples:g.glEnable(0x809d)
        self._floor()
        g.glEnable(0x0b50);g.glEnable(0x0ba1)
        g.glLightModeli(0x0b51,1)
        g.glLightModelfv(0x0b53,(C.c_float*4)(.25,.27,.31,1))
        lights=(((.98,.95,.89,1),(-360,420,800,1)),
                ((.34,.42,.54,1),(500,200,340,1)),
                ((.33,.43,.50,1),(80,-450,540,1)))
        for i,(color,position) in enumerate(lights):
            g.glEnable(0x4000+i)
            g.glLightfv(0x4000+i,0x1201,(C.c_float*4)(*color))
            g.glLightfv(0x4000+i,0x1202,(C.c_float*4)(*color))
            g.glLightfv(0x4000+i,0x1203,(C.c_float*4)(*position))
        g.glCallList(self._lists[0])
        g.glPushMatrix();g.glRotatef(self.pan,0,0,1)
        g.glCallList(self._lists[1])
        g.glTranslatef(0,0,235);g.glRotatef(self.tilt,1,0,0);g.glTranslatef(0,0,-235)
        g.glCallList(self._lists[2]);g.glPopMatrix()
        g.glDisable(0x0b50)
        self.last_gl_error=g.glGetError()
        g.glFlush()
        if swap:g.SwapBuffers(self.hdc)
        self._dirty=False

    def _release_native(self):
        g=self.api
        if not g:return
        if self.hrc:
            if g.wglMakeCurrent(self.hdc,self.hrc):
                for display in self._lists:g.glDeleteLists(display,1)
            g.wglMakeCurrent(None,None);g.wglDeleteContext(self.hrc);self.hrc=None
        if self.hdc and self.hwnd:g.ReleaseDC(self.hwnd,self.hdc);self.hdc=None
        if self.hwnd:g.DestroyWindow(self.hwnd);self.hwnd=None
        if getattr(self,'_class_name',None):g.UnregisterClassW(self._class_name,self._instance)

    def export_frame(self,path):
        """Export the actual renderer framebuffer for offline visual QA."""
        if not self._ready:raise RuntimeError(self.error or 'Renderer has not initialized')
        self._render_now(swap=False)
        g=self.api;rect=W.RECT();g.GetClientRect(self.hwnd,C.byref(rect))
        w,h=rect.right,rect.bottom
        pixels=(C.c_ubyte*(w*h*3))()
        g.glPixelStorei(0x0d05,1);g.glReadBuffer(0x0405)
        g.glReadPixels(0,0,w,h,0x1907,0x1401,C.cast(pixels,C.c_void_p))
        raw=bytes(pixels)
        scan=b''.join(b'\0'+raw[y*w*3:(y+1)*w*3] for y in range(h-1,-1,-1))
        def chunk(kind,data):return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
        png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(scan,6))+chunk(b'IEND',b'')
        Path(path).write_bytes(png)
        g.SwapBuffers(self.hdc)
        return w,h

    def _destroy(self,event):
        if event.widget is not self or self._closed:return
        self._closed=True
        if self._timer:
            try:self.after_cancel(self._timer)
            except tk.TclError:pass
        self._release_native()
