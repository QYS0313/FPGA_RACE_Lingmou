"""Parametric mechanical assembly. Model units describe proportions, not measured CAD."""
from dataclasses import dataclass
import math


# Diffuse, specular strength, shininess: anodised alloy, brushed metal and optics.
MATERIALS={
    'shell':((.27,.30,.34),.62,68),
    'cover':((.40,.44,.49),.72,80),
    'panel':((.21,.24,.28),.50,62),
    'edge':((.52,.56,.61),.72,90),
    'bracket':((.35,.39,.44),.62,76),
    'silver':((.65,.71,.77),.92,100),
    'steel':((.39,.45,.52),.85,92),
    'black':((.035,.047,.061),.30,48),
    'rubber':((.026,.033,.043),.10,16),
    'socket':((.012,.018,.025),.08,8),
    'glass':((.025,.18,.25),1.0,120),
    'lens':((.02,.06,.105),.95,110),
    'coating':((.08,.39,.46),.9,115),
    'pan':((.13,.62,.63),.80,80),
    'tilt':((.48,.39,.73),.80,80),
    'label':((.66,.72,.77),.20,30),
}


def normal(a,b,c):
    u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
    n=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
    length=math.sqrt(sum(x*x for x in n))
    return tuple(x/length for x in n) if length>1e-9 else (0,0,1)


def axis_point(center,axis,u,v,t):
    x,y,z=center
    return (x+t,y+u,z+v) if axis=='x' else (x+u,y+t,z+v) if axis=='y' else (x+u,y+v,z+t)


@dataclass
class Part:
    name:str
    group:str
    material:str
    vertices:list  # Per vertex: nx,ny,nz,x,y,z.


class Assembly:
    def __init__(self):self.parts=[]

    def part(self,name,group,material):
        p=Part(name,group,material,[]);self.parts.append(p);return p

    def face(self,part,points,n=None):
        if n is None:n=normal(*points[:3])
        for i in range(1,len(points)-1):
            for p in (points[0],points[i],points[i+1]):part.vertices.extend((*n,*p))

    def bevel(self,name,lo,hi,r,group,material):
        p=self.part(name,group,material)
        center=tuple((a+b)/2 for a,b in zip(lo,hi))
        half=tuple((b-a)/2 for a,b in zip(lo,hi))
        r=min(r,min(half)*.85)
        def face(points,n):
            points=[tuple(v[i]+center[i] for i in range(3)) for v in points]
            if sum(a*b for a,b in zip(normal(*points[:3]),n))<0:points.reverse()
            self.face(p,points,n)
        # Six planes, twelve chamfer strips and eight corner patches.
        for a in range(3):
            b,c=[i for i in range(3) if i!=a]
            for s in (-1,1):
                pts=[]
                for sb,sc in ((-1,-1),(1,-1),(1,1),(-1,1)):
                    q=[0,0,0];q[a]=s*half[a];q[b]=sb*(half[b]-r);q[c]=sc*(half[c]-r);pts.append(q)
                n=[0,0,0];n[a]=s;face(pts,n)
        for a,b,c in ((0,1,2),(0,2,1),(1,2,0)):
            for sa in (-1,1):
                for sb in (-1,1):
                    pts=[]
                    for t,side in ((-1,0),(1,0),(1,1),(-1,1)):
                        q=[0,0,0];q[a]=sa*(half[a]-(r if side else 0));q[b]=sb*(half[b]-(0 if side else r));q[c]=t*(half[c]-r);pts.append(q)
                    n=[0,0,0];n[a]=sa/math.sqrt(2);n[b]=sb/math.sqrt(2);face(pts,n)
        for sx in (-1,1):
            for sy in (-1,1):
                for sz in (-1,1):
                    signs=(sx,sy,sz);pts=[]
                    for axis in range(3):pts.append([signs[i]*(half[i]-(0 if axis==i else r)) for i in range(3)])
                    face(pts,[v/math.sqrt(3) for v in signs])
        return p

    def lathe(self,name,center,profile,axis,group,material,segments=72):
        p=self.part(name,group,material)
        for (t0,r0),(t1,r1) in zip(profile,profile[1:]):
            dt,dr=t1-t0,r1-r0;length=math.hypot(dt,dr)
            if length<1e-9:continue
            for i in range(segments):
                a,b=2*math.pi*i/segments,2*math.pi*(i+1)/segments
                points=[];normals=[]
                for angle,t,r in ((a,t0,r0),(b,t0,r0),(b,t1,r1),(a,t1,r1)):
                    u,v=math.cos(angle),math.sin(angle)
                    points.append(axis_point(center,axis,r*u,r*v,t))
                    normals.append(axis_point((0,0,0),axis,dt/length*u,dt/length*v,-dr/length))
                for j in (0,1,2,0,2,3):p.vertices.extend((*normals[j],*points[j]))
        return p

    def drum(self,name,center,radius,length,axis,group,material,bevel=1.2,segments=72):
        b=min(bevel,length*.3,radius*.2);h=length/2
        return self.lathe(name,center,[(-h,0),(-h,radius-b),(-h+b,radius),(h-b,radius),(h,radius-b),(h,0)],axis,group,material,segments)

    def ring(self,name,center,outer,inner,length,axis,group,material):
        h=length/2;b=min(.65,length/4)
        return self.lathe(name,center,[(-h,inner),(-h,outer-b),(-h+b,outer),(h-b,outer),(h,outer-b),(h,inner),(-h,inner)],axis,group,material)

    def bolt(self,center,axis,group='body',size=2.4):
        self.drum('socket cap screw',center,size,1.7,axis,group,'silver',.35,24)
        nose=axis_point(center,axis,0,0,.89)
        self.drum('hex socket recess',nose,size*.48,.07,axis,group,'socket',.01,6)

    def tube(self,name,points,radius,group,material,segments=10):
        part=self.part(name,group,material)
        for i in range(len(points)-1):
            a,b=points[i:i+2];d=[b[j]-a[j] for j in range(3)]
            length=math.sqrt(sum(v*v for v in d)) or 1;d=[v/length for v in d]
            seed=(0,0,1) if abs(d[2])<.9 else (0,1,0)
            u=normal((0,0,0),d,seed);v=normal((0,0,0),d,u)
            for k in range(segments):
                angles=(2*math.pi*k/segments,2*math.pi*(k+1)/segments)
                ns=[tuple(math.cos(t)*u[j]+math.sin(t)*v[j] for j in range(3)) for t in angles]
                ps=[tuple(p[j]+radius*n[j] for j in range(3)) for p,n in ((a,ns[0]),(a,ns[1]),(b,ns[1]),(b,ns[0]))]
                for index in (0,1,2,0,2,3):part.vertices.extend((*ns[(0,1,1,0)[index]],*ps[index]))


def build_assembly():
    m=Assembly()
    # Machined cruciform stand with raised mounting boss and four anti-slip feet.
    m.bevel('front to back stand',(-24,-102,2),(24,102,12),4,'base','bracket')
    m.bevel('left to right stand',(-112,-23,2),(112,23,12),4,'base','bracket')
    for x,y in ((-96,0),(96,0),(0,-87),(0,87)):
        m.drum('rubber foot',(x,y,2),12,4,'z','base','rubber',.7,40)
        m.bolt((x,y,12.8),'z','base',3)
    m.drum('base pedestal',(0,0,30),36,40,'z','base','shell',3)
    m.ring('lower pedestal lip',(0,0,13),38,20,5,'z','base','edge')
    m.ring('upper pedestal lip',(0,0,48),39,25,6,'z','base','silver')
    m.ring('pan bearing seal',(0,0,52),38,25,2,'z','pan','black')
    m.ring('pan identification band',(0,0,54),38.3,28,1.5,'z','pan','pan')
    m.bevel('pan drive enclosure',(-42,-38,55),(43,38,101),4,'pan','cover')
    m.bevel('pan drive faceplate',(-39,38,59),(40,40,98),2,'pan','bracket')
    for x in (-31,31):
        for y in (-28,28):m.bolt((x,y,101.8),'z','pan',2.9)
    for x in (-31,31):
        for z in (68,90):m.bolt((x,40.5,z),'y','pan',2)
    # L arm uses a main casting, edge flanges and a separate bolted bearing carrier.
    m.bevel('horizontal cantilever',(35,-27,65),(126,27,96),3,'pan','bracket')
    m.bevel('lower cantilever edge',(42,-30,64),(130,30,72),1.5,'pan','edge')
    m.bevel('upright casting',(99,-28,91),(127,28,247),3.5,'pan','bracket')
    m.bevel('upright front flange',(97,25,97),(131,32,242),1.6,'pan','cover')
    m.bevel('upright back flange',(97,-32,97),(131,-25,242),1.6,'pan','cover')
    m.bevel('upright inset face',(128,-19,109),(130,19,226),1,'pan','panel')
    for z in (117,148,179,210):
        m.bevel('upright stiffening rib',(130,-22,z),(132,22,z+3),.7,'pan','edge')
    m.bevel('upper bearing carrier',(95,-35,228),(134,35,252),3,'pan','cover')
    for y in (-23,23):
        for z in (233,246):m.bolt((134.8,y,z),'x','pan',2.4)
    for x in (106,121):
        for y in (-19,19):m.bolt((x,y,97),'z','pan',2.5)
    # Detailed pitch motor: endbell, stator, cooling grooves, bearing and spindle.
    m.drum('pitch stator',(83,0,235),29,27,'x','pan','silver',2)
    m.ring('motor rear endbell',(98,0,235),30,6,4,'x','pan','steel')
    m.ring('motor front endbell',(68,0,235),30,9,4,'x','pan','edge')
    for x in (73,77,81,85,89,93):m.ring('stator cooling groove',(x,0,235),29.4,28,1,'x','pan','steel')
    m.drum('pitch hub',(62,0,235),24,9,'x','body','black',1)
    m.ring('pitch marker band',(65.5,0,235),25,21,1.8,'x','body','tilt')
    for i in range(6):
        a=i*math.pi/3
        m.bolt((101,21*math.cos(a),235+21*math.sin(a)),'x','pan',2.1)
    # Payload: gasketed top cover and layered sheet-metal front and side panels.
    m.bevel('payload main housing',(-113,-65,174),(59,70,333),5,'body','shell')
    m.bevel('top cover gasket',(-115,-67,330),(61,72,334),1.4,'body','black')
    m.bevel('folded top cover',(-117,-70,333),(64,75,343),3,'body','cover')
    m.bevel('front cover flange',(-114,70,178),(59,76,326),2.5,'body','cover')
    m.bevel('front lower service panel',(-104,76,187),(49,78,251),1.3,'body','panel')
    m.bevel('left side panel',(-115,-55,186),(-113,60,319),1,'body','panel')
    m.bevel('right side panel',(59,-55,185),(61,56,317),1,'body','panel')
    m.bevel('lower rolled edge',(-110,-64,173),(57,70,182),1.8,'body','edge')
    # Optical mounting plate, retaining bezel, inner barrel and recessed glass.
    m.ring('front instrument mounting plate',(-13,79,267),34,15,3,'y','body','steel')
    m.ring('outer lens retaining bezel',(-13,82,267),24,15,5,'y','body','black')
    m.ring('bezel silver lip',(-13,85,267),22,17,1.4,'y','body','silver')
    m.ring('lens inner barrel',(-13,86,267),18,12,2,'y','body','panel')
    m.drum('optical glass',(-13,86.7,267),11.8,.4,'y','body','glass',.1)
    m.ring('inner coating reflection',(-13,87,267),10.8,8.7,.1,'y','body','coating')
    m.drum('deep lens',(-13,87.2,267),8.4,.15,'y','body','lens',.02)
    for i in range(4):
        a=math.pi/4+i*math.pi/2;m.bolt((-13+28*math.cos(a),81,267+28*math.sin(a)),'y','body',1.9)
    # Secondary optical slot and metal sun hood match the asymmetric reference.
    m.bevel('secondary aperture body',(-110,75,286),(-61,87,307),1.8,'body','black')
    m.bevel('secondary aperture glass',(-105,87,290),(-66,88,302),.8,'body','glass')
    m.bevel('aperture divider',(-105,88.3,296),(-66,89,298),.3,'body','silver')
    m.bevel('secondary sun hood',(-115,73,306),(-57,96,314),1.4,'body','cover')
    # Recessed vents, engraved panel border, flange screws and service connectors.
    for z in range(201,249,8):
        m.bevel('front louvre recess',(-99,78.1,z),(-68,78.7,z+2.4),.5,'body','socket')
        m.bevel('front louvre edge',(-98,78.8,z+2.4),(-68,79.4,z+3),.2,'body','edge')
    for y in range(-44,40,10):
        m.bevel('right side cooling slot',(61.1,y,286),(61.7,y+3,316),.5,'body','socket')
    for x in (-105,50):
        for z in (189,256,317):m.bolt((x,77.5,z),'y','body',2)
    for x in (-101,46):
        for y in (-54,59):m.bolt((x,y,343.8),'z','body',2.5)
    for y in (-52,52):
        for z in (190,315):m.bolt((62,y,z),'x','body',2.1)
    m.bevel('instrument name plate',(-47,78.5,205),(29,79.2,220),1,'body','black')
    for x in range(-40,22,8):m.bevel('etched plate markings',(x,79.4,210),(x+3,79.6,216),.15,'body','label')
    m.drum('rear circular connector',(-65,-68,211),10,7,'y','body','black',1)
    m.ring('connector lock collar',(-65,-72,211),11,7,3,'y','body','silver')
    # Cable follows the rotating support, routed clear of the two bearings.
    m.tube('motor cable harness',[(118,-31,109),(121,-37,131),(121,-37,174),(115,-36,208),(101,-29,225)],2.6,'pan','rubber')
    for z in (140,190):m.bevel('cable retaining clip',(116,-39,z),(125,-34,z+7),1,'pan','black')
    return m
