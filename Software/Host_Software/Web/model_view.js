import * as THREE from './assets/three.module.js';

const radians=Math.PI/180;
export const angleDelta=(angle,reference)=>((angle-reference+180)%360+360)%360-180;

export class AssemblyView {
  constructor(host){
    this.host=host;this.pan=0;this.tilt=0;this.displayPan=0;this.displayTilt=0;
    this.azimuth=Math.atan2(.88,-1.65);this.elevation=.35;this.zoom=1;this.internal=false;
    this.groups={};this.parts=[];this.lastFrame=performance.now();this.hasFeedback=false;
    this.scene=new THREE.Scene();
    this.renderer=new THREE.WebGLRenderer({alpha:true,antialias:true,preserveDrawingBuffer:true,powerPreference:'high-performance'});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio,1.6));
    this.renderer.outputColorSpace=THREE.SRGBColorSpace;
    this.renderer.toneMapping=THREE.ACESFilmicToneMapping;this.renderer.toneMappingExposure=1.22;
    this.renderer.shadowMap.enabled=true;this.renderer.shadowMap.type=THREE.PCFSoftShadowMap;
    this.host.prepend(this.renderer.domElement);
    this.camera=new THREE.OrthographicCamera(-300,300,190,-190,.1,3000);this.camera.up.set(0,0,1);
    this.target=new THREE.Vector3(-3,0,-1);
    this.modelRoot=new THREE.Group();this.scene.add(this.modelRoot);
    this.scene.add(new THREE.HemisphereLight(0xdde8f3,0x39404c,1.6));
    const key=new THREE.DirectionalLight(0xfff6e7,3.4);key.position.set(-200,-300,470);key.castShadow=true;
    key.shadow.mapSize.set(2048,2048);Object.assign(key.shadow.camera,{left:-220,right:220,top:220,bottom:-220,near:1,far:1000});
    key.shadow.bias=-.00035;key.shadow.normalBias=.35;key.shadow.radius=4;this.scene.add(key);
    const fill=new THREE.DirectionalLight(0xa5c5e4,1.3);fill.position.set(320,-10,120);this.scene.add(fill);
    const rim=new THREE.DirectionalLight(0x87cec9,1.5);rim.position.set(0,300,270);this.scene.add(rim);
    this.createEnvironment();this.createFloor();this.bindOrbit();
    this.observer=new ResizeObserver(()=>this.resize());this.observer.observe(host);this.resize();
    this.frame=requestAnimationFrame(t=>this.animate(t));
  }

  createEnvironment(){
    const canvas=document.createElement('canvas');canvas.width=512;canvas.height=256;
    const c=canvas.getContext('2d');const gradient=c.createLinearGradient(0,0,0,256);
    gradient.addColorStop(0,'#a9b8c9');gradient.addColorStop(.46,'#65717c');gradient.addColorStop(.56,'#26323d');gradient.addColorStop(1,'#46545e');
    c.fillStyle=gradient;c.fillRect(0,0,512,256);
    for(const [x,w,alpha] of [[80,42,.6],[210,70,.75],[410,32,.45]]){
      const g=c.createLinearGradient(x-w,0,x+w,0);g.addColorStop(0,'rgba(255,255,255,0)');g.addColorStop(.5,`rgba(255,255,255,${alpha})`);g.addColorStop(1,'rgba(255,255,255,0)');
      c.fillStyle=g;c.fillRect(x-w,15,w*2,170);
    }
    const texture=new THREE.CanvasTexture(canvas);texture.mapping=THREE.EquirectangularReflectionMapping;
    const generator=new THREE.PMREMGenerator(this.renderer);this.environment=generator.fromEquirectangular(texture);
    this.scene.environment=this.environment.texture;texture.dispose();generator.dispose();
  }

  createFloor(){
    const plane=new THREE.Mesh(new THREE.PlaneGeometry(900,900),new THREE.ShadowMaterial({opacity:.23}));
    plane.position.z=-147;plane.receiveShadow=true;this.scene.add(plane);
    const grid=new THREE.GridHelper(500,20,0x365460,0x253b49);grid.rotation.x=Math.PI/2;grid.position.z=-147.3;
    grid.material.transparent=true;grid.material.opacity=.22;grid.material.depthWrite=false;this.scene.add(grid);
    const circle=new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(Array.from({length:128},(_,i)=>
      new THREE.Vector3(129*Math.cos(i*Math.PI/64)-15,129*Math.sin(i*Math.PI/64),-147.1))),
      new THREE.LineBasicMaterial({color:0x417f86,transparent:true,opacity:.22}));this.scene.add(circle);
  }

  async load(){
    const [data,manifest]=await Promise.all([
      fetch(new URL('./assets/assembly.json',import.meta.url)).then(r=>{if(!r.ok)throw Error('模型文件读取失败');return r.json()}),
      fetch(new URL('./assets/assembly_manifest.json',import.meta.url)).then(r=>r.json())]);
    this.manifest=manifest;
    for(const [id,name] of Object.entries(data.groups)){
      const g=new THREE.Group();g.name=name;this.modelRoot.add(g);this.groups[id]=g;
    }
    this.triangles=0;
    for(const d of data.meshes){
      const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(d.positions,3));
      geometry.setIndex(d.indices);geometry.computeVertexNormals();
      const color=new THREE.Color().setRGB(...d.color.slice(0,3),THREE.LinearSRGBColorSpace);
      if(d.group<=10)color.setRGB(.31,.345,.39);
      const material=new THREE.MeshStandardMaterial({color,roughness:d.group<=10?.42:.64,
        metalness:d.group<=10?.52:.12,envMapIntensity:.68,side:THREE.DoubleSide});
      const mesh=new THREE.Mesh(geometry,material);mesh.name=d.name;mesh.castShadow=true;mesh.receiveShadow=true;
      this.groups[d.group].add(mesh);this.parts.push(mesh);this.triangles+=d.indices.length/3;
      if(d.group<=10){
        const edge=new THREE.LineSegments(new THREE.EdgesGeometry(geometry,36),new THREE.LineBasicMaterial({color:0x182633,transparent:true,opacity:.18}));
        edge.name='cad-outline';this.groups[d.group].add(edge);
      }
    }
    this.panRoot=new THREE.Group();this.panRoot.name='PAN joint';this.panRoot.position.fromArray(manifest.joints.pan_pivot);this.modelRoot.add(this.panRoot);
    this.tiltRoot=new THREE.Group();this.tiltRoot.name='TILT joint';this.tiltRoot.position.fromArray(manifest.joints.tilt_pivot);this.modelRoot.add(this.tiltRoot);
    this.panRoot.attach(this.tiltRoot);
    for(const id of manifest.joints.pan)this.panRoot.attach(this.groups[id]);
    for(const id of manifest.joints.tilt)this.tiltRoot.attach(this.groups[id]);
    this.ready=true;this.applyAngles();this.render();return manifest;
  }

  setAngles(pan,tilt,hasFeedback){
    if(hasFeedback&&!this.hasFeedback){this.displayPan=pan;this.displayTilt=tilt;}
    this.hasFeedback=hasFeedback;this.pan=pan;this.tilt=tilt;
    if(!hasFeedback)this.displayPan=this.displayTilt=this.pan=this.tilt=0;
  }
  applyAngles(){
    if(this.panRoot){this.panRoot.rotation.z=this.displayPan*radians;this.tiltRoot.rotation.x=this.displayTilt*radians;}
  }
  setInternal(active){
    this.internal=active;
    for(const id of [2,9])if(this.groups[id])this.groups[id].visible=!active;
    this.groups[6]?.traverse(o=>{if(o.isMesh){o.material.transparent=active;o.material.opacity=active?.16:1;o.material.depthWrite=!active;o.castShadow=!active;}if(o.isLineSegments)o.visible=!active;});
  }
  reset(front=false){this.azimuth=front?Math.PI:Math.atan2(.88,-1.65);this.elevation=front?.08:.35;this.zoom=1;this.resize();}
  bindOrbit(){
    const canvas=this.renderer.domElement;let drag;
    canvas.addEventListener('pointerdown',e=>{if(e.button!==0)return;drag={x:e.clientX,y:e.clientY,a:this.azimuth,e:this.elevation};canvas.setPointerCapture(e.pointerId);});
    canvas.addEventListener('pointermove',e=>{if(!drag)return;this.azimuth=drag.a-(e.clientX-drag.x)*.006;this.elevation=Math.max(-.05,Math.min(1.3,drag.e+(e.clientY-drag.y)*.004));});
    const release=()=>{drag=null};canvas.addEventListener('pointerup',release);canvas.addEventListener('lostpointercapture',release);
    canvas.addEventListener('wheel',e=>{e.preventDefault();this.zoom=Math.max(.65,Math.min(1.8,this.zoom*Math.exp(-e.deltaY*.001)));this.resize();},{passive:false});
    canvas.addEventListener('webglcontextlost',e=>{e.preventDefault();document.dispatchEvent(new CustomEvent('model-error',{detail:'三维渲染已暂停，请重新打开控制台'}));});
  }
  resize(){
    const w=this.host.clientWidth,h=this.host.clientHeight;if(!w||!h)return;
    this.renderer.setSize(w,h,false);const height=Math.max(380,380/(w/h))/this.zoom;
    this.camera.left=-height*w/h/2;this.camera.right=height*w/h/2;this.camera.top=height/2;this.camera.bottom=-height/2;this.camera.updateProjectionMatrix();
  }
  animate(now){
    const dt=Math.min(.1,(now-this.lastFrame)/1000);this.lastFrame=now;
    const f=1-Math.exp(-dt*12);this.displayPan+=(this.pan-this.displayPan)*f;this.displayTilt+=(this.tilt-this.displayTilt)*f;
    this.applyAngles();if(this.host.offsetParent!==null)this.render();
    this.frame=requestAnimationFrame(t=>this.animate(t));
  }
  render(){
    const r=750*Math.cos(this.elevation);
    this.camera.position.set(this.target.x+Math.sin(this.azimuth)*r,this.target.y+Math.cos(this.azimuth)*r,this.target.z+750*Math.sin(this.elevation));
    this.camera.lookAt(this.target);this.renderer.render(this.scene,this.camera);
    if(this.ready){this.positionPin('pan-pin',this.panRoot,-92,15);this.positionPin('tilt-pin',this.tiltRoot,20,-22);}
  }
  positionPin(id,object,dx,dy){
    const point=object.getWorldPosition(new THREE.Vector3()).project(this.camera),el=document.getElementById(id);
    el.style.left=`${Math.max(18,Math.min(this.host.clientWidth-115,(point.x+1)*this.host.clientWidth/2+dx))}px`;
    el.style.top=`${Math.max(80,Math.min(this.host.clientHeight-55,(1-point.y)*this.host.clientHeight/2+dy))}px`;
    el.classList.add('visible');
  }
}
