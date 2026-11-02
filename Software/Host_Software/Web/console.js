import {AssemblyView,angleDelta} from './model_view.js';

const $=id=>document.getElementById(id);
let state=null,view='tracking',lastLog=0,lastSource='',toastTimer,model,optionsSeen='';
const keys=new Set(),history=[[],[]],sampleIds=[null,null];
const errors=[];window.addEventListener('error',e=>errors.push(e.message));window.addEventListener('unhandledrejection',e=>errors.push(String(e.reason)));
function text(id,value){$(id).textContent=value;}
function toast(message){text('toast',message);$('toast').classList.add('visible');clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').classList.remove('visible'),4500);}
async function api(path,data){
  const response=await fetch('api/'+path,data===undefined?{cache:'no-store'}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
  const result=await response.json();if(!response.ok)throw Error(result.error||'请求未完成');return result;
}
async function send(action,value,extra={}){
  try{await api('command',{action,value,...extra});}catch(e){toast(e.message);}
}
const number=id=>{const input=$(id);if(!input.checkValidity())throw Error('请检查参数范围：'+id);const n=Number(input.value);if(!Number.isFinite(n))throw Error('请输入有效数值');return n;};
function catchInput(fn){return()=>{try{const result=fn();if(result?.catch)result.catch(e=>toast(e.message));}catch(e){toast(e.message);}};}
function showView(name){
  view=name;keys.clear();$('tracking-view').classList.toggle('hidden',name!=='tracking');$('video-view').classList.toggle('hidden',name!=='video');
  // Keep one renderer and one encoder-driven pose across both workspaces.
  $(name==='tracking'?'tracking-model-host':'video-model-host').prepend($('gimbal-model'));
  document.querySelectorAll('[data-view]').forEach(el=>el.classList.toggle('active',el.dataset.view===name));
  text('breadcrumb-view',name==='tracking'?'光点追踪':'视频算法');
  $('page-title').replaceChildren(document.createTextNode(name==='tracking'?'光点追踪工作台':'视频算法工作台'),Object.assign(document.createElement('span'),{className:'title-dot',textContent:'.'}));
  text('page-subtitle',name==='tracking'?'双轴运动可视化 · 视觉目标追踪':'视频算法切换 · 双轴姿态映射 · 图像增强与选区');
  requestAnimationFrame(()=>{model?.resize();drawChart();});
}
document.querySelectorAll('[data-view]').forEach(el=>el.onclick=()=>showView(el.dataset.view));
document.querySelectorAll('[data-action]').forEach(el=>el.onclick=()=>{keys.clear();send(el.dataset.action);});
$('connect').onclick=()=>send(state?.image?'disconnect':'connect',$('port').value);
$('refresh-ports').onclick=()=>poll(true);
for(const id of ['brightness','chroma']){
  $(id).oninput=()=>{text(id+'-output',$(id).value);$(id).style.setProperty('--fill',`${Number($(id).value)/Number($(id).max)*100}%`);};$(id).oninput();
}
$('start-track').onclick=catchInput(()=>send('track',{brightness:number('brightness'),chroma:number('chroma'),radius:number('radius'),laser:$('laser-on-start').checked}));
$('detect-only').onclick=catchInput(()=>send('detect',[number('brightness'),number('chroma')]));
$('laser-on').onclick=()=>send('laser',true);$('laser-off').onclick=()=>send('laser',false);$('record').onclick=()=>send('record');
$('choose-calibration').onclick=()=>$('calibration-upload').click();
$('calibration-upload').onchange=catchInput(async()=>{const file=$('calibration-upload').files[0];if(!file)return;if(file.size>120000)throw Error('标定文件过大');await api('calibration',JSON.parse(await file.text()));toast('视觉标定已加载');});
const shapeValue=()=>({color:$('shape-color').value,seconds:number('shape-seconds')});
$('shape-detect').onclick=catchInput(()=>send('shape_detect',shapeValue()));$('shape-draw').onclick=catchInput(()=>send('shape_draw',shapeValue()));
$('demo').onclick=catchInput(()=>api('demo',{enabled:!state?.demo}));
$('reset-view').onclick=()=>model?.reset();$('front-view').onclick=()=>model?.reset(true);
function section(active){model?.setInternal(active);$('assembled-view').classList.toggle('selected',!active);$('internal-view').classList.toggle('selected',active);$('section-view').classList.toggle('selected',active);}
$('assembled-view').onclick=()=>section(false);$('internal-view').onclick=()=>section(true);$('section-view').onclick=()=>section(!model?.internal);
document.addEventListener('model-error',e=>toast(e.detail));
for(const [button,dialog] of [['show-logs','logs-dialog'],['status-log-button','logs-dialog'],['show-model-config','model-dialog'],['show-switch','switch-dialog']]){
  $(button).onclick=()=>{if(dialog==='model-dialog'&&state){$('config-pan').value=state.model_config.reference_deg[0];$('config-tilt').value=state.model_config.reference_deg[1];$('direction-pan').value=state.model_config.direction[0];$('direction-tilt').value=state.model_config.direction[1];}$(dialog).showModal();};
}
document.querySelectorAll('.close-dialog').forEach(button=>button.onclick=()=>button.closest('dialog').close());
document.querySelectorAll('dialog').forEach(dialog=>dialog.addEventListener('click',e=>{if(e.target===dialog){const r=dialog.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)dialog.close();}}));
$('save-model-config').onclick=catchInput(async()=>{await api('model-config',{reference_deg:[number('config-pan'),number('config-tilt')],direction:[number('direction-pan'),number('direction-tilt')]});$('model-dialog').close();history.forEach(h=>h.length=0);toast('模型显示标定已保存');});
document.querySelectorAll('[data-switch]').forEach(el=>el.onclick=()=>{if(!$('flash-ready').checked){toast('请先确认已按说明配置双镜像Flash');return;}send('switch',Number(el.dataset.switch),{flash_ready:true});$('switch-dialog').close();});
document.querySelectorAll('[data-mode]').forEach(el=>el.onclick=()=>send('video',[0x11,Number(el.dataset.mode)]));
$('video-effect').onchange=()=>send('video',[0x64,Number($('video-effect').value)]);
$('video-color').onchange=()=>send('video',[0x21,Number($('video-color').value)]);
$('osd').onchange=()=>send('video',[0x10,Number($('osd').checked)]);
$('read-options').onclick=()=>send('read_video_options');
$('apply-options').onclick=catchInput(()=>{
  const flags=[...document.querySelectorAll('#algorithm-flags input')].reduce((n,el,i)=>n+(Number(el.checked)<<i),0);
  const scopes=number('scope-base')+(number('scope-effect')<<1)+(number('scope-histogram')<<2)+(number('region-source')<<3);
  const rect=['rect-x0','rect-y0','rect-x1','rect-y1'].map(number);
  if(rect[0]>rect[2]||rect[1]>rect[3])throw Error('矩形起点不能大于终点');
  send('video_options',[flags,scopes,rect,$('region-union').checked,$('region-locked').checked]);
});
$('apply-motion').onclick=catchInput(()=>send('video_motion',[number('motion-threshold'),number('motion-minimum')]));$('reset-motion').onclick=()=>send('motion_reset');
document.querySelectorAll('[data-jog]').forEach(button=>{
  button.onpointerdown=e=>{if(e.button!==0)return;button.setPointerCapture(e.pointerId);keys.add(button.dataset.jog);};
  button.onpointerup=button.onlostpointercapture=button.onpointercancel=()=>keys.clear();
});
document.addEventListener('keydown',e=>{
  if(e.key==='Escape'){keys.clear();send('stop');return;}
  if(view!=='video'||/^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName))return;
  const key=({ArrowUp:'w',ArrowDown:'s',ArrowLeft:'a',ArrowRight:'d'})[e.key]||e.key.toLowerCase();
  if(['w','a','s','d'].includes(key)){e.preventDefault();keys.add(key);}
});
document.addEventListener('keyup',e=>keys.delete(({ArrowUp:'w',ArrowDown:'s',ArrowLeft:'a',ArrowRight:'d'})[e.key]||e.key.toLowerCase()));
window.addEventListener('blur',()=>keys.clear());document.addEventListener('visibilitychange',()=>{if(document.hidden)keys.clear();});
function desired(){let v=Number($('jog-speed').value);if(!Number.isFinite(v)||v<.2||v>10)v=0;return view==='video'?[v*(Number(keys.has('a'))-Number(keys.has('d'))),v*(Number(keys.has('w'))-Number(keys.has('s')))]:[0,0];}
let heartbeatBusy=false;setInterval(async()=>{if(heartbeatBusy)return;heartbeatBusy=true;try{await api('heartbeat',{view,requested:desired()});}catch{}finally{heartbeatBusy=false;}},200);
window.addEventListener('pagehide',()=>{keys.clear();navigator.sendBeacon('api/release',new Blob(['{}'],{type:'application/json'}));});

function fillPorts(ports){
  const selected=$('port').value,values=[...new Set([selected||'COM5',...ports])];
  if([...$('port').options].map(o=>o.value).join()===values.join())return;
  $('port').replaceChildren(...values.map(value=>Object.assign(document.createElement('option'),{value,textContent:value})));$('port').value=selected||'COM5';
}
function updateState(data){
  const source=String(data.image)+'/'+data.demo;
  if(source!==lastSource){
    history.forEach(h=>h.length=0);sampleIds.fill(null);lastSource=source;model?.setAngles(0,0,false);
    if(data.demo){
      const now=performance.now()/1000;
      for(let t=-20;t<=0;t+=.2){
        history[0].push({time:now+t,value:26*Math.sin((data.demo_elapsed+t)*.35)});
        history[1].push({time:now+t,value:18*Math.sin((data.demo_elapsed+t)*.47+.5)});
      }
    }
  }
  state=data;fillPorts(data.ports);
  const live=data.image!==null,rows=data.poses,offsets=rows.map((row,i)=>row?data.model_config.direction[i]*angleDelta(row.angle_deg,data.model_config.reference_deg[i]):0);
  const fresh=rows.every(row=>row&&row.age_s<1.5),demo=data.demo;
  $('connection-dot').className='status-dot'+(live?' live':demo?' demo':'');
  text('connection-label',live?'设备已连接':demo?'离线演示':'设备未连接');
  $('connect').querySelector('span').textContent=live?'断开连接':'连接设备';
  text('mode-value',demo?'离线动作演示':data.image===0xD1?'视频 / 手动云台':data.image===0xD2?'光点追踪':data.image===0xE2?'形状实验模式':'等待连接');
  text('mode-note',demo?'非设备反馈':live?`工程 ${data.image.toString(16).toUpperCase()} · 串口在线`:'双轴视觉云台');
  text('mode-tag',demo?'DEMO':live?'CONNECTED':'OFFLINE');
  for(let i=0;i<2;i++){
    const id=i===0?'pan':'tilt',row=rows[i],mode=demo?'demo':row&&row.age_s<1.5?'live':row?'stale':'';
    $(id+'-angle').replaceChildren(document.createTextNode(row?row.angle_deg.toFixed(2):'—'),Object.assign(document.createElement('small'),{textContent:'°'}));
    text(id+'-speed',row?`${row.speed_rpm>=0?'+':''}${row.speed_rpm.toFixed(2)} rpm`:'— rpm');
    text(id+'-current',row?row.current_a.toFixed(3)+' A':'— A');
    text(id+'-state',demo?'演示数据':mode==='live'?(row.enabled?'已使能':'未使能'):mode==='stale'?'反馈超时':'等待反馈');$(id+'-state').className='axis-state '+mode;
    text(id+'-offset',row?`${offsets[i]>=0?'+':''}${offsets[i].toFixed(1)}°`:'—');
    if(row&&row.sample_id!==sampleIds[i]){sampleIds[i]=row.sample_id;history[i].push({time:performance.now()/1000,value:offsets[i]});}
    while(history[i].length&&(history[i][0].time<performance.now()/1000-21||history[i].length>400))history[i].shift();
  }
  model?.setAngles(offsets[0],offsets[1],rows.some(Boolean));
  $('feedback-pill').className='feedback-pill'+(demo?' demo':live&&fresh?' live':'');
  $('feedback-pill').lastChild.textContent=demo?'离线演示 · 非设备反馈':live&&fresh?'实际编码器反馈':live?'等待反馈 / 保持最后姿态':'等待设备反馈';
  $('demo').disabled=live;$('demo').classList.toggle('active',demo);$('demo').querySelector('span').textContent=demo?'停止离线演示':'离线动作演示';
  text('model-status',demo?'演示动作 · 未控制设备':live&&fresh?'编码器驱动 · 实际反馈':live?'等待反馈 · 保持姿态':'编码器驱动 · 参考零位');
  const target=data.target;
  text('target-value',target?.valid?`${target.x}, ${target.y}`:'等待检测');
  text('target-note',target?.valid?`目标尺寸 ${target.width} × ${target.height} px`:'相机画面从 HDMI 输出');
  text('calibration-name',data.calibration_name);
  $('shape-detect').disabled=$('shape-draw').disabled=!data.shape_capability;
  const shape=data.shape;
  text('shape-result',shape?`${shape.color} · ${shape.shape} · ${shape.valid?'有效目标':'未检测到目标'}`:data.shape_capability?'E7 已就绪':'连接支持 E7 的工程后启用');
  text('shape-phase',data.shape_phase);
  text('shape-diagnostics',shape?`锁定 ${shape.locked?'是':'否'} / 出光采样 ${shape.laser_now?'是':'否'} / 实时 ${shape.raw_color} ${shape.raw_shape}${shape.overflow?' / 坐标溢出':''}${shape.late?' / 超期':''}`:'');
  $('record').classList.toggle('active',data.recording);$('record').querySelector('span').textContent=data.recording?'停止':'记录';text('record-note',data.recording?'正在写入本地 CSV':'角度 · 转速 · 电流');
  if(data.video_options){
    const value=JSON.stringify([data.video_options_revision,data.video_options]);
    if(value!==optionsSeen){applyOptions(data.video_options);optionsSeen=value;}
  }
  selectVideoMode(data.video_mode);
  const stats=data.video_stats;
  text('video-stats',stats?`${stats.valid?`中心 (${stats.x}, ${stats.y}) · 尺寸 ${stats.width} × ${stats.height} px`:'未检测到目标'}\n采集实测 ${stats.camera_fps} FPS / 显示 ${stats.display_fps} FPS · 帧计数 ${stats.camera_frames} / ${stats.display_frames}\n二值阈值 ${stats.binary_threshold} / 边缘底噪 ${stats.edge_floor}`:'等待设备采样');
  const motion=data.video_motion;
  if(motion)text('motion-info',`${motion.paused?'云台移动，暂停':motion.warming?'建立参考帧':motion.scene_change?'全局变化抑制':'检测中'} · 区域 ${motion.count} 个 / 变化点 ${motion.changed_cells}`);
  updateLogs(data.logs);
  $('footer-dot').className='status-dot'+(live?' live':demo?' demo':'');text('footer-time',demo?'LOCAL / DEMO':live?'LOCAL / CONNECTED':'LOCAL / OFFLINE');
  drawChart();
}
function selectVideoMode(mode){
  document.querySelectorAll('[data-mode]').forEach(el=>{
    const selected=Number(el.dataset.mode)===mode;
    el.classList.toggle('selected',selected);el.setAttribute('aria-pressed',String(selected));
  });
}
function applyOptions(o){
  $('video-effect').value=o.effect;
  document.querySelectorAll('#algorithm-flags input').forEach((el,i)=>el.checked=Boolean(o.flags&(1<<i)));
  ['scope-base','scope-effect','scope-histogram'].forEach((id,i)=>$(id).value=Number(Boolean(o.scopes&(1<<i))));
  $('region-source').value=Number(Boolean(o.scopes&8));$('region-union').checked=o.union;$('region-locked').checked=o.locked;
  ['rect-x0','rect-y0','rect-x1','rect-y1'].forEach((id,i)=>$(id).value=o.rect[i]);
  text('video-options-note','已读回设备参数'+(o.quad_supported?' · 支持四点四边形特效选区':''));
}
function updateLogs(logs){
  if(!logs.length||logs.at(-1).id===lastLog)return;
  const latest=logs.at(-1);lastLog=latest.id;text('footer-message',latest.message);$('footer-message').title=latest.message;
  $('logs-list').replaceChildren(...[...logs].reverse().map(log=>{
    const entry=document.createElement('div');entry.className='log-entry '+log.level;
    entry.append(Object.assign(document.createElement('time'),{textContent:log.time}),Object.assign(document.createElement('span'),{textContent:log.message}));return entry;
  }));
  if(latest.level==='error')toast(latest.message);
}
let pollBusy=false,pollTimer;
async function poll(force=false){
  if(pollBusy)return;pollBusy=true;clearTimeout(pollTimer);
  try{updateState(await api('state'));}catch(e){
    text('connection-label','本地连接已中断');text('footer-message','本地控制通道已断开，请重新打开上位机');keys.clear();
    $('connection-dot').className='status-dot';$('feedback-pill').className='feedback-pill demo';
    $('feedback-pill').lastChild.textContent='连接中断 · 保持最后姿态';
    for(const id of ['pan','tilt']){$(id+'-state').className='axis-state stale';text(id+'-state','反馈中断');}
  }
  finally{pollBusy=false;pollTimer=setTimeout(poll,200);}
}
function drawChart(){
  const canvas=$('pose-chart'),w=canvas.clientWidth,h=canvas.clientHeight;if(!w||!h)return;
  const ratio=Math.min(devicePixelRatio,2);if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)){canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);}
  const c=canvas.getContext('2d');c.setTransform(ratio,0,0,ratio,0,0);c.clearRect(0,0,w,h);
  const left=43,right=w-13,top=18,bottom=h-25,now=performance.now()/1000;
  c.font='11px Consolas';c.textAlign='right';c.lineWidth=1;
  for(const value of [-60,0,60,120]){const y=bottom-(value+60)/180*(bottom-top);c.strokeStyle='#ffffff08';c.setLineDash([2,5]);c.beginPath();c.moveTo(left,y);c.lineTo(right,y);c.stroke();c.fillStyle='#415c70';c.fillText(value+'°',left-7,y+3);}
  c.setLineDash([]);c.textAlign='center';
  for(const s of [20,15,10,5,0]){c.fillStyle='#415c70';c.fillText(s===0?'NOW':'−'+s+'s',right-s/20*(right-left),h-6);}
  let any=false;
  history.forEach((samples,i)=>{
    const points=samples.filter(s=>s.time>=now-20);if(points.length<2)return;any=true;
    c.strokeStyle=i===0?'#9be6cf':'#bbadf0';c.lineWidth=1.8;c.beginPath();let previous;
    for(const p of points){const x=right-(now-p.time)/20*(right-left),y=bottom-(Math.max(-60,Math.min(120,p.value))+60)/180*(bottom-top);
      if(!previous||p.time-previous.time>1.5||Math.abs(p.value-previous.value)>180)c.moveTo(x,y);else c.lineTo(x,y);previous=p;
    }c.stroke();
  });
  $('chart-empty').classList.toggle('hidden',any);
}
new ResizeObserver(drawChart).observe($('pose-chart'));
async function startModel(){
  try{model=new AssemblyView($('model-viewport'));const manifest=await model.load();$('model-loading').classList.add('loaded');document.body.dataset.modelReady='true';
    window.consoleModel=model;window.consoleManifest=manifest;
  }catch(e){errors.push(String(e));$('model-loading').replaceChildren(Object.assign(document.createElement('p'),{className:'model-error',textContent:'三维模型加载失败：'+e.message}));}
}
poll();await startModel();
if(new URL(location.href).searchParams.has('qa')){
  await new Promise(resolve=>setTimeout(resolve,700));const checks=[];
  const check=(name,value)=>{checks.push({name,passed:Boolean(value)});};
  check('original_mesh_count',model?.parts.length===96);check('original_triangle_count',model?.triangles===170130);
  check('fixed_base',model?.groups[8].parent===model?.modelRoot&&model?.groups[3].parent===model?.modelRoot);
  check('pan_hierarchy',model?.groups[5].parent===model?.panRoot&&model?.tiltRoot.parent===model?.panRoot);
  check('tilt_hierarchy',model?.groups[6].parent===model?.tiltRoot&&model?.groups[14].parent===model?.tiltRoot);
  if(model?.ready){model.setInternal(true);check('internal_structure',!model.groups[2].visible&&!model.groups[9].visible);model.setInternal(false);check('assembled_structure',model.groups[2].visible&&model.groups[9].visible);
    model.displayPan=20;model.displayTilt=35;model.applyAngles();check('joint_rotation',Math.abs(model.panRoot.rotation.z-20*Math.PI/180)<1e-6&&Math.abs(model.tiltRoot.rotation.x-35*Math.PI/180)<1e-6);model.render();check('webgl',model.renderer.getContext().getError()===0);
  }
  check('encoder_wrap',angleDelta(.5,359.5)===1&&angleDelta(359.5,.5)===-1);
  const renderer=model?.renderer,azimuth=model?.azimuth;
  showView('video');model?.resize();
  check('video_workspace',!$('video-view').classList.contains('hidden')&&$('tracking-view').classList.contains('hidden'));
  check('video_model_visible',$('gimbal-model').parentElement===$('video-model-host')&&$('model-viewport').clientHeight>200&&$('model-viewport').getBoundingClientRect().width>0);
  check('video_layout_width',document.body.scrollWidth<=innerWidth);
  if(state?.preview&&model?.ready){
    const saved=state;
    for(let mode=0;mode<6;mode++){
      updateState({...saved,video_mode:mode});
      const selected=[...document.querySelectorAll('[data-mode].selected')];
      check('video_mode_'+mode,selected.length===1&&Number(selected[0].dataset.mode)===mode&&selected[0].getAttribute('aria-pressed')==='true');
    }
    const active=document.querySelector('[data-mode].selected'),inactive=document.querySelector('[data-mode]:not(.selected)');
    // Inspect the final selection colors rather than the first transition frame.
    for(const tile of [active,inactive])tile.getAnimations().forEach(animation=>animation.finish());
    const color=getComputedStyle(active).backgroundColor.match(/\d+/g).map(Number);
    check('video_selection_green',color[1]>color[0]&&color[1]>color[2]&&getComputedStyle(active).backgroundColor!==getComputedStyle(inactive).backgroundColor);
    const rows=[21,34].map((offset,i)=>({angle_deg:(saved.model_config.reference_deg[i]+offset/saved.model_config.direction[i]+360)%360,speed_rpm:1,current_a:.2,enabled:true,age_s:0,sample_id:'qa-'+i}));
    updateState({...saved,image:0xD1,demo:false,poses:rows});model.displayPan=model.pan;model.displayTilt=model.tilt;model.applyAngles();model.render();
    check('video_encoder_pose',Math.abs(model.panRoot.rotation.z-21*Math.PI/180)<1e-6&&Math.abs(model.tiltRoot.rotation.x-34*Math.PI/180)<1e-6);
    showView('tracking');showView('video');model.resize();
    check('shared_model_preserves_pose',model.renderer===renderer&&model.azimuth===azimuth&&Math.abs(model.pan-21)<1e-6&&Math.abs(model.tilt-34)<1e-6);
    check('corrected_pitch_direction',saved.model_config.direction[0]===1&&saved.model_config.direction[1]===-1);
    const jogSpeed=Number($('jog-speed').value);
    for(const [key,axis,sign] of [['w',1,1],['a',0,1],['s',1,-1],['d',0,-1]]){
      document.dispatchEvent(new KeyboardEvent('keydown',{key,bubbles:true}));
      const request=desired();
      check('wasd_request_'+key,request[axis]===sign*jogSpeed&&request[1-axis]===0);
      const feedback=[0,1].map(i=>({angle_deg:saved.model_config.reference_deg[i]+(i===axis?12*sign:0),speed_rpm:request[i],current_a:.2,enabled:true,age_s:0,sample_id:'qa-key-'+key+'-'+i}));
      updateState({...saved,image:0xD1,demo:false,poses:feedback});
      model.displayPan=0;model.displayTilt=0;
      // Advance the real animation callback, including its interpolation and rendering.
      cancelAnimationFrame(model.frame);model.lastFrame=performance.now()-50;model.animate(performance.now());
      const expected=12*sign*saved.model_config.direction[axis],rotation=axis===0?model.panRoot.rotation.z:model.tiltRoot.rotation.x;
      check('wasd_model_moves_'+key,Math.abs((axis===0?model.pan:model.tilt)-expected)<1e-6&&rotation*expected>0);
      document.dispatchEvent(new KeyboardEvent('keyup',{key,bubbles:true}));
      check('wasd_release_'+key,desired().every(value=>value===0));
    }
    showView('tracking');model.resize();
    for(const delta of [15,-15]){
      const feedback=[0,1].map(i=>({angle_deg:saved.model_config.reference_deg[i]+(i===1?delta:10),speed_rpm:0,current_a:.2,enabled:true,age_s:0,sample_id:'qa-track-'+delta+'-'+i}));
      updateState({...saved,image:0xD2,demo:false,poses:feedback});
      check('tracking_pitch_'+delta,Math.abs(model.pan-10)<1e-6&&Math.abs(model.tilt+delta)<1e-6);
    }
    updateState(saved);
  }
  showView('tracking');model?.resize();
  check('tracking_model_restored',$('gimbal-model').parentElement===$('tracking-model-host')&&document.querySelectorAll('#model-viewport canvas').length===1);
  check('layout_width',document.body.scrollWidth<=innerWidth);check('no_script_errors',errors.length===0);
  check('timeline_visible',$('pose-chart').getBoundingClientRect().bottom<=innerHeight-20);
  try{const reply=await api('heartbeat',{view:'tracking',requested:[0,0]});check('local_control_channel',reply.ok);}catch{check('local_control_channel',false);}
  const stopRect=document.querySelector('#tracking-view [data-action="stop"]').getBoundingClientRect();
  check('stop_button_visible',stopRect.bottom<=Math.min(innerHeight,document.querySelector('.control-column').getBoundingClientRect().bottom));
  const result={passed:checks.every(c=>c.passed),checks,errors,preview:state?.preview,source_sha256:model?.manifest.source_sha256,
    viewport:{width:innerWidth,height:innerHeight,timeline_bottom:$('pose-chart').getBoundingClientRect().bottom}};
  const node=Object.assign(document.createElement('pre'),{id:'qa-result',hidden:true,textContent:JSON.stringify(result)});document.body.append(node);document.body.dataset.qaPassed=String(result.passed);
  if(state){updateState(state);model.displayPan=model.pan;model.displayTilt=model.tilt;model.applyAngles();model.render();}
}
const previewParams=new URL(location.href).searchParams;
if(previewParams.get('internal')==='1')section(true);
if(previewParams.get('view')==='video')showView('video');
