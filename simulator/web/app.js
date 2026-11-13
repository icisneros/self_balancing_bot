'use strict';
const $ = id => document.getElementById(id);
const colors = {cyan:'#65e4c5', amber:'#e8b26a', violet:'#b6a0fa', muted:'#92a4b6'};
let defaults, result=null, time=0, playing=false, lastFrame=0, busy=false, dirty=false;
let motorGeometry=null;
const fields = [
  ['gain','kp','Proportional · Kp',0,120,.5,'PWM / degree'],
  ['gain','ki','Integral · Ki',0,120,.5,'PWM / (degree · second)'],
  ['gain','kd','Derivative · Kd',0,10,.05,'PWM · second / degree'],
  ['controller','setpoint','Pitch setpoint',-180,180,.1,'Pitch requested by the controller; match this to your IMU mounting.'],
  ['controller','sample_ms','PID sample time',1,100,1,'ms; actual updates also depend on IMU events.'],
  ['controller','min_pwm','Minimum motor PWM',0,100,1,'0 allows a true zero command. Original: 25.'],
  ['controller','sync_motors','Update both wheels together','check','Original sketch alternates left and right.'],
  ['controller','prime_pid','Initialize PID with the first angle','check','Avoids interpreting startup as an angle jump.'],
  ['sensor','imu_hz','IMU output rate',10,500,10,'Hz'],
  ['sensor','imu_delay_ms','IMU delay',0,100,1,'ms; a pure delay, independent of sample rate.'],
  ['sensor','noise_deg','Angle noise',0,2,.01,'degrees RMS; repeatable random seed.'],
  ['sensor','imu_x','IMU position · x',-1,1,.001,'m forward from axle; drawing only.'],
  ['sensor','imu_y','IMU position · y',-1,1,.001,'m left from axle; out of the side-view plane.'],
  ['sensor','imu_z','IMU position · z',-1,1,.001,'m above axle; drawing only.'],
  ['sensor','imu_roll_deg','IMU mounting · roll',-180,180,.1,'degrees; sensor-to-body Rx rotation.'],
  ['sensor','imu_pitch_deg','IMU mounting · pitch',-180,180,.1,'degrees; sensor-to-body Ry rotation.'],
  ['sensor','imu_yaw_deg','IMU mounting · yaw',-180,180,.1,'degrees; sensor-to-body Rz rotation.'],
  ['sensor','fold_angle','Use legacy ±90° folded pitch','check','Compare against the assumed continuous custom mapping.'],
  ['sensor','dropout_at','Dropout starts at',0,30,.1,'seconds'],
  ['sensor','dropout_ms','Dropout duration',0,2000,10,'ms; 0 disables missing sensor events.'],
  ['sensor','corrupt_every','Reject every Nth packet',0,1000,1,'0 disables; 1 rejects every sample.'],
  ['plant','body_mass','Body mass',.1,5,.05,'kg'],
  ['plant','body_height','Body height',.05,1,.005,'m; drawing dimension, inertia is specified separately.'],
  ['plant','body_depth','Body depth',.01,.5,.005,'m front to back; drawing dimension.'],
  ['plant','axle_mass','Axle / wheel mass',.1,5,.05,'kg; effective translating base mass.'],
  ['plant','com_height','Center of mass height',.05,.5,.005,'m above axle'],
  ['plant','com_x','Center of mass · x',-.3,.3,.001,'m forward of axle; affects gravitational torque.'],
  ['plant','pitch_inertia','Body pitch inertia',.00001,2,.00001,'kg · m² about the center of mass.'],
  ['plant','gravity','Gravity',.1,30,.01,'m/s²'],
  ['plant','wheel_radius','Wheel radius',.01,.15,.001,'m'],
  ['plant','motor_voltage','Motor terminal voltage',.1,24,.1,'V; assumed operating voltage, not a measured battery voltage.'],
  ['plant','max_force','Combined motor force',.001,500,.1,'N; derived from stall torque, voltage and wheel radius. Trial override.'],
  ['plant','motor_lag_ms','Motor response time',1,200,1,'ms; first-order time constant.'],
  ['plant','deadband','Motor deadband',0,150,1,'PWM; physical motor friction approximation.'],
  ['plant','free_speed','Wheel no-load speed',.001,20,.01,'m/s; derived from output RPM, voltage and radius. Trial override.'],
  ['plant','drag','Axle drag',0,3,.05,'N · s/m'],
  ['plant','angular_damping','Body angular damping',0,.2,.001,'N · m · s/rad'],
  ['plant','polarity','Motor polarity','select',[[1,'Corrective'],[-1,'Reversed']]],
  ['trial','initial_angle','Initial body tilt',-30,30,.5,'degrees from vertical'],
  ['trial','duration','Trial duration',1,30,1,'seconds; stops at a 60° fall.'],
  ['trial','push_at','Push time',-1,30,.05,'seconds; -1 disables the scheduled push.'],
  ['trial','push','Push impulse',-100,100,1,'degrees/s added to body angular velocity.'],
];

function createControls() {
  for (const [group, key, label, low, high, step, help] of fields) {
    const container = $(group+'-fields');
    const wrapper = document.createElement('div');
    if (low==='check') {
      wrapper.className='field-toggle';
      wrapper.innerHTML=`<input type="checkbox" id="${key}" name="${key}"><label for="${key}">${label}<small>${high}</small></label>`;
    } else if (low==='select') {
      wrapper.className='field';
      wrapper.innerHTML=`<div class="field-top"><label for="${key}">${label}</label><select id="${key}" name="${key}">${high.map(([value,text])=>`<option value="${value}">${text}</option>`).join('')}</select></div>`;
    } else {
      wrapper.className='field';
      // Numeric entries preserve precision; the range is a convenient coarse control.
      wrapper.innerHTML=`<div class="field-top"><label for="${key}">${label}</label><input type="number" id="${key}" name="${key}" min="${low}" max="${high}" step="any" required></div><input type="range" id="${key}-range" aria-label="${label}" min="${low}" max="${high}" step="${step}"><small>${help}</small>`;
      wrapper.querySelector('input[type=range]').addEventListener('input', e=>{
        $(key).value=e.target.value; markDirty();
      });
      wrapper.querySelector('input[type=number]').addEventListener('input', e=>{
        $(key+'-range').value=e.target.value;
      });
    }
    container.appendChild(wrapper);
  }
  $('controls').addEventListener('input',markDirty);
  $('controls').addEventListener('submit',e=>{e.preventDefault();runSimulation();});
}

function markDirty(event) {
  if(event && ['wheel_radius','wheel_radius-range','motor_voltage','motor_voltage-range'].includes(event.target.id)){
    const radius=Number($('wheel_radius').value), voltage=Number($('motor_voltage').value);
    if(motorGeometry && radius>0 && voltage>0){
      const voltageRatio=voltage/motorGeometry.voltage;
      for(const [key,scale] of [['max_force',motorGeometry.radius/radius*voltageRatio],['free_speed',radius/motorGeometry.radius*voltageRatio]]){
        $(key).value=Number($(key).value)*scale;$(key+'-range').value=$(key).value;
      }
      motorGeometry={radius,voltage};
    }
  }
  dirty=true;
  $('pending').textContent='Settings changed. Run again to apply them.';
  $('pending').classList.add('dirty');
}
function applySettings(values) {
  for (const [,key] of fields) {
    const value=values[key] ?? defaults[key], el=$(key);
    if(el.type==='checkbox') el.checked=Boolean(value); else el.value=value;
    if($(key+'-range')) $(key+'-range').value=value;
  }
  motorGeometry={radius:Number($('wheel_radius').value),voltage:Number($('motor_voltage').value)};
  markDirty();
}
function readSettings() {
  const invalid=$('controls').querySelector('input:invalid');
  if(invalid?.closest('details'))invalid.closest('details').open=true;
  if(!$('controls').reportValidity()) throw new Error('Check the highlighted setting.');
  return Object.fromEntries(fields.map(([,key])=>[key,$(key).type==='checkbox'?Number($(key).checked):Number($(key).value)]));
}
function preset(name) {
  const settings={...defaults};
  if(name==='tuning') Object.assign(settings,{kp:25,ki:80,kd:.3,min_pwm:0,sync_motors:1,prime_pid:1,sample_ms:10});
  applySettings(settings);
  $('preset-note').textContent=name==='tuning'
    ? 'An example that balances the assumed model: both wheels together, zero minimum PWM, primed PID, and 10 ms updates. This changes multiple settings.'
    : 'Preserves alternating wheel updates, the minimum PWM, and the original PID initialization.';
}
async function runSimulation({seek=0,autoPlay=true}={}) {
  if(busy) return;
  let settings;
  try { settings=readSettings(); } catch(error) { $('pending').textContent=error.message;return; }
  busy=true;playing=false;$('run').disabled=true;$('run').textContent='Running native sketch…';
  $('status').className='status';$('status').textContent='Computing…';
  try {
    const response=await fetch('/api/simulate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(settings)});
    const data=await response.json();
    if(!response.ok) throw new Error(data.error || 'Simulation failed');
    result=data;time=Math.min(seek,lastSample()[0]);playing=autoPlay;dirty=false;
    $('pending').textContent=`Arduino trial · robot YAML ${data.robot_config.sha256.slice(0,8)} · build ${data.build.slice(0,8)}`;
    $('pending').classList.remove('dirty');
    for(const id of ['play','restart','nudge','scrub','csv','settings']) $(id).disabled=false;
    $('scrub').max=lastSample()[0];$('scrub').value=time;
    const s=data.summary;
    $('balance-time').textContent=(s.fall_time ?? lastSample()[0]).toFixed(2)+' s';
    $('rms').textContent=s.rms_angle.toFixed(2)+'°';
    $('saturation').textContent=s.saturation_pct.toFixed(1)+'%';
    $('interval').textContent=s.pid_dt_min===s.pid_dt_max?`${s.pid_dt_min} ms`:`${s.pid_dt_min}–${s.pid_dt_max} ms`;
    $('diagnostics').textContent=`${s.pid_updates} PID updates · ${s.imu_reads} IMU reads · ${s.dropped_samples} missed · ${s.corrupt_samples} rejected packets`;
    $('status').className='status'+(s.fall_time===null?'':' fallen');
    $('status').textContent=s.fall_time===null?`Upright for ${lastSample()[0].toFixed(0)} s`:`Fell at ${s.fall_time.toFixed(2)} s`;
    updatePlayback();
  } catch(error) {
    $('pending').textContent=error.message;
    $('status').className='status error';$('status').textContent='Simulation failed';
  } finally {
    busy=false;$('run').disabled=false;$('run').innerHTML='Run simulation <span>↗</span>';
  }
}
function lastSample(){return result.samples[result.samples.length-1];}
function currentSample(){
  let low=0,high=result.samples.length-1;
  while(low<high){const middle=Math.ceil((low+high)/2);if(result.samples[middle][0]<=time)low=middle;else high=middle-1;}
  return result.samples[low];
}
function updatePlayback(){ $('play').textContent=playing?'Pause':'Play'; }
function context(id) {
  const canvas=$(id), dpr=window.devicePixelRatio||1;
  const w=canvas.clientWidth,h=canvas.clientHeight;
  if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){
    canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);
  }
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
  return [ctx,w,h];
}
function robot() {
  const [c,w,h]=context('robot'), s=result?currentSample():[0,3,0,0,0,0,0,0,0,0,0,-1];
  const config=result?result.settings:defaults;
  const cx=w*.5, axle=h-60, theta=s[1]*Math.PI/180;
  const scale=Math.min(145/config.body_height, 140/config.body_depth, 35/config.wheel_radius);
  const wheel=config.wheel_radius*scale;
  const bodyHeight=config.body_height*scale, bodyDepth=config.body_depth*scale;
  c.strokeStyle='#1e2e3d';c.lineWidth=1;
  for(let x=0;x<w;x+=35){c.beginPath();c.moveTo(x,0);c.lineTo(x,h);c.stroke();}
  for(let y=0;y<h;y+=35){c.beginPath();c.moveTo(0,y);c.lineTo(w,y);c.stroke();}
  c.strokeStyle='#344756';c.beginPath();c.moveTo(0,axle+wheel);c.lineTo(w,axle+wheel);c.stroke();
  const offset=((s[4]*200)%40+40)%40;
  for(let x=-40;x<w+40;x+=40){c.beginPath();c.moveTo(x-offset,axle+wheel+10);c.lineTo(x-offset+15,axle+wheel+10);c.stroke();}
  c.setLineDash([4,5]);c.strokeStyle='#49616c';c.beginPath();c.moveTo(cx,axle);c.lineTo(cx,22);c.stroke();
  if(result){const target=result.target_body_angle*Math.PI/180;c.strokeStyle=colors.muted;c.beginPath();c.moveTo(cx,axle);c.lineTo(cx+Math.sin(target)*150,axle-Math.cos(target)*150);c.stroke();}
  c.setLineDash([]);
  c.save();c.translate(cx,axle);c.rotate(theta);
  c.fillStyle='#1d3d3b';c.strokeStyle=colors.cyan;c.lineWidth=2;
  c.beginPath();c.roundRect(-bodyDepth/2,-bodyHeight,bodyDepth,bodyHeight,Math.min(10,bodyDepth/4));c.fill();c.stroke();
  const imuX=config.imu_x*scale, imuZ=-config.imu_z*scale;
  c.fillStyle=colors.amber;c.beginPath();c.arc(config.com_x*scale,-config.com_height*scale,4,0,Math.PI*2);c.fill();
  c.font='9px ui-monospace,monospace';c.fillStyle=colors.muted;c.textAlign='center';c.fillText('COM',config.com_x*scale,-config.com_height*scale+16);
  c.save();c.translate(imuX,imuZ);c.rotate(config.imu_pitch_deg*Math.PI/180);
  c.fillStyle=colors.cyan;c.fillRect(-6,-4,12,8);c.restore();
  c.fillStyle=colors.muted;c.fillText('IMU',imuX,imuZ-11);
  c.restore();
  c.fillStyle='#0b1018';c.strokeStyle='#b9c8d3';c.lineWidth=3;c.beginPath();c.arc(cx,axle,wheel,0,Math.PI*2);c.fill();c.stroke();
  c.save();c.translate(cx,axle);c.rotate(s[4]/config.wheel_radius);
  c.strokeStyle='#526c7e';c.lineWidth=1.5;
  for(let i=0;i<6;i++){c.rotate(Math.PI/3);c.beginPath();c.moveTo(0,0);c.lineTo(wheel*.82,0);c.stroke();}c.restore();
  c.fillStyle=colors.cyan;c.beginPath();c.arc(cx,axle,Math.min(5,wheel*.25),0,Math.PI*2);c.fill();
  c.textAlign='left';c.fillStyle=colors.muted;c.font='10px ui-monospace,monospace';
  c.fillText('← backward    forward →',18,22);c.fillText(`Axle position ${s[4].toFixed(2)} m`,18,h-8);
  c.textAlign='right';c.fillText('Positive tilt →',w-18,22);
  $('time').textContent=time.toFixed(2)+' s';
  $('live-angle').textContent=s[1].toFixed(2)+'°';
  $('live-left').textContent=Math.round(s[7]);$('live-right').textContent=Math.round(s[8]);
  $('live-age').textContent=s[11]<0?'waiting':Math.round(s[11])+' ms';
}
function plot(id,series,range,target=null) {
  const [c,w,h]=context(id), left=45,right=w-18,top=20,bottom=h-28;
  const end=result?Math.max(.1,lastSample()[0]):12;
  const xx=t=>left+t/end*(right-left), yy=v=>bottom-(v+range)/(2*range)*(bottom-top);
  c.font='9px ui-monospace,monospace';c.textAlign='right';c.strokeStyle='#273646';c.fillStyle=colors.muted;c.lineWidth=1;
  for(let i=0;i<=4;i++){const v=-range+i*range/2,y=yy(v);c.beginPath();c.moveTo(left,y);c.lineTo(right,y);c.stroke();c.fillText(v.toFixed(range<10?1:0),left-8,y+3);}
  c.textAlign='center';
  for(let i=0;i<=4;i++){const t=end*i/4;c.fillText(t.toFixed(end<3?2:1)+'s',xx(t),h-10);}
  if(target!==null){c.strokeStyle=colors.muted;c.setLineDash([4,4]);c.beginPath();c.moveTo(left,yy(target));c.lineTo(right,yy(target));c.stroke();c.setLineDash([]);}
  if(!result)return;
  c.save();c.beginPath();c.rect(left,top,right-left,bottom-top);c.clip();
  for(const [column,color,offset=0] of series){
    c.strokeStyle=color;c.lineWidth=1.5;c.beginPath();let started=false,previous=null;
    for(const s of result.samples){
      if(column===2&&s[13]===0)continue;
      const x=xx(s[0]),y=yy(s[column]-offset);
      if(!started){c.moveTo(x,y);started=true;}else{if(id==='motor-plot')c.lineTo(x,previous);c.lineTo(x,y);}
      previous=y;
    }c.stroke();
  }
  c.strokeStyle='#edf6fa88';c.lineWidth=1;c.beginPath();c.moveTo(xx(time),top);c.lineTo(xx(time),bottom);c.stroke();c.restore();
}
function render(now){
  if(playing&&result&&!busy){time=Math.min(lastSample()[0],time+Math.min(.1,(now-lastFrame)/1000)*Number($('speed').value));if(time>=lastSample()[0]){playing=false;updatePlayback();}$('scrub').value=time;}
  lastFrame=now;robot();
  const reference=result?result.imu_upright_pitch:0;
  const target=result?result.settings.setpoint-reference:0;
  const max=result?Math.max(5,result.summary.max_angle*1.15,Math.abs(target)*1.2):10;
  plot('angle-plot',[[1,colors.cyan],[2,colors.amber,reference]],max,target);
  plot('motor-plot',[[6,colors.amber],[7,colors.cyan],[8,colors.violet]],280);
  requestAnimationFrame(render);
}
function download(name,body,type){const url=URL.createObjectURL(new Blob([body],{type}));const a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}

$('run').addEventListener('click',()=>runSimulation());
$('preset').addEventListener('change',e=>preset(e.target.value));
$('reset').addEventListener('click',()=>{$('preset').value='original';preset('original');});
$('play').addEventListener('click',()=>{if(time>=lastSample()[0])time=0;playing=!playing;updatePlayback();});
$('restart').addEventListener('click',()=>{time=0;playing=true;updatePlayback();});
$('scrub').addEventListener('input',e=>{time=Number(e.target.value);playing=false;updatePlayback();});
$('nudge').addEventListener('click',()=>{
  if(!result||busy)return;
  // Recompute the same deterministic run with one scheduled angular impulse.
  applySettings(result.settings);
  $('push_at').value=(Math.round(Math.min(time+.15,result.settings.duration-.05)*20)/20).toFixed(2);
  $('push_at-range').value=$('push_at').value;$('push').value=15;$('push-range').value=15;
  runSimulation({seek:time});
});
$('csv').addEventListener('click',()=>download('balance-trial.csv',[result.columns.join(','),...result.samples.map(s=>s.join(','))].join('\n'),'text/csv'));
$('settings').addEventListener('click',()=>download('balance-settings.json',JSON.stringify(result.settings,null,2),'application/json'));
$('import').addEventListener('change',async e=>{
  try{
    if(!e.target.files[0])return;
    const settings=JSON.parse(await e.target.files[0].text());
    if(!settings||Array.isArray(settings)||typeof settings!=='object')throw new Error('Settings must be a JSON object.');
    for(const [key,value] of Object.entries(settings)){if(!(key in defaults)||typeof value!=='number'||!Number.isFinite(value))throw new Error(`Invalid setting: ${key}`);}
    applySettings(settings);$('preset-note').textContent='Imported settings. Review and run the trial.';
  }catch(error){$('pending').textContent=error.message;}finally{e.target.value='';}
});
(async()=>{
  try{
    const response=await fetch('/api/defaults');if(!response.ok){const error=await response.json();throw new Error(error.error||'Could not load defaults.');}
    defaults=await response.json();createControls();preset('original');
    requestAnimationFrame(render);await runSimulation({autoPlay:false});
  }catch(error){$('status').textContent='Server unavailable';$('pending').textContent=error.message;}
})();
