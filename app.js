const MOTOR = {
  NEO:    { free_rpm: 5676, stall_torque_Nm: 2.6, stall_current_A: 105, name:'NEO V1.1' },
  VORTEX: { free_rpm: 6784, stall_torque_Nm: 3.6, stall_current_A: 211, name:'NEO Vortex' }
};
const AZ = { total_ratio: 46.42, neo550_free_rpm: 11000 };
const g = 9.80665;
const BUMPER_M = 0.1524; // 6 in bumpers
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
const orientCanvas = document.getElementById('orient');
const orientCtx = orientCanvas.getContext('2d');
const hud = {
  mode: document.getElementById('mode'),
  cycle: document.getElementById('cycle'),
  pickups: document.getElementById('pickups'),
  spd: document.getElementById('spd'),
  om: document.getElementById('om'),
  freeV: document.getElementById('freeV'),
  freeW: document.getElementById('freeW'),
};
const ui = {
  motorSel: document.getElementById('motorSel'),
  ratioSel: document.getElementById('ratioSel'),
  profileSel: document.getElementById('profileSel'),
  modeSel: document.getElementById('modeSel'),
  calBtn: document.getElementById('calBtn'),
  defBtn: document.getElementById('defBtn'),
  recBtn: document.getElementById('recBtn'),
  playBtn: document.getElementById('playBtn'),
  calPanel: document.getElementById('calPanel'),
  sprint: document.getElementById('calSprint'),
  spin: document.getElementById('calSpin'),
  stop: document.getElementById('calStop'),
  applyCal: document.getElementById('applyCal'),
  closeCal: document.getElementById('closeCal'),
  toast: document.getElementById('toast'),
};
const field = { Wm: 16.54, Hm: 8.21, pxPerM: 50 };
let fieldPxW = 0, fieldPxH = 0;
const origin = { x:0, y:0 };
let profiles = JSON.parse(document.getElementById('profiles').textContent);
let gearChart = JSON.parse(document.getElementById('gearChart').textContent);
let profile = null;
let motor = 'NEO';
let driveRatio = 5.50;
let lim = { vFree: 4.5, Fmax: 0, Iz: 1, omAzFree: 20 };
let pose = { x:0, y:0, th: 0 };
let velCmd = { vx:0, vy:0, om:0 };
let velAct = { vx:0, vy:0, om:0 };
let fieldCentric = true;
let moduleAngles = [0,0,0,0];
let pickupTargets = [];
let carrying = false, scoreZone = { x:0, y:0, r:0 };
let pickupsDone = 0, cycleStart = performance.now();
let inputQ = []; let lastY=false, lastA=false;
let recording = false, recordBuf = [];
let playing = false, playBuf = [], playIdx = 0, playT0 = 0;
let defenderOn = false, defender = { x:0, y:0, vmax: 3.0, r: 18 };
const clamp = (v,a,b)=> Math.max(a, Math.min(b,v));
const db = (v,d)=> Math.abs(v)<d ? 0 : (v - Math.sign(v)*d)/(1-d);
function rotF(vx,vy,ang){ const c=Math.cos(ang), s=Math.sin(ang); return [c*vx - s*vy, s*vx + c*vy]; }
const dist = (x1,y1,x2,y2)=> Math.hypot(x2-x1,y2-y1);
function toast(msg){ ui.toast.textContent = msg; ui.toast.style.display='block'; setTimeout(()=>ui.toast.style.display='none', 2000); }
function loadUI(){
  ui.ratioSel.innerHTML = '';
  Object.keys(gearChart).forEach(k=>{ const opt = document.createElement('option'); opt.value = k; const n = gearChart[k].name; opt.textContent = `${n}`; ui.ratioSel.appendChild(opt); });
  ui.ratioSel.value = '5.50';
  ui.profileSel.innerHTML = '';
  Object.keys(profiles).forEach(k=>{ const o=document.createElement('option'); o.value=k; o.textContent = profiles[k].name||k; ui.profileSel.appendChild(o); });
  ui.profileSel.value = Object.keys(profiles)[0];
  setProfile(ui.profileSel.value);
  setMotor(ui.motorSel.value);
  setRatio(ui.ratioSel.value);
  setControlMode(true);
}
function setProfile(key){ profile = JSON.parse(JSON.stringify(profiles[key])); computeDerivedLimits(); resetCycle(); toast(`Profile: ${profile.name}`); }
function setMotor(m){ motor = m; computeDerivedLimits(); }
function setRatio(r){ driveRatio = parseFloat(r); computeDerivedLimits(); }
function setControlMode(fc){
  fieldCentric = fc;
  hud.mode.textContent = fieldCentric ? 'Field-centric' : 'Robot-centric';
  if (ui.modeSel) ui.modeSel.value = fieldCentric ? 'field' : 'robot';
}
function computeDerivedLimits(){
  const chart = gearChart[driveRatio.toFixed(2)];
  lim.vFree = chart ? chart[motor] : 4.5;
  lim.Fmax = (profile.mu || 1.2) * profile.mass_kg * 9.80665;
  const L = profile.wheelbase_m, W = profile.track_m; lim.Iz = (1/12) * profile.mass_kg * (L*L + W*W);
  const omMotor = (AZ.neo550_free_rpm * 2*Math.PI) / 60; lim.omAzFree = (omMotor / AZ.total_ratio);
  hud.freeV.textContent = lim.vFree.toFixed(2);
  hud.freeW.textContent = lim.omAzFree.toFixed(1);
}

function resizeSim(){
  const rect = canvas.getBoundingClientRect();
  const prev = field.pxPerM;
  canvas.width = rect.width;
  canvas.height = rect.height;
  const newPxPerM = Math.min(rect.width/field.Wm, rect.height/field.Hm);
  const scale = prev ? newPxPerM / prev : 1;
  field.pxPerM = newPxPerM;
  fieldPxW = field.Wm * field.pxPerM;
  fieldPxH = field.Hm * field.pxPerM;
  origin.x = (canvas.width - fieldPxW) / 2;
  origin.y = (canvas.height - fieldPxH) / 2;
  pose.x *= scale; pose.y *= scale;
  pickupTargets.forEach(p=>{ p.x *= scale; p.y *= scale; });
  scoreZone.x *= scale; scoreZone.y *= scale; scoreZone.r *= scale;
  defender.x *= scale; defender.y *= scale; defender.r *= scale;
}
function pollGamepad(){
  const gps = navigator.getGamepads?.() || [];
  const gp = gps[0];
  const t = performance.now();
  if (!gp){
    inputQ.push({t, lx:0, ly:0, rx:0, btnY:false, btnA:false});
    return;
  }
  const lx = db(-(gp.axes[0] ?? 0), profile.deadband);
  const ly = db(gp.axes[1] ?? 0, profile.deadband);

  // Different controllers expose the right stick X axis at different indices.
  // Pick the axis with the greatest magnitude from common locations so the
  // simulator reacts properly regardless of mapping.
  const cand = [gp.axes[2], gp.axes[3], gp.axes[4]];
  let rxRaw = 0;
  for (const val of cand){
    const v = val ?? 0;
    if (Math.abs(v) > Math.abs(rxRaw)) rxRaw = v;
  }
  const rx = db(rxRaw, profile.deadband);

  const btnY = !!(gp.buttons[3]?.pressed);
  const btnA = !!(gp.buttons[0]?.pressed);
  inputQ.push({t, lx, ly, rx, btnY, btnA});
}
function consumeInput(){
  const now = performance.now();
  const latency = (profile.latency_ms||0) + (profile.jitter_ms? (Math.random()*profile.jitter_ms - profile.jitter_ms/2) : 0);
  let inp = null; while (inputQ.length && now - inputQ[0].t >= latency) { inp = inputQ.shift(); }
  if (!inp && inputQ.length) inp = inputQ[0]; if (!inp) inp = {lx:0, ly:0, rx:0, btnY:false, btnA:false};
  if (inp.btnY && !lastY){ setControlMode(!fieldCentric); }
  lastY = inp.btnY;
  if (inp.btnA && !lastA){ resetCycle(); }
  lastA = inp.btnA;
  let vx =  inp.lx * lim.vFree;
  let vy = -inp.ly * lim.vFree;
  let om =  inp.rx * (lim.vFree / (Math.hypot(profile.wheelbase_m, profile.track_m)/2));
  if (!fieldCentric){ [vx, vy] = rotF(vx, vy, pose.th); }
  velCmd.vx = vx; velCmd.vy = vy; velCmd.om = om;
}
function step(dt){
  const [vrx, vry] = rotF(velCmd.vx, velCmd.vy, -pose.th);
  const L = profile.wheelbase_m, W = profile.track_m;
  const mods = [ {x:+L/2, y:+W/2}, {x:+L/2, y:-W/2}, {x:-L/2, y:+W/2}, {x:-L/2, y:-W/2} ];
  const desired = mods.map(m=>{
    const vxm = vrx - velCmd.om * m.y; const vym = vry + velCmd.om * m.x; const spd = Math.hypot(vxm, vym);
    const ang = Math.atan2(vym, vxm) || 0; return {x:m.x, y:m.y, vxm, vym, spd, ang};
  });
  const maxAz = lim.omAzFree;
  for (let i=0;i<4;i++){
    const target = desired[i].ang; let delta = wrapPI(target - moduleAngles[i]);
    const stepAng = clamp(delta, -maxAz*dt, maxAz*dt); moduleAngles[i] = wrapPI(moduleAngles[i] + stepAng);
  }
  const kV = 8.0, kOm = 6.0;
  let Fx_cmd = (velCmd.vx - velAct.vx) * kV * profile.mass_kg;
  let Fy_cmd = (velCmd.vy - velAct.vy) * kV * profile.mass_kg;
  const Fmag = Math.hypot(Fx_cmd, Fy_cmd);
  if (Fmag > lim.Fmax){ const s = lim.Fmax / (Fmag || 1e-6); Fx_cmd *= s; Fy_cmd *= s; }
  const rEff = Math.hypot(L/2, W/2);
  let Tz_cmd = (velCmd.om - velAct.om) * kOm * lim.Iz;
  const Tmax = lim.Fmax * rEff;
  Tz_cmd = clamp(Tz_cmd, -Tmax, Tmax);
  const ax = Fx_cmd / profile.mass_kg;
  const ay = Fy_cmd / profile.mass_kg;
  const alpha = Tz_cmd / lim.Iz;
  const dragV = 0.20, dragOm = 0.15;
  velAct.vx += (ax - dragV*velAct.vx) * dt;
  velAct.vy += (ay - dragV*velAct.vy) * dt;
  velAct.om += (alpha - dragOm*velAct.om) * dt;
  const vMag = Math.hypot(velAct.vx, velAct.vy);
  if (vMag < 0.02 && Math.hypot(velCmd.vx, velCmd.vy) === 0){ velAct.vx = 0; velAct.vy = 0; }
  if (Math.abs(velAct.om) < 0.02 && velCmd.om === 0){ velAct.om = 0; }
  if (vMag > lim.vFree){ const s = lim.vFree / vMag; velAct.vx*=s; velAct.vy*=s; }
  pose.x += velAct.vx * field.pxPerM * dt; pose.y += velAct.vy * field.pxPerM * dt; pose.th += velAct.om * dt;
  pose.x = clamp(pose.x, 60, fieldPxW-60); pose.y = clamp(pose.y, 60, fieldPxH-60);
  if (defenderOn){ const dx=pose.x-defender.x, dy=pose.y-defender.y, d=Math.hypot(dx,dy)||1e-6; const vmax=defender.vmax*field.pxPerM; defender.x+=dx/d*vmax*dt; defender.y+=dy/d*vmax*dt; }
  checkPickups(); if (recording){ recordBuf.push({t:performance.now(), x:pose.x, y:pose.y, th:pose.th}); }
}
function wrapPI(a){ while(a> Math.PI) a-=2*Math.PI; while(a<-Math.PI) a+=2*Math.PI; return a; }
function draw(){
  ctx.clearRect(0,0,canvas.width,canvas.height); ctx.save(); ctx.translate(origin.x, origin.y);
  ctx.fillStyle = '#111820'; ctx.fillRect(0,0,fieldPxW, fieldPxH);
  ctx.strokeStyle = '#1f2a36'; ctx.lineWidth = 2; ctx.strokeRect(0,0,fieldPxW, fieldPxH);
  ctx.strokeStyle = '#172432'; ctx.setLineDash([8,8]); ctx.beginPath(); ctx.moveTo(fieldPxW/2,0); ctx.lineTo(fieldPxW/2,fieldPxH); ctx.stroke(); ctx.setLineDash([]);
  ctx.strokeStyle = '#3fb950'; ctx.beginPath(); ctx.arc(scoreZone.x, scoreZone.y, scoreZone.r, 0, Math.PI*2); ctx.stroke();
  for (const p of pickupTargets){ ctx.fillStyle = p.taken ? '#2d3b47' : '#f0b429'; ctx.beginPath(); ctx.arc(p.x, p.y, 12, 0, Math.PI*2); ctx.fill(); }
  if (defenderOn){ ctx.fillStyle = '#b54747'; ctx.beginPath(); ctx.arc(defender.x, defender.y, defender.r, 0, Math.PI*2); ctx.fill(); }
  if (playing && playBuf.length){ const t = performance.now() - playT0; while (playIdx+1 < playBuf.length && (playBuf[playIdx+1].t - playBuf[0].t) <= t) playIdx++; ctx.strokeStyle = '#7b9acc'; ctx.lineWidth = 1.5; ctx.beginPath(); for(let i=1;i<=playIdx;i++){ const a=playBuf[i-1], b=playBuf[i]; ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);} ctx.stroke(); const gpt = playBuf[playIdx] || playBuf[playBuf.length - 1]; drawRobot(gpt.x, gpt.y, gpt.th, true); }
  drawRobot(pose.x, pose.y, pose.th, false);
  ctx.restore();
  drawOrientationWindow();
}
function drawOrientationWindow(){
  orientCtx.clearRect(0,0,orientCanvas.width, orientCanvas.height);
  orientCtx.save();
  orientCtx.translate(orientCanvas.width/2, orientCanvas.height/2);
  const scale = 40;
  const fac = scale / field.pxPerM;
  const track = profile.track_m, wheelbase = profile.wheelbase_m;
  const frameW = track * scale, frameL = wheelbase * scale;
  const bpx = BUMPER_M * scale;
  const outerW = frameW + 2*bpx, outerL = frameL + 2*bpx;
  orientCtx.rotate(pose.th);
  orientCtx.fillStyle = '#253140';
  roundRectPath(orientCtx, -frameL/2, -frameW/2, frameL, frameW, 10*fac); orientCtx.fill();
  orientCtx.fillStyle = '#7c2331';
  orientCtx.fillRect(-outerL/2, -outerW/2, outerL, bpx);
  orientCtx.fillRect(-outerL/2, outerW/2 - bpx, outerL, bpx);
  orientCtx.fillRect(-outerL/2, -outerW/2, bpx, outerW);
  orientCtx.fillRect(outerL/2 - bpx, -outerW/2, bpx, outerW);
  const wheelR = 12 * fac;
  const moduleHalf = 16 * fac;
  const corners = [
    {x:+frameL/2 - moduleHalf, y:-frameW/2 + moduleHalf},
    {x:+frameL/2 - moduleHalf, y:+frameW/2 - moduleHalf},
    {x:-frameL/2 + moduleHalf, y:-frameW/2 + moduleHalf},
    {x:-frameL/2 + moduleHalf, y:+frameW/2 - moduleHalf}
  ];
  for (let i=0;i<4;i++){
    const c = corners[i]; orientCtx.save(); orientCtx.translate(c.x, c.y);
    orientCtx.fillStyle = '#2b3a49'; roundRectPath(orientCtx, -16*fac,-16*fac,32*fac,32*fac,6*fac); orientCtx.fill();
    orientCtx.rotate(moduleAngles[i]);
    orientCtx.strokeStyle = '#9ecbff'; orientCtx.lineWidth = 2*fac;
    orientCtx.beginPath(); orientCtx.arc(0,0,wheelR,0,Math.PI*2); orientCtx.stroke();
    for(let k=0;k<6;k++){ const ang=k*Math.PI/3; orientCtx.beginPath(); orientCtx.moveTo(0,0); orientCtx.lineTo(wheelR*Math.cos(ang), wheelR*Math.sin(ang)); orientCtx.stroke(); }
    orientCtx.beginPath(); orientCtx.moveTo(0,0); orientCtx.lineTo(wheelR+10*fac,0); orientCtx.stroke();
    orientCtx.restore();
  }
  orientCtx.strokeStyle = '#58a6ff'; orientCtx.lineWidth = 2*fac; orientCtx.beginPath(); orientCtx.moveTo(0,0); orientCtx.lineTo(frameL/2,0); orientCtx.stroke();
  orientCtx.restore();
}
function drawRobot(x,y,th,ghost){
  const track = profile.track_m, wheelbase = profile.wheelbase_m;
  const frameW = track * field.pxPerM, frameL = wheelbase * field.pxPerM;
  const bpx = BUMPER_M * field.pxPerM;
  const outerW = frameW + 2*bpx, outerL = frameL + 2*bpx;
  const fac = field.pxPerM / 50;
  ctx.save(); ctx.translate(x,y); ctx.rotate(th);
  ctx.fillStyle = ghost ? '#253140aa' : '#253140';
  roundRectPath(ctx, -frameL/2, -frameW/2, frameL, frameW, 10*fac); ctx.fill();
  ctx.fillStyle = ghost ? '#7c2331aa' : '#7c2331';
  ctx.fillRect(-outerL/2, -outerW/2, outerL, bpx);
  ctx.fillRect(-outerL/2, outerW/2 - bpx, outerL, bpx);
  ctx.fillRect(-outerL/2, -outerW/2, bpx, outerW);
  ctx.fillRect(outerL/2 - bpx, -outerW/2, bpx, outerW);

  const moduleHalf = 16 * fac;
  const wheelR = 12 * fac;
  const corners = [
    {x:+frameL/2 - moduleHalf, y:-frameW/2 + moduleHalf},
    {x:+frameL/2 - moduleHalf, y:+frameW/2 - moduleHalf},
    {x:-frameL/2 + moduleHalf, y:-frameW/2 + moduleHalf},
    {x:-frameL/2 + moduleHalf, y:+frameW/2 - moduleHalf}
  ];
  for (let i=0;i<4;i++){
    const c = corners[i]; ctx.save(); ctx.translate(c.x, c.y);
    ctx.fillStyle = ghost ? '#2b3a49aa' : '#2b3a49';
    roundRectPath(ctx, -16*fac,-16*fac,32*fac,32*fac,6*fac); ctx.fill();
    ctx.rotate(moduleAngles[i]);
    ctx.strokeStyle = '#9ecbff'; ctx.lineWidth = 2*fac;
    ctx.beginPath(); ctx.arc(0,0,wheelR,0,Math.PI*2); ctx.stroke();
    for(let k=0;k<6;k++){ const ang=k*Math.PI/3; ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(wheelR*Math.cos(ang), wheelR*Math.sin(ang)); ctx.stroke(); }
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(wheelR+10*fac,0); ctx.stroke();
    ctx.restore();
  }

  const numFont = Math.min(bpx * 0.8, 18);
  ctx.fillStyle = '#ffffff';
  ctx.font = `bold ${numFont}px system-ui`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  const numOffset = outerW/2 - bpx/2;
  ctx.save(); ctx.translate(0, -numOffset); ctx.fillText('7190', 0, 0); ctx.restore();
  ctx.save(); ctx.rotate(Math.PI); ctx.translate(0, -numOffset); ctx.fillText('7190', 0, 0); ctx.restore();
  ctx.restore();
}
function roundRectPath(c,x,y,w,h,r){ c.beginPath(); c.moveTo(x+r,y); c.arcTo(x+w,y,x+w,y+h,r); c.arcTo(x+w,y+h,x,y+h,r); c.arcTo(x,y+h,x,y,r); c.arcTo(x,y,x+w,y,r); }
function spawnPickups(n){ const arr=[]; for(let i=0;i<n;i++){ arr.push({x: 80+Math.random()*(fieldPxW-160), y: 120+Math.random()*(fieldPxH-240), taken:false}); } return arr; }
function resetCycle(){
  pose.x = fieldPxW*0.25; pose.y = fieldPxH*0.75; pose.th = 0;
  velCmd={vx:0,vy:0,om:0}; velAct={vx:0,vy:0,om:0};
  moduleAngles=[0,0,0,0];
  carrying=false;
  pickupTargets=spawnPickups(5);
  scoreZone = { x: fieldPxW-140, y: 60, r: 50 };
  defender = { x: fieldPxW*0.7, y: fieldPxH*0.6, vmax:3.0, r:18 };
  cycleStart=performance.now(); pickupsDone=0; updateHUD();
  recording=false; recordBuf=[]; playing=false;
}
function updateHUD(){ const t=(performance.now()-cycleStart)/1000; hud.cycle.textContent = t.toFixed(2)+'s'; hud.pickups.textContent = pickupsDone; hud.spd.textContent = (Math.hypot(velAct.vx, velAct.vy)).toFixed(2); hud.om.textContent = (velAct.om).toFixed(2); }
function checkPickups(){ if (!carrying){ for(const p of pickupTargets){ if(!p.taken && dist(pose.x, pose.y, p.x, p.y) < 34){ p.taken=true; carrying=true; break; } } } else { if (dist(pose.x, pose.y, scoreZone.x, scoreZone.y) < scoreZone.r){ carrying=false; pickupsDone += 1; updateHUD(); } } }
function openCal(){ ui.calPanel.style.display='block'; }
function closeCal(){ ui.calPanel.style.display='none'; }
function applyCal(){
  const sprint = parseFloat(ui.sprint.value); const spin = parseFloat(ui.spin.value); const stopm = parseFloat(ui.stop.value); let changed=false;
  if (!isNaN(sprint) && sprint>0){ const vMeas = 5.0/sprint; const entry = gearChart[driveRatio.toFixed(2)]; const chartV = entry ? entry[motor] : lim.vFree; const scale = clamp(vMeas/chartV, 0.7, 1.2); profile.efficiency = clamp(profile.efficiency * scale, 0.1, 1.0); changed=true; }
  if (!isNaN(spin) && spin>0){ const omMeas=(2*Math.PI)/spin; const scale=clamp(omMeas/lim.omAzFree, 0.5, 1.5); lim.omAzFree*=scale; changed=true; }
  if (!isNaN(stopm) && stopm>0){ const v0=3.0; const aNeeded=v0*v0/(2*stopm); profile.mu = clamp((aNeeded / g) * 1.05, 0.6, 1.8); changed=true; }
  if (changed){ computeDerivedLimits(); toast('Calibration applied'); }
  closeCal();
}
ui.calBtn.onclick = ()=> openCal(); ui.closeCal.onclick = ()=> closeCal(); ui.applyCal.onclick = ()=> applyCal();
ui.defBtn.onclick = ()=> { defenderOn = !defenderOn; ui.defBtn.textContent = 'Defender: ' + (defenderOn?'On':'Off'); };
ui.recBtn.onclick = ()=> { if (!recording){ recordBuf=[]; recording=true; ui.recBtn.textContent='Stop Recording'; toast('Recording...'); } else { recording=false; localStorage.setItem('swerve_ghost', JSON.stringify(recordBuf)); ui.recBtn.textContent='Record Ghost'; toast('Saved ghost'); } };
ui.playBtn.onclick = ()=> { const data = localStorage.getItem('swerve_ghost'); if (!data){ toast('No ghost saved'); return; } playBuf = JSON.parse(data); playIdx=0; playT0=performance.now(); playing=true; toast('Playing ghost'); };
ui.profileSel.onchange = (e)=> setProfile(e.target.value);
ui.motorSel.onchange = (e)=> setMotor(e.target.value);
ui.ratioSel.onchange = (e)=> setRatio(e.target.value);
ui.modeSel.onchange = (e)=> setControlMode(e.target.value === 'field');
window.addEventListener('resize', resizeSim);
function init(){ loadUI(); resizeSim(); resetCycle(); loop(); }
let last = performance.now();
function loop(){ pollGamepad(); consumeInput(); const now=performance.now(); const dt = Math.min(0.04, (now-last)/1000); last = now; step(dt); draw(); updateHUD(); requestAnimationFrame(loop); }
init();
(function(){
  try{
    console.assert(wrapPI(4*Math.PI) === 0, 'wrapPI 4π');
    const r = wrapPI(-3*Math.PI/2); console.assert(r>=-Math.PI && r<=Math.PI, 'wrapPI range');
    console.assert(lim && lim.vFree>0 && lim.omAzFree>0, 'limits');
    step(0);
    console.assert(Number.isFinite(pose.x) && Number.isFinite(pose.y) && Number.isFinite(pose.th), 'pose finite');
    console.log('Self-tests passed');
  }catch(e){ console.error('Self-tests failed', e); }
})();

