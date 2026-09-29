'use strict';
const BodyStats = (() => {
  const lower = ['hips','left-upper-leg','left-lower-leg','left-foot-ankle','left-foot-ball',
    'right-upper-leg','right-lower-leg','right-foot-ankle','right-foot-ball'];
  function rangeUpdate(range, point) {
    for (let i=0;i<3;i++) { range.min[i]=Math.min(range.min[i],point[i]); range.max[i]=Math.max(range.max[i],point[i]); }
  }
  function extent(range) { return range.min.map((x,i)=>Number.isFinite(x)?+(range.max[i]-x).toFixed(4):null); }
  const range = () => ({min:[Infinity,Infinity,Infinity],max:[-Infinity,-Infinity,-Infinity]});
  class Stats {
    constructor() { this.frames=0; this.bodyFrames=0; this.joints=new Map(); this.errors=[]; }
    add(frame, reference) {
      this.frames++;
      if (!frame.body) return;
      this.bodyFrames++;
      const current=new Map();
      for (const [name, space] of frame.body) {
        let entry=this.joints.get(name);
        if (!entry) { entry={valid:0,emulated:0,unknown:0,position:range(),relative:range(),last:null}; this.joints.set(name,entry); }
        let p;
        try { p=frame.getPose(space,reference); }
        catch(e) { if(this.errors.length<3) this.errors.push(String(e)); continue; }
        if (!p) continue;
        const v=p.transform.position,q=p.transform.orientation;
        const sample=[v.x,v.y,v.z,q.x,q.y,q.z,q.w];
        if(!sample.every(Number.isFinite)) continue;
        entry.valid++;
        if(p.emulatedPosition===true) entry.emulated++;
        if(typeof p.emulatedPosition!=='boolean') entry.unknown++;
        rangeUpdate(entry.position,sample.slice(0,3));
        entry.last=sample; current.set(name,sample);
      }
      const hips=current.get('hips');
      if (hips) for(const [name,p] of current) rangeUpdate(this.joints.get(name).relative,p.slice(0,3).map((x,i)=>x-hips[i]));
    }
    report() {
      const joints={};
      for (const [name,j] of this.joints) joints[name]={valid_frames:j.valid,
        valid_fraction:this.frames?+(j.valid/this.frames).toFixed(3):0,
        emulated_frames:j.emulated, emulation_unknown_frames:j.unknown,
        position_range_m:extent(j.position), relative_to_hips_range_m:extent(j.relative), last_pose_xyzw:j.last};
      return {version:2,frames:this.frames,body_frames:this.bodyFrames,joint_count:this.joints.size,
        missing_lower_body:lower.filter(name=>!joints[name]?.valid_frames),joints,errors:this.errors,
        interpretation:'Joint availability and movement ranges only. Estimated joints may mimic motion without observing your legs. Full-body robot control is not enabled.'};
    }
  }
  return {Stats,lower};
})();
if(typeof module!=='undefined') module.exports=BodyStats;

'use strict';
const output=document.querySelector('#result');
let checking=false;
async function check(mode) {
  if(checking) return;
  checking=true;
  const stats=new BodyStats.Stats();
  let session,timer,canvas;
  const report={mode,browser:navigator.userAgent,checked_at:new Date().toISOString(),feature_granted:false};
  try {
    if(!navigator.xr) throw new Error('WebXR unavailable; use the headset browser over HTTPS.');
    if(!await navigator.xr.isSessionSupported(mode)) throw new Error(`${mode} unavailable; try the other button.`);
    session=await navigator.xr.requestSession(mode,{requiredFeatures:['local-floor','body-tracking']});
    report.feature_granted=true;
    report.enabled_features=session.enabledFeatures?Array.from(session.enabledFeatures):null;
    canvas=document.createElement('canvas');
    const gl=canvas.getContext('webgl',{xrCompatible:true,alpha:true});
    if(!gl) throw new Error('WebGL unavailable');
    await gl.makeXRCompatible();
    session.updateRenderState({baseLayer:new XRWebGLLayer(session,gl,{alpha:true})});
    const reference=await session.requestReferenceSpace('local-floor');
    await new Promise((resolve,reject)=>{
      let start=null,lastSample=-Infinity,done=false;
      function finish(error) { if(done)return; done=true; clearTimeout(timer); error?reject(error):resolve(); }
      session.addEventListener('end',()=>finish(),{once:true});
      timer=setTimeout(()=>finish(new Error('Capture timed out or XR stopped producing frames')),22000);
      function frame(t,f) {
        if(done)return;
        try {
          if(start===null) start=t;
          gl.bindFramebuffer(gl.FRAMEBUFFER,session.renderState.baseLayer.framebuffer);
          gl.clearColor(0,0,0,mode==='immersive-ar'?0:1); gl.clear(gl.COLOR_BUFFER_BIT);
          if(t-lastSample>=50 && session.visibilityState==='visible') { stats.add(f,reference); lastSample=t; }
          if(t-start>=15000) { finish(); return; }
          session.requestAnimationFrame(frame);
        } catch(e) { finish(e); }
      }
      session.requestAnimationFrame(frame);
    });
  } catch(e) { report.error=`${e.name}: ${e.message}`; }
  finally {
    clearTimeout(timer);
    if(session) { try { await session.end(); } catch(_) {} }
    Object.assign(report,stats.report());
    const lines=[`Feature granted: ${report.feature_granted}`, `Frames sampled: ${report.frames}`, `Joints exposed: ${report.joint_count}`,
      `Missing lower-body poses: ${report.missing_lower_body.join(', ') || 'none'}`];
    for(const name of BodyStats.lower) {
      const j=report.joints[name];
      lines.push(j ? `${name}: valid ${(100*j.valid_fraction).toFixed(0)}%, estimated ${j.emulated_frames}/${j.valid_frames}, movement relative to hips ${j.relative_to_hips_range_m.join(', ')} m` : `${name}: unavailable`);
    }
    if(report.error) lines.push(report.error);
    lines.push('Availability does not prove accurate leg tracking. Full-body control is not enabled.');
    output.textContent=lines.join('\n')+'\n\nDetailed report (stays in browser):\n'+JSON.stringify(report,null,2);
    checking=false;
  }
}
document.querySelector('#ar').onclick=()=>check('immersive-ar');
document.querySelector('#vr').onclick=()=>check('immersive-vr');
