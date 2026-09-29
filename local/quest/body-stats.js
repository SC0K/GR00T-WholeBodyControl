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
