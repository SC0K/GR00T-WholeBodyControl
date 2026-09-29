'use strict';
const assert=require('node:assert/strict');
const {Stats,lower}=require('./body-stats.js');
const stats=new Stats(); stats.add({body:null},{});
const body=new Map(lower.map(name=>[name,{name}]));
function frame(offset=0) { return {body,getPose(space) {
  if(space.name==='right-foot-ball') return null;
  const x=space.name==='left-foot-ankle'?offset:0;
  return {transform:{position:{x,y:space.name==='hips'?1:0,z:0},orientation:{x:0,y:0,z:0,w:1}},emulatedPosition:space.name!=='hips'};
}}; }
stats.add(frame(),{}); stats.add(frame(.2),{});
const report=stats.report();
assert.equal(report.frames,3); assert.equal(report.body_frames,2);
assert.deepEqual(report.missing_lower_body,['right-foot-ball']);
assert.equal(report.joints.hips.emulated_frames,0);
assert.equal(report.joints['left-foot-ankle'].emulated_frames,2);
assert.deepEqual(report.joints['left-foot-ankle'].relative_to_hips_range_m,[.2,0,0]);
assert.equal(report.joints['right-foot-ball'].last_pose_xyzw,null);
assert.doesNotThrow(()=>JSON.parse(JSON.stringify(report)));
assert.deepEqual(new Stats().report().missing_lower_body,lower);
const fs=require('node:fs');
const page=fs.readFileSync(__dirname+'/body-check.js','utf8');
assert.ok(page.startsWith(fs.readFileSync(__dirname+'/body-stats.js','utf8')));
assert.ok(!page.includes('fetch(') && !page.includes('WebSocket'));
console.log('PASS: joint availability, emulation flags, relative movement, bundled browser-only capture');
