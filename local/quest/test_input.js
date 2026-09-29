'use strict';
const assert = require('node:assert/strict');
const {read, Hold, selected} = require('./input.js');
const pose = {transform:{position:{x:.2,y:1,z:-.3}, orientation:{x:0,y:0,z:0,w:1}}};
function source(side) {
  return {handedness:side, targetRayMode:'tracked-pointer', gripSpace:{side},
    gamepad:{mapping:'xr-standard', buttons:[{value:.7},{value:0},{value:0},{value:1}], axes:[0,0,.2,-.3]}};
}
const sources = [source('left'), source('right')];
let calls = 0;
const frame = {getPose(space) { assert.ok(sources.some(s=>s.gripSpace===space)); calls++; return pose; }};
let input = read(frame, {}, sources, 'controllers');
assert.equal(calls, 2); assert.deepEqual(input.hands.left,[.2,1,-.3,0,0,0,1]);
assert.deepEqual(input.drive,[.3,-.2,-.2]);
assert.equal(input.pinch.right,.7); assert.equal(input.chord,true);
input = read({getPose:()=>({...pose,emulatedPosition:true})}, {}, sources, 'controllers');
assert.equal(input.hands.left,undefined);
input = read({getPose:()=>null}, {}, sources, 'controllers');
assert.equal(input.hands.right,undefined);
input = read(frame, {}, sources, 'hands');
assert.deepEqual(input.hands,{});
sources[0].gamepad.mapping='';
input = read(frame, {}, sources, 'controllers');
assert.equal(input.pinch.left,0); assert.equal(input.chord,false);
const hand = {handedness:'left',hand:new Map(['wrist','thumb-tip','index-finger-tip'].map(x=>[x,x]))};
input = read({getJointPose:x=>x==='wrist'?pose:null}, {}, [hand], 'hands');
assert.deepEqual(input.hands.left,[.2,1,-.3,0,0,0,1]); assert.equal(input.pinch.left,0);
input = read(frame, {}, [hand], 'controllers'); assert.deepEqual(input.hands,{});
const hold = new Hold();
assert.equal(hold.update(true,0),false); assert.equal(hold.update(true,999),false);
assert.equal(hold.update(true,1000),true); assert.equal(hold.update(true,3000),false);
assert.equal(hold.update(false,3001),false); assert.equal(hold.update(true,4000),false);
assert.equal(hold.update(true,5000),true);
console.log('PASS: controller grips, triggers, missing/emulated tracking, explicit input selection, hands, one-shot hold');

hold.disarm();
assert.equal(hold.update(true,6000),false);
assert.equal(hold.update(true,9000),false);
assert.equal(hold.update(false,9001),false);
assert.equal(hold.update(true,10000),false);
assert.equal(hold.update(true,11000),true);
assert.equal(selected(hand,'controllers'),false);
assert.equal(selected(sources[0],'controllers'),true);
assert.equal(selected({handedness:'none'},'controllers'),false);
console.log('PASS: walking axes, unrelated input filtering, fresh release required after loss');

hold.disarm();
assert.equal(hold.update(false,12000,false),false); // One button is still held.
assert.equal(hold.update(true,15000),false);
assert.equal(hold.update(false,15001,true),false);
assert.equal(hold.update(true,16000),false);
assert.equal(hold.update(true,17000),true);
