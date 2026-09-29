'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const QuestInput = require('./input.js');
const sockets = [], timers = [];
class Socket {
  static OPEN = 1;
  constructor() { this.readyState=0; sockets.push(this); this.sent=[]; }
  send(message) { this.sent.push(JSON.parse(message)); }
}
const status = {}, elements = {};
const sandbox = {QuestInput, WebSocket:Socket, location:{host:'test:8013'},
  document:{querySelector:id=>id==='#status'?status:{},getElementById:id=>elements[id]??={}},
  window:{addEventListener(){}}, setTimeout(fn,ms){timers.push({fn,ms});}};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(__dirname+'/quest.js','utf8'),sandbox);
assert.equal(sockets.length,1);
sockets[0].readyState=1; sockets[0].onopen();
vm.runInContext('session={end(){throw new Error("XR must remain open during network loss")}}',sandbox);
sockets[0].readyState=3; sockets[0].onclose();
assert.equal(timers.length,1); assert.equal(timers[0].ms,1000);
timers.shift().fn(); assert.equal(sockets.length,2);
sockets[1].readyState=1; sockets[1].onopen();
assert.equal(sockets[1].sent.length,0); // Reconnection never sends go/start.
assert.equal(vm.runInContext('hold.update(true,1000)',sandbox),false);
assert.equal(vm.runInContext('hold.update(true,5000)',sandbox),false);
assert.equal(vm.runInContext('hold.update(false,5001)',sandbox),false);
assert.equal(vm.runInContext('hold.update(true,6000)',sandbox),false);
assert.equal(vm.runInContext('hold.update(true,7000)',sandbox),true);
console.log('PASS: reconnect keeps XR alive, never auto-starts, requires new button release/hold');
