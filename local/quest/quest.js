'use strict';
const status = document.querySelector('#status');
const canvas = document.querySelector('#canvas');
let socket, session, seq = 0, lastSend = 0, gl;
let bridgeState = {}, inputMode = 'controllers', locomotion = 'sticks';
const hold = new QuestInput.Hold();
let retryDelay = 1000;
function connect() {
  const ws = new WebSocket(`wss://${location.host}/ws`);
  socket = ws;
  ws.onopen = () => {
    retryDelay = 1000;
    bridgeState = {};
    hold.disarm();
    status.textContent = 'Connected. Robot remains paused after a loss. Release both thumbsticks, then hold both clicks for 1 second to recalibrate.';
  };
  ws.onmessage = e => {
    const s = JSON.parse(e.data);
    bridgeState = s;
    status.textContent = `${s.status}\nTracking: ${s.tracking ? 'head + both inputs' : 'waiting'} · Frames: ${s.frames}`;
  };
  ws.onclose = () => {
    if (socket !== ws) return;
    hold.disarm();
    bridgeState = {};
    status.textContent = 'Connection lost. Reconnecting; teleop will remain paused. The running bridge returns the robot to standing.';
    setTimeout(() => { if (socket === ws) connect(); }, retryDelay);
    retryDelay = Math.min(5000, retryDelay * 2);
  };
  ws.onerror = () => { status.textContent = 'Connection unavailable. Retrying; check the PC bridge terminal.'; };
}
function lost(reason='Tracking unavailable') {
  hold.disarm();
  if (socket.readyState === WebSocket.OPEN && socket.bufferedAmount === 0)
    socket.send(JSON.stringify({type:'frame',tracked:false,seq:++seq,diagnostics:{reason:String(reason)}}));
}
async function enter(mode) {
  if (!navigator.xr) throw new Error('WebXR unavailable. Use Quest Browser over HTTPS.');
  if (session) return;
  if (socket.readyState !== WebSocket.OPEN) throw new Error('Bridge is not connected.');
  if (!await navigator.xr.isSessionSupported(mode)) throw new Error(`${mode} is not supported by this browser.`);
  hold.reset();
  session = await navigator.xr.requestSession(mode, {requiredFeatures:['local-floor']});
  try {
    gl = canvas.getContext('webgl', {xrCompatible:true,alpha:true});
    if (!gl) throw new Error('WebGL unavailable');
    await gl.makeXRCompatible();
    session.updateRenderState({baseLayer:new XRWebGLLayer(session,gl,{alpha:true})});
    const reference = await session.requestReferenceSpace('local-floor');
    reference.addEventListener('reset', () => lost('Reference space reset'));
    session.addEventListener('visibilitychange', () => { if(session.visibilityState !== 'visible') lost(`XR focus lost: ${session.visibilityState} (system menu, browser overlay, or headset visibility)`); });
    session.addEventListener('inputsourceschange', e => {
      if (Array.from(e.removed).some(source => QuestInput.selected(source, inputMode)))
        lost('Selected controller/input removed; recalibrate');
    });
    session.addEventListener('end', () => {lost('XR session ended; re-enter passthrough/VR'); session=null;});
    function frame(t, f) {
      const s = f.session;
      s.requestAnimationFrame(frame);
      gl.bindFramebuffer(gl.FRAMEBUFFER,s.renderState.baseLayer.framebuffer);
      // Transparent AR layer preserves passthrough instead of covering the room.
      gl.clearColor(0,0,0,mode==='immersive-ar'?0:1);
      gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
      if (socket.readyState !== WebSocket.OPEN || socket.bufferedAmount > 0) {
        hold.disarm();
        return;
      }
      if(t-lastSend<20) return;
      lastSend=t;
      const head = f.getViewerPose(reference);
      const input = QuestInput.read(f, reference, s.inputSources, inputMode);
      const {hands, pinch} = input;
      const tracked = !!(head && hands.left && hands.right && s.visibilityState === 'visible');
      socket.send(JSON.stringify({type:'frame', seq:++seq, tracked, input_mode:inputMode, locomotion, sample_time:t/1000,
        diagnostics:{visibility:s.visibilityState, head:!!head, left:!!hands.left, right:!!hands.right, input_mode:inputMode},
        ...(tracked ? {head:QuestInput.pose(head), left:hands.left, right:hands.right,
          pinch:[pinch.left,pinch.right], drive:input.chord ? [0,0,0] : input.drive} : {})}));
      // One deliberate chord per release. Starting always uses the server's
      // standing-first readiness checks and a fresh calibration.
      if (!tracked) hold.disarm();
      else if (hold.update(input.chord, t, !input.sticks.left?.clicked && !input.sticks.right?.clicked))
        socket.send(JSON.stringify({type:'command', command:'toggle'}));
    }
    session.requestAnimationFrame(frame);
  } catch(e) { await session.end(); session=null; throw e; }
}
for(const [id,mode] of [['enter','immersive-ar'],['vr','immersive-vr']])
  document.getElementById(id).onclick=()=>enter(mode).catch(e=>{status.textContent=e.message;lost(e.message);});
connect();

window.addEventListener('error', e => lost(`Browser error: ${e.message}`));
window.addEventListener('unhandledrejection', e => lost(`Browser error: ${String(e.reason)}`));
