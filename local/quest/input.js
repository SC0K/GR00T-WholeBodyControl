'use strict';
// WebXR xr-standard: trigger 0, squeeze 1, thumbstick button 3, axes 2/3.
const QuestInput = (() => {
  function pose(p) {
    const {position: v, orientation: q} = p.transform;
    return [v.x, v.y, v.z, q.x, q.y, q.z, q.w];
  }
  function button(gamepad, index) {
    if (gamepad?.mapping !== 'xr-standard') return 0;
    const value = gamepad.buttons[index]?.value;
    return Number.isFinite(value) ? Math.max(0, Math.min(1, value)) : 0;
  }
  function axis(gamepad, index) {
    if (gamepad?.mapping !== 'xr-standard') return 0;
    const value = gamepad.axes[index];
    return Number.isFinite(value) ? Math.max(-1, Math.min(1, value)) : 0;
  }
  function selected(source, mode) {
    return ['left', 'right'].includes(source.handedness) && (mode === 'controllers'
      ? !source.hand && source.targetRayMode === 'tracked-pointer' && !!source.gripSpace
      : !!source.hand);
  }
  function read(frame, reference, sources, mode) {
    const hands = {}, pinch = {}, sticks = {};
    for (const source of sources) {
      const side = source.handedness;
      if (!['left', 'right'].includes(side)) continue;
      if (mode === 'controllers') {
        if (source.hand || source.targetRayMode !== 'tracked-pointer' || !source.gripSpace) continue;
        const grip = frame.getPose(source.gripSpace, reference);
        if (!grip || grip.emulatedPosition) continue;
        hands[side] = pose(grip);
        pinch[side] = button(source.gamepad, 0);
        sticks[side] = {x: axis(source.gamepad, 2), y: axis(source.gamepad, 3),
                        clicked: button(source.gamepad, 3) > .8};
      } else {
        if (!source.hand) continue;
        const wrist = frame.getJointPose(source.hand.get('wrist'), reference);
        if (!wrist) continue;
        hands[side] = pose(wrist);
        const thumb = frame.getJointPose(source.hand.get('thumb-tip'), reference);
        const index = frame.getJointPose(source.hand.get('index-finger-tip'), reference);
        pinch[side] = 0;
        if (thumb && index) {
          const a = thumb.transform.position, b = index.transform.position;
          pinch[side] = Math.min(1, Math.max(0, (.065 - Math.hypot(a.x-b.x, a.y-b.y, a.z-b.z)) / .045));
        }
      }
    }
    return {hands, pinch, sticks,
      chord: !!(sticks.left?.clicked && sticks.right?.clicked),
      drive: [-(sticks.left?.y || 0), -(sticks.left?.x || 0), -(sticks.right?.x || 0)]};
  }
  class Hold {
    constructor() { this.reset(); }
    reset() { this.since = null; this.fired = false; this.blocked = false; }
    disarm() { this.reset(); this.blocked = true; }
    update(pressed, now, released = !pressed) {
      if (!pressed) {
        if (!this.blocked || released) this.reset();
        return false;
      }
      if (this.blocked) return false;
      if (this.since === null) this.since = now;
      if (!this.fired && now - this.since >= 1000) { this.fired = true; return true; }
      return false;
    }
  }
  return {read, pose, Hold, selected};
})();
if (typeof module !== 'undefined') module.exports = QuestInput;
