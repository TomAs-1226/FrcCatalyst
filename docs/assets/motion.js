/**
 * Catalyst motion — the springs the identity moves with.
 *
 * One file, copied verbatim into CatalystApp and Catalyst Console. CSS covers
 * most of it: `identity.css` carries each role as a `--cat-ease-*` curve and a
 * `--cat-dur-*` duration, and a transition that names a role is the right way
 * to move something. This module is for the two things CSS cannot do:
 *
 *   1. carry a gesture's release velocity into the spring that finishes it, and
 *   2. answer a press at the point it was pressed.
 *
 * The specs are Detent presets, named by role, and the gesture constants are
 * the platform's own (UIScrollView's deceleration, iOS's rubber band, WWDC18's
 * projection). Nothing here is tuned by eye.
 */

/** Spring specs by role, in SwiftUI's response/damping vocabulary. */
export const ROLE = {
  hold: { response: 0.15, damping: 0.86 },    // a value following the pointer
  release: { response: 0.5 / 1.36, damping: 0.85 }, // something arriving (snappy: bounce 0.15)
  smooth: { response: 0.5 / 1.36, damping: 1 },     // something leaving; never bounces
  settle: { response: 0.4, damping: 1 },      // a value travelling to where it was set
  detent: { response: 0.4, damping: 0.8 },    // landing on a detent
};

/** Gesture constants, from the platforms that defined them. */
export const GESTURE = {
  hysteresis: 10,        // px before a touch is a drag (iOS)
  sliderHysteresis: 3,   // a slider claims the touch at once
  decelerationRate: 0.998,   // UIScrollView's coast
  projectionRate: 0.99,      // WWDC18's "where will it stop" for snap targets
  rubberBand: 0.55,          // iOS past an edge
  maxHandoff: 5000,          // px/s handed to a spring, capped
};

/**
 * Where a flick ends up, so a snap target is chosen before the motion starts
 * rather than after it stops.
 *
 * @param {number} velocity px/s at release
 * @param {number} rate deceleration rate; GESTURE.projectionRate for snapping
 */
export function project(velocity, rate = GESTURE.projectionRate) {
  return (velocity / 1000) * (rate / (1 - rate));
}

/**
 * How far past an edge a drag is allowed to pull. Resistance grows with the
 * distance, so the edge is felt rather than enforced.
 */
export function rubberBand(offset, dimension, c = GESTURE.rubberBand) {
  if (dimension <= 0) return 0;
  const sign = Math.sign(offset);
  const x = Math.abs(offset);
  return sign * ((1 - 1 / (x * c / dimension + 1)) * dimension);
}

/**
 * Run a spring from `from` to `to`, calling `onFrame` each frame and `onRest`
 * once it has settled. Returns a function that stops it.
 *
 * Integrated rather than sampled, because a gesture hands it a velocity and a
 * sampled curve cannot start in the middle of a movement.
 */
export function spring({ from, to, velocity = 0, role = 'release', onFrame, onRest }) {
  const spec = ROLE[role] || ROLE.release;
  const w = (2 * Math.PI) / spec.response;   // natural frequency
  const z = spec.damping;
  let x = from;
  let v = Math.max(-GESTURE.maxHandoff, Math.min(GESTURE.maxHandoff, velocity));
  let raf = 0;
  let last = 0;
  let stopped = false;

  // Settled means both "close enough" and "slow enough": a spring that is on
  // its target at speed is about to leave it again.
  const restDistance = Math.max(1e-3, Math.abs(to - from) * 1e-3);
  const restVelocity = restDistance * 10;

  const step = (now) => {
    if (stopped) return;
    if (!last) last = now;
    // Clamp the step: a backgrounded window hands back a one-second frame, and
    // an explicit integrator handed a one-second step throws the value away.
    const dt = Math.min(0.064, (now - last) / 1000);
    last = now;

    // Semi-implicit Euler, substepped so a stiff spring stays stable.
    const steps = Math.max(1, Math.ceil(dt / 0.004));
    const h = dt / steps;
    for (let i = 0; i < steps; i++) {
      const a = -(w * w) * (x - to) - 2 * z * w * v;
      v += a * h;
      x += v * h;
    }

    if (Math.abs(x - to) < restDistance && Math.abs(v) < restVelocity) {
      x = to;
      onFrame?.(x, 0);
      onRest?.();
      return;
    }
    onFrame?.(x, v);
    raf = requestAnimationFrame(step);
  };

  raf = requestAnimationFrame(step);
  return () => { stopped = true; cancelAnimationFrame(raf); };
}

/**
 * A least-squares velocity over the last 100 ms, which is what Compose's
 * tracker does: the last two points alone report noise as speed.
 */
export function velocityTracker(window = 100) {
  const points = [];
  return {
    add(value, time = performance.now()) {
      points.push({ value, time });
      while (points.length && time - points[0].time > window) points.shift();
    },
    velocity() {
      if (points.length < 2) return 0;
      const t0 = points[0].time;
      let sx = 0, sy = 0, sxx = 0, sxy = 0;
      for (const p of points) {
        const x = (p.time - t0) / 1000;
        sx += x; sy += p.value; sxx += x * x; sxy += x * p.value;
      }
      const n = points.length;
      const d = n * sxx - sx * sx;
      return d === 0 ? 0 : (n * sxy - sx * sy) / d;
    },
    clear() { points.length = 0; },
  };
}

/**
 * Bezel's press answer: a state layer that grows from where the pointer went
 * down. The element needs `position: relative` and `overflow: hidden`; the
 * layer removes itself.
 */
export function stateLayer(el, event, { color = 'currentColor', opacity = 0.1 } = {}) {
  if (window.matchMedia?.('(prefers-reduced-motion: reduce)').matches) return;
  const r = el.getBoundingClientRect();
  const x = (event?.clientX ?? r.left + r.width / 2) - r.left;
  const y = (event?.clientY ?? r.top + r.height / 2) - r.top;
  const reach = Math.max(
    Math.hypot(x, y),
    Math.hypot(r.width - x, y),
    Math.hypot(x, r.height - y),
    Math.hypot(r.width - x, r.height - y),
  );

  const layer = document.createElement('span');
  layer.setAttribute('aria-hidden', 'true');
  Object.assign(layer.style, {
    position: 'absolute',
    left: `${x}px`,
    top: `${y}px`,
    width: '0px',
    height: '0px',
    borderRadius: '999px',
    background: color,
    opacity: String(opacity),
    transform: 'translate(-50%, -50%)',
    pointerEvents: 'none',
  });
  el.appendChild(layer);

  const style = getComputedStyle(document.documentElement);
  const dur = style.getPropertyValue('--cat-dur-layer').trim() || '0.155s';
  const ease = style.getPropertyValue('--cat-ease-layer').trim() || 'linear';
  layer.animate(
    [
      { width: '0px', height: '0px', opacity },
      { width: `${reach * 2}px`, height: `${reach * 2}px`, opacity: 0 },
    ],
    { duration: parseFloat(dur) * 1000, easing: ease, fill: 'forwards' },
  ).finished.catch(() => {}).finally(() => layer.remove());
}

/** The CSS transition for a role, for the rare case one is built in script. */
export function transition(property, role = 'release') {
  const style = getComputedStyle(document.documentElement);
  const dur = style.getPropertyValue(`--cat-dur-${role}`).trim() || '0.3s';
  const ease = style.getPropertyValue(`--cat-ease-${role}`).trim() || 'linear';
  return `${property} ${dur} ${ease}`;
}
