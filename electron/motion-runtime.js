// Use Motion's browser bundle: the ESM entry imports bare package specifiers,
// which cannot be resolved by an unbundled Electron file:// renderer.
window.motionRuntime = window.Motion?.animate
  ? {
      animate: window.Motion.animate,
      spring: window.Motion.spring,
      stagger: window.Motion.stagger,
    }
  : null;
window.motionRuntimeReady = Promise.resolve(window.motionRuntime);
