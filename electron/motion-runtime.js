window.motionRuntime = null;
window.motionRuntimeReady = import("./node_modules/motion/dist/es/index.mjs")
  .then((module) => {
    window.motionRuntime = {
      animate: module.animate,
      spring: module.spring,
      stagger: module.stagger,
    };
    return window.motionRuntime;
  })
  .catch((error) => {
    console.warn("Motion runtime unavailable.", error);
    window.motionRuntime = null;
    return null;
  });
