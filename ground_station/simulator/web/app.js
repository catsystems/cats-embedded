const canvas = document.querySelector('#display');
const ctx = canvas.getContext('2d', { alpha: false });
ctx.imageSmoothingEnabled = false;
const snapshotNode = document.querySelector('#snapshot');
const statusNode = document.querySelector('#status');
let paused = false;
let held = new Set();
let state = { activeScreen: 'menu', virtualTimeMs: 0, framebufferRevision: 0 };
let loadError = null;
let lastRealtimeMs = performance.now();
let realtimeRemainderMs = 0;

// The browser is only a host: the controller API owns state, actions, timing,
// and framebuffer bytes. A generated Emscripten module can be dropped beside
// this file without changing the controls below.
let wasm = null;
async function loadController() {
  try {
    const moduleFactory = (await import('./gs-sim.js')).default;
    wasm = await moduleFactory();
  } catch (error) {
    wasm = null;
    loadError = error instanceof Error ? error.message : String(error);
    console.error('Ground Station simulator WebAssembly failed to load:', error);
  }
  if (wasm) {
    loadError = null;
    wasm.ccall('gs_reset', null, [], []);
  }
  render();
  canvas.focus({ preventScroll: true });
}

function controllerCall(name, args = []) {
  if (!wasm) return;
  wasm.ccall(name, null, args.map(() => 'number'), args);
}

function refreshSnapshot() {
  if (wasm) {
    state = JSON.parse(wasm.ccall('gs_snapshot_json', 'string', [], []));
    statusNode.textContent = 'WebAssembly simulator ready.';
  } else {
    state = {
      activeScreen: 'unavailable',
      virtualTimeMs: 0,
      framebufferRevision: 0,
      error: loadError || 'gs-sim.js is missing. Run gs-sim.ps1 setup, then serve again.'
    };
    statusNode.textContent = `Simulator unavailable: ${state.error}`;
  }
  snapshotNode.textContent = JSON.stringify(state, null, 2);
}

function render() {
  refreshSnapshot();
  const image = ctx.createImageData(400, 240);
  for (let i = 0; i < image.data.length; i += 4) {
    image.data[i] = image.data[i + 1] = image.data[i + 2] = 255;
    image.data[i + 3] = 255;
  }
  if (wasm) {
    const size = wasm.ccall('gs_framebuffer_size', 'number', [], []);
    const ptr = wasm.ccall('gs_framebuffer', 'number', [], []);
    const bytes = wasm.HEAPU8.slice(ptr, ptr + size);
    for (let p = 0; p < 400 * 240; p++) {
      const white = (bytes[p >> 3] & (0x80 >> (p & 7))) !== 0;
      const value = white ? 255 : 0;
      image.data[p * 4] = image.data[p * 4 + 1] = image.data[p * 4 + 2] = value;
    }
  }
  ctx.putImageData(image, 0, 0);
  if (!wasm) {
    ctx.fillStyle = '#000';
    ctx.font = '14px monospace';
    ctx.fillText('WebAssembly simulator unavailable', 12, 28);
    ctx.fillText('Run gs-sim.ps1 setup, then serve again.', 12, 50);
  }
}

function press(button) {
  if (held.has(button)) return;
  held.add(button);
  if (wasm) wasm.ccall('gs_press', null, ['string'], [button]);
  render();
}
function release(button) {
  held.delete(button);
  if (wasm) wasm.ccall('gs_release', null, ['string'], [button]);
  render();
}

document.querySelectorAll('[data-button]').forEach(button => {
  const name = button.dataset.button;
  button.addEventListener('pointerdown', event => { event.preventDefault(); press(name); });
  button.addEventListener('pointerup', event => { event.preventDefault(); release(name); });
  button.addEventListener('pointerleave', () => release(name));
});
document.addEventListener('keydown', event => {
  const map = { ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right', Space: 'ok', Enter: 'ok', KeyA: 'ok', Escape: 'back', KeyB: 'back' };
  if (map[event.code]) { event.preventDefault(); press(map[event.code]); }
});
document.addEventListener('keyup', event => {
  const map = { ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right', Space: 'ok', Enter: 'ok', KeyA: 'ok', Escape: 'back', KeyB: 'back' };
  if (map[event.code]) { event.preventDefault(); release(map[event.code]); }
});
document.querySelector('#pause').addEventListener('click', event => {
  paused = !paused;
  event.target.textContent = paused ? 'Resume time' : 'Pause time';
  canvas.focus({ preventScroll: true });
});
document.querySelector('#reset').addEventListener('click', () => {
  if (wasm) controllerCall('gs_reset');
  render();
  canvas.focus({ preventScroll: true });
});
document.querySelector('#advance').addEventListener('click', () => {
  if (wasm) controllerCall('gs_advance', [Number(document.querySelector('#step').value) || 0]);
  render();
  canvas.focus({ preventScroll: true });
});

function realtimeLoop(nowMs) {
  const elapsed = Math.min(250, Math.max(0, nowMs - lastRealtimeMs));
  lastRealtimeMs = nowMs;
  if (!paused && wasm) {
    realtimeRemainderMs += elapsed;
    const ticks = Math.floor(realtimeRemainderMs / 20);
    if (ticks > 0) {
      realtimeRemainderMs -= ticks * 20;
      controllerCall('gs_advance', [ticks * 20]);
      render();
    }
  } else {
    realtimeRemainderMs = 0;
  }
  requestAnimationFrame(realtimeLoop);
}

loadController();
requestAnimationFrame(realtimeLoop);
