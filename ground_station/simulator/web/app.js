const canvas = document.querySelector('#display');
const ctx = canvas.getContext('2d', { alpha: false });
ctx.imageSmoothingEnabled = false;
const snapshotNode = document.querySelector('#snapshot');
const statusNode = document.querySelector('#status');
const fixtureSelect = document.querySelector('#fixture');
const fixtureDescriptionNode = document.querySelector('#fixture-description');
let paused = false;
let held = new Set();
let state = { activeScreen: 'menu', virtualTimeMs: 0, framebufferRevision: 0 };
let loadError = null;
let demoLogError = null;
let fixtureStatus = null;
let lastRealtimeMs = performance.now();
let realtimeRemainderMs = 0;
const simulatorBuildTag = 'cat-island-fixtures-20260816-1';

const demoLogNames = ['log_001.csv', 'log_002.csv'];
const csvHeader = 'link,ts[deciseconds],state,errors,lat[deg/10000],lon[deg/10000],altitude[m],velocity[m/s],battery[decivolts],pyro1,pyro2\n';
const fixtureDescriptions = {
  demo: 'Loads one complete mission log and one damaged or partial log. Use it to check valid statistics, unavailable values shown as --, and safe malformed-row handling.',
  dual: 'Runs a completed two-rocket mission. Both links record liftoff through touchdown and both recovery targets have valid locations.',
  many: 'Loads 24 completed logs. Use it to check list scrolling, viewport edge arrows, and selection at the first and last log.',
  'missing-fix': 'Runs Dual Recovery with a valid location for Link 1 and no location for Link 2. Use it to check target selection and No location behavior.',
  'zero-coordinate': 'Loads valid locations on the prime meridian and equator. Both must remain available as map QR codes.',
  never: 'Creates an active dual-link log with Stop Logging set to Never. Recording remains active after touchdown until Finalize Log is confirmed.',
  'delete-failure': 'Loads a finalized log and injects a storage deletion error. The log must remain in the catalog and an error message should appear.',
  'usb-delete': 'Loads a finalized log while USB mass storage is connected. Delete Log must be blocked until USB is disconnected.',
  'usb-storage': 'Connects USB while idle, automatically shares the logs, and opens Settings > General > USB Drive. Press B to return without disconnecting, or press A to return storage to the Ground Station.',
  'finalize-failure': 'Creates an active Never-mode log and injects a finalization error. The log must remain active after the failed attempt.',
  'recorder-fault': 'Injects a storage write failure as recording starts. Use it to check the recorder fault state and crossed logging indicator.'
};

function updateFixtureDescription() {
  fixtureDescriptionNode.textContent = fixtureDescriptions[fixtureSelect.value] || '';
}

function completeLog(name, offset = 0) {
  const rows = [];
  for (const link of [1, 2]) {
    const lat = 337363 + offset + link * 5;
    const lon = 1324829 + offset + link * 5;
    rows.push(`${link},10,3,0,${lat},${lon},100,30,42,1,1`);
    rows.push(`${link},20,5,0,${lat},${lon},1000,0,41,1,1`);
    rows.push(`${link},30,6,0,${lat},${lon},400,-25,40,1,1`);
    rows.push(`${link},50,7,0,${lat},${lon},5,0,39,1,1`);
  }
  return { name, csv: csvHeader + rows.join('\n') + '\n' };
}

function incompleteLog() {
  return {
    name: 'log_099.csv',
    csv: csvHeader +
      '1,10,3,0,337363,1324829,100,30,42,1,1\n' +
      'malformed,row\n' +
      '1,20,5,0,337369,1324832,900,0,41,1,1\n' +
      '2,12,3,0,0,0,80,20,42,1,1\n'
  };
}

function sendJson(name, value) {
  wasm.ccall(name, null, ['string'], [JSON.stringify(value)]);
}

function sendLink(link, value) {
  wasm.ccall('gs_set_link_json', null, ['string', 'string'], [String(link), JSON.stringify(value)]);
}

function tapSimulatorButton(button) {
  wasm.ccall('gs_press', null, ['string'], [button]);
  wasm.ccall('gs_release', null, ['string'], [button]);
}

async function applyFixture(name) {
  if (!wasm) return;
  wasm.ccall('gs_reset', null, [], []);
  if (name === 'demo') {
    sendJson('gs_set_logs_json', [completeLog('log_100.csv'), incompleteLog()]);
  } else if (name === 'many') {
    sendJson('gs_set_logs_json', Array.from({ length: 24 }, (_, index) => completeLog(`log_${String(124 - index).padStart(3, '0')}.csv`, index)));
  } else if (name === 'dual' || name === 'missing-fix') {
    sendJson('gs_set_logs_json', []);
    sendJson('gs_set_configuration_json', { dualReceiver: true, neverStopLogging: false });
    sendJson('gs_set_navigation_json', { homeLatitude: 33.7340, homeLongitude: 132.4730, northRad: 0.35 });
    sendLink(1, { state: 3, timestampDs: 10, latitude: 33.7363, longitude: 132.4829,
      altitudeM: 120, velocityMps: 42, voltage: 4.1, pyroContinuity: 3, connected: true });
    sendLink(2, { state: name === 'missing-fix' ? 2 : 3,
      timestampDs: 10, latitude: name === 'missing-fix' ? 0 : 33.7358,
      longitude: name === 'missing-fix' ? 0 : 132.4762, altitudeM: 110, velocityMps: 39,
      voltage: 4.1, pyroContinuity: 3, connected: true });
    for (const [state, timestampDs, altitudeM, velocityMps] of [[5, 20, 1000, 0], [6, 30, 400, -25], [7, 50, 5, 0]]) {
      sendLink(1, { state, timestampDs, latitude: 33.7363, longitude: 132.4829, altitudeM, velocityMps,
        voltage: 4.0, pyroContinuity: 3, connected: true });
      sendLink(2, { state: name === 'missing-fix' ? 2 : state, timestampDs,
        latitude: name === 'missing-fix' ? 0 : 33.7358, longitude: name === 'missing-fix' ? 0 : 132.4762,
        altitudeM, velocityMps, voltage: 4.0, pyroContinuity: 3, connected: true });
    }
  } else if (name === 'zero-coordinate') {
    sendJson('gs_set_navigation_json', {
      homeLatitude: 0.0, homeLongitude: -30.0, rocketLatitude: 1.0, rocketLongitude: -30.0
    });
    sendJson('gs_set_logs_json', [{
      name: 'zero-coordinate.csv',
      csv: csvHeader +
        '1,10,3,0,5000,0,100,20,42,0,0\n' +
        '2,11,3,0,0,-300000,100,20,42,0,0\n'
    }]);
  } else if (name === 'never' || name === 'finalize-failure') {
    sendJson('gs_set_logs_json', []);
    sendJson('gs_set_configuration_json', { dualReceiver: true, neverStopLogging: true });
    wasm.ccall('gs_set_link_json', null, ['string', 'string'], ['1', JSON.stringify({ state: 7, timestampDs: 50,
      latitude: 33.7363, longitude: 132.4829, connected: true })]);
    wasm.ccall('gs_set_link_json', null, ['string', 'string'], ['2', JSON.stringify({ state: 7, timestampDs: 50,
      latitude: 33.7358, longitude: 132.4762, connected: true })]);
    if (name === 'finalize-failure') sendJson('gs_set_device_status_json', { finalizeFailure: true });
  } else if (name === 'delete-failure') {
    sendJson('gs_set_logs_json', [completeLog('log_105.csv')]);
    sendJson('gs_set_device_status_json', { deleteFailure: true });
  } else if (name === 'usb-delete') {
    sendJson('gs_set_logs_json', [completeLog('log_106.csv')]);
    sendJson('gs_set_device_status_json', { usb: true, usbStorageState: 'host' });
  } else if (name === 'usb-storage') {
    sendJson('gs_set_device_status_json', { usb: true, usbStorageState: 'firmware' });
    for (const button of ['right', 'right', 'down', 'ok', 'down', 'down', 'down', 'ok']) {
      tapSimulatorButton(button);
    }
  } else if (name === 'recorder-fault') {
    sendJson('gs_set_logs_json', []);
    sendJson('gs_set_device_status_json', { recorderWriteFailure: true });
    wasm.ccall('gs_set_link_json', null, ['string', 'string'], ['1', JSON.stringify({ state: 3, timestampDs: 10,
      latitude: 33.7363, longitude: 132.4829, connected: true })]);
  }
  fixtureStatus = `${document.querySelector('#fixture').selectedOptions[0].textContent} fixture`;
  render();
}

// The browser is only a host: the controller API owns state, actions, timing,
// and framebuffer bytes. A generated Emscripten module can be dropped beside
// this file without changing the controls below.
let wasm = null;

async function loadDemoLogs() {
  if (!wasm) return;
  const logs = await Promise.all(demoLogNames.map(async name => {
    const response = await fetch(`./demo-logs/${name}`);
    if (!response.ok) throw new Error(`${name}: HTTP ${response.status}`);
    return { name, csv: await response.text() };
  }));
  wasm.ccall('gs_set_logs_json', null, ['string'], [JSON.stringify(logs)]);
}

async function loadController() {
  try {
    const moduleFactory = (await import(`./gs-sim.js?v=${simulatorBuildTag}`)).default;
    wasm = await moduleFactory({
      locateFile: path => `${path}?v=${simulatorBuildTag}`
    });
  } catch (error) {
    wasm = null;
    loadError = error instanceof Error ? error.message : String(error);
    console.error('Ground Station simulator WebAssembly failed to load:', error);
  }
  if (wasm) {
    loadError = null;
    wasm.ccall('gs_restart', null, [], []);
    try {
      await loadDemoLogs();
      demoLogError = null;
    } catch (error) {
      demoLogError = error instanceof Error ? error.message : String(error);
      console.error('Ground Station simulator demo logs failed to load:', error);
    }
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
    statusNode.textContent = fixtureStatus
      ? `WebAssembly simulator ready: ${fixtureStatus}.`
      : demoLogError
      ? `WebAssembly simulator ready; demo logs unavailable: ${demoLogError}`
      : 'WebAssembly simulator ready with 2 demo logs.';
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
  const button = event.currentTarget;
  button.querySelector('span').textContent = paused ? '▶' : '⏸';
  button.setAttribute('aria-label', paused ? 'Resume virtual time' : 'Pause virtual time');
  button.title = paused ? 'Resume virtual time' : 'Pause virtual time';
  canvas.focus({ preventScroll: true });
});
document.querySelector('#reset').addEventListener('click', async () => {
  if (wasm) controllerCall('gs_restart');
  fixtureStatus = null;
  try {
    await loadDemoLogs();
    demoLogError = null;
  } catch (error) {
    demoLogError = error instanceof Error ? error.message : String(error);
    console.error('Ground Station simulator demo logs failed to load:', error);
  }
  render();
  canvas.focus({ preventScroll: true });
});
document.querySelector('#advance').addEventListener('click', () => {
  if (wasm) controllerCall('gs_advance', [Number(document.querySelector('#step').value) || 0]);
  render();
  canvas.focus({ preventScroll: true });
});
document.querySelector('#apply-fixture').addEventListener('click', async () => {
  await applyFixture(fixtureSelect.value);
  canvas.focus({ preventScroll: true });
});
document.querySelector('#eject-usb').addEventListener('click', () => {
  if (wasm) sendJson('gs_set_device_status_json', { usbStorageState: 'firmware' });
  fixtureStatus = 'USB drive ejected';
  render();
  canvas.focus({ preventScroll: true });
});
fixtureSelect.addEventListener('change', updateFixtureDescription);

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

updateFixtureDescription();
loadController();
requestAnimationFrame(realtimeLoop);
