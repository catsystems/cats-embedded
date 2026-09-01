import { applyFixtureToWasm, fixtureById, injectReplayRow, loadFixtureManifest, parseReplayCsv } from './fixture-runtime.js';

const canvas = document.querySelector('#display');
const ctx = canvas.getContext('2d', { alpha: false });
ctx.imageSmoothingEnabled = false;
const snapshotNode = document.querySelector('#snapshot');
const statusNode = document.querySelector('#status');
const fixtureSelect = document.querySelector('#fixture');
const fixtureDescriptionNode = document.querySelector('#fixture-description');
const coverageBody = document.querySelector('#coverage-body');
const lightDemoButton = document.querySelector('#light-reset');
const lightRebootButton = document.querySelector('#light-reboot');
const phoneSensorPanel = document.querySelector('#phone-sensor-panel');
const phoneSensorButton = document.querySelector('#phone-sensor-toggle');
const phoneSensorStatus = document.querySelector('#phone-sensor-status');
const lightMode = new URLSearchParams(window.location.search).get('mode') === 'light';
document.documentElement.classList.toggle('light-mode', lightMode);
document.querySelector('h1').textContent = lightMode ? 'Try the CATS Ground Station' : 'CATS Ground Station Simulator';
let paused = false;
let held = new Set();
let state = { activeScreen: 'menu', virtualTimeMs: 0, framebufferRevision: 0 };
let loadError = null;
let demoLogError = null;
let fixtureStatus = null;
let fixtureManifest = null;
let demoPlayback = null;
let lastRealtimeMs = performance.now();
let realtimeRemainderMs = 0;
const simulatorBuildTag = 'compass-view-20260823-1';
const phoneSensorRefreshIntervalMs = 200;
const standardGravity = 9.80665;
const phoneSensorValues = { ax: 0, ay: 0, az: 1, gx: 0, gy: 0, gz: 0, pitchRad: 0, rollRad: 0 };
const phoneSensorLabels = { location: 'Location waiting', compass: 'Compass waiting', motion: 'Motion waiting' };
let phoneSensorsEnabled = false;
let geolocationWatchId = null;
let phoneSensorUpdateQueued = false;
let lastPhoneStatusUpdateMs = 0;

function isMobileDevice() {
  if (typeof navigator.userAgentData?.mobile === 'boolean') return navigator.userAgentData.mobile;
  return /Android|iPhone|iPad|iPod|Mobile/i.test(navigator.userAgent) ||
    (navigator.platform === 'MacIntel' && navigator.maxTouchPoints > 1);
}

function updatePhoneSensorStatus(force = false) {
  const now = performance.now();
  if (!force && now - lastPhoneStatusUpdateMs < 250) return;
  lastPhoneStatusUpdateMs = now;
  phoneSensorStatus.textContent = Object.values(phoneSensorLabels).join(' · ');
}

function queuePhoneSensorUpdate() {
  if (phoneSensorUpdateQueued) return;
  phoneSensorUpdateQueued = true;
  setTimeout(() => {
    phoneSensorUpdateQueued = false;
    if (phoneSensorsEnabled && wasm) sendJson('gs_set_sensor_json', phoneSensorValues);
    updatePhoneSensorStatus();
  }, phoneSensorRefreshIntervalMs);
}

function handleDeviceMotion(event) {
  const acceleration = event.accelerationIncludingGravity || event.acceleration;
  if (acceleration) {
    if (Number.isFinite(acceleration.x)) phoneSensorValues.ax = acceleration.x / standardGravity;
    if (Number.isFinite(acceleration.y)) phoneSensorValues.ay = acceleration.y / standardGravity;
    if (Number.isFinite(acceleration.z)) phoneSensorValues.az = acceleration.z / standardGravity;
  }
  const rotation = event.rotationRate;
  if (rotation) {
    if (Number.isFinite(rotation.beta)) phoneSensorValues.gx = rotation.beta;
    if (Number.isFinite(rotation.gamma)) phoneSensorValues.gy = rotation.gamma;
    if (Number.isFinite(rotation.alpha)) phoneSensorValues.gz = rotation.alpha;
  }
  phoneSensorLabels.motion = `Motion ${phoneSensorValues.ax.toFixed(2)}, ${phoneSensorValues.ay.toFixed(2)}, ${phoneSensorValues.az.toFixed(2)} g`;
  queuePhoneSensorUpdate();
}

function compassHeading(event) {
  if (Number.isFinite(event.webkitCompassHeading)) return event.webkitCompassHeading;
  if (event.absolute === true && Number.isFinite(event.alpha)) return (360 - event.alpha) % 360;
  return null;
}

function handleDeviceOrientation(event) {
  const heading = compassHeading(event);
  if (heading === null) return;
  const northRadians = -heading * Math.PI / 180;
  phoneSensorValues.northRad = Math.atan2(Math.sin(northRadians), Math.cos(northRadians));
  if (Number.isFinite(event.beta)) phoneSensorValues.pitchRad = event.beta * Math.PI / 180;
  if (Number.isFinite(event.gamma)) phoneSensorValues.rollRad = event.gamma * Math.PI / 180;
  phoneSensorLabels.compass = `Compass ${Math.round(heading)}°`;
  queuePhoneSensorUpdate();
}

function handleLocation(position) {
  phoneSensorValues.homeLatitude = position.coords.latitude;
  phoneSensorValues.homeLongitude = position.coords.longitude;
  phoneSensorLabels.location = `Location ±${Math.round(position.coords.accuracy)} m`;
  if (wasm) sendJson('gs_set_device_status_json', { gnss: true });
  queuePhoneSensorUpdate();
}

function handleLocationError(error) {
  phoneSensorLabels.location = error.code === 1 ? 'Location denied' : 'Location unavailable';
  if (wasm) sendJson('gs_set_device_status_json', { gnss: false });
  updatePhoneSensorStatus(true);
}

async function requestPhoneMotionPermissions() {
  const requests = [];
  if (typeof globalThis.DeviceMotionEvent?.requestPermission === 'function') {
    requests.push(globalThis.DeviceMotionEvent.requestPermission());
  }
  if (typeof globalThis.DeviceOrientationEvent?.requestPermission === 'function') {
    requests.push(globalThis.DeviceOrientationEvent.requestPermission());
  }
  if (!requests.length) return true;
  const results = await Promise.all(requests);
  return results.every(result => result === 'granted');
}

function stopPhoneSensors() {
  phoneSensorsEnabled = false;
  window.removeEventListener('devicemotion', handleDeviceMotion);
  window.removeEventListener('deviceorientationabsolute', handleDeviceOrientation);
  window.removeEventListener('deviceorientation', handleDeviceOrientation);
  if (geolocationWatchId !== null) navigator.geolocation.clearWatch(geolocationWatchId);
  geolocationWatchId = null;
  phoneSensorLabels.location = 'Location stopped';
  phoneSensorLabels.compass = 'Compass stopped';
  phoneSensorLabels.motion = 'Motion stopped';
  phoneSensorButton.textContent = 'Use phone sensors';
  updatePhoneSensorStatus(true);
}

async function startPhoneSensors() {
  phoneSensorButton.disabled = true;
  phoneSensorStatus.textContent = 'Requesting sensor access…';
  try {
    const motionAllowed = await requestPhoneMotionPermissions();
    phoneSensorsEnabled = true;
    if (motionAllowed) {
      window.addEventListener('devicemotion', handleDeviceMotion);
      window.addEventListener('deviceorientationabsolute', handleDeviceOrientation);
      window.addEventListener('deviceorientation', handleDeviceOrientation);
    } else {
      phoneSensorLabels.compass = 'Compass denied';
      phoneSensorLabels.motion = 'Motion denied';
    }
    if ('geolocation' in navigator) {
      geolocationWatchId = navigator.geolocation.watchPosition(handleLocation, handleLocationError, {
        enableHighAccuracy: true,
        maximumAge: 1000,
        timeout: 10000
      });
    } else {
      phoneSensorLabels.location = 'Location unavailable';
    }
    phoneSensorButton.textContent = 'Stop phone sensors';
    updatePhoneSensorStatus(true);
  } catch {
    phoneSensorsEnabled = false;
    phoneSensorStatus.textContent = 'Sensor access was not granted. You can try again.';
    phoneSensorButton.textContent = 'Try again';
  } finally {
    phoneSensorButton.disabled = false;
  }
}

if (lightMode && isMobileDevice()) phoneSensorPanel.hidden = false;
phoneSensorButton.addEventListener('click', () => {
  if (phoneSensorsEnabled) stopPhoneSensors();
  else startPhoneSensors();
});

function updateFixtureDescription() {
  if (!fixtureManifest) return;
  fixtureDescriptionNode.textContent = fixtureById(fixtureManifest, fixtureSelect.value).description;
}

function sendJson(name, value) {
  wasm.ccall(name, null, ['string'], [JSON.stringify(value)]);
}

async function applyFixture(name) {
  if (!wasm || !fixtureManifest) return;
  demoPlayback = null;
  const fixture = fixtureById(fixtureManifest, name);
  await applyFixtureToWasm(wasm, fixture);
  fixtureStatus = `${fixture.label} fixture`;
  render();
}

// The browser is only a host: the controller API owns state, actions, timing,
// and framebuffer bytes. A generated Emscripten module can be dropped beside
// this file without changing the controls below.
let wasm = null;

async function loadDefaultDemo(play = false) {
  const fixture = fixtureById(fixtureManifest, lightMode ? 'light-demo' : 'demo');
  const applied = await applyFixtureToWasm(wasm, fixture, { lightMode, reset: false });
  demoPlayback = null;
  if (lightMode && play && applied.logs.length) {
    wasm.ccall('gs_press', null, ['string'], ['ok']);
    wasm.ccall('gs_release', null, ['string'], ['ok']);
    demoPlayback = {
      rows: parseReplayCsv(applied.logs[0].csv),
      index: 0,
      elapsedMs: 0,
      speed: fixture.lightPlaybackSpeed || 1
    };
    advanceDemoPlayback(0);
  }
}

function advanceDemoPlayback(elapsedMs) {
  if (!demoPlayback || demoPlayback.index >= demoPlayback.rows.length) return false;
  demoPlayback.elapsedMs += elapsedMs * demoPlayback.speed;
  let changed = false;
  while (demoPlayback.index < demoPlayback.rows.length &&
         demoPlayback.rows[demoPlayback.index].elapsedMs <= demoPlayback.elapsedMs) {
    injectReplayRow(wasm, demoPlayback.rows[demoPlayback.index]);
    demoPlayback.index += 1;
    changed = true;
  }
  return changed;
}

async function loadFixtures() {
  fixtureManifest = await loadFixtureManifest();
  fixtureSelect.replaceChildren(...fixtureManifest.fixtures.map(fixture =>
    new Option(fixture.label, fixture.id)));
  updateFixtureDescription();
}

async function loadCoverage() {
  const response = await fetch('./coverage.json');
  if (!response.ok) throw new Error(`coverage: HTTP ${response.status}`);
  const coverage = await response.json();
  if (coverage.schemaVersion !== 1 || !Array.isArray(coverage.groups)) {
    throw new Error('Unsupported simulator coverage matrix.');
  }
  for (const group of coverage.groups) {
    const heading = document.createElement('tr');
    heading.className = 'coverage-group';
    const cell = document.createElement('th');
    cell.colSpan = 5;
    cell.scope = 'colgroup';
    cell.textContent = group.name;
    heading.append(cell);
    coverageBody.append(heading);
    for (const row of group.rows) {
      const tableRow = document.createElement('tr');
      if (row.gap) tableRow.className = 'coverage-gap';
      for (const value of [row.path, row.scenario, row.snapshot, row.framebuffer, row.browser]) {
        const tableCell = document.createElement('td');
        tableCell.textContent = value;
        tableRow.append(tableCell);
      }
      coverageBody.append(tableRow);
    }
  }
}

async function loadController() {
  try {
    await loadFixtures();
  } catch (error) {
    demoLogError = error instanceof Error ? error.message : String(error);
    console.error('Ground Station simulator fixtures failed to load:', error);
  }
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
  if (wasm && fixtureManifest) {
    loadError = null;
    wasm.ccall('gs_restart', null, [], []);
    try {
      await loadDefaultDemo();
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
      : lightMode
      ? 'Demo ready.'
      : 'WebAssembly simulator ready with the demo fixture.';
  } else {
    state = {
      activeScreen: 'unavailable',
      virtualTimeMs: 0,
      framebufferRevision: 0,
      error: loadError || 'gs-sim.js is missing. Run gs-sim.ps1 setup, then serve again.'
    };
    statusNode.textContent = `Simulator unavailable: ${state.error}`;
  }
  statusNode.hidden = lightMode && wasm !== null && demoLogError === null;
  if (!lightMode) snapshotNode.textContent = JSON.stringify(state, null, 2);
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
async function resetDemo(play = false) {
  if (wasm) controllerCall('gs_restart');
  fixtureStatus = null;
  try {
    await loadDefaultDemo(play);
    demoLogError = null;
    if (lightMode && play) lightDemoButton.textContent = 'Restart demo';
  } catch (error) {
    demoLogError = error instanceof Error ? error.message : String(error);
    console.error('Ground Station simulator demo logs failed to load:', error);
  }
  render();
  canvas.focus({ preventScroll: true });
}

function rebootDemo() {
  demoPlayback = null;
  fixtureStatus = null;
  if (wasm) controllerCall('gs_restart');
  lightDemoButton.textContent = 'Play demo';
  render();
  canvas.focus({ preventScroll: true });
}

document.querySelector('#reset').addEventListener('click', () => resetDemo());
lightDemoButton.addEventListener('click', () => resetDemo(true));
lightRebootButton.addEventListener('click', rebootDemo);
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
  const elapsed = Math.max(0, nowMs - lastRealtimeMs);
  const controllerElapsed = Math.min(250, elapsed);
  lastRealtimeMs = nowMs;
  if (!paused && wasm) {
    let changed = false;
    realtimeRemainderMs += controllerElapsed;
    const ticks = Math.floor(realtimeRemainderMs / 20);
    if (ticks > 0) {
      realtimeRemainderMs -= ticks * 20;
      controllerCall('gs_advance', [ticks * 20]);
      changed = true;
    }
    if (lightMode && advanceDemoPlayback(elapsed)) changed = true;
    if (changed) render();
  } else {
    realtimeRemainderMs = 0;
  }
  requestAnimationFrame(realtimeLoop);
}

updateFixtureDescription();
if (!lightMode) loadCoverage().catch(error => console.error('Coverage matrix failed to load:', error));
loadController();
requestAnimationFrame(realtimeLoop);
