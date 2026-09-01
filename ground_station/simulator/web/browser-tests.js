import {
  applyFixtureToWasm,
  DECISECOND_MS,
  fixtureById,
  injectReplayRow,
  loadFixtureManifest,
  parseReplayCsv
} from './fixture-runtime.js';

const resultNode = document.querySelector('#result');
const results = [];
const actualHashes = {};

function snapshot(wasm) {
  return JSON.parse(wasm.ccall('gs_snapshot_json', 'string', [], []));
}

function framebuffer(wasm) {
  const size = wasm.ccall('gs_framebuffer_size', 'number', [], []);
  const pointer = wasm.ccall('gs_framebuffer', 'number', [], []);
  return wasm.HEAPU8.slice(pointer, pointer + size);
}

async function framebufferHash(wasm) {
  const digest = await crypto.subtle.digest('SHA-256', framebuffer(wasm));
  return Array.from(new Uint8Array(digest), byte => byte.toString(16).padStart(2, '0')).join('');
}

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function assertClose(actual, expected, tolerance, label) {
  assert(Math.abs(actual - expected) <= tolerance, `${label}: expected ${expected}, got ${actual}`);
}

function tap(wasm, button) {
  wasm.ccall('gs_press', null, ['string'], [button]);
  wasm.ccall('gs_release', null, ['string'], [button]);
}

async function test(name, body) {
  try {
    await body();
    results.push({ name, passed: true });
  } catch (error) {
    results.push({ name, passed: false, error: error instanceof Error ? error.message : String(error) });
  }
}

async function run() {
  const [moduleImport, manifest, goldens] = await Promise.all([
    import('./gs-sim.js?v=browser-tests-1'),
    loadFixtureManifest(),
    fetch('./browser-golden.json').then(response => response.json())
  ]);
  const wasm = await moduleImport.default({ locateFile: path => `${path}?v=browser-tests-1` });

  await test('module bootstrap', async () => {
    wasm.ccall('gs_reset', null, [], []);
    const state = snapshot(wasm);
    assert(state.activeScreen === 'menu', `expected menu, got ${state.activeScreen}`);
    assert(framebuffer(wasm).length === 12000, 'expected a 12,000-byte framebuffer');
    actualHashes.menu = await framebufferHash(wasm);
  });

  await test('startup animation', async () => {
    wasm.ccall('gs_restart', null, [], []);
    let state = snapshot(wasm);
    assert(state.activeScreen === 'logo', `expected startup logo, got ${state.activeScreen}`);
    assert(state.startupPhase === 'rocket_flight', `expected rocket_flight, got ${state.startupPhase}`);
    wasm.ccall('gs_advance', null, ['number'], [600]);
    actualHashes.startupRocket = await framebufferHash(wasm);
    wasm.ccall('gs_advance', null, ['number'], [2900]);
    state = snapshot(wasm);
    assert(state.activeScreen === 'menu', `expected menu after startup, got ${state.activeScreen}`);
    assert(state.startupPhase === 'complete', `expected completed startup, got ${state.startupPhase}`);
  });

  await test('light demo', async () => {
    wasm.ccall('gs_restart', null, [], []);
    const fixture = fixtureById(manifest, 'light-demo');
    const applied = await applyFixtureToWasm(wasm, fixture, { lightMode: true, reset: false });
    tap(wasm, 'ok');
    const replayRows = parseReplayCsv(applied.logs[0].csv);
    assert(applied.logs.length === 1, `expected one demo replay, got ${applied.logs.length}`);
    assert(replayRows.length === 3300, `expected 3300 replay rows, got ${replayRows.length}`);
    assert(fixture.lightPlaybackSpeed === 1, `expected real-time playback, got ${fixture.lightPlaybackSpeed}x`);
    assert(DECISECOND_MS === 100, `expected one decisecond to equal 100 ms, got ${DECISECOND_MS} ms`);
    assert(replayRows[0].timestampDs === 5929, `expected first timestamp 5929 ds, got ${replayRows[0].timestampDs}`);
    assert(replayRows.at(-1).timestampDs === 8629, `expected last timestamp 8629 ds, got ${replayRows.at(-1).timestampDs}`);
    assert(replayRows.at(-1).elapsedMs === 270000, `expected 270000 ms duration, got ${replayRows.at(-1).elapsedMs}`);
    assert(replayRows.find(row => row.timestampDs === 5930)?.elapsedMs === 100,
      'expected a one-decisecond timestamp step to replay after 100 ms');
    assertClose(replayRows[0].latitude, 39.3899, 0.00001, 'replay latitude');
    assertClose(replayRows[0].longitude, -8.2895, 0.00001, 'replay longitude');
    assertClose(replayRows[0].voltage, 8.1, 0.001, 'replay voltage');
    injectReplayRow(wasm, replayRows[0]);
    const state = snapshot(wasm);
    assert(state.activeScreen === 'live', `expected live, got ${state.activeScreen}`);
    assert(state.logCount >= 1, `expected the demo log to remain available, got ${state.logCount}`);
    assertClose(state.navigation.homeLatitude, 39.3899, 0.00001, 'home latitude');
    assertClose(state.navigation.homeLongitude, -8.2895, 0.00001, 'home longitude');
    actualHashes.lightDemo = await framebufferHash(wasm);
  });

  await test('physical controls and snapshot', async () => {
    wasm.ccall('gs_reset', null, [], []);
    tap(wasm, 'right');
    tap(wasm, 'ok');
    const state = snapshot(wasm);
    assert(state.activeScreen === 'recovery', `expected recovery, got ${state.activeScreen}`);
    assert(state.framebufferRevision > 0, 'framebuffer revision did not advance');
    actualHashes.recovery = await framebufferHash(wasm);
  });

  await test('phone sensor injection', async () => {
    wasm.ccall('gs_set_sensor_json', null, ['string'], [JSON.stringify({
      homeLatitude: 47.1661,
      homeLongitude: 8.5155,
      northRad: -0.75,
      pitchRad: 0.2,
      rollRad: -0.1,
      ax: 0.1,
      ay: -0.2,
      az: 0.95,
      gx: 12.5,
      gy: -3.25,
      gz: 0.5
    })]);
    const state = snapshot(wasm);
    assertClose(state.navigation.homeLatitude, 47.1661, 0.00001, 'sensor latitude');
    assertClose(state.navigation.homeLongitude, 8.5155, 0.00001, 'sensor longitude');
    assertClose(state.navigation.northRad, -0.75, 0.00001, 'sensor heading');
    assertClose(state.navigation.pitchRad, 0.2, 0.00001, 'sensor pitch');
    assertClose(state.navigation.rollRad, -0.1, 0.00001, 'sensor roll');
    assertClose(state.navigation.ax, 0.1, 0.00001, 'sensor acceleration X');
    assertClose(state.navigation.ay, -0.2, 0.00001, 'sensor acceleration Y');
    assertClose(state.navigation.az, 0.95, 0.00001, 'sensor acceleration Z');
    assertClose(state.navigation.gx, 12.5, 0.00001, 'sensor rotation X');
    assertClose(state.navigation.gy, -3.25, 0.00001, 'sensor rotation Y');
    assertClose(state.navigation.gz, 0.5, 0.00001, 'sensor rotation Z');
  });

  await test('compass orientation view', async () => {
    wasm.ccall('gs_reset', null, [], []);
    wasm.ccall('gs_set_navigation_json', null, ['string'], [JSON.stringify({
      northRad: Math.PI / 2,
      pitchRad: -Math.PI / 15,
      rollRad: Math.PI / 22.5,
      mx: 940,
      my: 180,
      mz: -290
    })]);
    tap(wasm, 'down');
    tap(wasm, 'right');
    tap(wasm, 'ok');
    tap(wasm, 'right');
    const state = snapshot(wasm);
    assert(state.activeScreen === 'sensors', `expected sensors, got ${state.activeScreen}`);
    assert(state.sensorView === 'orientation', `expected orientation, got ${state.sensorView}`);
    actualHashes.compassOrientation = await framebufferHash(wasm);
  });

  await test('all declarative fixtures', async () => {
    for (const fixture of manifest.fixtures) {
      await applyFixtureToWasm(wasm, fixture);
      const state = snapshot(wasm);
      const bytes = framebuffer(wasm);
      assert(typeof state.activeScreen === 'string', `${fixture.id}: missing activeScreen`);
      assert(state.framebufferRevision > 0, `${fixture.id}: framebuffer was not rendered`);
      assert(bytes.length === 12000 && bytes.some(byte => byte !== 0), `${fixture.id}: invalid framebuffer`);
    }
  });

  await test('USB storage fixture', async () => {
    await applyFixtureToWasm(wasm, fixtureById(manifest, 'usb-storage'));
    const state = snapshot(wasm);
    assert(state.activeScreen === 'usb_storage', `expected usb_storage, got ${state.activeScreen}`);
    assert(state.usbStorageState === 'host', `expected host storage, got ${state.usbStorageState}`);
    actualHashes.usbStorage = await framebufferHash(wasm);
  });

  await test('recorder fault fixture', async () => {
    await applyFixtureToWasm(wasm, fixtureById(manifest, 'recorder-fault'));
    const state = snapshot(wasm);
    assert(state.recorderState === 'fault', `expected recorder fault, got ${state.recorderState}`);
    actualHashes.recorderFault = await framebufferHash(wasm);
  });

  await test('framebuffer goldens', async () => {
    for (const [name, actual] of Object.entries(actualHashes)) {
      assert(goldens[name], `missing browser golden for ${name}`);
      assert(actual === goldens[name], `${name}: expected ${goldens[name]}, got ${actual}`);
    }
  });

  const failed = results.filter(result => !result.passed);
  const report = { passed: failed.length === 0, results, framebufferSha256: actualHashes };
  globalThis.browserTestReport = report;
  resultNode.dataset.status = report.passed ? 'passed' : 'failed';
  resultNode.textContent = JSON.stringify(report, null, 2);
  document.title = report.passed ? 'PASS - Ground Station browser tests' : 'FAIL - Ground Station browser tests';
}

run().catch(error => {
  const report = { passed: false, fatal: error instanceof Error ? error.stack || error.message : String(error), results };
  globalThis.browserTestReport = report;
  resultNode.dataset.status = 'failed';
  resultNode.textContent = JSON.stringify(report, null, 2);
  document.title = 'FAIL - Ground Station browser tests';
});
