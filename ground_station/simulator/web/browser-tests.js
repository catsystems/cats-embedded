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
    fetch('./browser-golden.json', {cache: 'no-store'}).then(response => response.json())
  ]);
  const wasm = await moduleImport.default({ locateFile: path => `${path}?v=browser-tests-1` });

  await test('module bootstrap', async () => {
    wasm.ccall('gs_reset', null, [], []);
    const state = snapshot(wasm);
    assert(state.activeScreen === 'menu', `expected menu, got ${state.activeScreen}`);
    assert(framebuffer(wasm).length === 12000, 'expected a 12,000-byte framebuffer');
    actualHashes.menu = await framebufferHash(wasm);
    captureSelfTest('Main menu status bar');
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
    captureSelfTest('Live status bar');
  });

  await test('physical controls and snapshot', async () => {
    wasm.ccall('gs_reset', null, [], []);
    tap(wasm, 'right');
    tap(wasm, 'ok');
    const state = snapshot(wasm);
    assert(state.activeScreen === 'recovery', `expected recovery, got ${state.activeScreen}`);
    assert(state.framebufferRevision > 0, 'framebuffer revision did not advance');
    actualHashes.recovery = await framebufferHash(wasm);
    captureSelfTest('Recovery status bar');
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
    captureSelfTest('Compass status bar');
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
    captureSelfTest('Recorder fault status bar');
  });

  await test('framebuffer goldens', async () => {
    for (const [name, actual] of Object.entries(actualHashes)) {
      assert(goldens[name], `missing browser golden for ${name}`);
      assert(actual === goldens[name], `${name}: expected ${goldens[name]}, got ${actual}`);
    }
  });

  function openSelfTest(faults = {}) {
    wasm.ccall('gs_reset', null, [], []);
    wasm.ccall('gs_set_configuration_json', null, ['string'], [JSON.stringify({linkPhrase1: 'normal-1', linkPhrase2: 'normal-2'})]);
    wasm.ccall('gs_set_device_status_json', null, ['string'], [JSON.stringify({usb: true, batteryVoltage: 3.8, ...faults})]);
    for (const button of ['down', 'right', 'right', 'ok', 'right', 'right', 'down', 'down', 'down', 'ok']) tap(wasm, button);
    assert(snapshot(wasm).activeScreen === 'self_test', 'Settings > System > Self-Test opens');
  }
  function captureSelfTest(label) {
    const figure = document.createElement('figure');
    const caption = document.createElement('figcaption');
    caption.textContent = label;
    const canvas = document.createElement('canvas');
    canvas.width = 400; canvas.height = 240;
    canvas.style.cssText = 'width:600px;max-width:100%;image-rendering:pixelated;border:1px solid #888';
    const context = canvas.getContext('2d');
    const pixels = context.createImageData(400, 240);
    const bytes = framebuffer(wasm);
    for (let i = 0; i < 400 * 240; ++i) {
      const value = (bytes[i >> 3] & (0x80 >> (i & 7))) ? 255 : 0;
      pixels.data.set([value, value, value, 255], i * 4);
    }
    context.putImageData(pixels, 0, 0);
    figure.append(caption, canvas);
    document.body.append(figure);
  }
  const advanceSelfTest = ms => wasm.ccall('gs_advance', null, ['number'], [ms]);

  function leaveSelfTestAndCheckHeader() {
    // Inspect the first exit frame, before a release or subsequent redraw can
    // clear away the stale pixels we are trying to detect.
    wasm.ccall('gs_press', null, ['string'], ['back']);
    const exitHeader = framebuffer(wasm).slice(0, 400 * 19 / 8);
    wasm.ccall('gs_release', null, ['string'], ['back']);
    const cleanHeader = framebuffer(wasm).slice(0, exitHeader.length);
    assert(exitHeader.every((value, index) => value === cleanHeader[index]), 'first exit frame restores the entire status bar');
  }

  await test('factory self-test: leaving the entry restores the status bar', async () => {
    openSelfTest();
    leaveSelfTestAndCheckHeader();
    assert(snapshot(wasm).activeScreen === 'settings', 'back returns to Settings');
    captureSelfTest('Settings after leaving self-test entry');
  });

  function confirmSelfTestStep(phase, label, activePhase = phase) {
    let state = snapshot(wasm);
    assert(state.selfTestPhase === phase && state.selfTestWaiting, `${label} waits for Start`);
    const resultsBefore = JSON.stringify(state.selfTestChecks);
    const actionsBefore = state.actions.length;
    advanceSelfTest(120000);
    tap(wasm, 'up');
    state = snapshot(wasm);
    assert(state.selfTestPhase === phase && state.selfTestWaiting, `${label} does not start from waiting or another button`);
    assert(JSON.stringify(state.selfTestChecks) === resultsBefore, `${label} does not score before Start`);
    assert(state.actions.length === actionsBefore, `${label} has no radio actions before Start`);
    captureSelfTest(label);
    tap(wasm, 'ok');
    state = snapshot(wasm);
    assert(state.selfTestPhase === activePhase && !state.selfTestWaiting, `${label} starts only on A`);
  }

  await test('factory self-test: three phases and confirmation before every manual check', async () => {
    openSelfTest();
    captureSelfTest('Self-test entry');
    tap(wasm, 'ok');
    advanceSelfTest(800);
    captureSelfTest('Phase 1 - Ground Station');
    advanceSelfTest(3200);
    let state = snapshot(wasm);
    assert(!state.actions.some(action => action.type === 'self_test_radio'), 'Phase 1 does not change radio settings');
    confirmSelfTestStep(2, 'Phase 2 - ready for telemetry');
    advanceSelfTest(8000);
    captureSelfTest('Phase 2 - telemetry running');
    advanceSelfTest(64000);
    state = snapshot(wasm);
    assert(state.selfTestChecks.slice(0, 11).every(value => value === 'PASS'), `automatic results: ${state.selfTestChecks}`);
    assert(state.configuration.linkPhrase1 === 'normal-1' && state.configuration.linkPhrase2 === 'normal-2', 'test phrases never alter saved config');
    confirmSelfTestStep(3, '3.1 - ready for sensor movement');
    captureSelfTest('Sensor movement running');
    for (let axis = 0; axis < 3; ++axis) {
      for (const sign of [-1, 1]) {
        const sample = {ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0, mx: sign*1000, my: sign*800, mz: sign*600};
        sample[['ax', 'ay', 'az'][axis]] = sign;
        sample[['gx', 'gy', 'gz'][axis]] = 30;
        wasm.ccall('gs_set_sensor_json', null, ['string'], [JSON.stringify(sample)]);
      }
    }
    confirmSelfTestStep(4, '3.2 - ready for buttons');
    assert(snapshot(wasm).selfTestButtons === 0, 'Start press and release do not count as a tested button');
    wasm.ccall('gs_press', null, ['string'], ['up']);
    assert(snapshot(wasm).selfTestButtons === 0, 'a press without release does not pass');
    wasm.ccall('gs_release', null, ['string'], ['up']);
    captureSelfTest('Button press and release check');
    for (const button of ['down', 'left', 'right', 'center', 'ok', 'back']) tap(wasm, button);
    confirmSelfTestStep(5, '3.3 - ready for display');
    advanceSelfTest(2100);
    captureSelfTest('Display - black pattern');
    advanceSelfTest(2000);
    captureSelfTest('Display - white pattern');
    advanceSelfTest(2000);
    captureSelfTest('Display - checkerboard');
    advanceSelfTest(2000);
    captureSelfTest('Display - inverse checkerboard');
    advanceSelfTest(2000);
    captureSelfTest('Display confirmation');
    wasm.ccall('gs_press', null, ['string'], ['ok']);
    advanceSelfTest(15000);
    assert(snapshot(wasm).selfTestWaiting, 'holding display confirmation cannot start the LED check');
    wasm.ccall('gs_release', null, ['string'], ['ok']);
    confirmSelfTestStep(6, '3.4 - ready for receiver LEDs');
    advanceSelfTest(12100);
    captureSelfTest('Receiver LED confirmation');
    tap(wasm, 'ok');
    captureSelfTest('Restoring settings');
    advanceSelfTest(2600);
    assert(snapshot(wasm).selfTestPhase === 8, 'normal receiver settings restored before battery test');
    wasm.ccall('gs_set_device_status_json', null, ['string'], ['{"usb":false}']);
    confirmSelfTestStep(8, '3.5 - ready for battery', 9);
    advanceSelfTest(2000);
    captureSelfTest('Battery operation');
    advanceSelfTest(3200);
    assert(snapshot(wasm).selfTestPhase === 10, 'five battery seconds precede reconnect');
    wasm.ccall('gs_set_device_status_json', null, ['string'], ['{"usb":true}']);
    confirmSelfTestStep(10, '3.6 - ready for USB reconnect');
    advanceSelfTest(1000);
    assert(snapshot(wasm).selfTestPhase === 10, 'connecting before Start cannot pass reconnect');
    wasm.ccall('gs_set_device_status_json', null, ['string'], ['{"usb":false}']);
    advanceSelfTest(20);
    wasm.ccall('gs_set_device_status_json', null, ['string'], ['{"usb":true}']);
    advanceSelfTest(20);
    state = snapshot(wasm);
    assert(state.selfTestResult === 'PASS' && state.selfTestChecks.length === 18, `complete test: ${state.selfTestChecks}`);
    assert(state.usbStorageState === 'host', 'normal storage sharing resumes after the test');
    captureSelfTest('Completed result');
    for (let index = 0; index < 11; ++index) tap(wasm, 'down');
    captureSelfTest('Movement result detail');
    for (let index = 0; index < 6; ++index) tap(wasm, 'down');
    captureSelfTest('Restoration result detail');
    leaveSelfTestAndCheckHeader();
    captureSelfTest('Settings after completed self-test');
    state = snapshot(wasm);
    assert(state.activeScreen === 'settings' && state.settingsPage === 2 && state.settingsSelection === 2, 'return to self-test Settings entry');
  });

  await test('factory self-test: missing receiver cannot be hidden by the other', async () => {
    openSelfTest({selfTestMissingReceiver: 1});
    tap(wasm, 'ok'); advanceSelfTest(4000);
    tap(wasm, 'ok'); advanceSelfTest(140000);
    const state = snapshot(wasm);
    assert(state.selfTestChecks[0] === 'FAIL' && state.selfTestChecks[1] === 'PASS', 'independent firmware replies');
    assert(state.selfTestChecks[5] === 'FAIL' && state.selfTestChecks[7] === 'FAIL', 'both dual and single expose missing receiver');
    assert(state.selfTestChecks[9] === 'PASS', 'receiver 2 still tested independently');
    assert(state.selfTestPhase === 3 && state.selfTestWaiting, 'failures still reach the first manual confirmation');
  });

  await test('factory self-test: GNSS and storage failures never become passes', async () => {
    openSelfTest({selfTestGnss: false, selfTestStorageFailure: true});
    tap(wasm, 'ok'); advanceSelfTest(121000);
    let state = snapshot(wasm);
    assert(state.selfTestChecks[2] === 'FAIL', 'GNSS timeout fails acquisition');
    assert(state.selfTestChecks[10] === 'FAIL', 'storage failure is visible');
    assert(state.selfTestPhase === 2 && state.selfTestWaiting, 'telemetry waits for its own confirmation');
    tap(wasm, 'ok'); advanceSelfTest(72000);
    state = snapshot(wasm);
    assert(state.selfTestPhase === 3 && state.selfTestWaiting, 'manual checks wait after independent telemetry checks');
    assert(state.selfTestChecks[2] === 'FAIL' && state.selfTestChecks[10] === 'FAIL', 'later checks preserve Phase 1 failures');
  });

  await test('factory self-test: cancellation cannot pass and restores configuration', async () => {
    openSelfTest(); tap(wasm, 'ok'); advanceSelfTest(4000);
    tap(wasm, 'ok'); advanceSelfTest(4000);
    wasm.ccall('gs_hold', null, ['string', 'number'], ['back', 2100]);
    advanceSelfTest(2600);
    const state = snapshot(wasm);
    assert(state.selfTestPhase === 11 && state.selfTestResult === 'FAIL', `cancellation phase=${state.selfTestPhase} result=${state.selfTestResult} checks=${state.selfTestChecks}`);
    assert(state.selfTestChecks[11] === 'NOT TESTED', 'unfinished manual checks remain not tested');
    assert(state.actions.some(action => action.type === 'self_test_restored'), 'restore requested');
    assert(state.configuration.linkPhrase1 === 'normal-1', 'normal phrase retained');
    captureSelfTest('Cancelled result');
    leaveSelfTestAndCheckHeader();
    captureSelfTest('Settings after cancelled self-test');
  });

  await test('settings organization: controls, inactive phrase and System actions', async () => {
    wasm.ccall('gs_reset', null, [], []);
    wasm.ccall('gs_set_configuration_json', null, ['string'], [JSON.stringify({dualReceiver: false, linkPhrase1: 'cats_test_1', linkPhrase2: 'cats_test_2', testingPhrase: 'cats_test'})]);
    for (const button of ['down', 'right', 'right', 'ok']) tap(wasm, button);
    assert(snapshot(wasm).settingsPage === 0, 'Settings opens on Telemetry');
    captureSelfTest('Settings - Telemetry single');
    for (const button of ['down', 'down', 'down', 'ok']) tap(wasm, button);
    assert(snapshot(wasm).settingsState === 'list', 'Link Phrase 2 cannot be edited in Single mode');
    assert(snapshot(wasm).configuration.linkPhrase2 === 'cats_test_2', 'inactive phrase is retained');
    captureSelfTest('Settings - inactive Link Phrase 2');
    for (const button of ['up', 'up', 'right']) tap(wasm, button);
    assert(snapshot(wasm).configuration.dualReceiver, 'Receiver Mode changes the receiver setting');
    captureSelfTest('Settings - Telemetry dual');
    for (const key of ['linkPhrase1', 'linkPhrase2', 'testingPhrase']) {
      tap(wasm, 'down'); tap(wasm, 'ok');
      assert(snapshot(wasm).settingsState === 'keyboard', `${key} opens its keyboard`);
      const before = snapshot(wasm).configuration[key];
      tap(wasm, 'ok'); tap(wasm, 'back');
      assert(snapshot(wasm).configuration[key] === `${before}1`, `${key} edits the correct phrase`);
    }
    for (const button of ['up', 'up', 'up', 'up', 'right', 'down', 'right']) tap(wasm, button);
    assert(snapshot(wasm).settingsPage === 1 && snapshot(wasm).configuration.neverStopLogging, 'Stop Logging is first in Preferences');
    tap(wasm, 'left'); tap(wasm, 'down'); tap(wasm, 'right');
    assert(snapshot(wasm).configuration.timeZoneOffset === 1, 'Time Zone is second');
    tap(wasm, 'down'); tap(wasm, 'right');
    assert(snapshot(wasm).configuration.imperialUnits, 'Units is third');
    tap(wasm, 'down'); tap(wasm, 'left');
    assert(!snapshot(wasm).configuration.startupAnimation, 'Startup Animation is fourth');
    captureSelfTest('Settings - Preferences');
    for (const button of ['up', 'up', 'up', 'up', 'right', 'down']) tap(wasm, button);
    assert(snapshot(wasm).settingsPage === 2 && snapshot(wasm).settingsSelection === 0, 'Firmware Versions is first in System');
    captureSelfTest('Settings - System versions');
    tap(wasm, 'down'); tap(wasm, 'ok');
    assert(snapshot(wasm).activeScreen === 'usb_storage', 'USB Drive invokes storage');
    tap(wasm, 'back');
    assert(snapshot(wasm).activeScreen === 'settings' && snapshot(wasm).settingsPage === 2 && snapshot(wasm).settingsSelection === 1, 'USB Drive returns to its System entry');
    tap(wasm, 'down'); tap(wasm, 'ok');
    assert(snapshot(wasm).activeScreen === 'self_test', 'Self-Test is third');
    tap(wasm, 'back');
    assert(snapshot(wasm).settingsPage === 2 && snapshot(wasm).settingsSelection === 2, 'Self-Test returns to its System entry');
    tap(wasm, 'down');
    captureSelfTest('Settings - System update');
    tap(wasm, 'ok');
    assert(snapshot(wasm).activeScreen === 'firmware_update' && snapshot(wasm).firmwareUpdateSelection === 0, 'Update Firmware opens the target chooser');
    captureSelfTest('Update Firmware - Ground Station');
    tap(wasm, 'ok');
    assert(snapshot(wasm).activeScreen === 'bootloader' && snapshot(wasm).actions.some(action => action.type === 'bootloader_requested'), 'Ground Station invokes the existing bootloader');
  });

  await test('settings versions: startup cache, bounded retries and navigation', async () => {
    const setVersion = (link, firmwareVersion) => wasm.ccall('gs_set_link_json', null,
      ['string', 'string'], [String(link), JSON.stringify({firmwareVersion})]);
    const requests = link => snapshot(wasm).actions.filter(action => action.type === 'version_requested' && action.link === link).length;
    const bootVersions = (dualReceiver, first = '1.1.3', second = '2.0.0') => {
      wasm.ccall('gs_reset', null, [], []);
      wasm.ccall('gs_set_configuration_json', null, ['string'], [JSON.stringify({dualReceiver, startupAnimation: false})]);
      wasm.ccall('gs_restart', null, [], []);
      setVersion(1, first); setVersion(2, second);
      advanceSelfTest(2000);
      const startup = snapshot(wasm);
      assert(startup.telemetryVersion1 === (first || 'Reading...'), 'chip 1 acquired before opening Settings');
      assert(startup.telemetryVersion2 === (second || 'Reading...'), 'chip 2 acquired before opening Settings');
      for (const button of ['down', 'right', 'right', 'ok', 'right', 'right', 'down']) tap(wasm, button);
      const state = snapshot(wasm);
      assert(state.settingsPage === 2 && state.settingsSelection === 0, 'Firmware Versions item selected');
    };
    bootVersions(false);
    captureSelfTest('Settings versions - single mode');
    const before = framebuffer(wasm).slice();
    setVersion(1, '9.0.0'); setVersion(2, '');
    tap(wasm, 'ok');
    assert(before.every((value, index) => value === framebuffer(wasm)[index]), 'A leaves cached readout unchanged');
    tap(wasm, 'down'); tap(wasm, 'up');
    assert(before.every((value, index) => value === framebuffer(wasm)[index]), 'returning to Version restores cached values');
    wasm.ccall('gs_set_device_status_json', null, ['string'], ['{"usb":true}']);
    advanceSelfTest(4000);
    wasm.ccall('gs_set_device_status_json', null, ['string'], ['{"usb":false}']);
    advanceSelfTest(20);
    assert(requests(1) === 1 && requests(2) === 1, 'no reads from A, navigation, USB or elapsed time after success');
    tap(wasm, 'up');
    captureSelfTest('Settings after leaving Version');

    bootVersions(true);
    assert(snapshot(wasm).telemetryVersion2 === '2.0.0', 'both chips read in dual mode');
    captureSelfTest('Settings versions - dual mode');
    bootVersions(false, '1.1.3', '');
    captureSelfTest('Settings versions - reading');
    advanceSelfTest(6200);
    assert(snapshot(wasm).telemetryVersion2 === 'No response', 'missing chip times out independently');
    assert(requests(1) === 1 && requests(2) === 8, 'only the missing chip is retried');
    captureSelfTest('Settings versions - missing chip');
    setVersion(2, '2.0.1');
    tap(wasm, 'ok'); advanceSelfTest(4000);
    assert(snapshot(wasm).telemetryVersion2 === 'No response' && requests(2) === 8, 'no manual retry or ongoing polling after timeout');
    bootVersions(false, '1.1.3', '2.0.1');
    assert(snapshot(wasm).telemetryVersion2 === '2.0.1', 'restart acquires updated firmware version');
    captureSelfTest('Settings versions - recovered chip');
    bootVersions(false, '999.999.999-beta', '999.999.999-beta');
    captureSelfTest('Settings versions - long values');
    bootVersions(false, '', '');
    advanceSelfTest(6200);
    const state = snapshot(wasm);
    assert(state.telemetryVersion1 === 'No response' && state.telemetryVersion2 === 'No response', 'both missing chips shown');
    assert(requests(1) === 8 && requests(2) === 8, 'both startup retries bounded');
    captureSelfTest('Settings versions - both chips missing');
    bootVersions(false, '', '');
    advanceSelfTest(2600);
    setVersion(1, '1.1.3'); setVersion(2, '2.0.0');
    advanceSelfTest(1000);
    assert(snapshot(wasm).telemetryVersion1 === '1.1.3' && snapshot(wasm).telemetryVersion2 === '2.0.0', 'normal cold boot delay is covered');
    assert(requests(1) === 6 && requests(2) === 6, 'startup stops retrying after delayed replies');
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
