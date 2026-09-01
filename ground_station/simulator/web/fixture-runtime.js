export const CSV_HEADER = 'link,ts[deciseconds],state,errors,lat[deg/10000],lon[deg/10000],altitude[m],velocity[m/s],battery[decivolts],pyro1,pyro2\n';
export const DECISECOND_MS = 100;

export async function loadFixtureManifest(url = './fixtures.json') {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`fixtures: HTTP ${response.status}`);
  const manifest = await response.json();
  if (manifest.schemaVersion !== 1 || !Array.isArray(manifest.fixtures)) {
    throw new Error('Unsupported simulator fixture manifest.');
  }
  return manifest;
}

export function fixtureById(manifest, id) {
  const fixture = manifest.fixtures.find(candidate => candidate.id === id);
  if (!fixture) throw new Error(`Unknown simulator fixture: ${id}`);
  return fixture;
}

export function parseReplayCsv(csv) {
  const rows = [];
  for (const [order, line] of csv.split(/\r?\n/).entries()) {
    if (!line.trim() || line.toLowerCase().startsWith('link,')) continue;
    const fields = line.split(',').map(field => field.trim());
    if (fields.length < 11) continue;
    const values = fields.slice(0, 11).map(Number);
    if (values.some(value => !Number.isFinite(value)) || ![1, 2].includes(values[0])) continue;
    rows.push({
      link: values[0], timestampDs: values[1], state: values[2], errors: values[3],
      latitude: values[4] / 10000, longitude: values[5] / 10000,
      altitudeM: values[6], velocityMps: values[7], voltage: values[8] / 10,
      pyroContinuity: (values[9] ? 1 : 0) | (values[10] ? 2 : 0),
      linkQuality: 100, rssi: -50, snr: 10, connected: true, order
    });
  }
  rows.sort((left, right) => left.timestampDs - right.timestampDs || left.order - right.order);
  if (!rows.length) return rows;
  const firstTimestamp = rows[0].timestampDs;
  return rows.map(({ order, ...row }) => ({
    ...row,
    elapsedMs: (row.timestampDs - firstTimestamp) * DECISECOND_MS
  }));
}

export function injectReplayRow(wasm, row) {
  const { link, elapsedMs, ...value } = row;
  wasm.ccall('gs_set_link_json', null, ['string', 'string'], [String(link), JSON.stringify(value)]);
}

function sendJson(wasm, name, value) {
  wasm.ccall(name, null, ['string'], [JSON.stringify(value)]);
}

function completeLog(name, offset, spec) {
  const rows = [];
  for (const link of [1, 2]) {
    const latitude = spec.baseLatitudeE4 + offset + link * 5;
    const longitude = spec.baseLongitudeE4 + offset + link * 5;
    rows.push(`${link},10,3,0,${latitude},${longitude},100,30,42,1,1`);
    rows.push(`${link},20,5,0,${latitude},${longitude},1000,0,41,1,1`);
    rows.push(`${link},30,6,0,${latitude},${longitude},400,-25,40,1,1`);
    rows.push(`${link},50,7,0,${latitude},${longitude},5,0,39,1,1`);
  }
  return { name, csv: CSV_HEADER + rows.join('\n') + '\n' };
}

async function resolveLogs(spec, lightMode) {
  if (Array.isArray(spec)) return spec;
  if (spec.files) {
    const files = lightMode && spec.lightFiles ? spec.lightFiles : spec.files;
    return Promise.all(files.map(async file => {
      const response = await fetch(`./${file}`);
      if (!response.ok) throw new Error(`${file}: HTTP ${response.status}`);
      return { name: file.split('/').pop(), csv: await response.text() };
    }));
  }
  if (spec.generated) {
    const generated = spec.generated;
    return Array.from({ length: generated.count }, (_, offset) =>
      completeLog(`log_${String(generated.startIndex - offset).padStart(3, '0')}.csv`, offset, generated));
  }
  if (spec.entries) {
    return spec.entries.map(entry => ({
      name: entry.name,
      csv: CSV_HEADER + entry.rows.join('\n') + '\n'
    }));
  }
  throw new Error('Fixture logs must be an array, file list, generated set, or entry list.');
}

export async function applyFixtureToWasm(wasm, fixture, { lightMode = false, reset = true } = {}) {
  if (reset) wasm.ccall('gs_reset', null, [], []);
  let logs = [];
  for (const step of fixture.operations) {
    const entries = Object.entries(step);
    if (entries.length !== 1) throw new Error(`Fixture ${fixture.id} has an invalid operation.`);
    const [operation, value] = entries[0];
    if (operation === 'logs') {
      logs = await resolveLogs(value, lightMode);
      sendJson(wasm, 'gs_set_logs_json', logs);
    } else if (operation === 'configuration') {
      sendJson(wasm, 'gs_set_configuration_json', value);
    } else if (operation === 'navigation') {
      sendJson(wasm, 'gs_set_navigation_json', value);
    } else if (operation === 'deviceStatus') {
      sendJson(wasm, 'gs_set_device_status_json', value);
    } else if (operation === 'link') {
      wasm.ccall('gs_set_link_json', null, ['string', 'string'], [String(value.index), JSON.stringify(value.value)]);
    } else if (operation === 'tap') {
      wasm.ccall('gs_press', null, ['string'], [value]);
      wasm.ccall('gs_release', null, ['string'], [value]);
    } else if (operation === 'hold') {
      wasm.ccall('gs_hold', null, ['string', 'number'], [value.button, value.ms]);
    } else if (operation === 'advance') {
      wasm.ccall('gs_advance', null, ['number'], [value]);
    } else {
      throw new Error(`Fixture ${fixture.id} uses unknown operation: ${operation}`);
    }
  }
  if (lightMode && fixture.lightAdvanceMs) {
    wasm.ccall('gs_advance', null, ['number'], [fixture.lightAdvanceMs]);
  }
  return { logs };
}
