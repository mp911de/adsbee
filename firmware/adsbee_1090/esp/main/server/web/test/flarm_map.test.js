// Tests FLARM handling in adsbee.js: source, trails, altitude. Runs in CI (web_ui_test job).
// Run: node --test firmware/adsbee_1090/esp/main/server/web/test/flarm_map.test.js
'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const { AircraftStore, acSource, AircraftTable } = require('../adsbee.js');
const target = { hex: '~f09000', type: 'flarm', lat: 50, lon: 8, alt_geom: 3280.8,
    seen: 0, seen_pos: 0, fix_utc: 1600000000 };

test('FLARM uses its own source and adds one trail point per fix', () => {
    const store = new AircraftStore();
    store.ingest(target);
    store.ingest({ ...target, seen: 16, seen_pos: 16 });
    assert.equal(acSource(target), 'flarm');
    assert.equal(store.all()[0].srcLabel, 'FLARM');
    assert.equal(store.all()[0].seen_pos, 16);
    assert.equal(store.trails.get(target.hex).length, 1);
    assert.equal(store.trails.get(target.hex)[0].alt, 3280.8);
    store.ingest({ ...target, fix_utc: target.fix_utc + 1 });
    assert.equal(store.trails.get(target.hex).length, 2);
});

test('FLARM replacement clears old source fields and normal disconnect cleanup works', () => {
    const store = new AircraftStore();
    store.ingest({ hex: 'abcdef', type: 'adsb_icao', alt_baro: 7000, flight: 'OLD' });
    store.ingest({ ...target, hex: 'abcdef' });
    assert.equal(store.aircraft.get('abcdef').alt_baro, undefined);
    assert.equal(store.aircraft.get('abcdef').flight, undefined);
    store.clearAll();
    assert.equal(store.aircraft.size, 0); assert.equal(store.trails.size, 0);
});

test('table labels FLARM altitude as geometric', () => {
    const store = new AircraftStore(); store.ingest(target);
    const cells = Array.from({ length: 10 }, () => ({ style: {}, textContent: '' }));
    const row = { dataset: { key: target.hex }, querySelectorAll: () => cells,
        classList: { toggle() {} } };
    const tbody = { querySelectorAll: () => [row], querySelector: () => row, appendChild() {} };
    const table = Object.create(AircraftTable.prototype);
    Object.assign(table, { tbody, store, sortKey: 'hex', sortDir: 'asc' });
    table.update();
    assert.match(cells[4].textContent, / geom$/);
    assert.equal(cells[2].textContent, 'FLARM');
});

test('altitude sorting falls back to geometric altitude for any source', () => {
    const store = new AircraftStore(); store.ingest(target);
    store.ingest({ hex: 'abcdef', type: 'adsb_icao', alt_baro: 1000 });
    store.ingest({ hex: '123456', type: 'adsb_icao', alt_geom: 2000 });
    const table = Object.create(AircraftTable.prototype);
    Object.assign(table, { store, sortKey: 'alt_baro', sortDir: 'asc' });
    assert.deepEqual(table._sorted().map(ac => ac.hex), ['abcdef', '123456', target.hex]);
});
