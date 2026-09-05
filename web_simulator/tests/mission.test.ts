import assert from 'node:assert/strict';
import test from 'node:test';
import {
  advance,
  clearance,
  collisionReason,
  createWorld,
  FIELD,
  finish,
  inside,
  LAB_SLOTS,
  robotShapes,
  scoreWorld,
  SPEC,
  ZONES,
  type Item,
} from '../lib/mission.ts';
import { polygonDistance, rectanglePolygon } from '../lib/simulation.ts';

function solvedItems(): Item[] {
  const w = createWorld();
  for (const robot of w.robots)
    for (const job of robot.jobs) {
      const item = w.items.find((i) => i.id === job.itemId)!;
      Object.assign(item, job.drop, { carrier: null, released: true });
    }
  return w.items;
}

void test('default fleet is one hamster and three beavers with position telemetry and no drone', () => {
  const w = createWorld();
  assert.deepEqual(w.robots.map((r) => r.id).sort(), ['B1', 'B2', 'H1', 'H2']);
  assert.equal(w.robots.filter((r) => r.role === 'hamster').length, 1);
  assert.equal(w.robots.filter((r) => r.role === 'beaver').length, 3);
  assert.equal(w.observer.mode, 'localization');
  assert.equal(w.drone.enabled, false);
  assert.equal(createWorld(false).drone.enabled, false);
  assert.equal(createWorld('drone').drone.enabled, true);
  assert.equal(createWorld(true).observer.mode, 'drone');
  assert.equal(w.items.length, 19);
  assert.equal(w.robots.flatMap((r) => r.jobs).length, 16);
  assert.equal(
    new Set(w.robots.flatMap((r) => r.jobs.map((j) => j.itemId))).size,
    16,
  );
  for (const r of w.robots)
    for (const job of r.jobs) {
      const item = w.items.find((i) => i.id === job.itemId)!;
      assert.equal(item.kind === 'disc', r.role === 'hamster');
    }
  for (const r of w.robots.filter((r) => r.role === 'beaver'))
    assert.equal(r.magazine.length, r.id === 'H2' ? 0 : 2);
  assert.equal(
    w.items.filter((i) => i.carrier).every((i) => i.kind === 'cube'),
    true,
  );
  assert.equal(scoreWorld(w.items).points, 0);
});

void test('assumed 150mm Bat and four nominal bodies fit the 480 by 280mm start', () => {
  const w = createWorld('drone');
  const start = FIELD.startZone;
  const bat = rectanglePolygon({
    x: w.drone.pose.x - SPEC.droneWidth / 2,
    y: w.drone.pose.y - SPEC.droneLength / 2,
    width: SPEC.droneWidth,
    height: SPEC.droneLength,
  });
  for (const shape of [
    bat,
    ...w.robots.flatMap((r) => robotShapes(r, w.items)),
  ]) {
    for (const p of shape) {
      assert.ok(p.x >= start.x - 1e-9 && p.x <= start.x + start.width + 1e-9);
      assert.ok(p.y >= start.y - 1e-9 && p.y <= start.y + start.height + 1e-9);
    }
  }
  for (const r of w.robots)
    for (const body of robotShapes(r, w.items))
      assert.ok(polygonDistance(body, bat) >= SPEC.margin);
  assert.ok(clearance(w) >= SPEC.margin);
});

void test('final correct snapshot earns 30 disc + 40 cube + 90 cylinder = 160', () => {
  const result = scoreWorld(solvedItems());
  assert.equal(result.points, 160);
  assert.deepEqual(result.counts, {
    discs: 3,
    cubes: 4,
    red: 3,
    yellow: 3,
    green: 3,
  });
  assert.equal(result.delivered, 16);
});

void test('a disc needs its own LAB circle and strictly less than 2mm center error', () => {
  const items = solvedItems(),
    disc = items.find((i) => i.id === 'D1')!;
  disc.x = LAB_SLOTS[0].x + 0.0019;
  assert.equal(scoreWorld(items).counts.discs, 3);
  disc.x = LAB_SLOTS[0].x + 0.002;
  assert.equal(scoreWorld(items).counts.discs, 2);
  Object.assign(disc, LAB_SLOTS[1]);
  assert.equal(scoreWorld(items).counts.discs, 2);
  disc.x = NaN;
  assert.equal(scoreWorld(items).counts.discs, 2);
});

void test('held objects and boundary-touching footprints never score', () => {
  const items = solvedItems(),
    red = items.find((i) => i.id === 'R1')!;
  red.carrier = 'B1';
  assert.equal(scoreWorld(items).counts.red, 2);
  red.carrier = null;
  red.released = false;
  assert.equal(scoreWorld(items).counts.red, 2);
  red.released = true;
  red.x = ZONES.H.x + 0.01;
  assert.equal(inside(red, ZONES.H), false);
  assert.equal(scoreWorld(items).counts.red, 2);
  const cube = items.find((i) => i.id === 'C1')!;
  cube.x = ZONES.H.x + 0.0125;
  assert.equal(scoreWorld(items).counts.cubes, 3);
});

void test('wrong-color cylinder invalidates only that zone cylinder score, not its cubes', () => {
  const items = solvedItems(),
    spare = items.find((i) => i.id === 'G4')!;
  Object.assign(spare, { x: 0.73, y: 1.06, released: true });
  const result = scoreWorld(items);
  assert.deepEqual(result.contaminants, ['H']);
  assert.equal(result.counts.red, 0);
  assert.equal(result.counts.cubes, 4);
  assert.equal(result.counts.yellow, 3);
});

void test('yellow split is physical: contaminated left PCC loses only left cylinder points', () => {
  const items = solvedItems(),
    wrong = items.find((i) => i.id === 'R4')!;
  Object.assign(wrong, { x: 0.22, y: 1.04, released: true });
  const result = scoreWorld(items);
  assert.equal(result.yellowSplit, true);
  assert.equal(result.counts.yellow, 2);
  assert.equal(result.counts.cubes, 4);
});

void test('putting all yellow in one PCC yields zero yellow points', () => {
  const items = solvedItems();
  items
    .filter((i) => i.color === 'yellow' && i.selected)
    .forEach((item, i) => Object.assign(item, { x: 0.07 + i * 0.04, y: 1.05 }));
  assert.equal(scoreWorld(items).counts.yellow, 0);
});

void test('duplicate IDs are rejected and scoring is not a delivery history counter', () => {
  const items = solvedItems();
  assert.throws(() => scoreWorld([...items, items[0]]), /Duplicate/);
  items.find((i) => i.id === 'D1')!.x = 0.9;
  assert.equal(scoreWorld(items).points, 150);
});

void test('the entire default mission carries all 16 objects continuously with 12mm model clearance', () => {
  const w = createWorld();
  let smallestGap = Infinity;
  for (let i = 0; i < 6001 && !w.ended; i++) {
    const previous = w.items.map((p) => ({ x: p.x, y: p.y }));
    advance(w);
    smallestGap = Math.min(smallestGap, clearance(w));
    for (const robot of w.robots)
      assert.equal(
        collisionReason(w, robot),
        null,
        `${w.elapsed} ${robot.id} ${robot.phase}`,
      );
    w.items.forEach((item, index) =>
      assert.ok(
        Math.hypot(item.x - previous[index].x, item.y - previous[index].y) <
          0.011,
        `${item.id} jumped`,
      ),
    );
  }
  assert.ok(w.ended && w.elapsed < 120);
  assert.ok(w.robots.every((r) => r.phase === 'complete'));
  assert.ok(smallestGap >= SPEC.margin - 1e-8);
  assert.equal(scoreWorld(w.items).points, 160);
  assert.equal(
    w.robots.reduce((sum, r) => sum + r.served, 0),
    16,
  );
  assert.ok(w.robots.every((r) => r.velocity.x === 0 && r.velocity.y === 0));
});

void test('optional Bat mode completes the same ground mission with one flying observer', () => {
  const w = createWorld('drone');
  for (let i = 0; i < 6001 && !w.ended; i++) advance(w);
  assert.equal(scoreWorld(w.items).points, 160);
  assert.ok(w.drone.altitude >= 0.6 && w.drone.altitude <= 1.0);
});

void test('an optical sensor timeout cannot attach or score a disc', () => {
  const w = createWorld();
  w.faultRobot = 'H1';
  for (
    let i = 0;
    i < 1500 && w.robots.find((r) => r.id === 'H1')!.phase !== 'fault';
    i++
  )
    advance(w);
  const hamster = w.robots.find((r) => r.id === 'H1')!;
  assert.equal(hamster.phase, 'fault');
  assert.equal(hamster.payload, null);
  assert.equal(w.items.find((i) => i.id === 'D1')!.carrier, null);
  assert.equal(scoreWorld(w.items).counts.discs, 0);
});

void test('release confirmation timeout keeps payload attached and unscored', () => {
  const w = createWorld();
  const hamster = w.robots.find((r) => r.id === 'H1')!;
  for (let i = 0; i < 2000 && hamster.phase !== 'verify-release'; i++)
    advance(w);
  assert.equal(hamster.phase, 'verify-release');
  w.faultRobot = 'H1';
  for (let i = 0; i < 100; i++) advance(w);
  assert.equal(hamster.phase, 'fault');
  assert.equal(w.items.find((i) => i.id === 'D1')!.carrier, 'H1');
  assert.equal(scoreWorld(w.items).counts.discs, 0);
});

void test('0.5s stale observer feedback freezes ground motion but not the match deadline', () => {
  const w = createWorld();
  for (let i = 0; i < 450; i++) advance(w);
  assert.equal(w.observer.mode, 'localization');
  assert.equal(w.drone.enabled, false);
  w.observer.lost = true;
  for (let i = 0; i < 30; i++) advance(w);
  const poses = w.robots.map((r) => ({ ...r.pose })),
    elapsed = w.elapsed;
  for (let i = 0; i < 50; i++) advance(w);
  assert.deepEqual(
    w.robots.map((r) => r.pose),
    poses,
  );
  assert.ok(w.elapsed > elapsed);
  assert.ok(w.observer.frameAge > 0.5);
});

void test('deadline and early declaration freeze a final snapshot; invalid dt is ignored', () => {
  const w = createWorld();
  advance(w, NaN);
  advance(w, -1);
  assert.equal(w.elapsed, 0);
  w.elapsed = 119.99;
  advance(w);
  assert.equal(w.elapsed, 120);
  assert.equal(w.ended, true);
  const snapshot = JSON.stringify(w);
  advance(w);
  assert.equal(JSON.stringify(w), snapshot);
  const early = createWorld();
  finish(early);
  const stopped = JSON.stringify(early);
  advance(early);
  assert.equal(JSON.stringify(early), stopped);
});
