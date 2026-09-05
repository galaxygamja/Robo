import assert from 'node:assert/strict';
import test from 'node:test';
import {
  createWorld,
  advance,
  collisionReason,
  scoreWorld,
  clearance,
  SPEC,
} from '../lib/mission.ts';
import {
  AERIAL,
  canObserve,
  calibrationAt,
  chooseView,
  rayBlocked,
  sceneOccluders,
  selectObservations,
} from '../lib/aerial.ts';

void test('three-dimensional ray and FOV distinguish an overhead obstruction from a lateral clear view', () => {
  const w = createWorld('drone');
  w.drone.occlusionId = 'H2';
  const boxes = sceneOccluders(w),
    target = { x: 1.055, y: 0.065, z: 0.075 };
  const central = { x: 0.5715, y: 0.61, z: 0.8 },
    side = { x: 0.38, y: 0.61, z: 0.8 };
  assert.equal(rayBlocked(central, target, boxes[0]), false); // centre clear but tag corners are covered
  assert.equal(canObserve(central, target, boxes), false);
  assert.equal(canObserve(side, target, boxes), true);
  assert.equal(calibrationAt(side, boxes).valid, true);
  assert.equal(canObserve({ ...side, z: 0.05 }, target, boxes), false);
  assert.equal(canObserve(side, { ...target, x: 5 }, boxes), false);
  assert.equal(calibrationAt({ x: 0.1, y: 0.1, z: 0.15 }).valid, false);
});

void test('viewpoint priority finds a visible missing tag while retaining spread field anchors', () => {
  const w = createWorld('drone');
  w.drone.occlusionId = 'H2';
  const camera = { x: 0.5715, y: 0.61, z: 0.8 },
    target = { x: 1.055, y: 0.065, z: 0.075, id: 'H2', size: 0.04, weight: 40 };
  const boxes = sceneOccluders(w),
    chosen = chooseView(camera, [target], boxes);
  assert.ok(chosen.visible.includes('H2'));
  assert.ok(calibrationAt(chosen.pose, boxes).valid);
  assert.ok(chosen.pose.z >= AERIAL.minZ && chosen.pose.z <= AERIAL.maxZ);
});

void test('identical 12s missing-tag/hood scene: active drone recovers while central hover and no drone hold', () => {
  const results = [];
  for (const strategy of ['none', 'hover', 'active'] as const) {
    const w = createWorld(strategy === 'none' ? 'localization' : 'drone');
    w.observer.missingId = 'H2';
    w.drone.occlusionId = 'H2';
    if (strategy !== 'none') w.drone.strategy = strategy;
    while (w.elapsed < 12 - 1e-8) {
      const previous = { ...w.drone.pose, z: w.drone.altitude };
      advance(w);
      assert.ok(
        Math.hypot(w.drone.pose.x - previous.x, w.drone.pose.y - previous.y) <=
          AERIAL.speed * SPEC.dt + 1e-9,
      );
      assert.ok(
        Math.abs(w.drone.altitude - previous.z) <=
          AERIAL.climb * SPEC.dt + 1e-9,
      );
      for (const r of w.robots) assert.equal(collisionReason(w, r), null);
    }
    results.push(w);
  }
  assert.ok(results[0].drone.holdSeconds > 11.9);
  assert.ok(results[1].drone.holdSeconds > 11.9);
  assert.ok(results[2].drone.holdSeconds < 5);
  assert.ok(results[2].drone.recoveredRobotSeconds > 7);
  assert.equal(results[2].observer.poses.H2.source, 'drone');
  assert.ok(results[2].robots[0].served > 0);
});

void test('active drone keeps the entire mission collision-free after recovering the fixed blind spot', () => {
  const w = createWorld('drone');
  w.observer.missingId = 'H2';
  w.drone.occlusionId = 'H2';
  let minimum = Infinity;
  while (!w.ended) {
    advance(w);
    minimum = Math.min(minimum, clearance(w));
    for (const r of w.robots) assert.equal(collisionReason(w, r), null);
  }
  assert.ok(w.elapsed < 120);
  assert.equal(scoreWorld(w.items).points, 160);
  assert.ok(w.drone.recoveredRobotSeconds > w.elapsed - 5);
  assert.ok(minimum >= SPEC.margin - 1e-8);
});

void test('source fallback never rolls a robot back to an older timestamp', () => {
  const w = createWorld('drone');
  w.elapsed = 1.02;
  w.drone.calibrationValid = true;
  for (const r of w.robots)
    w.observer.basePoses[r.id] = {
      source: 'base',
      at: 0.95,
      x_mm: r.pose.x * 1000,
      y_mm: r.pose.y * 1000,
      heading_rad: 0,
      errorMm: 0.5,
    };
  w.drone.observations.B1 = {
    ...w.observer.basePoses.B1,
    source: 'drone',
    at: 1,
  };
  w.drone.visibleRobots = ['B1'];
  selectObservations(w, 0.02);
  assert.equal(w.observer.poses.B1.source, 'drone');
  w.drone.videoLost = true;
  selectObservations(w, 0.02);
  assert.ok(w.observer.unavailableIds.includes('B1'));
  assert.equal(w.observer.poses.B1.at, 1);
  w.observer.basePoses.B1.at = 1.02;
  selectObservations(w, 0.02);
  assert.equal(w.observer.poses.B1.source, 'base');
});

void test('contradictory simultaneous sources do not become a fused robot location', () => {
  const w = createWorld('drone');
  w.elapsed = 1;
  w.drone.calibrationValid = true;
  for (const r of w.robots)
    w.observer.basePoses[r.id] = {
      source: 'base',
      at: 1,
      x_mm: r.pose.x * 1000,
      y_mm: r.pose.y * 1000,
      heading_rad: 0,
      errorMm: 0.5,
    };
  w.drone.observations.B1 = {
    ...w.observer.basePoses.B1,
    source: 'drone',
    x_mm: 100,
  };
  w.drone.visibleRobots = ['B1'];
  selectObservations(w, 0.02);
  assert.deepEqual(w.observer.conflictingIds, ['B1']);
  assert.ok(w.observer.unavailableIds.includes('B1'));
  assert.equal(w.observer.poses.B1, undefined);
});

const recovered = () => {
  const w = createWorld('drone');
  w.observer.missingId = 'H2';
  w.drone.occlusionId = 'H2';
  for (let i = 0; i < 300; i++) advance(w);
  assert.ok(w.drone.recoveredIds.includes('H2'));
  return w;
};

void test('per-frame calibration failure and video loss cannot reuse an earlier valid drone observation', () => {
  for (const fault of ['calibrationLost', 'videoLost'] as const) {
    const w = recovered(),
      last = w.drone.observations.H2.at;
    w.drone[fault] = true;
    const before = w.robots.map((r) => ({ ...r.pose }));
    for (let i = 0; i < 20; i++) advance(w);
    assert.deepEqual(
      w.robots.map((r) => r.pose),
      before,
    );
    assert.ok(w.observer.unavailableIds.includes('H2'));
    assert.equal(w.drone.observations.H2.at, last);
    assert.equal(w.drone.recoveredIds.length, 0);
    w.observer.missingId = null;
    for (let i = 0; i < 10; i++) advance(w);
    assert.equal(w.safetyReason, '');
    assert.equal(w.observer.poses.H2.source, 'base');
  }
});

void test('450ms aerial packets stay stale and cannot revive a missing robot; queues remain bounded', () => {
  const w = recovered();
  w.drone.delayMs = 450;
  w.drone.queue = [];
  for (let i = 0; i < 30; i++) advance(w);
  const before = w.robots.map((r) => ({ ...r.pose }));
  for (let i = 0; i < 100; i++) advance(w);
  assert.deepEqual(
    w.robots.map((r) => r.pose),
    before,
  );
  assert.ok(w.observer.unavailableIds.includes('H2'));
  assert.ok(w.drone.queue.length <= 20);
  w.drone.delayMs = 80;
  w.drone.queue = [];
  for (let i = 0; i < 50; i++) advance(w);
  assert.ok(w.drone.recoveredIds.includes('H2'));
});

void test('emergency stop and match finish freeze drone position as well as ground robots', () => {
  const w = recovered();
  w.emergencyStopped = true;
  const before = { pose: { ...w.drone.pose }, z: w.drone.altitude };
  for (let i = 0; i < 50; i++) advance(w);
  assert.deepEqual({ pose: w.drone.pose, z: w.drone.altitude }, before);
  assert.equal(w.drone.phase, 'hold');
  w.elapsed = 119.99;
  advance(w);
  const snapshot = JSON.stringify(w);
  advance(w);
  assert.equal(JSON.stringify(w), snapshot);
});

void test('route cost uses the known map and fresh inventory without changing task ownership or cube order', () => {
  const w = createWorld('drone');
  w.elapsed = 14;
  w.drone.pose = { x: 0.57, y: 0.61 };
  w.drone.altitude = 0.8;
  w.drone.lastSampleAt = 14;
  w.drone.lastPlanAt = 14;
  w.drone.objects.Y2 = {
    id: 'Y2',
    x_mm: 350,
    y_mm: 531,
    at: 14,
    streak: 3,
    confirmed: true,
    kind: 'cylinder',
    color: 'yellow',
  };
  const h2 = w.robots.find((r) => r.id === 'H2')!,
    ids = h2.jobs.map((j) => j.itemId).toSorted();
  advance(w);
  assert.equal(h2.jobs[0].itemId, 'G2');
  assert.equal(h2.taskReorders, 1);
  assert.deepEqual(h2.jobs.map((j) => j.itemId).toSorted(), ids);
  assert.equal(w.robots[0].jobs[0].itemId, 'C1');
});
