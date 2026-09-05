import assert from 'node:assert/strict';
import test from 'node:test';
import {
  advance,
  createWorld,
  smoothObservedPath,
  planPath,
  routeSegmentSafe,
  optimizePendingJobs,
} from '../lib/mission.ts';
import { createComparison, runComparison } from '../lib/experiments.ts';

void test('ordinary no-drone construction never injects a failure, including after a failed aerial test', () => {
  const broken = createComparison('none', 'occlusion');
  for (let i = 0; i < 220; i++) advance(broken);
  assert.ok(broken.safetyReason);
  const clean = createComparison('none');
  assert.equal(clean.scenario, 'normal');
  assert.equal(clean.observer.missingId, null);
  assert.equal(clean.drone.occlusionId, null);
  assert.equal(clean.drone.enabled, false);
  for (let i = 0; i < 230; i++) advance(clean);
  assert.equal(clean.safetyReason, '');
  assert.ok(clean.robots[0].distanceTravelled > 0);
});

void test('shared optimization improves the healthy mission without granting a drone speed bonus', () => {
  const baseline = runComparison('none', 'normal', false);
  const ground = runComparison('none', 'normal');
  const air = runComparison('active', 'normal');
  for (const result of [baseline, ground, air]) {
    assert.equal(result.score, 160);
    assert.equal(result.completed, true);
    assert.equal(result.collisions, 0);
    assert.ok(result.minimumClearanceMm >= 12 - 1e-6);
  }
  assert.ok(ground.time < baseline.time - 10);
  assert.ok(ground.distance < baseline.distance);
  assert.equal(ground.time, air.time);
  assert.equal(ground.distance, air.distance);
  assert.ok(
    ground.shortcuts > 0 && ground.staging > 0 && ground.taskReorders > 0,
  );
});

void test('identical intermittent input failure still completes without a drone; aerial input reduces holds', () => {
  const ground = runComparison('none', 'intermittent');
  const air = runComparison('active', 'intermittent');
  for (const result of [ground, air]) {
    assert.equal(result.score, 160);
    assert.ok(result.completed);
    assert.equal(result.collisions, 0);
    assert.ok(result.minimumClearanceMm >= 12 - 1e-6);
  }
  assert.ok(ground.inputHold > 4);
  assert.ok(air.inputHold < ground.inputHold);
  assert.ok(air.aerialRecovery > 4);
  assert.ok(air.time < ground.time);
});

void test('smoothing preserves goal and heading and validates every loaded swept segment', () => {
  const w = createWorld();
  advance(w);
  const r = w.robots[0],
    goal = { x: 0.48, y: 1.025 };
  const original = planPath(w, r, goal);
  assert.ok(original.length > 1);
  const result = smoothObservedPath(w, r, original);
  assert.ok(result.length > 0 && result.length <= original.length);
  assert.deepEqual(result.at(-1), goal);
  const length = (p: { x: number; y: number }[]) =>
    p.reduce(
      (sum, v, i) =>
        sum +
        Math.hypot(
          v.x - (i ? p[i - 1] : r.pose).x,
          v.y - (i ? p[i - 1] : r.pose).y,
        ),
      0,
    );
  assert.ok(length(result) <= length(original) + 1e-10);
  let start = { ...r.pose };
  for (const end of result) {
    assert.ok(routeSegmentSafe(w, r, start, end));
    start = { ...end, heading: r.pose.heading };
  }
  // A newly inserted object on the former route invalidates that shortcut.
  w.items.push({
    id: 'thin-new-obstacle',
    kind: 'cylinder',
    selected: false,
    carrier: null,
    released: false,
    x: (r.pose.x + result[0].x) / 2,
    y: (r.pose.y + result[0].y) / 2,
  });
  assert.equal(routeSegmentSafe(w, r, r.pose, result[0]), false);
  assert.deepEqual(smoothObservedPath(w, r, [result[0], result[0]]), []);
});

void test('stale positions disable new optimization and task search preserves started work, slots and magazine', () => {
  const w = createWorld();
  advance(w);
  const r = w.robots.find((r) => r.id === 'H2')!;
  const jobs = structuredClone(r.jobs),
    ids = jobs.map((j) => j.itemId).sort();
  optimizePendingJobs(w, r);
  assert.deepEqual(r.jobs.map((j) => j.itemId).sort(), ids);
  for (const job of r.jobs)
    assert.deepEqual(
      job,
      jobs.find((j) => j.itemId === job.itemId),
    );
  const cubeRobot = w.robots[0],
    magazineJobs = structuredClone(cubeRobot.jobs);
  optimizePendingJobs(w, cubeRobot);
  assert.deepEqual(cubeRobot.jobs, magazineJobs);
  w.elapsed = 1;
  const before = structuredClone(r.jobs),
    plans = w.coordination.taskPlans;
  optimizePendingJobs(w, r);
  assert.deepEqual(r.jobs, before);
  assert.equal(w.coordination.taskPlans, plans);
  const path = [
    { x: 0.5, y: 0.5 },
    { x: 0.6, y: 0.6 },
  ];
  assert.equal(smoothObservedPath(w, r, path), path);
});

void test('the initial map does not silently follow hidden object changes', () => {
  const w = createWorld('drone');
  const initial = structuredClone(w.initialItems);
  w.items.find((i) => i.id === 'G2')!.x += 0.1;
  assert.deepEqual(w.initialItems, initial);
});

void test('capture-based repeated detection confirms despite 200ms delivery delay and resets after a missed capture', () => {
  const w = createWorld('drone');
  w.drone.delayMs = 200;
  for (let i = 0; i < 160; i++) advance(w);
  assert.ok(Object.values(w.drone.objects).some((p) => p.confirmed));
  w.drone.calibrationLost = true;
  for (let i = 0; i < 6; i++) advance(w);
  assert.deepEqual(w.drone.captureStreaks, {});
});
