import assert from 'node:assert/strict';
import test from 'node:test';
import {
  advance,
  collisionReason,
  clearance,
  scoreWorld,
  SPEC,
} from '../lib/mission.ts';
import { createExperiment, experimentReport } from '../lib/experiments.ts';
import { parsePositionLog } from '../lib/localization.ts';

void test('five-robot congestion fixture yields and relocates a parked body, all 16 items delivered without teleport/contact', () => {
  const w = createExperiment('localization', 5);
  let minimum = Infinity;
  while (!w.ended) {
    const before = w.robots.map((r) => ({ ...r.pose }));
    const objects = w.items.map((i) => ({ x: i.x, y: i.y }));
    advance(w);
    minimum = Math.min(minimum, clearance(w));
    w.robots.forEach((r, i) => {
      assert.equal(collisionReason(w, r), null, `${w.elapsed}: ${r.id}`);
      assert.ok(
        Math.hypot(r.pose.x - before[i].x, r.pose.y - before[i].y) <=
          SPEC.speed * SPEC.dt + 1e-8,
      );
    });
    w.items.forEach((r, i) =>
      assert.ok(Math.hypot(r.x - objects[i].x, r.y - objects[i].y) < 0.011),
    );
  }
  assert.ok(w.elapsed < 120);
  assert.equal(scoreWorld(w.items).points, 160);
  assert.ok(w.robots.every((r) => r.phase === 'complete'));
  assert.ok(w.robots.find((r) => r.id === 'extra5')!.recoveryAttempts > 0);
  assert.ok(minimum >= SPEC.margin - 1e-8);
  assert.equal(experimentReport(w).device_io, false);
  assert.throws(() => createExperiment('drone', 5), /parking footprint/);
});

void test('delayed observations contain past positions; 400ms input cannot move a robot', () => {
  const w = createExperiment('localization');
  w.observer.delayMs = 100;
  for (let i = 0; i < 300; i++) advance(w);
  assert.ok(w.observer.frameAge >= 0.1 - 1e-9 && w.observer.frameAge < 0.21);
  assert.notEqual(w.observer.poses.B1.y_mm, w.robots[0].pose.y * 1000);
  w.observer.delayMs = 400;
  w.observer.queue = [];
  for (let i = 0; i < 30; i++) advance(w);
  const poses = w.robots.map((r) => ({ ...r.pose }));
  for (let i = 0; i < 100; i++) advance(w);
  assert.deepEqual(
    w.robots.map((r) => r.pose),
    poses,
  );
  assert.match(w.safetyReason, /300ms/);
  assert.ok(w.observer.queue.length <= 20);
  w.observer.delayMs = 0;
  advance(w);
  assert.equal(w.safetyReason, '');
});

void test('bounded coordinate jitter is real telemetry, excessive error holds; e-stop stays latched', () => {
  const w = createExperiment('localization');
  w.observer.noiseMm = 0.5;
  advance(w);
  const p = w.observer.poses.B1,
    r = w.robots[0];
  assert.ok(
    Math.abs(
      Math.hypot(p.x_mm - r.pose.x * 1000, p.y_mm - r.pose.y * 1000) - 0.5,
    ) < 1e-8,
  );
  w.observer.noiseMm = 3;
  const before = w.robots.map((r) => ({ ...r.pose }));
  for (let i = 0; i < 300; i++) advance(w);
  assert.deepEqual(
    w.robots.map((r) => r.pose),
    before,
  );
  assert.match(w.safetyReason, /1mm/);
  w.emergencyStopped = true;
  w.observer.noiseMm = 0;
  for (let i = 0; i < 100; i++) advance(w);
  assert.deepEqual(
    w.robots.map((r) => r.pose),
    before,
  );
  assert.match(w.safetyReason, /비상정지/);
  assert.equal(createExperiment('localization').emergencyStopped, false);
});

void test('100ms / 0.5mm observation-health scenario preserves ideal-geometry mission, not a physical precision claim', () => {
  const w = createExperiment('localization');
  w.observer.delayMs = 100;
  w.observer.noiseMm = 0.5;
  while (!w.ended) advance(w);
  assert.equal(scoreWorld(w.items).points, 160);
  assert.ok(w.elapsed < 120);
});

void test('object-track JSONL shows stable identity, ambiguity and owner instead of raw duplicates', () => {
  const row = {
    status: 'detected',
    source_name: 'test',
    sequence: 1,
    captured_at_s: 1,
    robots: [],
    objects: [],
    object_tracks: [
      {
        object_id: 'O0001',
        color: 'red',
        kind: 'cylinder',
        center_mm: [300, 400],
        state: 'ambiguous',
        identity_uncertain: true,
        owner_robot_id: 'B1',
      },
    ],
  };
  const result = parsePositionLog(JSON.stringify(row))[0].objects;
  assert.equal(result[0].id, 'O0001');
  assert.equal(result[0].uncertain, true);
  assert.equal(result[0].owner, 'B1');
  row.object_tracks.push(row.object_tracks[0]);
  assert.throws(() => parsePositionLog(JSON.stringify(row)), /물체 ID/);
});
