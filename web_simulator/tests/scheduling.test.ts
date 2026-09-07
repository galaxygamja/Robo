import assert from 'node:assert/strict';
import test from 'node:test';
import { advance, createWorld, scoreWorld, SPEC } from '../lib/mission.ts';
import {
  applySchedule,
  captureSchedule,
  evaluateSchedule,
  scheduleSignature,
} from '../lib/scheduling.ts';
import { applyReferenceSchedule } from '../lib/optimized-world.ts';
import { referenceSchedule } from '../lib/reference-schedule.ts';
import {
  createComparison,
  createExperiment,
  runComparison,
} from '../lib/experiments.ts';

void test('cached winner reproduces its measured full-score finish in the exact engine', () => {
  assert.ok(referenceSchedule);
  const original = createWorld();
  assert.equal(referenceSchedule.signature, scheduleSignature(original));
  const before = evaluateSchedule(
    original,
    captureSchedule(original, false),
  ).measurement;
  const after = evaluateSchedule(
    original,
    referenceSchedule.schedule,
  ).measurement;
  assert.deepEqual(after, referenceSchedule.best);
  assert.deepEqual(before, referenceSchedule.baseline);
  assert.ok(after.completed && after.score === 160 && after.collisions === 0);
  assert.ok(after.minimumClearanceMm >= SPEC.margin * 1000 - 1e-5);
  assert.ok(after.time < before.time - 10);
  assert.ok(
    after.distance < before.distance && after.trafficWait < before.trafficWait,
  );
});

void test('schedule retains every destination, role, color quota and preloaded cube order', () => {
  const world = createWorld();
  const previous = structuredClone(world);
  const destinations = new Map(
    world.robots.flatMap((r) => r.jobs.map((j) => [j.itemId, j])),
  );
  applySchedule(world, referenceSchedule!.schedule);
  assert.deepEqual(
    world.robots.map((r) => r.magazine),
    previous.robots.map((r) => r.magazine),
  );
  assert.equal(
    new Set(world.robots.flatMap((r) => r.jobs.map((j) => j.itemId))).size,
    16,
  );
  for (const r of world.robots)
    for (const j of r.jobs) {
      const original = destinations.get(j.slotId!)!;
      assert.equal(j.destination, original.destination);
      assert.deepEqual(j.drop, original.drop);
      const item = world.items.find((i) => i.id === j.itemId)!;
      const source = world.items.find((i) => i.id === original.itemId)!;
      assert.equal(item.color, source.color);
      assert.equal(item.kind === 'disc', r.role === 'hamster');
      if (item.kind === 'cube') assert.equal(item.carrier, r.id);
    }
  for (const color of ['red', 'yellow', 'green'])
    assert.equal(
      world.items.filter(
        (i) => i.kind === 'cylinder' && i.color === color && i.selected,
      ).length,
      3,
    );
  // Applying a plan never moves an item or a robot.
  assert.deepEqual(
    world.robots.map((r) => r.pose),
    previous.robots.map((r) => r.pose),
  );
  assert.deepEqual(
    world.items.map((i) => [i.id, i.x, i.y, i.carrier]),
    previous.items.map((i) => [i.id, i.x, i.y, i.carrier]),
  );
  const once = structuredClone(world);
  applySchedule(world, captureSchedule(world));
  assert.deepEqual(
    world,
    once,
    'a captured logical schedule can be reapplied before departure',
  );
});

void test('invalid assignments are rejected atomically and started work is immutable', () => {
  const edits = [
    (p: ReturnType<typeof captureSchedule>) => {
      p.robots[0].jobs[0] = p.robots[0].jobs[1];
    },
    (p: ReturnType<typeof captureSchedule>) => {
      p.selection = { G1: 'R4' };
    },
    (p: ReturnType<typeof captureSchedule>) => {
      p.selection = { G1: 'G3' };
    },
    (p: ReturnType<typeof captureSchedule>) => {
      p.robots[0].jobs.reverse();
    },
    (p: ReturnType<typeof captureSchedule>) => {
      p.robots[0].delay = NaN;
    },
    (p: ReturnType<typeof captureSchedule>) => {
      p.robots[0].delay = 120;
    },
  ];
  for (const edit of edits) {
    const w = createWorld(),
      before = structuredClone(w),
      p = captureSchedule(w);
    edit(p);
    assert.throws(() => applySchedule(w, p));
    assert.deepEqual(w, before);
  }
  const started = createWorld(),
    plan = captureSchedule(started);
  advance(started);
  assert.throws(() => applySchedule(started, plan));
});

void test('five-robot stress fixture donates a cylinder, never a preloaded medical kit', () => {
  const original = createWorld();
  const expanded = createExperiment('localization', 5);
  for (const robot of original.robots) {
    const actual = expanded.robots.find((r) => r.id === robot.id)!;
    assert.deepEqual(actual.magazine, robot.magazine);
    assert.deepEqual(
      actual.jobs.slice(0, robot.magazine.length),
      robot.jobs.slice(0, robot.magazine.length),
    );
  }
  const extra = expanded.robots.find((r) => r.id === 'extra5')!;
  assert.deepEqual(extra.magazine, []);
  assert.deepEqual(
    extra.jobs.map((j) => j.itemId),
    ['G2'],
  );
  assert.equal(applyReferenceSchedule(expanded), false);
});

void test('changed layouts, sensor conditions and expanded fleets cannot use cached reference measurements', () => {
  for (const mutate of [
    (w: ReturnType<typeof createWorld>) => {
      w.items[0].x += 0.01;
    },
    (w: ReturnType<typeof createWorld>) => {
      w.robots[0].pose.x += 0.01;
    },
    (w: ReturnType<typeof createWorld>) => {
      w.observer.delayMs = 200;
    },
    (w: ReturnType<typeof createWorld>) => {
      w.scenario = 'intermittent';
    },
    (w: ReturnType<typeof createWorld>) => {
      w.robots[0].jobs.reverse();
    },
  ]) {
    const w = createWorld();
    mutate(w);
    assert.equal(applyReferenceSchedule(w), false);
    assert.equal(w.coordination.fixedSchedule, undefined);
  }
});

void test('deadline pruning does not report a partial high-score run as completion', () => {
  const w = createWorld();
  const partial = evaluateSchedule(
    w,
    referenceSchedule!.schedule,
    10,
  ).measurement;
  assert.equal(partial.completed, false);
  assert.ok(partial.score < 160);
  assert.ok(partial.time < 10.03);
});

void test('fixed team plan still freezes immediately on emergency stop and stale input', () => {
  for (const stop of ['emergency', 'stale']) {
    const w = createComparison('none');
    for (let i = 0; i < 120; i++) advance(w);
    if (stop === 'emergency') w.emergencyStopped = true;
    else {
      w.observer.lost = true;
      for (let i = 0; i < 20; i++) advance(w);
    }
    const poses = structuredClone(w.robots.map((r) => r.pose)),
      score = scoreWorld(w.items).points;
    for (let i = 0; i < 100; i++) advance(w);
    assert.deepEqual(
      w.robots.map((r) => r.pose),
      poses,
    );
    assert.equal(scoreWorld(w.items).points, score);
  }
});

void test('healthy hover mode also reproduces the same winning schedule without a speed bonus', () => {
  const hover = runComparison('hover', 'normal');
  assert.equal(hover.score, 160);
  assert.ok(hover.completed && hover.collisions === 0);
  assert.equal(hover.time, referenceSchedule!.best.time);
});
