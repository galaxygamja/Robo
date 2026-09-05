import assert from 'node:assert/strict';
import test from 'node:test';
import {
  FIELD,
  FOOTPRINT,
  MECANUM_FOOTPRINT,
  OFFICIAL_LAYOUT,
  ROUTES,
  createInitialRobots,
  minimumClearance,
  mixMecanumCommand,
  mixMecanumWheels,
  polygonsOverlap,
  stepWorld,
  transformFootprint,
} from '../lib/simulation.ts';

void test('SCAD nominal operating footprint is 126 by 100 mm', () => {
  const xs = FOOTPRINT.map((point) => point.x);
  const ys = FOOTPRINT.map((point) => point.y);
  assert.equal(Math.max(...xs) - Math.min(...xs), 0.126);
  assert.equal(Math.max(...ys) - Math.min(...ys), 0.1);
});

void test('all six fixed-heading starts fit without overlap', () => {
  const robots = createInitialRobots();
  const footprints = robots.map((robot) => transformFootprint(robot.pose));
  for (const footprint of footprints) {
    for (const point of footprint) {
      assert.ok(point.x >= FIELD.startZone.x - 1e-9);
      assert.ok(point.x <= FIELD.startZone.x + FIELD.startZone.width + 1e-9);
      assert.ok(point.y >= FIELD.startZone.y - 1e-9);
      assert.ok(point.y <= FIELD.startZone.y + FIELD.startZone.height + 1e-9);
    }
  }
  for (let index = 0; index < footprints.length; index += 1) {
    for (let other = index + 1; other < footprints.length; other += 1) {
      assert.equal(
        polygonsOverlap(footprints[index], footprints[other]),
        false,
      );
    }
  }
});

void test('official reference geometry keeps the 480 by 280 mm start at lower right', () => {
  assert.equal(FIELD.width, 1.143);
  assert.equal(FIELD.height, 1.181);
  assert.deepEqual(FIELD.startZone, {
    x: 0.663,
    y: 0,
    width: 0.48,
    height: 0.28,
  });
  assert.equal(OFFICIAL_LAYOUT.groundPoints.length, 12);
  assert.equal(
    OFFICIAL_LAYOUT.healthcare.pccLeft.width +
      OFFICIAL_LAYOUT.tapeWidth +
      OFFICIAL_LAYOUT.healthcare.hospital.width +
      OFFICIAL_LAYOUT.tapeWidth +
      OFFICIAL_LAYOUT.healthcare.pccRight.width,
    FIELD.width,
  );
});

void test('mecanum mixer produces the intended cardinal patterns', () => {
  assert.deepEqual(mixMecanumWheels(0.18, 0, 0), {
    fl: 1,
    fr: 1,
    rl: 1,
    rr: 1,
  });
  assert.deepEqual(mixMecanumWheels(0, 0.18, 0), {
    fl: -1,
    fr: 1,
    rl: 1,
    rr: -1,
  });
  assert.deepEqual(mixMecanumWheels(-0.18, 0, 0), {
    fl: -1,
    fr: -1,
    rl: -1,
    rr: -1,
  });
  assert.deepEqual(mixMecanumWheels(0, -0.18, 0), {
    fl: 1,
    fr: -1,
    rl: -1,
    rr: 1,
  });
  assert.deepEqual(mixMecanumWheels(0, 0, 2.4), {
    fl: -1,
    fr: 1,
    rl: -1,
    rr: 1,
  });
  assert.deepEqual(mixMecanumWheels(0, 0, -2.4), {
    fl: 1,
    fr: -1,
    rl: 1,
    rr: -1,
  });
});

void test('the 15 mm wall margin stops a manual command before contact', () => {
  const robot = createInitialRobots()[0];
  robot.pose = { x: 0.078, y: 0.5, heading: 0 };
  robot.status = 'manual';
  const result = stepWorld(
    [robot],
    0.01,
    1,
    'mecanum',
    true,
    robot.id,
    new Set(['a']),
  );
  assert.equal(result.robots[0].status, 'blocked');
  assert.deepEqual(result.robots[0].pose, robot.pose);
});

void test('the 15 mm robot margin stops a pair before contact', () => {
  const [moving, parked] = createInitialRobots();
  moving.pose = { x: 0.4, y: 0.8, heading: 0 };
  moving.status = 'manual';
  parked.pose = { x: 0.541, y: 0.8, heading: 0 };
  parked.status = 'complete';
  const result = stepWorld(
    [moving, parked],
    0.01,
    1,
    'mecanum',
    true,
    moving.id,
    new Set(['d']),
  );
  assert.equal(result.robots[0].status, 'blocked');
  assert.deepEqual(result.robots[0].pose, moving.pose);
});

void test('combined mecanum commands desaturate wheel and body motion together', () => {
  const mixed = mixMecanumCommand(0.18, 0, 2.4);
  assert.ok(mixed.scale > 0 && mixed.scale < 1);
  assert.equal(
    Math.max(...Object.values(mixed.wheels).map((value) => Math.abs(value))),
    1,
  );
  const robot = createInitialRobots()[0];
  robot.pose = { x: 1, y: 0.9, heading: 0 };
  robot.status = 'manual';
  const moved = stepWorld(
    [robot],
    0.01,
    1,
    'mecanum',
    true,
    robot.id,
    new Set(['w', 'q']),
  ).robots[0];
  assert.ok(Math.abs(moved.pose.x - robot.pose.x) < 1e-12);
  assert.ok(
    Math.abs(moved.pose.y - (robot.pose.y + 0.18 * mixed.scale * 0.01)) < 1e-12,
  );
  assert.ok(Math.abs(moved.pose.heading - 2.4 * mixed.scale * 0.01) < 1e-12);
});

void test('differential replay of the reference routes also completes safely', () => {
  let robots = createInitialRobots();
  let closest = Infinity;
  for (let tick = 1; tick <= FIELD.duration * 100; tick += 1) {
    robots = stepWorld(
      robots,
      0.01,
      tick * 0.01,
      'differential',
      false,
      'R1',
      new Set(),
    ).robots;
    closest = Math.min(closest, minimumClearance(robots, 'differential'));
  }
  assert.equal(robots.filter((robot) => robot.status === 'complete').length, 6);
  assert.ok(closest >= FIELD.safetyMargin - 1e-9);
});

void test('the provisional six-route replay stays collision-free and completes', () => {
  let robots = createInitialRobots();
  const keys = new Set<string>();
  let closest = Infinity;
  for (let tick = 1; tick <= FIELD.duration * 100; tick += 1) {
    robots = stepWorld(
      robots,
      0.01,
      tick * 0.01,
      'mecanum',
      false,
      'R1',
      keys,
    ).robots;
    closest = Math.min(closest, minimumClearance(robots, 'mecanum'));
  }
  assert.equal(robots.filter((robot) => robot.status === 'blocked').length, 0);
  assert.equal(robots.filter((robot) => robot.status === 'complete').length, 6);
  assert.ok(closest >= FIELD.safetyMargin - 1e-9);
  robots.forEach((robot, index) => {
    const goal = ROUTES[index][ROUTES[index].length - 1];
    assert.ok(Math.hypot(robot.pose.x - goal.x, robot.pose.y - goal.y) < 0.03);
  });
});

void test('mecanum collision footprint includes all four rendered wheel corners', () => {
  for (const centerX of [-0.057, 0.057]) {
    for (const centerY of [-0.032, 0.032]) {
      for (const dx of [-0.006, 0.006]) {
        for (const dy of [-0.0125, 0.0125]) {
          const corner = { x: centerX + dx, y: centerY + dy };
          // Every edge of this counter-clockwise hull must contain the corner.
          MECANUM_FOOTPRINT.forEach((a, i) => {
            const b = MECANUM_FOOTPRINT[(i + 1) % MECANUM_FOOTPRINT.length];
            assert.ok(
              (b.x - a.x) * (corner.y - a.y) - (b.y - a.y) * (corner.x - a.x) >=
                -1e-12,
            );
          });
        }
      }
    }
  }
});

void test('world does not advance after the 120 second limit', () => {
  const robots = createInitialRobots();
  const after = stepWorld(
    robots,
    0.01,
    FIELD.duration,
    'mecanum',
    false,
    'R1',
    new Set(),
  ).robots;
  // The world itself freezes pose and clears every motor output at the deadline.
  assert.deepEqual(
    robots.map((robot) => robot.pose),
    after.map((robot) => robot.pose),
  );
  after.forEach((robot) => {
    assert.deepEqual(robot.wheels, { fl: 0, fr: 0, rl: 0, rr: 0 });
  });
});
