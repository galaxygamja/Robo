import assert from 'node:assert/strict';
import test from 'node:test';
import { parsePositionLog, toSimulationHeading } from '../lib/localization.ts';
import { advance, createWorld, collisionReason } from '../lib/mission.ts';

const row = (count = 5) => ({
  status: 'detected',
  source_name: 'camera:session',
  sequence: 1,
  captured_at_s: 10,
  field_size_mm: [1143, 1181],
  coordinate_system: 'bottom_left_x_right_y_up_mm',
  host_age_ms: 10,
  robots: Array.from({ length: count }, (_, i) => ({
    robot_id: `robot${i}`,
    robot_center_mm: [100 + i * 50, 200],
    heading_rad: Math.PI / 2,
  })),
  objects: [{ color: 'red', kind: 'cylinder', center_mm: [500, 600] }],
});

void test('real logs preserve arbitrary 5/6/12 identities, millimetres and headings', () => {
  for (const n of [5, 6, 12]) {
    const f = parsePositionLog(JSON.stringify(row(n)))[0];
    assert.equal(f.poses.length, n);
    assert.equal(f.poses[0].x, 100);
    assert.equal(toSimulationHeading(f.poses[0].heading), 0);
    assert.equal(toSimulationHeading(0), -Math.PI / 2);
    assert.equal(f.objects[0].color, 'red');
  }
});
void test('malformed, duplicate and out-of-field logs are rejected, reverse frames marked', () => {
  for (const mutate of [
    (r: ReturnType<typeof row>) => {
      r.robots[1].robot_id = r.robots[0].robot_id;
    },
    (r: ReturnType<typeof row>) => {
      r.robots[0].robot_center_mm[0] = 99999;
    },
    (r: ReturnType<typeof row>) => {
      r.coordinate_system = 'pixels';
    },
  ]) {
    const r = row();
    mutate(r);
    assert.throws(() => parsePositionLog(JSON.stringify(r)));
  }
  const r = row(),
    back = { ...r, sequence: 0, captured_at_s: 9 };
  assert.equal(
    parsePositionLog([r, back].map((v) => JSON.stringify(v)).join('\n'))[1]
      .rejected,
    true,
  );
  assert.throws(() => parsePositionLog(''));
});
void test('missing tracks with null positions never teleport to coordinate zero', () => {
  const r = {
    ...row(),
    tracks: [
      {
        robot_id: 'missing',
        robot_center_mm: null,
        heading_rad: null,
        state: 'missing',
        age_ms: null,
      },
    ],
  };
  assert.deepEqual(parsePositionLog(JSON.stringify(r))[0].poses, []);
});
void test('five configured bodies and arbitrary IDs use the shared localization stop gate', () => {
  const fleet = createWorld().robots;
  fleet.push({
    ...fleet[3],
    id: 'extra5',
    name: '확장 비버',
    pose: { x: 0.74, y: 0.065, heading: 0 },
    jobs: [],
    magazine: [],
    delay: 1,
    park: { x: 0.8, y: 0.9 },
  });
  const w = createWorld('localization', fleet);
  assert.equal(w.robots.length, 5);
  for (const r of w.robots) assert.equal(collisionReason(w, r), null);
  advance(w);
  assert.equal(Object.keys(w.observer.poses).length, 5);
  const before = w.robots.map((r) => ({ ...r.pose }));
  w.observer.missingId = 'extra5';
  for (let i = 0; i < 30; i++) advance(w);
  assert.deepEqual(
    w.robots.map((r) => r.pose),
    before,
  );
  assert.ok(w.elapsed > 0.5);
  assert.throws(() => createWorld('localization', [...fleet, fleet[0]]));
});
