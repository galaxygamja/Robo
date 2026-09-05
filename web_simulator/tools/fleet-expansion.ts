// A reproducible CONGESTION experiment, not a certified five-robot preset.
// node --experimental-strip-types web_simulator/tools/fleet-expansion.ts
import {
  createWorld,
  advance,
  scoreWorld,
  collisionReason,
} from '../lib/mission.ts';
const fleet = createWorld().robots;
const donor = fleet.find((r) => r.id === 'H2')!;
const job = donor.jobs.shift()!;
fleet.push({
  ...donor,
  id: 'extra5',
  name: '5th congestion test',
  pose: { x: 0.74, y: 0.065, heading: 0 },
  delay: 1,
  jobs: [job],
  magazine: [],
  staging: { x: 0.4, y: 0.35 },
  park: { x: 0.8, y: 0.9 },
});
const world = createWorld('localization', fleet);
let collisionSteps = 0;
while (!world.ended) {
  advance(world);
  collisionSteps += world.robots.filter((r) =>
    collisionReason(world, r),
  ).length;
}
console.log(
  JSON.stringify(
    {
      test: 'synthetic_five_robot_congestion_not_hardware',
      elapsed_s: world.elapsed,
      points: scoreWorld(world.items).points,
      collisionSteps,
      complete: world.robots.every((r) => r.phase === 'complete'),
      robots: world.robots.map((r) => ({
        id: r.id,
        phase: r.phase,
        waitingFor: r.blockedBy,
      })),
    },
    null,
    2,
  ),
);
