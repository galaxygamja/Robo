// A reproducible CONGESTION experiment, not a certified five-robot preset.
// node --experimental-strip-types web_simulator/tools/fleet-expansion.ts
import { advance, scoreWorld, collisionReason } from '../lib/mission.ts';
import { createExperiment } from '../lib/experiments.ts';
const world = createExperiment('localization', 5);
let collisionSteps = 0;
while (!world.ended) {
  advance(world);
  if (process.argv.includes('--trace') && Math.abs(world.elapsed % 10) < 0.01)
    console.log(
      JSON.stringify({
        time: world.elapsed,
        locks: world.locks,
        robots: world.robots.map((r) => ({
          id: r.id,
          pose: r.pose,
          target: r.target,
          phase: r.phase,
          job: r.jobs[r.jobIndex]?.itemId,
          blocked: r.blockedBy,
        })),
      }),
    );
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
