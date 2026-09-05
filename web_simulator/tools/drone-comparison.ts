import {
  createWorld,
  advance,
  scoreWorld,
  collisionReason,
} from '../lib/mission.ts';
for (const strategy of ['none', 'hover', 'active'] as const) {
  const w = createWorld(strategy === 'none' ? 'localization' : 'drone');
  w.observer.missingId = 'H2';
  w.drone.occlusionId = 'H2';
  if (strategy !== 'none') w.drone.strategy = strategy;
  let collisions = 0;
  const until = process.argv.includes('--full') ? 120 : 12;
  while (!w.ended && w.elapsed < until - 0.001) {
    advance(w);
    collisions += w.robots.filter((r) => collisionReason(w, r)).length;
  }
  console.log(
    JSON.stringify({
      strategy,
      time: w.elapsed,
      hold_s: w.drone.holdSeconds,
      recovered_robot_s: w.drone.recoveredRobotSeconds,
      pose: w.drone.pose,
      z: w.drone.altitude,
      anchors: w.drone.anchorIds,
      visible: w.drone.visibleRobots,
      reason: w.safetyReason,
      score: scoreWorld(w.items).points,
      collisions,
      taskReorders: w.robots.reduce((s, r) => s + r.taskReorders, 0),
    }),
  );
}
