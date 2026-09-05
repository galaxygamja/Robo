import {
  createWorld,
  scoreWorld,
  clearance,
  advance,
  collisionReason,
  type ObservationMode,
  type World,
} from './mission.ts';

export type ComparisonMode = 'none' | 'hover' | 'active';
export type ComparisonScenario = World['scenario'];
export const SCENARIO_LABEL: Record<ComparisonScenario, string> = {
  normal: '정상 입력 · 고장 없음',
  intermittent: '간헐 누락 · H2 입력 10초마다 1.2초 중단',
  occlusion: '가림 복구 시험 · H2 입력 지속 중단',
};
export const MODE_LABEL: Record<ComparisonMode, string> = {
  none: '드론 없음',
  hover: '중앙 관측',
  active: '이동 관측',
};
export function createComparison(
  mode: ComparisonMode,
  scenario: ComparisonScenario = 'normal',
  optimize = true,
): World {
  const world = createWorld(mode === 'none' ? 'localization' : 'drone');
  if (mode !== 'none') world.drone.strategy = mode;
  world.scenario = scenario;
  world.coordination.enabled = optimize;
  if (scenario === 'occlusion') {
    world.observer.missingId = 'H2';
    world.drone.occlusionId = 'H2';
  }
  return world;
}
export function comparisonResult(
  world: World,
  collisions: number,
  minimumClearance: number,
) {
  return {
    mode: (world.drone.enabled
      ? world.drone.strategy
      : 'none') as ComparisonMode,
    scenario: world.scenario,
    optimized: world.coordination.enabled,
    time: world.elapsed,
    remaining: 120 - world.elapsed,
    score: scoreWorld(world.items).points,
    completed: world.robots.every((r) => r.phase === 'complete'),
    distance: world.robots.reduce((sum, r) => sum + r.distanceTravelled, 0),
    rotation: world.robots.reduce((sum, r) => sum + r.rotationRadians, 0),
    trafficWait: world.robots.reduce((sum, r) => sum + r.blockedSeconds, 0),
    inputHold: world.drone.holdSeconds,
    aerialRecovery: world.drone.recoveredRobotSeconds,
    shortcuts: world.coordination.routeShortcuts,
    staging: world.coordination.stagingChanges,
    taskReorders: world.robots.reduce((sum, r) => sum + r.taskReorders, 0),
    collisions,
    minimumClearanceMm: minimumClearance * 1000,
  };
}
export type ComparisonResult = ReturnType<typeof comparisonResult>;
// Runs in a dedicated browser worker (or the CLI), never changes the visible
// match or sends device commands. Every number is measured from this engine.
export function runComparison(
  mode: ComparisonMode,
  scenario: ComparisonScenario,
  optimize = true,
): ComparisonResult {
  const world = createComparison(mode, scenario, optimize);
  let collisions = 0,
    minimumClearance = Infinity;
  while (!world.ended) {
    advance(world);
    collisions += world.robots.filter((r) => collisionReason(world, r)).length;
    minimumClearance = Math.min(minimumClearance, clearance(world));
  }
  return comparisonResult(world, collisions, minimumClearance);
}

// One repeatable stress fixture, not a promise for arbitrary fleet layouts.
export function createExperiment(
  mode: ObservationMode,
  count: 4 | 5 = 4,
): World {
  if (count === 4) return createWorld(mode);
  if (mode === 'drone')
    throw new Error(
      'Five-robot stress fixture uses the ground drone parking footprint: choose localization',
    );
  const fleet = createWorld(mode).robots;
  const donor = fleet.find((r) => r.id === 'H2')!;
  const job = donor.jobs.shift()!;
  fleet.push({
    ...donor,
    id: 'extra5',
    name: '확장 비버 · B4',
    pose: { x: 0.74, y: 0.065, heading: 0 },
    delay: 1,
    jobs: [job],
    magazine: [],
    staging: { x: 0.4, y: 0.35 },
    park: { x: 0.8, y: 0.9 },
  });
  return createWorld(mode, fleet);
}

export function experimentReport(world: World) {
  return {
    schema: 'robo-synthetic-experiment-v3',
    device_io: false,
    model:
      'ideal_geometry_with_observation_health_gate_not_physical_controller',
    field_layout: 'practice_reference_B1_B4_unconfirmed',
    scenario: world.scenario,
    scheduled_missing_id: world.scheduledMissingId,
    coordination: world.coordination,
    observation: {
      mode: world.observer.mode,
      delay_ms: world.observer.delayMs,
      error_bound_mm: world.observer.noiseMm,
      frame_age_ms: world.observer.frameAge * 1000,
      missing_id: world.observer.missingId,
      lost: world.observer.lost,
      selected_sources: Object.fromEntries(
        Object.entries(world.observer.poses).map(([id, p]) => [id, p.source]),
      ),
      unavailable_ids: world.observer.unavailableIds,
      conflicting_ids: world.observer.conflictingIds,
      speed_scale: world.observer.speedScale,
    },
    aerial: {
      strategy: world.drone.enabled ? world.drone.strategy : 'none',
      pose: world.drone.pose,
      altitude_m: world.drone.altitude,
      reason: world.drone.reason,
      covered_robot_ids: world.drone.visibleRobots,
      recovered_robot_ids: world.drone.recoveredIds,
      recovered_robot_seconds: world.drone.recoveredRobotSeconds,
      hold_seconds: world.drone.holdSeconds,
      horizontal_distance_m: world.drone.distance,
      anchor_ids: world.drone.anchorIds,
      calibration_valid: world.drone.calibrationValid,
      video_delay_ms: world.drone.delayMs,
      video_lost: world.drone.videoLost,
      calibration_lost: world.drone.calibrationLost,
      occlusion_test_robot: world.drone.occlusionId,
      confirmed_object_ids: Object.values(world.drone.objects)
        .filter((p) => p.confirmed && world.elapsed - p.at < 0.3)
        .map((p) => p.id),
    },
    elapsed_s: world.elapsed,
    ended: world.ended,
    reason: world.reason,
    safety_reason: world.safetyReason,
    emergency_stop: world.emergencyStopped,
    score: scoreWorld(world.items),
    current_clearance_mm: clearance(world) * 1000,
    robots: world.robots.map((r) => ({
      id: r.id,
      role: r.role,
      phase: r.phase,
      completed_tasks: r.served,
      recovery_attempts: r.recoveryAttempts,
      task_reorders: r.taskReorders,
      distance_m: r.distanceTravelled,
      rotation_rad: r.rotationRadians,
      traffic_wait_s: r.blockedSeconds,
      completed_at_s: r.completedAt,
      waiting_for: r.blockedBy,
    })),
    logs: world.logs,
  };
}
