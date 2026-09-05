import {
  createWorld,
  scoreWorld,
  clearance,
  type ObservationMode,
  type World,
} from './mission.ts';

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
    schema: 'robo-synthetic-experiment-v2',
    device_io: false,
    model:
      'ideal_geometry_with_observation_health_gate_not_physical_controller',
    field_layout: 'practice_reference_B1_B4_unconfirmed',
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
      observed_task_reorders: r.taskReorders,
      waiting_for: r.blockedBy,
    })),
    logs: world.logs,
  };
}
