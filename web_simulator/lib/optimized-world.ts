import type { World } from './mission.ts';
import { applySchedule, scheduleSignature } from './scheduling.ts';
import { referenceSchedule } from './reference-schedule.ts';

export function applyReferenceSchedule(world: World): boolean {
  if (
    !referenceSchedule ||
    !world.coordination.enabled ||
    world.elapsed !== 0 ||
    scheduleSignature(world) !== referenceSchedule.signature
  )
    return false;
  applySchedule(world, referenceSchedule.schedule);
  return true;
}
