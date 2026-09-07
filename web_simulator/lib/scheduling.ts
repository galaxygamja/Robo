import {
  advance,
  clearance,
  collisionReason,
  createWorld,
  scoreWorld,
  SPEC,
  tip,
  type World,
} from './mission.ts';
import {
  FIELD,
  MECANUM_FOOTPRINT,
  normalizeAngle,
  type Point,
} from './simulation.ts';

export type TeamSchedule = {
  robots: { id: string; delay: number; jobs: string[] }[];
  fixedOrder: boolean;
  selection?: Record<string, string>;
};
export type ScheduleMeasurement = {
  completed: boolean;
  time: number;
  score: number;
  distance: number;
  trafficWait: number;
  minimumClearanceMm: number;
  collisions: number;
  robots: {
    id: string;
    completedAt: number | null;
    distance: number;
    wait: number;
  }[];
};
export type ScheduleSearchResult = {
  signature: string;
  schedule: TeamSchedule;
  baseline: ScheduleMeasurement;
  best: ScheduleMeasurement;
  tested: number;
  budget: number;
};

// Bump when movement, manipulator timing or collision semantics change. This
// cache key also includes all geometry, placements, jobs and robot identities.
export function scheduleSignature(world: World): string {
  return JSON.stringify({
    model: 'mission-makespan-v1',
    field: FIELD,
    spec: SPEC,
    footprint: MECANUM_FOOTPRINT,
    items: world.items,
    scenario: world.scenario,
    observations: {
      delay: world.observer.delayMs,
      noise: world.observer.noiseMm,
      lost: world.observer.lost,
      missing: world.observer.missingId,
    },
    robots: world.robots.map((r) => ({
      id: r.id,
      role: r.role,
      pose: r.pose,
      delay: r.delay,
      jobs: r.jobs,
      magazine: r.magazine,
      staging: r.staging,
      park: r.park,
    })),
  });
}

export function captureSchedule(world: World, fixedOrder = true): TeamSchedule {
  return {
    fixedOrder,
    selection: Object.fromEntries(
      world.robots.flatMap((r) =>
        r.jobs
          .filter((j) => j.slotId && j.slotId !== j.itemId)
          .map((j) => [j.slotId!, j.itemId]),
      ),
    ),
    robots: world.robots.map((r) => ({
      id: r.id,
      delay: r.delay,
      jobs: r.jobs.map((j) => j.slotId ?? j.itemId),
    })),
  };
}

// Never change physical item identities, destination slots, loaded magazine
// ownership/order, robot roles, geometry or time constants while optimizing.
export function applySchedule(world: World, plan: TeamSchedule): void {
  if (
    world.elapsed !== 0 ||
    world.ended ||
    world.robots.some((r) => r.phase !== 'waiting')
  )
    throw new Error('A schedule can only be applied before the match starts');
  if (
    !plan ||
    typeof plan.fixedOrder !== 'boolean' ||
    !Array.isArray(plan.robots)
  )
    throw new Error('Invalid schedule');
  const jobs = new Map(
    world.robots.flatMap((r) =>
      r.jobs.map((j) => [j.slotId ?? j.itemId, j] as const),
    ),
  );
  const ids = plan.robots.flatMap((r) => r.jobs);
  const actual = (id: string) => plan.selection?.[id] ?? id;
  if (
    plan.robots.length !== world.robots.length ||
    new Set(plan.robots.map((r) => r.id)).size !== world.robots.length ||
    ids.length !== jobs.size ||
    new Set(ids).size !== jobs.size ||
    ids.some((id) => !jobs.has(id))
  )
    throw new Error(
      'Schedule must retain every robot and every assigned item exactly once',
    );
  if (
    Object.keys(plan.selection ?? {}).some((id) => !jobs.has(id)) ||
    new Set(ids.map(actual)).size !== ids.length
  )
    throw new Error('Repeated or unknown selected object');
  for (const id of ids) {
    const original = world.items.find((i) => i.id === id)!;
    const selected = world.items.find((i) => i.id === actual(id));
    if (
      !selected ||
      (actual(id) !== id &&
        (original.kind !== 'cylinder' ||
          selected.kind !== 'cylinder' ||
          original.color !== selected.color ||
          selected.carrier ||
          selected.released))
    )
      throw new Error(
        'Only an available same-color cylinder can replace a pickup',
      );
  }
  for (const entry of plan.robots) {
    const robot = world.robots.find((r) => r.id === entry.id);
    if (
      !robot ||
      !Number.isFinite(entry.delay) ||
      entry.delay < 0 ||
      entry.delay >= FIELD.duration
    )
      throw new Error('Invalid robot or departure time');
    const cubes: string[] = [];
    for (const id of entry.jobs) {
      const item = world.items.find((i) => i.id === actual(id))!;
      if (
        (item.kind === 'disc') !== (robot.role === 'hamster') ||
        (item.kind === 'cube' && item.carrier !== robot.id)
      )
        throw new Error(
          'Schedule violates robot role or loaded cube ownership',
        );
      if (item.kind === 'cube') cubes.push(id);
    }
    if (
      JSON.stringify(cubes) !== JSON.stringify(robot.magazine) ||
      robot.magazine.some((id, i) => entry.jobs[i] !== id)
    )
      throw new Error(
        'Loaded cube order must remain at the front of its owner queue',
      );
  }
  // Commit only after the complete plan has passed validation.
  for (const entry of plan.robots) {
    const robot = world.robots.find((r) => r.id === entry.id)!;
    robot.delay = entry.delay;
    robot.jobs = entry.jobs.map((id) => ({
      ...jobs.get(id)!,
      slotId: id,
      itemId: actual(id),
      drop: { ...jobs.get(id)!.drop },
    }));
  }
  for (const item of world.items)
    if (item.kind === 'cylinder')
      item.selected = ids.some((id) => actual(id) === item.id);
  world.coordination.enabled = true;
  world.coordination.fixedSchedule = plan.fixedOrder;
  world.coordination.scheduleName = '전체 완료 시간 탐색';
}

function measure(
  world: World,
  collisions: number,
  minimum: number,
): ScheduleMeasurement {
  return {
    completed: world.ended && world.robots.every((r) => r.phase === 'complete'),
    time: world.elapsed,
    score: scoreWorld(world.items).points,
    distance: world.robots.reduce((sum, r) => sum + r.distanceTravelled, 0),
    trafficWait: world.robots.reduce((sum, r) => sum + r.blockedSeconds, 0),
    minimumClearanceMm: minimum * 1000,
    collisions,
    robots: world.robots.map((r) => ({
      id: r.id,
      completedAt: r.completedAt,
      distance: r.distanceTravelled,
      wait: r.blockedSeconds,
    })),
  };
}

export function evaluateSchedule(
  initial: World,
  schedule: TeamSchedule,
  cutoff: number = FIELD.duration,
  workLimitMs = Infinity,
) {
  const world = structuredClone(initial);
  applySchedule(world, schedule);
  let collisions = world.robots.filter((r) => collisionReason(world, r)).length,
    minimum = clearance(world);
  const deadline = performance.now() + workLimitMs;
  while (!world.ended && world.elapsed < cutoff - 1e-8) {
    if (performance.now() >= deadline) break;
    advance(world);
    minimum = Math.min(minimum, clearance(world));
    collisions += world.robots.filter((r) => collisionReason(world, r)).length;
    if (collisions || world.robots.some((r) => r.phase === 'fault')) break;
  }
  return {
    measurement: measure(world, collisions, minimum),
    schedule: captureSchedule(world, schedule.fixedOrder),
  };
}

function feasible(m: ScheduleMeasurement, requiredScore: number) {
  return (
    m.completed &&
    m.score === requiredScore &&
    m.collisions === 0 &&
    m.minimumClearanceMm >= SPEC.margin * 1000 - 1e-5
  );
}
function better(
  a: ScheduleMeasurement,
  b: ScheduleMeasurement,
  requiredScore: number,
) {
  if (!feasible(a, requiredScore)) return false;
  if (!feasible(b, requiredScore)) return true;
  return (
    a.time < b.time - 0.009 ||
    (Math.abs(a.time - b.time) < 0.009 &&
      (a.distance < b.distance - 0.0001 ||
        (Math.abs(a.distance - b.distance) < 0.0001 &&
          a.trafficWait < b.trafficWait)))
  );
}
const distance = (a: Point, b: Point) => Math.hypot(a.x - b.x, a.y - b.y);
const approach = (p: Point, heading: number) => ({
  x: p.x + Math.sin(heading) * SPEC.armExtended,
  y: p.y - Math.cos(heading) * SPEC.armExtended,
  heading,
});

// This estimate ranks candidates only. Real engine replay (including waits,
// turns, arm movement, releases and parking) is the acceptance criterion.
function estimatedFinish(initial: World, plan: TeamSchedule) {
  const allJobs = new Map(
    initial.robots.flatMap((r) =>
      r.jobs.map((j) => [j.slotId ?? j.itemId, j] as const),
    ),
  );
  const times = plan.robots.map((entry) => {
    const robot = initial.robots.find((r) => r.id === entry.id)!;
    let p = { ...robot.pose },
      time = entry.delay;
    for (const id of entry.jobs) {
      const item = initial.items.find(
          (i) => i.id === (plan.selection?.[id] ?? id),
        )!,
        job = allJobs.get(id)!;
      const drop = approach(
        job.drop,
        job.destination === 'LAB' || job.destination === 'RZ' ? Math.PI : 0,
      );
      if (item.kind !== 'cube') {
        const pick = approach(
          item,
          item.kind === 'disc' || item.y > 0.63 ? Math.PI : 0,
        );
        time +=
          Math.abs(normalizeAngle(pick.heading - p.heading)) / 2 +
          distance(p, pick) / SPEC.speed +
          2.06;
        const exit = tip(pick, -0.1);
        time += 0.1 / SPEC.speed;
        p = { ...exit, heading: pick.heading };
      }
      time +=
        Math.abs(normalizeAngle(drop.heading - p.heading)) / 2 +
        distance(p, drop) / SPEC.speed +
        1.7;
      const exit = robot.role === 'hamster' ? robot.park : tip(drop, -0.11);
      time += distance(drop, exit) / SPEC.speed;
      p = { ...exit, heading: drop.heading };
    }
    return time + distance(p, robot.park) / SPEC.speed;
  });
  return Math.max(...times) + times.reduce((a, b) => a + b, 0) * 0.025;
}

function scheduleKey(plan: TeamSchedule) {
  return JSON.stringify(plan);
}
function neighbors(initial: World, plan: TeamSchedule): TeamSchedule[] {
  const result: TeamSchedule[] = [];
  const cylinder = (id: string) =>
    initial.items.find((i) => i.id === id)?.kind === 'cylinder';
  const push = (edit: (p: TeamSchedule) => void) => {
    const p = structuredClone(plan);
    edit(p);
    p.fixedOrder = true;
    result.push(p);
  };
  const selected = new Set(
    plan.robots.flatMap((r) => r.jobs.map((id) => plan.selection?.[id] ?? id)),
  );
  for (const entry of plan.robots)
    for (const id of entry.jobs) {
      const original = initial.items.find((item) => item.id === id)!;
      if (original.kind !== 'cylinder') continue;
      for (const item of initial.items)
        if (
          item.kind === 'cylinder' &&
          item.color === original.color &&
          !selected.has(item.id)
        )
          push((p) => {
            p.selection ??= {};
            p.selection[id] = item.id;
          });
    }
  for (let i = 0; i < plan.robots.length; i++) {
    const r = plan.robots[i];
    for (const offset of [-2, -1, -0.5, 0.5, 1, 2])
      if (r.delay + offset >= 0 && r.delay + offset < FIELD.duration)
        push((p) => {
          p.robots[i].delay += offset;
        });
    // Permute cylinder visits, retaining the physically preloaded cube prefix.
    for (let a = 0; a < r.jobs.length; a++) {
      if (!cylinder(r.jobs[a])) continue;
      for (let j = i; j < plan.robots.length; j++) {
        if (
          initial.robots.find((v) => v.id === plan.robots[j].id)!.role !==
          'beaver'
        )
          continue;
        for (let b = j === i ? a + 1 : 0; b < plan.robots[j].jobs.length; b++) {
          if (!cylinder(plan.robots[j].jobs[b])) continue;
          push((p) => {
            [p.robots[i].jobs[a], p.robots[j].jobs[b]] = [
              p.robots[j].jobs[b],
              p.robots[i].jobs[a],
            ];
          });
        }
      }
      // Directed transfers in both directions, at every position after the
      // recipient's immutable preloaded magazine prefix.
      for (let j = 0; j < plan.robots.length; j++) {
        const recipient = initial.robots.find(
          (v) => v.id === plan.robots[j].id,
        )!;
        if (j === i || recipient.role !== 'beaver') continue;
        for (
          let b = recipient.magazine.length;
          b <= plan.robots[j].jobs.length;
          b++
        )
          push((p) => {
            p.robots[j].jobs.splice(b, 0, p.robots[i].jobs.splice(a, 1)[0]);
          });
      }
    }
  }
  return result;
}

export function searchSchedules(
  initial: World,
  budget = 64,
  progress?: (result: ScheduleSearchResult) => void,
  seed?: TeamSchedule,
): ScheduleSearchResult {
  if (initial.elapsed !== 0)
    throw new Error('Reset the match before searching');
  if (!Number.isFinite(budget)) throw new Error('Search budget must be finite');
  budget = Math.max(4, Math.min(128, Math.floor(budget)));
  const signature = scheduleSignature(initial);
  const first = evaluateSchedule(initial, captureSchedule(initial, false));
  let best = first.measurement,
    schedule = { ...first.schedule, fixedOrder: true },
    tested = 1;
  const baseline = { ...best };
  const solved = structuredClone(initial.items);
  for (const r of initial.robots)
    for (const job of r.jobs)
      Object.assign(
        solved.find((i) => i.id === job.itemId)!,
        job.drop,
        { carrier: null, released: true },
      );
  const requiredScore = scoreWorld(solved).points;
  const seen = new Set<string>();
  const report = () => ({
    signature,
    schedule: structuredClone(schedule),
    baseline,
    best: { ...best },
    tested,
    budget,
  });
  const evaluate = (candidate: TeamSchedule, workLimitMs = 5000) => {
    const key = scheduleKey(candidate);
    if (seen.has(key) || tested >= budget) return;
    seen.add(key);
    const result = evaluateSchedule(
      initial,
      candidate,
      feasible(best, requiredScore) ? best.time + SPEC.dt : FIELD.duration,
      workLimitMs,
    );
    tested++;
    if (better(result.measurement, best, requiredScore)) {
      best = result.measurement;
      schedule = { ...result.schedule, fixedOrder: true };
    }
    progress?.(report());
  };
  if (seed) evaluate(seed, Infinity);
  // First compare staggered launches inferred from the actual two-row grid.
  const frontY = Math.max(...initial.robots.map((r) => r.pose.y));
  for (const spacing of [3, 2, 4]) {
    const early = captureSchedule(initial, false);
    const rear = initial.robots
      .filter((r) => frontY - r.pose.y > 0.07)
      .sort((a, b) => a.pose.x - b.pose.x);
    early.robots.forEach((r) => {
      const index = rear.findIndex((v) => v.id === r.id);
      r.delay = index < 0 ? 0 : (index + 1) * spacing;
    });
    evaluate(early);
  }
  // Bounded deterministic local search: ranking is cheap, finalists are tested
  // against the current best completion time, so a deadlock cannot win.
  while (tested < budget) {
    const previous = scheduleKey(schedule);
    const options = neighbors(initial, schedule)
      .filter((p) => !seen.has(scheduleKey(p)))
      .sort(
        (a, b) => estimatedFinish(initial, a) - estimatedFinish(initial, b),
      );
    if (!options.length) break;
    for (const candidate of options.slice(0, Math.min(16, budget - tested)))
      evaluate(candidate);
    if (scheduleKey(schedule) === previous && options.length <= 16) break;
  }
  // Verify the exact exported fixed order with no wall-clock search cutoff.
  // Candidate wall-clock limits only discard incomplete trials; never certify them.
  const verified = evaluateSchedule(initial, schedule);
  best = verified.measurement;
  return report();
}

export function createScheduleReference() {
  return createWorld('localization');
}
