import type { World } from './mission.ts';
import { normalizeAngle, type Point } from './simulation.ts';

export type SpatialPoint = Point & { z: number };
export type Occluder = { id: string; min: SpatialPoint; max: SpatialPoint };
export type PoseSample = {
  x_mm: number;
  y_mm: number;
  heading_rad: number;
  at: number;
  source: 'base' | 'drone';
  errorMm: number;
};
export type ObjectSighting = {
  id: string;
  x_mm: number;
  y_mm: number;
  at: number;
  streak: number;
  confirmed: boolean;
  kind: string;
  color?: string;
};
type Target = SpatialPoint & {
  id: string;
  weight: number;
  size: number;
  uncertainty?: number;
};
type AerialPacket = {
  at: number;
  sequence: number;
  valid: boolean;
  poses: Record<string, PoseSample>;
  objects: Record<string, ObjectSighting>;
};
export type DroneState = {
  enabled: boolean;
  strategy: 'active' | 'hover';
  pose: Point;
  altitude: number;
  phase: 'ground' | 'takeoff' | 'hover' | 'hold' | 'reposition';
  target: SpatialPoint;
  reason: string;
  focusIds: string[];
  videoLost: boolean;
  calibrationLost: boolean;
  delayMs: number;
  occlusionId: string | null;
  lastPlanAt: number;
  lastSampleAt: number;
  sequence: number;
  frameAt: number;
  queue: AerialPacket[];
  observations: Record<string, PoseSample>;
  objects: Record<string, ObjectSighting>;
  visibleRobots: string[];
  captureStreaks: Record<string, { sequence: number; streak: number }>;
  visibleObjects: string[];
  recoveredIds: string[];
  recoveredRobotSeconds: number;
  holdSeconds: number;
  distance: number;
  anchorIds: string[];
  calibrationValid: boolean;
  frameErrorMm: number;
  trail: Point[];
  replans: number;
};
// All camera, marker and obstruction dimensions below are explicit synthetic
// assumptions, not the supplied toy drone's specs or official arena fixtures.
export const AERIAL = {
  minZ: 0.6,
  maxZ: 1.0,
  cruiseZ: 0.8,
  speed: 0.28,
  climb: 0.4,
  tanHalfX: Math.tan(Math.PI / 4),
  tanHalfY: Math.tan((39 * Math.PI) / 180),
  imageWidth: 1920,
  imageHeight: 1080,
  tagSize: 0.04,
  tagHeight: 0.075,
  ttl: 0.3,
} as const;
export const REFERENCE_MARKERS: SpatialPoint[] = [
  { x: 0.02, y: 0.02, z: 0 },
  { x: 0.57, y: 0.02, z: 0 },
  { x: 1.123, y: 0.02, z: 0 },
  { x: 0.02, y: 0.59, z: 0 },
  { x: 0.57, y: 0.59, z: 0 },
  { x: 1.123, y: 0.59, z: 0 },
  { x: 0.02, y: 1.161, z: 0 },
  { x: 0.57, y: 1.161, z: 0 },
  { x: 1.123, y: 1.161, z: 0 },
];
const distance = (a: Point, b: Point) => Math.hypot(a.x - b.x, a.y - b.y);
export function createDrone(enabled: boolean): DroneState {
  return {
    enabled,
    strategy: 'active',
    pose: { x: 0.742, y: 0.14 },
    altitude: 0,
    phase: 'ground',
    target: { x: 0.5715, y: 0.61, z: 0.8 },
    reason: '전체 경기장 탐색 준비',
    focusIds: [],
    videoLost: false,
    calibrationLost: false,
    delayMs: 80,
    occlusionId: null,
    lastPlanAt: -Infinity,
    lastSampleAt: -Infinity,
    sequence: 0,
    frameAt: -Infinity,
    queue: [],
    observations: {},
    objects: {},
    captureStreaks: {},
    visibleRobots: [],
    visibleObjects: [],
    recoveredIds: [],
    recoveredRobotSeconds: 0,
    holdSeconds: 0,
    distance: 0,
    anchorIds: [],
    calibrationValid: false,
    frameErrorMm: Infinity,
    trail: [{ x: 0.742, y: 0.14 }],
    replans: 0,
  };
}
export function footprint(camera: SpatialPoint, height = 0) {
  const z = Math.max(0, camera.z - height);
  return {
    x: camera.x - z * AERIAL.tanHalfX,
    y: camera.y - z * AERIAL.tanHalfY,
    width: 2 * z * AERIAL.tanHalfX,
    height: 2 * z * AERIAL.tanHalfY,
  };
}
export function rayBlocked(
  camera: SpatialPoint,
  target: SpatialPoint,
  box: Occluder,
): boolean {
  let low = 0,
    high = 1;
  for (const key of ['x', 'y', 'z'] as const) {
    const delta = camera[key] - target[key];
    if (Math.abs(delta) < 1e-12) {
      if (target[key] < box.min[key] || target[key] > box.max[key])
        return false;
    } else {
      const a = (box.min[key] - target[key]) / delta,
        b = (box.max[key] - target[key]) / delta;
      low = Math.max(low, Math.min(a, b));
      high = Math.min(high, Math.max(a, b));
      if (low > high) return false;
    }
  }
  return high > 1e-6 && low < 1 - 1e-6;
}
export function canObserve(
  camera: SpatialPoint,
  target: SpatialPoint,
  boxes: Occluder[] = [],
  size = 0.04,
) {
  if (camera.z <= target.z) return false;
  const f = footprint(camera, target.z);
  if (
    target.x - size / 2 < f.x ||
    target.x + size / 2 > f.x + f.width ||
    target.y - size / 2 < f.y ||
    target.y + size / 2 > f.y + f.height
  )
    return false;
  if (
    Math.min(
      (size * AERIAL.imageWidth) / f.width,
      (size * AERIAL.imageHeight) / f.height,
    ) < 12
  )
    return false;
  const rays = [
    target,
    ...[-1, 1].flatMap((x) =>
      [-1, 1].map((y) => ({
        ...target,
        x: target.x + (x * size) / 2,
        y: target.y + (y * size) / 2,
      })),
    ),
  ];
  return !boxes.some((b) => rays.some((p) => rayBlocked(camera, p, b)));
}
export function calibrationAt(camera: SpatialPoint, boxes: Occluder[] = []) {
  const anchors = REFERENCE_MARKERS.map((p, i) => ({
    p,
    id: `F${i + 1}`,
  })).filter(({ p }) => canObserve(camera, p, boxes));
  const span = (key: 'x' | 'y') =>
    Math.max(...anchors.map((a) => a.p[key])) -
    Math.min(...anchors.map((a) => a.p[key]));
  return {
    valid: anchors.length >= 4 && span('x') >= 0.5 && span('y') >= 0.5,
    ids: anchors.map((a) => a.id),
  };
}
export function sceneOccluders(world: World): Occluder[] {
  const id = world.drone.occlusionId;
  const r = world.robots.find((r) => r.id === id);
  if (!r) return [];
  // Fixed overhead obstruction at the selected robot's initial position.
  // It blocks light, not floor travel; leaving it changes the available view.
  const origin = world.initialRobotPoses[r.id];
  return [
    {
      id: 'test-hood',
      min: { x: origin.x - 0.075, y: origin.y - 0.075, z: 0.18 },
      max: { x: origin.x + 0.075, y: origin.y + 0.075, z: 0.24 },
    },
  ];
}
function missionTargets(world: World): Target[] {
  return world.robots.flatMap((r) => {
    const seen = world.observer.poses[r.id];
    const point = seen
      ? { x: seen.x_mm / 1000, y: seen.y_mm / 1000 }
      : world.initialRobotPoses[r.id];
    const baseMissing =
      world.observer.missingId === r.id ||
      world.scheduledMissingId === r.id ||
      world.observer.noiseMm > 1;
    const age = seen ? world.elapsed - seen.at : 1;
    const precision = ['align-pick', 'align-drop', 'verify-release'].includes(
      r.phase,
    );
    const target: Target = {
      ...point,
      z: AERIAL.tagHeight,
      id: r.id,
      size: AERIAL.tagSize,
      uncertainty: world.observer.unavailableIds.includes(r.id)
        ? Math.min(0.07, 0.01 + age * 0.22)
        : 0.005,
      weight:
        2 +
        (baseMissing ? 30 : 0) +
        (age > 0.2 ? 12 : 0) +
        (precision ? 4 : 0) +
        (r.blockedBy ? 3 : 0),
    };
    const job = r.jobs[r.jobIndex];
    if (!job) return [target];
    // Job destinations and initial task map are known planning data. Unseen
    // moving-object ground truth is never fed to the viewpoint optimizer.
    const last = world.drone.objects[job.itemId];
    const initial = world.initialItems.find((i) => i.id === job.itemId);
    const taskPoint =
      r.payload || initial?.kind === 'cube'
        ? job.drop
        : last
          ? { x: last.x_mm / 1000, y: last.y_mm / 1000 }
          : initial;
    return taskPoint
      ? [
          target,
          {
            ...taskPoint,
            z: 0.02,
            id: job.itemId,
            size: 0.02,
            weight: precision ? 5 : 1.5,
          },
        ]
      : [target];
  });
}
export function chooseView(
  camera: SpatialPoint,
  targets: Target[],
  boxes: Occluder[],
) {
  const value = (p: SpatialPoint) => {
    if (!calibrationAt(p, boxes).valid) return -Infinity;
    const coverage = targets.reduce((sum, t) => {
      const u = t.uncertainty ?? 0;
      const samples = [
        t,
        { ...t, x: t.x - u },
        { ...t, x: t.x + u },
        { ...t, y: t.y - u },
        { ...t, y: t.y + u },
      ];
      const visible = samples.filter((sample) =>
        canObserve(p, sample, boxes, t.size),
      ).length;
      // A stale location is a search region, not a precise hidden ground truth.
      return (
        sum +
        t.weight *
          (visible === samples.length ? 1 : (visible / samples.length) * 0.25)
      );
    }, 0);
    return (
      coverage -
      2 * distance(p, camera) -
      2 * Math.abs(p.z - camera.z) -
      0.6 * Math.abs(p.z - AERIAL.cruiseZ)
    );
  };
  let best = { pose: camera, value: value(camera) };
  for (const z of [0.65, 0.8, 1])
    for (const x of [0.18, 0.38, 0.5715, 0.78, 0.963])
      for (const y of [0.2, 0.4, 0.61, 0.8, 0.98]) {
        const pose = { x, y, z },
          score = value(pose);
        if (score > best.value + 0.35) best = { pose, value: score };
      }
  return {
    ...best,
    visible: targets
      .filter((t) => canObserve(best.pose, t, boxes, t.size))
      .map((t) => t.id),
  };
}
export function advanceAerial(world: World, dt: number) {
  const d = world.drone;
  if (!d.enabled) return;
  if (world.emergencyStopped || world.observer.lost || d.videoLost) {
    d.phase = 'hold';
    d.calibrationValid = false;
    d.reason = '영상·정지 상태 확인 대기';
    d.visibleRobots = [];
    d.visibleObjects = [];
    d.captureStreaks = {};
    return;
  }
  const boxes = sceneOccluders(world);
  if (d.altitude < AERIAL.minZ - 1e-9) {
    d.phase = 'takeoff';
    d.altitude = Math.min(AERIAL.minZ, d.altitude + AERIAL.climb * dt);
    d.reason = '시작구역에서 관측 높이 확보';
  } else {
    if (world.elapsed - d.lastPlanAt >= 0.5) {
      d.lastPlanAt = world.elapsed;
      if (d.strategy === 'active') {
        const targets = missionTargets(world);
        const chosen = chooseView({ ...d.pose, z: d.altitude }, targets, boxes);
        if (Number.isFinite(chosen.value)) {
          if (
            distance(chosen.pose, d.target) > 0.05 ||
            Math.abs(chosen.pose.z - d.target.z) > 0.05
          )
            d.replans++;
          d.target = chosen.pose;
          d.focusIds = targets
            .toSorted((a, b) => b.weight - a.weight)
            .slice(0, 3)
            .map((t) => t.id);
          d.reason = world.observer.missingId
            ? `${world.observer.missingId} 가림 복구 시점 탐색`
            : world.robots.some((r) => r.blockedBy)
              ? '혼잡 로봇과 다음 작업구역 함께 관측'
              : '로봇·작업 물체와 기준점 동시 관측';
        } else d.reason = '기준점이 보이는 관측 위치 확보';
      } else {
        d.target = { x: 0.5715, y: 0.61, z: 0.8 };
        d.reason = '중앙 정지 관측';
      }
    }
    const dist = distance(d.pose, d.target),
      move = Math.min(dist, AERIAL.speed * dt);
    const previous = { ...d.pose };
    if (dist > 1e-9)
      d.pose = {
        x: d.pose.x + ((d.target.x - d.pose.x) * move) / dist,
        y: d.pose.y + ((d.target.y - d.pose.y) * move) / dist,
      };
    d.altitude +=
      Math.sign(d.target.z - d.altitude) *
      Math.min(Math.abs(d.target.z - d.altitude), AERIAL.climb * dt);
    d.distance += distance(previous, d.pose);
    d.phase =
      dist > 0.015 || Math.abs(d.altitude - d.target.z) > 0.015
        ? 'reposition'
        : 'hover';
    if (distance(d.trail[d.trail.length - 1], d.pose) > 0.02) {
      d.trail.push({ ...d.pose });
      d.trail = d.trail.slice(-400);
    }
  }
  const camera = { ...d.pose, z: d.altitude };
  const calibration = calibrationAt(camera, boxes);
  d.anchorIds = calibration.ids;
  d.calibrationValid =
    calibration.valid && !d.calibrationLost && d.altitude >= AERIAL.minZ;
  d.frameErrorMm =
    0.35 +
    Math.max(0, d.altitude - 0.65) * 0.4 +
    (d.phase === 'reposition' ? 0.25 : 0);
  if (world.elapsed - d.lastSampleAt >= 0.1 - 1e-9) {
    d.lastSampleAt = world.elapsed;
    d.sequence++;
    const packet: AerialPacket = {
      at: world.elapsed,
      sequence: d.sequence,
      valid: d.calibrationValid,
      poses: {},
      objects: {},
    };
    const captureStreaks: DroneState['captureStreaks'] = {};
    if (packet.valid) {
      for (const r of world.robots) {
        if (!canObserve(camera, { ...r.pose, z: AERIAL.tagHeight }, boxes))
          continue;
        packet.poses[r.id] = {
          x_mm: r.pose.x * 1000,
          y_mm: r.pose.y * 1000,
          heading_rad: normalizeAngle(r.pose.heading + Math.PI / 2),
          at: world.elapsed,
          source: 'drone',
          errorMm: d.frameErrorMm,
        };
      }
      for (const item of world.items.filter((i) => !i.carrier)) {
        if (
          !canObserve(
            camera,
            { ...item, z: 0.02 },
            boxes,
            item.kind === 'disc' ? 0.056 : 0.02,
          )
        )
          continue;
        const prev = d.captureStreaks[item.id];
        const streak = prev?.sequence === d.sequence - 1 ? prev.streak + 1 : 1;
        captureStreaks[item.id] = { sequence: d.sequence, streak };
        packet.objects[item.id] = {
          id: item.id,
          x_mm: item.x * 1000,
          y_mm: item.y * 1000,
          at: world.elapsed,
          streak,
          confirmed: streak >= 3,
          kind: item.kind,
          color: item.color,
        };
      }
    }
    d.captureStreaks = captureStreaks;
    d.queue.push(packet);
  }
  while (
    d.queue.length &&
    world.elapsed - d.queue[0].at >= d.delayMs / 1000 - 1e-9
  ) {
    const packet = d.queue.shift()!;
    if (packet.at < d.frameAt) continue;
    d.frameAt = packet.at;
    // Replace per-frame visibility. A missed target is not a fresh detection.
    d.visibleRobots = packet.valid ? Object.keys(packet.poses) : [];
    d.visibleObjects = packet.valid ? Object.keys(packet.objects) : [];
    if (packet.valid && d.calibrationValid) {
      Object.assign(d.observations, packet.poses);
      Object.assign(d.objects, packet.objects);
    }
  }
  d.queue = d.queue.slice(-20);
  if (!d.calibrationValid || world.elapsed - d.frameAt > AERIAL.ttl) {
    d.visibleRobots = [];
    d.visibleObjects = [];
  }
}

export function selectObservations(world: World, dt: number) {
  const o = world.observer,
    d = world.drone;
  o.unavailableIds = [];
  o.conflictingIds = [];
  d.recoveredIds = [];
  let scale = 1;
  for (const r of world.robots) {
    const base = o.basePoses[r.id],
      air = d.observations[r.id];
    const usable = (p: PoseSample | undefined) =>
      !!p && world.elapsed - p.at <= AERIAL.ttl + 1e-9 && p.errorMm <= 1;
    const baseOK =
      usable(base) && o.missingId !== r.id && !o.lost && o.noiseMm <= 1;
    const airOK =
      usable(air) &&
      d.enabled &&
      !d.videoLost &&
      !d.calibrationLost &&
      d.calibrationValid &&
      d.visibleRobots.includes(r.id) &&
      !o.lost;
    if (
      baseOK &&
      airOK &&
      (Math.hypot(base.x_mm - air.x_mm, base.y_mm - air.y_mm) >
        base.errorMm + air.errorMm + 220 * Math.abs(base.at - air.at) + 1e-6 ||
        Math.abs(normalizeAngle(base.heading_rad - air.heading_rad)) >
          2 * Math.abs(base.at - air.at) + 0.06)
    ) {
      o.conflictingIds.push(r.id);
      o.unavailableIds.push(r.id);
      continue;
    }
    const selected =
      baseOK && airOK
        ? base.at >= air.at
          ? base
          : air
        : baseOK
          ? base
          : airOK
            ? air
            : undefined;
    if (!selected) {
      o.unavailableIds.push(r.id);
      continue;
    }
    if (selected.at < (o.poses[r.id]?.at ?? -Infinity)) {
      o.unavailableIds.push(r.id);
      continue;
    }
    o.poses[r.id] = selected;
    if (selected.source === 'drone' && !baseOK) d.recoveredIds.push(r.id);
    if (selected.errorMm > 0.7 || world.elapsed - selected.at > 0.24)
      scale = Math.min(scale, 0.65);
  }
  d.recoveredRobotSeconds += d.recoveredIds.length * dt;
  o.speedScale = scale;
  o.frameAge = Math.max(
    ...world.robots.map((r) => world.elapsed - (o.poses[r.id]?.at ?? 0)),
  );
  if (o.unavailableIds.length) d.holdSeconds += dt;
}
