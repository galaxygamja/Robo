import {
  FIELD,
  OFFICIAL_LAYOUT,
  normalizeAngle,
  polygonDistance,
  transformFootprint,
  type Point,
  type Pose,
} from './simulation.ts';
import {
  createDrone,
  advanceAerial,
  selectObservations,
  type DroneState,
  type PoseSample,
} from './aerial.ts';

// Geometry is a configurable reference scenario, not the unpublished Korean B-4 layout.
export { FIELD, OFFICIAL_LAYOUT };
export type Kind = 'disc' | 'cylinder' | 'cube';
export type PatientColor = 'red' | 'yellow' | 'green';
export type ZoneId = 'H' | 'PCC-L' | 'PCC-R' | 'RZ';
export type ObservationMode = 'localization' | 'drone';
export type FleetSpec = {
  id: string;
  name: string;
  role: 'hamster' | 'beaver';
  color: string;
  pose: Pose;
  delay: number;
  jobs: Job[];
  magazine: string[];
  staging: Point;
  park: Point;
};
export type Phase =
  | 'waiting'
  | 'to-pick'
  | 'align-pick'
  | 'grip'
  | 'verify-pick'
  | 'retract'
  | 'clear-pick'
  | 'to-drop'
  | 'align-drop'
  | 'release'
  | 'verify-release'
  | 'retreat'
  | 'park'
  | 'complete'
  | 'fault';
export type Item = Point & {
  id: string;
  kind: Kind;
  color?: PatientColor;
  carrier: string | null;
  released: boolean;
  selected: boolean;
};
export type Job = {
  itemId: string;
  destination: ZoneId | 'LAB';
  drop: Point;
  slot?: number;
};
export type Robot = {
  id: string;
  name: string;
  role: 'hamster' | 'beaver';
  color: string;
  pose: Pose;
  phase: Phase;
  jobs: Job[];
  jobIndex: number;
  payload: string | null;
  magazine: string[];
  path: Point[];
  trail: Point[];
  target: Pose | null;
  timer: number;
  wait: number;
  delay: number;
  blockedBy: string | null;
  servo: number;
  magazineServo: number;
  sensor: boolean;
  arm: number;
  velocity: Point;
  fault: string | null;
  served: number;
  staging: Point;
  park: Point;
  stagnant: number;
  recoveryTarget: Pose | null;
  recoveryAttempts: number;
  taskReorders: number;
  distanceTravelled: number;
  blockedSeconds: number;
  completedAt: number | null;
  rotationRadians: number;
};
export type LogEntry = {
  time: number;
  text: string;
  level: 'info' | 'success' | 'warning';
};
type ObservationPacket = {
  at: number;
  sequence: number;
  poses: Record<string, PoseSample>;
};
export type World = {
  elapsed: number;
  robots: Robot[];
  items: Item[];
  ended: boolean;
  reason: string;
  logs: LogEntry[];
  locks: Record<string, string>;
  faultRobot: string | null;
  emergencyStopped: boolean;
  safetyReason: string;
  observer: {
    mode: ObservationMode;
    frameAge: number;
    lost: boolean;
    missingId: string | null;
    sampledAt: number;
    sequence: number;
    delayMs: number;
    noiseMm: number;
    sentSequence: number;
    queue: ObservationPacket[];
    poses: Record<string, PoseSample>;
    basePoses: Record<string, PoseSample>;
    unavailableIds: string[];
    conflictingIds: string[];
    speedScale: number;
  };
  drone: DroneState;
  initialItems: Item[];
  initialRobotPoses: Record<string, Pose>;
  coordination: {
    enabled: boolean;
    routeShortcuts: number;
    stagingChanges: number;
    plannedDistanceSaved: number;
    taskPlans: number;
  };
  scenario: 'normal' | 'intermittent' | 'occlusion';
  scheduledMissingId: string | null;
};
export const ZONES: Record<
  ZoneId,
  { x: number; y: number; width: number; height: number }
> = {
  H: OFFICIAL_LAYOUT.healthcare.hospital,
  'PCC-L': OFFICIAL_LAYOUT.healthcare.pccLeft,
  'PCC-R': OFFICIAL_LAYOUT.healthcare.pccRight,
  // RZ extent is a scenario assumption within the starting zone, pending B-1.
  RZ: { x: 0.683, y: 0.02, width: 0.44, height: 0.24 },
};
export const LAB_SLOTS: Point[] = [
  { x: 0.38, y: 0.075 },
  { x: 0.48, y: 0.075 },
  { x: 0.58, y: 0.075 },
];
export const SPEC = {
  bodyWidth: 0.126,
  bodyLength: 0.1,
  speed: 0.22,
  margin: 0.012,
  armExtended: 0.11,
  armRetracted: 0.072,
  droneWidth: 0.15,
  droneLength: 0.15,
  sensorTimeout: 1.5,
  dt: 0.02,
} as const;
export const COLOR_HEX = {
  red: '#ef4444',
  yellow: '#facc15',
  green: '#22c55e',
} as const;
export const PHASE_LABEL: Record<Phase, string> = {
  waiting: '출발 대기',
  'to-pick': '물체 접근',
  'align-pick': '집기 정렬',
  grip: '서보 고정',
  'verify-pick': '감지 확인',
  retract: '집게 수납',
  'clear-pick': '집기 구역 이탈',
  'to-drop': '운반',
  'align-drop': '내려놓기 정렬',
  release: '서보 해제',
  'verify-release': '분리 확인',
  retreat: '후퇴',
  park: '작업구역 비우기',
  complete: '임무 완료',
  fault: '센서 오류 정지',
};
export const itemRadius = (item: Item) =>
  item.kind === 'disc'
    ? 0.028
    : item.kind === 'cube'
      ? Math.SQRT2 * 0.0125
      : 0.01;
const dist = (a: Point, b: Point) => Math.hypot(a.x - b.x, a.y - b.y);
export function tip(pose: Pose, reach: number): Point {
  return {
    x: pose.x - Math.sin(pose.heading) * reach,
    y: pose.y + Math.cos(pose.heading) * reach,
  };
}
const jobOf = (robot: Robot) => robot.jobs[robot.jobIndex];
const approach = (point: Point, heading: number): Pose => ({
  x: point.x + Math.sin(heading) * SPEC.armExtended,
  y: point.y - Math.cos(heading) * SPEC.armExtended,
  heading,
});
const dropHeading = (job: Job) =>
  job.destination === 'LAB' || job.destination === 'RZ' ? Math.PI : 0;
const log = (world: World, text: string, level: LogEntry['level'] = 'info') => {
  world.logs.unshift({ time: world.elapsed, text, level });
  world.logs.length = Math.min(80, world.logs.length);
};

export function createWorld(
  observation: ObservationMode | boolean = 'localization',
  fleet?: FleetSpec[],
): World {
  const observationMode: ObservationMode =
    typeof observation === 'boolean'
      ? observation
        ? 'drone'
        : 'localization'
      : observation;
  const items: Item[] = [];
  const add = (
    id: string,
    kind: Kind,
    point: Point,
    color?: PatientColor,
    selected = true,
    carrier: string | null = null,
  ) =>
    items.push({
      id,
      kind,
      ...point,
      color,
      selected,
      carrier,
      released: false,
    });
  [0.08, 0.15, 0.22].forEach((x, i) => add(`D${i + 1}`, 'disc', { x, y: 0.2 }));
  const counts = { red: 0, yellow: 0, green: 0 };
  OFFICIAL_LAYOUT.groundPoints.forEach((p) => {
    const color: PatientColor =
      p.color === '#ef4444'
        ? 'red'
        : p.color === '#22c55e'
          ? 'green'
          : 'yellow';
    counts[color]++;
    add(
      `${color[0].toUpperCase()}${counts[color]}`,
      'cylinder',
      p,
      color,
      counts[color] <= 3,
    );
  });
  const jobs: Record<string, Job[]> = {
    H1: [
      { itemId: 'D1', destination: 'LAB', drop: LAB_SLOTS[0], slot: 0 },
      { itemId: 'D2', destination: 'LAB', drop: LAB_SLOTS[1], slot: 1 },
      { itemId: 'D3', destination: 'LAB', drop: LAB_SLOTS[2], slot: 2 },
    ],
    H2: [
      { itemId: 'R2', destination: 'H', drop: { x: 0.55, y: 1.045 } },
      { itemId: 'Y2', destination: 'PCC-R', drop: { x: 1.04, y: 1.045 } },
      { itemId: 'G2', destination: 'RZ', drop: { x: 1.01, y: 0.055 } },
    ],
    B1: [
      { itemId: 'C1', destination: 'H', drop: { x: 0.48, y: 1.135 } },
      { itemId: 'C2', destination: 'PCC-L', drop: { x: 0.18, y: 1.135 } },
      { itemId: 'R1', destination: 'H', drop: { x: 0.45, y: 1.045 } },
      { itemId: 'Y1', destination: 'PCC-L', drop: { x: 0.12, y: 1.045 } },
      { itemId: 'G1', destination: 'RZ', drop: { x: 0.75, y: 0.055 } },
    ],
    B2: [
      { itemId: 'C3', destination: 'H', drop: { x: 0.66, y: 1.135 } },
      { itemId: 'C4', destination: 'PCC-R', drop: { x: 0.96, y: 1.135 } },
      { itemId: 'R3', destination: 'H', drop: { x: 0.65, y: 1.045 } },
      { itemId: 'Y3', destination: 'PCC-R', drop: { x: 0.96, y: 1.045 } },
      { itemId: 'G3', destination: 'RZ', drop: { x: 0.88, y: 0.055 } },
    ],
  };
  const make = (spec: FleetSpec): Robot => ({
    ...spec,
    pose: { ...spec.pose },
    jobs: spec.jobs.map((j) => ({ ...j, drop: { ...j.drop } })),
    jobIndex: 0,
    phase: 'waiting',
    payload: null,
    magazine: [...spec.magazine],
    path: [],
    trail: [{ x: spec.pose.x, y: spec.pose.y }],
    target: null,
    timer: 0,
    wait: 0,
    blockedBy: null,
    servo: 0,
    magazineServo: 0,
    sensor: false,
    arm: 0.045,
    velocity: { x: 0, y: 0 },
    fault: null,
    served: 0,
    stagnant: 0,
    recoveryTarget: null,
    recoveryAttempts: 0,
    taskReorders: 0,
    distanceTravelled: 0,
    blockedSeconds: 0,
    completedAt: null,
    rotationRadians: 0,
  });
  const defaults: FleetSpec[] = [
    {
      id: 'B1',
      name: '한가한 비버',
      role: 'beaver',
      pose: { x: 0.9, y: 0.21, heading: 0 },
      delay: 4,
      color: '#60a5fa',
      jobs: jobs.B1,
      magazine: ['C1', 'C2'],
      staging: { x: 0.46, y: 0.6 },
      park: { x: 0.12, y: 0.88 },
    },
    {
      id: 'H1',
      name: '햄스터',
      role: 'hamster',
      pose: { x: 1.055, y: 0.21, heading: Math.PI },
      delay: 7,
      color: '#c084fc',
      jobs: jobs.H1,
      magazine: [],
      staging: { x: 0.6, y: 0.34 },
      park: { x: 0.6, y: 0.34 },
    },
    {
      id: 'B2',
      name: '바쁜 비버',
      role: 'beaver',
      pose: { x: 0.9, y: 0.065, heading: 0 },
      delay: 10,
      color: '#38bdf8',
      jobs: jobs.B2,
      magazine: ['C3', 'C4'],
      staging: { x: 0.68, y: 0.61 },
      park: { x: 1.02, y: 0.88 },
    },
    {
      id: 'H2',
      name: '세 번째 비버 · B3',
      role: 'beaver',
      pose: { x: 1.055, y: 0.065, heading: 0 },
      delay: 13,
      color: '#fb923c',
      jobs: jobs.H2,
      magazine: [],
      staging: { x: 0.4, y: 0.35 },
      park: { x: 0.57, y: 0.87 },
    },
  ];
  const selectedFleet = fleet ?? defaults;
  if (
    !selectedFleet.length ||
    new Set(selectedFleet.map((r) => r.id)).size !== selectedFleet.length ||
    selectedFleet.some(
      (r) =>
        !/^[A-Za-z][A-Za-z0-9_-]{0,31}$/.test(r.id) ||
        !['hamster', 'beaver'].includes(r.role) ||
        ![
          r.pose.x,
          r.pose.y,
          r.pose.heading,
          r.delay,
          r.staging.x,
          r.staging.y,
          r.park.x,
          r.park.y,
        ].every(Number.isFinite) ||
        r.delay < 0 ||
        r.magazine.length > 2 ||
        (r.role === 'hamster' && r.magazine.length),
    )
  )
    throw new Error('Invalid or duplicate fleet configuration');
  const robots = selectedFleet.map(make);
  robots.forEach((robot) =>
    robot.magazine.forEach((id) =>
      add(id, 'cube', robot.pose, undefined, true, robot.id),
    ),
  );
  const assignments = robots.flatMap((r) =>
    r.jobs.map((j) => ({ robot: r, job: j })),
  );
  if (
    new Set(items.map((i) => i.id)).size !== items.length ||
    new Set(assignments.map((a) => a.job.itemId)).size !== assignments.length ||
    assignments.some(({ robot, job }) => {
      const item = items.find((i) => i.id === job.itemId);
      return (
        !item ||
        (item.kind === 'disc') !== (robot.role === 'hamster') ||
        (item.kind === 'cube' && item.carrier !== robot.id) ||
        ![job.drop.x, job.drop.y].every(Number.isFinite)
      );
    })
  )
    throw new Error('Invalid task ownership or repeated physical object');
  return {
    elapsed: 0,
    robots,
    items,
    ended: false,
    reason: '',
    locks: {},
    faultRobot: null,
    emergencyStopped: false,
    safetyReason: '',
    logs: [
      {
        time: 0,
        text:
          observationMode === 'localization'
            ? '드론 없음 · 실시간 좌표 관측 모의로 지상팀 준비'
            : '박쥐 드론 관측으로 지상팀 준비',
        level: 'info',
      },
    ],
    observer: {
      mode: observationMode,
      frameAge: 0,
      lost: false,
      missingId: null,
      sampledAt: -1,
      sequence: 0,
      delayMs: 0,
      noiseMm: 0,
      sentSequence: 0,
      queue: [],
      poses: {},
      basePoses: {},
      unavailableIds: [],
      conflictingIds: [],
      speedScale: 1,
    },
    drone: createDrone(observationMode === 'drone'),
    initialItems: items.map((item) => ({ ...item })),
    initialRobotPoses: Object.fromEntries(
      robots.map((robot) => [robot.id, { ...robot.pose }]),
    ),
    coordination: {
      enabled: true,
      routeShortcuts: 0,
      stagingChanges: 0,
      plannedDistanceSaved: 0,
      taskPlans: 0,
    },
    scenario: 'normal',
    scheduledMissingId: null,
  };
}

export function inside(item: Item, zone: typeof ZONES.H): boolean {
  const r = item.kind === 'cube' ? 0.0125 : itemRadius(item);
  return (
    Number.isFinite(item.x) &&
    Number.isFinite(item.y) &&
    item.x - r > zone.x + 1e-9 &&
    item.x + r < zone.x + zone.width - 1e-9 &&
    item.y - r > zone.y + 1e-9 &&
    item.y + r < zone.y + zone.height - 1e-9
  );
}
export function scoreWorld(items: Item[]) {
  // Score is derived from the final world, never from completed-waypoint counters.
  const seen = new Set<string>();
  const ground = items.filter((item) => {
    if (seen.has(item.id))
      throw new Error(`Duplicate physical item ID: ${item.id}`);
    seen.add(item.id);
    return item.released && !item.carrier;
  });
  const contaminants: ZoneId[] = [];
  const cylinders: Record<ZoneId, Item[]> = {
    H: [],
    'PCC-L': [],
    'PCC-R': [],
    RZ: [],
  };
  for (const zone of Object.keys(ZONES) as ZoneId[]) {
    cylinders[zone] = ground.filter(
      (item) => item.kind === 'cylinder' && inside(item, ZONES[zone]),
    );
    const expected = zone === 'H' ? 'red' : zone === 'RZ' ? 'green' : 'yellow';
    if (cylinders[zone].some((item) => item.color !== expected))
      contaminants.push(zone);
  }
  const valid = (zone: ZoneId) =>
    contaminants.includes(zone) ? 0 : cylinders[zone].length;
  const yellowSplit =
    cylinders['PCC-L'].some((item) => item.color === 'yellow') &&
    cylinders['PCC-R'].some((item) => item.color === 'yellow');
  const discs = ground.filter((item) => item.kind === 'disc');
  const used = new Set<string>();
  const diskCount = LAB_SLOTS.reduce((sum, slot) => {
    const match = discs.find(
      (item) => !used.has(item.id) && dist(item, slot) + 0.028 < 0.03 - 1e-9,
    );
    if (match) used.add(match.id);
    return sum + Number(!!match);
  }, 0);
  const cubes = (['H', 'PCC-L', 'PCC-R'] as ZoneId[]).reduce(
    (sum, zone) =>
      sum +
      Math.min(
        zone === 'H' ? 2 : 1,
        ground.filter(
          (item) => item.kind === 'cube' && inside(item, ZONES[zone]),
        ).length,
      ),
    0,
  );
  const counts = {
    discs: diskCount,
    cubes,
    red: Math.min(3, valid('H')),
    yellow: yellowSplit ? Math.min(3, valid('PCC-L') + valid('PCC-R')) : 0,
    green: Math.min(3, valid('RZ')),
  };
  return {
    counts,
    points: Object.values(counts).reduce((sum, n) => sum + n * 10, 0),
    max: 160,
    contaminants,
    yellowSplit,
    delivered: ground.filter((item) => item.selected).length,
  };
}

function discPolygon(point: Point, radius: number): Point[] {
  const sides = radius >= 0.025 ? 16 : 8;
  return Array.from({ length: sides }, (_, i) => ({
    x:
      point.x +
      (Math.cos((i * 2 * Math.PI) / sides) * radius) /
        Math.cos(Math.PI / sides),
    y:
      point.y +
      (Math.sin((i * 2 * Math.PI) / sides) * radius) /
        Math.cos(Math.PI / sides),
  }));
}
export function robotShapes(
  robot: Robot,
  items: Item[],
  pose = robot.pose,
): Point[][] {
  const shapes = [transformFootprint(pose, 'mecanum')];
  if (robot.arm > 0.051) {
    const a = tip(pose, 0.045),
      b = tip(pose, robot.arm);
    const dx = Math.cos(pose.heading) * 0.009,
      dy = Math.sin(pose.heading) * 0.009;
    shapes.push([
      { x: a.x - dx, y: a.y - dy },
      { x: a.x + dx, y: a.y + dy },
      { x: b.x + dx, y: b.y + dy },
      { x: b.x - dx, y: b.y - dy },
    ]);
  }
  const payload = items.find((item) => item.id === robot.payload);
  if (payload)
    shapes.push(discPolygon(tip(pose, robot.arm), itemRadius(payload)));
  const cube = items.find(
    (item) =>
      item.id === jobOf(robot)?.itemId &&
      item.kind === 'cube' &&
      item.carrier === robot.id,
  );
  if (cube && ['align-drop', 'release', 'verify-release'].includes(robot.phase))
    shapes.push(discPolygon(tip(pose, robot.arm), itemRadius(cube)));
  return shapes;
}
function bounds(shapes: Point[][]) {
  const points = shapes.flat();
  return {
    left: Math.min(...points.map((p) => p.x)),
    right: Math.max(...points.map((p) => p.x)),
    bottom: Math.min(...points.map((p) => p.y)),
    top: Math.max(...points.map((p) => p.y)),
  };
}
function separation(a: Point[][], b: Point[][]) {
  return Math.min(...a.flatMap((p) => b.map((q) => polygonDistance(p, q))));
}
function staticObstacleShapes(
  world: World,
  robot: Robot,
): { id: string; shapes: Point[][] }[] {
  const job = jobOf(robot);
  // Only the active pickup may enter the gripper approach envelope.
  const ignore = ['to-pick', 'align-pick', 'grip', 'verify-pick'].includes(
    robot.phase,
  )
    ? job?.itemId
    : null;
  return world.items
    .filter((item) => !item.carrier && item.id !== ignore)
    .map((item) => ({
      id: item.id,
      shapes: [discPolygon(item, itemRadius(item))],
    }));
}
export function clearance(world: World): number {
  let minimum = Infinity;
  world.robots.forEach((r, i) =>
    world.robots.slice(i + 1).forEach((other) => {
      minimum = Math.min(
        minimum,
        separation(
          robotShapes(r, world.items),
          robotShapes(other, world.items),
        ),
      );
    }),
  );
  return minimum;
}
function obstacleList(world: World, robot: Robot) {
  const obstacles = staticObstacleShapes(world, robot);
  world.robots
    .filter((other) => other.id !== robot.id)
    .forEach((other) =>
      obstacles.push({ id: other.id, shapes: robotShapes(other, world.items) }),
    );
  return obstacles;
}
function blockedShapes(
  shapes: Point[][],
  obstacles: ReturnType<typeof obstacleList>,
): string | null {
  if (
    shapes
      .flat()
      .some(
        (p) =>
          p.x < SPEC.margin - 1e-9 ||
          p.x > FIELD.width - SPEC.margin + 1e-9 ||
          p.y < SPEC.margin - 1e-9 ||
          p.y > FIELD.height - SPEC.margin + 1e-9,
      )
  )
    return '외곽 경계';
  const a = bounds(shapes);
  for (const obstacle of obstacles) {
    const b = bounds(obstacle.shapes);
    if (
      a.right + SPEC.margin < b.left ||
      b.right + SPEC.margin < a.left ||
      a.top + SPEC.margin < b.bottom ||
      b.top + SPEC.margin < a.bottom
    )
      continue;
    if (separation(shapes, obstacle.shapes) < SPEC.margin - 1e-8)
      return obstacle.id;
  }
  return null;
}
function safePose(
  robot: Robot,
  items: Item[],
  pose: Pose,
  obstacles: ReturnType<typeof obstacleList>,
): string | null {
  return blockedShapes(robotShapes(robot, items, pose), obstacles);
}
export function collisionReason(world: World, robot: Robot): string | null {
  return safePose(robot, world.items, robot.pose, obstacleList(world, robot));
}
function convexHull(points: Point[]): Point[] {
  const sorted = points.toSorted((a, b) => a.x - b.x || a.y - b.y);
  const cross = (o: Point, a: Point, b: Point) =>
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  const half = (list: Point[]) => {
    const result: Point[] = [];
    for (const p of list) {
      while (
        result.length >= 2 &&
        cross(result[result.length - 2], result[result.length - 1], p) <= 0
      )
        result.pop();
      result.push(p);
    }
    return result.slice(0, -1);
  };
  return [...half(sorted), ...half([...sorted].reverse())];
}
function segmentFree(
  a: Pose,
  b: Point,
  robot: Robot,
  world: World,
  obstacles: ReturnType<typeof obstacleList>,
) {
  // A translated convex part sweeps its endpoint convex hull exactly. This
  // avoids coarse sampling that could plan through a corner then stall there.
  const shapes = robotShapes(robot, world.items, a).map((part) =>
    convexHull([
      ...part,
      ...part.map((p) => ({ x: p.x + b.x - a.x, y: p.y + b.y - a.y })),
    ]),
  );
  return !blockedShapes(shapes, obstacles);
}

// Both modes share the same optimizer and known practice map. An optimization
// gain is not attributed to owning a drone. A drone adds an observation source.
export function coordinationUsable(world: World): boolean {
  return (
    world.coordination.enabled &&
    !world.observer.lost &&
    !world.emergencyStopped &&
    !world.observer.unavailableIds.length &&
    !world.observer.conflictingIds.length &&
    world.robots.every((r) => {
      const pose = world.observer.poses[r.id];
      return !!pose && world.elapsed - pose.at <= 0.3 && pose.errorMm <= 1;
    })
  );
}

export function smoothObservedPath(
  world: World,
  robot: Robot,
  path: Point[],
): Point[] {
  if (path.length < 2 || !coordinationUsable(world)) return path;
  const obstacles = obstacleList(world, robot),
    result: Point[] = [];
  let current: Pose = robot.pose,
    index = 0;
  while (index < path.length) {
    let farthest = index;
    for (let j = path.length - 1; j > index; j--) {
      if (segmentFree(current, path[j], robot, world, obstacles)) {
        farthest = j;
        break;
      }
    }
    if (!segmentFree(current, path[farthest], robot, world, obstacles))
      return [];
    result.push(path[farthest]);
    current = { ...path[farthest], heading: robot.pose.heading };
    index = farthest + 1;
  }
  const length = (points: Point[]) =>
    points.reduce(
      (sum, p, i) => sum + dist(i ? points[i - 1] : robot.pose, p),
      0,
    );
  const saved = length(path) - length(result);
  if (saved > 0.0001) {
    world.coordination.routeShortcuts++;
    world.coordination.plannedDistanceSaved += saved;
  }
  return result;
}
export function routeSegmentSafe(
  world: World,
  robot: Robot,
  start: Pose,
  end: Point,
): boolean {
  return segmentFree(start, end, robot, world, obstacleList(world, robot));
}

// Replace a long fixed staging detour only when a nearby escape point permits
// the complete loaded turn and subsequent straight corridor to the drop pose.
export function observedStaging(world: World, robot: Robot, job: Job): Point {
  const baseline = robot.staging,
    destination = approach(job.drop, dropHeading(job));
  if (!coordinationUsable(world)) return baseline;
  const obstacles = obstacleList(world, robot);
  const baselineLength =
    dist(robot.pose, baseline) + dist(baseline, destination);
  const candidates = [0.08, 0.12, 0.16, 0.2].map((reach) =>
    tip(robot.pose, -reach),
  );
  let best = baseline,
    bestLength = baselineLength;
  for (const candidate of candidates) {
    const length = dist(robot.pose, candidate) + dist(candidate, destination);
    if (
      length + 0.04 >= bestLength ||
      !segmentFree(robot.pose, candidate, robot, world, obstacles)
    )
      continue;
    const turn = normalizeAngle(destination.heading - robot.pose.heading);
    let free = true;
    for (let i = 0; i <= 64; i++) {
      if (
        safePose(
          robot,
          world.items,
          { ...candidate, heading: robot.pose.heading + (turn * i) / 64 },
          obstacles,
        )
      ) {
        free = false;
        break;
      }
    }
    if (
      !free ||
      !segmentFree(
        { ...candidate, heading: destination.heading },
        destination,
        robot,
        world,
        obstacles,
      )
    )
      continue;
    best = candidate;
    bestLength = length;
  }
  if (best !== baseline) {
    world.coordination.stagingChanges++;
    world.coordination.plannedDistanceSaved += baselineLength - bestLength;
    log(world, `${robot.name} · 충돌 검사된 짧은 회전 대기점 사용`);
  }
  return best;
}
// Eight-connected A*: all other bodies + loose/delivered objects block the grid.
// Pose checks run again per physics step; stale paths never override collision checks.
export function planPath(world: World, robot: Robot, goal: Point): Point[] {
  const obstacles = obstacleList(world, robot);
  if (segmentFree(robot.pose, goal, robot, world, obstacles)) return [goal];
  const cell = 0.025,
    nx = Math.floor(FIELD.width / cell),
    ny = Math.floor(FIELD.height / cell);
  const size = nx * ny,
    costs = new Float64Array(size).fill(Infinity),
    parents = new Int32Array(size).fill(-1),
    closed = new Uint8Array(size),
    safe = new Int8Array(size);
  const node = (index: number): Point => ({
    x: ((index % nx) + 0.5) * cell,
    y: (Math.floor(index / nx) + 0.5) * cell,
  });
  const from =
    Math.max(0, Math.min(ny - 1, Math.floor(robot.pose.y / cell))) * nx +
    Math.max(0, Math.min(nx - 1, Math.floor(robot.pose.x / cell)));
  const open: number[] = [from];
  costs[from] = 0;
  const isFree = (n: number) => {
    if (!safe[n])
      safe[n] = safePose(
        robot,
        world.items,
        { ...node(n), heading: robot.pose.heading },
        obstacles,
      )
        ? -1
        : 1;
    return safe[n] === 1;
  };
  while (open.length) {
    let best = 0;
    for (let i = 1; i < open.length; i++)
      if (
        costs[open[i]] + dist(node(open[i]), goal) <
        costs[open[best]] + dist(node(open[best]), goal)
      )
        best = i;
    const current = open.splice(best, 1)[0];
    if (closed[current]) continue;
    closed[current] = 1;
    const p =
      current === from
        ? robot.pose
        : { ...node(current), heading: robot.pose.heading };
    if (
      dist(p, goal) < cell * 1.8 &&
      segmentFree(p, goal, robot, world, obstacles)
    ) {
      const path: Point[] = [goal];
      let n = current;
      while (n !== from && n >= 0) {
        path.unshift(node(n));
        n = parents[n];
      }
      return path;
    }
    for (const [dx, dy] of [
      [-1, -1],
      [0, -1],
      [1, -1],
      [-1, 0],
      [1, 0],
      [-1, 1],
      [0, 1],
      [1, 1],
    ]) {
      const x = (current % nx) + dx,
        y = Math.floor(current / nx) + dy;
      if (x < 0 || x >= nx || y < 0 || y >= ny) continue;
      const next = y * nx + x;
      if (
        closed[next] ||
        !isFree(next) ||
        !segmentFree(p, node(next), robot, world, obstacles)
      )
        continue;
      const cost = costs[current] + dist(p, node(next));
      if (cost < costs[next]) {
        costs[next] = cost;
        parents[next] = current;
        open.push(next);
      }
    }
  }
  return [];
}
function enter(robot: Robot, phase: Phase) {
  robot.phase = phase;
  robot.timer = 0;
  robot.wait = 0;
  robot.blockedBy = null;
  robot.stagnant = 0;
  robot.recoveryTarget = null;
}
function assignTarget(robot: Robot, target: Pose) {
  robot.target = target;
  robot.path = [];
  robot.wait = 0.8;
}
function lock(world: World, robot: Robot, resource: string) {
  if (world.locks[resource] && world.locks[resource] !== robot.id) {
    robot.blockedBy = `${resource} 사용 중`;
    return false;
  }
  world.locks[resource] = robot.id;
  return true;
}
function unlock(world: World, robot: Robot) {
  Object.keys(world.locks).forEach((key) => {
    if (world.locks[key] === robot.id) delete world.locks[key];
  });
}
export function optimizePendingJobs(world: World, robot: Robot) {
  if (!coordinationUsable(world) || robot.payload || robot.role !== 'beaver')
    return;
  const remaining = robot.jobs.slice(robot.jobIndex);
  // Never reorder the loaded cube magazine or change item ownership/drop slots.
  if (
    remaining.length < 2 ||
    remaining.length > 6 ||
    remaining.some(
      (job) =>
        world.initialItems.find((item) => item.id === job.itemId)?.kind !==
        'cylinder',
    )
  )
    return;
  const measured = world.observer.poses[robot.id];
  const origin = { x: measured.x_mm / 1000, y: measured.y_mm / 1000 };
  const pointOf = (job: Job): Point => {
    const initial = world.initialItems.find((item) => item.id === job.itemId)!;
    const d = world.drone,
      seen = d.objects[job.itemId];
    return d.enabled &&
      !d.videoLost &&
      !d.calibrationLost &&
      d.calibrationValid &&
      d.visibleObjects.includes(job.itemId) &&
      seen?.confirmed &&
      world.elapsed - seen.at <= 0.3 &&
      seen.kind === initial.kind &&
      seen.color === initial.color
      ? { x: seen.x_mm / 1000, y: seen.y_mm / 1000 }
      : initial;
  };
  const cost = (jobs: Job[]) => {
    let current: Point = origin,
      total = 0;
    for (const job of jobs) {
      const object = pointOf(job),
        pick = approach(object, object.y > 0.63 ? Math.PI : 0);
      const drop = approach(job.drop, dropHeading(job));
      total +=
        dist(current, pick) +
        dist(pick, robot.staging) +
        dist(robot.staging, drop) +
        0.11;
      current = tip(drop, -0.11);
    }
    return total + dist(current, robot.park);
  };
  let best = remaining,
    bestCost = cost(remaining);
  const visit = (prefix: Job[], rest: Job[]) => {
    if (!rest.length) {
      const candidateCost = cost(prefix);
      if (candidateCost + 0.04 < bestCost) {
        best = prefix;
        bestCost = candidateCost;
      }
      return;
    }
    rest.forEach((job, index) =>
      visit(
        [...prefix, job],
        rest.filter((_, i) => i !== index),
      ),
    );
  };
  visit([], remaining);
  world.coordination.taskPlans++;
  if (best !== remaining) {
    robot.jobs.splice(robot.jobIndex, remaining.length, ...best);
    robot.taskReorders++;
    log(
      world,
      `${robot.name} · 남은 운반거리 비교: ${best.map((job) => job.itemId).join(' → ')}`,
    );
  }
}
function startJob(world: World, robot: Robot) {
  optimizePendingJobs(world, robot);
  const job = jobOf(robot);
  if (!job) {
    enter(robot, 'park');
    const spot = robot.park;
    assignTarget(robot, { ...spot, heading: robot.pose.heading });
    return;
  }
  const item = world.items.find((i) => i.id === job.itemId)!;
  if (item.kind === 'cube') {
    enter(robot, 'to-drop');
    assignTarget(robot, approach(job.drop, dropHeading(job)));
  } else {
    enter(robot, 'to-pick');
    assignTarget(
      robot,
      approach(item, item.kind === 'disc' || item.y > 0.63 ? Math.PI : 0),
    );
  }
  log(world, `${robot.name} · ${item.id} → ${job.destination}`);
}
function move(world: World, robot: Robot, dt: number): boolean {
  const target = robot.recoveryTarget ?? robot.target;
  if (!target) return false;
  const obstacles = obstacleList(world, robot);
  // Keep gripper facing the requested station while translating (mecanum model).
  const error = normalizeAngle(target.heading - robot.pose.heading);
  if (Math.abs(error) > 0.003) {
    const pose = {
      ...robot.pose,
      heading: normalizeAngle(
        robot.pose.heading +
          Math.sign(error) * Math.min(Math.abs(error), 2 * dt),
      ),
    };
    const collision = safePose(robot, world.items, pose, obstacles);
    if (collision) {
      robot.blockedBy = collision;
      robot.wait += dt;
      return false;
    }
    robot.rotationRadians += Math.abs(
      normalizeAngle(pose.heading - robot.pose.heading),
    );
    robot.pose = pose;
    robot.path = [];
    return false;
  }
  if (dist(robot.pose, target) <= 0.0008) {
    if (robot.recoveryTarget) {
      robot.recoveryTarget = null;
      robot.path = [];
      robot.wait = 0.8;
      robot.stagnant = 0;
      return false;
    }
    return true;
  }
  if (robot.path.length === 0 && robot.wait >= 0.75) {
    robot.path = smoothObservedPath(
      world,
      robot,
      planPath(world, robot, target),
    );
    robot.wait = 0;
  }
  const next = robot.path[0];
  if (!next) {
    robot.wait += dt;
    robot.blockedBy = '통로 확보 대기';
    return false;
  }
  const d = dist(robot.pose, next),
    step = Math.min(d, SPEC.speed * world.observer.speedScale * dt);
  const pose = {
    x: robot.pose.x + ((next.x - robot.pose.x) / (d || 1)) * step,
    y: robot.pose.y + ((next.y - robot.pose.y) / (d || 1)) * step,
    heading: robot.pose.heading,
  };
  const collision = safePose(robot, world.items, pose, obstacles);
  if (collision) {
    robot.blockedBy = collision;
    robot.wait += dt;
    if (robot.wait >= 0.8) robot.path = [];
    return false;
  }
  robot.velocity = {
    x: (pose.x - robot.pose.x) / dt,
    y: (pose.y - robot.pose.y) / dt,
  };
  robot.pose = pose;
  robot.distanceTravelled += step;
  robot.blockedBy = null;
  robot.wait = 0;
  if (dist(pose, next) < 0.0005) robot.path.shift();
  if (dist(robot.trail[robot.trail.length - 1], pose) > 0.015) {
    robot.trail.push({ x: pose.x, y: pose.y });
    if (robot.trail.length > 400) robot.trail.shift();
  }
  return !robot.recoveryTarget && dist(robot.pose, target) <= 0.0008;
}

// Local, bounded yielding: translate with the existing heading before retrying
// a blocked turn/path. No teleport, altered collision margin or task completion.
function recoverTraffic(world: World, robot: Robot) {
  if (!robot.target || robot.recoveryTarget || robot.stagnant < 2) return;
  if (world.robots.some((r) => r.recoveryTarget)) return;
  const obstacles = obstacleList(world, robot);
  const original = { ...robot.pose };
  const candidates: { pose: Pose; cost: number }[] = [];
  for (const radius of [0.12, 0.2, 0.28]) {
    for (let i = 0; i < 16; i++) {
      const angle = (i * Math.PI) / 8;
      const pose = {
        x: original.x + Math.cos(angle) * radius,
        y: original.y + Math.sin(angle) * radius,
        heading: original.heading,
      };
      if (!segmentFree(original, pose, robot, world, obstacles)) continue;
      const turn = normalizeAngle(robot.target.heading - pose.heading);
      let free = true;
      for (let t = 0; t <= 32; t++) {
        if (
          safePose(
            robot,
            world.items,
            { ...pose, heading: pose.heading + (turn * t) / 32 },
            obstacles,
          )
        ) {
          free = false;
          break;
        }
      }
      if (!free) continue;
      candidates.push({ pose, cost: radius + dist(pose, robot.target) });
    }
  }
  // Bound expensive path searches so a failed recovery cannot lock up the UI.
  const chosen = candidates
    .sort((a, b) => a.cost - b.cost)
    .slice(0, 3)
    .find(({ pose }) => {
      robot.pose = { ...pose, heading: robot.target!.heading };
      const reachable = planPath(world, robot, robot.target!).length > 0;
      robot.pose = original;
      return reachable;
    });
  robot.stagnant = 0;
  if (!chosen) {
    // A finished robot is still a physical obstacle. Relocate it, never erase it.
    const parked = world.robots.find(
      (r) =>
        r.phase === 'complete' &&
        r.role === 'beaver' &&
        r.recoveryAttempts < 3 &&
        dist(r.pose, robot.pose) < 0.5,
    );
    if (!parked) return;
    const parkedObstacles = obstacleList(world, parked);
    const options: { pose: Pose; cost: number }[] = [];
    for (const radius of [0.18, 0.3, 0.42])
      for (let i = 0; i < 16; i++) {
        const pose = {
          x: parked.pose.x + radius * Math.cos((i * Math.PI) / 8),
          y: parked.pose.y + radius * Math.sin((i * Math.PI) / 8),
          heading: parked.pose.heading,
        };
        if (!segmentFree(parked.pose, pose, parked, world, parkedObstacles))
          continue;
        const active = world.robots.filter(
          (r) => r.id !== parked.id && r.phase !== 'complete',
        );
        const room = Math.min(
          ...active.flatMap((r) => [
            dist(pose, r.pose),
            r.target ? dist(pose, r.target) : Infinity,
          ]),
        );
        if (room < 0.19) continue;
        options.push({ pose, cost: radius - room });
      }
    const spot = options.sort((a, b) => a.cost - b.cost)[0];
    if (!spot) return;
    parked.park = spot.pose;
    enter(parked, 'park');
    assignTarget(parked, spot.pose);
    parked.recoveryAttempts++;
    log(world, `${parked.name} · 완료 후 주차 위치 양보`, 'warning');
    return;
  }
  robot.recoveryTarget = chosen.pose;
  robot.path = [chosen.pose];
  robot.wait = 0;
  robot.recoveryAttempts++;
  log(
    world,
    `${robot.name} · 혼잡 회복 ${robot.recoveryAttempts}회: 안전한 옆길로 양보`,
    'warning',
  );
}
function fail(world: World, robot: Robot, message: string) {
  enter(robot, 'fault');
  robot.fault = message;
  robot.velocity = { x: 0, y: 0 };
  log(world, `${robot.name} · ${message}`, 'warning');
}
function extendArm(world: World, robot: Robot, dt: number): boolean {
  const previous = robot.arm;
  robot.arm = Math.min(SPEC.armExtended, robot.arm + dt * 0.1);
  const blocked = collisionReason(world, robot);
  if (blocked) {
    robot.arm = previous;
    robot.blockedBy = blocked;
    return false;
  }
  robot.blockedBy = null;
  return true;
}
export function finish(
  world: World,
  reason = '조기 종료 선언 · 최종 배치 판정',
): World {
  if (world.ended) return world;
  world.ended = true;
  world.reason = reason;
  world.drone.phase = 'hold';
  world.robots.forEach((robot) => {
    robot.velocity = { x: 0, y: 0 };
  });
  log(world, reason);
  return world;
}

export function advance(world: World, dt: number = SPEC.dt): World {
  if (world.ended || !Number.isFinite(dt) || dt <= 0) return world;
  // Bound each advance; callers accumulate real time into fixed steps.
  const step = Math.min(dt, SPEC.dt, FIELD.duration - world.elapsed);
  world.elapsed = Math.min(FIELD.duration, world.elapsed + step);
  world.robots.forEach((r) => {
    r.velocity = { x: 0, y: 0 };
  });
  if (world.elapsed >= FIELD.duration - 1e-8) {
    world.elapsed = FIELD.duration;
    return finish(world, '120초 종료 · 최종 배치 판정');
  }
  const observer = world.observer;
  // Explicit repeatable sensor dropout scenario shared by ALL modes. It is
  // never inserted into normal operation or the ordinary no-drone button.
  world.scheduledMissingId =
    world.scenario === 'intermittent' &&
    world.elapsed >= 8 &&
    (world.elapsed - 8) % 10 < 1.2
      ? 'H2'
      : null;
  // The observer can seek a new view while the ground team is held for missing
  // localization. A ground hold must not prevent the drone from recovering it.
  advanceAerial(world, step);
  if (!observer.lost && world.elapsed - observer.sampledAt >= 0.1 - 1e-9) {
    observer.sampledAt = world.elapsed;
    observer.sentSequence++;
    const packet: ObservationPacket = {
      at: world.elapsed,
      sequence: observer.sentSequence,
      poses: {},
    };
    for (const [index, r] of world.robots.entries()) {
      if (r.id === observer.missingId || r.id === world.scheduledMissingId)
        continue;
      const jitter = observer.sentSequence * 1.618 + index * 2.17;
      packet.poses[r.id] = {
        x_mm: r.pose.x * 1000 + Math.cos(jitter) * observer.noiseMm,
        y_mm: r.pose.y * 1000 + Math.sin(jitter) * observer.noiseMm,
        heading_rad: normalizeAngle(r.pose.heading + Math.PI / 2),
        at: world.elapsed,
        source: 'base',
        errorMm: observer.noiseMm,
      };
    }
    observer.queue.push(packet);
  }
  while (
    observer.queue.length &&
    world.elapsed - observer.queue[0].at >= observer.delayMs / 1000 - 1e-9
  ) {
    const packet = observer.queue.shift()!;
    observer.sequence = packet.sequence;
    Object.assign(observer.basePoses, packet.poses);
  }
  observer.queue = observer.queue.slice(-20);
  selectObservations(world, step);
  // Synthetic observation health gate; physics still uses ideal geometry.
  // The separate Python mock controller closes the measured-pose velocity loop.
  world.safetyReason = world.emergencyStopped
    ? '비상정지 잠금 · 초기화해야 해제'
    : observer.conflictingIds.length
      ? `관측원 좌표 불일치 · ${observer.conflictingIds.join(', ')}`
      : observer.unavailableIds.length
        ? observer.noiseMm > 1
          ? '위치 오차 상한 1mm 초과 · 정밀 배치 보류'
          : observer.frameAge > 0.3
            ? `위치 관측 300ms 만료 · ${observer.unavailableIds.join(', ')}`
            : `유효한 위치 관측 대기 · ${observer.unavailableIds.join(', ')}`
        : '';
  if (world.safetyReason) {
    return world;
  }
  for (const robot of world.robots) {
    if (robot.phase === 'fault' || robot.phase === 'complete') continue;
    if (robot.phase === 'waiting') {
      if (world.elapsed >= robot.delay) startJob(world, robot);
      else continue;
    }
    robot.timer += step;
    const previousPose = { ...robot.pose };
    const job = jobOf(robot),
      item = job ? world.items.find((i) => i.id === job.itemId)! : null;
    if (robot.phase === 'to-pick') {
      if (robot.role === 'hamster' && !lock(world, robot, '격리/LAB')) {
        robot.blockedSeconds += step;
        continue;
      }
      if (move(world, robot, step)) enter(robot, 'align-pick');
    } else if (robot.phase === 'align-pick') {
      if (!extendArm(world, robot, step)) continue;
      if (
        robot.timer >= 0.8 &&
        item &&
        dist(tip(robot.pose, robot.arm), item) < 0.002
      )
        enter(robot, 'grip');
      else if (robot.timer >= 3) fail(world, robot, '집기 정렬 시간초과');
    } else if (robot.phase === 'grip') {
      robot.servo = Math.min(1, robot.timer / 0.5);
      if (robot.timer >= 0.5) enter(robot, 'verify-pick');
    } else if (robot.phase === 'verify-pick') {
      robot.sensor = world.faultRobot !== robot.id && robot.timer >= 0.25;
      if (robot.sensor && item) {
        robot.payload = item.id;
        const collision = collisionReason(world, robot);
        if (collision) {
          robot.payload = null;
          fail(world, robot, `집기 공간 부족 · ${collision}`);
          continue;
        }
        item.carrier = robot.id;
        item.released = false;
        enter(robot, 'retract');
        log(world, `${robot.name} · ${item.id} 고정 확인`, 'success');
      } else if (robot.timer >= SPEC.sensorTimeout)
        fail(world, robot, '물체 감지 시간초과');
    } else if (robot.phase === 'retract') {
      robot.arm = Math.max(SPEC.armRetracted, robot.arm - step * 0.1);
      if (robot.timer >= 0.45) {
        enter(robot, 'clear-pick');
        // Leave the crowded object row before rotating a loaded gripper.
        const staging =
          robot.role === 'hamster'
            ? tip(robot.pose, -0.1)
            : observedStaging(world, robot, job);
        assignTarget(robot, { ...staging, heading: robot.pose.heading });
      }
    } else if (robot.phase === 'clear-pick') {
      if (move(world, robot, step)) {
        enter(robot, 'to-drop');
        assignTarget(robot, approach(job.drop, dropHeading(job)));
      }
    } else if (robot.phase === 'to-drop') {
      // Drop zones serialize the two beavers. Hamsters serialize the narrow LAB work area.
      if (
        !lock(
          world,
          robot,
          job.destination === 'LAB' ? '격리/LAB' : job.destination,
        )
      ) {
        robot.blockedSeconds += step;
        continue;
      }
      if (move(world, robot, step)) enter(robot, 'align-drop');
    } else if (robot.phase === 'align-drop') {
      if (!extendArm(world, robot, step)) continue;
      if (
        robot.timer >= 0.8 &&
        dist(tip(robot.pose, robot.arm), job.drop) < 0.001
      )
        enter(robot, 'release');
      else if (robot.timer >= 3) fail(world, robot, '배치 정렬 시간초과');
    } else if (robot.phase === 'release') {
      if (item?.kind === 'cube')
        robot.magazineServo = Math.min(1, robot.timer / 0.6);
      else robot.servo = Math.max(0, 1 - robot.timer / 0.5);
      if (robot.timer >= 0.6) enter(robot, 'verify-release');
    } else if (robot.phase === 'verify-release') {
      const confirmed = world.faultRobot !== robot.id && robot.timer >= 0.3;
      if (confirmed && item) {
        const location = tip(robot.pose, robot.arm);
        item.x = location.x;
        item.y = location.y;
        item.carrier = null;
        item.released = true;
        robot.payload = null;
        robot.magazine = robot.magazine.filter((id) => id !== item.id);
        robot.sensor = false;
        robot.magazineServo = 0;
        robot.arm = 0.045;
        robot.served++;
        enter(robot, 'retreat');
        const exit =
          robot.role === 'hamster' ? robot.park : tip(robot.pose, -0.11);
        assignTarget(robot, { ...exit, heading: robot.pose.heading });
        log(
          world,
          `${robot.name} · ${item.id} ${job.destination} 분리·배치 확인`,
          'success',
        );
      } else if (robot.timer >= SPEC.sensorTimeout)
        fail(world, robot, '해제 확인 시간초과');
    } else if (robot.phase === 'retreat') {
      if (move(world, robot, step)) {
        unlock(world, robot);
        robot.jobIndex++;
        startJob(world, robot);
      }
    } else if (robot.phase === 'park') {
      if (move(world, robot, step)) {
        enter(robot, 'complete');
        robot.completedAt = world.elapsed;
        unlock(world, robot);
      }
    }
    world.items
      .filter((i) => i.carrier === robot.id)
      .forEach((i) => {
        // The current cube feeds continuously from the mock hopper to its outlet.
        // No object jumps from a pickup coordinate to a destination coordinate.
        const reach =
          i.kind === 'cube'
            ? robot.phase === 'release'
              ? robot.arm * Math.min(1, robot.timer / 0.6)
              : robot.phase === 'verify-release'
                ? robot.arm
                : 0
            : robot.arm;
        const position =
          robot.payload === i.id || (job?.itemId === i.id && i.kind === 'cube')
            ? tip(robot.pose, reach)
            : robot.pose;
        i.x = position.x;
        i.y = position.y;
      });
    if (
      robot.blockedBy &&
      dist(previousPose, robot.pose) < 1e-7 &&
      Math.abs(normalizeAngle(previousPose.heading - robot.pose.heading)) < 1e-7
    ) {
      robot.blockedSeconds += step;
      robot.stagnant += step;
      if (
        ['to-pick', 'clear-pick', 'to-drop', 'retreat', 'park'].includes(
          robot.phase,
        )
      )
        recoverTraffic(world, robot);
    } else robot.stagnant = 0;
  }
  if (world.robots.every((r) => r.phase === 'complete'))
    finish(world, '임무 완료 · 조기 종료 모의');
  return world;
}
