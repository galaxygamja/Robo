import {
  FIELD,
  OFFICIAL_LAYOUT,
  normalizeAngle,
  polygonDistance,
  transformFootprint,
  type Point,
  type Pose,
} from './simulation.ts';

// Geometry is a configurable reference scenario, not the unpublished Korean B-4 layout.
export { FIELD, OFFICIAL_LAYOUT };
export type Kind = 'disc' | 'cylinder' | 'cube';
export type PatientColor = 'red' | 'yellow' | 'green';
export type ZoneId = 'H' | 'PCC-L' | 'PCC-R' | 'RZ';
export type ObservationMode = 'fixed' | 'drone';
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
};
export type LogEntry = {
  time: number;
  text: string;
  level: 'info' | 'success' | 'warning';
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
  observer: {
    mode: ObservationMode;
    cameraCount: number;
    frameAge: number;
    lost: boolean;
  };
  drone: {
    enabled: boolean;
    pose: Point;
    altitude: number;
    phase: 'ground' | 'takeoff' | 'hover' | 'hold';
  };
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
  observation: ObservationMode | boolean = 'fixed',
): World {
  const observationMode: ObservationMode =
    typeof observation === 'boolean'
      ? observation
        ? 'drone'
        : 'fixed'
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
      { itemId: 'D3', destination: 'LAB', drop: LAB_SLOTS[2], slot: 2 },
    ],
    H2: [{ itemId: 'D2', destination: 'LAB', drop: LAB_SLOTS[1], slot: 1 }],
    B1: [
      { itemId: 'C1', destination: 'H', drop: { x: 0.48, y: 1.135 } },
      { itemId: 'C2', destination: 'PCC-L', drop: { x: 0.18, y: 1.135 } },
      { itemId: 'R1', destination: 'H', drop: { x: 0.45, y: 1.045 } },
      { itemId: 'Y1', destination: 'PCC-L', drop: { x: 0.12, y: 1.045 } },
      { itemId: 'G1', destination: 'RZ', drop: { x: 0.75, y: 0.055 } },
      { itemId: 'R2', destination: 'H', drop: { x: 0.55, y: 1.045 } },
    ],
    B2: [
      { itemId: 'C3', destination: 'H', drop: { x: 0.66, y: 1.135 } },
      { itemId: 'C4', destination: 'PCC-R', drop: { x: 0.96, y: 1.135 } },
      { itemId: 'R3', destination: 'H', drop: { x: 0.65, y: 1.045 } },
      { itemId: 'Y3', destination: 'PCC-R', drop: { x: 0.96, y: 1.045 } },
      { itemId: 'G3', destination: 'RZ', drop: { x: 0.88, y: 0.055 } },
      { itemId: 'Y2', destination: 'PCC-R', drop: { x: 1.04, y: 1.045 } },
      { itemId: 'G2', destination: 'RZ', drop: { x: 1.01, y: 0.055 } },
    ],
  };
  const make = (
    id: string,
    name: string,
    role: Robot['role'],
    pose: Pose,
    delay: number,
    color: string,
  ): Robot => ({
    id,
    name,
    role,
    color,
    pose,
    delay,
    jobs: jobs[id],
    jobIndex: 0,
    phase: 'waiting',
    payload: null,
    magazine:
      role === 'beaver' ? (id === 'B1' ? ['C1', 'C2'] : ['C3', 'C4']) : [],
    path: [],
    trail: [{ x: pose.x, y: pose.y }],
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
  });
  const robots = [
    make(
      'B1',
      '한가한 비버',
      'beaver',
      { x: 0.9, y: 0.21, heading: 0 },
      4,
      '#60a5fa',
    ),
    make(
      'H1',
      '햄스터 1',
      'hamster',
      { x: 1.055, y: 0.21, heading: Math.PI },
      7,
      '#c084fc',
    ),
    make(
      'B2',
      '바쁜 비버',
      'beaver',
      { x: 0.9, y: 0.065, heading: 0 },
      10,
      '#38bdf8',
    ),
    make(
      'H2',
      '햄스터 2',
      'hamster',
      { x: 1.055, y: 0.065, heading: Math.PI },
      13,
      '#e879f9',
    ),
  ];
  robots.forEach((robot) =>
    robot.magazine.forEach((id) =>
      add(id, 'cube', robot.pose, undefined, true, robot.id),
    ),
  );
  return {
    elapsed: 0,
    robots,
    items,
    ended: false,
    reason: '',
    locks: {},
    faultRobot: null,
    logs: [
      {
        time: 0,
        text:
          observationMode === 'fixed'
            ? '드론 없음 · 고정 카메라 2대 관측으로 지상팀 준비'
            : '박쥐 드론 관측으로 지상팀 준비',
        level: 'info',
      },
    ],
    observer: {
      mode: observationMode,
      cameraCount: observationMode === 'fixed' ? 2 : 1,
      frameAge: 0,
      lost: false,
    },
    drone: {
      enabled: observationMode === 'drone',
      pose: { x: 0.742, y: 0.14 },
      altitude: 0,
      phase: 'ground',
    },
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
function startJob(world: World, robot: Robot) {
  const job = jobOf(robot);
  if (!job) {
    enter(robot, 'park');
    const spot =
      robot.id === 'H1'
        ? { x: 0.6, y: 0.34 }
        : robot.id === 'H2'
          ? { x: 0.1, y: 0.09 }
          : robot.id === 'B1'
            ? { x: 0.12, y: 0.88 }
            : { x: 1.02, y: 0.88 };
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
  const target = robot.target;
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
    robot.pose = pose;
    robot.path = [];
    return false;
  }
  if (dist(robot.pose, target) <= 0.0008) return true;
  if (robot.path.length === 0 && robot.wait >= 0.75) {
    robot.path = planPath(world, robot, target);
    robot.wait = 0;
  }
  const next = robot.path[0];
  if (!next) {
    robot.wait += dt;
    robot.blockedBy = '통로 확보 대기';
    return false;
  }
  const d = dist(robot.pose, next),
    step = Math.min(d, SPEC.speed * dt);
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
  robot.blockedBy = null;
  robot.wait = 0;
  if (dist(pose, next) < 0.0005) robot.path.shift();
  if (dist(robot.trail[robot.trail.length - 1], pose) > 0.015) {
    robot.trail.push({ x: pose.x, y: pose.y });
    if (robot.trail.length > 400) robot.trail.shift();
  }
  return dist(robot.pose, target) <= 0.0008;
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
  observer.frameAge = observer.lost
    ? observer.frameAge + step
    : world.elapsed % 0.1;
  if (observer.frameAge > 0.5) {
    world.drone.phase = 'hold';
    return world;
  }
  const drone = world.drone;
  if (drone.enabled) {
    drone.altitude = Math.min(0.8, world.elapsed * 0.4);
    drone.phase = world.elapsed < 2 ? 'takeoff' : 'hover';
    const fraction = Math.min(1, Math.max(0, (world.elapsed - 2) / 1.5));
    drone.pose = {
      x: 0.742 + (0.5715 - 0.742) * fraction,
      y: 0.14 + (0.61 - 0.14) * fraction,
    };
  }
  for (const robot of world.robots) {
    if (robot.phase === 'fault' || robot.phase === 'complete') continue;
    if (robot.phase === 'waiting') {
      if (world.elapsed >= robot.delay) startJob(world, robot);
      else continue;
    }
    robot.timer += step;
    const job = jobOf(robot),
      item = job ? world.items.find((i) => i.id === job.itemId)! : null;
    if (robot.phase === 'to-pick') {
      if (robot.role === 'hamster' && !lock(world, robot, '격리/LAB')) continue;
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
            : robot.id === 'B1'
              ? { x: 0.46, y: 0.6 }
              : { x: 0.68, y: 0.61 };
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
      )
        continue;
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
          robot.role === 'hamster'
            ? robot.id === 'H1'
              ? { x: 0.6, y: 0.34 }
              : { x: 0.1, y: 0.09 }
            : tip(robot.pose, -0.11);
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
  }
  if (world.robots.every((r) => r.phase === 'complete'))
    finish(world, '임무 완료 · 조기 종료 모의');
  return world;
}
