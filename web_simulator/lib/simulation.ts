export const FIELD = {
  width: 1.143,
  height: 1.181,
  duration: 120,
  startZone: { x: 0, y: 0.901, width: 0.48, height: 0.28 },
  safetyMargin: 0.015,
} as const;

export const ROBOT = {
  width: 0.126,
  length: 0.1,
  height: 0.1,
  displayRadius: 0.081,
  maxLinearSpeed: 0.18,
  maxAngularSpeed: 2.4,
  mecanumWheelCenterX: 0.057,
  mecanumWheelCenterY: 0.032,
  differentialHalfTrack: 0.058,
} as const;

export type Point = { x: number; y: number };
export type Pose = Point & { heading: number };
export type DriveMode = 'differential' | 'mecanum';
export type RobotStatus =
  | 'waiting'
  | 'moving'
  | 'manual'
  | 'complete'
  | 'blocked'
  | 'timeout';

export type WheelSpeeds = {
  fl: number;
  fr: number;
  rl: number;
  rr: number;
};

export type RobotState = {
  id: string;
  color: string;
  pose: Pose;
  waypoint: number;
  status: RobotStatus;
  delay: number;
  trail: Point[];
  wheels: WheelSpeeds;
};

export type SimEvent = {
  id: number;
  time: number;
  level: 'info' | 'success' | 'warning';
  text: string;
};

export type StepResult = {
  robots: RobotState[];
  events: Omit<SimEvent, 'id'>[];
};

export const ROBOT_COLORS = [
  '#5eead4',
  '#60a5fa',
  '#c084fc',
  '#fbbf24',
  '#fb7185',
  '#a3e635',
] as const;

// SCAD 명목치로 계산한 운용 외곽의 보수적 볼록껍질:
// 본체 100×100 mm + 좌우 바퀴 미리보기의 돌출 범위.
// local x = 오른쪽, local y = 전진 방향. 단위는 m.
export const FOOTPRINT: Point[] = [
  { x: -0.05, y: -0.05 },
  { x: 0.05, y: -0.05 },
  { x: 0.063, y: -0.0375 },
  { x: 0.063, y: 0.0075 },
  { x: 0.05, y: 0.05 },
  { x: -0.05, y: 0.05 },
  { x: -0.063, y: 0.0075 },
  { x: -0.063, y: -0.0375 },
];

// 개조안의 앞뒤 메카넘 휠은 SCAD의 단일 차축보다 앞쪽으로 돌출된다.
// 100mm 본체와 12×25mm 휠 4개의 외곽을 모두 포함한다.
export const MECANUM_FOOTPRINT: Point[] = [
  { x: -0.05, y: -0.05 },
  { x: 0.05, y: -0.05 },
  { x: 0.063, y: -0.0445 },
  { x: 0.063, y: 0.0445 },
  { x: 0.05, y: 0.05 },
  { x: -0.05, y: 0.05 },
  { x: -0.063, y: 0.0445 },
  { x: -0.063, y: -0.0445 },
];

export function footprintForMode(mode: DriveMode): Point[] {
  return mode === 'mecanum' ? MECANUM_FOOTPRINT : FOOTPRINT;
}

export const OBSTACLES = [
  { x: 0.5, y: 0.4, width: 0.14, height: 0.3, label: '임시 A' },
  { x: 0.22, y: 0.5, width: 0.1, height: 0.12, label: '임시 B' },
  { x: 0.84, y: 0.48, width: 0.1, height: 0.12, label: '임시 C' },
] as const;

export const STARTS: Pose[] = [
  { x: 0.094, y: 0.971, heading: Math.PI },
  { x: 0.24, y: 0.971, heading: Math.PI },
  { x: 0.386, y: 0.971, heading: Math.PI },
  { x: 0.094, y: 1.111, heading: Math.PI },
  { x: 0.24, y: 1.111, heading: Math.PI },
  { x: 0.386, y: 1.111, heading: Math.PI },
];

// 공식 고정 배치도는 아직 배포 전이므로, 아래 좌표는 알고리즘 확인용이다.
export const ROUTES: Point[][] = [
  [
    { x: 0.094, y: 0.76 },
    { x: 0.1, y: 0.18 },
  ],
  [
    { x: 0.24, y: 0.75 },
    { x: 0.12, y: 0.69 },
    { x: 0.12, y: 0.39 },
    { x: 0.3, y: 0.27 },
    { x: 0.3, y: 0.18 },
  ],
  [
    { x: 0.418, y: 0.76 },
    { x: 0.418, y: 0.34 },
    { x: 0.49, y: 0.23 },
    { x: 0.49, y: 0.14 },
  ],
  [
    { x: 0.094, y: 0.8 },
    { x: 0.755, y: 0.8 },
    { x: 0.755, y: 0.32 },
    { x: 0.68, y: 0.18 },
  ],
  [
    { x: 0.24, y: 0.84 },
    { x: 0.74, y: 0.84 },
    { x: 0.74, y: 0.35 },
    { x: 0.856, y: 0.27 },
    { x: 0.856, y: 0.18 },
  ],
  [
    { x: 0.386, y: 0.88 },
    { x: 1.04, y: 0.88 },
    { x: 1.04, y: 0.31 },
  ],
];

// 3×2 밀집 출발에서는 임의 회전/동시 이탈이 위험하므로 두 번째 줄은 순차 출발한다.
const DEPARTURE_DELAYS = [0, 0.7, 1.4, 4.8, 14, 24];

export function createInitialRobots(): RobotState[] {
  return STARTS.map((pose, index) => ({
    id: `R${index + 1}`,
    color: ROBOT_COLORS[index],
    pose: { ...pose },
    waypoint: 0,
    status: 'waiting',
    delay: DEPARTURE_DELAYS[index],
    trail: [{ x: pose.x, y: pose.y }],
    wheels: zeroWheels(),
  }));
}

export function zeroWheels(): WheelSpeeds {
  return { fl: 0, fr: 0, rl: 0, rr: 0 };
}

export function normalizeAngle(value: number): number {
  let angle = value;
  while (angle > Math.PI) angle -= Math.PI * 2;
  while (angle < -Math.PI) angle += Math.PI * 2;
  return angle;
}

export function transformFootprint(pose: Pose, mode: DriveMode = 'differential'): Point[] {
  const c = Math.cos(pose.heading);
  const s = Math.sin(pose.heading);
  return footprintForMode(mode).map((point) => ({
    x: pose.x + point.x * c - point.y * s,
    y: pose.y + point.x * s + point.y * c,
  }));
}

export function rectanglePolygon(rect: {
  x: number;
  y: number;
  width: number;
  height: number;
}): Point[] {
  return [
    { x: rect.x, y: rect.y },
    { x: rect.x + rect.width, y: rect.y },
    { x: rect.x + rect.width, y: rect.y + rect.height },
    { x: rect.x, y: rect.y + rect.height },
  ];
}

function projection(poly: Point[], axis: Point) {
  let min = Infinity;
  let max = -Infinity;
  for (const point of poly) {
    const value = point.x * axis.x + point.y * axis.y;
    min = Math.min(min, value);
    max = Math.max(max, value);
  }
  return { min, max };
}

export function polygonsOverlap(a: Point[], b: Point[]): boolean {
  for (const poly of [a, b]) {
    for (let index = 0; index < poly.length; index += 1) {
      const p1 = poly[index];
      const p2 = poly[(index + 1) % poly.length];
      const edge = { x: p2.x - p1.x, y: p2.y - p1.y };
      const axis = { x: -edge.y, y: edge.x };
      const aProjection = projection(a, axis);
      const bProjection = projection(b, axis);
      if (
        aProjection.max < bProjection.min ||
        bProjection.max < aProjection.min
      ) {
        return false;
      }
    }
  }
  return true;
}

function pointSegmentDistance(point: Point, a: Point, b: Point): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const lengthSquared = dx * dx + dy * dy;
  if (lengthSquared === 0) return Math.hypot(point.x - a.x, point.y - a.y);
  const t = Math.max(
    0,
    Math.min(1, ((point.x - a.x) * dx + (point.y - a.y) * dy) / lengthSquared),
  );
  return Math.hypot(point.x - (a.x + t * dx), point.y - (a.y + t * dy));
}

export function polygonDistance(a: Point[], b: Point[]): number {
  if (polygonsOverlap(a, b)) return 0;
  let distance = Infinity;
  for (const [poly, other] of [
    [a, b],
    [b, a],
  ] as const) {
    for (const point of poly) {
      for (let index = 0; index < other.length; index += 1) {
        distance = Math.min(
          distance,
          pointSegmentDistance(
            point,
            other[index],
            other[(index + 1) % other.length],
          ),
        );
      }
    }
  }
  return distance;
}

function footprintInsideField(poly: Point[]): boolean {
  return poly.every(
    (point) =>
      point.x >= FIELD.safetyMargin &&
      point.x <= FIELD.width - FIELD.safetyMargin &&
      point.y >= FIELD.safetyMargin &&
      point.y <= FIELD.height - FIELD.safetyMargin,
  );
}

export function minimumClearance(robots: RobotState[], mode: DriveMode = 'differential'): number {
  let minimum = Infinity;
  const footprints = robots.map((robot) => transformFootprint(robot.pose, mode));
  for (let index = 0; index < footprints.length; index += 1) {
    const footprint = footprints[index];
    for (const point of footprint) {
      minimum = Math.min(
        minimum,
        point.x,
        FIELD.width - point.x,
        point.y,
        FIELD.height - point.y,
      );
    }
    for (const obstacle of OBSTACLES) {
      minimum = Math.min(
        minimum,
        polygonDistance(footprint, rectanglePolygon(obstacle)),
      );
    }
    for (let other = index + 1; other < footprints.length; other += 1) {
      minimum = Math.min(minimum, polygonDistance(footprint, footprints[other]));
    }
  }
  return Number.isFinite(minimum) ? minimum : 0;
}

function collisionReason(
  candidate: RobotState,
  allRobots: RobotState[],
  mode: DriveMode,
): string | null {
  const footprint = transformFootprint(candidate.pose, mode);
  if (!footprintInsideField(footprint)) return '경기장 안전 경계';
  for (const obstacle of OBSTACLES) {
    if (
      polygonDistance(footprint, rectanglePolygon(obstacle)) <
      FIELD.safetyMargin
    ) {
      return `${obstacle.label} 장애물 15mm 안전여유`;
    }
  }
  for (const other of allRobots) {
    if (other.id === candidate.id) continue;
    if (
      polygonDistance(footprint, transformFootprint(other.pose, mode)) <
      FIELD.safetyMargin
    ) {
      return `${other.id} 15mm 안전여유`;
    }
  }
  return null;
}

export function mixMecanumCommand(
  vForward: number,
  vLeft: number,
  omega: number,
) {
  const k = ROBOT.mecanumWheelCenterX + ROBOT.mecanumWheelCenterY;
  const raw = {
    fl: vForward - vLeft - k * omega,
    fr: vForward + vLeft + k * omega,
    rl: vForward + vLeft - k * omega,
    rr: vForward - vLeft + k * omega,
  };
  const rawPeak = Math.max(
    Math.abs(raw.fl),
    Math.abs(raw.fr),
    Math.abs(raw.rl),
    Math.abs(raw.rr),
  );
  const scale =
    rawPeak > ROBOT.maxLinearSpeed
      ? ROBOT.maxLinearSpeed / rawPeak
      : 1;
  return {
    scale,
    wheels: {
      fl: (raw.fl * scale) / ROBOT.maxLinearSpeed,
      fr: (raw.fr * scale) / ROBOT.maxLinearSpeed,
      rl: (raw.rl * scale) / ROBOT.maxLinearSpeed,
      rr: (raw.rr * scale) / ROBOT.maxLinearSpeed,
    },
  };
}

export function mixMecanumWheels(
  vForward: number,
  vLeft: number,
  omega: number,
): WheelSpeeds {
  return mixMecanumCommand(vForward, vLeft, omega).wheels;
}

function mixDifferentialCommand(vForward: number, omega: number) {
  const halfTrack = ROBOT.differentialHalfTrack;
  const left = vForward - halfTrack * omega;
  const right = vForward + halfTrack * omega;
  const rawPeak = Math.max(Math.abs(left), Math.abs(right));
  const scale =
    rawPeak > ROBOT.maxLinearSpeed
      ? ROBOT.maxLinearSpeed / rawPeak
      : 1;
  return {
    scale,
    wheels: {
      fl: (left * scale) / ROBOT.maxLinearSpeed,
      rl: (left * scale) / ROBOT.maxLinearSpeed,
      fr: (right * scale) / ROBOT.maxLinearSpeed,
      rr: (right * scale) / ROBOT.maxLinearSpeed,
    },
  };
}

function commandToPose(
  pose: Pose,
  vForward: number,
  vLeft: number,
  omega: number,
  dt: number,
): Pose {
  const s = Math.sin(pose.heading);
  const c = Math.cos(pose.heading);
  const worldX = -vForward * s - vLeft * c;
  const worldY = vForward * c - vLeft * s;
  return {
    x: pose.x + worldX * dt,
    y: pose.y + worldY * dt,
    heading: normalizeAngle(pose.heading + omega * dt),
  };
}

function keyPressed(keys: Set<string>, ...names: string[]) {
  return names.some((name) => keys.has(name));
}

export function stepWorld(
  current: RobotState[],
  dt: number,
  elapsed: number,
  driveMode: DriveMode,
  manual: boolean,
  selectedId: string,
  keys: Set<string>,
): StepResult {
  if (elapsed >= FIELD.duration) {
    return {
      robots: current.map((robot) => ({
        ...robot,
        wheels: zeroWheels(),
        status:
          robot.status === 'complete' || robot.status === 'blocked'
            ? robot.status
            : 'timeout',
      })),
      events: [],
    };
  }
  const events: Omit<SimEvent, 'id'>[] = [];
  const next = current.map((robot) => ({
    ...robot,
    pose: { ...robot.pose },
    trail: [...robot.trail],
    wheels: { ...robot.wheels },
  }));

  for (let index = 0; index < next.length; index += 1) {
    const robot = next[index];
    if (robot.status === 'complete' || robot.status === 'blocked') continue;

    let vForward = 0;
    let vLeft = 0;
    let omega = 0;
    let status: RobotStatus = robot.status;

    if (manual) {
      if (robot.id !== selectedId) {
        robot.wheels = zeroWheels();
        continue;
      }
      const forward = Number(keyPressed(keys, 'w', 'arrowup'));
      const reverse = Number(keyPressed(keys, 's', 'arrowdown'));
      const left = Number(keyPressed(keys, 'a', 'arrowleft'));
      const right = Number(keyPressed(keys, 'd', 'arrowright'));
      const ccw = Number(keyPressed(keys, 'q'));
      const cw = Number(keyPressed(keys, 'e'));
      vForward = (forward - reverse) * ROBOT.maxLinearSpeed;
      vLeft =
        driveMode === 'mecanum'
          ? (left - right) * ROBOT.maxLinearSpeed
          : 0;
      omega = (ccw - cw) * ROBOT.maxAngularSpeed;
      status = 'manual';
    } else {
      if (elapsed < robot.delay) {
        robot.status = 'waiting';
        robot.wheels = zeroWheels();
        continue;
      }
      let target = ROUTES[index][robot.waypoint];
      if (!target) {
        robot.status = 'complete';
        robot.wheels = zeroWheels();
        continue;
      }
      let dx = target.x - robot.pose.x;
      let dy = target.y - robot.pose.y;
      let distance = Math.hypot(dx, dy);
      if (distance < 0.018) {
        robot.waypoint += 1;
        target = ROUTES[index][robot.waypoint];
        if (!target) {
          robot.status = 'complete';
          robot.wheels = zeroWheels();
          events.push({
            time: elapsed,
            level: 'success',
            text: `${robot.id} 목표 도착`,
          });
          continue;
        }
        dx = target.x - robot.pose.x;
        dy = target.y - robot.pose.y;
        distance = Math.hypot(dx, dy);
      }
      status = 'moving';
      if (driveMode === 'mecanum') {
        const worldVx = (dx / distance) * ROBOT.maxLinearSpeed;
        const worldVy = (dy / distance) * ROBOT.maxLinearSpeed;
        const s = Math.sin(robot.pose.heading);
        const c = Math.cos(robot.pose.heading);
        vForward = -worldVx * s + worldVy * c;
        vLeft = -worldVx * c - worldVy * s;
      } else {
        const targetHeading = Math.atan2(-dx, dy);
        const error = normalizeAngle(targetHeading - robot.pose.heading);
        omega = Math.max(
          -ROBOT.maxAngularSpeed,
          Math.min(ROBOT.maxAngularSpeed, error * 3.2),
        );
        vForward =
          Math.abs(error) < 0.62
            ? ROBOT.maxLinearSpeed * Math.max(0.22, Math.cos(error))
            : 0;
      }
    }

    const mixed =
      driveMode === 'mecanum'
        ? mixMecanumCommand(vForward, vLeft, omega)
        : mixDifferentialCommand(vForward, omega);
    robot.wheels = mixed.wheels;
    vForward *= mixed.scale;
    vLeft *= mixed.scale;
    omega *= mixed.scale;
    if (vForward === 0 && vLeft === 0 && omega === 0) {
      robot.status = status;
      continue;
    }

    const candidate: RobotState = {
      ...robot,
      pose: commandToPose(robot.pose, vForward, vLeft, omega, dt),
      status,
    };
    const others = next.map((item, otherIndex) =>
      otherIndex === index ? current[index] : item,
    );
    const reason = collisionReason(candidate, others, driveMode);
    if (reason) {
      robot.status = 'blocked';
      robot.wheels = zeroWheels();
      events.push({
        time: elapsed,
        level: 'warning',
        text: `${robot.id} 안전정지 · ${reason}`,
      });
      continue;
    }

    robot.pose = candidate.pose;
    robot.status = candidate.status;
    const lastTrail = robot.trail[robot.trail.length - 1];
    if (
      !lastTrail ||
      Math.hypot(robot.pose.x - lastTrail.x, robot.pose.y - lastTrail.y) > 0.012
    ) {
      robot.trail.push({ x: robot.pose.x, y: robot.pose.y });
      if (robot.trail.length > 260) robot.trail.shift();
    }
  }

  return { robots: next, events };
}
