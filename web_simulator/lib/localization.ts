// Display contract shared with robo_control.vision --track. Millimetres, +X zero.
export type MeasuredPose = {
  id: string;
  x: number;
  y: number;
  heading: number;
  ageMs: number | null;
  state: string;
};
export type MeasuredObject = {
  color: string;
  kind: string;
  x: number;
  y: number;
};
export type PositionFrame = {
  sequence: number;
  at: number;
  poses: MeasuredPose[];
  objects: MeasuredObject[];
  rejected: boolean;
  reason: string;
  field: [number, number];
};
export const toSimulationHeading = (angle: number) =>
  ((((angle - Math.PI / 2 + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) %
    (2 * Math.PI)) -
  Math.PI;
const finite = (v: unknown): v is number =>
  typeof v === 'number' && Number.isFinite(v);
const tuple = (v: unknown): v is [number, number] =>
  Array.isArray(v) && v.length === 2 && v.every(finite);

export function parsePositionLog(text: string): PositionFrame[] {
  if (text.length > 10_000_000)
    throw new Error('로그는 10MB 이하로 나눠 주세요.');
  const lines = text.split(/\r?\n/).filter((s) => s.trim());
  if (!lines.length || lines.length > 20000)
    throw new Error('1~20,000프레임 JSONL이 필요합니다.');
  let lastSeq = -1,
    lastAt = -Infinity,
    source: string | undefined;
  return lines.map((line, index) => {
    const r = JSON.parse(line);
    if (!r || typeof r !== 'object')
      throw new Error(`${index + 1}행: 잘못된 레코드`);
    if (
      r.coordinate_system &&
      r.coordinate_system !== 'bottom_left_x_right_y_up_mm'
    )
      throw new Error('지원하지 않는 좌표계');
    const seq = r.sequence ?? r.frame_sequence,
      at = r.captured_at_s;
    if (
      !Number.isInteger(seq) ||
      !finite(at) ||
      typeof r.source_name !== 'string' ||
      !r.source_name
    )
      throw new Error(`${index + 1}행: 프레임 식별자 오류`);
    if (source && source !== r.source_name)
      throw new Error('한 로그에는 한 소스·세션만 넣어 주세요.');
    source = r.source_name;
    const outOfOrder = seq <= lastSeq || at < lastAt;
    lastSeq = Math.max(seq, lastSeq);
    lastAt = Math.max(at, lastAt);
    const field: [number, number] =
      tuple(r.field_size_mm) && r.field_size_mm.every((v: number) => v > 0)
        ? r.field_size_mm
        : [1143, 1181];
    const rejected = r.status !== 'detected' || outOfOrder;
    const raw = Array.isArray(r.tracks) ? r.tracks : (r.robots ?? []);
    if (!Array.isArray(raw) || raw.length > 1000)
      throw new Error('잘못된 로봇 목록');
    const ids = new Set<string>();
    const poses: MeasuredPose[] = [];
    for (const p of raw) {
      if (typeof p.robot_id !== 'string' || ids.has(p.robot_id))
        throw new Error('중복·잘못된 로봇 ID');
      ids.add(p.robot_id);
      // A missing track may intentionally contain null, never convert null to zero.
      if (p.robot_center_mm == null) continue;
      if (!tuple(p.robot_center_mm) || !finite(p.heading_rad))
        throw new Error('유효하지 않은 위치·방향');
      const [x, y] = p.robot_center_mm;
      if (x < 0 || y < 0 || x > field[0] || y > field[1])
        throw new Error('경기장 밖 위치');
      const age = p.age_ms ?? r.host_age_ms ?? null;
      if (age !== null && (!finite(age) || age < 0))
        throw new Error('잘못된 관측 나이');
      poses.push({
        id: p.robot_id,
        x,
        y,
        heading: p.heading_rad,
        ageMs: age,
        state: rejected ? 'rejected' : (p.state ?? 'observed'),
      });
    }
    const objects: MeasuredObject[] = [];
    for (const p of r.objects ?? []) {
      if (
        !tuple(p.center_mm) ||
        typeof p.color !== 'string' ||
        typeof p.kind !== 'string'
      )
        throw new Error('잘못된 색 물체 관측');
      const [x, y] = p.center_mm;
      if (x < 0 || y < 0 || x > field[0] || y > field[1])
        throw new Error('경기장 밖 물체');
      objects.push({ color: p.color, kind: p.kind, x, y });
    }
    return {
      sequence: seq,
      at,
      field,
      poses,
      objects: rejected ? [] : objects,
      rejected,
      reason: r.reason ?? (outOfOrder ? '역순 프레임' : ''),
    };
  });
}
