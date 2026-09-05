'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import {
  Play,
  Pause,
  RotateCcw,
  Radio,
  Camera,
  CheckCircle2,
  AlertTriangle,
  ChevronRight,
  ExternalLink,
} from 'lucide-react';
import {
  advance,
  createWorld,
  finish,
  scoreWorld,
  clearance,
  FIELD,
  OFFICIAL_LAYOUT,
  LAB_SLOTS,
  ZONES,
  SPEC,
  PHASE_LABEL,
  COLOR_HEX,
  type World,
  type Item,
  type Robot,
  type ObservationMode,
} from '@/lib/mission';
import ArenaSimulator from './arena-simulator';
import PositionMonitor from './position-monitor';
import SimulationLab from './simulation-lab';
import { createExperiment } from '@/lib/experiments';
import AerialPanel from './aerial-panel';
import { footprint, sceneOccluders, REFERENCE_MARKERS } from '@/lib/aerial';
import './mission.css';

const px = (x: number) => x * 1000;
const py = (y: number) => (FIELD.height - y) * 1000;
const labelItem = (item: Item) =>
  item.kind === 'disc'
    ? '샘플 디스크'
    : item.kind === 'cube'
      ? '의료키트 큐브'
      : `${{ red: '빨강', yellow: '노랑', green: '초록' }[item.color!]} 원기둥`;
const destinationOf = (world: World, id: string) =>
  world.robots.flatMap((r) => r.jobs).find((j) => j.itemId === id)
    ?.destination ?? '운반 제외';

function FieldView({
  world,
  selectedId,
  setSelected,
  routes,
  labels,
}: {
  world: World;
  selectedId: string;
  setSelected: (id: string) => void;
  routes: boolean;
  labels: boolean;
}) {
  const zone = (name: string, r: typeof ZONES.H, fill: string) => (
    <g key={name}>
      <rect
        x={px(r.x)}
        y={py(r.y + r.height)}
        width={px(r.width)}
        height={px(r.height)}
        fill={fill}
      />
      <text
        x={px(r.x + r.width / 2)}
        y={py(r.y + r.height) + 29}
        className="map-zone"
      >
        {name}
      </text>
    </g>
  );
  const itemGlyph = (item: Item) => (
    <g
      key={item.id}
      transform={`translate(${px(item.x)} ${py(item.y)})`}
      opacity={!item.selected ? 0.65 : 1}
    >
      <title>
        {`${item.id} · ${labelItem(item)} · ${
          item.carrier
            ? `${item.carrier} 운반 중`
            : item.released
              ? '내려놓음'
              : '초기 위치'
        } → ${destinationOf(world, item.id)}`}
      </title>
      {item.kind === 'cube' ? (
        <g>
          <rect
            x="-12.5"
            y="-12.5"
            width="25"
            height="25"
            rx="2"
            fill="#f1f5f9"
            stroke="#475569"
            strokeWidth="2"
          />
          <path d="M-7 0H7M0-7V7" stroke="#dc2626" strokeWidth="5" />
        </g>
      ) : item.kind === 'disc' ? (
        <g>
          <circle
            r="28"
            fill="#334155"
            stroke={item.released ? '#14b8a6' : '#0f172a'}
            strokeWidth="3"
          />
          <circle r="18" fill="none" stroke="#94a3b8" strokeWidth="1.5" />
          <text y="5" className="disc-id">
            {item.id}
          </text>
        </g>
      ) : (
        <circle
          r="10"
          fill={COLOR_HEX[item.color!]}
          stroke="#334155"
          strokeWidth="2"
        />
      )}
      {labels && item.kind !== 'disc' && !item.carrier && (
        <text y="-18" className="map-item">
          {item.id}
        </text>
      )}
    </g>
  );
  const robotGlyph = (robot: Robot) => (
    <g
      key={robot.id}
      onClick={() => setSelected(robot.id)}
      className="map-robot"
    >
      <title>{`${robot.name} · ${PHASE_LABEL[robot.phase]}`}</title>
      <g
        transform={`translate(${px(robot.pose.x)} ${py(robot.pose.y)}) rotate(${(-robot.pose.heading * 180) / Math.PI})`}
      >
        {selectedId === robot.id && (
          <rect
            x="-76"
            y="-64"
            width="152"
            height="128"
            rx="15"
            fill="none"
            stroke={robot.color}
            strokeDasharray="6 5"
            strokeWidth="2"
          />
        )}
        <rect
          x="-50"
          y="-50"
          width="100"
          height="100"
          rx={robot.role === 'hamster' ? 15 : 7}
          fill="#102334"
          stroke={robot.color}
          strokeWidth="4"
        />
        {[-57, 57].flatMap((x) =>
          [-32, 32].map((y) => (
            <rect
              key={`${x}-${y}`}
              x={x - 6}
              y={y - 13}
              width="12"
              height="26"
              rx="2"
              fill="#1e293b"
              stroke="#94a3b8"
              strokeWidth="2"
            />
          )),
        )}
        <path
          d={`M-12 -42V${-robot.arm * 1000}M12 -42V${-robot.arm * 1000}`}
          fill="none"
          stroke={robot.color}
          strokeWidth="5"
        />
        {robot.role === 'hamster' ? (
          <line
            x1="-17"
            x2="17"
            y1={-robot.arm * 1000}
            y2={-robot.arm * 1000}
            stroke={robot.sensor ? '#34d399' : robot.color}
            strokeWidth={robot.servo > 0.5 ? 7 : 2}
            strokeDasharray={robot.servo > 0.5 ? undefined : '3 3'}
          />
        ) : (
          <path
            d={`M-12 ${-robot.arm * 1000}l${robot.servo * 10} -8M12 ${-robot.arm * 1000}l${-robot.servo * 10} -8`}
            fill="none"
            stroke={robot.color}
            strokeWidth="5"
          />
        )}
        <path
          d="M-8 -20L0-29L8-20"
          stroke="white"
          strokeWidth="2"
          fill="none"
        />
      </g>
      <text
        x={px(robot.pose.x)}
        y={py(robot.pose.y) + 14}
        className="robot-map-id"
      >
        {robot.id}
      </text>
      <text
        x={px(robot.pose.x)}
        y={py(robot.pose.y) + 72}
        className="robot-map-label"
      >
        {robot.blockedBy ? '양보 대기' : PHASE_LABEL[robot.phase]}
      </text>
    </g>
  );
  return (
    <svg
      className="mission-map"
      viewBox="-62 -65 1267 1336"
      aria-label={`햄스터 1대와 비버 ${world.robots.length - 1}대가 운반하는 연습 경기장`}
    >
      <title>햄스터·비버·박쥐 예선 경기장</title>
      <defs>
        <clipPath id="aerial-field">
          <rect width="1143" height="1181" />
        </clipPath>
        <pattern
          id="mission-grid"
          width="100"
          height="100"
          patternUnits="userSpaceOnUse"
        >
          <path
            d="M100 0H0V100"
            fill="none"
            stroke="#64748b"
            strokeOpacity=".14"
            strokeWidth="1"
          />
        </pattern>
      </defs>
      <rect
        width="1143"
        height="1181"
        fill="#e9ece7"
        stroke="#a88454"
        strokeWidth="10"
      />
      {zone('PCC-L', ZONES['PCC-L'], '#dbeafe')}
      {zone('병원 H', ZONES.H, '#fee2e2')}
      {zone('PCC-R', ZONES['PCC-R'], '#dbeafe')}
      {zone('격리구역', OFFICIAL_LAYOUT.quarantine, '#ede9fe')}
      {zone('START · 480 × 280', FIELD.startZone, '#d1fae5')}
      <rect
        x={px(ZONES.RZ.x)}
        y={py(ZONES.RZ.y + ZONES.RZ.height)}
        width={px(ZONES.RZ.width)}
        height={px(ZONES.RZ.height)}
        fill="none"
        stroke="#059669"
        strokeWidth="2"
        strokeDasharray="8 6"
      />
      <text x="890" y="1148" className="map-zone" fill="#047857">
        RZ · 회복
      </text>
      <rect width="1143" height="1181" fill="url(#mission-grid)" />
      {[
        { x: 0, y: 0.981, width: 1.143, height: 0.02 },
        { x: 0.3, y: 1.001, width: 0.02, height: 0.18 },
        { x: 0.823, y: 1.001, width: 0.02, height: 0.18 },
        ...OFFICIAL_LAYOUT.centerTape,
        { x: 0, y: 0.28, width: 0.3, height: 0.02 },
        { x: 0.28, y: 0, width: 0.02, height: 0.3 },
        { x: 0.643, y: 0, width: 0.02, height: 0.3 },
        { x: 0.643, y: 0.28, width: 0.5, height: 0.02 },
      ].map((r, i) => (
        <rect
          key={i}
          x={px(r.x)}
          y={py(r.y + r.height)}
          width={px(r.width)}
          height={px(r.height)}
          fill="#24282c"
        />
      ))}
      <text x="610" y="567" className="map-detail">
        통과 가능
      </text>
      {LAB_SLOTS.map((p, i) => (
        <g key={i}>
          <circle
            cx={px(p.x)}
            cy={py(p.y)}
            r="30"
            fill="#fff"
            stroke="#64748b"
            strokeWidth="2"
          />
          <text x={px(p.x)} y={py(p.y) + 53} className="map-item">
            LAB {i + 1}
          </text>
        </g>
      ))}
      <text x="480" y="960" className="map-zone">
        LAB · Ø60 mm
      </text>
      {world.drone.enabled &&
        world.drone.altitude > 0.1 &&
        (() => {
          const view = footprint({
            ...world.drone.pose,
            z: world.drone.altitude,
          });
          return (
            <g clipPath="url(#aerial-field)" pointerEvents="none">
              <rect
                x={px(view.x)}
                y={py(view.y + view.height)}
                width={px(view.width)}
                height={px(view.height)}
                fill="#0891b2"
                fillOpacity=".08"
                stroke={world.drone.calibrationValid ? '#0891b2' : '#d97706'}
                strokeWidth="3"
                strokeDasharray="14 7"
              />
              <polyline
                points={world.drone.trail
                  .map((p) => `${px(p.x)},${py(p.y)}`)
                  .join(' ')}
                stroke="#0891b2"
                strokeWidth="3"
                fill="none"
                strokeDasharray="4 8"
              />
              {REFERENCE_MARKERS.map((p, i) => (
                <rect
                  key={i}
                  x={px(p.x) - 7}
                  y={py(p.y) - 7}
                  width="14"
                  height="14"
                  fill={
                    world.drone.anchorIds.includes(`F${i + 1}`)
                      ? '#0891b2'
                      : '#64748b'
                  }
                >
                  <title>{`가상 보정 기준점 F${i + 1}`}</title>
                </rect>
              ))}
              {world.robots
                .filter((r) => world.drone.visibleRobots.includes(r.id))
                .map((r) => (
                  <line
                    key={r.id}
                    x1={px(world.drone.pose.x)}
                    y1={py(world.drone.pose.y)}
                    x2={px(r.pose.x)}
                    y2={py(r.pose.y)}
                    stroke={
                      world.drone.recoveredIds.includes(r.id)
                        ? '#059669'
                        : '#0891b2'
                    }
                    strokeOpacity=".5"
                    strokeWidth={
                      world.drone.recoveredIds.includes(r.id) ? 5 : 2
                    }
                    strokeDasharray="6 6"
                  />
                ))}
            </g>
          );
        })()}
      {routes &&
        world.robots.map((r) => (
          <polyline
            key={r.id}
            points={[r.pose, ...r.path]
              .map((p) => `${px(p.x)},${py(p.y)}`)
              .join(' ')}
            fill="none"
            stroke={r.color}
            strokeWidth="4"
            strokeDasharray="8 7"
            opacity=".7"
          />
        ))}
      {world.robots.map((r) => (
        <polyline
          key={r.id}
          points={r.trail.map((p) => `${px(p.x)},${py(p.y)}`).join(' ')}
          fill="none"
          stroke={r.color}
          strokeWidth="2"
          opacity=".2"
        />
      ))}
      {world.items.filter((i) => !i.carrier).map(itemGlyph)}
      {world.robots.map(robotGlyph)}
      {world.items
        .filter((i) => i.carrier)
        .map((item, i) =>
          item.kind === 'cube' &&
          !['align-drop', 'release', 'verify-release'].includes(
            world.robots.find((r) => r.id === item.carrier)!.phase,
          )
            ? itemGlyph({
                ...item,
                x: item.x + (i % 2 ? 0.018 : -0.018),
                y: item.y - 0.02,
              })
            : itemGlyph(item),
        )}
      {sceneOccluders(world).map((b) => (
        <g key={b.id} pointerEvents="none">
          <rect
            x={px(b.min.x)}
            y={py(b.max.y)}
            width={px(b.max.x - b.min.x)}
            height={px(b.max.y - b.min.y)}
            fill="#f59e0b"
            fillOpacity=".15"
            stroke="#b45309"
            strokeWidth="3"
            strokeDasharray="9 6"
          />
          <text
            x={px((b.min.x + b.max.x) / 2)}
            y={py(b.max.y) - 10}
            className="map-item"
          >
            태그 위 가림막 · 시험용
          </text>
        </g>
      ))}
      {world.drone.enabled && (
        <g
          transform={`translate(${px(world.drone.pose.x)} ${py(world.drone.pose.y)})`}
          opacity={world.drone.altitude > 0.1 ? 0.65 : 1}
        >
          <title>{`박쥐 · ${world.drone.altitude.toFixed(2)}m · 관측 모의`}</title>
          <rect
            x="-75"
            y="-75"
            width="150"
            height="150"
            fill="none"
            stroke="#0e7490"
            strokeWidth="2"
            strokeDasharray="8 5"
          />
          {[-46, 46].flatMap((x) =>
            [-46, 46].map((y) => (
              <g key={`${x}${y}`}>
                <line
                  x1="0"
                  y1="0"
                  x2={x}
                  y2={y}
                  stroke="#155e75"
                  strokeWidth="5"
                />
                <circle
                  cx={x}
                  cy={y}
                  r="22"
                  stroke="#0e7490"
                  strokeWidth="3"
                  fill="#cffafe"
                  fillOpacity=".65"
                />
              </g>
            )),
          )}
          <rect x="-20" y="-24" width="40" height="48" rx="8" fill="#164e63" />
          <text y="8" className="robot-map-id">
            BAT
          </text>
          <text y="100" className="map-item">
            {world.drone.strategy === 'active'
              ? '박쥐 · 시야 탐색'
              : '박쥐 · 중앙 관측'}
          </text>
        </g>
      )}
      <text x="571.5" y="-25" className="map-dimension">
        1,143 mm
      </text>
      <text
        transform="translate(-28 590) rotate(-90)"
        className="map-dimension"
      >
        1,181 mm
      </text>
    </svg>
  );
}

export default function MissionSimulator() {
  const [world, setWorld] = useState<World>(() => createWorld('drone'));
  const worldRef = useRef<World>(world);
  const [running, setRunning] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [selectedId, setSelectedId] = useState('H1');
  const [routes, setRoutes] = useState(true);
  const [labels, setLabels] = useState(true);
  const [legacy, setLegacy] = useState(false);
  const runningRef = useRef(false);
  const publish = useCallback(() => {
    const w = worldRef.current;
    setWorld({
      ...w,
      robots: [...w.robots],
      items: [...w.items],
      logs: [...w.logs],
      observer: { ...w.observer },
      drone: { ...w.drone },
    });
    if (w.ended) setRunning(false);
  }, []);
  const reset = useCallback(
    (mode?: ObservationMode) => {
      const observation = mode ?? worldRef.current.observer.mode;
      worldRef.current = createExperiment(
        observation,
        worldRef.current.robots.length === 5 && observation !== 'drone' ? 5 : 4,
      );
      setRunning(false);
      publish();
    },
    [publish],
  );
  useEffect(() => {
    runningRef.current = running;
  }, [running]);
  useEffect(() => {
    if (!running || legacy) return;
    let frame = 0,
      last = performance.now(),
      bank = 0;
    const tick = (now: number) => {
      bank += Math.min(0.15, (now - last) / 1000) * speed;
      last = now;
      const count = Math.min(40, Math.floor(bank / SPEC.dt));
      for (let i = 0; i < count; i++) advance(worldRef.current);
      bank -= count * SPEC.dt;
      if (count) publish();
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
  }, [running, speed, publish, legacy]);
  useEffect(() => {
    type Context = {
      registerTool: (
        tool: {
          name: string;
          description: string;
          inputSchema: Record<string, unknown>;
          annotations?: Record<string, boolean>;
          execute: () => unknown;
        },
        options: { signal: AbortSignal },
      ) => void | Promise<void>;
    };
    if (legacy) return;
    const context = (document as Document & { modelContext?: Context })
      .modelContext;
    if (!context) return;
    const controller = new AbortController();
    const tools = [
      {
        name: 'read_simulation_state',
        description:
          'Read the configured ground fleet, synthetic observations, carried items, final-state score, and servo/sensor state.',
        annotations: { readOnlyHint: true },
        execute: () => ({
          world: worldRef.current,
          score: scoreWorld(worldRef.current.items),
          running: runningRef.current,
        }),
      },
      {
        name: 'start_route_replay',
        description:
          'Start the autonomous pickup, transport and release mission in the visible simulator.',
        execute: () => {
          setRunning(true);
          return { running: true };
        },
      },
      {
        name: 'pause_simulation',
        description: 'Pause the mission simulation.',
        execute: () => {
          setRunning(false);
          return { running: false };
        },
      },
      {
        name: 'reset_simulation',
        description:
          'Reset the qualifier practice scenario, configured fleet and emergency-stop latch.',
        execute: () => {
          reset();
          return { reset: true };
        },
      },
    ];
    tools.forEach((tool) => {
      try {
        void Promise.resolve(
          context.registerTool(
            {
              ...tool,
              inputSchema: {
                type: 'object',
                properties: {},
                additionalProperties: false,
              },
            },
            { signal: controller.signal },
          ),
        ).catch(() => undefined);
      } catch {
        /* Optional browser capability. */
      }
    });
    return () => controller.abort();
  }, [reset, legacy]);
  const score = scoreWorld(world.items),
    selected = world.robots.find((r) => r.id === selectedId)!;
  const job = selected.jobs[selected.jobIndex];
  const time = Math.max(0, 120 - world.elapsed),
    green = score.points === 160;
  return (
    <div className="mission-root">
      <div className="mission-topnav">
        <span>ROBO / RESCUE LAB</span>
        <div>
          <button
            onClick={() => {
              setLegacy(false);
            }}
            aria-pressed={!legacy}
          >
            예선 임무
          </button>
          <button
            onClick={() => {
              setRunning(false);
              setLegacy(true);
            }}
            aria-pressed={legacy}
          >
            기존 구동 시험
          </button>
          <a
            href="https://github.com/galaxygamja/Robo"
            target="_blank"
            rel="noreferrer"
          >
            코드 <ExternalLink size={14} />
          </a>
        </div>
      </div>
      {legacy ? (
        <ArenaSimulator />
      ) : (
        <main className="qualifier-shell">
          <header className="qualifier-header">
            <div>
              <p className="mission-kicker">SENIOR 예선 · 120초 점수제</p>
              <h1>
                햄스터 × 1 · 비버 × {world.robots.length - 1} ·{' '}
                {world.observer.mode === 'localization'
                  ? '좌표 추적'
                  : '박쥐 × 1'}
              </h1>
              <p>
                {world.observer.mode === 'localization'
                  ? '로봇 ID·위치·방향으로 지상팀의 운반을 계획합니다.'
                  : '가려진 위치를 다시 찾고, 확인된 물체부터 운반합니다.'}
              </p>
            </div>
            <div className="score-total">
              <span>{world.ended ? '최종 모의 점수' : '현재 배치 점수'}</span>
              <strong>
                {score.points}
                <small> / 160</small>
              </strong>
            </div>
          </header>
          <div className="score-breakdown">
            {[
              ['디스크 → LAB', score.counts.discs, 3],
              ['큐브 → H·PCC', score.counts.cubes, 4],
              ['빨강 → H', score.counts.red, 3],
              ['노랑 → 양쪽 PCC', score.counts.yellow, 3],
              ['초록 → RZ', score.counts.green, 3],
            ].map(([label, n, max], i) => (
              <div key={label} className={`score-kind kind-${i}`}>
                <span>{label}</span>
                <strong>
                  {n}
                  <small> / {max}개</small>
                </strong>
              </div>
            ))}
          </div>
          <div className="qualifier-workspace">
            <section className="field-surface">
              <div className="field-toolbar">
                <div>
                  <strong>경기장 · 실물 비율</strong>
                  <span>물체를 클릭하지 않아도 역할별로 자동 운반</span>
                </div>
                <div>
                  <label>
                    <input
                      type="checkbox"
                      checked={routes}
                      onChange={(e) => setRoutes(e.target.checked)}
                    />{' '}
                    경로
                  </label>
                  <label>
                    <input
                      type="checkbox"
                      checked={labels}
                      onChange={(e) => setLabels(e.target.checked)}
                    />{' '}
                    물체 ID
                  </label>
                </div>
              </div>
              <FieldView
                world={world}
                selectedId={selectedId}
                setSelected={setSelectedId}
                routes={routes}
                labels={labels}
              />
              <div className="map-footnote">
                <InfoMark />
                B-1/B-4 공개 전 연습 배치. 색 순서·샘플·LAB·RZ 좌표와 기구
                돌출은 잠정값.
              </div>
            </section>
            <aside className="mission-controls">
              <section className="control-card clock-card">
                <div className="clock-row">
                  <span>남은 시간</span>
                  <strong>
                    {Math.floor(time / 60)}:
                    {(time % 60).toFixed(1).padStart(4, '0')}
                  </strong>
                </div>
                <div className="mission-buttons">
                  <button
                    className="run-mission"
                    disabled={world.ended}
                    onClick={() => setRunning(!running)}
                  >
                    {running ? <Pause size={18} /> : <Play size={18} />}{' '}
                    {running ? '일시정지' : '임무 시작'}
                  </button>
                  <button
                    className="reset-mission"
                    onClick={() => reset()}
                    aria-label="경기 초기화"
                  >
                    <RotateCcw size={18} />
                  </button>
                </div>
                <div className="play-speed">
                  <span>재생 속도</span>
                  {[0.5, 1, 2, 4].map((n) => (
                    <button
                      key={n}
                      aria-pressed={n === speed}
                      onClick={() => setSpeed(n)}
                    >
                      {n}×
                    </button>
                  ))}
                </div>
                {world.ended && (
                  <p className={green ? 'result-good' : 'result-partial'}>
                    {green ? (
                      <CheckCircle2 size={17} />
                    ) : (
                      <AlertTriangle size={17} />
                    )}{' '}
                    {world.reason}
                  </p>
                )}
                <p className="compact-note">
                  잡은 물체는 0점. 완전 안착·해제 후의 상태로 판정.
                  {world.ended &&
                    ' 종료 후 위치는 계속 고정됩니다. 최종 촬영 5초 이상 유지하세요.'}
                </p>
                <button
                  className="declare-end"
                  disabled={world.ended || world.elapsed === 0}
                  onClick={() => {
                    finish(worldRef.current);
                    publish();
                  }}
                >
                  지금 조기 종료 · 최종 채점
                </button>
              </section>
              <AerialPanel
                world={world}
                change={(fn) => {
                  fn(worldRef.current);
                  publish();
                }}
                compare={(mode) => {
                  const next = createWorld(
                    mode === 'none' ? 'localization' : 'drone',
                  );
                  next.observer.missingId = 'H2';
                  next.drone.occlusionId = 'H2';
                  if (mode !== 'none') next.drone.strategy = mode;
                  worldRef.current = next;
                  setSelectedId('H2');
                  setRunning(true);
                  publish();
                }}
              />
              <section className="control-card">
                <h2>
                  지상팀 <small>{world.robots.length}대</small>
                </h2>
                <div className="team-list">
                  {world.robots.map((r) => (
                    <button
                      key={r.id}
                      className={r.id === selectedId ? 'selected' : ''}
                      onClick={() => setSelectedId(r.id)}
                      style={{ '--unit': r.color } as React.CSSProperties}
                    >
                      <span className="unit-id">{r.id}</span>
                      <span>
                        <strong>{r.name}</strong>
                        <small>
                          {r.blockedBy
                            ? `양보 · ${r.blockedBy}`
                            : PHASE_LABEL[r.phase]}
                        </small>
                      </span>
                      <em>
                        {r.served}/{r.jobs.length}
                      </em>
                      <ChevronRight size={15} />
                    </button>
                  ))}
                </div>
              </section>
              <section className="control-card inspector-card">
                <h2>{selected.name}</h2>
                <p className="current-job">
                  {job
                    ? `${job.itemId} → ${job.destination}`
                    : '담당 임무 완료'}{' '}
                  <span>{PHASE_LABEL[selected.phase]}</span>
                </p>
                <dl>
                  <div>
                    <dt>
                      {selected.role === 'hamster'
                        ? '디스크 게이트'
                        : '집게 서보'}
                    </dt>
                    <dd>
                      {Math.round(selected.servo * 100)}% ·{' '}
                      {selected.servo > 0.5 ? '고정' : '열림'}
                    </dd>
                  </div>
                  {selected.role === 'beaver' && (
                    <div>
                      <dt>큐브 배출 서보</dt>
                      <dd>
                        {Math.round(selected.magazineServo * 100)}% · 잔량{' '}
                        {selected.magazine.length}
                      </dd>
                    </div>
                  )}
                  <div>
                    <dt>
                      {selected.role === 'hamster'
                        ? '광센서 모의'
                        : '집게 감지 모의'}
                    </dt>
                    <dd className={selected.sensor ? 'sensor-on' : ''}>
                      {selected.sensor ? '물체 감지' : '빈 상태'}
                    </dd>
                  </div>
                  <div>
                    <dt>운반 중</dt>
                    <dd>{selected.payload ?? '없음'}</dd>
                  </div>
                  <div>
                    <dt>지상로봇 최소 간격</dt>
                    <dd>{Math.round(clearance(world) * 1000)}mm</dd>
                  </div>
                </dl>
                <label className="test-fault">
                  <input
                    type="checkbox"
                    checked={world.faultRobot === selectedId}
                    onChange={(e) => {
                      worldRef.current.faultRobot = e.target.checked
                        ? selectedId
                        : null;
                      publish();
                    }}
                  />{' '}
                  {selected.name} 센서 미감지 시험
                </label>
                {selected.fault && (
                  <p className="fault-message">
                    {selected.fault} · 초기화 후 재시험
                  </p>
                )}
              </section>
              <section className="control-card bat-card">
                <h2>
                  {world.observer.mode === 'localization' ? (
                    <Camera size={17} />
                  ) : (
                    <Radio size={17} />
                  )}{' '}
                  관측 방식 <small>변경 시 초기화</small>
                </h2>
                <div className="observer-mode" aria-label="관측 방식">
                  <button
                    type="button"
                    aria-pressed={world.observer.mode === 'localization'}
                    onClick={() => reset('localization')}
                  >
                    <Camera size={16} />
                    <span>
                      <strong>실시간 좌표 추적</strong>
                      <small>기본 위치 입력만 사용</small>
                    </span>
                  </button>
                  <button
                    type="button"
                    aria-pressed={world.observer.mode === 'drone'}
                    onClick={() => reset('drone')}
                  >
                    <Radio size={16} />
                    <span>
                      <strong>박쥐 드론 1대</strong>
                      <small>시야 탐색·가림 복구</small>
                    </span>
                  </button>
                </div>
                <p>
                  {world.drone.enabled
                    ? `${world.drone.phase === 'takeoff' ? '이륙' : world.drone.phase === 'ground' ? '시작구역 대기' : world.drone.phase === 'hold' ? '대기' : '상공 관측'} · 높이 ${world.drone.altitude.toFixed(2)}m`
                    : '위치·방향 관측 10Hz · 측정 시각·누락 상태 표시'}
                </p>
                <p className="compact-note">
                  {world.observer.mode === 'localization'
                    ? '카메라 수를 고정하지 않습니다. 실제 영상→AprilTag 좌표→추적 순으로 연결하며 현재 화면은 모의입니다.'
                    : '이동 카메라는 매 프레임 보정이 필요합니다. 실제 드론 크기·영상·비행 제어는 미연결.'}
                </p>
                <label className="test-fault">
                  <input
                    type="checkbox"
                    checked={world.observer.lost}
                    onChange={(e) => {
                      worldRef.current.observer.lost = e.target.checked;
                      publish();
                    }}
                  />{' '}
                  전체 위치 입력 끊김 시험
                </label>
                {world.observer.frameAge > 0.3 && (
                  <p className="fault-message">관측 300ms 초과 · 지상팀 대기</p>
                )}
                <label className="test-fault">
                  <input
                    type="checkbox"
                    checked={world.observer.missingId === selected.id}
                    onChange={(e) => {
                      worldRef.current.observer.missingId = e.target.checked
                        ? selected.id
                        : null;
                      publish();
                    }}
                  />{' '}
                  선택 로봇 기본 위치 입력 누락
                </label>
                {world.observer.missingId && (
                  <p className="fault-message">
                    {world.observer.missingId} 기본 입력 누락 ·{' '}
                    {world.drone.recoveredIds.includes(world.observer.missingId)
                      ? '드론 관측으로 복구'
                      : '대체 관측 확보 필요'}
                  </p>
                )}
              </section>
            </aside>
          </div>
          <SimulationLab
            world={world}
            change={(fn) => {
              fn(worldRef.current);
              publish();
            }}
            fleet={(n) => {
              worldRef.current = createExperiment(
                n === 5 ? 'localization' : worldRef.current.observer.mode,
                n,
              );
              setSelectedId('H1');
              setRunning(false);
              publish();
            }}
          />
          <PositionMonitor world={world} />
          <section className="rules-and-log">
            <div className="control-card rule-card">
              <h2>이번 경기의 판정 조건</h2>
              <p>
                <b>디스크:</b> 3개를 서로 다른 LAB 원에. 디스크 색 대응 규칙은
                없음. Ø56 → Ø60, 중심 오차 2mm 미만.
              </p>
              <p>
                <b>큐브:</b> B1·B2에 2개씩 사전 적재, 세 번째 비버는 원기둥
                담당. 병원 2개, PCC-L·PCC-R 각각 1개.
              </p>
              <p>
                <b>원기둥:</b> 색별 4개 중 3개씩 선택. 빨강→H, 초록→RZ,
                노랑→양쪽 PCC에 최소 1개씩.
              </p>
              <p>
                <b>오배치:</b> 다른 색이 남은 구역은 해당 실린더 점수 무효.
                노랑이 한쪽에만 있으면 노랑 점수 무효.
              </p>
              <p>
                <b>역할 동선:</b> 출발할 때 비버 왼쪽·햄스터 오른쪽. 햄스터는
                왼쪽 아래 격리구역으로 이동하므로 모든 지상 로봇을 충돌 검사.
              </p>
              <a
                href="https://robotics-2026.web.app/resources"
                target="_blank"
                rel="noreferrer"
              >
                공식 예선 규정·자료실 <ExternalLink size={14} />
              </a>
            </div>
            <div className="control-card mission-log">
              <h2>집기부터 배치까지</h2>
              <div role="log" aria-label="임무 실행 기록">
                {world.logs.slice(0, 24).map((entry, index) => (
                  <p key={`${entry.time}-${index}`} className={entry.level}>
                    <time>{entry.time.toFixed(1)}s</time>
                    <span>{entry.text}</span>
                  </p>
                ))}
              </div>
            </div>
          </section>
          <section className="control-card inventory">
            <h2>물체별 상태</h2>
            <div className="item-list">
              {world.items.map((item) => (
                <div key={item.id}>
                  <span
                    className={`object-key ${item.kind}`}
                    style={{
                      background: item.color
                        ? COLOR_HEX[item.color]
                        : item.kind === 'disc'
                          ? '#475569'
                          : '#e2e8f0',
                    }}
                  >
                    {item.id}
                  </span>
                  <span>
                    <strong>{labelItem(item)}</strong>
                    <small>
                      {destinationOf(world, item.id)} ·{' '}
                      {item.carrier
                        ? `${item.carrier} 적재/운반`
                        : item.released
                          ? '배치 완료'
                          : item.selected
                            ? '접근 대기'
                            : '필드 잔류'}
                    </small>
                  </span>
                </div>
              ))}
            </div>
          </section>
          <footer className="mission-footer">
            예선 v2 초안 기준 모의 채점 · 고정 색 배치 미확정 · 이동은
            메카넘·센서는 정상 응답을 가정 · 실물 서보 각도와 기구 치수 보정
            필요
          </footer>
        </main>
      )}
    </div>
  );
}
function InfoMark() {
  return <AlertTriangle size={15} />;
}
