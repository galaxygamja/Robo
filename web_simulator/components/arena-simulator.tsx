'use client';

import {
  AlertTriangle,
  Bot,
  ExternalLink,
  Gauge,
  Gamepad2,
  Info,
  Pause,
  Play,
  RotateCcw,
  Route,
  ShieldCheck,
  StepForward,
} from 'lucide-react';
import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  FIELD,
  OFFICIAL_LAYOUT,
  ROBOT,
  ROUTES,
  type DriveMode,
  type Point,
  type RobotState,
  type SimEvent,
  createInitialRobots,
  footprintForMode,
  minimumClearance,
  stepWorld,
} from '@/lib/simulation';

const FIXED_DT = 0.01;

type WebMcpTool = {
  name: string;
  title?: string;
  description: string;
  inputSchema: Record<string, unknown>;
  annotations?: { readOnlyHint?: boolean; untrustedContentHint?: boolean };
  execute: (
    input: unknown,
  ) => Record<string, unknown> | Promise<Record<string, unknown>>;
};

type WebMcpContext = {
  registerTool: (
    tool: WebMcpTool,
    options?: { signal?: AbortSignal },
  ) => void | Promise<void>;
};

function formatTime(seconds: number) {
  const remainingTenths = Math.max(
    0,
    Math.round((FIELD.duration - seconds) * 10),
  );
  const minutes = Math.floor(remainingTenths / 600);
  const secondsTenths = remainingTenths - minutes * 600;
  return `${minutes}:${(secondsTenths / 10).toFixed(1).padStart(4, '0')}`;
}

function statusLabel(status: RobotState['status']) {
  return {
    waiting: '대기',
    moving: '경로 주행',
    manual: '수동 제어',
    complete: '도착',
    blocked: '안전정지',
    timeout: '경기 종료',
  }[status];
}

type CanvasOptions = {
  showGrid: boolean;
  showRoutes: boolean;
  showTrails: boolean;
  showSafety: boolean;
};

function drawArena(
  canvas: HTMLCanvasElement,
  robots: RobotState[],
  selectedId: string,
  driveMode: DriveMode,
  options: CanvasOptions,
) {
  const parent = canvas.parentElement;
  if (!parent) return;
  const rect = parent.getBoundingClientRect();
  if (rect.width === 0 || rect.height === 0) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(rect.width * dpr);
  canvas.height = Math.round(rect.height * dpr);
  canvas.style.width = `${rect.width}px`;
  canvas.style.height = `${rect.height}px`;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);

  const padding = rect.width < 560 ? 34 : 50;
  const scale = Math.min(
    (rect.width - padding * 2) / FIELD.width,
    (rect.height - padding * 2) / FIELD.height,
  );
  const arenaWidth = FIELD.width * scale;
  const arenaHeight = FIELD.height * scale;
  const ox = (rect.width - arenaWidth) / 2;
  const oy = (rect.height - arenaHeight) / 2;
  const toCanvas = (point: Point) => ({
    x: ox + point.x * scale,
    y: oy + (FIELD.height - point.y) * scale,
  });
  const fillWorldRect = (
    worldRect: { x: number; y: number; width: number; height: number },
    fill: string,
  ) => {
    const topLeft = toCanvas({
      x: worldRect.x,
      y: worldRect.y + worldRect.height,
    });
    ctx.fillStyle = fill;
    ctx.fillRect(
      topLeft.x,
      topLeft.y,
      worldRect.width * scale,
      worldRect.height * scale,
    );
  };
  const labelWorld = (
    text: string,
    point: Point,
    color = '#111827',
    size = 10,
  ) => {
    const p = toCanvas(point);
    ctx.fillStyle = color;
    ctx.font = `750 ${size}px ui-monospace, SFMono-Regular, monospace`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, p.x, p.y);
  };

  ctx.save();
  ctx.shadowColor = 'rgba(1, 10, 20, .55)';
  ctx.shadowBlur = 24;
  ctx.fillStyle = '#d9d7c9';
  ctx.fillRect(ox, oy, arenaWidth, arenaHeight);
  ctx.restore();

  if (options.showGrid) {
    ctx.strokeStyle = 'rgba(30, 41, 59, .09)';
    ctx.lineWidth = 1;
    for (let x = 0.1; x < FIELD.width; x += 0.1) {
      const p = toCanvas({ x, y: 0 });
      ctx.beginPath();
      ctx.moveTo(p.x, oy);
      ctx.lineTo(p.x, oy + arenaHeight);
      ctx.stroke();
    }
    for (let y = 0.1; y < FIELD.height; y += 0.1) {
      const p = toCanvas({ x: 0, y });
      ctx.beginPath();
      ctx.moveTo(ox, p.y);
      ctx.lineTo(ox + arenaWidth, p.y);
      ctx.stroke();
    }
  }

  // 국제 룰북 13~16쪽의 한 팀용 필드 선형. 검은 20mm 선은 통과 가능한 테이프다.
  fillWorldRect(OFFICIAL_LAYOUT.healthcare.pccLeft, 'rgba(96, 165, 250, .19)');
  fillWorldRect(OFFICIAL_LAYOUT.healthcare.hospital, 'rgba(248, 113, 113, .2)');
  fillWorldRect(OFFICIAL_LAYOUT.healthcare.pccRight, 'rgba(96, 165, 250, .19)');
  fillWorldRect(OFFICIAL_LAYOUT.quarantine, 'rgba(168, 85, 247, .12)');
  fillWorldRect(FIELD.startZone, 'rgba(16, 185, 129, .14)');

  const tape = '#171717';
  fillWorldRect(
    { x: 0, y: 0.981, width: FIELD.width, height: OFFICIAL_LAYOUT.tapeWidth },
    tape,
  );
  fillWorldRect({ x: 0.3, y: 1.001, width: 0.02, height: 0.18 }, tape);
  fillWorldRect({ x: 0.823, y: 1.001, width: 0.02, height: 0.18 }, tape);
  OFFICIAL_LAYOUT.centerTape.forEach((segment) => fillWorldRect(segment, tape));
  fillWorldRect({ x: 0, y: 0.28, width: 0.3, height: 0.02 }, tape);
  fillWorldRect({ x: 0.28, y: 0, width: 0.02, height: 0.3 }, tape);
  fillWorldRect({ x: 0.643, y: 0, width: 0.02, height: 0.3 }, tape);
  fillWorldRect({ x: 0.643, y: 0.28, width: 0.5, height: 0.02 }, tape);

  labelWorld('PCC', { x: 0.15, y: 1.145 }, '#1e3a8a', 11);
  labelWorld('H', { x: 0.5715, y: 1.145 }, '#7f1d1d', 14);
  labelWorld('PCC', { x: 0.993, y: 1.145 }, '#1e3a8a', 11);
  labelWorld('격리구역', { x: 0.14, y: 0.245 }, '#581c87', 9);
  labelWorld('START 480 × 280', { x: 0.903, y: 0.245 }, '#065f46', 9);

  const rzTopLeft = toCanvas({ x: 0.69, y: 0.18 });
  ctx.strokeStyle = 'rgba(5, 150, 105, .68)';
  ctx.lineWidth = 1.3;
  ctx.setLineDash([5, 4]);
  ctx.strokeRect(rzTopLeft.x, rzTopLeft.y, 0.42 * scale, 0.13 * scale);
  ctx.setLineDash([]);
  labelWorld('RZ', { x: 0.9, y: 0.115 }, '#047857', 10);

  OFFICIAL_LAYOUT.groundPoints.forEach((point) => {
    const p = toCanvas(point);
    ctx.beginPath();
    ctx.arc(p.x, p.y, Math.max(3.2, 0.01 * scale), 0, Math.PI * 2);
    ctx.fillStyle = point.color;
    ctx.fill();
    ctx.strokeStyle = 'rgba(15, 23, 42, .78)';
    ctx.lineWidth = 1;
    ctx.stroke();
  });

  OFFICIAL_LAYOUT.laboratorySlots.forEach((point) => {
    const p = toCanvas(point);
    ctx.beginPath();
    ctx.arc(p.x, p.y, Math.max(5, 0.03 * scale), 0, Math.PI * 2);
    ctx.fillStyle = '#ede9d5';
    ctx.fill();
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1.2;
    ctx.stroke();
  });
  labelWorld('LAB · 좌표 잠정', { x: 0.49, y: 0.16 }, '#475569', 8);

  [0.07, 0.14, 0.21].forEach((x) => {
    const p = toCanvas({ x, y: 0.08 });
    ctx.beginPath();
    ctx.arc(p.x, p.y, Math.max(5, 0.028 * scale), 0, Math.PI * 2);
    ctx.fillStyle = '#20252b';
    ctx.fill();
    ctx.strokeStyle = '#020617';
    ctx.lineWidth = 1;
    ctx.stroke();
  });

  if (options.showRoutes) {
    ROUTES.forEach((route, index) => {
      const robot = robots[index];
      const points = [robot.trail[0], ...route];
      ctx.beginPath();
      points.forEach((point, pointIndex) => {
        const p = toCanvas(point);
        if (pointIndex === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      });
      ctx.strokeStyle = `${robot.color}bb`;
      ctx.lineWidth = robot.id === selectedId ? 2 : 1;
      ctx.setLineDash([5, 7]);
      ctx.stroke();
      ctx.setLineDash([]);
    });
  }

  ROUTES.forEach((route, index) => {
    const goal = toCanvas(route[route.length - 1]);
    ctx.beginPath();
    ctx.arc(goal.x, goal.y, Math.max(7, 0.018 * scale), 0, Math.PI * 2);
    ctx.fillStyle = '#f8fafc';
    ctx.fill();
    ctx.strokeStyle = robots[index].color;
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = robots[index].color;
    ctx.font = '700 9px ui-monospace, SFMono-Regular, monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(`${index + 1}`, goal.x, goal.y + 0.5);
  });
  ctx.textAlign = 'start';
  ctx.textBaseline = 'alphabetic';

  if (options.showTrails) {
    for (const robot of robots) {
      if (robot.trail.length < 2) continue;
      ctx.beginPath();
      robot.trail.forEach((point, index) => {
        const p = toCanvas(point);
        if (index === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      });
      ctx.strokeStyle = robot.color;
      ctx.globalAlpha = 0.58;
      ctx.lineWidth = robot.id === selectedId ? 2.2 : 1.2;
      ctx.stroke();
      ctx.globalAlpha = 1;
    }
  }

  for (const robot of robots) {
    const center = toCanvas(robot.pose);
    const selected = robot.id === selectedId;
    if (options.showSafety && selected) {
      ctx.beginPath();
      ctx.arc(center.x, center.y, ROBOT.displayRadius * scale, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(251, 191, 36, .72)';
      ctx.lineWidth = 1.2;
      ctx.setLineDash([4, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    ctx.save();
    ctx.translate(center.x, center.y);
    ctx.rotate(-robot.pose.heading);
    ctx.beginPath();
    footprintForMode(driveMode).forEach((point, index) => {
      const x = point.x * scale;
      const y = -point.y * scale;
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.shadowColor = selected ? robot.color : 'rgba(0, 0, 0, .6)';
    ctx.shadowBlur = selected ? 15 : 5;
    ctx.fillStyle = robot.status === 'blocked' ? '#3b1420' : '#0e2635';
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.strokeStyle = robot.status === 'blocked' ? '#fb7185' : robot.color;
    ctx.lineWidth = selected ? 2.4 : 1.4;
    ctx.stroke();

    const wheelCenters =
      driveMode === 'mecanum'
        ? [
            [-ROBOT.mecanumWheelCenterX, -ROBOT.mecanumWheelCenterY],
            [ROBOT.mecanumWheelCenterX, -ROBOT.mecanumWheelCenterY],
            [-ROBOT.mecanumWheelCenterX, ROBOT.mecanumWheelCenterY],
            [ROBOT.mecanumWheelCenterX, ROBOT.mecanumWheelCenterY],
          ]
        : [
            [-ROBOT.differentialHalfTrack, -0.015],
            [ROBOT.differentialHalfTrack, -0.015],
          ];
    for (const [wx, wy] of wheelCenters) {
      const width = 0.012 * scale;
      const height = (driveMode === 'mecanum' ? 0.025 : 0.045) * scale;
      ctx.fillStyle = '#02070c';
      ctx.fillRect(
        wx * scale - width / 2,
        -wy * scale - height / 2,
        width,
        height,
      );
      if (driveMode === 'mecanum') {
        ctx.strokeStyle = '#7dd3fc';
        ctx.lineWidth = 1;
        const direction = wx * wy < 0 ? 1 : -1;
        for (let offset = -0.008; offset <= 0.008; offset += 0.008) {
          ctx.beginPath();
          ctx.moveTo(
            wx * scale - width / 2,
            -(wy + offset) * scale - direction * 3,
          );
          ctx.lineTo(
            wx * scale + width / 2,
            -(wy + offset) * scale + direction * 3,
          );
          ctx.stroke();
        }
      }
    }
    if (driveMode === 'differential') {
      ctx.beginPath();
      ctx.arc(0, 0.034 * scale, Math.max(1.5, 0.006 * scale), 0, Math.PI * 2);
      ctx.fillStyle = '#94a3b8';
      ctx.fill();
    }

    ctx.beginPath();
    ctx.moveTo(0, -0.014 * scale);
    ctx.lineTo(0, -0.041 * scale);
    ctx.lineTo(-0.007 * scale, -0.033 * scale);
    ctx.moveTo(0, -0.041 * scale);
    ctx.lineTo(0.007 * scale, -0.033 * scale);
    ctx.strokeStyle = robot.color;
    ctx.lineWidth = 1.6;
    ctx.stroke();
    ctx.restore();

    ctx.fillStyle = selected ? '#f8fafc' : '#cbd5e1';
    ctx.font = `${selected ? 700 : 600} ${selected ? 11 : 9}px ui-monospace, SFMono-Regular, monospace`;
    ctx.textAlign = 'center';
    ctx.fillText(robot.id, center.x, center.y + 4);
  }
  ctx.textAlign = 'start';

  ctx.strokeStyle = '#8b6b3f';
  ctx.lineWidth = Math.max(2, 0.012 * scale);
  ctx.strokeRect(ox, oy, arenaWidth, arenaHeight);
  ctx.fillStyle = '#7895a5';
  ctx.font = '600 10px ui-monospace, SFMono-Regular, monospace';
  ctx.textAlign = 'center';
  ctx.fillText('1,143 mm', ox + arenaWidth / 2, oy - 16);
  ctx.save();
  ctx.translate(ox - 22, oy + arenaHeight / 2);
  ctx.rotate(-Math.PI / 2);
  ctx.fillText('1,181 mm', 0, 0);
  ctx.restore();
  ctx.textAlign = 'start';
}

function Toggle({
  checked,
  onChange,
  label,
}: {
  checked: boolean;
  onChange: (value: boolean) => void;
  label: string;
}) {
  return (
    <label className="toggle-row">
      <span>{label}</span>
      <input
        type="checkbox"
        checked={checked}
        onChange={(event) => onChange(event.target.checked)}
      />
      <span className="toggle-track" aria-hidden="true">
        <span />
      </span>
    </label>
  );
}

function WheelMeter({ label, value }: { label: string; value: number }) {
  const magnitude = Math.min(100, Math.abs(value) * 100);
  return (
    <div className="wheel-meter">
      <div className="wheel-meter-label">
        <span>{label}</span>
        <strong>
          {value >= 0 ? '+' : ''}
          {value.toFixed(2)}
        </strong>
      </div>
      <div className="wheel-meter-track">
        <span
          className={value >= 0 ? 'positive' : 'negative'}
          style={{ width: `${magnitude}%` }}
        />
      </div>
    </div>
  );
}

export default function ArenaSimulator() {
  const [robots, setRobots] = useState(createInitialRobots);
  const robotsRef = useRef(robots);
  const [selectedId, setSelectedId] = useState('R1');
  const [driveMode, setDriveMode] = useState<DriveMode>('mecanum');
  const [running, setRunning] = useState(false);
  const [manual, setManual] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [elapsed, setElapsed] = useState(0);
  const elapsedRef = useRef(0);
  const [events, setEvents] = useState<SimEvent[]>([
    {
      id: 1,
      time: 0,
      level: 'info',
      text: '시뮬레이터 준비 완료 · 자동 출력 없음',
    },
  ]);
  const eventId = useRef(2);
  const keys = useRef(new Set<string>());
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [showGrid, setShowGrid] = useState(true);
  const [showRoutes, setShowRoutes] = useState(true);
  const [showTrails, setShowTrails] = useState(true);
  const [showSafety, setShowSafety] = useState(true);

  const selected = useMemo(
    () => robots.find((robot) => robot.id === selectedId) ?? robots[0],
    [robots, selectedId],
  );

  const pushEvents = useCallback((incoming: Omit<SimEvent, 'id'>[]) => {
    if (incoming.length === 0) return;
    const stamped = incoming.map((event) => ({
      ...event,
      id: eventId.current++,
    }));
    setEvents((current) => [...stamped, ...current].slice(0, 12));
  }, []);

  const runFixedSteps = useCallback(
    (count: number) => {
      let working = robotsRef.current;
      const accumulatedEvents: Omit<SimEvent, 'id'>[] = [];
      for (let index = 0; index < count; index += 1) {
        if (elapsedRef.current >= FIELD.duration) break;
        elapsedRef.current = Math.min(
          FIELD.duration,
          elapsedRef.current + FIXED_DT,
        );
        const result = stepWorld(
          working,
          FIXED_DT,
          elapsedRef.current,
          driveMode,
          manual,
          selectedId,
          keys.current,
        );
        working = result.robots;
        accumulatedEvents.push(...result.events);
      }
      robotsRef.current = working;
      setRobots(working);
      setElapsed(elapsedRef.current);
      pushEvents(accumulatedEvents);
      if (
        elapsedRef.current >= FIELD.duration ||
        working.every(
          (robot) => robot.status === 'complete' || robot.status === 'blocked',
        )
      ) {
        setRunning(false);
      }
    },
    [driveMode, manual, pushEvents, selectedId],
  );

  useEffect(() => {
    if (!running) return;
    let animationFrame = 0;
    let last = performance.now();
    let accumulator = 0;
    const frame = (now: number) => {
      const frameSeconds = Math.min((now - last) / 1000, 0.1) * speed;
      last = now;
      accumulator += frameSeconds;
      const steps = Math.min(20, Math.floor(accumulator / FIXED_DT));
      if (steps > 0) {
        runFixedSteps(steps);
        accumulator -= steps * FIXED_DT;
      }
      animationFrame = requestAnimationFrame(frame);
    };
    animationFrame = requestAnimationFrame(frame);
    return () => cancelAnimationFrame(animationFrame);
  }, [runFixedSteps, running, speed]);

  useEffect(() => {
    const pressedKeys = keys.current;
    const down = (event: KeyboardEvent) => {
      const tag = (event.target as HTMLElement | null)?.tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
      const key = event.key.toLowerCase();
      if (
        [
          'w',
          'a',
          's',
          'd',
          'q',
          'e',
          'arrowup',
          'arrowdown',
          'arrowleft',
          'arrowright',
        ].includes(key)
      ) {
        pressedKeys.add(key);
        event.preventDefault();
      }
      if (key === ' ' && tag !== 'BUTTON' && tag !== 'A' && !event.repeat) {
        setRunning((value) => !value);
        event.preventDefault();
      }
    };
    const up = (event: KeyboardEvent) =>
      pressedKeys.delete(event.key.toLowerCase());
    const clearKeys = () => pressedKeys.clear();
    const clearHiddenKeys = () => {
      if (document.hidden) pressedKeys.clear();
    };
    window.addEventListener('keydown', down);
    window.addEventListener('keyup', up);
    window.addEventListener('blur', clearKeys);
    document.addEventListener('visibilitychange', clearHiddenKeys);
    return () => {
      window.removeEventListener('keydown', down);
      window.removeEventListener('keyup', up);
      window.removeEventListener('blur', clearKeys);
      document.removeEventListener('visibilitychange', clearHiddenKeys);
      pressedKeys.clear();
    };
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const redraw = () =>
      drawArena(canvas, robots, selectedId, driveMode, {
        showGrid,
        showRoutes,
        showTrails,
        showSafety,
      });
    redraw();
    const observer = new ResizeObserver(redraw);
    if (canvas.parentElement) observer.observe(canvas.parentElement);
    return () => observer.disconnect();
  }, [
    driveMode,
    robots,
    selectedId,
    showGrid,
    showRoutes,
    showSafety,
    showTrails,
  ]);

  const reset = useCallback(() => {
    const initial = createInitialRobots();
    robotsRef.current = initial;
    elapsedRef.current = 0;
    keys.current.clear();
    setRobots(initial);
    setElapsed(0);
    setRunning(false);
    setManual(false);
    setEvents([
      {
        id: eventId.current++,
        time: 0,
        level: 'info',
        text: `${driveMode === 'mecanum' ? '메카넘 개조안' : '현재 차동구동'} 초기화`,
      },
    ]);
  }, [driveMode]);

  const changeDriveMode = useCallback((value: DriveMode) => {
    setDriveMode(value);
    const initial = createInitialRobots();
    robotsRef.current = initial;
    elapsedRef.current = 0;
    keys.current.clear();
    setRobots(initial);
    setElapsed(0);
    setRunning(false);
    setManual(false);
    setEvents([
      {
        id: eventId.current++,
        time: 0,
        level: 'info',
        text:
          value === 'mecanum'
            ? '메카넘 개조안 선택'
            : '현재 SCAD 차동구동 선택',
      },
    ]);
  }, []);

  useEffect(() => {
    const context = (document as Document & { modelContext?: WebMcpContext })
      .modelContext;
    if (!context?.registerTool) return;
    const lifecycle = new AbortController();
    const afterVisibleUpdate = () =>
      new Promise<void>((resolve) =>
        requestAnimationFrame(() => requestAnimationFrame(() => resolve())),
      );
    const register = (tool: WebMcpTool) => {
      try {
        void Promise.resolve(
          context.registerTool(tool, { signal: lifecycle.signal }),
        ).catch(() => undefined);
      } catch {
        // WebMCP is optional; visible controls remain the source of truth.
      }
    };

    register({
      name: 'read_simulation_state',
      title: '시뮬레이션 상태 읽기',
      description: '현재 경기 시간, 구동 모델, 로봇별 위치와 상태를 읽습니다.',
      inputSchema: {
        type: 'object',
        properties: {},
        additionalProperties: false,
      },
      annotations: { readOnlyHint: true, untrustedContentHint: false },
      execute: () => ({
        elapsedSeconds: Number(elapsedRef.current.toFixed(2)),
        driveMode,
        selectedRobot: selectedId,
        robots: robotsRef.current.map((robot) => ({
          id: robot.id,
          xMm: Math.round(robot.pose.x * 1000),
          yMm: Math.round(robot.pose.y * 1000),
          headingDeg: Math.round((robot.pose.heading * 180) / Math.PI),
          status: robot.status,
        })),
      }),
    });
    register({
      name: 'start_route_replay',
      title: '자동 경로 재생 시작',
      description: '화면의 자동 경로 재생을 현재 위치에서 시작합니다.',
      inputSchema: {
        type: 'object',
        properties: {},
        additionalProperties: false,
      },
      annotations: { readOnlyHint: false, untrustedContentHint: false },
      execute: async () => {
        setManual(false);
        setRunning(true);
        await afterVisibleUpdate();
        return { running: true, mode: 'route_replay', driveMode };
      },
    });
    register({
      name: 'pause_simulation',
      title: '시뮬레이션 일시정지',
      description: '자동 또는 수동 시뮬레이션 시간을 일시정지합니다.',
      inputSchema: {
        type: 'object',
        properties: {},
        additionalProperties: false,
      },
      annotations: { readOnlyHint: false, untrustedContentHint: false },
      execute: async () => {
        setRunning(false);
        await afterVisibleUpdate();
        return {
          running: false,
          elapsedSeconds: Number(elapsedRef.current.toFixed(2)),
        };
      },
    });
    register({
      name: 'reset_simulation',
      title: '시뮬레이션 초기화',
      description:
        '6대 로봇과 경기 시간을 선택한 구동 모델의 시작 상태로 되돌립니다.',
      inputSchema: {
        type: 'object',
        properties: {},
        additionalProperties: false,
      },
      annotations: { readOnlyHint: false, untrustedContentHint: false },
      execute: async () => {
        reset();
        await afterVisibleUpdate();
        return { reset: true, driveMode };
      },
    });
    register({
      name: 'set_drive_model',
      title: '구동 모델 설정',
      description:
        '메카넘 개조안 또는 현재 SCAD 차동구동 모델로 바꾸고 초기화합니다.',
      inputSchema: {
        type: 'object',
        properties: {
          mode: { type: 'string', enum: ['mecanum', 'differential'] },
        },
        required: ['mode'],
        additionalProperties: false,
      },
      annotations: { readOnlyHint: false, untrustedContentHint: false },
      execute: async (input) => {
        const value = (input as { mode?: unknown } | null)?.mode;
        if (value !== 'mecanum' && value !== 'differential') {
          throw new Error('mode는 mecanum 또는 differential이어야 합니다.');
        }
        changeDriveMode(value);
        await afterVisibleUpdate();
        return { driveMode: value, reset: true };
      },
    });
    return () => lifecycle.abort();
  }, [changeDriveMode, driveMode, reset, selectedId]);

  const pressDriveKey = useCallback(
    (event: React.PointerEvent<HTMLButtonElement>) => {
      const key = event.currentTarget.dataset.driveKey;
      if (!key) return;
      event.currentTarget.setPointerCapture(event.pointerId);
      keys.current.add(key);
    },
    [],
  );

  const releaseDriveKey = useCallback(
    (event: React.PointerEvent<HTMLButtonElement>) => {
      const key = event.currentTarget.dataset.driveKey;
      if (key) keys.current.delete(key);
    },
    [],
  );

  const clearance = minimumClearance(robots, driveMode);
  const completed = robots.filter(
    (robot) => robot.status === 'complete',
  ).length;
  const blocked = robots.filter((robot) => robot.status === 'blocked').length;

  return (
    <main className="app-shell">
      <header className="topbar">
        <div className="brand-block">
          <div className="brand-mark">
            <Bot size={19} />
          </div>
          <div>
            <p className="eyebrow">ROBO CONTROL LAB · WEB SIM v0.3</p>
            <h1>6대 로봇 경기 시뮬레이터</h1>
          </div>
        </div>
        <div className="header-pills">
          <span>
            <i className="status-dot" /> 브라우저 로컬 연산
          </span>
          <span>실장치 출력 없음</span>
        </div>
      </header>

      <section className="mission-strip" aria-label="경기 상태">
        <div className="mission-clock">
          <span>ROUND CLOCK</span>
          <strong>{formatTime(elapsed)}</strong>
        </div>
        <div className="metric">
          <span>진행률</span>
          <strong>
            {completed}
            <small>/6</small>
          </strong>
        </div>
        <div className="metric">
          <span>최소 간격</span>
          <strong className={clearance < FIELD.safetyMargin ? 'danger' : ''}>
            {Math.round(clearance * 1000)}
            <small> mm</small>
          </strong>
        </div>
        <div className="metric">
          <span>안전정지</span>
          <strong className={blocked ? 'danger' : ''}>{blocked}</strong>
        </div>
        <div className="metric mode-metric">
          <span>구동 모델</span>
          <strong>
            {driveMode === 'mecanum' ? 'MECANUM' : 'DIFFERENTIAL'}
          </strong>
        </div>
      </section>

      <section className="workspace">
        <div className="arena-panel panel">
          <div className="panel-heading">
            <div>
              <p className="eyebrow">FIELD / TRUE SCALE</p>
              <h2>국제 룰북 기준 · 한 팀용 경기장</h2>
            </div>
            <div className="legend">
              <span>
                <i className="legend-start" /> 출발/RZ
              </span>
              <span>
                <i className="legend-tape" /> 20mm 테이프
              </span>
              <span>
                <i className="legend-patient" /> 환자 위치
              </span>
              <span>
                <i className="legend-route" /> 계획 경로
              </span>
            </div>
          </div>
          <div className="canvas-wrap">
            <canvas ref={canvasRef} aria-label="6대 로봇 경기장 시뮬레이션" />
          </div>
          <div className="arena-caption">
            <Info size={14} />
            <span>
              국제 룰북 13~16쪽의 필드 선형·구역을 반영했습니다. 한국 예선
              B-1/B-2/B-4 고정 배치도는 아직 미공개이므로 LAB 좌표, 6대 시작점과
              자동 경로는 검증용 잠정값입니다.
            </span>
            <a
              href="https://robotics-2026.web.app/resources"
              target="_blank"
              rel="noreferrer"
            >
              공식 자료실 <ExternalLink size={11} />
            </a>
          </div>
        </div>

        <aside className="control-column">
          <section className="panel control-panel">
            <div className="panel-heading compact">
              <div>
                <p className="eyebrow">DRIVE MODEL</p>
                <h2>구동 방식 비교</h2>
              </div>
            </div>
            <fieldset className="segmented-control" aria-label="구동 방식">
              <button
                type="button"
                className={driveMode === 'mecanum' ? 'active' : ''}
                aria-pressed={driveMode === 'mecanum'}
                onClick={() => changeDriveMode('mecanum')}
              >
                메카넘 개조안
              </button>
              <button
                type="button"
                className={driveMode === 'differential' ? 'active' : ''}
                aria-pressed={driveMode === 'differential'}
                onClick={() => changeDriveMode('differential')}
              >
                현재 SCAD
              </button>
            </fieldset>
            <p className="mode-note">
              {driveMode === 'mecanum'
                ? '4개 휠을 독립 구동해 차체 방향을 유지한 채 좌우 이동합니다.'
                : '2개 구동륜 + 캐스터로 같은 잠정 경로를 재생합니다. 횡이동 대신 매 구간에서 차체 방향을 돌립니다.'}
            </p>

            <div className="primary-controls">
              <button
                type="button"
                className="control-primary"
                onClick={() => {
                  if (manual) {
                    setManual(false);
                    setRunning(true);
                  } else {
                    setRunning((value) => !value);
                  }
                }}
              >
                {running && !manual ? <Pause size={17} /> : <Play size={17} />}
                {running && !manual ? '일시정지' : '자동 경로 시작'}
              </button>
              <button
                type="button"
                className="icon-control"
                onClick={reset}
                aria-label="초기화"
              >
                <RotateCcw size={17} />
              </button>
              <button
                type="button"
                className="icon-control"
                onClick={() => runFixedSteps(10)}
                disabled={running || elapsed >= FIELD.duration}
                aria-label="100밀리초 한 단계 실행"
              >
                <StepForward size={17} />
              </button>
            </div>

            <div className="speed-row">
              <span>
                <Gauge size={14} /> 재생 속도
              </span>
              <div>
                {[0.25, 0.5, 1, 2, 4].map((value) => (
                  <button
                    key={value}
                    type="button"
                    className={speed === value ? 'active' : ''}
                    onClick={() => setSpeed(value)}
                  >
                    {value}×
                  </button>
                ))}
              </div>
            </div>
            <div className="view-options">
              <Toggle
                checked={showGrid}
                onChange={setShowGrid}
                label="100mm 격자"
              />
              <Toggle
                checked={showRoutes}
                onChange={setShowRoutes}
                label="계획 경로"
              />
              <Toggle
                checked={showTrails}
                onChange={setShowTrails}
                label="주행 궤적"
              />
              <Toggle
                checked={showSafety}
                onChange={setShowSafety}
                label="회전 안전원"
              />
            </div>
          </section>

          <section className="panel robot-panel">
            <div className="panel-heading compact">
              <div>
                <p className="eyebrow">ROBOT INSPECTOR</p>
                <h2>로봇 선택 · 수동 시험</h2>
              </div>
              <Gamepad2 size={18} />
            </div>
            <fieldset className="robot-tabs" aria-label="로봇 선택">
              {robots.map((robot) => (
                <button
                  type="button"
                  key={robot.id}
                  className={selectedId === robot.id ? 'active' : ''}
                  aria-pressed={selectedId === robot.id}
                  onClick={() => {
                    keys.current.clear();
                    setSelectedId(robot.id);
                  }}
                  style={
                    { '--robot-color': robot.color } as React.CSSProperties
                  }
                >
                  {robot.id}
                </button>
              ))}
            </fieldset>
            <div className="robot-readout">
              <div>
                <span>상태</span>
                <strong>{statusLabel(selected.status)}</strong>
              </div>
              <div>
                <span>X / Y</span>
                <strong>
                  {Math.round(selected.pose.x * 1000)} /{' '}
                  {Math.round(selected.pose.y * 1000)} mm
                </strong>
              </div>
              <div>
                <span>방향</span>
                <strong>
                  {Math.round((selected.pose.heading * 180) / Math.PI)}°
                </strong>
              </div>
            </div>
            <button
              type="button"
              className={manual ? 'manual-toggle active' : 'manual-toggle'}
              aria-pressed={manual}
              onClick={() => {
                keys.current.clear();
                setManual((value) => !value);
                setRunning(true);
              }}
            >
              <Gamepad2 size={15} />{' '}
              {manual ? '수동 제어 중' : '수동 제어 켜기'}
            </button>
            <div className="keypad" aria-label="수동 이동 버튼">
              {[
                ['q', 'Q', '좌회전'],
                ['w', 'W', '전진'],
                ['e', 'E', '우회전'],
                ['a', 'A', '좌이동'],
                ['s', 'S', '후진'],
                ['d', 'D', '우이동'],
              ].map(([key, main, sub]) => (
                <button
                  type="button"
                  key={key}
                  data-drive-key={key}
                  onPointerDown={pressDriveKey}
                  onPointerUp={releaseDriveKey}
                  onPointerCancel={releaseDriveKey}
                  onPointerLeave={releaseDriveKey}
                  disabled={
                    driveMode !== 'mecanum' && (key === 'a' || key === 'd')
                  }
                >
                  {main}
                  <small>{sub}</small>
                </button>
              ))}
            </div>
            {driveMode === 'differential' && (
              <p className="inline-warning">
                <AlertTriangle size={13} /> 현재 구조에서 A/D 횡이동은
                불가능합니다.
              </p>
            )}
          </section>

          <section className="panel wheels-panel">
            <div className="panel-heading compact">
              <div>
                <p className="eyebrow">NORMALIZED OUTPUT</p>
                <h2>바퀴 명령</h2>
              </div>
            </div>
            <div className="wheel-grid">
              <WheelMeter label="FL" value={selected.wheels.fl} />
              <WheelMeter label="FR" value={selected.wheels.fr} />
              <WheelMeter label="RL" value={selected.wheels.rl} />
              <WheelMeter label="RR" value={selected.wheels.rr} />
            </div>
            <p className="fine-print">
              +는 기준 정회전, −는 역회전. 실기 부호는 배선 확인 후 보정합니다.
            </p>
          </section>
        </aside>
      </section>

      <section className="lower-grid">
        <div className="panel spec-panel">
          <div className="panel-heading">
            <div>
              <p className="eyebrow">CALCULATED GEOMETRY</p>
              <h2>SCAD 명목치 기반 운용 규격</h2>
            </div>
            <ShieldCheck size={19} />
          </div>
          <div className="spec-grid">
            <article>
              <span>국제 1개 필드</span>
              <strong>1,143 × 1,181</strong>
              <small>mm · 팀 진행 방향 기준</small>
            </article>
            <article>
              <span>SCAD 명목 운용 외형</span>
              <strong>126 × 100 × 100</strong>
              <small>mm</small>
            </article>
            <article>
              <span>출발구역</span>
              <strong>480 × 280</strong>
              <small>mm</small>
            </article>
            <article>
              <span>선택 안전원</span>
              <strong>Ø 162</strong>
              <small>mm</small>
            </article>
          </div>
          <p className="panel-footnote">
            차동구동은 SCAD 외형을, 메카넘은 앞뒤 휠 4개까지 감싼 8각형을 사용해
            100Hz로 간격을 검사합니다. 연속 swept 충돌은 모델링하지 않으며,
            출력·조립 뒤 실측 보정이 필요합니다.
          </p>
        </div>

        <div className="panel matrix-panel">
          <div className="panel-heading">
            <div>
              <p className="eyebrow">MECANUM MIX</p>
              <h2>회전 없이 좌우로 가는 원리</h2>
            </div>
            <Route size={19} />
          </div>
          <table className="movement-table">
            <caption className="sr-only">메카넘 바퀴 회전 조합</caption>
            <thead>
              <tr className="movement-head">
                <th>명령</th>
                <th>FL</th>
                <th>FR</th>
                <th>RL</th>
                <th>RR</th>
              </tr>
            </thead>
            <tbody>
              {[
                ['전진', '+', '+', '+', '+'],
                ['후진', '−', '−', '−', '−'],
                ['좌이동', '−', '+', '+', '−'],
                ['우이동', '+', '−', '−', '+'],
                ['반시계', '−', '+', '−', '+'],
              ].map((row) => (
                <tr className="movement-row" key={row[0]}>
                  {row.map((cell, index) =>
                    index === 0 ? (
                      <th scope="row" key={`${row[0]}-${index}`}>
                        {cell}
                      </th>
                    ) : (
                      <td key={`${row[0]}-${index}`}>{cell}</td>
                    ),
                  )}
                </tr>
              ))}
            </tbody>
          </table>
          <p className="panel-footnote">
            사선 롤러가 만드는 힘을 네 바퀴에서 합성합니다. 정석 제어에는 바퀴별
            양방향 모터 4채널, 엔코더 PID, IMU 방향 보정이 필요합니다.
          </p>
        </div>

        <div className="panel event-panel">
          <div className="panel-heading">
            <div>
              <p className="eyebrow">EVENT STREAM</p>
              <h2>실행 기록</h2>
            </div>
          </div>
          <div className="event-list" aria-live="polite">
            {events.map((event) => (
              <div className={`event ${event.level}`} key={event.id}>
                <time>{event.time.toFixed(2)}s</time>
                <span>{event.text}</span>
              </div>
            ))}
          </div>
        </div>
      </section>

      <section className="analysis-panel panel">
        <div>
          <p className="eyebrow">RC CAR TEARDOWN DECISION</p>
          <h2>쿠팡 RC카에서 가져올 것은 차체가 아니라 ‘메카넘 구동 원리’</h2>
          <p>
            상품 사진과 주행 설명은 사선 롤러 4륜의 메카넘형 구조와 일치합니다.
            하지만 공개 정보만으로 바퀴별 독립 모터, 모터 전압, 스톨 전류,
            엔코더 유무는 확정할 수 없습니다. 36×17cm급 동일 외형 제품은 이
            경기장과 현재 126×100mm 로봇에는 너무 크므로, 휠 배열과 제어식을
            소형 CAD에 다시 설계하는 편이 맞습니다.
          </p>
        </div>
        <div className="decision-list">
          <span>
            <b>1</b> X자 방향 메카넘 휠 4개
          </span>
          <span>
            <b>2</b> DC 모터 4개 + 4채널 양방향 드라이버
          </span>
          <span>
            <b>3</b> ESP32 1개, 엔코더 4개, IMU 1개
          </span>
          <span>
            <b>4</b> 상부 카메라 x/y/yaw 폐루프 보정
          </span>
        </div>
        <a
          href="https://www.coupang.com/vp/products/9483204917"
          target="_blank"
          rel="noreferrer"
          className="source-link"
        >
          분석한 상품 보기 <ExternalLink size={14} />
        </a>
      </section>

      <footer>
        <span>
          Robo Control Lab · official international field reference · 100 Hz
          model
        </span>
        <span>실물 연결 전: 치수 실측 → 1대 HIL → 2대 교차 → 6대 순서</span>
      </footer>
    </main>
  );
}
