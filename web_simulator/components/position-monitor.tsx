'use client';
import { useState } from 'react';
import type { World } from '@/lib/mission';
import {
  parsePositionLog,
  type PositionFrame,
  type MeasuredPose,
} from '@/lib/localization';

export default function PositionMonitor({ world }: { world: World }) {
  const [frames, setFrames] = useState<PositionFrame[]>([]);
  const [index, setIndex] = useState(0);
  const [error, setError] = useState('');
  const current = frames[index];
  const synthetic: MeasuredPose[] = world.robots.flatMap((r) => {
    const p = world.observer.poses[r.id];
    return p
      ? [
          {
            id: r.id,
            x: p.x_mm,
            y: p.y_mm,
            heading: p.heading_rad,
            ageMs: (world.elapsed - p.at) * 1000,
            state:
              world.observer.missingId === r.id
                ? 'missing'
                : world.elapsed - p.at > 0.5
                  ? 'stale'
                  : 'observed',
          },
        ]
      : [];
  });
  const poses = current?.poses ?? synthetic,
    field = current?.field ?? [1143, 1181];
  return (
    <section className="control-card position-monitor">
      <div className="position-heading">
        <h2>실시간 위치 · mm / 방향</h2>
        <span>{current ? '실제 검출 로그 재생' : '10Hz 좌표 관측 모의'}</span>
      </div>
      <p className="compact-note">
        고정 카메라 화면 대신 측정 좌표를 확인합니다. 현재 공개 화면은 모의
        센서이며 실물 로봇·영상은 연결되지 않았습니다. H2 = 태그 1번을 쓰는 세
        번째 비버(B3).
      </p>
      <div className="position-grid">
        <svg
          viewBox={`-30 -30 ${field[0] + 60} ${field[1] + 60}`}
          aria-label="측정 좌표 지도 · 왼쪽 아래 원점"
        >
          <rect
            width={field[0]}
            height={field[1]}
            fill="#0f2333"
            stroke="#58738a"
            strokeWidth="3"
          />
          {(
            current?.objects ??
            world.items
              .filter((o) => o.kind === 'cylinder' && !o.carrier)
              .map((o) => ({
                x: o.x * 1000,
                y: o.y * 1000,
                color: o.color!,
                kind: o.kind,
              }))
          ).map((o, i) => (
            <circle
              key={i}
              cx={o.x}
              cy={field[1] - o.y}
              r="10"
              fill={
                { red: '#ef4444', yellow: '#facc15', green: '#22c55e' }[
                  o.color
                ] ?? '#ddd'
              }
            />
          ))}
          {poses.map((p) => (
            <g
              key={p.id}
              transform={`translate(${p.x} ${field[1] - p.y})`}
              opacity={p.state === 'observed' ? 1 : 0.35}
            >
              <circle
                r="36"
                fill="none"
                stroke={p.state === 'observed' ? '#5eead4' : '#fb923c'}
                strokeWidth="4"
              />
              <path
                d={`M0 0L${Math.cos(p.heading) * 55} ${-Math.sin(p.heading) * 55}`}
                stroke="#5eead4"
                strokeWidth="6"
              />
              <text y="63" fill="white" textAnchor="middle" fontSize="30">
                {p.id}
              </text>
            </g>
          ))}
        </svg>
        <div className="position-table-wrap">
          <table className="position-table">
            <caption>
              {current
                ? `프레임 ${current.sequence} · 상대 ${(current.at - frames[0].at).toFixed(2)}초`
                : `프레임 ${world.observer.sequence} · +X가 0°`}
            </caption>
            <thead>
              <tr>
                <th>ID</th>
                <th>x / y mm</th>
                <th>방향</th>
                <th>상태 / 나이</th>
              </tr>
            </thead>
            <tbody>
              {poses.map((p) => (
                <tr key={p.id}>
                  <th>{p.id}</th>
                  <td>
                    {p.x.toFixed(1)} / {p.y.toFixed(1)}
                  </td>
                  <td>{((p.heading * 180) / Math.PI).toFixed(1)}°</td>
                  <td>
                    {(
                      {
                        observed: '관측',
                        missing: '누락',
                        stale: '만료',
                        rejected: '거부',
                      } as Record<string, string>
                    )[p.state] ?? p.state}{' '}
                    / {p.ageMs === null ? '—' : Math.round(p.ageMs)}ms
                  </td>
                </tr>
              ))}
              {!poses.length && (
                <tr>
                  <td colSpan={4}>시뮬레이션 시작 또는 검출 JSONL 불러오기</td>
                </tr>
              )}
            </tbody>
          </table>
        </div>
      </div>
      <div className="position-import">
        <label>
          검출 JSONL 확인{' '}
          <input
            type="file"
            accept=".jsonl,.json"
            onChange={async (e) => {
              const f = e.target.files?.[0];
              if (!f) return;
              try {
                if (f.size > 10_000_000)
                  throw new Error('10MB 이하 로그를 사용해 주세요.');
                const result = parsePositionLog(await f.text());
                setFrames(result);
                setIndex(0);
                setError('');
              } catch (err) {
                setFrames([]);
                setError(err instanceof Error ? err.message : '로그 읽기 실패');
              }
              e.target.value = '';
            }}
          />
        </label>
        {current && (
          <button
            type="button"
            onClick={() => {
              setFrames([]);
              setIndex(0);
            }}
          >
            모의 좌표로 돌아가기
          </button>
        )}
      </div>
      {current && (
        <label className="position-slider">
          프레임 {index + 1} / {frames.length}
          <input
            aria-label="검출 로그 프레임"
            type="range"
            min="0"
            max={frames.length - 1}
            value={index}
            onChange={(e) => setIndex(Number(e.target.value))}
          />
        </label>
      )}
      {(error || current?.rejected) && (
        <p className="fault-message">
          {error || `거부된 프레임 · ${current.reason}`}
        </p>
      )}
      <p className="compact-note">
        파일은 이 브라우저 안에서만 읽습니다. 실시간 장비 확인은 GitHub의
        --track --colors --preview 실행법을 사용하세요. 색은 종류 후보이며 같은
        색 물체의 고유 ID나 집기 성공을 보장하지 않습니다.
      </p>
    </section>
  );
}
