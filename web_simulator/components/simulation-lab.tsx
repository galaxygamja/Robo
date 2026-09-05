'use client';
import type { World } from '@/lib/mission';
import { experimentReport } from '@/lib/experiments';

export default function SimulationLab({
  world,
  change,
  fleet,
}: {
  world: World;
  change: (fn: (world: World) => void) => void;
  fleet: (n: 4 | 5) => void;
}) {
  return (
    <section className="control-card simulation-lab">
      <div className="position-heading">
        <h2>재료 없이 · 안전 시험실</h2>
        <span>실물 출력 OFF</span>
      </div>
      <div className="lab-controls">
        <label>
          로봇 구성 (변경 시 초기화)
          <select
            value={world.robots.length}
            onChange={(e) => fleet(Number(e.target.value) as 4 | 5)}
          >
            <option value="4">기본 · 햄스터 1 + 비버 3</option>
            <option value="5">혼잡 시험 · 햄스터 1 + 비버 4</option>
          </select>
        </label>
        <label>
          위치 입력 지연
          <select
            value={world.observer.delayMs}
            onChange={(e) =>
              change((w) => {
                w.observer.delayMs = Number(e.target.value);
                w.observer.queue = [];
              })
            }
          >
            <option value="0">0ms · 이상적</option>
            <option value="100">100ms</option>
            <option value="400">400ms · 만료 정지 시험</option>
          </select>
        </label>
        <label>
          설정한 위치 오차 상한
          <select
            value={world.observer.noiseMm}
            onChange={(e) =>
              change((w) => {
                w.observer.noiseMm = Number(e.target.value);
              })
            }
          >
            <option value="0">0mm · 이상적</option>
            <option value="0.5">0.5mm · 좌표 흔들림</option>
            <option value="3">3mm · 정밀 작업 보류</option>
          </select>
        </label>
        <button
          className="lab-estop"
          type="button"
          onClick={() =>
            change((w) => {
              w.emergencyStopped = true;
              w.safetyReason = '비상정지 잠금 · 초기화해야 해제';
              w.robots.forEach((r) => {
                r.velocity = { x: 0, y: 0 };
              });
            })
          }
        >
          비상정지 잠금
        </button>
        <button
          type="button"
          onClick={() => {
            const url = URL.createObjectURL(
              new Blob([JSON.stringify(experimentReport(world), null, 2)], {
                type: 'application/json',
              }),
            );
            const anchor = document.createElement('a');
            anchor.href = url;
            anchor.download = 'robo-experiment.json';
            anchor.click();
            setTimeout(() => URL.revokeObjectURL(url), 1000);
          }}
        >
          시험 결과 JSON 저장
        </button>
      </div>
      {world.safetyReason && (
        <output className="fault-message">{world.safetyReason}</output>
      )}
      <p className="compact-note">
        300ms를 넘긴 위치 입력은 지상팀을 정지시킵니다. 비상정지는 초기화 전까지
        유지됩니다. 양보{' '}
        {world.robots.reduce((n, r) => n + r.recoveryAttempts, 0)}회 ·
        경로·팔·적재물을 포함해 충돌 검사.
      </p>
      <p className="compact-note">
        지도 운반은 이상적 기하 모델이며 오차는 관측 좌표와 안전 보류에
        반영됩니다. 실제 오차에 따른 바퀴 제어는 GitHub의 Python 모의 폐루프
        데모에서 따로 시험합니다. 이 화면은 물리 엔진이나 실물 안정성 인증이
        아닙니다.
      </p>
      {world.robots.length === 5 && (
        <p className="compact-note">
          5대는 드론 없는 지정 혼잡 시험입니다. 박쥐 모드로 바꾸면 출발 공간을
          위해 기본 4대로 초기화합니다. 임의 배치·더 많은 로봇의 120초 완주를
          보장하지 않습니다.
        </p>
      )}
    </section>
  );
}
