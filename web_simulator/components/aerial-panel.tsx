'use client';
import { useEffect, useRef, useState } from 'react';
import type { World } from '@/lib/mission';
import {
  MODE_LABEL,
  SCENARIO_LABEL,
  type ComparisonMode,
  type ComparisonScenario,
  type ComparisonResult,
} from '@/lib/experiments';
export default function AerialPanel({
  world,
  change,
  compare,
}: {
  world: World;
  change: (f: (world: World) => void) => void;
  compare: (
    mode: ComparisonMode,
    scenario: ComparisonScenario,
    optimize: boolean,
  ) => void;
}) {
  const d = world.drone;
  const [scenario, setScenario] = useState<ComparisonScenario>('intermittent');
  const [results, setResults] = useState<ComparisonResult[]>([]);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const workerRef = useRef<Worker | null>(null);
  useEffect(() => () => workerRef.current?.terminate(), []);
  const cancel = () => {
    workerRef.current?.terminate();
    workerRef.current = null;
    setBusy(false);
  };
  const benchmark = () => {
    cancel();
    setResults([]);
    setError('');
    setBusy(true);
    try {
      const worker = new Worker(
        new URL('../lib/comparison-worker.ts', import.meta.url),
        { type: 'module' },
      );
      workerRef.current = worker;
      worker.onmessage = (event) => {
        if (workerRef.current !== worker) return;
        if (event.data.result)
          setResults((previous) => [...previous, event.data.result]);
        if (event.data.error) setError(event.data.error);
        if (event.data.complete || event.data.error) cancel();
      };
      worker.onerror = () => {
        if (workerRef.current !== worker) return;
        setError(
          '비교 계산을 시작하지 못했습니다. 아래 개별 실행을 이용해 주세요.',
        );
        cancel();
      };
      worker.postMessage({ scenario, optimize: world.coordination.enabled });
    } catch {
      setError('이 브라우저에서는 개별 실행으로 비교해 주세요.');
      cancel();
    }
  };
  const seenObjects =
    d.enabled &&
    d.calibrationValid &&
    !d.videoLost &&
    world.elapsed - d.frameAt <= 0.3
      ? Object.values(d.objects).filter(
          (p) => p.confirmed && world.elapsed - p.at <= 0.3,
        )
      : [];
  return (
    <section className="control-card aerial-panel">
      <div className="position-heading">
        <h2>실행 방식과 드론 활용</h2>
        <span>
          {d.enabled
            ? d.strategy === 'active'
              ? '이동 관측'
              : '중앙 관측'
            : '드론 없음'}
        </span>
      </div>
      <div className="normal-mode-buttons" aria-label="고장 없는 정상 실행">
        <button
          type="button"
          onClick={() => compare('none', 'normal', world.coordination.enabled)}
        >
          드론 없이 정상 실행
        </button>
        <button
          type="button"
          onClick={() =>
            compare('active', 'normal', world.coordination.enabled)
          }
        >
          박쥐와 정상 실행
        </button>
      </div>
      <p className="compact-note">
        선택하면 고장 설정을 지우고 바로 시작합니다. 첫 로봇은 4초에 출발합니다.
      </p>
      <label className="shared-optimizer">
        <input
          type="checkbox"
          checked={world.coordination.enabled}
          onChange={(e) => {
            cancel();
            setResults([]);
            compare(
              d.enabled ? d.strategy : 'none',
              world.scenario,
              e.target.checked,
            );
          }}
        />
        공통 경로·작업 순서 최적화 (변경 시 재시작)
      </label>
      <p className="compact-note">
        속도·안전 간격·최적화 알고리즘은 두 방식에 같습니다. 드론은 추가 위치
        관측을 제공합니다.
      </p>
      <div className="coordination-totals">
        <span>경로 단축 {world.coordination.routeShortcuts}회</span>
        <span>짧은 회전 대기점 {world.coordination.stagingChanges}회</span>
        <span>
          작업 재정렬 {world.robots.reduce((sum, r) => sum + r.taskReorders, 0)}
          회
        </span>
        <span>
          지상 이동{' '}
          {world.robots
            .reduce((sum, r) => sum + r.distanceTravelled, 0)
            .toFixed(2)}
          m
        </span>
      </div>
      <p className="aerial-reason">
        {d.enabled ? d.reason : '기본 위치 입력으로 지상팀 운용'}
      </p>
      {d.enabled && (
        <>
          <div className="aerial-metrics">
            <div>
              <span>유효 태그 관측</span>
              <strong>
                {d.visibleRobots.length}
                <small> / {world.robots.length}</small>
              </strong>
            </div>
            <div>
              <span>기준점</span>
              <strong>
                {d.anchorIds.length}
                <small> / 9</small>
              </strong>
            </div>
            <div>
              <span>드론으로 복구</span>
              <strong>
                {d.recoveredIds.length}
                <small>대</small>
              </strong>
            </div>
          </div>
          <p className={d.calibrationValid ? 'aerial-good' : 'compact-note'}>
            {d.calibrationValid
              ? '현재 시점 기준점 확인'
              : '유효한 기준점 시야 확보 중'}{' '}
            · 높이 {d.altitude.toFixed(2)}m
          </p>
          <p className="compact-note">
            우선 관측: {d.focusIds.join(' · ') || '전체 경기장'} · 지상 속도{' '}
            {Math.round(world.observer.speedScale * 100)}%
          </p>
          <details className="aerial-details">
            <summary>연속 확인한 물체 {seenObjects.length}개</summary>
            <p className="compact-note">
              {seenObjects.map((p) => p.id).join(' · ') ||
                '새 관측 3회를 기다립니다.'}
            </p>
            <p className="compact-note">
              유효한 새 관측은 남은 작업의 위치 정보에 반영합니다. 영상
              확인만으로 집기·해제 성공을 판정하지 않습니다.
            </p>
          </details>
          <label className="aerial-select">
            관측 전략
            <select
              value={d.strategy}
              onChange={(e) =>
                change((w) => {
                  w.drone.strategy = e.target.value as 'active' | 'hover';
                  w.drone.lastPlanAt = -Infinity;
                })
              }
            >
              <option value="active">가림·작업을 따라 위치 선택</option>
              <option value="hover">경기장 중앙에 정지</option>
            </select>
          </label>
          <details className="aerial-details">
            <summary>드론 입력 시험</summary>
            <label>
              <input
                type="checkbox"
                checked={d.videoLost}
                onChange={(e) =>
                  change((w) => {
                    w.drone.videoLost = e.target.checked;
                    w.drone.queue = [];
                  })
                }
              />{' '}
              드론 영상 끊김
            </label>
            <label>
              <input
                type="checkbox"
                checked={d.calibrationLost}
                onChange={(e) =>
                  change((w) => {
                    w.drone.calibrationLost = e.target.checked;
                    w.drone.queue = [];
                  })
                }
              />{' '}
              경기장 기준점 보정 실패
            </label>
            <label className="aerial-select">
              드론 영상 지연
              <select
                value={d.delayMs}
                onChange={(e) =>
                  change((w) => {
                    w.drone.delayMs = Number(e.target.value);
                    w.drone.queue = [];
                  })
                }
              >
                <option value="80">80ms</option>
                <option value="200">200ms</option>
                <option value="450">450ms · 관측 만료</option>
              </select>
            </label>
          </details>
        </>
      )}
      <div className="aerial-challenge">
        <strong>같은 조건 · 세 모드 비교</strong>
        <label className="aerial-select">
          비교할 조건
          <select
            value={scenario}
            onChange={(e) => {
              cancel();
              setResults([]);
              setScenario(e.target.value as ComparisonScenario);
            }}
          >
            {Object.entries(SCENARIO_LABEL).map(([key, label]) => (
              <option key={key} value={key}>
                {label}
              </option>
            ))}
          </select>
        </label>
        <p>
          {scenario === 'normal'
            ? '고장 없이 동일한 지도를 사용합니다. 관측 정보가 같으면 완주 시간도 같을 수 있습니다.'
            : scenario === 'intermittent'
              ? '8초부터 H2의 기본 위치 입력을 10초마다 1.2초씩 끊습니다. 드론 없이도 입력이 돌아오면 계속 운반합니다. 실측값이 아닌 비교용 조건입니다.'
              : 'H2 기본 입력을 계속 끊고 시작 위치 상공에 가림막을 둡니다. 드론 없음이 멈추는 의도적인 복구 시험입니다.'}
        </p>
        <button
          className="benchmark-button"
          type="button"
          onClick={busy ? cancel : benchmark}
        >
          {busy ? '비교 계산 중 · 중단' : '세 모드 결과 한 번에 계산'}
        </button>
        <p className="compact-note">
          현재 경기와 별도로 브라우저에서 계산합니다. 고정된 결과값을 재생하지
          않습니다.
        </p>
        {error && <p role="alert">{error}</p>}
        <div className="comparison-results" aria-live="polite">
          {results.map((result) => (
            <article key={result.mode}>
              <div>
                <strong>{MODE_LABEL[result.mode]}</strong>
                <span>
                  {result.completed
                    ? `${result.remaining.toFixed(2)}초 남음`
                    : '제한시간 종료'}
                </span>
              </div>
              <p>
                {result.score}점 · 위치 대기 {result.inputHold.toFixed(2)}초 ·
                이동 {result.distance.toFixed(2)}m
              </p>
              <small>
                {result.optimized ? '공통 최적화 켬' : '기존 계획'} · 충돌 검출{' '}
                {result.collisions}회 · 최소 간격{' '}
                {result.minimumClearanceMm.toFixed(2)}mm
              </small>
            </article>
          ))}
        </div>
        <p className="compact-note">선택한 조건을 지도에서 직접 보기:</p>
        <div>
          {(
            [
              ['none', '드론 없음'],
              ['hover', '중앙 정지'],
              ['active', '이동 관측'],
            ] as const
          ).map(([mode, label]) => (
            <button
              type="button"
              key={mode}
              onClick={() =>
                compare(mode, scenario, world.coordination.enabled)
              }
            >
              {label}
            </button>
          ))}
        </div>
        <p className="compact-note">
          위치 부족 대기 {d.holdSeconds.toFixed(1)}초 · 드론 복구{' '}
          {d.recoveredRobotSeconds.toFixed(1)} 로봇·초
        </p>
      </div>
      <p className="compact-note">
        청록 영역은 카메라의 바닥 시야입니다. 기준점·카메라·가림막은 가상 시험
        조건이며 실제 드론 영상은 연결되지 않았습니다.
      </p>
    </section>
  );
}
