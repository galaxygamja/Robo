'use client';
import type { World } from '@/lib/mission';
export type DroneComparison = 'none' | 'hover' | 'active';
export default function AerialPanel({
  world,
  change,
  compare,
}: {
  world: World;
  change: (f: (world: World) => void) => void;
  compare: (mode: DroneComparison) => void;
}) {
  const d = world.drone;
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
        <h2>박쥐 · 관측 위치 계획</h2>
        <span>
          {d.enabled
            ? d.strategy === 'active'
              ? '이동 관측'
              : '중앙 관측'
            : '드론 없음'}
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
              최근에 확인한 원기둥을 먼저 고를 수 있습니다. 영상 확인만으로
              집기·해제 성공을 판정하지 않습니다.
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
        <strong>같은 가림 조건으로 비교</strong>
        <p>
          H2의 기본 위치 입력을 끊고, 시작 위치 위에 고정 가림막을 놓습니다.
          선택하면 처음부터 실행합니다.
        </p>
        <div>
          {(
            [
              ['none', '드론 없음'],
              ['hover', '중앙 정지'],
              ['active', '이동 관측'],
            ] as const
          ).map(([mode, label]) => (
            <button type="button" key={mode} onClick={() => compare(mode)}>
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
