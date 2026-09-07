'use client';
import { useEffect, useRef, useState } from 'react';
import type { World } from '@/lib/mission';
import type { ScheduleSearchResult, TeamSchedule } from '@/lib/scheduling';
import { referenceSchedule } from '@/lib/reference-schedule';

export default function SchedulePanel({
  world,
  load,
}: {
  world: World;
  load: (plan: TeamSchedule | null) => void;
}) {
  const [result, setResult] = useState<ScheduleSearchResult | null>(
    referenceSchedule,
  );
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const workerRef = useRef<Worker | null>(null);
  useEffect(() => () => workerRef.current?.terminate(), []);
  const cancel = () => {
    workerRef.current?.terminate();
    workerRef.current = null;
    setBusy(false);
  };
  const search = () => {
    cancel();
    setBusy(true);
    setError('');
    try {
      const worker = new Worker(
        new URL('../lib/scheduling-worker.ts', import.meta.url),
        { type: 'module' },
      );
      workerRef.current = worker;
      worker.onmessage = (event) => {
        if (workerRef.current !== worker) return;
        if (event.data.progress) setResult(event.data.progress);
        if (event.data.error) {
          setError(event.data.error);
          cancel();
        }
        if (event.data.complete) {
          const completed = event.data.complete as ScheduleSearchResult;
          setResult(completed);
          cancel();
          if (
            completed.best.completed &&
            completed.best.score === 160 &&
            completed.best.collisions === 0
          )
            load(completed.schedule);
          else
            setError(
              '모든 작업을 마친 동선을 찾지 못했습니다. 기존 동선을 유지합니다.',
            );
        }
      };
      worker.onerror = () => {
        if (workerRef.current !== worker) return;
        setError('계산을 시작하지 못했습니다. 검증된 새 동선을 실행해 주세요.');
        cancel();
      };
      worker.postMessage({ search: true });
    } catch {
      setError('이 브라우저에서 추가 탐색을 시작하지 못했습니다.');
      cancel();
    }
  };
  return (
    <section className="control-card schedule-panel">
      <h2>가장 늦게 끝나는 로봇까지 빠르게</h2>
      <p className="compact-note">
        햄스터 1대·비버 3대, 정상 위치 입력 기준. 작업 배분·출발 시각·색별
        원기둥 선택을 함께 비교합니다.
      </p>
      {result && (
        <>
          <div className="schedule-score" aria-live="polite">
            <span>
              기존 동선 <b>{result.baseline.time.toFixed(2)}초</b>
            </span>
            <span>
              찾은 동선 <b>{result.best.time.toFixed(2)}초</b>
            </span>
            <span>
              단축{' '}
              <b>{(result.baseline.time - result.best.time).toFixed(2)}초</b>
            </span>
          </div>
          <p className="compact-note">
            {result.best.score}/160점 · 충돌 {result.best.collisions}회 · 최소
            로봇 간격 {result.best.minimumClearanceMm.toFixed(2)}mm · 이동{' '}
            {result.best.distance.toFixed(2)}m
          </p>
        </>
      )}
      <div className="normal-mode-buttons">
        <button
          type="button"
          disabled={busy}
          onClick={() => {
            cancel();
            load(null);
          }}
        >
          기존 동선 실행
        </button>
        <button
          type="button"
          disabled={busy || !result}
          onClick={() => {
            if (result) {
              cancel();
              load(result.schedule);
            }
          }}
        >
          새 동선 실행
        </button>
      </div>
      <button
        className="benchmark-button"
        type="button"
        onClick={busy ? cancel : search}
      >
        {busy
          ? `후보 ${result?.tested ?? 0}/${result?.budget ?? 48} 비교 중 · 중단`
          : '동선 추가 탐색 후 실행'}
      </button>
      {error && <p role="alert">{error}</p>}
      <p className="compact-note">
        후보별로 집기·해제·회전·주차까지 다시 계산합니다. 표시값은 정상 조건의
        모의 측정이며, 모든 가능한 동선 중 최단이라는 보장은 아닙니다.
      </p>
      <details>
        <summary>현재 로봇별 출발·작업 순서</summary>
        <div className="schedule-orders">
          {world.robots.map((r) => (
            <p key={r.id}>
              <b>{r.name}</b> · {r.delay.toFixed(1)}초 출발
              <br />
              {r.jobs.map((j) => j.itemId).join(' → ')}
            </p>
          ))}
        </div>
      </details>
    </section>
  );
}
