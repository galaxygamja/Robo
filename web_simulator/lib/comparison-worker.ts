import { runComparison, type ComparisonScenario } from './experiments.ts';

const worker = globalThis as unknown as {
  onmessage: ((event: MessageEvent) => void) | null;
  postMessage: (message: unknown) => void;
};
worker.onmessage = (event) => {
  const { scenario, optimize } = event.data ?? {};
  if (
    !['normal', 'intermittent', 'occlusion'].includes(scenario) ||
    typeof optimize !== 'boolean'
  ) {
    worker.postMessage({ error: '비교 조건이 올바르지 않습니다.' });
    return;
  }
  try {
    for (const mode of ['none', 'hover', 'active'] as const) {
      worker.postMessage({
        result: runComparison(mode, scenario as ComparisonScenario, optimize),
      });
    }
    worker.postMessage({ complete: true });
  } catch {
    worker.postMessage({
      error: '비교 계산에 실패했습니다. 다시 실행해 주세요.',
    });
  }
};
