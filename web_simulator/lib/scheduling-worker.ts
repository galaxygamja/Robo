import { createScheduleReference, searchSchedules } from './scheduling.ts';
import { referenceSchedule } from './reference-schedule.ts';
const worker = globalThis as unknown as {
  onmessage: ((event: MessageEvent) => void) | null;
  postMessage: (data: unknown) => void;
};
worker.onmessage = () => {
  try {
    const result = searchSchedules(
      createScheduleReference(),
      48,
      (progress) => worker.postMessage({ progress }),
      referenceSchedule?.schedule,
    );
    worker.postMessage({ complete: result });
  } catch {
    worker.postMessage({
      error:
        '동선 계산을 완료하지 못했습니다. 검증된 새 동선을 실행할 수 있습니다.',
    });
  }
};
