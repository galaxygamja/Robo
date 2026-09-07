import { searchSchedules, createScheduleReference } from '../lib/scheduling.ts';
const budget = Number(
  process.argv.find((v) => v.startsWith('--budget='))?.split('=')[1] ?? 64,
);
let last = Infinity;
const result = searchSchedules(createScheduleReference(), budget, (r) => {
  if (r.best.time < last || r.tested % 8 === 0) {
    console.error(
      JSON.stringify({
        tested: r.tested,
        seconds: r.best.time,
        score: r.best.score,
      }),
    );
    last = r.best.time;
  }
});
console.log(JSON.stringify(result, null, 2));
