import { runComparison, type ComparisonScenario } from '../lib/experiments.ts';

const scenario =
  process.argv.find((arg) => arg.startsWith('--scenario='))?.split('=')[1] ??
  'intermittent';
if (!['normal', 'intermittent', 'occlusion'].includes(scenario))
  throw new Error('Use --scenario=normal|intermittent|occlusion');
for (const mode of ['none', 'hover', 'active'] as const) {
  console.log(
    JSON.stringify(
      runComparison(
        mode,
        scenario as ComparisonScenario,
        !process.argv.includes('--legacy'),
      ),
    ),
  );
}
