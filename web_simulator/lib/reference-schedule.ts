import type { ScheduleSearchResult } from './scheduling.ts';
// Generated for the 1/2/1 medical-kit assignment, then replayed in the exact engine.
export const referenceSchedule: ScheduleSearchResult | null = {
  signature:
    '{"model":"mission-makespan-v1","field":{"width":1.143,"height":1.181,"duration":120,"startZone":{"x":0.663,"y":0,"width":0.48,"height":0.28},"safetyMargin":0.015},"spec":{"bodyWidth":0.126,"bodyLength":0.1,"speed":0.22,"margin":0.012,"armExtended":0.11,"armRetracted":0.072,"droneWidth":0.15,"droneLength":0.15,"sensorTimeout":1.5,"dt":0.02},"footprint":[{"x":-0.05,"y":-0.05},{"x":0.05,"y":-0.05},{"x":0.063,"y":-0.0445},{"x":0.063,"y":0.0445},{"x":0.05,"y":0.05},{"x":-0.05,"y":0.05},{"x":-0.063,"y":0.0445},{"x":-0.063,"y":-0.0445}],"items":[{"id":"D1","kind":"disc","x":0.08,"y":0.2,"selected":true,"carrier":null,"released":false},{"id":"D2","kind":"disc","x":0.15,"y":0.2,"selected":true,"carrier":null,"released":false},{"id":"D3","kind":"disc","x":0.22,"y":0.2,"selected":true,"carrier":null,"released":false},{"id":"Y1","kind":"cylinder","x":0.15,"y":0.731,"color":"yellow","selected":true,"carrier":null,"released":false},{"id":"G1","kind":"cylinder","x":0.25,"y":0.731,"color":"green","selected":true,"carrier":null,"released":false},{"id":"R1","kind":"cylinder","x":0.35,"y":0.731,"color":"red","selected":true,"carrier":null,"released":false},{"id":"R2","kind":"cylinder","x":0.15,"y":0.531,"color":"red","selected":true,"carrier":null,"released":false},{"id":"G2","kind":"cylinder","x":0.25,"y":0.531,"color":"green","selected":true,"carrier":null,"released":false},{"id":"Y2","kind":"cylinder","x":0.35,"y":0.531,"color":"yellow","selected":true,"carrier":null,"released":false},{"id":"R3","kind":"cylinder","x":0.793,"y":0.731,"color":"red","selected":true,"carrier":null,"released":false},{"id":"G3","kind":"cylinder","x":0.893,"y":0.731,"color":"green","selected":true,"carrier":null,"released":false},{"id":"Y3","kind":"cylinder","x":0.993,"y":0.731,"color":"yellow","selected":true,"carrier":null,"released":false},{"id":"Y4","kind":"cylinder","x":0.793,"y":0.531,"color":"yellow","selected":false,"carrier":null,"released":false},{"id":"G4","kind":"cylinder","x":0.893,"y":0.531,"color":"green","selected":false,"carrier":null,"released":false},{"id":"R4","kind":"cylinder","x":0.993,"y":0.531,"color":"red","selected":false,"carrier":null,"released":false},{"id":"C2","kind":"cube","x":0.9,"y":0.21,"heading":0,"selected":true,"carrier":"B1","released":false},{"id":"C1","kind":"cube","x":0.9,"y":0.065,"heading":0,"selected":true,"carrier":"B2","released":false},{"id":"C3","kind":"cube","x":0.9,"y":0.065,"heading":0,"selected":true,"carrier":"B2","released":false},{"id":"C4","kind":"cube","x":1.055,"y":0.065,"heading":0,"selected":true,"carrier":"H2","released":false}],"scenario":"normal","observations":{"delay":0,"noise":0,"lost":false,"missing":null},"robots":[{"id":"B1","role":"beaver","pose":{"x":0.9,"y":0.21,"heading":0},"delay":4,"jobs":[{"itemId":"C2","destination":"PCC-L","drop":{"x":0.18,"y":1.135}},{"itemId":"R1","destination":"H","drop":{"x":0.45,"y":1.045}},{"itemId":"Y1","destination":"PCC-L","drop":{"x":0.12,"y":1.045}},{"itemId":"G1","destination":"RZ","drop":{"x":0.75,"y":0.055}}],"magazine":["C2"],"staging":{"x":0.46,"y":0.6},"park":{"x":0.12,"y":0.88}},{"id":"H1","role":"hamster","pose":{"x":1.055,"y":0.21,"heading":3.141592653589793},"delay":7,"jobs":[{"itemId":"D1","destination":"LAB","drop":{"x":0.38,"y":0.075},"slot":0},{"itemId":"D2","destination":"LAB","drop":{"x":0.48,"y":0.075},"slot":1},{"itemId":"D3","destination":"LAB","drop":{"x":0.58,"y":0.075},"slot":2}],"magazine":[],"staging":{"x":0.6,"y":0.34},"park":{"x":0.6,"y":0.34}},{"id":"B2","role":"beaver","pose":{"x":0.9,"y":0.065,"heading":0},"delay":10,"jobs":[{"itemId":"C1","destination":"H","drop":{"x":0.48,"y":1.135}},{"itemId":"C3","destination":"H","drop":{"x":0.66,"y":1.135}},{"itemId":"R3","destination":"H","drop":{"x":0.65,"y":1.045}},{"itemId":"R2","destination":"H","drop":{"x":0.55,"y":1.045}},{"itemId":"G3","destination":"RZ","drop":{"x":0.88,"y":0.055}}],"magazine":["C1","C3"],"staging":{"x":0.68,"y":0.61},"park":{"x":0.57,"y":0.87}},{"id":"H2","role":"beaver","pose":{"x":1.055,"y":0.065,"heading":0},"delay":13,"jobs":[{"itemId":"C4","destination":"PCC-R","drop":{"x":0.96,"y":1.135}},{"itemId":"Y3","destination":"PCC-R","drop":{"x":0.96,"y":1.045}},{"itemId":"Y2","destination":"PCC-R","drop":{"x":1.04,"y":1.045}},{"itemId":"G2","destination":"RZ","drop":{"x":1.01,"y":0.055}}],"magazine":["C4"],"staging":{"x":0.4,"y":0.35},"park":{"x":1.02,"y":0.88}}]}',
  schedule: {
    fixedOrder: true,
    selection: {
      G3: 'G4',
      R2: 'R4',
    },
    robots: [
      {
        id: 'B1',
        delay: 0,
        jobs: ['C2', 'G1', 'R1', 'G2'],
      },
      {
        id: 'H1',
        delay: 0,
        jobs: ['D1', 'D2', 'D3'],
      },
      {
        id: 'B2',
        delay: 2,
        jobs: ['C1', 'C3', 'R3', 'G3', 'R2'],
      },
      {
        id: 'H2',
        delay: 6,
        jobs: ['C4', 'Y3', 'Y1', 'Y2'],
      },
    ],
  },
  baseline: {
    completed: true,
    time: 69.66000000000328,
    score: 160,
    distance: 20.137940320101848,
    trafficWait: 14.059999999999913,
    minimumClearanceMm: 12.083443786701045,
    collisions: 0,
    robots: [
      {
        id: 'B1',
        completedAt: 43.06000000000113,
        distance: 3.96512912729815,
        wait: 1.7600000000000011,
      },
      {
        id: 'H1',
        completedAt: 36.22000000000006,
        distance: 3.9457532123517796,
        wait: 0,
      },
      {
        id: 'B2',
        completedAt: 61.74000000000405,
        distance: 5.558289228461819,
        wait: 5.399999999999973,
      },
      {
        id: 'H2',
        completedAt: 69.66000000000328,
        distance: 6.668768751990101,
        wait: 6.899999999999941,
      },
    ],
  },
  best: {
    completed: true,
    time: 45.18000000000146,
    score: 160,
    distance: 18.508680083049224,
    trafficWait: 0.7600000000000003,
    minimumClearanceMm: 13.78649090800981,
    collisions: 0,
    robots: [
      {
        id: 'B1',
        completedAt: 45.14000000000146,
        distance: 6.031302910129428,
        wait: 0,
      },
      {
        id: 'H1',
        completedAt: 29.619999999999457,
        distance: 4.032901418524452,
        wait: 0,
      },
      {
        id: 'B2',
        completedAt: 42.30000000000101,
        distance: 4.078379181840325,
        wait: 0.7600000000000003,
      },
      {
        id: 'H2',
        completedAt: 45.18000000000146,
        distance: 4.36609657255502,
        wait: 0,
      },
    ],
  },
  tested: 64,
  budget: 64,
};
