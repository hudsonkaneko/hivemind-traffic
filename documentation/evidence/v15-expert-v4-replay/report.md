# Expert v4 offline replay: open-loop kinematic projection on a stream reconstructed from saved SUMO records; neither SUMO nor physics; post hoc

Source: outputs/highway-merge-v15/phase-2/run-03 (rule:expert-v3). Situations of the 31 ramp cars (decision 15): clear 7, stream 12, blocked 5, i110 7. Only the stream and clear cars are replayed.

Movement checks are to the stop line, not merge completion, and post hoc (reported, not gates). Open loop: the mainline cars do not react, so a rear response is the braking it would need.

## Fidelity (part A: choices at v3's recorded states)

| Situation | Cars | Decisions | v3 replay = record | v4 differs from v3 |
|---|---|---|---|---|
| stream | 12 | 2495 | 2436 | 1890 |
| clear | 7 | 232 | 232 | 113 |

## Movement and crossings (part B: projection to the stop line)

| Situation | Driver | Crossed | M1 | M2 | M4 | Fail any | M3 flags / applies | Median crossing (s) | Median crossing speed (m/s) | Median lowest speed (m/s) | Median time below 5 m/s (s) | Undone commits | Cars with a dilemma commit |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| stream | v3 recorded (SUMO) | 12 | 10 | 8 | 3 | 11 | 8 / 8 | 19 | 4.46 | 0.69 | 14.1 | - | - |
| stream | v3 projected | 12 | 10 | 8 | 2 | 11 | 8 / 8 | 19 | 4.31 | 0.65 | 14.1 | 13 | 0 |
| stream | v4 projected | 12 | 9 | 8 | 3 | 12 | 8 / 8 | 19 | 4.01 | 0.25 | 13.9 | 10 | 0 |
| stream | v4 as written (changes 1-3), projected | 12 | 8 | 7 | 3 | 10 | 8 / 8 | 19 | 5.11 | 0.67 | 14 | 9 | 2 |
| clear | v3 recorded (SUMO) | 7 | 0 | 0 | 0 | 0 | 7 / 7 | 3.3 | 15.51 | 12.24 | 0 | - | - |
| clear | v3 projected | 7 | 0 | 0 | 0 | 0 | 7 / 7 | 3.3 | 15.51 | 12.24 | 0 | 0 | 0 |
| clear | v4 projected | 7 | 0 | 0 | 0 | 0 | 7 / 7 | 3.3 | 15.6 | 12.12 | 0 | 0 | 0 |
| clear | v4 as written (changes 1-3), projected | 7 | 0 | 0 | 0 | 0 | 7 / 7 | 3.3 | 15.54 | 12.24 | 0 | 0 | 0 |

## Each stream car

Lowest speed, time below 5 m/s, hardest braking, crossing time from the first active-entry decision and crossing speed; flags 1-4 = M1-M4 failed; rear = the braking the next mainline car would need at the crossing (m/s^2, open loop; v3's gap test allows 2.0).

| Row | Car | Driver | Lowest (m/s) | Below 5 m/s (s) | Hardest braking (m/s^2) | Crossing (s) | Crossing speed (m/s) | Flags | Rear response (m/s^2) |
|---|---|---|---|---|---|---|---|---|---|
| 8 | agent_0 | v3 recorded (SUMO) | 0.37 | 13.4 | -3 | 23.9 | 6.2 | - | - |
| 8 | agent_0 | v3 projected | 0.37 | 13.5 | -3 | 23.9 | 6.04 | - | - |
| 8 | agent_0 | v4 projected | 0.31 | 14.2 | -1.72 | 23.7 | 5.37 | 4 | - |
| 8 | agent_0 | v4 as written (changes 1-3), projected | 0.31 | 14.6 | -2.7 | 23.7 | 5.37 | 4 | - |
| 8 | agent_2 | v3 recorded (SUMO) | 1.23 | 13.4 | -3 | 27.1 | 7.78 | 4 | - |
| 8 | agent_2 | v3 projected | 1.23 | 13.5 | -3 | 27.2 | 7.91 | 4 | - |
| 8 | agent_2 | v4 projected | 1.03 | 13.7 | -1.92 | 26.9 | 7.34 | 4 | - |
| 8 | agent_2 | v4 as written (changes 1-3), projected | 1.02 | 13.8 | -2.68 | 26.9 | 7.34 | 14 | - |
| 8 | agent_6 | v3 recorded (SUMO) | 0.7 | 10.7 | -3 | 19.8 | 2.46 | 123 | - |
| 8 | agent_6 | v3 projected | 0.66 | 10.8 | -3 | 19.9 | 2.43 | 123 | - |
| 8 | agent_6 | v4 projected | 0.11 | 12.7 | -3 | 19.8 | 1.87 | 1234 | - |
| 8 | agent_6 | v4 as written (changes 1-3), projected | 0.15 | 12.7 | -3 | 19.8 | 1.9 | 1234 | - |
| 9 | agent_0 | v3 recorded (SUMO) | 2.01 | 21.4 | -3 | 28.2 | 7.04 | 124 | - |
| 9 | agent_0 | v3 projected | 1.91 | 21.3 | -3.03 | 28.2 | 7.04 | 124 | - |
| 9 | agent_0 | v4 projected | 1.56 | 21.4 | -3.02 | 27.9 | 6.75 | 12 | - |
| 9 | agent_0 | v4 as written (changes 1-3), projected | 1.57 | 21.4 | -3.02 | 27.9 | 6.75 | 12 | - |
| 9 | agent_2 | v3 recorded (SUMO) | 2.52 | 18.7 | -2.76 | 31.2 | 8.16 | 14 | - |
| 9 | agent_2 | v3 projected | 2.52 | 18.1 | -3.3 | 31.2 | 8.16 | 1 | - |
| 9 | agent_2 | v4 projected | 2.05 | 16.2 | -3.3 | 30.9 | 7.9 | 1 | - |
| 9 | agent_2 | v4 as written (changes 1-3), projected | 2.05 | 16.2 | -3.3 | 30.9 | 7.9 | 1 | - |
| 22 | agent_0 | v3 recorded (SUMO) | 4.49 | 10.9 | -3 | 16.6 | 8.91 | 123 | - |
| 22 | agent_0 | v3 projected | 4.49 | 10.9 | -3 | 16.6 | 8.91 | 123 | - |
| 22 | agent_0 | v4 projected | 2.67 | 9.7 | -2.09 | 16.9 | 4.85 | 23 | - |
| 22 | agent_0 | v4 as written (changes 1-3), projected | 2.67 | 9.7 | -2.09 | 16.9 | 4.85 | 23 | - |
| 24 | agent_0 | v3 recorded (SUMO) | 5.24 | 0 | -3 | 11.8 | 15.91 | 13 | - |
| 24 | agent_0 | v3 projected | 5.24 | 0 | -3 | 11.8 | 15.91 | 13 | -1.46 |
| 24 | agent_0 | v4 projected | 6.28 | 0 | -3 | 11.2 | 14.41 | 13 | -1.16 |
| 24 | agent_0 | v4 as written (changes 1-3), projected | 13.15 | 0 | -3.08 | 7.8 | 17.07 | 3 | -3.773 |
| 37 | agent_0 | v3 recorded (SUMO) | 0.12 | 13.6 | -4.5 | 17.4 | 2.72 | 123 | - |
| 37 | agent_0 | v3 projected | 0.13 | 13.5 | -4.5 | 17.3 | 2.59 | 123 | - |
| 37 | agent_0 | v4 projected | 0.19 | 13.2 | -3 | 17.2 | 3.17 | 123 | - |
| 37 | agent_0 | v4 as written (changes 1-3), projected | 13.24 | 0 | 0 | 3.4 | 16.57 | 3 | -7.99 |
| 38 | agent_0 | v3 recorded (SUMO) | 0.56 | 15.9 | -3 | 19.1 | 2.36 | 123 | - |
| 38 | agent_0 | v3 projected | 0.53 | 16 | -3 | 19.2 | 2.34 | 123 | - |
| 38 | agent_0 | v4 projected | 0.09 | 15.1 | -3 | 19.1 | 1.86 | 123 | - |
| 38 | agent_0 | v4 as written (changes 1-3), projected | 0.09 | 15.1 | -3 | 19.1 | 1.86 | 123 | - |
| 39 | agent_0 | v3 recorded (SUMO) | 0.67 | 15 | -3 | 18.2 | 2.21 | 123 | - |
| 39 | agent_0 | v3 projected | 0.63 | 15.1 | -3 | 18.3 | 2.18 | 123 | - |
| 39 | agent_0 | v4 projected | 0.1 | 14.2 | -3 | 18.2 | 1.64 | 123 | - |
| 39 | agent_0 | v4 as written (changes 1-3), projected | 0.1 | 14.2 | -3 | 18.2 | 1.64 | 123 | - |
| 53 | agent_0 | v3 recorded (SUMO) | 0.63 | 14.5 | -3 | 17.4 | 2.18 | 123 | - |
| 53 | agent_0 | v3 projected | 0.6 | 14.6 | -3 | 17.5 | 2.14 | 123 | - |
| 53 | agent_0 | v4 projected | 0.11 | 13.7 | -3 | 17.4 | 1.65 | 123 | - |
| 53 | agent_0 | v4 as written (changes 1-3), projected | 0.11 | 13.7 | -3 | 17.4 | 1.65 | 123 | - |
| 67 | agent_0 | v3 recorded (SUMO) | 0.54 | 16.2 | -3 | 18.8 | 2.1 | 123 | - |
| 67 | agent_0 | v3 projected | 0.51 | 16.3 | -3 | 18.9 | 2.07 | 123 | - |
| 67 | agent_0 | v4 projected | 0.1 | 15.4 | -3 | 18.8 | 1.64 | 123 | - |
| 67 | agent_0 | v4 as written (changes 1-3), projected | 0.1 | 15.4 | -3 | 18.8 | 1.64 | 123 | - |

## Speed every second (m/s), stream cars

- **Row 8, agent_0**
  - v3 projected: 12.1, 12.6, 13.1, 11.8, 10.1, 9.2, 8.5, 7.9, 7.5, 6.7, 4.2, 3.1, 2.9, 2.6, 2.2, 1.7, 1.8, 1.6, 0.5, 0.8, 1.4, 2.5, 3.6, 4.8
  - v4 projected: 12.1, 12.9, 13.8, 14.2, 13, 11.5, 9.9, 8.2, 6.5, 5.3, 4, 3, 2.3, 1.8, 1.5, 1.3, 1.1, 0.8, 0.4, 0.6, 1.1, 1.9, 3, 4.4
  - v4 as written (changes 1-3), projected: 12.1, 12.9, 13.8, 14.4, 13.8, 12, 10.4, 8.4, 6.7, 4.6, 3.1, 2.5, 2, 1.7, 1.4, 1.2, 1, 0.8, 0.4, 0.6, 1.1, 1.9, 3, 4.4
- **Row 8, agent_2**
  - v3 projected: 12.1, 11.7, 12, 12.9, 13.1, 12.7, 11.8, 10.7, 9.8, 8.7, 6.2, 5.3, 4.8, 4.2, 3.6, 2.9, 2.7, 2.4, 1.4, 1.4, 1.6, 1.6, 1.5, 2.6, 3.7, 5, 6.3, 7.6
  - v4 projected: 12.1, 11.7, 12, 13, 13.7, 13.7, 13.6, 12.3, 10.4, 8.8, 7.1, 5.7, 4.6, 3.8, 3.1, 2.5, 2.1, 1.7, 1.2, 1.1, 1.3, 1.5, 1.5, 2.4, 3.4, 4.7, 6.1
  - v4 as written (changes 1-3), projected: 12.1, 11.7, 12, 13, 13.7, 13.7, 13.7, 12.7, 10.9, 8.7, 6.7, 5.4, 4.5, 3.7, 3, 2.5, 2.1, 1.7, 1.2, 1.1, 1.3, 1.5, 1.5, 2.4, 3.4, 4.7, 6.1
- **Row 8, agent_6**
  - v3 projected: 12.1, 12.1, 10.8, 7.8, 6.3, 6.1, 6.1, 6.1, 6.1, 5.2, 3.6, 1.6, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 0.9, 0.9
  - v4 projected: 12.1, 13, 13.7, 11.9, 9.5, 8, 6.5, 5, 3.7, 2.8, 1.7, 1.1, 0.7, 0.6, 0.5, 0.5, 0.5, 0.3, 0.2, 0.5
  - v4 as written (changes 1-3), projected: 12.1, 13, 14, 13.4, 10.6, 8.3, 6.8, 5, 3.5, 1.4, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.4, 0.2, 0.5
- **Row 9, agent_0**
  - v3 projected: 12.1, 12.6, 12.1, 10.5, 7.9, 5.1, 4.2, 4.2, 4, 3.5, 3, 2.6, 2.2, 2, 2.1, 2.4, 2.9, 3.4, 3.5, 3.1, 2.7, 2.9, 2.7, 2.5, 3, 3.8, 4.7, 5.7, 6.8
  - v4 projected: 12.1, 12.6, 12.6, 10.4, 7.9, 5.2, 4.3, 4.3, 4.3, 4.3, 4.3, 4.2, 3.9, 3.5, 3.2, 2.9, 2.7, 2.5, 2.2, 1.8, 1.6, 1.8, 2, 2.2, 2.7, 3.5, 4.5, 5.7
  - v4 as written (changes 1-3), projected: 12.1, 12.6, 12.6, 10.4, 7.9, 5.2, 4.3, 4.3, 4.3, 4.3, 4.3, 4.2, 3.9, 3.5, 3.2, 2.9, 2.7, 2.5, 2.2, 1.8, 1.6, 1.8, 2, 2.2, 2.7, 3.5, 4.5, 5.7
- **Row 9, agent_2**
  - v3 projected: 12.1, 12.1, 12.1, 12.1, 11.3, 8.8, 8.1, 7.1, 6.6, 5.8, 4.9, 4.2, 3.6, 3.1, 2.9, 2.9, 3, 3.1, 3.3, 3.3, 3, 3, 2.9, 2.7, 2.5, 2.7, 3.4, 4.1, 4.9, 5.8, 6.9, 7.9
  - v4 projected: 12.1, 12.1, 12.1, 12.1, 11.3, 8.8, 8.2, 7.2, 6.7, 6.4, 6, 5.6, 5.1, 4.5, 4.1, 3.7, 3.4, 3.1, 2.7, 2.4, 2.1, 2.1, 2.1, 2.2, 2.3, 2.5, 3, 3.7, 4.6, 5.7, 6.8
  - v4 as written (changes 1-3), projected: 12.1, 12.1, 12.1, 12.1, 11.3, 8.8, 8.2, 7.2, 6.7, 6.4, 6, 5.6, 5.1, 4.5, 4.1, 3.7, 3.3, 3.1, 2.7, 2.4, 2.1, 2.1, 2.1, 2.2, 2.3, 2.5, 3, 3.7, 4.6, 5.7, 6.8
- **Row 22, agent_0**
  - v3 projected: 13.2, 11.5, 8.5, 5.6, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.8, 6.4, 8
  - v4 projected: 13.2, 12, 10.3, 8.8, 7.8, 6.9, 6, 5.2, 4.5, 4, 3.7, 3.6, 3.5, 3.5, 3.5, 2.8, 3.3
  - v4 as written (changes 1-3), projected: 13.2, 12, 10.3, 8.8, 7.8, 6.9, 6, 5.2, 4.4, 3.9, 3.7, 3.6, 3.6, 3.5, 3.5, 2.8, 3.3
- **Row 24, agent_0**
  - v3 projected: 13.2, 11.5, 9, 6.3, 5.2, 5.9, 7.2, 8.7, 10.2, 11.7, 13.2, 14.7
  - v4 projected: 13.2, 12.3, 11.1, 10.8, 10.2, 7.4, 6.3, 7.6, 9.1, 10.6, 12.3, 14.1
  - v4 as written (changes 1-3), projected: 13.2, 13.2, 13.2, 13.2, 13.6, 15.1, 16.5, 17.5
- **Row 37, agent_0**
  - v3 projected: 13.2, 13.6, 12.2, 8.1, 4, 1.2, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.9, 2.2
  - v4 projected: 13.2, 11.6, 8.6, 6.5, 5, 3.3, 2.3, 1.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.4, 1.6, 2.9
  - v4 as written (changes 1-3), projected: 13.2, 14.3, 15, 16.2
- **Row 38, agent_0**
  - v3 projected: 13.2, 11.6, 8.6, 5.6, 2.6, 1.2, 1.2, 1.1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.8, 0.6, 2
  - v4 projected: 13.2, 11.6, 8.6, 6.4, 4.9, 3.7, 2.5, 0.7, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.3, 0.2, 0.2, 1.7
  - v4 as written (changes 1-3), projected: 13.2, 11.6, 8.6, 6.4, 4.9, 3.7, 2.5, 0.7, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.3, 0.2, 0.2, 1.7
- **Row 39, agent_0**
  - v3 projected: 13.2, 11.6, 8.6, 5.6, 2.6, 1.3, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 0.9, 0.7, 1.8
  - v4 projected: 13.2, 11.6, 8.6, 6.5, 5, 3.6, 2.1, 1.1, 0.7, 0.6, 0.5, 0.5, 0.5, 0.5, 0.5, 0.4, 0.2, 0.2, 1.4
  - v4 as written (changes 1-3), projected: 13.2, 11.6, 8.6, 6.5, 5, 3.6, 2.1, 1.1, 0.7, 0.6, 0.5, 0.5, 0.5, 0.5, 0.5, 0.4, 0.2, 0.2, 1.4
- **Row 53, agent_0**
  - v3 projected: 12.2, 10.6, 7.6, 4.6, 2, 1.2, 1.2, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1, 0.6, 1.5
  - v4 projected: 12.2, 10.6, 7.7, 6, 4.4, 2.9, 1.7, 0.9, 0.6, 0.5, 0.5, 0.5, 0.5, 0.5, 0.4, 0.2, 0.1, 1.1
  - v4 as written (changes 1-3), projected: 12.2, 10.6, 7.7, 6, 4.4, 2.9, 1.7, 0.9, 0.6, 0.5, 0.5, 0.5, 0.5, 0.5, 0.4, 0.2, 0.1, 1.1
- **Row 67, agent_0**
  - v3 projected: 11.4, 9.7, 6.7, 3.7, 1.5, 1.4, 1.2, 1.1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.6, 0.9
  - v4 projected: 11.4, 9.7, 7, 5.5, 3.3, 2.1, 2.1, 1.1, 0.7, 0.6, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.3, 0.1, 0.6
  - v4 as written (changes 1-3), projected: 11.4, 9.7, 7, 5.5, 3.3, 2.1, 2.1, 1.1, 0.7, 0.6, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.3, 0.1, 0.6
