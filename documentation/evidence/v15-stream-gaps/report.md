# The stream merges: who drives the stream, whether a gap exists, how open the loop is

Label: offline reading of saved SUMO records (expert v3: Phase 2 run 3; round 16: Phase 1 run 1) and of the stream the offline replay reconstructs from them; nothing simulated; post hoc.

Times are seconds from the car's first decision at the entry. "Best case": v3's gap test at the earliest crossing each gap allows and the highest speed the car could have at the line.

## Who drives the stream

| Row | Our controlled cars | Stream cars (passing order) | Ours in the stream |
|---|---|---|---|
| 8 | 8 | agent_1 (ours), main_8 (SUMO), main_5 (SUMO), main_2 (SUMO) | 1 of 4 |
| 9 | 8 | agent_1 (ours), main_8 (SUMO), main_5 (SUMO), main_2 (SUMO) | 1 of 4 |
| 22 | 8 | main_11 (SUMO), agent_5 (ours), agent_1 (ours), main_2 (SUMO) | 2 of 4 |
| 24 | 1 | main_11 (SUMO), main_8 (SUMO), main_5 (SUMO) | 0 of 3 |
| 37 | 8 | agent_5 (ours), main_10 (SUMO), main_7 (SUMO), agent_4 (ours), agent_7 (ours) | 3 of 5 |
| 38 | 1 | main_13 (SUMO), main_10 (SUMO), main_7 (SUMO), main_4 (SUMO), main_1 (SUMO) | 0 of 5 |
| 39 | 1 | main_13 (SUMO), main_10 (SUMO), main_7 (SUMO), main_4 (SUMO), main_1 (SUMO) | 0 of 5 |
| 53 | 1 | main_12 (SUMO), main_9 (SUMO), main_6 (SUMO), main_3 (SUMO), main_0 (SUMO) | 0 of 5 |
| 67 | 1 | main_14 (SUMO), main_11 (SUMO), main_8 (SUMO), main_5 (SUMO), main_2 (SUMO) | 0 of 5 |

## Is there a gap? (v3's test, best case)

| Row | Car | Start speed / distance | Best line speed (v3 run-up / curve cap) | Stream: passing s (speed) | First gap at 2.0, v3 run-up | at 2.0, curve cap | at 3.0, curve cap | Car ahead on the ramp (crossed, v3 record) | at 2.0, curve cap, after it | Least time below 5 m/s before it | v3 crossed (record) | v4 (projection) | Round 16 crossed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 8 | agent_0 | 12.1 m/s, 129 m | 19.8 / 18.6 | 6.4 (21), 10.6 (23), 15.5 (23), 18.4 (22) | 7.5 | 7.5 | 7.5 | agent_6, ours (19.8) | 20.8 (after the last car) | 0.0 s | 23.9 | 23.7 | 8.3 |
| 8 | agent_2 | 12.1 m/s, 171 m | 19.8 / 18.6 | 6.4 (21), 10.6 (23), 15.5 (23), 18.4 (22) | 11.6 | 11.6 | 11.6 | agent_0, ours (23.9) | 24.9 (after the last car) | 0.0 s | 27.1 | 26.9 | 10.6 |
| 8 | agent_6 | 12.1 m/s, 87 m | 19.8 / 18.6 | 6.4 (21), 10.6 (23), 15.5 (23), 18.4 (22) | 7.4 | 7.4 | 7.4 | ramp_0, SUMO (3.1) | 7.4 | 0.0 s | 19.8 | 19.8 | 5.8 |
| 9 | agent_0 | 12.1 m/s, 129 m | 19.8 / 19.8 | 11.6 (9), 14.0 (11), 16.4 (13), 19.1 (15) | 7.5 | 7.5 | 7.5 | agent_6, ours (25.0) | 26.0 (after the last car) | 0.2 s | 28.2 | 27.9 | 11.3 |
| 9 | agent_2 | 12.1 m/s, 171 m | 19.8 / 19.8 | 11.6 (9), 14.0 (11), 16.4 (13), 19.1 (15) | 9.6 | 9.6 | 9.6 | agent_0, ours (28.2) | 29.2 (after the last car) | 0.0 s | 31.2 | 30.9 | 13.8 |
| 22 | agent_0 | 13.2 m/s, 97 m | 19.8 / 18.6 | 7.1 (23), 9.5 (24), 12.0 (25), 15.6 (25) | 16.5 (after the last car) | 16.5 (after the last car) | 16.5 (after the last car) | ramp_0, SUMO (3.2) | 16.5 (after the last car) | 0.0 s | 16.6 | 16.9 | 5.9 |
| 24 | agent_0 | 13.2 m/s, 114 m | 19.8 / 18.6 | 6.6 (22), 10.0 (22), 14.8 (20) | 11.0 | 11.0 | 7.6 | ramp_0, SUMO (3.2) | 11.0 | 0.0 s | 11.8 | 11.2 | 6.7 |
| 37 | agent_0 | 13.2 m/s, 50 m | 18.0 / 18.0 | 5.2 (27), 8.2 (25), 11.1 (23), 13.6 (24), 15.9 (25) | 16.9 (after the last car) | 16.9 (after the last car) | 16.9 (after the last car) | none | — | 7.0 s | 17.4 | 17.2 | 3.4 |
| 38 | agent_0 | 13.2 m/s, 50 m | 18.0 / 18.0 | 4.9 (24), 8.3 (23), 11.3 (23), 14.6 (23), 17.7 (23) | 18.7 (after the last car) | 18.7 (after the last car) | 18.7 (after the last car) | none | — | 8.7 s | 19.1 | 19.1 | 3.3 |
| 39 | agent_0 | 13.2 m/s, 50 m | 18.0 / 18.0 | 4.0 (24), 7.3 (23), 10.4 (23), 13.8 (23), 16.7 (23) | 17.7 (after the last car) | 17.7 (after the last car) | 17.7 (after the last car) | none | — | 7.7 s | 18.2 | 18.2 | 3.3 |
| 53 | agent_0 | 12.2 m/s, 45 m | 16.9 / 16.9 | -2.6 (20), 5.2 (23), 8.8 (22), 12.1 (23), 15.9 (22) | 16.9 (after the last car) | 16.9 (after the last car) | 16.9 (after the last car) | none | — | 7.9 s | 17.4 | 17.4 | 3.2 |
| 67 | agent_0 | 11.4 m/s, 42 m | 16.0 / 16.0 | 3.2 (23), 6.5 (22), 10.2 (22), 13.8 (23), 17.3 (23) | 18.3 (after the last car) | 18.3 (after the last car) | 18.3 (after the last car) | none | — | 9.9 s | 18.8 | 18.8 | 3.2 |

## How open the loop is

| Row | Same SUMO car, same instant, before either crossed: matched (cars) | Largest difference: distance / speed | Up to | Round 16: crossed at (speed) | Open loop: the car behind would need | What SUMO logged with round 16 |
|---|---|---|---|---|---|---|
| 8 agent_0 | 57 (1) | 1.524 m / 0.725 m/s | 5.7 s | 8.3 s (20.0 m/s) | not projected: the stream differed | main_8 emergency braking 9.0 m/s² at 10.9 s; main_8 junction collision with agent_2 at 11.0 s |
| 8 agent_2 | 57 (1) | 1.524 m / 0.725 m/s | 5.7 s | 10.6 s (20.0 m/s) | not projected: the stream differed | main_8 emergency braking 9.0 m/s² at 10.9 s; main_8 junction collision with agent_2 at 11.0 s |
| 8 agent_6 | 57 (1) | 1.524 m / 0.725 m/s | 5.7 s | 5.8 s (18.6 m/s) | not projected: the stream differed | main_8 emergency braking 9.0 m/s² at 10.9 s; main_8 junction collision with agent_2 at 11.0 s |
| 9 agent_0 | 107 (2) | 27.094 m / 9.588 m/s | 8.0 s | 11.3 s (14.4 m/s) | not projected: the stream differed | main_8 emergency braking 9.0 m/s² at 8.1 s; main_5 emergency braking 9.0 m/s² at 13.8 s |
| 9 agent_2 | 107 (2) | 27.094 m / 9.588 m/s | 8.0 s | 13.8 s (15.7 m/s) | not projected: the stream differed | main_8 emergency braking 9.0 m/s² at 8.1 s; main_5 emergency braking 9.0 m/s² at 13.8 s |
| 22 agent_0 | 53 (1) | 0.000 m / 0.000 m/s | 5.8 s | 5.9 s (20.1 m/s) | -10.5 m/s² (main_11) | nothing |
| 24 agent_0 | 37 (2) | 0.000 m / 0.000 m/s | 6.6 s | 6.7 s (19.9 m/s) | -0.9 m/s² (main_8) | agent_0 junction collision with main_11 at 7.1 s; agent_0 emergency braking 9.0 m/s² at 7.6 s |
| 37 agent_0 | 0 (0) | — m / — m/s | — s | 3.4 s (16.5 m/s) | — | nothing |
| 38 agent_0 | 24 (1) | 0.000 m / 0.000 m/s | 3.2 s | 3.3 s (17.4 m/s) | -7.9 m/s² (main_13) | nothing |
| 39 agent_0 | 29 (1) | 0.000 m / 0.000 m/s | 2.9 s | 3.3 s (16.9 m/s) | -99.0 m/s² (main_13) | main_13 emergency braking 9.0 m/s² at 3.3 s |
| 53 agent_0 | 53 (2) | 0.000 m / 0.000 m/s | 3.1 s | 3.2 s (16.2 m/s) | -5.8 m/s² (main_9) | nothing |
| 67 agent_0 | 20 (1) | 0.000 m / 0.000 m/s | 2.6 s | 3.2 s (15.3 m/s) | -1.6 m/s² (main_11) | nothing |

## What the next steps would cost (SUMO, from recorded decisions)

| Option | Decisions (projected) | Episodes | Clock (expected) | Decision allowance |
|---|---|---|---|---|
| step 2 as configured (70 rows) | 274,736 (288,473) | 143 | 170 s | 290,000 |
| stream rows only (9 rows) | 27,863 (29,256) | 21 | 73 s | 30,000 |

The clock is the re-check's own estimate: the decisions at the measured rate, or the longest episode at one worker's rate, whichever is longer, plus start and settling time. Step 2's proposed time allowance is 300 s. A stream-only run could not apply the no-regression rule, which needs every development row.

In the open-loop column, -99 means the car behind would have overlapped ours.

Study time left: 1414.8 s.
