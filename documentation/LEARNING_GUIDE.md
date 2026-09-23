# Engineering & Learning Guide

[Open the native Google Doc](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit)

Created as a learning companion with actual document tabs, not a replacement for
source code, reproducibility artifacts, or the roadmap. The initial edition
covers implementation history through commit `69d3cb3`; historical changes are
grouped by meaningful engineering step, and several share one integration commit.
Reported checks are historical evidence, not tests rerun during document authoring.

## Initial tabs

| Tab | Topic | Change reference |
| --- | --- | --- |
| 00 | Start here: architecture, scope, reading order | Overview |
| 01 | Repository boundaries | a308dbe, 6d42287 |
| 02 | Dependencies and portability | 687e785 |
| 03 | SUMO and TraCI | d5a82fd |
| 04 | Isaac and lidar decision | 9d9d07e |
| 05 | PettingZoo and curved traffic | a322a11 |
| 06 | Scripted and random baselines | 91a1e41 |
| 07 | Shared PPO | 32dc5c6 |
| 08 | Awareness and safety refinement | 61e1b6a |
| 09 | Static RTX lidar | 1b75cda |
| 10 | Moving traffic in OpenUSD | 1b75cda |
| 11 | Repository transfer | 1b75cda |
| 12 | GUI lidar fix | 1b75cda |
| 13 | ovrtx architecture decision | 6c8d007 |
| 14 | Shared exporter and ovrtx | 69d3cb3 |
| 15 | Interview and study workbook | Learning exercises |
| 16 | Moving lidar geometry and timing | See moving-lidar-validation.md |
| 17 | Curved replay lidar validation | See replay-lidar-validation.md |
| 18 | Ground truth vs lidar perception | Architecture clarification; no code change |
| 19 | Lidar failure diagnostics | See lidar-diagnostics.md |
| 20 | Two-car lidar identity isolation | See lidar-lifecycle-validation.md |

## Ongoing update contract

For each meaningful implementation, experiment, asset, or parameter change,
append a dedicated tab containing:

1. What changed and the problem it addresses.
2. The technology and mechanism, explained in plain language.
3. Why this approach was chosen and its tradeoffs.
4. Relevant source files and the committed change.
5. Steps to reproduce or a small independent learning exercise.
6. What was verified, with evidence and remaining limitations.
7. An accurate explanation suitable for an interview.

Read existing content before updating; preserve user notes. Link to committed
source rather than implying ignored local checkpoints are available on GitHub.
Do not describe planned lidar-based control or explicit vehicle communication as
implemented. Record a pending entry here if the connected document is unavailable.

This documentation-only change introduced the guide and its upkeep requirement;
it did not alter simulation, training, sensors, or rendering behavior.
