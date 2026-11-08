# Highway flow RL v02 - prepared, not trained

This version changes the I-110 visibility fixture and prepares two reward terms:
eligible legal-speed cruising and avoiding optional ending-lane detours. No v02
`policy.pt` exists yet. Preparation replays use the frozen experimental
[v01 round-two actor](../highway-flow-rl-v01/README.md).

- Shared MAPPO; unchanged 117-input actor and 2,832-input training-only critic.
- Actions: normalized acceleration and right/stay/left intent; native 10 Hz SUMO
  applies -4.5..+2 m/s2 with the same safety modes, limits and masks.
- Intended scope: smoother highway entry/cruising and better lane/route choices,
  with varying connected-vehicle counts. These are goals, not verified capability.
- Clear-approach native braking disappears with the visibility candidate, but
  frozen-policy full-route behavior regresses and development gates still fail.
- Initial pilot prepared for 30,000 active-agent decisions; training disabled
  until the resolved configuration is reviewed. No acceptance promotion.

[Exact configuration](../../../configs/training/highway-flow-rl-v02.json) ·
[Implementation, tests and measurements](../../../documentation/highway-flow-rl-v02.md)
