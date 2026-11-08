# Rejected adjacent-preview reference v5

After all six fixed v5 native cases completed, v5 was rejected and the exact v4
source and test snapshots were restored. No training or optimizer updates were
performed. The [audit](v5-rejected-adjacent-preview.json) records source hashes,
all six report hashes, and measured results; the [patch](v5-rejected-adjacent-preview.patch)
reconstructs the rejected v5 changes from the restored v4 files.

V5 tested potential cut-in caution using only existing adjacent-front observations.
For closing adjacent cars with valid lane/presence flags, let
`R=required_gap(ego,adjacent)+3`, `T=2 s`. Within `gap<R+ego_speed*T`, project
`gap_T=max(0,gap-(ego_speed-adjacent_speed)*T)` and add the soft speed target
`min(legal_buffer,max(0,adjacent_speed+0.35*(gap_T-R)))` before the unchanged jerk
slew. This inferred no lane-change intent and changed no hard bound, lane mask,
native controller, intervention count, or acceptance threshold.

The extra caution failed its unchanged comfort/progress checks. Controlled peak
jerk remained 16.166 m/s³ in cruise, 25.394 in following, 11.357 in passing,
35.842 in I-110 merge, and 6.571 in synthetic merge. Passing achieved one pass,
versus seven with v4; speed ratios fell to 0.810 in passing, 0.862 in following,
and 0.813 in I-110. Dense-following peak was 3.690, but that isolated result does
not accept the treatment. All failures and full native records remain preserved.

The restored v4 remains the prepared candidate, **not ready for training**: four
nominal cases still fail peak comfort. Mandatory/native braking is not weakened
or excluded. Another design requires review of the information and progress
tradeoffs, not an unbounded parameter search or retrospective gate changes.
