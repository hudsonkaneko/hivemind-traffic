"""Version 5 selection for complete, matched Stage 1 validation reports.

Higher lexicographic ranks are better. Actual safety and the existing per-task
competence gates precede intervention minimization. Once those requirements are
met, failing-case count, worst-case intervention rate and pooled intervention
rate precede marginal cruise speed or reward. Complete boundary-inclusive trip
accounting remains mandatory; unfinished trips and total cohort burden remain
tie-breaks. No arrival-rate target is invented for the short, long-road fixture.

This does not introduce a following-speed acceptance gate. Following slowdown
remains a review metric: a low intervention rate alone cannot certify following
competence. The caller must use the frozen matched validation matrix and never
use this ranking on held-out tests to select a checkpoint.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from numbers import Real


RANKING_VERSION = 5
RANK_FIELDS = (
    "negative_safety_events",
    "negative_task_failure_cases",
    "negative_override_failure_cases",
    "negative_worst_override_fraction",
    "negative_pooled_override_fraction",
    "negative_unfinished_trips",
    "negative_boundary_time_s",
    "negative_external_wait_s",
    "minimum_cruise_speed_fraction",
    "successful_merges",
    "passes",
    "mean_speed_fraction",
    "reward",
)
TASKS = frozenset(("cruise", "following", "passing", "merge"))


def _number(value, name, *, minimum=None, maximum=None, integer=False):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if (minimum is not None and value < minimum) or (maximum is not None and value > maximum):
        raise ValueError(f"{name} is outside its valid range")
    if integer and value != int(value):
        raise ValueError(f"{name} must be an integer")
    return int(value) if integer else float(value)


def _required(item, key, context):
    if not isinstance(item, Mapping) or key not in item:
        raise ValueError(f"Missing {context}.{key}")
    return item[key]


def _metric(item, key, context, **limits):
    return _number(_required(item, key, context), f"{context}.{key}", **limits)


def _validated_report(report):
    case = _required(report, "case_id", "report")
    if not isinstance(case, str) or not case:
        raise ValueError("case_id must be a nonempty string")
    task = _required(report, "task", case)
    if not isinstance(task, str) or task not in TASKS:
        raise ValueError(f"Unknown task for {case}")
    seed = _metric(report, "seed", case, integer=True, minimum=0)
    for key in ("primary_horizon_complete", "accounting_comparison_valid"):
        if _required(report, key, case) is not True:
            raise ValueError(f"{case} requires complete comparable primary-horizon evidence")
    if report.get("partial_control_horizon") or report.get("incomplete_episode"):
        raise ValueError(f"{case} contains a partial/incomplete control horizon")
    steps = _metric(report, "agent_steps", case, integer=True, minimum=1)
    overrides = _metric(report, "overrides", case, integer=True, minimum=0, maximum=steps)
    fraction = _metric(report, "intervention_fraction", case, minimum=0, maximum=1)
    if not math.isclose(fraction, overrides / steps, rel_tol=1e-10, abs_tol=1e-12):
        raise ValueError(f"{case} intervention fraction disagrees with counts")
    safety = sum(_metric(report, key, case, integer=True, minimum=0)
                 for key in ("collisions", "teleports", "overspeed_steps"))
    speed = _metric(report, "mean_fraction_of_speed_limit", case, minimum=0)
    passes = _metric(report, "passes", case, integer=True, minimum=0)
    merges = _metric(report, "successful_merges", case, integer=True, minimum=0)
    if merges > 2:
        raise ValueError(f"{case} exceeds the two-ramp-agent Stage 1 contract")
    reward = _metric(report, "reward", case)

    accounting = _required(report, "accounting", case)
    if _required(accounting, "comparison_valid", case + ".accounting") is not True:
        raise ValueError(f"{case} requires a valid trip ledger")
    totals = _required(accounting, "totals", case + ".accounting")
    counts = {key: _metric(totals, key, case + ".accounting", integer=True, minimum=0)
              for key in ("scheduled", "waiting", "active", "teleporting", "completed", "removed",
                          "requested", "unfinished", "teleported")}
    if counts["requested"] < 1 or counts["removed"] or counts["teleported"]:
        raise ValueError(f"{case} has an empty or failed trip cohort")
    if counts["requested"] != sum(counts[k] for k in ("waiting", "active", "teleporting", "completed", "removed")):
        raise ValueError(f"{case} trip ledger does not conserve the requested cohort")
    if counts["unfinished"] != counts["requested"] - counts["completed"]:
        raise ValueError(f"{case} unfinished trips disagree with the requested cohort")
    for top, ledger in (("requested_trips", "requested"), ("completed_trips", "completed")):
        if _metric(report, top, case, integer=True, minimum=0) != counts[ledger]:
            raise ValueError(f"{case} {top} disagrees with the trip ledger")
    boundary = _metric(totals, "boundary_time_s", case + ".accounting", minimum=0)
    waiting = _metric(totals, "external_wait_s", case + ".accounting", minimum=0)
    if waiting > boundary + 1e-8:
        raise ValueError(f"{case} external wait exceeds total trip burden")
    primary_time = _metric(report, "primary_horizon_time", case, minimum=0)
    ledger_time = _metric(accounting, "time", case + ".accounting", minimum=0)
    if not math.isclose(primary_time, ledger_time, rel_tol=1e-10, abs_tol=1e-8):
        raise ValueError(f"{case} ledger is not measured at the primary horizon")
    return dict(case_id=case, seed=seed, task=task, safety_events=safety,
                agent_steps=steps, overrides=overrides, fraction=fraction, speed=speed,
                passes=passes, merges=merges, reward=reward, requested=counts["requested"],
                unfinished=counts["unfinished"], boundary_time_s=boundary, external_wait_s=waiting)


def selection_diagnostics(reports: Sequence[Mapping], cfg: Mapping) -> dict:
    """Validate complete evidence and return an auditable rank and gate counts.

    Raises ValueError for incomplete, inconsistent or nonfinite evidence rather
    than letting missing metrics silently make a checkpoint look better. Safety
    events in otherwise complete reports are ranked explicitly, never hidden.
    """
    evaluation = _required(cfg, "evaluation", "config")
    targets = _required(evaluation, "provisional_targets", "evaluation")
    maximum = _metric(targets, "maximum_intervention_fraction", "targets", minimum=0, maximum=1)
    cruise_minimum = _metric(targets, "cruise_mean_speed_limit_fraction_min", "targets", minimum=0, maximum=1)
    pass_minimum = _metric(targets, "passes_min_per_passing_episode", "targets", integer=True, minimum=1)
    merge_minimum = _metric(targets, "merge_success_fraction_min", "targets", minimum=0, maximum=1)
    configured_seeds = _required(evaluation, "validation_seeds", "evaluation")
    expected = [_number(s, "validation seed", integer=True, minimum=0) for s in configured_seeds]
    if not expected or len(set(expected)) != len(expected):
        raise ValueError("Configured full validation seeds must be nonempty and unique")
    if not reports:
        raise ValueError("Full validation reports are required")
    rows = sorted((_validated_report(r) for r in reports), key=lambda r: r["seed"])
    if len(rows) != len(expected) or {r["seed"] for r in rows} != set(expected):
        raise ValueError("Ranking requires exactly the configured full validation seed set")
    if {r["task"] for r in rows} != TASKS:
        raise ValueError("Full validation must contain every Stage 1 driving task")
    task_failures = []
    override_failures = []
    for row in rows:
        failed = ((row["task"] == "cruise" and row["speed"] < cruise_minimum)
                  or (row["task"] == "passing" and row["passes"] < pass_minimum)
                  or (row["task"] == "merge" and row["merges"] / 2 < merge_minimum))
        identity = dict(case_id=row["case_id"], seed=row["seed"])
        if failed:
            task_failures.append(identity)
        if row["fraction"] > maximum:
            override_failures.append(identity | dict(intervention_fraction=row["fraction"]))
    safety = sum(r["safety_events"] for r in rows)
    decisions = sum(r["agent_steps"] for r in rows)
    overrides = sum(r["overrides"] for r in rows)
    worst = max(r["fraction"] for r in rows)
    pooled = overrides / decisions
    unfinished = sum(r["unfinished"] for r in rows)
    boundary = sum(r["boundary_time_s"] for r in rows)
    wait = sum(r["external_wait_s"] for r in rows)
    cruise = min(r["speed"] for r in rows if r["task"] == "cruise")
    rank = (-safety, -len(task_failures), -len(override_failures), -worst, -pooled,
            -unfinished, -boundary, -wait, cruise, sum(r["merges"] for r in rows),
            sum(r["passes"] for r in rows), sum(r["speed"] for r in rows) / len(rows),
            sum(r["reward"] for r in rows))
    if not all(math.isfinite(value) for value in rank):
        raise ValueError("Aggregated validation metrics must remain finite")
    return dict(ranking_version=RANKING_VERSION, rank_fields=RANK_FIELDS, rank=rank,
                cases=len(rows), quality_accepted=safety == 0 and not task_failures and not override_failures,
                safety_events=safety, task_failure_cases=task_failures,
                override_failure_cases=override_failures, worst_override_fraction=worst,
                pooled_override_fraction=pooled, override_decisions=overrides, agent_decisions=decisions,
                unfinished_trips=unfinished, boundary_time_s=boundary, external_wait_s=wait,
                limitation="Existing task gates retained; following slowdown remains a separate review metric")


def selection_rank(reports: Sequence[Mapping], cfg: Mapping) -> tuple:
    """Return the version 5 lexicographic rank; greater is better."""
    return selection_diagnostics(reports, cfg)["rank"]
