# <Date> — <Milestone name>

Status: Planned / In progress / Verified / Blocked.
Roadmap step: <number>. Branch / source commit: <reference>.

## Purpose and scope

What question does this change answer? What stays unchanged? Which subsystem
owns motion, and which observations are simulator state versus sensor-derived?

## What changed and why

| Change | Purpose / alternative considered | Source file or decision |
| --- | --- | --- |
| <change> | <reason> | <path> |

Explain important terms, interfaces, units, and technology choices in language a
new contributor could use to reproduce the work or explain it in an interview.

## Predeclared acceptance

- Cases, seed set, controls, safety guardrails, and numeric tolerances:
- Primary metric and failure rule:
- Runtime versions, hardware, dependencies, and observation provenance:
- Unchanged demand, role assignments, authority, and sensor allocation for comparisons:

## Demonstrate or reproduce

Run from the repository root in <named environment>. State any required local
configuration and whether the command opens a GUI. Use portable paths and a
single command per line; do not depend on shell-specific continuation characters.

```text
<verified command>
```

Expected visible behavior: <description>. Stop/cleanup behavior: <description>.
Do not present a proposed command as verified.

## Evidence and results

Run ID / output directory: <unique path>. Manifest / source hash: <reference>.
Git state at run time: <commit and clean/dirty; retain dirty diff>.

| Gate / metric | Expected | Observed | Pass / fail / not tested |
| --- | --- | --- | --- |
| <gate> | <threshold> | <value or N/A> | <status> |

Link raw records, derived reports, versions, exact command, seeds, and all failed
or interrupted attempts. State which artifacts are ignored local files and
therefore unavailable in an ordinary Git clone. Report repeat differences and
unfinished episodes rather than silently excluding them.

## Limits and next gate

What is not demonstrated? Which risks or decisions remain? What specific test
would justify moving to the next roadmap step?

## Learning document synchronization

Document / tab: <link>. Source commit or PR: <link>.
Sync status: pending / updated and read-back verified / blocked with reason.
Record what was added, not just that an API request was attempted.
