# Flow test artifacts

The flow contract, schedule, environment adapters, bank geometry, reference,
recording metrics, summaries, and visualization tests run on synthetic CPU data.
They do not launch SUMO, the GPU renderer, or any optimizer step.

`test_highway_flow_training.py` builds pure configurations with
`build_config(None)`. This deliberately does not validate an unavailable native
bank or assert launch readiness. The committed proposed configuration remains
bound to its recorded bank and stays disabled.

Three tests additionally inspect local experiment artifacts:

- The committed resolved-configuration check requires the bank manifest named by
  that configuration under ignored `outputs/`.
- The two-initializer migration check requires both original checkpoint files
  named in the pinned initializer contract and the frozen final actor export.
- The matched frozen-gradient check requires the final checkpoint and actor
  export. It checks derivatives and state preservation without optimizer steps.

Only these tests skip when their required files are absent, with paths in the
skip reason. Present but incorrect artifact hashes still fail; tests do not
download weights, create substitutes, or relabel missing evidence as passing.
All other flow tests continue running in a clean clone. Temporary contract
outputs are created below the checkout's ignored `outputs/` directory and
removed by their tests.

Set `HIGHWAY_FLOW_PORTABLE_TESTS=1` for an explicit portability check on a
workstation that already has the local artifacts. This forces those same three
artifact-backed skips while retaining the pure tests. For example:

```powershell
$env:HIGHWAY_FLOW_PORTABLE_TESTS = '1'
.venv-traffic\Scripts\python.exe -m pytest tests/test_highway_flow_training.py -q -rs
Remove-Item Env:HIGHWAY_FLOW_PORTABLE_TESTS
```

The remaining flow context/bank tests construct their own small XML fixtures;
they do not depend on historical networks under `outputs/` or invoke a native
network builder. A skipped artifact test is not new validation evidence.
