# Synthetic open-merge accounting fixture

This is an engineering fixture, not I-80, calibrated human behavior or an Esri
road. Two mainline lanes and one entering ramp feed a two-lane downstream road.
Twelve explicit requests include simultaneous arrivals that wait for insertion
and two late requests at 35 s. Role and origin are separate ledger dimensions.
Native SUMO controls every vehicle in the fixture, including the two vehicles
labeled `controlled` for cohort accounting; no learned or communicating policy
is exercised.

Rebuild with the configured SUMO `netconvert` executable:

```text
netconvert --node-files scenarios/accounting_merge/nodes.nod.xml --edge-files scenarios/accounting_merge/edges.edg.xml --connection-files scenarios/accounting_merge/connections.con.xml --output-file scenarios/accounting_merge/network.net.xml --no-turnarounds true --no-warnings true
```

The checked-in network was built with SUMO 1.27.1. Its generated timestamp uses
the runtime's timezone and is provenance, not a user-facing schedule. Run the
40 s primary horizon plus at most 90 s drain as documented in
`documentation/merge-accounting-2026-10-22.md`. All inputs remain unchanged by runs.
