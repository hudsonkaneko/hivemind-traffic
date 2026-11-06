# Highway flow v01 — preparation only

**No new trained policy or accepted checkpoint.** Training is disabled pending
the diagnostic gates, review of the final configuration, and explicit approval.

| Detail | Prepared contract |
| --- | --- |
| Policy type | Shared MAPPO actor; centralized masked population critic during training |
| Actor input | 117 fields: 69 local/map/neighbor fields plus six eight-field delivered messages |
| Action | Acceleration −4.5…2 m/s² and masked right/stay/left intent, every 0.1 s |
| Population | Up to 24 controlled slots; shared actor runs per vehicle |
| Changes | Route continuation/runway, declared entry visibility, per-corridor response estimates, lateral motion, qualified component-wise imitation labels |
| First proposed task | Cruise/following foundation; later passing/merging and cooperation require separate advancement evidence |
| Learning comparison | Identical legacy-weight objective and new reference; acceleration imitation multiplier 1 vs 5 only |
| Budget | 30,000 decisions per arm, including up to 2,048 critic warmup; shared 5,400 seconds including preparation and evaluation |
| Limitations | Structured simulator/map data, constant-neighbor-speed gap forecasts, no persistent pre-maneuver gap identity, no camera/LiDAR policy input or physical-control qualification |

The old 99-field v05 actor and 93-field initializer migrate with zero columns for
the new inputs. Migration preserves frozen actions; it does not teach the actor
to use those inputs. Existing policy files remain unchanged.

[Resolved disabled configuration](../../../configs/training/highway-flow-v01.json)
and [implementation, measurements and replay](../../../documentation/highway-flow-preparation-v01.md).
Raw evidence/checkpoints remain local under `outputs/`, not inside this folder.
