# PROJECT_STATUS

## 1. What this project is doing now

The project has already shifted away from the old baseline UAV relay experiments.

Current goal:

- read an external JSON schedule
- replay UAV movement, role changes, and transfer tasks in NS-3
- verify whether all required data can be delivered before deadlines

Current task flow:

**Sensor -> Slave UAV -> CH / Master UAV -> LEO**

At this stage, `uav-vanet.cc` is the **NS-3-side replay / verifier** for the UAV-side collection path, and `uav-to-leo.cc` is the separate Master/CH UAV to LEO uplink simulator. A first-stage batch handoff between the two has been added through CSV files, but the fully synchronized end-to-end simulator is not complete yet.

---

## 2. Code evolution summary

### Earlier phase
The project started from the Lab2 / LEO setup:
- NS-3.35
- leo module
- `calculate_delay.cc`
- delay / SNR / data-rate related experiments

### Current phase
`uav-vanet.cc` has been refactored into a schedule-driven engine that:

- parses JSON traces
- replays `MOVE`
- replays `ROLE_SET`
- replays `SENSOR_TO_UAV`
- replays `SLAVE_TO_MASTER`
- tracks task completion and mission finish times

Recent update:
- `uav-vanet.cc` now outputs `master_buffer_trace.csv`, recording chunk-level data arrival at the Master/CH UAV.
- `uav-to-leo.cc` has been added to the build through `wscript` and consumes the master buffer CSV as the first handoff input.
- `run_e2e_batch.sh` and the updated `make_e2e_summary.py` run the first-stage batch pipeline and summarize outputs.
- `convert_scheduler_to_vanet.py` converts scheduler/probe-derived traces into the `uav-vanet` replay JSON format.

Important internal logic:
- transfer events become replay tasks
- sensor-uploaded bytes are stored in a UAV buffer
- slave forwarding can only use bytes already received by that UAV
- mission summary reports task completion and deadline satisfaction

---

## 3. Current file roles

### `calculate_delay.cc`
Earlier LEO / delay / SNR-rate basis.

### `uav-vanet.cc`
Current mainline replay / verification engine.

### `schedule_trace.json`
Simple topology validation input.

### `topo1.json`
First larger topology already tested.

### `topo2.json`
Additional topology input.

### `scheduler_trace.json` / `topology_summary.json`
Scheduler/probe-derived input data.
These are converted into `scheduler_trace_vanet.json` before replay.

### `scheduler_trace_vanet.json`
Converted replay input for `uav-vanet.cc`.
Current validated batch pipeline run uses this file.

### `uav-link-probe.cc`
Current UAV-to-UAV probing tool.

### `uav-to-leo.cc`
Master/CH UAV to LEO uplink simulator.
It computes satellite link budget / capacity and is now used after `uav-vanet` through the master buffer CSV handoff.

### `run_e2e_batch.sh` / `make_e2e_summary.py`
First-stage batch pipeline helper scripts.
They run `uav-vanet`, extract the master-arrived bytes, invoke `uav-to-leo`, and generate `pipeline_summary.csv`.

### `convert_scheduler_to_vanet.py`
Small conversion helper.
It converts `scheduler_trace.json` and `topology_summary.json` into the event format accepted by `uav-vanet.cc`.

---

## 4. Current validated progress

### Simple topology
`schedule_trace.json` was the first functional validation trace.

Meaning:
- schedule parsing works
- replay works
- mission summary works

### Larger topology
`topo1.json` is the first larger topology already validated against the newer code line.

This confirms that the current code is no longer limited to the simple trace.

### Probing mode
An initial `uav-link-probe.cc` has been added.

Current status:
- program builds and runs
- outputs per-link `src / dst / role / position / distance / snr / estRate / throughput`
- useful as the first functional probing version

Current limitation:
- with conservative probe pacing, measured throughput may be limited by application sending interval rather than the actual wireless link capacity
- further parameter sweeps are still needed before producing the final throughput table for scheduling

### Master buffer handoff / batch E2E test
`uav-vanet.cc` can now generate `master_buffer_trace.csv`, which records Master/CH-side received data for the next LEO upload phase.

A first-stage batch pipeline has been added:

```text
uav-vanet -> master_buffer_trace.csv -> uav-to-leo -> pipeline_summary.csv
```

Current observed status:
- `uav-vanet` finishes and generates `output_vanet.txt` / `master_buffer_trace.csv` correctly.
- the updated `uav-to-leo.cc` completes the Master/CH UAV to LEO fixed-volume upload in the batch pipeline.
- the current representative output set is stored under `outputs/` as listed in the repo layout.

Latest validated run:
- `tasksDone=619/619`, `allDone=YES`
- `metMissionDeadline=YES`
- `totalMasterArrivedBytes=1238529`
- `leo_total_rx_bytes=1238529`, `leo_done=YES`

---

## 5. Current limitations

Still missing or not fully integrated yet:

1. initial slave-to-master probing throughput mode is implemented, but still needs:
   - parameter tuning for more discriminative throughput measurement
   - final teammate-aligned output format confirmation
2. UAV-to-LEO / beamforming line is connected through the CSV-based batch pipeline, but not yet integrated into a single synchronized simulator
3. fully synchronized single-simulator E2E pipeline is not complete yet
4. slot-based progress trace is not implemented yet
5. full scheduler feedback loop

So the current repo should be described as:

> **schedule replay / verification implemented, first-stage CSV-based E2E batch pipeline validated, fully synchronized E2E simulator not yet complete**

---

## 6. Immediate next steps

1. keep the current CSV handoff as the short-term validated integration path
2. keep README / docs aligned with the representative `outputs/` result set
3. add slot-based progress output later: arrived bytes, uploaded bytes, remaining bytes per slot
4. align final scheduler input/output format with teammates
5. later merge the two simulator lines into a synchronized E2E NS-3 program if needed

---

## 7. Practical continuation note

If continuing from this repo:

- treat `uav-vanet.cc` as the current mainline
- use `scheduler_trace_vanet.json` as the current validated scheduler-derived replay input
- keep `schedule_trace.json` and `topo1.json` as older reference inputs
- keep future changes focused on schedule-driven replay, throughput probing, LEO integration, and slot-based progress reporting
- use the `outputs/` files in the repo layout as the current representative successful batch-output set
