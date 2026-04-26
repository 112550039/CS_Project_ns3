# PROJECT_STATUS

## 1. What this project is doing now

The project has already shifted away from the old baseline UAV relay experiments.

Current goal:

- read an external JSON schedule
- replay UAV movement, role changes, and transfer tasks in NS-3
- verify whether all required data can be delivered before deadlines

Current task flow:

**Sensor -> Slave UAV -> CH / Master UAV**

At this stage, the code is mainly the **NS-3-side replay / verifier**.

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
Next topology to test.

### `uav-link-probe.cc`
Current UAV-to-UAV probing tool.

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

---

## 5. Current limitations

Still missing or not fully integrated yet:

1. initial slave-to-master probing throughput mode is implemented, but still needs:
   - parameter tuning for more discriminative throughput measurement
   - final teammate-aligned output format confirmation
2. UAV-to-LEO / beamforming integration
3. full scheduler feedback loop

So the current repo should be described as:

> **schedule replay / verification implemented, full end-to-end pipeline not yet complete**

---

## 6. Immediate next steps

1. refine `uav-link-probe.cc` probing parameters and validate more discriminative throughput results
2. align final scheduler input/output format with teammates
3. later connect the UAV-side replay with the UAV-to-LEO / beamforming line

---

## 7. Practical continuation note

If continuing from this repo:

- treat `uav-vanet.cc` as the current mainline
- use `schedule_trace.json` and `topo1.json` as reference inputs
- keep future changes focused on schedule-driven replay, throughput probing, and LEO integration
