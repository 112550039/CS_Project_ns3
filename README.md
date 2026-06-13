# UAV-Assisted SAGIN Data Collection Simulation

This repo contains our NS-3 project for **UAV-assisted data collection**.

Current focus:

- replay an external JSON schedule in NS-3
- simulate `MOVE / ROLE_SET / SENSOR_TO_UAV / SLAVE_TO_MASTER`
- check whether all tasks finish before their deadlines
- report mission finish time, sensor finish time, and slave finish time
- probe slave-to-master UAV links and output per-link estimated data rate / measured throughput for scheduling

Current mainline components:
- **`uav-vanet.cc`** — schedule-driven replay / verification engine
- **`uav-link-probe.cc`** — UAV-to-UAV probing tool for slave-to-master link measurement
- **`uav-to-leo.cc`** — Master/CH UAV to LEO uplink simulator
- **`run_e2e_batch.sh` / `make_e2e_summary.py`** — first-stage batch pipeline scripts

It is no longer the old random baseline simulator. It now acts as a **schedule-driven replay / verification engine**.

---

## Current status

Validated inputs so far:

- `schedule_trace.json` — simple topology
- `topo1.json` — larger topology
- `topo2.json` — larger topology

Current repo role:

- **implemented:** schedule replay / mission verification
- **implemented (initial):** UAV-to-UAV probing for slave-to-master links
- **implemented (initial):** Master buffer trace handoff from `uav-vanet.cc` to `uav-to-leo.cc`
- **implemented:** first-stage batch E2E pipeline through CSV handoff
- **validated:** `uav-vanet -> master_buffer_trace.csv -> uav-to-leo -> pipeline_summary.csv` on the TA scheduler trace
- **not yet complete:** fully synchronized end-to-end NS-3 pipeline, slot-based progress observation, full scheduler feedback loop

---

## Main files

### `ns3/contrib/leo/examples/uav-vanet.cc`
Current mainline simulator.

Main functions:
- parse JSON schedule
- replay UAV movement and roles
- replay sensor upload and slave forwarding tasks
- maintain UAV buffer dependency
- report mission summary

#### Master Buffer Trace Handoff:
Currently, `uav-vanet.cc` is outputting `master_buffer_trace.csv`.

This file contains a **chunk-level trace of data received by the Master/CH UAV**, which can be used as input for the subsequent `Master/CH UAV → LEO` upload phase.

Each column represents a chunk actually received by the Master/CH at a specific point in time.

Currently, LEO upload is not yet integrated, so `masterBufferBytes` will only accumulate and will not decrease due to uploads to LEO.

---

#### CSV Column Format

```csv
timeSec,event,taskId,type,src,dstMaster,bytesArrived,masterBufferBytes,masterArrivedBytes,totalMasterArrivedBytes
```

| Column | Meaning |
|---|---|
| `timeSec` | NS-3 simulation time, in seconds |
| `event` | Master buffer event, e.g., `SLAVE_TO_MASTER_RX` or `SENSOR_TO_MASTER_RX` |
| `taskId` | Transfer task ID from the corresponding schedule JSON |
| `type` | Original transfer type, e.g., `SLAVE_TO_MASTER` / `SENSOR_TO_UAV` |
| `src` | Source of data, e.g., `UAV_0` or `Sensor_458` |
| `dstMaster` | Data received Master/CH UAV |
| `bytesArrived` | The actual number of bytes received in this chunk |
| `masterBufferBytes` | The buffer bytes currently accumulated by this Master that have not yet been uploaded to LEO |
| `masterArrivedBytes` | The total number of bytes accumulated by this Master since the simulation started |
| `totalMasterArrivedBytes` | The total number of bytes accumulated by all Masters/CHs |
---

### `ns3/contrib/leo/examples/calculate_delay.cc`
Earlier LEO / delay / SNR-rate line.  
Useful as the basis for future UAV-to-LEO integration.

### `ns3/contrib/leo/examples/uav-link-probe.cc`
Initial UAV-to-UAV probing tool.

Main functions:
- probe slave-to-master links sequentially
- output `src / dst / role / position / distance`
- estimate SNR and Shannon-based data rate
- measure average effective throughput with probing packets

Current note:
- conservative probe pacing can under-drive the link, so throughput results should be re-tested with more aggressive probing intervals before final use in scheduling

### `ns3/contrib/leo/examples/run_e2e_batch.sh`
First-stage batch pipeline script.

Main flow:
- run `uav-vanet` with a schedule JSON
- generate `master_buffer_trace.csv`
- extract `totalMasterArrivedBytes`
- pass the total volume and CSV trigger file to `uav-to-leo`
- generate a short `pipeline_summary.csv`

Current note:
- this is a **file-based batch handoff**, not a fully synchronized single-simulator pipeline yet
- recent testing shows the first-stage batch pipeline can complete successfully on the TA scheduler trace
- current validated output set is stored under `pipeline_output/` and includes `output_vanet.txt`, `master_buffer_trace.csv`, `output_uav_to_leo.txt`, `pipeline_summary.csv`, `uav-to-leo_result.csv`, `visibility_windows.csv`, `csv_trigger.csv`, and `adaptiveRateLog.csv`

### `ns3/contrib/leo/examples/make_e2e_summary.py`
Helper script used by `run_e2e_batch.sh`.
It parses `output_vanet.txt`, `master_buffer_trace.csv`, `output_uav_to_leo.txt`, and available LEO CSV outputs, then writes `pipeline_summary.csv`.

### `ns3/contrib/leo/examples/convert_scheduler_to_vanet.py`
Helper script for converting TA scheduler output into the replay JSON format accepted by `uav-vanet.cc`.


### JSON inputs
- `schedule_trace.json`
- `topo1.json`
- `topo2.json`
- `scheduler_trace.json` / `topology_summary.json` — TA scheduler/probe-derived input
- `scheduler_trace_vanet.json` — converted replay input for `uav-vanet`

These traces define:
- UAV count
- sensor positions
- mission deadline
- event list (`MOVE`, `ROLE_SET`, `SENSOR_TO_UAV`, `SLAVE_TO_MASTER`)

### `ns3/contrib/leo/examples/uav-to-leo.cc`
Single-hop UAV-to-LEO satellite uplink simulator.

Main functions:
- build satellite constellation and auto-select nearest LEO satellite
- compute link budget: FSPL → SNR → Shannon capacity
- apply hybrid beamforming gain via MATLAB-generated CSV lookup
- measure effective throughput at application layer
- support fixed-volume mode: stop simulation once `maxBytes` are received, measuring transmission time rather than running a fixed duration

Key parameters:
- `--band`: frequency band preset (`Ku-User` default)
- `--bpFile`: path to MATLAB beam pattern CSV
- `--nAnt` / `--nRF`: antenna element count and RF chain count for hybrid beamforming
- `--maxBytes`: total bytes to transmit
- `--fixedVolume`: fixed-volume or fixed-time mode
- `--rateInterval`: adaptive rate update interval (seconds)

Current notes:
- the updated fixed-volume / handoff reporting path has been validated in the first-stage batch pipeline output set
- TCP receive buffer is set to 8 MB (`RcvBufSize` / `SndBufSize`) to prevent the window from capping effective throughput below Shannon capacity
- effective throughput is significantly lower than Shannon capacity due to TCP slow start; approximately 35% efficiency over the 110 ms transfer window for a 10 MB payload
- beamforming gain from CSV lookup is verified to increase effective throughput (752 → 784 Mbps) compared to analytical fallback (2128 → 2456 Mbps Shannon increase maps to ~4% application-layer gain, dominated by cwnd ramp-up)

---

## Repo layout

```text
repo/
├─ README.md
├─ docs/
│  ├─ PROJECT_STATUS.md
│  └─ SETUP_AND_IO.md
├─ ns3/
│  └─ contrib/leo/examples/
│     ├─ beam_pattern.csv
│     ├─ calculate_delay.cc
│     ├─ uav-vanet.cc
│     ├─ uav-link-probe.cc
│     ├─ uav-to-leo.cc
│     ├─ run_e2e_batch.sh
│     ├─ make_e2e_summary.py
│     ├─ convert_scheduler_to_vanet.py
│     ├─ wscript
│     ├─ schedule_trace.json
│     ├─ topo1.json
│     ├─ topo2.json
│     ├─ scheduler_trace.json
│     ├─ topology_summary.json
│     └─ scheduler_trace_vanet.json
├─ pipeline_output/
│  └─ successful first-stage E2E batch output set
├─ outputs/
│  ├─ output_schedule_vv12.txt
│  └─ output_topo1_vv13.txt
└─ legacy/
```

Notes:

- `legacy/` can remain empty for now
- only stable and representative files need to be committed first
- old debug outputs / half-broken intermediate versions can be added later if needed

---

## First-stage E2E validation output

A successful batch output set is stored under `pipeline_output/`.
The validated flow is:

```text
uav-vanet -> master_buffer_trace.csv -> uav-to-leo -> pipeline_summary.csv
```

Current successful run summary:
- `uav-vanet`: `tasksDone=104/104`, `allDone=YES`, `metMissionDeadline=YES`
- Master buffer handoff: `totalMasterArrivedBytes = 149618`
- `uav-to-leo`: `leo_total_rx_bytes = 149618`, `leo_done=YES`

This is still a CSV-based batch handoff, not a fully synchronized single NS-3 simulator.

---

## Build

From the NS-3 root:

```bash
./waf build
```

---

## Example runs

### `calculate_delay`
```bash
./waf --run calculate_delay
```

### `uav-vanet` with `topo1.json`
```bash
./waf --run "uav-vanet \
  --scheduleFile=/home/demo/ns3/ns-3.35/contrib/leo/examples/topo1.json \
  --simTime=9000 \
  --uavAltitude=50 \
  --accessRange3d=250 \
  --backhaulRange3d=1500 \
  --wifiFreqHz=2490000000 \
  --txPowerDbm=20 \
  --backhaulBandwidthHz=20000000 \
  --backhaulNoiseFigureDb=6 \
  --taskChunkBytes=1024 \
  --taskGapUs=5000 \
  --outFile=/home/demo/Desktop/output_topo1_v1.txt"
```

### `uav-vanet` with `topo1.json` with output format of MASTER UAV
```bash
./waf --run "uav-vanet \
  --scheduleFile=/home/demo/ns3/ns-3.35/contrib/leo/examples/topo1.json \
  --simTime=9000 \
  --uavAltitude=50 \
  --accessRange3d=250 \
  --backhaulRange3d=1500 \
  --wifiFreqHz=2490000000 \
  --txPowerDbm=20 \
  --backhaulBandwidthHz=20000000 \
  --backhaulNoiseFigureDb=6 \
  --taskChunkBytes=1024 \
  --taskGapUs=5000 \
  --masterUavId=-1 \
  --masterTraceFile=/home/demo/Desktop/master_buffer_trace.csv \
  --outFile=/home/demo/Desktop/output_masterbuf_topo1.txt"
```

### `uav-link-probe`
```bash
./waf --run "uav-link-probe \
  --uavPos='0,0;300,0;600,0;900,0;1200,0' \
  --masterUavId=2 \
  --probeAllSlavesToMaster=1 \
  --uavAltitude=50 \
  --backhaulRange3d=1500 \
  --wifiFreqHz=2490000000 \
  --txPowerDbm=20 \
  --backhaulBandwidthHz=20000000 \
  --backhaulNoiseFigureDb=6 \
  --packetSize=1000 \
  --intervalUs=20000 \
  --probeDurationSec=1.0 \
  --pairGapSec=2.0 \
  --outFile=/home/demo/Desktop/uav_link_probe.txt \
  --csvOutFile=/home/demo/Desktop/uav_link_probe.csv"
```

### `uav-to-leo`
```bash
./waf --run "uav-to-leo \
    --orbitFile:       CSV file with orbit parameters []
    --traceFile:       CSV file to redirect stdout to []
    --precision:       The time precision with which to compute position updates. 0 means arbitrary precision (ns3::LeoCircularOrbitMobilityModel::Precision) [+1e+09ns]
    --duration:        Simulation duration (seconds) [300]
    --uavLat:          UAV latitude  (degrees N) [24.8]
    --uavLon:          UAV longitude (degrees E) [120.97]
    --uavAlt:          UAV altitude  (meters ASL) [300]
    --band:            Sat band: Ku-User|Ka-Gateway|Ka-User|S-band [Ku-User]
    --targetSatIndex:  Satellite index (-1 = auto-closest) [-1]
    --maxBytes:        Total bytes to send (0 = unlimited) [10485760]
    --sendSize:        TCP segment size (bytes) [1024]
    --ttlThresh:       AODV TTL threshold [0]
    --routeTimeout:    AODV ActiveRouteTimeout (seconds) [300]
    --rateInterval:    Adaptive rate update interval (seconds) [10]
    --bpFile:          MATLAB beam pattern CSV file []
    --nAnt:            Number of antenna elements (1/4/16/64) [16]
    --nRF:             Number of RF chains (hybrid: nRF <= nAnt) [4]
    --destOnly:        Indicates only the destination may respond to this RREQ. (ns3::aodv::RoutingProtocol::DestinationOnly) [false]
    --pcap:            Enable PCAP packet capture [false]
    --fixedVolume:     Stop simulation when maxBytes received (default true; pass false to run full duration) [true]"
```

---

## Notes

- The JSON schedule is treated as the source of truth.
- Newer code versions automatically resize subnets for larger topologies.
- The current code is best understood as the **NS-3-side verifier**, not yet the full end-to-end scheduler.

---

## References

- course Lab2 setup / LEO environment
- project thesis slides: *Hierarchical Deadline-Aware Data Collection in UAV-Assisted SAGIN*
- `calculate_delay.cc`
- `uav-vanet.cc`


### First-stage E2E batch pipeline

The helper scripts are currently placed under `contrib/leo/examples/`.
`wscript` does not need to register `.sh` or `.py` files; it only needs the C++ programs such as `uav-vanet`, `uav-link-probe`, and `uav-to-leo`.

```bash
cd ~/ns3/ns-3.35
chmod +x contrib/leo/examples/run_e2e_batch.sh
chmod +x contrib/leo/examples/make_e2e_summary.py
./contrib/leo/examples/run_e2e_batch.sh contrib/leo/examples/topo1.json topo1
```

Expected output directory:

```text
outputs/e2e_topo1_<timestamp>/
├─ output_vanet.txt
├─ master_buffer_trace.csv
├─ output_uav_to_leo.txt
└─ pipeline_summary.csv
```

Current known issue:
- `uav-vanet` output and `master_buffer_trace.csv` are generated correctly in the current tests.
- `uav-to-leo` may still run for too long in the full batch pipeline; the fixed-volume stop condition / dynamic handoff behavior is under debugging.

