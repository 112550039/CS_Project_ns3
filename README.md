# UAV-Assisted SAGIN Data Collection Simulation

This repo contains our NS-3 project for **UAV-assisted data collection**.

Current focus:

- replay an external JSON schedule in NS-3
- simulate `MOVE / ROLE_SET / SENSOR_TO_UAV / SLAVE_TO_MASTER`
- check whether all tasks finish before their deadlines
- report mission finish time, sensor finish time, and slave finish time

The current mainline is **`uav-vanet.cc`**.  
It is no longer the old random baseline simulator. It now acts as a **schedule-driven replay / verification engine**.

---

## Current status

Validated inputs so far:

- `schedule_trace.json` — simple topology
- `topo1.json` — larger topology
- `topo2.json` — next target

Current repo role:

- **implemented:** schedule replay / mission verification
- **not yet complete:** probing throughput mode, UAV-to-LEO integration, full scheduler feedback loop

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

### `ns3/contrib/leo/examples/calculate_delay.cc`
Earlier LEO / delay / SNR-rate line.  
Useful as the basis for future UAV-to-LEO integration.

### JSON inputs
- `schedule_trace.json`
- `topo1.json`
- `topo2.json`

These traces define:
- UAV count
- sensor positions
- mission deadline
- event list (`MOVE`, `ROLE_SET`, `SENSOR_TO_UAV`, `SLAVE_TO_MASTER`)

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
│     ├─ calculate_delay.cc
│     ├─ uav-vanet.cc
│     ├─ wscript
│     ├─ schedule_trace.json
│     ├─ topo1.json
│     └─ topo2.json
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
