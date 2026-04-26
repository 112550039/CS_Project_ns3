# SETUP_AND_IO

## 1. Environment

Current working environment:

- Ubuntu
- NS-3.35
- leo module
- source files under `ns3/contrib/leo/examples/`

Important files:
- `calculate_delay.cc`
- `uav-vanet.cc`
- `wscript`
- `schedule_trace.json`
- `topo1.json`
- `topo2.json`
- `uav-link-probe.cc`

---

## 2. Build

From the NS-3 root directory:

```bash
./waf build
```

Optional configure step if needed:

```bash
./waf configure --enable-examples --enable-tests --enable-mpi --disable-werror
```

---

## 3. Run commands

### `calculate_delay`
```bash
./waf --run calculate_delay
```

### `uav-vanet` with simple trace
```bash
./waf --run "uav-vanet \
  --scheduleFile=/home/demo/ns3/ns-3.35/contrib/leo/examples/schedule_trace.json \
  --simTime=4200 \
  --uavAltitude=50 \
  --accessRange3d=250 \
  --backhaulRange3d=1000 \
  --wifiFreqHz=2490000000 \
  --txPowerDbm=20 \
  --backhaulBandwidthHz=20000000 \
  --backhaulNoiseFigureDb=6 \
  --taskChunkBytes=1024 \
  --taskGapUs=3000 \
  --outFile=/home/demo/Desktop/output_schedule.txt"
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

### Suggested first run for `topo2.json`
```bash
./waf --run "uav-vanet \
  --scheduleFile=/home/demo/ns3/ns-3.35/contrib/leo/examples/topo2.json \
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
  --outFile=/home/demo/Desktop/output_topo2_v1.txt"
```

### `uav-link-probe` (UAV-to-UAV probing)
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
  --outFile=/home/demo/Desktop/uav_link_probe_v1.txt \
  --csvOutFile=/home/demo/Desktop/uav_link_probe_v1.csv"

---

## 4. Current JSON schema (high level)

Each schedule JSON currently follows this general structure:

```json
{
  "mission_deadline_sec": ...,
  "nodes": {
    "uavs": ...,
    "sensors": [...]
  },
  "events": [...]
}
```

Supported event types:

- `MOVE`
- `ROLE_SET`
- `SENSOR_TO_UAV`
- `SLAVE_TO_MASTER`

The current code treats the schedule file as the source of truth for:
- UAV count
- sensor count
- event timeline
- transfer tasks

---

## 5. Important parameters

### `--scheduleFile`
Path to the JSON schedule.

### `--uavAltitude`
Default UAV altitude for initialization.

### `--accessRange3d`
3D communication range between sensors and UAVs.

### `--backhaulRange3d`
3D communication range between UAVs.

### `--taskChunkBytes`
Chunk size used by the replay sender.

### `--taskGapUs`
Gap between replay chunks.
Smaller value = more aggressive offered load.

### `--outFile`
Output log path.

### `uav-link-probe` parameters

#### `--uavPos`
Semicolon-separated UAV coordinates, e.g. `x,y;x,y;...`

#### `--masterUavId`
Defines which UAV is treated as the master / CH in probing mode.

#### `--packetSize`
Probe packet payload size.

#### `--intervalUs`
Gap between probe packets.
Smaller value = more aggressive offered load.

#### `--probeDurationSec`
How long each probing flow lasts.

#### `--csvOutFile`
Optional CSV output path for downstream use.

---

## 6. What the code automatically adapts to

The newer `uav-vanet.cc` will automatically adapt to:

- number of UAVs from the JSON
- number of sensors from the JSON
- required simulation time according to task deadlines
- subnet size according to host count

This is why changing only `--scheduleFile` is usually enough for the first test of a new same-schema topology.

---

## 7. What to inspect in output logs

For a healthy run, first look for:

- `[SCHEDULE]`
- `[IP-PLAN]`
- `[ReplayMode]`
- `[TASK-DONE]`
- `[MISSION]`

Quick interpretation:

- `[SCHEDULE]` → file parsed correctly
- `[IP-PLAN]` → subnet sizing is correct
- `[ReplayMode]` → task counts and pacing are correct
- `[TASK-DONE]` → replay tasks are actually completing
- `[MISSION]` → final success / finish times / deadline results

---

## 8. Practical sync workflow

Because development is often done inside a VM, a simple workflow is:

1. modify / test inside the VM
2. manually copy only selected files back to the host
3. organize them into the repo structure
4. commit from the host-side Git repo

This is fine as long as:
- file names are consistent
- only intended files are copied
- temporary junk is not mixed into the repo root
