# UAV-to-LEO — Integration & Execution Guide

> Audience: teammate working on `uav-vanet.cc`. This explains what
> `uav-to-leo.cc` does, how it consumes your `master_buffer_trace.csv`, and
> exactly how to build and run it.

---

## 1. Where this fits in the pipeline

Two simulators stay **separate**; the only interface between them is one CSV.

```
  uav-vanet.cc  ──writes──▶  outputs/master_buffer_trace.csv  ──read by──▶  uav-to-leo.cc
  (your part:                 (cumulative master-UAV buffer        (this part:
   sensors→UAV,                over sim time)                       UAV→LEO uplink,
   slave→master)                                                    link budget, handoff)
```

- **`uav-vanet.cc`** (you): simulates sensors → UAVs → master/CH UAV, then
  writes `master_buffer_trace.csv` — a row per data chunk arriving at the
  master, with the running `masterBufferBytes` it has accumulated.
- **`uav-to-leo.cc`** (this file): reads that CSV to decide **when** the master
  has enough buffered data to start uploading to LEO, then simulates the
  TCP uplink with adaptive rate, beamforming, and satellite handoff.

---

## 2. What `uav-to-leo.cc` does

High-level flow each run:

1. **Build a LEO constellation** (default: a Telesat-like shell, 264 sats) and
   place one stationary **UAV** at a lat/lon/alt.
2. **Pick the initial satellite** — the closest one overhead at `t=0`.
3. **(NEW) Decide the upload start time** from your CSV (see §3). Without a CSV,
   it starts at `t=0` exactly as before.
4. **Run a fixed-volume TCP upload** (`--maxBytes`) from the UAV to the chosen
   satellite. A link budget (FSPL + beamforming gain → SNR → Shannon capacity)
   sets the link data rate, updated every `--rateInterval` seconds.
5. **Handoff automatically**: when the serving satellite drops below the band's
   elevation cutoff (link down), it scans for the best visible satellite and
   re-points the upload. Each UAV↔satellite connection is one **window**.
6. **Report** per-window throughput, delay, efficiency, and write CSV outputs.

---

## 3. The CSV integration (the feature you care about)

When you pass `--csvFile`, `uav-to-leo` does this **before** the simulation:

1. Reads `master_buffer_trace.csv` and scans the `masterBufferBytes` column.
2. Finds the **first row where `masterBufferBytes > --bufferThreshold`**.
3. Uses that row's `timeSec` as the **upload trigger time** — the upload (and
   all rate/handoff logic) is deferred to start at that simulated second.
4. If **no row** ever crosses the threshold, it does **not** abort — it falls
   back to the **last row's `timeSec`** and uploads then. (The threshold is a
   "wait until enough is buffered" gate, not a kill switch.)

**Why re-pick the satellite at trigger time?** The `t=0` closest satellite has
usually flown out of view by, say, `t=320s`. So at the trigger moment the code
re-selects the best *currently visible* satellite and starts the upload against
it. You'll see this in the log as `re-selected target Sat[X] -> Sat[Y]`.

`--maxBytes` (how much to upload) is unchanged and independent of the threshold.

### CSV schema this reads (must match what `uav-vanet.cc` writes)

Column order matters — the parser reads **column 0** (`timeSec`) and
**column 7** (`masterBufferBytes`):

```
timeSec,event,taskId,type,src,dstMaster,bytesArrived,masterBufferBytes,masterArrivedBytes,totalMasterArrivedBytes
```

The header line is auto-detected (any line starting with `timeSec` is skipped).
Plain comma-separated, no quoting.

---

## 4. Build

From the ns-3 root (`ns-allinone-3.35/ns-3.35`):

```bash
./waf build
```

To rebuild only this program (faster):

```bash
./waf --target=uav-to-leo build
```

---

## 5. Run

### Normal way

```bash
./waf --run "uav-to-leo \
  --csvFile=outputs/master_buffer_trace.csv \
  --bufferThreshold=10000 \
  --duration=1000 \
  --bpFile=contrib/leo/examples/beam_pattern.csv"
```

### Fallback way (skip waf's program wrapper)

If `./waf --run` ever misbehaves, run the built binary directly:

```bash
./waf --target=uav-to-leo build
LD_LIBRARY_PATH=build/lib \
  ./build/contrib/leo/examples/ns3.35-uav-to-leo-debug \
  --csvFile=outputs/master_buffer_trace.csv --bufferThreshold=10000 --duration=1000
```

> **Note on `--duration`:** the trigger time must be **less than** `--duration`,
> otherwise there's no time left to upload (the program errors out and tells you
> to increase `--duration`). If your CSV triggers at ~320s, give it room, e.g.
> `--duration=1000` or more.

---

## 6. Key parameters

| Flag | Default | Meaning |
|---|---|---|
| `--csvFile` | *(empty)* | Path to `master_buffer_trace.csv`. **Empty = start upload at t=0** (original behavior). |
| `--bufferThreshold` | `10000` | Bytes. Upload triggers at the first CSV row where `masterBufferBytes` exceeds this. Never crossed → falls back to last row. |
| `--maxBytes` | `10485760` (10 MiB) | Total bytes to upload **per window**. Independent of the threshold. |
| `--duration` | `7200` | Simulation length (seconds). Must exceed the trigger time. |
| `--band` | `Ku-User` | `Ku-User \| Ka-Gateway \| Ka-User \| S-band`. Sets frequency, bandwidth, and **elevation cutoff** (decides link-down/handoff). |
| `--bpFile` | *(empty)* | MATLAB beam-pattern CSV. Without it, an analytical beamforming model is used. |
| `--nAnt` / `--nRF` | `16` / `4` | Antenna elements / RF chains (hybrid beamforming). |
| `--rateInterval` | `1.0` | Seconds between adaptive-rate updates / link-down checks. |
| `--fixedVolume` | `true` | `true`: stop a window once `--maxBytes` is received. `false`: run the full `--duration`. |
| `--uavLat`/`--uavLon`/`--uavAlt` | `24.80`/`120.97`/`300` | UAV position (deg N, deg E, m ASL). |
| `--targetSatIndex` | `-1` | Force a satellite index; `-1` = auto-closest. |

Run `./waf --run "uav-to-leo --PrintHelp"` for the full list.

---

## 7. End-to-end example

```bash
# 1. (your simulator) produce the buffer trace
#    -> must write outputs/master_buffer_trace.csv
./waf --run "uav-vanet ...your-args..."

# 2. upload phase: trigger once the master has buffered > 10 KB
./waf --run "uav-to-leo \
  --csvFile=outputs/master_buffer_trace.csv \
  --bufferThreshold=10000 \
  --duration=1000 \
  --bpFile=contrib/leo/examples/beam_pattern.csv"
```

---

## 8. Outputs

### Terminal — the new section

When `--csvFile` is given, the results include a dedicated block:

```
--- CSV Upload Trigger ---
Source CSV:          outputs/master_buffer_trace.csv
Buffer threshold:    10000 B
Trigger time:        320.006 s (threshold crossed)
Buffer at trigger:   10240 B
Initial (t=0) pick:  Sat[38]
Re-selected target:  Sat[37] (elev=46.84 deg, dist=1543.2 km)
```

Read it as: *"Your buffer passed 10 KB at 320.006 s. The t=0 satellite (38) was
no longer best by then, so we uploaded via satellite 37, which was 46.8° up."*

There's also a **per-window throughput table** showing each satellite
connection, its duration, bytes received, and effective Mbps.

### Files written to `outputs/`

| File | Contents |
|---|---|
| `csv_trigger.csv` | **NEW.** One row summarizing the trigger decision (schema below). Written only when `--csvFile` is used. |
| `visibility_windows.csv` | One row per UAV↔satellite window: id, start, end, avg effective throughput. |
| `adaptiveRateLog.csv` | Per-interval link state: time, distance, elevation, BF gain, SNR, rate, OK/DOWN. |
| `uav-to-leo_result.csv` | Single-row headline metrics (effective throughput, cutoff, visible window). |

#### `csv_trigger.csv` schema

```
source_csv,threshold_bytes,trigger_time_sec,threshold_crossed,buffer_at_trigger_bytes,initial_sat,selected_sat,selected_elev_deg,selected_dist_km
outputs/master_buffer_trace.csv,10000,320.006,1,10240,38,37,46.84,1543.2
```

- `threshold_crossed` = `1` if the threshold was actually exceeded, `0` if it
  fell back to the last CSV row.
- `initial_sat` vs `selected_sat` show the t=0 pick vs the satellite actually
  used at trigger time.

---

## 9. Troubleshooting

| Symptom | Cause / Fix |
|---|---|
| `[CSV] ERROR: cannot open '...'` then exit | Wrong `--csvFile` path. It's relative to the directory you run `./waf` from (the ns-3 root). |
| `ERROR: triggerTimeSec (...) >= duration (...)` | The buffer crosses the threshold later than the sim ends. Increase `--duration` or lower `--bufferThreshold`. |
| Window 1 shows `rx=0 B` / handoff fires immediately | The trigger satellite set just barely; the link dropped within one interval and handed off. Normal — later windows carry the data. |
| `json.decoder.JSONDecodeError ... compile_commands.json` during build | An earlier build was interrupted, leaving an empty `build/compile_commands.json`. Already hardened in `waf-tools/clang_compilation_database.py`; if it ever recurs, `rm build/compile_commands.json` and rebuild. |

---

## 10. Quick mental model

- **No `--csvFile`** → behaves exactly like the original single-shot uplink at t=0.
- **With `--csvFile`** → "wait until the master UAV has buffered more than
  `--bufferThreshold` bytes (per your trace), then upload `--maxBytes` over the
  best visible satellite, handing off as satellites set."
