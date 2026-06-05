#!/usr/bin/env python3
import argparse
import csv
import os
import re


def read_text(path: str) -> str:
    if not path or not os.path.exists(path):
        return ""
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        return f.read()


def extract(pattern: str, text: str, default: str = "") -> str:
    m = re.search(pattern, text)
    if not m:
        return default
    return m.group(1)


def parse_master_trace(path: str) -> dict:
    result = {
        "first_master_time_sec": "",
        "last_master_time_sec": "",
        "total_master_arrived_bytes": "0",
        "last_master_buffer_bytes": "0",
        "rows": "0",
    }

    if not path or not os.path.exists(path):
        return result

    rows = 0
    first_time = ""
    last_time = ""
    last_total = "0"
    last_buffer = "0"

    with open(path, "r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows += 1
            time_sec = row.get("timeSec", "")
            if rows == 1:
                first_time = time_sec
            last_time = time_sec
            last_total = row.get("totalMasterArrivedBytes", last_total)
            last_buffer = row.get("masterBufferBytes", last_buffer)

    result["first_master_time_sec"] = first_time
    result["last_master_time_sec"] = last_time
    result["total_master_arrived_bytes"] = last_total
    result["last_master_buffer_bytes"] = last_buffer
    result["rows"] = str(rows)
    return result


def parse_csv_trigger(path: str) -> dict:
    result = {
        "trigger_time_sec": "",
        "threshold_bytes": "",
        "threshold_crossed": "",
        "buffer_at_trigger_bytes": "",
        "selected_sat": "",
        "selected_elev_deg": "",
        "selected_dist_km": "",
    }

    if not path or not os.path.exists(path):
        return result

    with open(path, "r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            result["trigger_time_sec"] = row.get("trigger_time_sec", "")
            result["threshold_bytes"] = row.get("threshold_bytes", "")
            result["threshold_crossed"] = row.get("threshold_crossed", "")
            result["buffer_at_trigger_bytes"] = row.get("buffer_at_trigger_bytes", "")
            result["selected_sat"] = row.get("selected_sat", "")
            result["selected_elev_deg"] = row.get("selected_elev_deg", "")
            result["selected_dist_km"] = row.get("selected_dist_km", "")
            break

    return result


def parse_leo_result(path: str) -> dict:
    result = {
        "leo_effective_throughput_mbps": "",
        "leo_visible_start_sec": "",
        "leo_visible_end_sec": "",
    }

    if not path or not os.path.exists(path):
        return result

    with open(path, "r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Header from uav-to-leo_result.csv:
            # Effective Throughput (Mbps),Elevation Cutoff (Deg),
            # Visible Time Window Start (s),Visible Time Window End (s)
            result["leo_effective_throughput_mbps"] = row.get("Effective Throughput (Mbps)", "")
            result["leo_visible_start_sec"] = row.get("Visible Time Window Start (s)", "")
            result["leo_visible_end_sec"] = row.get("Visible Time Window End (s)", "")
            break

    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vanet-log", required=True)
    ap.add_argument("--master-trace", required=True)
    ap.add_argument("--leo-log", required=True)
    ap.add_argument("--csv-trigger", required=True)
    ap.add_argument("--leo-result", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    vanet_text = read_text(args.vanet_log)
    leo_text = read_text(args.leo_log)

    master = parse_master_trace(args.master_trace)
    trig = parse_csv_trigger(args.csv_trigger)
    leo_result = parse_leo_result(args.leo_result)

    # Parse uav-vanet mission summary
    # Example:
    # [MISSION] tasksTotal=212 tasksDone=212 allDone=YES ...
    tasks_total = extract(r"tasksTotal=([0-9]+)", vanet_text, "")
    tasks_done = extract(r"tasksDone=([0-9]+)", vanet_text, "")
    all_done = extract(r"allDone=([A-Z]+)", vanet_text, "")
    finish_sec = extract(r"finishSec=([0-9.]+)", vanet_text, "")
    sensor_done = extract(r"sensorDone=([0-9]+/[0-9]+)", vanet_text, "")
    slave_done = extract(r"slaveDone=([0-9]+/[0-9]+)", vanet_text, "")
    sensor_finish_sec = extract(r"sensorFinishSec=([0-9.]+)", vanet_text, "")
    slave_finish_sec = extract(r"slaveFinishSec=([0-9.]+)", vanet_text, "")
    met_deadline = extract(r"metMissionDeadline=([A-Z]+)", vanet_text, "")

    # Parse uav-to-leo terminal output
    leo_total_rx = extract(r"Bytes received:\s*([0-9]+)", leo_text, "")
    leo_eff = extract(r"Effective throughput:\s*([0-9.]+)", leo_text, "")
    leo_first_tx = extract(r"First Tx time:\s*([0-9.]+)", leo_text, "")
    leo_last_rx = extract(r"Last\s+Rx time:\s*([0-9.]+)", leo_text, "")

    if not leo_eff:
        leo_eff = leo_result["leo_effective_throughput_mbps"]

    # Simple upload completion check
    try:
        leo_done = int(leo_total_rx) >= int(master["total_master_arrived_bytes"])
    except Exception:
        leo_done = False

    end_to_end_finish = leo_last_rx if leo_done and leo_last_rx else ""

    fieldnames = [
        "tasks_total",
        "tasks_done",
        "all_done",
        "sensor_done",
        "slave_done",
        "vanet_finish_sec",
        "sensor_finish_sec",
        "slave_finish_sec",
        "met_mission_deadline",

        "master_trace_rows",
        "first_master_time_sec",
        "last_master_time_sec",
        "total_master_arrived_bytes",
        "last_master_buffer_bytes",

        "leo_trigger_time_sec",
        "leo_threshold_bytes",
        "leo_threshold_crossed",
        "leo_buffer_at_trigger_bytes",
        "leo_selected_sat",
        "leo_selected_elev_deg",
        "leo_selected_dist_km",

        "leo_total_rx_bytes",
        "leo_first_tx_sec",
        "leo_last_rx_sec",
        "leo_effective_throughput_mbps",
        "leo_visible_start_sec",
        "leo_visible_end_sec",
        "leo_done",
        "end_to_end_finish_sec",
    ]

    row = {
        "tasks_total": tasks_total,
        "tasks_done": tasks_done,
        "all_done": all_done,
        "sensor_done": sensor_done,
        "slave_done": slave_done,
        "vanet_finish_sec": finish_sec,
        "sensor_finish_sec": sensor_finish_sec,
        "slave_finish_sec": slave_finish_sec,
        "met_mission_deadline": met_deadline,

        "master_trace_rows": master["rows"],
        "first_master_time_sec": master["first_master_time_sec"],
        "last_master_time_sec": master["last_master_time_sec"],
        "total_master_arrived_bytes": master["total_master_arrived_bytes"],
        "last_master_buffer_bytes": master["last_master_buffer_bytes"],

        "leo_trigger_time_sec": trig["trigger_time_sec"],
        "leo_threshold_bytes": trig["threshold_bytes"],
        "leo_threshold_crossed": trig["threshold_crossed"],
        "leo_buffer_at_trigger_bytes": trig["buffer_at_trigger_bytes"],
        "leo_selected_sat": trig["selected_sat"],
        "leo_selected_elev_deg": trig["selected_elev_deg"],
        "leo_selected_dist_km": trig["selected_dist_km"],

        "leo_total_rx_bytes": leo_total_rx,
        "leo_first_tx_sec": leo_first_tx,
        "leo_last_rx_sec": leo_last_rx,
        "leo_effective_throughput_mbps": leo_eff,
        "leo_visible_start_sec": leo_result["leo_visible_start_sec"],
        "leo_visible_end_sec": leo_result["leo_visible_end_sec"],
        "leo_done": "YES" if leo_done else "NO",
        "end_to_end_finish_sec": end_to_end_finish,
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(row)

    print(f"[OK] wrote summary: {args.out}")


if __name__ == "__main__":
    main()
