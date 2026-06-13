#!/usr/bin/env python3
import argparse
import json
import re


def trailing_index(name: str) -> int:
    m = re.search(r"_(\d+)$", name)
    if not m:
        raise ValueError(f"bad id format: {name}")
    return int(m.group(1))


def sensor_name(name: str) -> str:
    # TA format: S_51
    # uav-vanet format: Sensor_51
    if name.startswith("S_"):
        return "Sensor_" + name.split("_", 1)[1]
    return name


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scheduler", required=True)
    ap.add_argument("--topology", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--uav-alt", type=float, default=50.0)
    args = ap.parse_args()

    with open(args.scheduler, "r", encoding="utf-8") as f:
        sched = json.load(f)

    with open(args.topology, "r", encoding="utf-8") as f:
        topo = json.load(f)

    events = sched["events"]
    fleet_size = int(topo.get("fleet_size", 0))
    mission_deadline_sec = float(topo.get("mission_deadline_min", 90.0)) * 60.0

    # cluster centroid map
    cluster_pos = {}
    for c in topo.get("clusters", []):
        cluster_pos[int(c["cluster_id"])] = (
            float(c["centroid_x"]),
            float(c["centroid_y"]),
        )

    # infer sensors
    max_sensor_id = -1
    sensor_cluster = {}
    for ev in events:
        if str(ev.get("src", "")).startswith("S_"):
            sid = trailing_index(ev["src"])
            max_sensor_id = max(max_sensor_id, sid)
            sensor_cluster.setdefault(sid, int(ev.get("cluster_id", 0)))

    sensors = []
    for sid in range(max_sensor_id + 1):
        cid = sensor_cluster.get(sid, None)
        if cid in cluster_pos:
            x, y = cluster_pos[cid]
        else:
            x, y = 0.0, 0.0
        sensors.append({
            "id": f"Sensor_{sid}",
            "x": x,
            "y": y
        })

    out_events = []

    # move all UAVs to each visited cluster centroid at cluster arrival time.
    # This is a compatibility replay layout, not the final physical path model.
    for visit in sched.get("cluster_visits", []):
        cid = int(visit["cluster_id"])
        if cid not in cluster_pos:
            continue

        x, y = cluster_pos[cid]
        t = float(visit["arrival_time_sec"])

        for u in range(fleet_size):
            out_events.append({
                "time": t,
                "type": "MOVE",
                "node_id": f"UAV_{u}",
                "target_pos": [x, y, args.uav_alt]
            })

    # role + transfer events
    next_role_id = 0
    for ev in events:
        link_type = ev["link_type"]
        if link_type == "satellite":
            # uav-vanet only handles Sensor/Slave -> Master.
            # Master -> LEO is handled by uav-to-leo later.
            continue

        start = float(ev["start_time_sec"])
        deadline = float(ev["deadline_sec"])
        payload = int(ev["payload_bytes"])

        src = ev["src"]
        dst = ev["dst"]
        src_role = ev.get("src_role", "")
        dst_role = ev.get("dst_role", "")

        # Add ROLE_SET slightly before transfer start.
        role_time = max(0.0, start - 1e-6)

        if src.startswith("UAV_"):
            out_events.append({
                "time": role_time,
                "type": "ROLE_SET",
                "node_id": src,
                "role": src_role
            })

        if dst.startswith("UAV_"):
            out_events.append({
                "time": role_time,
                "type": "ROLE_SET",
                "node_id": dst,
                "role": dst_role
            })

        if link_type == "access":
            out_events.append({
                "time": start,
                "type": "SENSOR_TO_UAV",
                "src_id": sensor_name(src),
                "dst_id": dst,
                "data_size_bytes": payload,
                "deadline_sec": deadline
            })
        elif link_type == "relay":
            out_events.append({
                "time": start,
                "type": "SLAVE_TO_MASTER",
                "src_id": src,
                "dst_id": dst,
                "data_size_bytes": payload,
                "deadline_sec": deadline
            })

    out_events.sort(key=lambda e: float(e["time"]))

    converted = {
        "mission_deadline_sec": mission_deadline_sec,
        "nodes": {
            "uavs": fleet_size,
            "sensors": sensors,
            "satellites": 1
        },
        "events": out_events
    }

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(converted, f, indent=2)

    print(f"[OK] wrote {args.out}")
    print(f"     uavs={fleet_size}")
    print(f"     sensors={len(sensors)}")
    print(f"     events={len(out_events)}")
    print(f"     mission_deadline_sec={mission_deadline_sec}")


if __name__ == "__main__":
    main()
