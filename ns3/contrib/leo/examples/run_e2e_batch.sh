#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# End-to-End Batch Pipeline
#
# uav-vanet
#   -> master_buffer_trace.csv
#   -> uav-to-leo
#   -> pipeline_summary.csv
#
# Run from ns-3 root:
#   ./scripts/run_e2e_batch.sh contrib/leo/examples/topo1.json topo1
# ============================================================

NS3_ROOT="${NS3_ROOT:-$(pwd)}"
EXAMPLE_DIR="${EXAMPLE_DIR:-$NS3_ROOT/contrib/leo/examples}"

SCHEDULE_FILE="${1:-$EXAMPLE_DIR/topo1.json}"
RUN_NAME="${2:-$(basename "$SCHEDULE_FILE" .json)}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR:-$NS3_ROOT/outputs/e2e_${RUN_NAME}_${TIMESTAMP}}"

mkdir -p "$OUT_DIR"

MASTER_TRACE="$OUT_DIR/master_buffer_trace.csv"
VANET_LOG="$OUT_DIR/output_vanet.txt"
LEO_LOG="$OUT_DIR/output_uav_to_leo.txt"
SUMMARY_CSV="$OUT_DIR/pipeline_summary.csv"

# -----------------------------
# uav-vanet parameters
# -----------------------------
SIM_TIME="${SIM_TIME:-9000}"
UAV_ALTITUDE="${UAV_ALTITUDE:-50}"
ACCESS_RANGE_3D="${ACCESS_RANGE_3D:-250}"
BACKHAUL_RANGE_3D="${BACKHAUL_RANGE_3D:-1500}"
WIFI_FREQ_HZ="${WIFI_FREQ_HZ:-2490000000}"
TX_POWER_DBM="${TX_POWER_DBM:-20}"
BACKHAUL_BW_HZ="${BACKHAUL_BW_HZ:-20000000}"
BACKHAUL_NOISE_FIGURE_DB="${BACKHAUL_NOISE_FIGURE_DB:-6}"
TASK_CHUNK_BYTES="${TASK_CHUNK_BYTES:-1024}"
TASK_GAP_US="${TASK_GAP_US:-5000}"

# -1 = follow ROLE_SET in scheduler JSON
MASTER_UAV_ID="${MASTER_UAV_ID:--1}"

# -----------------------------
# uav-to-leo parameters
# -----------------------------
LEO_DURATION="${LEO_DURATION:-12000}"

# These are temporary / demo defaults.
# Later you can align them with the actual scenario.
UAV_LAT="${UAV_LAT:-24.80}"
UAV_LON="${UAV_LON:-120.97}"
UAV_ALT="${UAV_ALT:-300}"

BAND="${BAND:-Ku-User}"
TARGET_SAT_INDEX="${TARGET_SAT_INDEX:--1}"
SEND_SIZE="${SEND_SIZE:-1024}"
RATE_INTERVAL="${RATE_INTERVAL:-1.0}"
FIXED_VOLUME="${FIXED_VOLUME:-true}"

# 0 means upload can trigger once masterBufferBytes > 0.
# Increase this to model "master collects for a while before uploading".
BUFFER_THRESHOLD="${BUFFER_THRESHOLD:-0}"

echo "========== E2E Batch Pipeline =========="
echo "[INFO] NS3_ROOT       = $NS3_ROOT"
echo "[INFO] SCHEDULE_FILE  = $SCHEDULE_FILE"
echo "[INFO] RUN_NAME       = $RUN_NAME"
echo "[INFO] OUT_DIR        = $OUT_DIR"
echo "========================================"

cd "$NS3_ROOT"

echo
echo "[1/5] Build ns-3 programs"
./waf build

echo
echo "[2/5] Run uav-vanet: Sensor/Slave -> Master"
./waf --run "uav-vanet \
  --scheduleFile=$SCHEDULE_FILE \
  --simTime=$SIM_TIME \
  --uavAltitude=$UAV_ALTITUDE \
  --accessRange3d=$ACCESS_RANGE_3D \
  --backhaulRange3d=$BACKHAUL_RANGE_3D \
  --wifiFreqHz=$WIFI_FREQ_HZ \
  --txPowerDbm=$TX_POWER_DBM \
  --backhaulBandwidthHz=$BACKHAUL_BW_HZ \
  --backhaulNoiseFigureDb=$BACKHAUL_NOISE_FIGURE_DB \
  --taskChunkBytes=$TASK_CHUNK_BYTES \
  --taskGapUs=$TASK_GAP_US \
  --masterUavId=$MASTER_UAV_ID \
  --masterTraceFile=$MASTER_TRACE \
  --outFile=$VANET_LOG"

echo
echo "[3/5] Extract total bytes arrived at Master"

if [[ ! -s "$MASTER_TRACE" ]]; then
  echo "[ERR] master trace not found or empty: $MASTER_TRACE"
  exit 1
fi

TOTAL_MASTER_BYTES="$(awk -F, '
  NR > 1 && $10 ~ /^[0-9]+$/ {
    last = $10
  }
  END {
    if (last == "") print 0;
    else print last;
  }
' "$MASTER_TRACE")"

if [[ "$TOTAL_MASTER_BYTES" -le 0 ]]; then
  echo "[ERR] totalMasterArrivedBytes is zero. Check $MASTER_TRACE"
  exit 1
fi

echo "[INFO] totalMasterArrivedBytes = $TOTAL_MASTER_BYTES bytes"

echo
echo "[4/5] Run uav-to-leo: Master -> LEO"

./waf --run "uav-to-leo \
  --duration=$LEO_DURATION \
  --uavLat=$UAV_LAT \
  --uavLon=$UAV_LON \
  --uavAlt=$UAV_ALT \
  --band=$BAND \
  --targetSatIndex=$TARGET_SAT_INDEX \
  --maxBytes=$TOTAL_MASTER_BYTES \
  --sendSize=$SEND_SIZE \
  --rateInterval=$RATE_INTERVAL \
  --fixedVolume=$FIXED_VOLUME \
  --csvFile=$MASTER_TRACE \
  --bufferThreshold=$BUFFER_THRESHOLD \
  --traceFile=$LEO_LOG"

echo
echo "[INFO] Copy uav-to-leo generated CSV outputs"

for f in \
  "$NS3_ROOT/outputs/adaptiveRateLog.csv" \
  "$NS3_ROOT/outputs/uav-to-leo_result.csv" \
  "$NS3_ROOT/outputs/visibility_windows.csv" \
  "$NS3_ROOT/outputs/csv_trigger.csv"
do
  if [[ -f "$f" ]]; then
    cp -f "$f" "$OUT_DIR/$(basename "$f")"
  fi
done

echo
echo "[5/5] Generate pipeline summary"

python3 "$EXAMPLE_DIR/make_e2e_summary.py" \
  --vanet-log "$VANET_LOG" \
  --master-trace "$MASTER_TRACE" \
  --leo-log "$LEO_LOG" \
  --csv-trigger "$OUT_DIR/csv_trigger.csv" \
  --leo-result "$OUT_DIR/uav-to-leo_result.csv" \
  --out "$SUMMARY_CSV"

echo
echo "========== DONE =========="
echo "[OUT] vanet log:        $VANET_LOG"
echo "[OUT] master trace:     $MASTER_TRACE"
echo "[OUT] leo log:          $LEO_LOG"
echo "[OUT] summary:          $SUMMARY_CSV"
echo "=========================="
