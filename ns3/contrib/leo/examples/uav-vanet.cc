/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Simple UAV + ground VANET example.
 *
 *  - nUav 架在空中的 UAV 節點，使用 ConstantVelocity 移動模型
 *  - nGround 地面節點，使用 RandomWaypoint 在平面區域上隨機移動
 *  - 所有節點用 802.11n ad-hoc Wi-Fi 做通訊
 *  - IP 上跑 AODV routing
 *  - 一條 UDP echo flow：ground node 0 -> UAV node 0
 *
 *  之後要量 SNR / rate，可以在這個骨架上接 FlowMonitor / PHY trace。
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/aodv-module.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/packet-sink.h"

#include <fstream>
#include <vector>
#include <limits>
#include <cmath>
#include <map>
#include <string>
#include <sstream>
#include <algorithm>
#include "nlohmann/json.hpp"

using namespace ns3;

using json = nlohmann::json;

struct Point2D
{
  double x;
  double y;
};

struct MoveEvent
{
  double timeSec;
  uint32_t uavId;
  Vector target;
};

struct RoleSetEvent
{
  double timeSec;
  uint32_t uavId;
  std::string role;
};

struct TransferEvent
{
  uint32_t taskId;
  std::string type;     // SENSOR_TO_UAV / SLAVE_TO_MASTER
  double timeSec;
  std::string srcName;  // Sensor_0 / UAV_0 ...
  std::string dstName;
  uint32_t srcIndex;    // sensor index 或 uav index
  uint32_t dstIndex;
  uint64_t bytes;
  double deadlineSec;
};

struct ScheduleTrace
{
  double missionDeadlineSec = 0.0;
  uint32_t nUavs = 0;
  std::vector<Point2D> sensors;
  std::vector<MoveEvent> moves;
  std::vector<RoleSetEvent> roles;
  std::vector<TransferEvent> transfers;
  double lastEventTimeSec = 0.0;
};

struct ReplayTaskState
{
  uint32_t id = 0;
  std::string type;     // SENSOR_TO_UAV / SLAVE_TO_MASTER
  std::string srcName;
  std::string dstName;
  uint32_t srcIndex = 0;
  uint32_t dstIndex = 0;

  uint64_t targetBytes = 0;
  uint64_t txBytes = 0;
  uint64_t rxBytes = 0;

  double startSec = 0.0;
  double deadlineSec = 0.0;

  bool started = false;
  bool done = false;
  double finishSec = -1.0;
};

static std::map<uint32_t, ReplayTaskState> g_replayTasks;
static std::map<uint32_t, uint64_t> g_taskRemainingBytes;

// 每台 UAV 目前「已經從 sensor 收到、可再往上送」的 buffer bytes
static std::map<uint32_t, uint64_t> g_uavBufferedBytes;

// 每台 Master/CH UAV 目前已經收集好、準備往 LEO 上傳的 buffer bytes
static std::map<uint32_t, uint64_t> g_masterBufferedBytes;

// 每台 Master/CH UAV 累積收到過的總 bytes，之後可用來和 LEO upload total 對照
static std::map<uint32_t, uint64_t> g_masterArrivedBytes;

// 所有 Master/CH 累積收到過的總 bytes
static uint64_t g_totalMasterArrivedBytes = 0;

// 若 >= 0，強制指定某台 UAV 是 master。
// 若 = -1，則依 ROLE_SET 中的 CH / MASTER 判斷。
static int32_t g_forcedMasterUavId = -1;

// Master buffer trace CSV
static std::string g_masterTraceFile = "";
static std::ofstream g_masterTraceCsv;

// 角色紀錄（ROLE_SET）
static std::map<uint32_t, std::string> g_uavRoles;

static uint32_t g_replayTasksDone = 0;
static double g_allTasksFinishSec = -1.0;

static uint32_t
ParseTrailingIndex(const std::string& id)
{
  size_t pos = id.rfind('_');
  NS_ABORT_MSG_IF(pos == std::string::npos, "Bad node id format: " << id);
  return static_cast<uint32_t>(std::stoul(id.substr(pos + 1)));
}

static ScheduleTrace
LoadScheduleTrace(const std::string& path)
{
  ScheduleTrace tr;

  std::ifstream ifs(path.c_str());
  NS_ABORT_MSG_IF(!ifs.is_open(), "Cannot open schedule file: " << path);

  json j;
  ifs >> j;

  tr.missionDeadlineSec = j.value("mission_deadline_sec", 0.0);
  tr.nUavs = j["nodes"].value("uavs", 0u);

  for (const auto& s : j["nodes"]["sensors"])
  {
    Point2D p;
    p.x = s["x"].get<double>();
    p.y = s["y"].get<double>();
    tr.sensors.push_back(p);
  }

  uint32_t nextTaskId = 1;

  for (const auto& ev : j["events"])
  {
    const double t = ev["time"].get<double>();
    tr.lastEventTimeSec = std::max(tr.lastEventTimeSec, t);

    const std::string type = ev["type"].get<std::string>();

    if (type == "MOVE")
    {
      MoveEvent m;
      m.timeSec = t;
      m.uavId = ParseTrailingIndex(ev["node_id"].get<std::string>());
      m.target = Vector(ev["target_pos"][0].get<double>(),
                        ev["target_pos"][1].get<double>(),
                        ev["target_pos"][2].get<double>());
      tr.moves.push_back(m);
    }
    else if (type == "ROLE_SET")
    {
      RoleSetEvent r;
      r.timeSec = t;
      r.uavId = ParseTrailingIndex(ev["node_id"].get<std::string>());
      r.role = ev["role"].get<std::string>();
      tr.roles.push_back(r);
    }
    else if (type == "SENSOR_TO_UAV" || type == "SLAVE_TO_MASTER")
    {
      TransferEvent te;
      te.taskId = nextTaskId++;
      te.type = type;
      te.timeSec = t;
      te.srcName = ev["src_id"].get<std::string>();
      te.dstName = ev["dst_id"].get<std::string>();
      te.bytes = ev["data_size_bytes"].get<uint64_t>();
      te.deadlineSec = ev["deadline_sec"].get<double>();
      te.srcIndex = ParseTrailingIndex(te.srcName);
      te.dstIndex = ParseTrailingIndex(te.dstName);
      tr.transfers.push_back(te);
    }
  }

  return tr;
}

static std::vector<Vector>
BuildInitialUavPositions(uint32_t nUav,
                         const std::vector<MoveEvent>& moves,
                         double defaultAltitude)
{
  std::vector<Vector> initPos(nUav, Vector(0.0, 0.0, defaultAltitude));
  std::vector<double> firstTime(nUav, std::numeric_limits<double>::max());

  for (const auto& m : moves)
  {
    if (m.uavId >= nUav)
    {
      continue;
    }

    if (m.timeSec < firstTime[m.uavId])
    {
      firstTime[m.uavId] = m.timeSec;
      initPos[m.uavId] = m.target;
    }
  }

  return initPos;
}

static uint32_t
SmallestPrefixForHosts(uint32_t hostCount)
{
  // 至少保留 network / broadcast，因此 usable = 2^(32-prefix) - 2
  for (uint32_t prefix = 30; prefix >= 1; --prefix)
  {
    uint64_t total = (1ULL << (32 - prefix));
    if (total >= 2 && (total - 2) >= hostCount)
    {
      return prefix;
    }
    if (prefix == 1)
    {
      break;
    }
  }

  // 最保守 fallback
  return 1;
}

static Ipv4Mask
PrefixToIpv4Mask(uint32_t prefix)
{
  uint32_t mask = (prefix == 0) ? 0u : (0xFFFFFFFFu << (32 - prefix));

  std::ostringstream oss;
  oss << ((mask >> 24) & 0xFF) << "."
      << ((mask >> 16) & 0xFF) << "."
      << ((mask >> 8) & 0xFF) << "."
      << (mask & 0xFF);

  return Ipv4Mask(oss.str().c_str());
}

static bool
IsSensorTask(const ReplayTaskState& st)
{
  return st.type == "SENSOR_TO_UAV";
}

static bool
IsSlaveTask(const ReplayTaskState& st)
{
  return st.type == "SLAVE_TO_MASTER";
}

static bool
UavRoleIsSlave(uint32_t uavId)
{
  auto it = g_uavRoles.find(uavId);
  return (it != g_uavRoles.end() && it->second == "SLAVE");
}

static bool
UavRoleIsMaster(uint32_t uavId)
{
  if (g_forcedMasterUavId >= 0 &&
      static_cast<uint32_t>(g_forcedMasterUavId) == uavId)
  {
    return true;
  }

  auto it = g_uavRoles.find(uavId);
  if (it == g_uavRoles.end())
  {
    return false;
  }

  const std::string& role = it->second;
  return role == "CH" ||
         role == "MASTER" ||
         role == "Master" ||
         role == "master" ||
         role == "CLUSTER_HEAD";
}

static void
OpenMasterTraceFile(const std::string& path)
{
  if (path.empty())
  {
    return;
  }

  g_masterTraceCsv.open(path.c_str(), std::ios::out | std::ios::trunc);
  if (!g_masterTraceCsv.is_open())
  {
    std::cerr << "[WARN] Cannot open masterTraceFile=" << path << "\n";
    return;
  }

  g_masterTraceCsv
      << "timeSec,event,taskId,type,src,dstMaster,"
      << "bytesArrived,masterBufferBytes,masterArrivedBytes,totalMasterArrivedBytes\n";
}

static void
CloseMasterTraceFile()
{
  if (g_masterTraceCsv.is_open())
  {
    g_masterTraceCsv.close();
  }
}

static void
AccountMasterRxBytes(const ReplayTaskState& st, uint64_t bytesArrived)
{
  if (bytesArrived == 0)
  {
    return;
  }

  bool shouldAccount = false;
  std::string eventName = "";

  // slave -> master：destination 一定視為 master/CH 收到資料
  if (IsSlaveTask(st))
  {
    shouldAccount = true;
    eventName = "SLAVE_TO_MASTER_RX";
  }
  // sensor -> UAV：只有 dst 目前是 CH/master，或由 masterUavId 強制指定時，才算進 master buffer
  else if (IsSensorTask(st) && UavRoleIsMaster(st.dstIndex))
  {
    shouldAccount = true;
    eventName = "SENSOR_TO_MASTER_RX";
  }

  if (!shouldAccount)
  {
    return;
  }

  uint32_t masterId = st.dstIndex;

  g_masterBufferedBytes[masterId] += bytesArrived;
  g_masterArrivedBytes[masterId] += bytesArrived;
  g_totalMasterArrivedBytes += bytesArrived;

  if (g_masterTraceCsv.is_open())
  {
    g_masterTraceCsv
        << Simulator::Now().GetSeconds() << ","
        << eventName << ","
        << st.id << ","
        << st.type << ","
        << st.srcName << ","
        << "UAV_" << masterId << ","
        << bytesArrived << ","
        << g_masterBufferedBytes[masterId] << ","
        << g_masterArrivedBytes[masterId] << ","
        << g_totalMasterArrivedBytes
        << "\n";
  }
}

static void
ApplyMove(Ptr<MobilityModel> mob, uint32_t uavId, Vector target)
{
  mob->SetPosition(target);
  std::cout << "[MOVE] t=" << Simulator::Now().GetSeconds()
            << " UAV[" << uavId << "] -> ("
            << target.x << ", " << target.y << ", " << target.z << ")\n";
}

static void
ApplyRoleSet(uint32_t uavId, const std::string& role)
{
  g_uavRoles[uavId] = role;
  std::cout << "[ROLE] t=" << Simulator::Now().GetSeconds()
            << " UAV[" << uavId << "] = " << role << "\n";
}

static uint64_t
GetTaskMissingBytes(uint32_t taskId)
{
  const ReplayTaskState& st = g_replayTasks[taskId];
  if (st.rxBytes >= st.targetBytes)
  {
    return 0;
  }
  return st.targetBytes - st.rxBytes;
}

static void
SendSourceChunk(Ptr<Socket> sock,
                uint32_t taskId,
                uint32_t chunkBytes,
                Time gap)
{
  ReplayTaskState& st = g_replayTasks[taskId];

  if (st.done)
  {
    return;
  }

  uint64_t missing = GetTaskMissingBytes(taskId);
  if (missing == 0)
  {
    return;
  }

  uint32_t toSend = static_cast<uint32_t>(
      std::min<uint64_t>(static_cast<uint64_t>(chunkBytes), missing));

  sock->Send(Create<Packet>(toSend));
  st.txBytes += toSend;

  // 不再用 sender-side remaining 當停止條件，
  // 而是看 receiver-side rxBytes 是否真的補滿。
  Simulator::Schedule(gap, &SendSourceChunk, sock, taskId, chunkBytes, gap);
}

static void
StartSensorTask(Ptr<Socket> sock,
                uint32_t taskId,
                uint32_t chunkBytes,
                Time gap)
{
  ReplayTaskState& st = g_replayTasks[taskId];
  st.started = true;
  SendSourceChunk(sock, taskId, chunkBytes, gap);
}

static void
TrySendSlaveChunk(Ptr<Socket> sock,
                  uint32_t taskId,
                  uint32_t chunkBytes,
                  Time gap)
{
  ReplayTaskState& st = g_replayTasks[taskId];

  if (st.done)
  {
    return;
  }

  // 只有在目前角色真的是 SLAVE 時才允許往 master 送
  if (!UavRoleIsSlave(st.srcIndex))
  {
    Simulator::Schedule(gap, &TrySendSlaveChunk, sock, taskId, chunkBytes, gap);
    return;
  }

  uint64_t missing = GetTaskMissingBytes(taskId);
  if (missing == 0)
  {
    return;
  }

  uint64_t& buf = g_uavBufferedBytes[st.srcIndex];
  if (buf == 0)
  {
    // 還沒從 sensor 收到足夠資料，先等
    Simulator::Schedule(gap, &TrySendSlaveChunk, sock, taskId, chunkBytes, gap);
    return;
  }

  uint64_t allowed = std::min<uint64_t>(missing, static_cast<uint64_t>(chunkBytes));
  allowed = std::min<uint64_t>(allowed, buf);

  uint32_t toSend = static_cast<uint32_t>(allowed);
  if (toSend == 0)
  {
    Simulator::Schedule(gap, &TrySendSlaveChunk, sock, taskId, chunkBytes, gap);
    return;
  }

  sock->Send(Create<Packet>(toSend));

  st.txBytes += toSend;
  buf -= toSend;

  // 一樣改成看 receiver-side rxBytes 是否補滿
  Simulator::Schedule(gap, &TrySendSlaveChunk, sock, taskId, chunkBytes, gap);
}

static void
StartSlaveTask(Ptr<Socket> sock,
               uint32_t taskId,
               uint32_t chunkBytes,
               Time gap)
{
  ReplayTaskState& st = g_replayTasks[taskId];
  st.started = true;
  TrySendSlaveChunk(sock, taskId, chunkBytes, gap);
}

static void
CountReplayTaskRx(uint32_t taskId, Ptr<const Packet> pkt, const Address& from)
{
  ReplayTaskState& st = g_replayTasks[taskId];

  if (st.done)
  {
    return;
  }

  uint64_t pktSize = pkt->GetSize();

  // 避免極端情況下 rxBytes 超過 targetBytes，後面 buffer accounting 也只計有效 bytes
  uint64_t missingBeforeRx = 0;
  if (st.rxBytes < st.targetBytes)
  {
    missingBeforeRx = st.targetBytes - st.rxBytes;
  }

  uint64_t acceptedBytes = std::min<uint64_t>(pktSize, missingBeforeRx);
  if (acceptedBytes == 0)
  {
    return;
  }

  st.rxBytes += acceptedBytes;

  // 如果是 sensor->uav，代表 UAV 真的拿到資料了，buffer 才增加
  if (IsSensorTask(st))
  {
    g_uavBufferedBytes[st.dstIndex] += acceptedBytes;
  }

  // 新增：若資料已到 CH/master，記到 master buffer，之後可作為 LEO upload input
  AccountMasterRxBytes(st, acceptedBytes);

  if (st.rxBytes >= st.targetBytes)
  {
    st.done = true;
    st.finishSec = Simulator::Now().GetSeconds();
    ++g_replayTasksDone;

    bool metDeadline = ((st.finishSec - st.startSec) <= st.deadlineSec);

    std::cout << "[TASK-DONE]"
              << " id=" << st.id
              << " type=" << st.type
              << " " << st.srcName << "->" << st.dstName
              << " bytes=" << st.targetBytes
              << " finishSec=" << st.finishSec
              << " durationSec=" << (st.finishSec - st.startSec)
              << " metDeadline=" << (metDeadline ? "YES" : "NO")
              << "\n";

    if (g_replayTasksDone == g_replayTasks.size())
    {
      g_allTasksFinishSec = st.finishSec;
      Simulator::Stop(Simulator::Now() + NanoSeconds(1));
    }
  }
}

static void
PrintMissionSummary(double missionDeadlineSec)
{
  uint32_t sensorTotal = 0, sensorDone = 0;
  uint32_t slaveTotal  = 0, slaveDone  = 0;
  uint32_t metTaskDeadlines = 0;
  uint64_t totalBytes = 0;

  double sensorFinishSec = -1.0;
  double slaveFinishSec  = -1.0;

  for (const auto& kv : g_replayTasks)
  {
    const ReplayTaskState& st = kv.second;
    totalBytes += st.targetBytes;

    if (IsSensorTask(st))
    {
      ++sensorTotal;
      if (st.done)
      {
        ++sensorDone;
        sensorFinishSec = std::max(sensorFinishSec, st.finishSec);
      }
    }
    else if (IsSlaveTask(st))
    {
      ++slaveTotal;
      if (st.done)
      {
        ++slaveDone;
        slaveFinishSec = std::max(slaveFinishSec, st.finishSec);
      }
    }

    if (st.done && ((st.finishSec - st.startSec) <= st.deadlineSec))
    {
      ++metTaskDeadlines;
    }
  }

  std::cout << "[MISSION]"
            << " tasksTotal=" << g_replayTasks.size()
            << " tasksDone=" << g_replayTasksDone
            << " allDone=" << (g_replayTasksDone == g_replayTasks.size() ? "YES" : "NO")
            << " sensorDone=" << sensorDone << "/" << sensorTotal
            << " slaveDone=" << slaveDone << "/" << slaveTotal
            << " totalBytes=" << totalBytes
            << " finishSec=" << g_allTasksFinishSec
            << " sensorFinishSec=" << sensorFinishSec
            << " slaveFinishSec=" << slaveFinishSec
            << " missionDeadlineSec=" << missionDeadlineSec
            << " metMissionDeadline="
            << ((g_allTasksFinishSec >= 0.0 && g_allTasksFinishSec <= missionDeadlineSec) ? "YES" : "NO")
            << " taskDeadlinePass=" << metTaskDeadlines << "/" << g_replayTasks.size()
            << "\n";

  uint32_t printed = 0;
  for (const auto& kv : g_replayTasks)
  {
    const ReplayTaskState& st = kv.second;
    if (!st.done && printed < 10)
    {
      std::cout << "[UNFINISHED]"
                << " id=" << st.id
                << " type=" << st.type
                << " " << st.srcName << "->" << st.dstName
                << " targetBytes=" << st.targetBytes
                << " txBytes=" << st.txBytes
                << " rxBytes=" << st.rxBytes
                << " startSec=" << st.startSec
                << " deadlineSec=" << st.deadlineSec
                << "\n";
      ++printed;
    }
  }

  std::cout << "[UAV-BUFFER]";
  for (const auto& kv : g_uavBufferedBytes)
  {
    std::cout << " UAV[" << kv.first << "]=" << kv.second;
  }
  std::cout << "\n";
}

double
Distance2D(const Point2D& a, const Point2D& b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

// 讀出所有地面節點的 (x, y) 位置（z 忽略不管）
std::vector<Point2D>
GetGroundNodePositions(const NodeContainer& groundNodes)
{
  std::vector<Point2D> sensors;
  sensors.reserve(groundNodes.GetN());

  for (uint32_t i = 0; i < groundNodes.GetN(); ++i)
  {
    Ptr<MobilityModel> mob = groundNodes.Get(i)->GetObject<MobilityModel>();
    Vector p = mob->GetPosition();
    sensors.push_back(Point2D{p.x, p.y});
  }

  return sensors;
}

// 簡單版 k-means：給定感測點 sensors，找 k 個中心
std::vector<Point2D>
RunKMeans(const std::vector<Point2D>& sensors,
          uint32_t k,
          uint32_t maxIterations = 20)
{
  std::vector<Point2D> centers;
  if (sensors.empty() || k == 0)
  {
    return centers;
  }

  uint32_t n = sensors.size();
  centers.reserve(k);

  // 初始化：直接拿前 k 個感測點當初始中心
  for (uint32_t i = 0; i < k && i < n; ++i)
  {
    centers.push_back(sensors[i]);
  }
  // 如果 sensor 比 k 少，就重複塞
  while (centers.size() < k)
  {
    centers.push_back(sensors[centers.size() % n]);
  }

  std::vector<uint32_t> assignment(n, 0);

  for (uint32_t iter = 0; iter < maxIterations; ++iter)
  {
    // Step 1: 依照目前 center，把每個 sensor 指派給距離最近的那個 center
    for (uint32_t i = 0; i < n; ++i)
    {
      double bestDist = std::numeric_limits<double>::max();
      uint32_t bestIdx = 0;

      for (uint32_t c = 0; c < k; ++c)
      {
        double d = Distance2D(sensors[i], centers[c]);
        if (d < bestDist)
        {
          bestDist = d;
          bestIdx = c;
        }
      }
      assignment[i] = bestIdx;
    }

    // Step 2: 根據指派好的 cluster，重新計算每個中心的位置
    std::vector<double> sumX(k, 0.0);
    std::vector<double> sumY(k, 0.0);
    std::vector<uint32_t> count(k, 0);

    for (uint32_t i = 0; i < n; ++i)
    {
      uint32_t c = assignment[i];
      sumX[c] += sensors[i].x;
      sumY[c] += sensors[i].y;
      count[c] += 1;
    }

    bool changed = false;
    for (uint32_t c = 0; c < k; ++c)
    {
      if (count[c] == 0)
      {
        // 這個 cluster 沒有人，就隨便拉回某個感測點
        centers[c] = sensors[c % n];
        continue;
      }

      Point2D newCenter;
      newCenter.x = sumX[c] / count[c];
      newCenter.y = sumY[c] / count[c];

      if (Distance2D(newCenter, centers[c]) > 1e-3)
      {
        centers[c] = newCenter;
        changed   = true;
      }
    }

    if (!changed)
    {
      // 中心幾乎不動了，當成收斂
      break;
    }
  }

  return centers;
}

// 計算在給定「3D 通訊距離」range3d (m) 下，被至少一台 UAV 覆蓋到的 sensor 個數
// 這裡假設 sensor 在地面 z=0，UAV 高度固定為 uavAltitude。
uint32_t
CountCoveredSensors3D(const std::vector<Point2D>& sensors,
                      const std::vector<Point2D>& centers,
                      double range3d,
                      double uavAltitude)
{
  if (sensors.empty() || centers.empty())
  {
    return 0;
  }

  const double r2 = range3d * range3d;
  const double h2 = uavAltitude * uavAltitude;
  uint32_t covered = 0;

  for (const auto& s : sensors)
  {
    bool isCovered = false;

    for (const auto& c : centers)
    {
      const double dx = s.x - c.x;
      const double dy = s.y - c.y;
      const double d3d2 = dx * dx + dy * dy + h2; // slant distance^2

      if (d3d2 <= r2)
      {
        isCovered = true;
        break;
      }
    }

    if (isCovered)
    {
      ++covered;
    }
  }

  return covered;
}

// 根據 centers，把每個 sensor 指派給最近的中心，回傳長度 = sensors.size() 的陣列
std::vector<uint32_t>
AssignSensorsToCenters(const std::vector<Point2D>& sensors,
                       const std::vector<Point2D>& centers)
{
  std::vector<uint32_t> assignment;
  if (sensors.empty() || centers.empty())
  {
    return assignment;
  }

  assignment.resize(sensors.size());

  for (uint32_t i = 0; i < sensors.size(); ++i)
  {
    double bestDist = std::numeric_limits<double>::max();
    uint32_t bestIdx = 0;

    for (uint32_t c = 0; c < centers.size(); ++c)
    {
      double d = Distance2D(sensors[i], centers[c]);
      if (d < bestDist)
      {
        bestDist = d;
        bestIdx  = c;
      }
    }

    assignment[i] = bestIdx;
  }

  return assignment;
}

// ===== End-to-end / split metrics =====
static uint64_t g_groundTxBytes = 0;           // ground 實際送出的 bytes

static std::map<uint32_t, uint64_t> g_uavAccessRxBytes; // 每台 UAV 在 access sink 收到多少
static std::map<uint32_t, Time> g_nextRelayTxTime;
static std::map<uint32_t, uint64_t> g_relayEnqBytes;
static std::map<uint32_t, uint64_t> g_relaySentBytes;

// ground 送包 loop：每次送一包，送完就 schedule 下一次
static void
GroundSendLoop(Ptr<Socket> sock,
               uint32_t* remainingPkts,
               uint32_t packetSize,
               Time interval)
{
  if (*remainingPkts == 0)
  {
    return;
  }

  sock->Send(Create<Packet>(packetSize));
  g_groundTxBytes += packetSize;
  (*remainingPkts)--;

  Simulator::Schedule(interval, &GroundSendLoop, sock, remainingPkts, packetSize, interval);
}

// ===== Backhaul SNR/rate helpers =====


NS_LOG_COMPONENT_DEFINE("UavVanetExample");

int main (int argc, char *argv[])
{
        uint32_t nUav     = 3;       // UAV 節點數
        uint32_t nGround  = 100;     // 地面節點數
        double   simTime  = 20.0;    // 模擬時間（秒）
        double   areaSize = 500.0;   // 地面節點灑點區域邊長（公尺）

        // 拓樸控制
        int32_t masterUavId = -1;         // -1 = use ROLE_SET; >=0 = force this UAV as master/CH
        uint32_t maxActiveRelayUavs = 0;  // 0 = 所有非 master UAV 都可 relay；1 = 只開一台；2 = 開兩台...
        bool     relayOnlyTraffic = false; // true = 只讓「啟用 relay 的 UAV 所屬地面節點」送流量

        // 物理/幾何參數
        double   uavAltitude = 60.0;
        double   accessRange3d   = 120.0;
        double   backhaulRange3d = 1000.0;

        // Wi-Fi PHY / SNR-rate 參數
        double   wifiFreqHz  = 2.49e9;
        double   txPowerDbm  = 20.0;
        double   backhaulBandwidthHz   = 20e6;
        double   backhaulNoiseFigureDb = 6.0;

        // k-means 參數
        uint32_t kMeansIters = 20;

        std::string outFile = "/home/demo/Desktop/output.txt";

        // debug
        bool     verbose = false;
        uint32_t debugLimit = 20;

        // traffic
        uint32_t maxPackets = 200;
        double   intervalSec = 0.1;
        
        // schedule replay
        std::string scheduleFile = "";
        uint32_t taskChunkBytes = 1024;   // 每次送出的 payload 大小
        double   taskGapUs = 100.0;       // 每次送 chunk 的間隔 (microseconds)
        
        std::string masterTraceFile = "";

    CommandLine cmd;
    cmd.AddValue("outFile", "Write all stdout/stderr to this file (empty = console)", outFile);
    cmd.AddValue("nUav", "Number of UAV nodes", nUav);
    cmd.AddValue("nGround", "Number of ground nodes", nGround);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("areaSize", "Side length of square area for ground nodes (m)", areaSize);

    cmd.AddValue("uavAltitude", "UAV altitude (m)", uavAltitude);
    cmd.AddValue("accessRange3d", "3D range (m) for Ground <-> UAV (access)", accessRange3d);
    cmd.AddValue("backhaulRange3d", "3D range (m) for UAV <-> UAV (backhaul)", backhaulRange3d);

    cmd.AddValue("wifiFreqHz", "Wi-Fi carrier frequency (Hz) for Friis loss model", wifiFreqHz);
    cmd.AddValue("txPowerDbm", "Tx power (dBm)", txPowerDbm);
    cmd.AddValue("backhaulBandwidthHz", "Backhaul bandwidth in Hz for Shannon rate", backhaulBandwidthHz);
    cmd.AddValue("backhaulNoiseFigureDb", "Backhaul noise figure in dB", backhaulNoiseFigureDb);
  
    cmd.AddValue("maxActiveRelayUavs",
             "0=all non-master UAVs relay, 1=only first relay UAV enabled, 2=first two enabled, ...",
             maxActiveRelayUavs);
    cmd.AddValue("relayOnlyTraffic",
             "If true, only ground nodes assigned to enabled relay UAVs send traffic",
             relayOnlyTraffic);

    cmd.AddValue("kMeansIters", "Max iterations for k-means", kMeansIters);

    cmd.AddValue("verbose", "Print all debug (positions / assignments / distances)", verbose);
    cmd.AddValue("debugLimit", "If verbose=false, print only first N sensors for assignment debug", debugLimit);

    cmd.AddValue("maxPackets", "UdpEchoClient MaxPackets per ground node", maxPackets);
    cmd.AddValue("intervalSec", "UdpEchoClient Interval (sec)", intervalSec);
    cmd.AddValue("scheduleFile", "Path to schedule_trace.json (empty = use old baseline mode)", scheduleFile);
    cmd.AddValue("taskChunkBytes", "Chunk size for each replay task", taskChunkBytes);
    cmd.AddValue("taskGapUs", "Gap (microseconds) between replay chunks", taskGapUs);
    
    cmd.AddValue("masterTraceFile",
             "Optional CSV file for Master/CH buffer arrivals",
             masterTraceFile);

    cmd.AddValue("masterUavId",
             "Force a UAV id as Master/CH for master buffer accounting (-1 = use ROLE_SET)",
             masterUavId);

cmd.Parse(argc, argv);

// schedule-only version: scheduleFile is required
if (scheduleFile.empty())
{
  std::cerr << "[ERR] scheduleFile is required in this version\n";
  return 1;
}

// output redirection
std::ofstream ofs;
std::streambuf* coutBuf = std::cout.rdbuf();
std::streambuf* cerrBuf = std::cerr.rdbuf();

if (!outFile.empty())
{
  ofs.open(outFile, std::ios::out | std::ios::trunc);
  if (ofs.is_open())
  {
    std::cout.rdbuf(ofs.rdbuf());
    std::cerr.rdbuf(ofs.rdbuf());
  }
  else
  {
    std::cerr << "[WARN] Cannot open outFile=" << outFile << ", fallback to console\n";
  }
}

g_forcedMasterUavId = masterUavId;
g_masterTraceFile = masterTraceFile;
OpenMasterTraceFile(g_masterTraceFile);

ScheduleTrace schedule = LoadScheduleTrace(scheduleFile);
bool useScheduleReplay = true;

// use trace contents as the source of truth
nUav = schedule.nUavs;
nGround = static_cast<uint32_t>(schedule.sensors.size());

double latestTransferAbsDeadlineSec = 0.0;
for (const auto& te : schedule.transfers)
{
  latestTransferAbsDeadlineSec =
      std::max(latestTransferAbsDeadlineSec, te.timeSec + te.deadlineSec);
}

double safeStopSec = std::max(schedule.missionDeadlineSec, latestTransferAbsDeadlineSec) + 60.0;
simTime = std::max(simTime, safeStopSec);

std::cout << "[SCHEDULE]"
          << " file=" << scheduleFile
          << " uavs=" << nUav
          << " sensors=" << nGround
          << " transfers=" << schedule.transfers.size()
          << " moves=" << schedule.moves.size()
          << " roles=" << schedule.roles.size()
          << " missionDeadlineSec=" << schedule.missionDeadlineSec
          << " lastEventSec=" << schedule.lastEventTimeSec
          << " latestTransferAbsDeadlineSec=" << latestTransferAbsDeadlineSec
          << " simTime=" << simTime
          << "\n";

if (nUav == 0)
{
  std::cerr << "[ERR] nUav must be >= 1\n";
  return 1;
}

if (accessRange3d <= uavAltitude)
{
  std::cout << "[WARN] accessRange3d <= uavAltitude => ground coverage may be poor.\n";
}
if (backhaulRange3d <= 0.0)
{
  std::cout << "[WARN] backhaulRange3d <= 0 => UAV backhaul will never work.\n";
}

    // 如有需要可以開 log
    // LogComponentEnable("UavVanetExample", LOG_LEVEL_INFO);

    // -------------------------
    // 1. 建立節點
    // -------------------------
    NodeContainer uavNodes;
    uavNodes.Create(nUav);

    NodeContainer groundNodes;
    groundNodes.Create(nGround);

    NodeContainer allNodes;
    allNodes.Add(uavNodes);
    allNodes.Add(groundNodes);


// -------------------------
// 2. Mobility + UAV placement (schedule-only)
// -------------------------

MobilityHelper groundMobility;
groundMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
groundMobility.Install(groundNodes);

MobilityHelper uavMobility;
uavMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
uavMobility.Install(uavNodes);

std::vector<Point2D> sensors = schedule.sensors;

// exact sensor positions from JSON
for (uint32_t i = 0; i < groundNodes.GetN(); ++i)
{
  Ptr<MobilityModel> mob = groundNodes.Get(i)->GetObject<MobilityModel>();
  mob->SetPosition(Vector(sensors[i].x, sensors[i].y, 0.0));
}

// initial UAV positions from the earliest MOVE target in trace
std::vector<Vector> initUavPos = BuildInitialUavPositions(nUav, schedule.moves, uavAltitude);
for (uint32_t u = 0; u < nUav; ++u)
{
  Ptr<MobilityModel> mob = uavNodes.Get(u)->GetObject<MobilityModel>();
  mob->SetPosition(initUavPos[u]);
}

std::cout << "Total ground nodes: " << nGround << std::endl;
std::cout << "Schedule-driven mode: loaded exact sensor positions from JSON\n";
    // （後面原本的 Wi-Fi 設定、AODV、Application、FlowMonitor 全部照舊）

    // -------------------------
    // 4. Wi-Fi networks (Option B)
    //    - Access Wi-Fi: ground <-> UAV  (10.0.0.0/24)
    //    - Backhaul Wi-Fi: UAV <-> UAV   (10.1.0.0/24)
    // -------------------------

    // ===== Access Wi-Fi (ground + UAV) =====
    YansWifiChannelHelper accessChannel;
    accessChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    accessChannel.AddPropagationLoss("ns3::FriisPropagationLossModel",
                                     "Frequency", DoubleValue(wifiFreqHz));
    accessChannel.AddPropagationLoss("ns3::RangePropagationLossModel",
                                     "MaxRange", DoubleValue(accessRange3d));

    YansWifiPhyHelper accessPhy;
    accessPhy.SetErrorRateModel("ns3::NistErrorRateModel");
    accessPhy.SetChannel(accessChannel.Create());
    accessPhy.Set("TxPowerStart", DoubleValue(txPowerDbm));
    accessPhy.Set("TxPowerEnd",   DoubleValue(txPowerDbm));

    WifiMacHelper accessMac;
    accessMac.SetType("ns3::AdhocWifiMac",
                      "QosSupported", BooleanValue(false));

    WifiHelper accessWifi;
    accessWifi.SetStandard(WIFI_STANDARD_80211g);
    accessWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                       "DataMode", StringValue("ErpOfdmRate6Mbps"),
                                       "ControlMode", StringValue("ErpOfdmRate6Mbps"));

    NodeContainer accessNodes = allNodes; // UAVs first, then grounds
    NetDeviceContainer accessDevices = accessWifi.Install(accessPhy, accessMac, accessNodes);

    // ===== Backhaul Wi-Fi (UAV only) =====
    YansWifiChannelHelper backhaulChannel;
    backhaulChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    backhaulChannel.AddPropagationLoss("ns3::FriisPropagationLossModel",
                                       "Frequency", DoubleValue(wifiFreqHz));
    backhaulChannel.AddPropagationLoss("ns3::RangePropagationLossModel",
                                       "MaxRange", DoubleValue(backhaulRange3d));

    YansWifiPhyHelper backhaulPhy;
    backhaulPhy.SetErrorRateModel("ns3::NistErrorRateModel");
    backhaulPhy.SetChannel(backhaulChannel.Create());
    backhaulPhy.Set("TxPowerStart", DoubleValue(txPowerDbm));
    backhaulPhy.Set("TxPowerEnd",   DoubleValue(txPowerDbm));

    WifiMacHelper backhaulMac;
    backhaulMac.SetType("ns3::AdhocWifiMac",
                        "QosSupported", BooleanValue(false));

    WifiHelper backhaulWifi;
    backhaulWifi.SetStandard(WIFI_STANDARD_80211g);
    backhaulWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                         "DataMode", StringValue("ErpOfdmRate6Mbps"),
                                         "ControlMode", StringValue("ErpOfdmRate6Mbps"));

    NetDeviceContainer backhaulDevices = backhaulWifi.Install(backhaulPhy, backhaulMac, uavNodes);

// -------------------------
// 5. Internet stack + IP address assignment
//    schedule-driven：依節點數自動選 subnet prefix
// -------------------------
InternetStackHelper stack;
stack.Install(allNodes);

// Access: all UAVs + all sensors
uint32_t accessHostCount = accessNodes.GetN();
uint32_t accessPrefix = SmallestPrefixForHosts(accessHostCount);
Ipv4Mask accessMask = PrefixToIpv4Mask(accessPrefix);

Ipv4AddressHelper accessAddr;
accessAddr.SetBase(Ipv4Address("10.0.0.0"), accessMask);
Ipv4InterfaceContainer accessIfs = accessAddr.Assign(accessDevices);

// Backhaul: UAV only
uint32_t backhaulHostCount = backhaulDevices.GetN();
uint32_t backhaulPrefix = SmallestPrefixForHosts(backhaulHostCount);
Ipv4Mask backhaulMask = PrefixToIpv4Mask(backhaulPrefix);

Ipv4AddressHelper backhaulAddr;
backhaulAddr.SetBase(Ipv4Address("10.1.0.0"), backhaulMask);
Ipv4InterfaceContainer backhaulIfs = backhaulAddr.Assign(backhaulDevices);

std::cout << "[IP-PLAN]"
          << " accessHosts=" << accessHostCount
          << " accessPrefix=/" << accessPrefix
          << " backhaulHosts=" << backhaulHostCount
          << " backhaulPrefix=/" << backhaulPrefix
          << "\n";

// -------------------------
// 6. Applications
//    - schedule mode: replay tasks from JSON
//    - baseline mode: keep old vv6 logic
// -------------------------

if (useScheduleReplay)
{
  const uint16_t accessTaskBasePort   = 30000;
  const uint16_t backhaulTaskBasePort = 40000;
  uint16_t nextAccessPort = accessTaskBasePort;
  uint16_t nextBackhaulPort = backhaulTaskBasePort;

  Time taskGap = Seconds(taskGapUs * 1e-6);

  // 6A. 先 schedule MOVE / ROLE_SET
  for (const auto& mv : schedule.moves)
  {
    Ptr<MobilityModel> mob = uavNodes.Get(mv.uavId)->GetObject<MobilityModel>();
    Simulator::Schedule(Seconds(mv.timeSec), &ApplyMove, mob, mv.uavId, mv.target);
  }

  for (const auto& re : schedule.roles)
  {
    Simulator::Schedule(Seconds(re.timeSec), &ApplyRoleSet, re.uavId, re.role);
  }

  // 6B. 對每筆 transfer 建 sink + source socket
  for (const auto& te : schedule.transfers)
  {
    ReplayTaskState st;
    st.id = te.taskId;
    st.type = te.type;
    st.srcName = te.srcName;
    st.dstName = te.dstName;
    st.srcIndex = te.srcIndex;
    st.dstIndex = te.dstIndex;
    st.targetBytes = te.bytes;
    st.startSec = te.timeSec;
    st.deadlineSec = te.deadlineSec;
    g_replayTasks[te.taskId] = st;

    uint16_t port = 0;
    Ptr<Node> srcNode;
    Ptr<Node> dstNode;
    Ipv4Address dstIp;

    if (te.type == "SENSOR_TO_UAV")
    {
      port = nextAccessPort++;
      srcNode = groundNodes.Get(te.srcIndex);
      dstNode = uavNodes.Get(te.dstIndex);
      dstIp = accessIfs.GetAddress(te.dstIndex);   // UAV 在 access subnet 的 IP
    }
    else // SLAVE_TO_MASTER
    {
      port = nextBackhaulPort++;
      srcNode = uavNodes.Get(te.srcIndex);
      dstNode = uavNodes.Get(te.dstIndex);
      dstIp = backhaulIfs.GetAddress(te.dstIndex); // UAV 在 backhaul subnet 的 IP
    }

PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), port));
ApplicationContainer sinkApps = sinkHelper.Install(dstNode);
sinkApps.Start(Seconds(0.0));
sinkApps.Stop(Seconds(simTime));

Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApps.Get(0));
sink->TraceConnectWithoutContext("Rx",
                                 MakeBoundCallback(&CountReplayTaskRx, te.taskId));

Ptr<Socket> sock = Socket::CreateSocket(srcNode, UdpSocketFactory::GetTypeId());
sock->Connect(InetSocketAddress(dstIp, port));

    if (te.type == "SENSOR_TO_UAV")
    {
      Simulator::Schedule(Seconds(te.timeSec),
                          &StartSensorTask,
                          sock,
                          te.taskId,
                          taskChunkBytes,
                          taskGap);
    }
    else // SLAVE_TO_MASTER
    {
      Simulator::Schedule(Seconds(te.timeSec),
                          &StartSlaveTask,
                          sock,
                          te.taskId,
                          taskChunkBytes,
                          taskGap);
    }
  }

  uint32_t sensorCnt = 0;
  uint32_t slaveCnt = 0;
  for (const auto& te : schedule.transfers)
  {
    if (te.type == "SENSOR_TO_UAV") ++sensorCnt;
    else if (te.type == "SLAVE_TO_MASTER") ++slaveCnt;
  }

  std::cout << "[ReplayMode]"
            << " transfers=" << schedule.transfers.size()
            << " sensorTasks=" << sensorCnt
            << " slaveTasks=" << slaveCnt
            << " taskChunkBytes=" << taskChunkBytes
            << " taskGapUs=" << taskGapUs
            << "\n";
}

Simulator::Stop(Seconds(simTime));

std::cout << "[DBG-1] Simulator::Run() START"
          << " (scheduleMode=1"
          << ", simTime=" << simTime
          << ", nUav=" << nUav
          << ", nGround=" << nGround
          << ", transfers=" << schedule.transfers.size()
          << ", taskChunkBytes=" << taskChunkBytes
          << ", taskGapUs=" << taskGapUs
          << ")\n" << std::flush;

Simulator::Run();

PrintMissionSummary(schedule.missionDeadlineSec);
CloseMasterTraceFile();

std::cout << "[DBG-END] Simulator::Destroy() BEGIN\n" << std::flush;
Simulator::Destroy();
std::cout << "[DBG-END] Exit main\n" << std::flush;

if (ofs.is_open())
{
  std::cout.rdbuf(coutBuf);
  std::cerr.rdbuf(cerrBuf);
  ofs.close();
}

return 0;
}

