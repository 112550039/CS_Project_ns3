/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/packet-sink.h"
#include "ns3/udp-socket-factory.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UavLinkProbe");

struct ProbeFlowState
{
  uint32_t flowId = 0;
  uint32_t srcId = 0;
  uint32_t dstId = 0;

  std::string srcRole;
  std::string dstRole;

  Vector srcPos;
  Vector dstPos;

  double distanceM = 0.0;
  double snrDb = -1e9;
  double estRateMbps = 0.0;

  uint64_t txBytes = 0;
  uint64_t rxBytes = 0;
  uint32_t txPkts = 0;
  uint32_t rxPkts = 0;

  double firstTxSec = -1.0;
  double lastTxSec = -1.0;
  double firstRxSec = -1.0;
  double lastRxSec = -1.0;
};

static std::map<uint32_t, ProbeFlowState> g_probeFlows;

static uint32_t
SmallestPrefixForHosts(uint32_t hostCount)
{
  for (uint32_t prefix = 30; prefix >= 1; --prefix)
  {
    uint64_t total = (1ULL << (32 - prefix));
    if (total >= 2 && (total - 2) >= hostCount)
    {
      return prefix;
    }
    if (prefix == 1)
      break;
  }
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

static std::vector<Vector>
ParseUavPositions(const std::string& s, double altitude)
{
  std::vector<Vector> out;
  std::stringstream ss(s);
  std::string token;

  while (std::getline(ss, token, ';'))
  {
    if (token.empty())
      continue;

    std::stringstream xy(token);
    std::string sx, sy;

    if (!std::getline(xy, sx, ','))
      continue;
    if (!std::getline(xy, sy, ','))
      continue;

    double x = std::stod(sx);
    double y = std::stod(sy);
    out.push_back(Vector(x, y, altitude));
  }

  return out;
}

static double
ComputeDistanceM(const Vector& a, const Vector& b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static double
ComputeFriisRxPowerDbm(double txPowerDbm, double freqHz, double distanceM)
{
  double d = std::max(distanceM, 1.0); // avoid log(0)
  double lambda = 3e8 / freqHz;
  double fspl = 20.0 * std::log10(4.0 * M_PI * d / lambda);
  return txPowerDbm - fspl;
}

static double
ComputeNoiseDbm(double bandwidthHz, double noiseFigureDb)
{
  return -174.0 + 10.0 * std::log10(bandwidthHz) + noiseFigureDb;
}

static double
ComputeShannonRateBps(double bandwidthHz, double snrDb)
{
  double snrLin = std::pow(10.0, snrDb / 10.0);
  return bandwidthHz * std::log2(1.0 + snrLin);
}

static void
CountProbeRx(uint32_t flowId, Ptr<const Packet> pkt, const Address& from)
{
  ProbeFlowState& st = g_probeFlows[flowId];
  double now = Simulator::Now().GetSeconds();

  if (st.rxPkts == 0)
  {
    st.firstRxSec = now;
  }
  st.lastRxSec = now;
  st.rxPkts += 1;
  st.rxBytes += pkt->GetSize();
}

static void
SendProbePacket(Ptr<Socket> sock,
                uint32_t flowId,
                uint32_t packetSize,
                uint32_t remainingPkts,
                Time interval)
{
  if (remainingPkts == 0)
  {
    return;
  }

  ProbeFlowState& st = g_probeFlows[flowId];
  Ptr<Packet> pkt = Create<Packet>(packetSize);

  int sent = sock->Send(pkt);
  double now = Simulator::Now().GetSeconds();

  if (sent > 0)
  {
    if (st.txPkts == 0)
    {
      st.firstTxSec = now;
    }
    st.lastTxSec = now;
    st.txPkts += 1;
    st.txBytes += static_cast<uint32_t>(sent);
  }

  if (remainingPkts > 1)
  {
    Simulator::Schedule(interval,
                        &SendProbePacket,
                        sock,
                        flowId,
                        packetSize,
                        remainingPkts - 1,
                        interval);
  }
}

static void
PrintProbeSummaryCsv(std::ostream& os, const ProbeFlowState& st)
{
  double effectiveThroughputMbps = 0.0;
  double deliveryRatio = 0.0;
  double probeSpanSec = 0.0;

  if (st.txBytes > 0)
  {
    deliveryRatio = static_cast<double>(st.rxBytes) / static_cast<double>(st.txBytes);
  }

  if (st.rxBytes > 0 && st.firstTxSec >= 0.0 && st.lastRxSec >= st.firstTxSec)
  {
    probeSpanSec = std::max(1e-9, st.lastRxSec - st.firstTxSec);
    effectiveThroughputMbps =
        (static_cast<double>(st.rxBytes) * 8.0) / probeSpanSec / 1e6;
  }

  os << st.flowId << ","
     << "UAV_" << st.srcId << ","
     << "UAV_" << st.dstId << ","
     << st.srcRole << ","
     << st.dstRole << ","
     << st.srcPos.x << "," << st.srcPos.y << "," << st.srcPos.z << ","
     << st.dstPos.x << "," << st.dstPos.y << "," << st.dstPos.z << ","
     << st.distanceM << ","
     << st.snrDb << ","
     << st.estRateMbps << ","
     << st.txPkts << ","
     << st.rxPkts << ","
     << st.txBytes << ","
     << st.rxBytes << ","
     << deliveryRatio << ","
     << probeSpanSec << ","
     << effectiveThroughputMbps
     << "\n";
}

static void
PrintProbeSummary(const std::string& csvOutFile)
{
  std::cout << std::fixed << std::setprecision(6);

  std::ofstream csv;
  if (!csvOutFile.empty())
  {
    csv.open(csvOutFile.c_str());
    if (csv.is_open())
    {
      csv << "flowId,src,dst,srcRole,dstRole,"
          << "srcX,srcY,srcZ,dstX,dstY,dstZ,"
          << "distanceM,snrDb,estRateMbps,"
          << "txPkts,rxPkts,txBytes,rxBytes,deliveryRatio,probeSpanSec,effThroughputMbps\n";
    }
  }

  for (const auto& kv : g_probeFlows)
  {
    const ProbeFlowState& st = kv.second;

    double effectiveThroughputMbps = 0.0;
    double deliveryRatio = 0.0;
    double probeSpanSec = 0.0;

    if (st.txBytes > 0)
    {
      deliveryRatio = static_cast<double>(st.rxBytes) / static_cast<double>(st.txBytes);
    }

    if (st.rxBytes > 0 && st.firstTxSec >= 0.0 && st.lastRxSec >= st.firstTxSec)
    {
      probeSpanSec = std::max(1e-9, st.lastRxSec - st.firstTxSec);
      effectiveThroughputMbps =
          (static_cast<double>(st.rxBytes) * 8.0) / probeSpanSec / 1e6;
    }

    std::cout << "[PROBE-RESULT]"
              << " flowId=" << st.flowId
              << " src=UAV_" << st.srcId
              << " dst=UAV_" << st.dstId
              << " srcRole=" << st.srcRole
              << " dstRole=" << st.dstRole
              << " srcPos=(" << st.srcPos.x << "," << st.srcPos.y << "," << st.srcPos.z << ")"
              << " dstPos=(" << st.dstPos.x << "," << st.dstPos.y << "," << st.dstPos.z << ")"
              << " distanceM=" << st.distanceM
              << " snrDb=" << st.snrDb
              << " estRateMbps=" << st.estRateMbps
              << " txPkts=" << st.txPkts
              << " rxPkts=" << st.rxPkts
              << " txBytes=" << st.txBytes
              << " rxBytes=" << st.rxBytes
              << " deliveryRatio=" << deliveryRatio
              << " probeSpanSec=" << probeSpanSec
              << " effThroughputMbps=" << effectiveThroughputMbps
              << "\n";

    if (csv.is_open())
    {
      PrintProbeSummaryCsv(csv, st);
    }
  }
}

int
main(int argc, char* argv[])
{
  std::string outFile = "";
  std::string csvOutFile = "";
  std::string uavPos = "0,0;300,0;600,0;900,0;1200,0";

  uint32_t masterUavId = 2;
  bool probeAllSlavesToMaster = true;
  uint32_t srcId = 0;
  uint32_t dstId = 1;

  double uavAltitude = 50.0;
  double backhaulRange3d = 1500.0;
  double wifiFreqHz = 2.49e9;
  double txPowerDbm = 20.0;
  double backhaulBandwidthHz = 20e6;
  double backhaulNoiseFigureDb = 6.0;

  std::string rateManager = "ideal";

  uint32_t packetSize = 1000;
  double intervalUs = 20000.0;     // 20 ms
  double probeDurationSec = 1.0;   // each pair probes for 1 second by default
  double startSec = 1.0;
  double pairGapSec = 2.0;

  CommandLine cmd;
  cmd.AddValue("outFile", "Write stdout/stderr to this file (empty = console)", outFile);
  cmd.AddValue("csvOutFile", "Optional CSV output file", csvOutFile);
  cmd.AddValue("uavPos",
               "Semicolon-separated UAV positions: x,y;x,y;... (z uses uavAltitude)",
               uavPos);

  cmd.AddValue("masterUavId", "Master/CH UAV id for slave->master probing", masterUavId);
  cmd.AddValue("probeAllSlavesToMaster",
               "If true, probe all non-master UAVs -> master sequentially",
               probeAllSlavesToMaster);
  cmd.AddValue("srcId", "Source UAV id when probeAllSlavesToMaster=false", srcId);
  cmd.AddValue("dstId", "Destination UAV id when probeAllSlavesToMaster=false", dstId);

  cmd.AddValue("uavAltitude", "UAV altitude (m)", uavAltitude);
  cmd.AddValue("backhaulRange3d", "3D range for UAV-UAV backhaul (m)", backhaulRange3d);
  cmd.AddValue("wifiFreqHz", "Carrier frequency (Hz)", wifiFreqHz);
  cmd.AddValue("txPowerDbm", "Tx power (dBm)", txPowerDbm);
  cmd.AddValue("backhaulBandwidthHz", "Bandwidth (Hz) for SNR/Shannon estimate", backhaulBandwidthHz);
  cmd.AddValue("backhaulNoiseFigureDb", "Noise figure (dB)", backhaulNoiseFigureDb);
  cmd.AddValue("rateManager",
               "Wi-Fi rate control: ideal or constant6",
               rateManager);

  cmd.AddValue("packetSize", "Probe packet payload size (bytes)", packetSize);
  cmd.AddValue("intervalUs", "Probe packet interval (microseconds)", intervalUs);
  cmd.AddValue("probeDurationSec", "Duration of each probe flow (seconds)", probeDurationSec);
  cmd.AddValue("startSec", "Start time of the first probe flow (seconds)", startSec);
  cmd.AddValue("pairGapSec", "Gap between probe flows (seconds)", pairGapSec);
  cmd.Parse(argc, argv);

  std::streambuf* coutBuf = std::cout.rdbuf();
  std::streambuf* cerrBuf = std::cerr.rdbuf();
  std::ofstream ofs;

  if (!outFile.empty())
  {
    ofs.open(outFile.c_str());
    if (ofs.is_open())
    {
      std::cout.rdbuf(ofs.rdbuf());
      std::cerr.rdbuf(ofs.rdbuf());
    }
  }

  std::vector<Vector> positions = ParseUavPositions(uavPos, uavAltitude);
  NS_ABORT_MSG_IF(positions.empty(), "uavPos is empty or invalid");
  uint32_t nUav = static_cast<uint32_t>(positions.size());

  NS_ABORT_MSG_IF(masterUavId >= nUav, "masterUavId out of range");
  NS_ABORT_MSG_IF(srcId >= nUav, "srcId out of range");
  NS_ABORT_MSG_IF(dstId >= nUav, "dstId out of range");
  NS_ABORT_MSG_IF(packetSize == 0, "packetSize must be > 0");
  NS_ABORT_MSG_IF(intervalUs <= 0.0, "intervalUs must be > 0");
  NS_ABORT_MSG_IF(probeDurationSec <= 0.0, "probeDurationSec must be > 0");

  uint32_t probePackets = std::max<uint32_t>(
      1, static_cast<uint32_t>(std::floor(probeDurationSec * 1e6 / intervalUs)));

  NodeContainer uavNodes;
  uavNodes.Create(nUav);

  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(uavNodes);

  for (uint32_t i = 0; i < nUav; ++i)
  {
    Ptr<MobilityModel> mob = uavNodes.Get(i)->GetObject<MobilityModel>();
    mob->SetPosition(positions[i]);
  }

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
  
  if (rateManager == "ideal")
  {
    backhaulWifi.SetRemoteStationManager("ns3::IdealWifiManager");
  }
  else if (rateManager == "constant6")
  {
    backhaulWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                         "DataMode", StringValue("ErpOfdmRate6Mbps"),
                                         "ControlMode", StringValue("ErpOfdmRate6Mbps"));
  }
  else
  {
    NS_ABORT_MSG("Unsupported rateManager: " << rateManager
                 << " (supported: ideal, constant6)");
  }

  NetDeviceContainer devices = backhaulWifi.Install(backhaulPhy, backhaulMac, uavNodes);

  InternetStackHelper stack;
  stack.Install(uavNodes);

  uint32_t hostCount = uavNodes.GetN();
  uint32_t prefix = SmallestPrefixForHosts(hostCount);
  Ipv4Mask mask = PrefixToIpv4Mask(prefix);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase(Ipv4Address("10.1.0.0"), mask);
  Ipv4InterfaceContainer ifs = ipv4.Assign(devices);

  std::cout << "[PROBE-SETUP]"
            << " nUav=" << nUav
            << " masterUavId=" << masterUavId
            << " rateManager=" << rateManager
            << " backhaulRange3d=" << backhaulRange3d
            << " packetSize=" << packetSize
            << " intervalUs=" << intervalUs
            << " probeDurationSec=" << probeDurationSec
            << " probePackets=" << probePackets
            << " subnetPrefix=/" << prefix
            << "\n";

  std::vector<std::pair<uint32_t, uint32_t>> pairs;
  if (probeAllSlavesToMaster)
  {
    for (uint32_t u = 0; u < nUav; ++u)
    {
      if (u == masterUavId)
        continue;
      pairs.push_back({u, masterUavId});
    }
  }
  else
  {
    NS_ABORT_MSG_IF(srcId == dstId, "srcId and dstId must be different");
    pairs.push_back({srcId, dstId});
  }

  uint16_t basePort = 45000;
  double currentStartSec = startSec;
  Time interval = MicroSeconds(static_cast<int64_t>(intervalUs));

  for (uint32_t i = 0; i < pairs.size(); ++i)
  {
    uint32_t flowId = i + 1;
    uint32_t s = pairs[i].first;
    uint32_t d = pairs[i].second;
    uint16_t port = basePort + static_cast<uint16_t>(flowId);

    Ptr<Node> srcNode = uavNodes.Get(s);
    Ptr<Node> dstNode = uavNodes.Get(d);
    Ipv4Address dstIp = ifs.GetAddress(d);

    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApps = sinkHelper.Install(dstNode);
    sinkApps.Start(Seconds(0.0));

    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApps.Get(0));
    sink->TraceConnectWithoutContext("Rx",
                                     MakeBoundCallback(&CountProbeRx, flowId));

    Ptr<Socket> sock = Socket::CreateSocket(srcNode, UdpSocketFactory::GetTypeId());
    sock->Connect(InetSocketAddress(dstIp, port));

    Vector ps = srcNode->GetObject<MobilityModel>()->GetPosition();
    Vector pd = dstNode->GetObject<MobilityModel>()->GetPosition();
    double distanceM = ComputeDistanceM(ps, pd);
    double rxPowerDbm = ComputeFriisRxPowerDbm(txPowerDbm, wifiFreqHz, distanceM);
    double noiseDbm = ComputeNoiseDbm(backhaulBandwidthHz, backhaulNoiseFigureDb);
    double snrDb = rxPowerDbm - noiseDbm;
    double estRateMbps = ComputeShannonRateBps(backhaulBandwidthHz, snrDb) / 1e6;

    ProbeFlowState st;
    st.flowId = flowId;
    st.srcId = s;
    st.dstId = d;
    st.srcRole = (s == masterUavId ? "MASTER" : "SLAVE");
    st.dstRole = (d == masterUavId ? "MASTER" : "SLAVE");
    st.srcPos = ps;
    st.dstPos = pd;
    st.distanceM = distanceM;
    st.snrDb = snrDb;
    st.estRateMbps = estRateMbps;
    g_probeFlows[flowId] = st;

    std::cout << "[PROBE-PLAN]"
              << " flowId=" << flowId
              << " src=UAV_" << s
              << " dst=UAV_" << d
              << " srcRole=" << st.srcRole
              << " dstRole=" << st.dstRole
              << " startSec=" << currentStartSec
              << " srcPos=(" << ps.x << "," << ps.y << "," << ps.z << ")"
              << " dstPos=(" << pd.x << "," << pd.y << "," << pd.z << ")"
              << " distanceM=" << distanceM
              << " snrDb=" << snrDb
              << " estRateMbps=" << estRateMbps
              << " dstIp=" << dstIp
              << "\n";

    Simulator::Schedule(Seconds(currentStartSec),
                        &SendProbePacket,
                        sock,
                        flowId,
                        packetSize,
                        probePackets,
                        interval);

    double probeWindowSec =
        std::max(1.0, (probePackets - 1) * intervalUs * 1e-6 + 1.0);
    currentStartSec += probeWindowSec + pairGapSec;
  }

  double simStopSec = currentStartSec + 2.0;
  Simulator::Stop(Seconds(simStopSec));
  Simulator::Run();

  PrintProbeSummary(csvOutFile);

  Simulator::Destroy();

  if (ofs.is_open())
  {
    std::cout.rdbuf(coutBuf);
    std::cerr.rdbuf(cerrBuf);
    ofs.close();
  }

  return 0;
}
