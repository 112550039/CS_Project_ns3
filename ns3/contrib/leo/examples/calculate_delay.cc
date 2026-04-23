/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Tim Schubert <ns-3-leo@timschubert.net>
 */

#include <iostream>
#include <unordered_map>
#include <vector>
#include <numeric>
#include <iomanip>
#include <algorithm>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/leo-module.h"
#include "ns3/network-module.h"
#include "ns3/aodv-module.h"
#include "ns3/udp-server.h"
// === SNR / Rate helpers (add this above main) ===
#include "ns3/mock-net-device.h"
#include "ns3/mock-channel.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include <cmath>
#include <limits>
#include "ns3/leo-starlink-constants.h" 

using namespace ns3;

// 送出時間表（key=TCP序號，value=送出時刻）
static std::unordered_map<uint32_t, ns3::Time> g_txTime;

// 收到每個封包的 E2E delay（秒）
static std::vector<double> g_e2eDelays;

// 這個 callback 會同時接住 TCP 的 Tx 與 Rx 事件
static void EchoTxRx(std::string ctx,
                     const ns3::Ptr<const ns3::Packet> pkt,
                     const ns3::TcpHeader& hdr,
                     const ns3::Ptr<const ns3::TcpSocketBase> sock)
{
    using namespace ns3;
    const uint32_t seq = hdr.GetSequenceNumber().GetValue();
    const Time now = Simulator::Now();

    // 保留原本的「逐行輸出」
    std::cout << now.GetNanoSeconds() << "ns:" << ctx
              << ":" << pkt->GetUid()
              << ":" << sock->GetNode()
              << ":" << hdr.GetSequenceNumber()
              << std::endl;

    // 1) Tx：記錄該序號的送出時間
    if (ctx.find("/Tx") != std::string::npos) {
        g_txTime[seq] = now;
        return;
    }

    // 2) Rx：找出相同序號的送出時間，計算 E2E delay
    if (ctx.find("/Rx") != std::string::npos) {
        auto it = g_txTime.find(seq);
        if (it != g_txTime.end()) {
            const double dsec = (now - it->second).GetSeconds();
            g_e2eDelays.push_back(dsec);
            g_txTime.erase(it);
        }
        return;
    }
}

// 依 TA 範例：固定雜訊 -90 dBm；這裡改成「把 B_Hz 當參數」而不是問 device
static double FindDataRate(double B_Hz, double rxPowerDbm) {
  const double noiseDbm = -90.0; // 若要物理雜訊：改為 -174 + 10log10(B_Hz) + NF
  const double snrDb  = rxPowerDbm - noiseDbm;
  const double snrLin = std::pow(10.0, snrDb / 10.0);
  const double se_bps_per_hz = std::log2(1.0 + snrLin);    // Shannon
  return B_Hz * se_bps_per_hz;
}
/* 若要改成物理雜訊：請把上面的 noiseDbm 換成
   double NF_dB = 5.0; // 先假設 5 dB，報告註明
   double noiseDbm = -174.0 + 10.0*std::log10(B_Hz) + NF_dB;
*/

// 找出 node 上屬於 LEO 無線（MockChannel）的 MockNetDevice（可回傳多個）
static std::vector<Ptr<MockNetDevice>> FindMockDevsOnMockChannel(Ptr<Node> n) {
  std::vector<Ptr<MockNetDevice>> out;
  for (uint32_t i = 0; i < n->GetNDevices(); ++i) {
    Ptr<MockNetDevice> md = DynamicCast<MockNetDevice>(n->GetDevice(i));
    if (md && DynamicCast<MockChannel>(md->GetChannel())) {
      out.push_back(md);
    }
  }
  return out;
}

// 給定 A 的某個 MockNetDevice，從同一條 MockChannel 上找出「RxPower 最好」的對端裝置
static Ptr<MockNetDevice> FindBestPeerOnSameChannel(Ptr<MockNetDevice> mdevA, Ptr<MobilityModel> mA, double txPowerDbm, double* bestRxDbmOut) {
  Ptr<MockChannel> chA = DynamicCast<MockChannel>(mdevA->GetChannel());
  if (!chA) return nullptr;

  Ptr<PropagationLossModel> pLoss = chA->GetPropagationLoss();
  if (!pLoss) return nullptr;

  double bestRx = -1e9;
  Ptr<MockNetDevice> bestDev = nullptr;

  // 掃過所有節點的所有 MockNetDevice，找出「在同一條 channel」的對端
  for (uint32_t nid = 0; nid < NodeList::GetNNodes(); ++nid) {
    Ptr<Node> n = NodeList::GetNode(nid);
    for (uint32_t i = 0; i < n->GetNDevices(); ++i) {
      Ptr<MockNetDevice> md = DynamicCast<MockNetDevice>(n->GetDevice(i));
      if (!md) continue;
      Ptr<MockChannel> chB = DynamicCast<MockChannel>(md->GetChannel());
      if (!chB || chB != chA) continue;                  // 不同通道略過
      if (md == mdevA) continue;                         // 自己略過

      Ptr<MobilityModel> mB = n->GetObject<MobilityModel>();
      if (!mB) continue;

      double rxDbm = pLoss->CalcRxPower(txPowerDbm, mA, mB);
      if (rxDbm > bestRx) {
        bestRx = rxDbm;
        bestDev = md;
      }
    }
  }

  if (bestDev && bestRxDbmOut) *bestRxDbmOut = bestRx;
  return bestDev;
}


static void ReportLinkSNR(uint32_t nodeA, uint32_t /*nodeB_ignored*/, const std::string& tag = "TCP-PAIR") {
  Ptr<Node> A = NodeList::GetNode(nodeA);
  if (!A) {
    std::cout << "[SNR] invalid node " << nodeA << std::endl;
    return;
  }

  // 1) 在 A 上找到屬於 LEO 無線的裝置（可能有多個）
  auto mdevsA = FindMockDevsOnMockChannel(A);
  if (mdevsA.empty()) {
    std::cout << "[SNR] node " << nodeA << " has no MockNetDevice on MockChannel\n";
    return;
  }

  Ptr<MobilityModel> mA = A->GetObject<MobilityModel>();
  if (!mA) {
    std::cout << "[SNR] node " << nodeA << " missing MobilityModel\n";
    return;
  }

  // 我們挑第一個（通常每個角色只掛一個），也可改成逐一比對
  Ptr<MockNetDevice> mdevA = mdevsA.front();

  // 2) 從相同通道上找出最佳對端（通常是與之相連的衛星/地面站）
  double txPowerDbm = mdevA->GetTxPower();               // 以 EIRP 發送
  double bestRxDbm = -1e9;
  Ptr<MockNetDevice> mdevBest = FindBestPeerOnSameChannel(mdevA, mA, txPowerDbm, &bestRxDbm);

  if (!mdevBest) {
    std::cout << "[SNR] node " << nodeA << " no peer on same MockChannel (link cut)\n";
    return;
  }

  Ptr<Node> B = mdevBest->GetNode();
  Ptr<MobilityModel> mB = B->GetObject<MobilityModel>();
  if (!mB) {
    std::cout << "[SNR] peer node missing MobilityModel\n";
    return;
  }

  // 距離（公尺）
  double d_m = mA->GetDistanceFrom(mB);

  // 3) 頻寬來自 Starlink USER 常數（GHz→Hz）
  const double B_Hz  = LEO_STARLINK_USER_BANDWIDTH * 1e9;
  const double B_MHz = B_Hz / 1e6;

  // 4) 估算可支援資料率
  double rate_bps  = FindDataRate(B_Hz, bestRxDbm);
  double rate_Mbps = rate_bps / 1e6;

  std::cout << "[SNR] " << tag
            << " A=" << nodeA << " BestPeer=" << B->GetId()
            << " d=" << d_m/1000.0 << " km"
            << " Rx=" << bestRxDbm << " dBm"
            << " EstRate≈" << rate_Mbps << " Mbps"
            << " (B=" << B_MHz << " MHz)"
            << std::endl;
}




void connect ()
{
    Config::ConnectFailSafe(
        "/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Tx",
        MakeCallback(&EchoTxRx));

    Config::ConnectFailSafe(
        "/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Rx",
        MakeCallback(&EchoTxRx));
}

void initial_position (const NodeContainer &satellites, int sz)
{
    for(int i = 0; i < std::min((int)satellites.GetN(), sz); i++){
        // Get satellite position
        Vector pos = satellites.Get(i)->GetObject<MobilityModel>()->GetPosition();
        // Convert position to latitude & longtitude
        double r = sqrt(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z);
        double lat = asin(pos.z / r) * 180.0 / M_PI;
        double longit = atan2(pos.y, pos.x) * 180 / M_PI;
        std::cout << "Satellite " << i << " latitude = " << lat << ", longtitude = " << longit << endl;
    }
}

NS_LOG_COMPONENT_DEFINE ("LeoBulkSendTracingExample");

int main (int argc, char *argv[])
{

    CommandLine cmd;
    std::string orbitFile;
    std::string traceFile;
    LeoLatLong source (6.06692, 73.0213);
    LeoLatLong destination (7.06692, 74.0213);
    std::string islRate = "2Gbps";
    std::string constellation = "TelesatGateway";
    uint16_t port = 9;
    uint32_t latGws = 20;
    uint32_t lonGws = 20;
    double duration = 100;
    bool islEnabled = false;
    bool pcap = false;
    uint64_t ttlThresh = 0;
    std::string routingProto = "aodv";

    cmd.AddValue("orbitFile", "CSV file with orbit parameters", orbitFile);
    cmd.AddValue("traceFile", "CSV file to store mobility trace in", traceFile);
    cmd.AddValue("precision", "ns3::LeoCircularOrbitMobilityModel::Precision");
    cmd.AddValue("duration", "Duration of the simulation in seconds", duration);
    cmd.AddValue("source", "Traffic source", source);
    cmd.AddValue("destination", "Traffic destination", destination);
    cmd.AddValue("islRate", "ns3::MockNetDevice::DataRate");
    cmd.AddValue("constellation", "LEO constellation link settings name", constellation);
    cmd.AddValue("routing", "Routing protocol", routingProto);
    cmd.AddValue("islEnabled", "Enable inter-satellite links", islEnabled);
    cmd.AddValue("latGws", "Latitudal rows of gateways", latGws);
    cmd.AddValue("lonGws", "Longitudinal rows of gateways", lonGws);
    cmd.AddValue("ttlThresh", "TTL threshold", ttlThresh);
    cmd.AddValue("destOnly", "ns3::aodv::RoutingProtocol::DestinationOnly");
    cmd.AddValue("routeTimeout", "ns3::aodv::RoutingProtocol::ActiveRouteTimeout");
    cmd.AddValue("pcap", "Enable packet capture", pcap);
    cmd.Parse (argc, argv);

    std::streambuf *coutbuf = std::cout.rdbuf();
    // redirect cout if traceFile
    std::ofstream out;
    out.open (traceFile);
    if (out.is_open ())
    {
        std::cout.rdbuf(out.rdbuf());
    }

    LeoOrbitNodeHelper orbit;
    NodeContainer satellites;
    if (!orbitFile.empty())
    {
        satellites = orbit.Install (orbitFile);
    }
    else
    {
        satellites = orbit.Install ({ LeoOrbit (1200, 20, 5, 5) });
    }

    LeoGndNodeHelper ground;
    NodeContainer users = ground.Install (source, destination);

    LeoChannelHelper utCh;
    utCh.SetConstellation (constellation);
    utCh.SetGndDeviceAttribute("DataRate", StringValue("8kbps"));
    NetDeviceContainer utNet = utCh.Install (satellites, users);

    initial_position(satellites, 5);

    InternetStackHelper stack;
    AodvHelper aodv;
    aodv.Set ("EnableHello", BooleanValue (false));
    //aodv.Set ("HelloInterval", TimeValue (Seconds (10)));
    if (ttlThresh != 0)
    {
        aodv.Set ("TtlThreshold", UintegerValue (ttlThresh));
        aodv.Set ("NetDiameter", UintegerValue (2*ttlThresh));
    }
    stack.SetRoutingHelper (aodv);

    // Install internet stack on nodes
    stack.Install (satellites);
    stack.Install (users);

    Ipv4AddressHelper ipv4;

    ipv4.SetBase ("10.1.0.0", "255.255.0.0");
    ipv4.Assign (utNet);

    if (islEnabled)
    {
        std::cerr << "ISL enabled" << std::endl;
        IslHelper islCh;
        NetDeviceContainer islNet = islCh.Install (satellites);
        ipv4.SetBase ("10.2.0.0", "255.255.0.0");
        ipv4.Assign (islNet);
    }

    Ipv4Address remote = users.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
    BulkSendHelper sender ("ns3::TcpSocketFactory",
            InetSocketAddress (remote, port));
    // Set the amount of data to send in bytes.  Zero is unlimited.
    sender.SetAttribute ("MaxBytes", UintegerValue (1024));
    sender.SetAttribute ("SendSize", UintegerValue (512));
    ApplicationContainer sourceApps = sender.Install (users.Get (0));
    sourceApps.Start (Seconds (0.0));

    //
    // Create a PacketSinkApplication and install it on node 1
    //
    PacketSinkHelper sink ("ns3::TcpSocketFactory",
            InetSocketAddress (Ipv4Address::GetAny (), port));
    ApplicationContainer sinkApps = sink.Install (users.Get (1));
    sinkApps.Start (Seconds (0.0));

    // Fix segmentation fault
    Simulator::Schedule(Seconds(1e-7), &connect);

    //
    // Set up tracing if enabled
    //
    if (pcap)
    {
        AsciiTraceHelper ascii;
        utCh.EnableAsciiAll (ascii.CreateFileStream ("tcp-bulk-send.tr"));
        utCh.EnablePcapAll ("tcp-bulk-send", false);
    }

    std::cerr << "LOCAL =" << users.Get (0)->GetId () << std::endl;
    std::cerr << "REMOTE=" << users.Get (1)->GetId () << ",addr=" << Ipv4Address::ConvertFrom (remote) << std::endl;

    Simulator::Schedule(Seconds(0.2), &ReportLinkSNR, (uint32_t)25, (uint32_t)26, std::string("TCP-PAIR"));


    NS_LOG_INFO ("Run Simulation.");
    Simulator::Stop (Seconds (duration));
    Simulator::Run ();
    Simulator::Destroy ();
    NS_LOG_INFO ("Done.");

    Ptr<PacketSink> sink1 = DynamicCast<PacketSink> (sinkApps.Get (0));
    std::cout << users.Get (0)->GetId () << ":" << users.Get (1)->GetId () << ": " << sink1->GetTotalRx () << std::endl;

    // TODO: Output End-to-end Delay
    double avg = 0.0;
    if (!g_e2eDelays.empty()) {
        const double sum = std::accumulate(g_e2eDelays.begin(), g_e2eDelays.end(), 0.0);
        avg = sum / g_e2eDelays.size();
    }
    std::cout << "Packet average end-to-end delay is "
            << std::fixed << std::setprecision(6) << avg << "s" << std::endl;

    out.close ();
    std::cout.rdbuf(coutbuf);

    return 0;
}
