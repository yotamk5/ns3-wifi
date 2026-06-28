/*
 * wifi-e2e-demo.cc
 *
 * Measure Wi-Fi e2e delay per packet, per AP, per STA, per RngRun.
 * Supports UDP and TCP, uplink and downlink.
 *
 * Definition of wifi_e2e_ms:
 *   time from MAC queue enqueue on sender
 *   to MacRx delivery on receiver (pure Wi-Fi layer, no app layer).
 *
 * Run examples:
 *   ./ns3 run "wifi-e2e-demo"
 *   ./ns3 run "wifi-e2e-demo --nAps=2 --nStas=3 --RngRun=2 --proto=tcp --dir=ul"
 *   ./ns3 run "wifi-e2e-demo --proto=udp --dir=both"
 *
 * Output: wifi_e2e_run<N>.csv
 * Columns: run,sender_node,receiver_node,dir,proto,time_s,wifi_e2e_ms
 */

#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/ssid.h"
#include "ns3/wifi-mpdu.h"
#include "ns3/wifi-net-device.h"

#include <fstream>
#include <unordered_map>
#include <sstream>
#include <string>

using namespace ns3;

// ── node identity lookup ──────────────────────────────────────────────────────

struct NodeInfo
{
    bool isAp;
};

static std::unordered_map<uint32_t, NodeInfo> g_nodeInfo; // nodeId -> info

// ── shared context ────────────────────────────────────────────────────────────

struct FlowRecord
{
    Ptr<PacketSink> sink;
    uint32_t        senderNodeId;
    uint32_t        receiverNodeId;
    std::string     dir;
};

struct SimCtx
{
    uint32_t    run;
    std::string proto;      // "udp" or "tcp"
    Time        warmupEnd;  // packets received before this time are ignored
    std::unordered_map<uint64_t, std::pair<Time, uint32_t>> enqueueTime;
    // UID -> {enqueue time, sender nodeId}
    std::shared_ptr<std::ofstream> csvDelay;
    std::shared_ptr<std::ofstream> csvTput;
    std::vector<FlowRecord>        flows;
};

// ── callbacks ─────────────────────────────────────────────────────────────────

// Fires when any Wi-Fi frame enters a MAC TX queue (AP or STA)
static void
MacEnqueue(std::shared_ptr<SimCtx> ctx, uint32_t senderNodeId, Ptr<const WifiMpdu> mpdu)
{
    uint64_t uid = mpdu->GetPacket()->GetUid();
    ctx->enqueueTime[uid] = {Simulator::Now(), senderNodeId};
}

// Fires when a Wi-Fi frame is delivered from MAC to IP on receiver (AP or STA)
static void
MacRx(std::shared_ptr<SimCtx> ctx, uint32_t receiverNodeId, Ptr<const Packet> pkt)
{
    if (Simulator::Now() < ctx->warmupEnd)
    {
        return; // still in warmup phase, discard
    }

    uint64_t uid = pkt->GetUid();
    auto it = ctx->enqueueTime.find(uid);
    if (it == ctx->enqueueTime.end())
    {
        return; // management/control frame or not our flow
    }

    Time     enqTime      = it->second.first;
    uint32_t senderNodeId = it->second.second;
    ctx->enqueueTime.erase(it);

    double e2eMs = (Simulator::Now() - enqTime).GetSeconds() * 1000.0;

    // Determine direction from sender/receiver node identity
    auto& sender   = g_nodeInfo.at(senderNodeId);
    auto& receiver = g_nodeInfo.at(receiverNodeId);

    // Skip if sender == receiver (shouldn't happen but guard anyway)
    if (senderNodeId == receiverNodeId)
    {
        return;
    }

    std::string dir;
    if (sender.isAp && !receiver.isAp)
    {
        dir = "dl";
    }
    else if (!sender.isAp && receiver.isAp)
    {
        dir = "ul";
    }
    else
    {
        return; // AP->AP or STA->STA — not expected in this topology
    }

    *ctx->csvDelay << ctx->run << ',' << senderNodeId << ',' << receiverNodeId << ','
                   << dir << ',' << ctx->proto << ','
                   << Simulator::Now().GetSeconds() << ','
                   << e2eMs << '\n';
    ctx->csvDelay->flush();
}

// ── helpers ───────────────────────────────────────────────────────────────────

static std::shared_ptr<SimCtx>
OpenCsvFiles(uint32_t rngRun, const std::string& proto, double warmupTime)
{
    auto ctx       = std::make_shared<SimCtx>();
    ctx->run       = rngRun;
    ctx->proto     = proto;
    ctx->warmupEnd = Seconds(warmupTime);

    ctx->csvDelay = std::make_shared<std::ofstream>();
    ctx->csvDelay->open("wifi_e2e_run" + std::to_string(rngRun) + ".csv");
    *ctx->csvDelay << "run,sender_node,receiver_node,dir,proto,time_s,wifi_e2e_ms\n";

    ctx->csvTput = std::make_shared<std::ofstream>();
    ctx->csvTput->open("wifi_tput_run" + std::to_string(rngRun) + ".csv");
    *ctx->csvTput << "run,ap_node,sta_node,dir,proto,throughput_mbps\n";

    return ctx;
}

static void
ConnectEnqueue(std::shared_ptr<SimCtx> ctx, Ptr<Node> node)
{
    std::ostringstream path;
    path << "/NodeList/" << node->GetId()
         << "/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/Queue/Enqueue";
    Config::ConnectWithoutContext(path.str(),
        MakeBoundCallback(&MacEnqueue, ctx, node->GetId()));
}

static void
ConnectMacRx(std::shared_ptr<SimCtx> ctx, Ptr<Node> node)
{
    std::ostringstream path;
    path << "/NodeList/" << node->GetId()
         << "/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx";
    Config::ConnectWithoutContext(path.str(),
        MakeBoundCallback(&MacRx, ctx, node->GetId()));
}

// ── main ──────────────────────────────────────────────────────────────────────

int
main(int argc, char* argv[])
{
    uint32_t    nAps       = 2;
    uint32_t    nStas      = 3;
    uint32_t    rngRun     = 1;
    double      simTime    = 5.0;
    double      warmupTime = 1.0; // seconds — rows before this are discarded
    double      jitter     = 0.5; // max random offset added to traffic start time
    std::string proto      = "udp";  // "udp" or "tcp"
    std::string dir        = "dl";   // "dl", "ul", or "both"
    std::string rate       = "512Kbps";

    CommandLine cmd;
    cmd.AddValue("nAps",       "Number of APs",                    nAps);
    cmd.AddValue("nStas",      "STAs per AP",                      nStas);
    cmd.AddValue("RngRun",     "RNG run index (seed)",             rngRun);
    cmd.AddValue("warmupTime", "Warmup duration to skip (s)",      warmupTime);
    cmd.AddValue("jitter",     "Max random traffic start offset (s)", jitter);
    cmd.AddValue("simTime", "Simulation time (s)",          simTime);
    cmd.AddValue("proto",   "Traffic protocol: udp or tcp", proto);
    cmd.AddValue("dir",     "Direction: dl, ul, or both",   dir);
    cmd.AddValue("rate",    "Data rate per flow",            rate);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(proto != "udp" && proto != "tcp", "proto must be udp or tcp");
    NS_ABORT_MSG_IF(dir != "dl" && dir != "ul" && dir != "both", "dir must be dl, ul, or both");

    RngSeedManager::SetRun(rngRun);

    std::string socketFactory = (proto == "udp") ? "ns3::UdpSocketFactory"
                                                  : "ns3::TcpSocketFactory";

    auto jitterRng = CreateObject<UniformRandomVariable>();
    jitterRng->SetAttribute("Min", DoubleValue(0.0));
    jitterRng->SetAttribute("Max", DoubleValue(jitter));

    // --- channel ---
    auto channel = CreateObject<MultiModelSpectrumChannel>();
    auto loss    = CreateObject<LogDistancePropagationLossModel>();
    auto delay   = CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->AddPropagationLossModel(loss);
    channel->SetPropagationDelayModel(delay);

    // --- PHY ---
    SpectrumWifiPhyHelper phy;
    phy.SetChannel(channel);
    phy.Set("ChannelSettings", StringValue("{6, 20, BAND_2_4GHZ, 0}"));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

    InternetStackHelper inet;
    Ipv4AddressHelper   addrHelper;
    addrHelper.SetBase("10.0.0.0", "255.255.255.0");

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // --- open CSV ---
    auto ctx = OpenCsvFiles(rngRun, proto, warmupTime);

    // topology records — built in the install loop, consumed in the traffic loop
    struct TopoRecord
    {
        Ptr<Node>          apNode;
        Ipv4Address        apAddr;
        Ptr<Node>          staNode;
        Ipv4Address        staAddr;
        uint32_t           apIdx;
        uint32_t           staIdx;
    };
    std::vector<TopoRecord> topo;

    // --- loop 1: install nodes, PHY/MAC, IP, traces ---
    for (uint32_t ap = 0; ap < nAps; ++ap)
    {
        Ssid ssid("ap" + std::to_string(ap));

        NodeContainer apNode;
        apNode.Create(1);
        mob.Install(apNode);
        apNode.Get(0)->GetObject<MobilityModel>()->SetPosition(
            {static_cast<double>(ap) * 20.0, 0, 0});

        WifiMacHelper macAp;
        macAp.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        auto apDev = wifi.Install(phy, macAp, apNode);
        inet.Install(apNode);
        auto apIf = addrHelper.Assign(apDev);
        addrHelper.NewNetwork();

        g_nodeInfo[apNode.Get(0)->GetId()] = {true};
        ConnectEnqueue(ctx, apNode.Get(0));
        ConnectMacRx(ctx, apNode.Get(0));

        for (uint32_t s = 0; s < nStas; ++s)
        {
            NodeContainer staNode;
            staNode.Create(1);
            mob.Install(staNode);
            staNode.Get(0)->GetObject<MobilityModel>()->SetPosition(
                {static_cast<double>(ap) * 20.0 + static_cast<double>(s + 1) * 3.0, 5, 0});

            WifiMacHelper macSta;
            macSta.SetType("ns3::StaWifiMac",
                           "Ssid", SsidValue(ssid),
                           "ActiveProbing", BooleanValue(false));
            auto staDev = wifi.Install(phy, macSta, staNode);
            inet.Install(staNode);
            auto staIf = addrHelper.Assign(staDev);
            addrHelper.NewNetwork();

            g_nodeInfo[staNode.Get(0)->GetId()] = {false};
            ConnectEnqueue(ctx, staNode.Get(0));
            ConnectMacRx(ctx, staNode.Get(0));

            topo.push_back({apNode.Get(0), apIf.GetAddress(0),
                            staNode.Get(0), staIf.GetAddress(0),
                            ap, s});
        }
    }

    // --- loop 2: install traffic ---
    for (auto& t : topo)
    {
        uint16_t portDl = 9000 + t.apIdx * 100 + t.staIdx;
        uint16_t portUl = 9500 + t.apIdx * 100 + t.staIdx;

        // Downlink: AP -> STA
        if (dir == "dl" || dir == "both")
        {
            double startDl = warmupTime + jitterRng->GetValue();
            PacketSinkHelper sinkHelper(socketFactory,
                                        InetSocketAddress(Ipv4Address::GetAny(), portDl));
            auto sinkApps = sinkHelper.Install(t.staNode);
            sinkApps.Start(Seconds(0));

            OnOffHelper onoff(socketFactory, InetSocketAddress(t.staAddr, portDl));
            onoff.SetConstantRate(DataRate(rate), 1024);
            auto app = onoff.Install(t.apNode);
            app.Start(Seconds(startDl));
            app.Stop(Seconds(simTime));

            ctx->flows.push_back({sinkApps.Get(0)->GetObject<PacketSink>(),
                                   t.apNode->GetId(), t.staNode->GetId(), "dl"});
        }

        // Uplink: STA -> AP
        if (dir == "ul" || dir == "both")
        {
            double startUl = warmupTime + jitterRng->GetValue();
            PacketSinkHelper sinkHelper(socketFactory,
                                        InetSocketAddress(Ipv4Address::GetAny(), portUl));
            auto sinkApps = sinkHelper.Install(t.apNode);
            sinkApps.Start(Seconds(0));

            OnOffHelper onoff(socketFactory, InetSocketAddress(t.apAddr, portUl));
            onoff.SetConstantRate(DataRate(rate), 1024);
            auto app = onoff.Install(t.staNode);
            app.Start(Seconds(startUl));
            app.Stop(Seconds(simTime));

            ctx->flows.push_back({sinkApps.Get(0)->GetObject<PacketSink>(),
                                   t.staNode->GetId(), t.apNode->GetId(), "ul"});
        }
    }

    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();
    Simulator::Destroy();

    // --- write throughput CSV ---
    double measuredDuration = simTime - warmupTime;
    for (auto& f : ctx->flows)
    {
        double tputMbps = (f.sink->GetTotalRx() * 8.0) / measuredDuration / 1e6;
        uint32_t apNode  = (f.dir == "dl") ? f.senderNodeId   : f.receiverNodeId;
        uint32_t staNode = (f.dir == "dl") ? f.receiverNodeId  : f.senderNodeId;
        *ctx->csvTput << ctx->run << ',' << apNode << ',' << staNode << ','
                      << f.dir << ',' << ctx->proto << ','
                      << tputMbps << '\n';
    }

    ctx->csvDelay->close();
    ctx->csvTput->close();
    std::cout << "Done. Results in wifi_e2e_run" << rngRun << ".csv"
              << " and wifi_tput_run" << rngRun << ".csv\n";
    return 0;
}
