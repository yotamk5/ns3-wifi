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
#include "ns3/ap-wifi-mac.h"
#include "ns3/wifi-phy-state-helper.h"
#include "ns3/qos-txop.h"
#include "ns3/wifi-tx-vector.h"
#include "ns3/wifi-psdu.h"

#include <fstream>
#include <iomanip>
#include <map>
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

// ── queue stats tracking ─────────────────────────────────────────────────────

struct QueueStatsAccum
{
    double   sumBytes{0};
    double   sumHoqMs{0};
    uint32_t count{0};

    void Add(double bytes, double hoqMs) { sumBytes += bytes; sumHoqMs += hoqMs; ++count; }
    double AvgBytes() const { return count > 0 ? sumBytes / count : 0.0; }
    double AvgHoqMs() const { return count > 0 ? sumHoqMs / count : 0.0; }
    void Clear() { sumBytes = 0; sumHoqMs = 0; count = 0; }
};

struct QueueStatsSnapshot
{
    uint32_t apNodeId;
    double   timeS;
    double   avgQueueBytes;
    double   avgHoqMs;
    uint32_t sampleCount;
};

// nodeId -> accumulator for the current 100ms window
static std::unordered_map<uint32_t, QueueStatsAccum> g_queueStatsAccum;

// ── TX stats tracking ────────────────────────────────────────────────────────

struct TxStatsAccum
{
    std::map<uint16_t, uint32_t> bwHist; // bw MHz -> tx count
    double   sumRateMbps{0};
    uint32_t count{0};

    void Add(uint16_t bw, double rateMbps) { bwHist[bw]++; sumRateMbps += rateMbps; ++count; }
    double AvgRateMbps() const { return count > 0 ? sumRateMbps / count : 0.0; }
    void Clear() { bwHist.clear(); sumRateMbps = 0; count = 0; }
};

struct TxStatsSnapshot
{
    uint32_t apNodeId;
    double   timeS;
    std::map<uint16_t, uint32_t> bwHist;
    double   avgRateMbps;
    uint32_t count;
    uint32_t run;
};

// nodeId -> accumulator for the current 100ms window
static std::unordered_map<uint32_t, TxStatsAccum> g_txStatsAccum;

// ── PHY state tracking ────────────────────────────────────────────────────────

struct PhyStateRecord
{
    uint64_t duration[(uint32_t)WifiPhyState::OFF] = {}; // ns per state
    uint64_t total{0};                                    // ns total

    double PctOf(WifiPhyState s) const
    {
        return total > 0 ? double(duration[(uint32_t)s]) * 100.0 / double(total) : 0.0;
    }

    void Clear()
    {
        std::fill(std::begin(duration), std::end(duration), 0);
        total = 0;
    }
};

// nodeId -> accumulated PHY state durations (AP nodes only)
static std::unordered_map<uint32_t, PhyStateRecord> g_phyState;

// Callback: fired by WifiPhyStateHelper "State" trace
static void
PhyStateCallback(uint32_t nodeId, Time warmupEnd,
                 Time start, Time duration, WifiPhyState state)
{
    if (Simulator::Now() < warmupEnd)
    {
        return; // ignore warmup
    }
    auto& rec = g_phyState[nodeId]; // zero-initializes on first access
    rec.duration[(uint32_t)state] += duration.GetNanoSeconds();
    rec.total                     += duration.GetNanoSeconds();
}

// Read percentages for a node and clear the record
static PhyStateRecord
ReadAndClearPhyState(uint32_t nodeId)
{
    PhyStateRecord rec = g_phyState[nodeId];
    g_phyState[nodeId].Clear();
    return rec;
}

// Print PHY state summary to stdout and write to CSV
static void
WritePhyStateSummary(std::shared_ptr<SimCtx> ctx)
{
    std::cout << "\nPHY state summary (AP nodes):\n";
    std::cout << std::fixed << std::setprecision(1);
    ctx->csvPhy->precision(2);
    *ctx->csvPhy << std::fixed;

    for (auto& [nodeId, rec] : g_phyState)
    {
        double idle    = rec.PctOf(WifiPhyState::IDLE);
        double ccaBusy = rec.PctOf(WifiPhyState::CCA_BUSY);
        double tx      = rec.PctOf(WifiPhyState::TX);
        double rx      = rec.PctOf(WifiPhyState::RX);

        std::cout << "  AP node " << nodeId
                  << "  IDLE="     << idle    << "%"
                  << "  CCA_BUSY=" << ccaBusy << "%"
                  << "  TX="       << tx      << "%"
                  << "  RX="       << rx      << "%\n";

        *ctx->csvPhy << ctx->run << ',' << nodeId << ','
                     << idle << ',' << ccaBusy << ',' << tx << ',' << rx << '\n';
    }
    ctx->csvPhy->close();
}

// ── shared context ────────────────────────────────────────────────────────────

struct SimCtx
{
    uint32_t    run;
    std::string proto;      // "udp" or "tcp"
    Time        warmupEnd;  // packets received before this time are ignored
    std::unordered_map<uint64_t, std::pair<Time, uint32_t>> enqueueTime;
    // UID -> {enqueue time, sender nodeId}
    std::shared_ptr<std::ofstream> csvDelay;
    std::shared_ptr<std::ofstream> csvPhy;
    std::shared_ptr<std::ofstream> csvQueueStats;
    std::shared_ptr<std::ofstream> csvTxStats;
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

    ctx->csvPhy = std::make_shared<std::ofstream>();
    ctx->csvPhy->open("wifi_phy_run" + std::to_string(rngRun) + ".csv");
    *ctx->csvPhy << "run,ap_node,idle_pct,cca_busy_pct,tx_pct,rx_pct\n";

    ctx->csvQueueStats = std::make_shared<std::ofstream>();
    ctx->csvQueueStats->open("wifi_queuestats_run" + std::to_string(rngRun) + ".csv");
    *ctx->csvQueueStats << "run,ap_node,time_s,avg_queue_bytes,avg_hoq_ms,sample_count\n";

    ctx->csvTxStats = std::make_shared<std::ofstream>();
    ctx->csvTxStats->open("wifi_txstats_run" + std::to_string(rngRun) + ".csv");
    *ctx->csvTxStats << "run,ap_node,time_s,bw_20_tx,bw_40_tx,bw_80_tx,bw_160_tx,avg_phy_rate_mbps,tx_count\n";

    return ctx;
}

static void
ConnectPhyState(Ptr<Node> node, Time warmupEnd)
{
    std::ostringstream path;
    path << "/NodeList/" << node->GetId()
         << "/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State";
    Config::ConnectWithoutContext(path.str(),
        MakeBoundCallback(&PhyStateCallback, node->GetId(), warmupEnd));
}

// Fires at the end of every BE TXOP. Accumulates BE queue byte count into g_queueStatsAccum.
static void
QueueStatsSample(std::shared_ptr<SimCtx> ctx, uint32_t apNodeId, Ptr<Node> apNode,
          Time /* txopStart */, Time /* txopDuration */, uint8_t /* linkId */)
{
    if (Simulator::Now() < ctx->warmupEnd)
    {
        return;
    }
    auto wnd   = DynamicCast<WifiNetDevice>(apNode->GetDevice(0));
    auto mac   = DynamicCast<ApWifiMac>(wnd->GetMac());
    auto queue = mac->GetQosTxop(AC_BE)->GetWifiMacQueue();
    double bytes = static_cast<double>(queue->GetNBytes());
    auto front   = queue->Peek();
    double hoqMs = front ? (Simulator::Now() - front->GetTimestamp()).GetSeconds() * 1000.0 : 0.0;
    g_queueStatsAccum[apNodeId].Add(bytes, hoqMs);
}

// Called every 100ms: reads accumulators, fills QueueStatsSnapshot per AP, writes to CSV.
static void
ComputeAndStoreQueueStatsAvg(std::shared_ptr<SimCtx> ctx,
                      std::vector<uint32_t> apNodeIds,
                      Time interval)
{
    if (Simulator::Now() >= ctx->warmupEnd)
    {
        for (uint32_t apNodeId : apNodeIds)
        {
            QueueStatsSnapshot snap;
            snap.apNodeId    = apNodeId;
            snap.timeS       = Simulator::Now().GetSeconds();
            snap.avgQueueBytes = g_queueStatsAccum[apNodeId].AvgBytes();
            snap.avgHoqMs      = g_queueStatsAccum[apNodeId].AvgHoqMs();
            snap.sampleCount   = g_queueStatsAccum[apNodeId].count;
            snap.bwHist        = g_queueStatsAccum[apNodeId].bwHist;
            g_queueStatsAccum[apNodeId].Clear();

            *ctx->csvQueueStats << ctx->run << ',' << snap.apNodeId << ','
                              << snap.timeS << ','
                              << std::fixed << std::setprecision(0) << snap.avgQueueBytes << ','
                              << std::defaultfloat << std::setprecision(6) << snap.avgHoqMs << ','
                              << snap.sampleCount << ',';
            for (uint16_t bw : {20, 40, 80, 160})
            {
                auto it = snap.bwHist.find(bw);
                *ctx->csvQueueStats << (it != snap.bwHist.end() ? it->second : 0);
                if (bw != 160) *ctx->csvQueueStats << ',';
            }
            *ctx->csvQueueStats << '\n';
            ctx->csvQueueStats->flush();
        }
    }
    Simulator::Schedule(interval, &ComputeAndStoreQueueStatsAvg, ctx, apNodeIds, interval);
}

static void
ConnectQueueStatsTrace(std::shared_ptr<SimCtx> ctx, Ptr<Node> apNode)
{
    auto wnd  = DynamicCast<WifiNetDevice>(apNode->GetDevice(0));
    auto mac  = DynamicCast<ApWifiMac>(wnd->GetMac());
    auto txop = mac->GetQosTxop(AC_BE);
    bool ok   = txop->TraceConnectWithoutContext("TxopTrace",
                    MakeBoundCallback(&QueueStatsSample, ctx, apNode->GetId(), apNode));
    NS_ABORT_MSG_IF(!ok, "Failed to connect TxopTrace on AP node " << apNode->GetId());
}

// Fires at the start of every PHY transmission. Records BW and PHY rate.
static void
PhyTxStatsCallback(std::shared_ptr<SimCtx> ctx, uint32_t apNodeId,
                   WifiConstPsduMap /* psduMap */, WifiTxVector txVector, double /* txPowerW */)
{
    if (Simulator::Now() < ctx->warmupEnd)
    {
        return;
    }
    uint16_t bw       = txVector.GetChannelWidth();
    double rateMbps   = txVector.GetMode().GetDataRate(txVector) / 1e6;
    g_txStatsAccum[apNodeId].Add(bw, rateMbps);
}

static void
ConnectPhyTxStats(std::shared_ptr<SimCtx> ctx, Ptr<Node> apNode)
{
    std::ostringstream path;
    path << "/NodeList/" << apNode->GetId()
         << "/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxPsduBegin";
    Config::ConnectWithoutContext(path.str(),
        MakeBoundCallback(&PhyTxStatsCallback, ctx, apNode->GetId()));
}

// Called every 100ms: reads TX stats accumulators and writes to CSV.
static void
ComputeAndStoreTxStatsAvg(std::shared_ptr<SimCtx> ctx,
                          std::vector<uint32_t> apNodeIds,
                          Time interval)
{
    if (Simulator::Now() >= ctx->warmupEnd)
    {
        for (uint32_t apNodeId : apNodeIds)
        {
            TxStatsSnapshot snap;
            snap.run         = ctx->run;
            snap.apNodeId    = apNodeId;
            snap.timeS       = Simulator::Now().GetSeconds();
            snap.bwHist      = g_txStatsAccum[apNodeId].bwHist;
            snap.avgRateMbps = g_txStatsAccum[apNodeId].AvgRateMbps();
            snap.count       = g_txStatsAccum[apNodeId].count;
            g_txStatsAccum[apNodeId].Clear();

            *ctx->csvTxStats << snap.run << ',' << snap.apNodeId << ',' << snap.timeS << ',';
            for (uint16_t bw : {20, 40, 80, 160})
            {
                auto it = snap.bwHist.find(bw);
                *ctx->csvTxStats << (it != snap.bwHist.end() ? it->second : 0) << ',';
            }
            *ctx->csvTxStats << snap.avgRateMbps << ',' << snap.count << '\n';
            ctx->csvTxStats->flush();
        }
    }
    Simulator::Schedule(interval, &ComputeAndStoreTxStatsAvg, ctx, apNodeIds, interval);
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

    // --- build topology ---
    std::vector<uint32_t> apNodeIds;
    for (uint32_t ap = 0; ap < nAps; ++ap)
    {
        Ssid ssid("ap" + std::to_string(ap));

        // AP node
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

        uint32_t apNodeId = apNode.Get(0)->GetId();
        g_nodeInfo[apNodeId] = {true};
        apNodeIds.push_back(apNodeId);

        ConnectEnqueue(ctx, apNode.Get(0));
        ConnectMacRx(ctx, apNode.Get(0));
        ConnectPhyState(apNode.Get(0), ctx->warmupEnd);
        ConnectQueueStatsTrace(ctx, apNode.Get(0));
        ConnectPhyTxStats(ctx, apNode.Get(0));

        // STA nodes
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

            uint32_t staNodeId = staNode.Get(0)->GetId();
            g_nodeInfo[staNodeId] = {false};

            ConnectEnqueue(ctx, staNode.Get(0));
            ConnectMacRx(ctx, staNode.Get(0));

            uint16_t portDl = 9000 + ap * 100 + s;
            uint16_t portUl = 9500 + ap * 100 + s;

            // Downlink: AP -> STA
            if (dir == "dl" || dir == "both")
            {
                double startDl = warmupTime + jitterRng->GetValue();
                PacketSinkHelper sinkHelper(socketFactory,
                                            InetSocketAddress(Ipv4Address::GetAny(), portDl));
                auto sinkApps = sinkHelper.Install(staNode);
                sinkApps.Start(Seconds(0));

                OnOffHelper onoff(socketFactory,
                                  InetSocketAddress(staIf.GetAddress(0), portDl));
                onoff.SetConstantRate(DataRate(rate), 1024);
                auto app = onoff.Install(apNode);
                app.Start(Seconds(startDl));
                app.Stop(Seconds(simTime));
            }

            // Uplink: STA -> AP
            if (dir == "ul" || dir == "both")
            {
                double startUl = warmupTime + jitterRng->GetValue();
                PacketSinkHelper sinkHelper(socketFactory,
                                            InetSocketAddress(Ipv4Address::GetAny(), portUl));
                auto sinkApps = sinkHelper.Install(apNode);
                sinkApps.Start(Seconds(0));

                OnOffHelper onoff(socketFactory,
                                  InetSocketAddress(apIf.GetAddress(0), portUl));
                onoff.SetConstantRate(DataRate(rate), 1024);
                auto app = onoff.Install(staNode);
                app.Start(Seconds(startUl));
                app.Stop(Seconds(simTime));
            }
        }
    }

    // --- schedule periodic queue and TX stats computation ---
    Time statsInterval = MilliSeconds(100);
    Simulator::Schedule(statsInterval, &ComputeAndStoreQueueStatsAvg, ctx, apNodeIds, statsInterval);
    Simulator::Schedule(statsInterval, &ComputeAndStoreTxStatsAvg,    ctx, apNodeIds, statsInterval);

    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();
    Simulator::Destroy();

    // --- write PHY state CSV and print summary ---
    WritePhyStateSummary(ctx);

    ctx->csvDelay->close();
    ctx->csvQueueStats->close();
    ctx->csvTxStats->close();
    std::cout << "\nDone. Results in wifi_e2e_run" << rngRun << ".csv"
              << ", wifi_phy_run" << rngRun << ".csv"
              << ", wifi_queuestats_run" << rngRun << ".csv"
              << ", wifi_txstats_run" << rngRun << ".csv\n";
    return 0;
}
