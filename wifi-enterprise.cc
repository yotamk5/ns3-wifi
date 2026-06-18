/*
 * Wi-Fi Enterprise Network Simulation
 *
 * Configurable multi-AP indoor Wi-Fi network for enterprise research.
 * Supports 802.11a/n/ac/ax/be, SpectrumWifiPhy, indoor propagation presets,
 * UDP/TCP traffic, DL/UL/both directions, FlowMonitor, and PCAP output.
 *
 * Usage:
 *   ./ns3 run "wifi-enterprise --help"
 *   ./ns3 run "wifi-enterprise --nApsRow=2 --nApsCol=2 --nStasPerAp=5 --simTime=10"
 *
 * Warmup phase: each AP pings all its STAs before traffic starts, ensuring
 * BA sessions are active and Minstrel has probed rates before measurement.
 */

#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/neighbor-cache-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-net-device.h"
#include "ns3/ping-helper.h"
#include "ns3/wifi-static-setup-helper.h"
#include "ns3/pointer.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiEnterprise");

// =============================================================================
//  Configuration — all CLI-settable parameters in one struct
// =============================================================================

struct SimConfig
{
    // Topology
    uint32_t nApsRow{2};     ///< Rows of APs in the grid
    uint32_t nApsCol{2};     ///< Columns of APs in the grid
    double   apDistX{50.0};  ///< AP spacing on X axis (m)
    double   apDistY{50.0};  ///< AP spacing on Y axis (m)
    uint32_t nStasPerAp{5};  ///< Number of STAs per AP
    double   cellRadius{15.0}; ///< Max random STA distance from its AP (m)

    // Standard / PHY
    std::string standard{"11be"};      ///< 11a/11n/11ac/11ax/11be
    double      frequency{5.0};        ///< Band: 2.4, 5, or 6 GHz
    uint32_t    channelWidth{20};      ///< MHz: 20/40/80/160/320
    uint32_t    channelNumber{36};      ///< 0 = auto-select
    double      txPower{20.0};         ///< dBm
    double      rxSensitivity{-82.0};  ///< dBm
    uint32_t    guardInterval{800};    ///< ns: 400/800/1600/3200

    // Rate control
    std::string rateManager{"minstrel"}; ///< minstrel / ideal / constant
    int         mcs{7};                  ///< MCS index (constant mode only)

    // MAC
    uint32_t rtsThreshold{65535}; ///< bytes; 0=always RTS, 65535=disabled
    bool     enableAmpdu{true};
    bool     enableAmsdu{false};

    // Propagation / indoor environment
    std::string indoorEnv{"office"};   ///< office/stadium/library/hospital/plant
    double      pathLossExp{0.0};      ///< 0 = use indoorEnv preset
    double      shadowingSigma{0.0};   ///< dB; 0 = use indoorEnv preset

    // Traffic
    std::string traffic{"udp"};        ///< udp / tcp
    std::string direction{"downlink"}; ///< uplink / downlink / both
    double      dataRate{54.0};        ///< Mbps per STA (offered load)
    uint32_t    packetSize{1472};      ///< bytes

    // Simulation control
    double      simTime{10.0};   ///< seconds of actual data transfer
    uint32_t    nRuns{1};        ///< number of independent runs (seeds)
    uint32_t    seed{1};         ///< base RNG seed; run k uses seed+k
    bool        enablePcap{false};
    std::string outputPrefix{"wifi-enterprise"};
};

// =============================================================================
//  Indoor environment presets
// =============================================================================

struct IndoorParams
{
    double ple;      ///< path-loss exponent
    double sigma;    ///< shadowing std-dev (dB)
    double refLoss;  ///< reference loss at 1 m (dB), frequency-dependent
};

static IndoorParams
GetIndoorParams(const std::string& env, double freq)
{
    double ref = (freq < 3.0) ? 40.0 : (freq > 5.5) ? 48.0 : 46.0;
    if (env == "office")   return {3.0,  7.0, ref};
    if (env == "stadium")  return {2.2,  6.0, ref};
    if (env == "library")  return {3.3,  8.0, ref};
    if (env == "hospital") return {3.3,  9.0, ref};
    if (env == "plant")    return {3.5, 10.0, ref};
    NS_LOG_WARN("Unknown indoorEnv '" << env << "', using office defaults.");
    return {3.0, 7.0, ref};
}

// =============================================================================
//  Wi-Fi standard / MCS string helpers
// =============================================================================

static WifiStandard
ParseStandard(const std::string& s)
{
    if (s == "11a")  return WIFI_STANDARD_80211a;
    if (s == "11b")  return WIFI_STANDARD_80211b;
    if (s == "11g")  return WIFI_STANDARD_80211g;
    if (s == "11n")  return WIFI_STANDARD_80211n;
    if (s == "11ac") return WIFI_STANDARD_80211ac;
    if (s == "11ax") return WIFI_STANDARD_80211ax;
    if (s == "11be") return WIFI_STANDARD_80211be;
    NS_FATAL_ERROR("Unknown Wi-Fi standard '" << s
                   << "'. Valid: 11a 11b 11g 11n 11ac 11ax 11be");
}

// Returns the PHY mode string used by ConstantRateWifiManager
static std::string
MakeMcsString(const std::string& std, int mcs)
{
    if (std == "11be") return "EhtMcs"  + std::to_string(mcs);
    if (std == "11ax") return "HeMcs"   + std::to_string(mcs);
    if (std == "11ac") return "VhtMcs"  + std::to_string(mcs);
    if (std == "11n")  return "HtMcs"   + std::to_string(mcs);
    // 11a/g/b: OFDM rate, mcs 0-7 maps to 6/9/12/18/24/36/48/54 Mbps
    const char* ofdm[] = {
        "OfdmRate6Mbps",  "OfdmRate9Mbps",  "OfdmRate12Mbps", "OfdmRate18Mbps",
        "OfdmRate24Mbps", "OfdmRate36Mbps", "OfdmRate48Mbps", "OfdmRate54Mbps"
    };
    return ofdm[std::max(0, std::min(mcs, 7))];
}

static std::string
MakeBandStr(double freq)
{
    if (freq < 3.0) return "BAND_2_4GHZ";
    if (freq > 5.5) return "BAND_6GHZ";
    return "BAND_5GHZ";
}

// =============================================================================
//  Per-flow result record
// =============================================================================

struct FlowResult
{
    uint32_t    run;
    uint32_t    apIdx;
    uint32_t    staIdx;
    std::string dir;           ///< "dl" or "ul"
    double      throughputMbps;
    double      meanDelayMs;
    double      jitterMs;
    uint64_t    txPkts;
    uint64_t    rxPkts;
    double      lossPct;
    double      distM;         ///< STA-to-AP distance (m)
};

// =============================================================================
//  Single simulation run
// =============================================================================

static std::vector<FlowResult>
RunSimulation(const SimConfig& cfg, uint32_t runIdx)
{
    RngSeedManager::SetSeed(cfg.seed);
    RngSeedManager::SetRun(runIdx);

    const uint32_t nAps      = cfg.nApsRow * cfg.nApsCol;
    const uint32_t totalStas = nAps * cfg.nStasPerAp;

    // -------------------------------------------------------------------------
    //  Propagation parameters (channels are created per-BSS below)
    //  Each AP gets its own MultiModelSpectrumChannel so concurrent
    //  transmissions from different APs never trigger PHY-level collisions.
    // -------------------------------------------------------------------------
    IndoorParams ip = GetIndoorParams(cfg.indoorEnv, cfg.frequency);
    double ple   = (cfg.pathLossExp    > 0.0) ? cfg.pathLossExp    : ip.ple;
    double sigma = (cfg.shadowingSigma > 0.0) ? cfg.shadowingSigma : ip.sigma;

    // -------------------------------------------------------------------------
    //  Global MAC/PHY defaults
    // -------------------------------------------------------------------------
    Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold",
                       UintegerValue(cfg.rtsThreshold));

    if (!cfg.enableAmpdu)
    {
        Config::SetDefault("ns3::WifiMac::BE_MaxAmpduSize", UintegerValue(0));
        Config::SetDefault("ns3::WifiMac::BK_MaxAmpduSize", UintegerValue(0));
        Config::SetDefault("ns3::WifiMac::VI_MaxAmpduSize", UintegerValue(0));
        Config::SetDefault("ns3::WifiMac::VO_MaxAmpduSize", UintegerValue(0));
    }
    if (!cfg.enableAmsdu)
    {
        Config::SetDefault("ns3::WifiMac::BE_MaxAmsduSize", UintegerValue(0));
        Config::SetDefault("ns3::WifiMac::BK_MaxAmsduSize", UintegerValue(0));
        Config::SetDefault("ns3::WifiMac::VI_MaxAmsduSize", UintegerValue(0));
        Config::SetDefault("ns3::WifiMac::VO_MaxAmsduSize", UintegerValue(0));
    }

    // Short GI for 11n/ac must be set before Install via global default
    if (cfg.standard == "11n" || cfg.standard == "11ac")
        Config::SetDefault("ns3::HtConfiguration::ShortGuardIntervalSupported",
                           BooleanValue(cfg.guardInterval == 400));

    // -------------------------------------------------------------------------
    //  Nodes
    // -------------------------------------------------------------------------
    NodeContainer apNodes;
    apNodes.Create(nAps);
    NodeContainer staNodes;
    staNodes.Create(totalStas);

    // -------------------------------------------------------------------------
    //  Positions
    // -------------------------------------------------------------------------
    // APs on a regular grid
    auto apPosAlloc = CreateObject<ListPositionAllocator>();
    std::vector<Vector> apPos(nAps);
    for (uint32_t r = 0; r < cfg.nApsRow; ++r)
        for (uint32_t c = 0; c < cfg.nApsCol; ++c)
        {
            uint32_t idx = r * cfg.nApsCol + c;
            apPos[idx]   = Vector(c * cfg.apDistX, r * cfg.apDistY, 0.0);
            apPosAlloc->Add(apPos[idx]);
        }

    MobilityHelper mob;
    mob.SetPositionAllocator(apPosAlloc);
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(apNodes);

    // STAs: uniformly random within cellRadius of their assigned AP
    // Uniform-in-disk: r = R * sqrt(U[0,1]), theta = U[0, 2*pi]
    auto staPosAlloc = CreateObject<ListPositionAllocator>();
    std::vector<Vector> staPos(totalStas);

    auto rhoRv = CreateObject<UniformRandomVariable>();
    rhoRv->SetAttribute("Min", DoubleValue(0.0));
    rhoRv->SetAttribute("Max", DoubleValue(1.0));
    auto thetaRv = CreateObject<UniformRandomVariable>();
    thetaRv->SetAttribute("Min", DoubleValue(0.0));
    thetaRv->SetAttribute("Max", DoubleValue(2.0 * M_PI));

    for (uint32_t a = 0; a < nAps; ++a)
        for (uint32_t s = 0; s < cfg.nStasPerAp; ++s)
        {
            double r     = cfg.cellRadius * std::sqrt(rhoRv->GetValue());
            double theta = thetaRv->GetValue();
            uint32_t g   = a * cfg.nStasPerAp + s;
            staPos[g]    = Vector(apPos[a].x + r * std::cos(theta),
                                  apPos[a].y + r * std::sin(theta), 0.0);
            staPosAlloc->Add(staPos[g]);
        }

    mob.SetPositionAllocator(staPosAlloc);
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(staNodes);

    // -------------------------------------------------------------------------
    //  Wi-Fi helper — rate manager
    // -------------------------------------------------------------------------
    WifiHelper wifi;
    wifi.SetStandard(ParseStandard(cfg.standard));

    if (cfg.rateManager == "ideal")
    {
        wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    }
    else if (cfg.rateManager == "constant")
    {
        std::string mode = MakeMcsString(cfg.standard, cfg.mcs);
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                     "DataMode",    StringValue(mode),
                                     "ControlMode", StringValue(mode));
    }
    else if (cfg.rateManager == "thompson")
    {
        wifi.SetRemoteStationManager("ns3::ThompsonSamplingWifiManger");
    }
    else if (cfg.rateManager == "minstrel")
    {
        bool legacy = (cfg.standard == "11a" ||
                       cfg.standard == "11b" ||
                       cfg.standard == "11g");
        wifi.SetRemoteStationManager(legacy ? "ns3::MinstrelWifiManager"
                                            : "ns3::MinstrelHtWifiManager");
    }
    else
    {
        NS_FATAL_ERROR("Unknown Rate Selection");
    }

    // Channel settings string: {channelNum, widthMHz, BAND, primary20index}
    std::ostringstream chStr;
    chStr << "{" << cfg.channelNumber << ", " << cfg.channelWidth
          << ", " << MakeBandStr(cfg.frequency) << ", 0}";

    // -------------------------------------------------------------------------
    //  Install Wi-Fi devices per AP BSS
    //  Each BSS gets its own spectrum channel (per-AP isolation) so concurrent
    //  transmissions across BSSes do not cause PHY-level assertion failures.
    //  Each AP gets a unique SSID; WifiStaticSetupHelper handles association.
    // -------------------------------------------------------------------------
    WifiMacHelper mac;
    std::vector<NetDeviceContainer> apDevs(nAps);
    std::vector<NetDeviceContainer> staDevs(nAps);

    for (uint32_t a = 0; a < nAps; ++a)
    {
        // Per-BSS channel with identical propagation parameters
        auto bssChannel = CreateObject<MultiModelSpectrumChannel>();

        auto logDist = CreateObject<LogDistancePropagationLossModel>();
        logDist->SetAttribute("Exponent",      DoubleValue(ple));
        logDist->SetAttribute("ReferenceLoss", DoubleValue(ip.refLoss));
        if (sigma > 0.0)
        {
            auto shadowLoss = CreateObject<RandomPropagationLossModel>();
            auto rv         = CreateObject<NormalRandomVariable>();
            rv->SetAttribute("Mean",     DoubleValue(0.0));
            rv->SetAttribute("Variance", DoubleValue(sigma * sigma));
            shadowLoss->SetAttribute("Variable", PointerValue(rv));
            logDist->SetNext(shadowLoss);
        }
        bssChannel->AddPropagationLossModel(logDist);
        bssChannel->SetPropagationDelayModel(
            CreateObject<ConstantSpeedPropagationDelayModel>());

        SpectrumWifiPhyHelper phy;
        phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
        phy.SetChannel(bssChannel);
        phy.Set("ChannelSettings",  StringValue(chStr.str()));
        phy.Set("TxPowerStart",     DoubleValue(cfg.txPower));
        phy.Set("TxPowerEnd",       DoubleValue(cfg.txPower));
        phy.Set("RxSensitivity",    DoubleValue(cfg.rxSensitivity));

        Ssid ssid("ns3-ap-" + std::to_string(a));

        NodeContainer bssStas;
        for (uint32_t s = 0; s < cfg.nStasPerAp; ++s)
            bssStas.Add(staNodes.Get(a * cfg.nStasPerAp + s));

        mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
        staDevs[a] = wifi.Install(phy, mac, bssStas);

        mac.SetType("ns3::ApWifiMac",
                    "Ssid",               SsidValue(ssid),
                    "EnableBeaconJitter", BooleanValue(false),
                    "BeaconGeneration",   BooleanValue(false));
        apDevs[a] = wifi.Install(phy, mac, apNodes.Get(a));

        auto apDev = DynamicCast<WifiNetDevice>(apDevs[a].Get(0));
        WifiStaticSetupHelper::SetStaticAssociation(apDev, staDevs[a]);
        WifiStaticSetupHelper::SetStaticBlockAck(apDev, staDevs[a], {0});

        if (cfg.enablePcap)
        {
            std::string prefix = cfg.outputPrefix
                               + "-run" + std::to_string(runIdx)
                               + "-ap"  + std::to_string(a);
            phy.EnablePcap(prefix, apDevs[a]);
        }
    }

    // Guard interval for 11ax/11be: set after Install via HeConfiguration attribute path
    if (cfg.standard == "11ax" || cfg.standard == "11be")
        Config::Set(
            "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
            TimeValue(NanoSeconds(cfg.guardInterval)));

    // -------------------------------------------------------------------------
    //  Internet stack + IP addressing
    //  Each BSS gets its own /24 subnet: 10.<apIdx>.0.0/24
    //  AP = .1, STAs = .2, .3, ...
    // -------------------------------------------------------------------------
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNodes);

    std::vector<Ipv4InterfaceContainer> apIfaces(nAps);
    std::vector<Ipv4InterfaceContainer> staIfaces(nAps);

    Ipv4AddressHelper addr;
    addr.SetBase("10.0.0.0", "255.255.0.0");

    for (uint32_t a = 0; a < nAps; ++a)
    {
        NetDeviceContainer bssDevices;
        bssDevices.Add(apDevs[a]);
        bssDevices.Add(staDevs[a]);        

        Ipv4InterfaceContainer bssIfaces = addr.Assign(bssDevices);

        apIfaces[a].Add(bssIfaces.Get(0));
        for (uint32_t s = 0; s < cfg.nStasPerAp; s++)
        {
            staIfaces[a].Add(bssIfaces.Get(1+s));
        }
    }

    NeighborCacheHelper nbCache;
    nbCache.PopulateNeighborCache();
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // -------------------------------------------------------------------------
    //  Warmup phase: one ping per AP-STA pair, staggered by 5 ms each
    //
    //  One ICMP exchange is enough to exercise the pre-configured BA session
    //  and let Minstrel record its first rate sample before traffic starts.
    //  Warmup window = nStasPerAp * 5 ms + 200 ms reply buffer, min 0.5 s.
    //  ICMP flows are invisible to FlowMonitor results (port-based filter).
    // -------------------------------------------------------------------------
    // Warmup window: wide enough for all pings to complete even if some start
    // near the end. Each ping needs ~100 ms max for request + reply.
    const double warmupTime = std::max(0.5,
                                       nAps * cfg.nStasPerAp * 0.002 + 0.3);

    // Random jitter spreads ping starts uniformly over [0, warmupTime - 0.15s]
    // so no two pings fire at the exact same time, letting CSMA/CA work.
    auto jitterRv = CreateObject<UniformRandomVariable>();
    jitterRv->SetAttribute("Min", DoubleValue(0.0));
    jitterRv->SetAttribute("Max", DoubleValue(std::max(0.01, warmupTime - 0.15)));

    for (uint32_t a = 0; a < nAps; ++a)
    {
        Ptr<Node> apNode = apNodes.Get(a);
        for (uint32_t s = 0; s < cfg.nStasPerAp; ++s)
        {
            Ipv4Address staAddr = staIfaces[a].GetAddress(s);

            PingHelper ping(staAddr);
            ping.SetAttribute("Count",       UintegerValue(1));
            ping.SetAttribute("Size",        UintegerValue(56));
            ping.SetAttribute("VerboseMode", EnumValue(Ping::VerboseMode::SILENT));

            auto pingApp = ping.Install(apNode);
            pingApp.Start(Seconds(jitterRv->GetValue()));
            pingApp.Stop(Seconds(warmupTime));
        }
    }

    // -------------------------------------------------------------------------
    //  Traffic installation
    //
    //  Port scheme (avoids collisions, max 256 STAs/AP, 32 APs):
    //    DL sink port = 5000 + apIdx*256 + staIdx
    //    UL sink port = 6000 + apIdx*256 + staIdx
    // -------------------------------------------------------------------------
    if (cfg.traffic == "tcp")
        Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(cfg.packetSize));

    const std::string sockFact = (cfg.traffic == "tcp")
                                 ? "ns3::TcpSocketFactory"
                                 : "ns3::UdpSocketFactory";
    const DataRate offeredRate(static_cast<uint64_t>(cfg.dataRate * 1e6));

    const Time appStart = Seconds(warmupTime);
    const Time appStop  = Seconds(warmupTime + cfg.simTime);

    const uint16_t DL_BASE = 5000;
    const uint16_t UL_BASE = 6000;

    // Lambda: install one OnOff->PacketSink flow; start time passed per-AP
    auto installFlow = [&](Ptr<Node> srcNode, Ptr<Node> dstNode,
                           Ipv4Address dstAddr, uint16_t port, Time start)
    {
        PacketSinkHelper sink(sockFact, InetSocketAddress(Ipv4Address::GetAny(), port));
        auto sinkApp = sink.Install(dstNode);
        sinkApp.Start(Seconds(0.0));
        sinkApp.Stop(appStop + Seconds(0.2));

        OnOffHelper onoff(sockFact, InetSocketAddress(dstAddr, port));
        onoff.SetAttribute("OnTime",
                           StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute("OffTime",
                           StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        onoff.SetAttribute("PacketSize", UintegerValue(cfg.packetSize));
        onoff.SetAttribute("DataRate",   DataRateValue(offeredRate));
        auto srcApp = onoff.Install(srcNode);
        srcApp.Start(start);
        srcApp.Stop(appStop);
    };

    for (uint32_t a = 0; a < nAps; ++a)
    {
        Ipv4Address apAddr = apIfaces[a].GetAddress(0);
        // 2 ms per-AP offset so no two APs begin their first frame simultaneously.
        const Time apStart = appStart + MilliSeconds(a * 2);

        for (uint32_t s = 0; s < cfg.nStasPerAp; ++s)
        {
            Ptr<Node>    apNode  = apNodes.Get(a);
            Ptr<Node>    staNode = staNodes.Get(a * cfg.nStasPerAp + s);
            Ipv4Address  staAddr = staIfaces[a].GetAddress(s);
            uint16_t     portDl  = DL_BASE + a * 256 + s;
            uint16_t     portUl  = UL_BASE + a * 256 + s;

            if (cfg.direction == "downlink" || cfg.direction == "both")
                installFlow(apNode, staNode, staAddr, portDl, apStart);
            if (cfg.direction == "uplink" || cfg.direction == "both")
                installFlow(staNode, apNode, apAddr, portUl, apStart);
        }
    }

    // -------------------------------------------------------------------------
    //  FlowMonitor + run
    // -------------------------------------------------------------------------
    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor>  fm = fmHelper.InstallAll();

    Simulator::Stop(Seconds(warmupTime + cfg.simTime + 0.2));
    Simulator::Run();

    // -------------------------------------------------------------------------
    //  Collect per-flow statistics
    // -------------------------------------------------------------------------
    fm->CheckForLostPackets();
    auto classifier = DynamicCast<Ipv4FlowClassifier>(fmHelper.GetClassifier());
    auto stats      = fm->GetFlowStats();

    std::vector<FlowResult> results;
    for (const auto& entry : stats)
    {
        auto     t  = classifier->FindFlow(entry.first);
        auto&    fs = entry.second;
        uint16_t dp = t.destinationPort;

        uint32_t    a;
        uint32_t    s;
        std::string dir;

        if (dp >= DL_BASE && dp < static_cast<uint16_t>(DL_BASE + nAps * 256))
        {
            uint16_t off = dp - DL_BASE;
            a = off / 256;
            s = off % 256;
            dir = "dl";
        }
        else if (dp >= UL_BASE && dp < static_cast<uint16_t>(UL_BASE + nAps * 256))
        {
            uint16_t off = dp - UL_BASE;
            a = off / 256;
            s = off % 256;
            dir = "ul";
        }
        else
        {
            continue; // TCP ACK or other control flow
        }

        if (a >= nAps || s >= cfg.nStasPerAp)
            continue;

        uint32_t g  = a * cfg.nStasPerAp + s;
        double   dx = staPos[g].x - apPos[a].x;
        double   dy = staPos[g].y - apPos[a].y;

        // Throughput over the configured sim time (consistent denominator across flows)
        double tput   = (cfg.simTime > 0.0) ? (fs.rxBytes * 8.0 / 1e6 / cfg.simTime) : 0.0;
        double delay  = (fs.rxPackets > 0)
                        ? (fs.delaySum.GetSeconds() * 1e3 / fs.rxPackets) : 0.0;
        double jitter = (fs.rxPackets > 1)
                        ? (fs.jitterSum.GetSeconds() * 1e3 / (fs.rxPackets - 1)) : 0.0;
        double loss   = (fs.txPackets > 0)
                        ? (100.0 * (fs.txPackets - fs.rxPackets) / fs.txPackets) : 0.0;

        results.push_back({runIdx, a, s, dir,
                           tput, delay, jitter,
                           fs.txPackets, fs.rxPackets, loss,
                           std::sqrt(dx * dx + dy * dy)});
    }

    // FlowMonitor XML dump for this run
    fm->SerializeToXmlFile(cfg.outputPrefix + "-run" + std::to_string(runIdx) + ".xml",
                           true, true);

    Simulator::Destroy();
    return results;
}

// =============================================================================
//  main
// =============================================================================

int
main(int argc, char* argv[])
{
    SimConfig cfg;

    CommandLine cmd(__FILE__);

    // Topology
    cmd.AddValue("nApsRow",       "AP grid rows",                                    cfg.nApsRow);
    cmd.AddValue("nApsCol",       "AP grid columns",                                 cfg.nApsCol);
    cmd.AddValue("apDistX",       "AP spacing on X axis (m)",                        cfg.apDistX);
    cmd.AddValue("apDistY",       "AP spacing on Y axis (m)",                        cfg.apDistY);
    cmd.AddValue("nStasPerAp",    "STAs per AP (max 256)",                           cfg.nStasPerAp);
    cmd.AddValue("cellRadius",    "Max random STA distance from AP (m)",             cfg.cellRadius);
    // Standard / PHY
    cmd.AddValue("standard",      "Wi-Fi standard: 11a/11n/11ac/11ax/11be",          cfg.standard);
    cmd.AddValue("frequency",     "Band (GHz): 2.4, 5, or 6",                        cfg.frequency);
    cmd.AddValue("channelWidth",  "Channel width (MHz): 20/40/80/160/320",           cfg.channelWidth);
    cmd.AddValue("channelNumber", "Channel number (0=auto)",                         cfg.channelNumber);
    cmd.AddValue("txPower",       "TX power (dBm)",                                  cfg.txPower);
    cmd.AddValue("rxSensitivity", "RX sensitivity (dBm)",                            cfg.rxSensitivity);
    cmd.AddValue("guardInterval", "Guard interval (ns): 400/800/1600/3200",          cfg.guardInterval);
    // Rate control
    cmd.AddValue("rateManager",   "Rate manager: minstrel / ideal / constant",       cfg.rateManager);
    cmd.AddValue("mcs",           "MCS index for constant rate manager",             cfg.mcs);
    // MAC
    cmd.AddValue("rtsThreshold",  "RTS/CTS threshold (bytes): 0=always, 65535=off", cfg.rtsThreshold);
    cmd.AddValue("enableAmpdu",   "Enable A-MPDU aggregation",                       cfg.enableAmpdu);
    cmd.AddValue("enableAmsdu",   "Enable A-MSDU aggregation",                       cfg.enableAmsdu);
    // Propagation
    cmd.AddValue("indoorEnv",     "Env preset: office/stadium/library/hospital/plant", cfg.indoorEnv);
    cmd.AddValue("pathLossExp",   "Path-loss exponent (0 = use indoorEnv preset)",   cfg.pathLossExp);
    cmd.AddValue("shadowingSigma","Shadowing std-dev (dB; 0 = use indoorEnv preset)", cfg.shadowingSigma);
    // Traffic
    cmd.AddValue("traffic",       "Traffic type: udp / tcp",                         cfg.traffic);
    cmd.AddValue("direction",     "Flow direction: uplink / downlink / both",        cfg.direction);
    cmd.AddValue("dataRate",      "Offered rate per STA (Mbps)",                     cfg.dataRate);
    cmd.AddValue("packetSize",    "Packet payload size (bytes)",                     cfg.packetSize);
    // Simulation control
    cmd.AddValue("simTime",       "Data-transfer duration (s)",                      cfg.simTime);
    cmd.AddValue("nRuns",         "Number of independent runs (different seeds)",    cfg.nRuns);
    cmd.AddValue("seed",          "Base RNG seed; run k uses seed+k",                cfg.seed);
    cmd.AddValue("enablePcap",    "Enable PCAP capture on AP interfaces",            cfg.enablePcap);
    cmd.AddValue("outputPrefix",  "Prefix for all output files",                     cfg.outputPrefix);
    cmd.Parse(argc, argv);

    // ---- Validate ----
    NS_ABORT_MSG_IF(cfg.nApsRow == 0 || cfg.nApsCol == 0,
                    "nApsRow and nApsCol must be >= 1");
    NS_ABORT_MSG_IF(cfg.nStasPerAp == 0 || cfg.nStasPerAp > 256,
                    "nStasPerAp must be 1..256");
    NS_ABORT_MSG_IF(cfg.nApsRow * cfg.nApsCol > 32,
                    "Grid too large (max 32 APs to keep port space valid)");
    NS_ABORT_MSG_IF(cfg.traffic != "udp" && cfg.traffic != "tcp",
                    "traffic must be 'udp' or 'tcp'");
    NS_ABORT_MSG_IF(cfg.direction != "uplink" && cfg.direction != "downlink" &&
                    cfg.direction != "both",
                    "direction must be 'uplink', 'downlink', or 'both'");
    NS_ABORT_MSG_IF(cfg.rateManager != "minstrel" && cfg.rateManager != "ideal" &&
                    cfg.rateManager != "constant",
                    "rateManager must be 'minstrel', 'ideal', or 'constant'");

    // ---- Print run parameters ----
    uint32_t nAps = cfg.nApsRow * cfg.nApsCol;
    std::cout << "\n=== Wi-Fi Enterprise Simulation ===\n"
              << "  Standard    : " << cfg.standard << "  (" << cfg.frequency << " GHz, "
              << cfg.channelWidth << " MHz, GI " << cfg.guardInterval << " ns)\n"
              << "  Topology    : " << cfg.nApsRow << "x" << cfg.nApsCol << " grid, "
              << nAps << " APs, " << cfg.nStasPerAp << " STAs/AP = "
              << nAps * cfg.nStasPerAp << " total STAs\n"
              << "  AP spacing  : " << cfg.apDistX << " x " << cfg.apDistY << " m\n"
              << "  Cell radius : " << cfg.cellRadius << " m\n"
              << "  Environment : " << cfg.indoorEnv << "\n"
              << "  Rate manager: " << cfg.rateManager
              << (cfg.rateManager == "constant" ? " (MCS " + std::to_string(cfg.mcs) + ")" : "")
              << "\n"
              << "  Traffic     : " << cfg.traffic << ", " << cfg.direction
              << ", " << cfg.dataRate << " Mbps/STA, pkt=" << cfg.packetSize << " B\n"
              << "  Runs        : " << cfg.nRuns << " x " << cfg.simTime << " s\n"
              << "  Output      : " << cfg.outputPrefix << "_results.csv\n\n";

    // ---- Open CSV ----
    std::string csvPath = cfg.outputPrefix + "_results.csv";
    std::ofstream csv(csvPath);
    NS_ABORT_MSG_IF(!csv.is_open(), "Cannot open output CSV: " << csvPath);
    csv << "run,ap_id,sta_id,direction,throughput_mbps,mean_delay_ms,"
           "jitter_ms,tx_pkts,rx_pkts,loss_pct,distance_m\n";

    // ---- Multi-run loop ----
    std::vector<FlowResult> all;

    for (uint32_t run = 1; run <= cfg.nRuns; ++run)
    {
        std::cout << "--- Run " << run << " / " << cfg.nRuns << " ---\n";
        auto runRes = RunSimulation(cfg, run);

        for (const auto& r : runRes)
        {
            csv << r.run << "," << r.apIdx << "," << r.staIdx << "," << r.dir << ","
                << std::fixed << std::setprecision(4)
                << r.throughputMbps << "," << r.meanDelayMs << "," << r.jitterMs << ","
                << r.txPkts        << "," << r.rxPkts       << ","
                << r.lossPct       << "," << r.distM        << "\n";
        }
        all.insert(all.end(), runRes.begin(), runRes.end());

        // Per-run stdout summary
        if (!runRes.empty())
        {
            double st = 0, sd = 0, sl = 0;
            for (const auto& r : runRes) { st += r.throughputMbps; sd += r.meanDelayMs; sl += r.lossPct; }
            double n = static_cast<double>(runRes.size());
            std::cout << "    flows=" << runRes.size()
                      << "  tput=" << std::fixed << std::setprecision(2) << st / n << " Mbps"
                      << "  delay=" << sd / n << " ms"
                      << "  loss="  << sl / n << " %\n";
        }
    }
    csv.close();

    // ---- Aggregate summary ----
    if (!all.empty())
    {
        double st = 0, sd = 0, sl = 0;
        uint64_t stx = 0, srx = 0;
        for (const auto& r : all) { st += r.throughputMbps; sd += r.meanDelayMs; sl += r.lossPct; stx += r.txPkts; srx += r.rxPkts; }
        double n = static_cast<double>(all.size());

        std::cout << "\n=== Aggregate Summary ("
                  << cfg.nRuns << " run(s), " << all.size() << " flow records) ===\n"
                  << std::fixed << std::setprecision(3)
                  << "  Mean throughput / flow : " << st / n  << " Mbps\n"
                  << "  Mean delay             : " << sd / n  << " ms\n"
                  << "  Mean packet loss       : " << sl / n  << " %\n"
                  << "  Total TX packets       : " << stx << "\n"
                  << "  Total RX packets       : " << srx << "\n"
                  << "  Results written to     : " << csvPath << "\n\n";
    }

    return 0;
}
