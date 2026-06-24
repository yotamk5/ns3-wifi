/*
 * wifi-e2e-demo.cc
 *
 * Measure Wi-Fi e2e delay per packet, per AP, per STA, per RngRun.
 * Definition: time from MAC queue enqueue on sender (AP)
 *             to MacRx delivery on receiver (STA).
 *
 * Topology: nAps APs, nStas STAs per AP, one UDP flow per AP->STA pair
 *
 * Run: ./ns3 run "wifi-e2e-demo --nAps=2 --nStas=3 --RngRun=1"
 * Output: wifi_e2e_run<N>.csv
 */

#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
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

using namespace ns3;

// ── shared state ─────────────────────────────────────────────────────────────

struct E2eCtx
{
    uint32_t run;
    std::unordered_map<uint64_t, Time> enqueueTime; // UID -> enqueue time on sender
    std::shared_ptr<std::ofstream>     csv;
};

// ── callbacks ─────────────────────────────────────────────────────────────────

// Fires when a Wi-Fi frame enters the MAC TX queue on the AP (sender)
// apIdx identifies which AP this queue belongs to
static void
MacEnqueue(std::shared_ptr<E2eCtx> ctx, uint32_t apIdx, Ptr<const WifiMpdu> mpdu)
{
    uint64_t uid = mpdu->GetPacket()->GetUid();
    ctx->enqueueTime[uid] = Simulator::Now();
}

// Fires when a Wi-Fi frame is delivered from MAC to IP on the STA (receiver)
// apIdx + staIdx identify which STA received it
static void
MacRx(std::shared_ptr<E2eCtx> ctx, uint32_t apIdx, uint32_t staIdx, Ptr<const Packet> pkt)
{
    uint64_t uid = pkt->GetUid();
    auto it = ctx->enqueueTime.find(uid);
    if (it == ctx->enqueueTime.end())
    {
        return; // management frames, frames from other flows, etc.
    }

    double e2eMs = (Simulator::Now() - it->second).GetSeconds() * 1000.0;
    ctx->enqueueTime.erase(it);

    *ctx->csv << ctx->run << ',' << apIdx << ',' << staIdx << ','
              << Simulator::Now().GetSeconds() << ',' << uid << ',' << e2eMs << '\n';
    ctx->csv->flush();
}

// ── main ──────────────────────────────────────────────────────────────────────

int
main(int argc, char* argv[])
{
    uint32_t nAps    = 2;
    uint32_t nStas   = 3;
    uint32_t rngRun  = 1;
    double   simTime = 5.0;

    CommandLine cmd;
    cmd.AddValue("nAps",    "Number of APs",          nAps);
    cmd.AddValue("nStas",   "STAs per AP",             nStas);
    cmd.AddValue("RngRun",  "RNG run index (seed)",    rngRun);
    cmd.AddValue("simTime", "Simulation time (s)",     simTime);
    cmd.Parse(argc, argv);

    RngSeedManager::SetRun(rngRun);

    // --- shared channel ---
    auto channel = CreateObject<MultiModelSpectrumChannel>();
    auto loss    = CreateObject<LogDistancePropagationLossModel>();
    auto delay   = CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->AddPropagationLossModel(loss);
    channel->SetPropagationDelayModel(delay);

    // --- PHY / MAC helpers ---
    SpectrumWifiPhyHelper phy;
    phy.SetChannel(channel);
    phy.Set("ChannelSettings", StringValue("{6, 20, BAND_2_4GHZ, 0}"));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

    InternetStackHelper inet;
    Ipv4AddressHelper   addr;
    addr.SetBase("10.0.0.0", "255.255.255.0");

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // --- open CSV ---
    auto ctx = std::make_shared<E2eCtx>();
    ctx->run = rngRun;
    ctx->csv = std::make_shared<std::ofstream>();

    std::ostringstream fname;
    fname << "wifi_e2e_run" << rngRun << ".csv";
    ctx->csv->open(fname.str());
    *ctx->csv << "run,ap,sta,time_s,uid,wifi_e2e_ms\n";

    // --- build topology per AP ---
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
        auto apIf = addr.Assign(apDev);
        addr.NewNetwork();

        // Connect Enqueue on the AP — only sender side
        // Path scoped to this AP's node index so we don't double-count
        std::ostringstream enqueuePath;
        enqueuePath << "/NodeList/" << apNode.Get(0)->GetId()
                    << "/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/Queue/Enqueue";
        Config::ConnectWithoutContext(
            enqueuePath.str(),
            MakeBoundCallback(&MacEnqueue, ctx, ap));

        // STA nodes for this AP
        NodeContainer staNodes;
        staNodes.Create(nStas);
        mob.Install(staNodes);

        for (uint32_t s = 0; s < nStas; ++s)
        {
            staNodes.Get(s)->GetObject<MobilityModel>()->SetPosition(
                {static_cast<double>(ap) * 20.0 + static_cast<double>(s + 1) * 3.0, 5, 0});

            WifiMacHelper macSta;
            macSta.SetType("ns3::StaWifiMac",
                           "Ssid", SsidValue(ssid),
                           "ActiveProbing", BooleanValue(false));
            auto staDev = wifi.Install(phy, macSta, NodeContainer(staNodes.Get(s)));
            inet.Install(NodeContainer(staNodes.Get(s)));
            auto staIf = addr.Assign(staDev);
            addr.NewNetwork();

            // Connect MacRx only on this STA — fixes the duplicate problem
            // because we no longer use wildcard * for NodeList
            std::ostringstream macrxPath;
            macrxPath << "/NodeList/" << staNodes.Get(s)->GetId()
                      << "/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx";
            Config::ConnectWithoutContext(
                macrxPath.str(),
                MakeBoundCallback(&MacRx, ctx, ap, s));

            // UDP flow: AP -> STA
            uint16_t port = 9000 + ap * 100 + s;
            PacketSinkHelper sink("ns3::UdpSocketFactory",
                                  InetSocketAddress(Ipv4Address::GetAny(), port));
            sink.Install(NodeContainer(staNodes.Get(s))).Start(Seconds(0));

            OnOffHelper onoff("ns3::UdpSocketFactory",
                              InetSocketAddress(staIf.GetAddress(0), port));
            onoff.SetConstantRate(DataRate("512Kbps"), 1024);
            auto app = onoff.Install(apNode);
            app.Start(Seconds(1.0));
            app.Stop(Seconds(simTime));
        }
    }

    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();
    Simulator::Destroy();

    ctx->csv->close();
    std::cout << "Done. Results in " << fname.str() << "\n";
    return 0;
}
