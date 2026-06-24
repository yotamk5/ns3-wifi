/*
 * wifi-e2e-demo.cc
 *
 * Minimal example: measure Wi-Fi e2e delay per packet.
 * Definition: time from MAC queue enqueue on sender
 *             to MacRx delivery on receiver.
 *
 * Topology: one AP, one STA, one UDP flow AP->STA
 *
 * Run: ./ns3 run wifi-e2e-demo
 * Output: wifi_e2e.csv
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

using namespace ns3;

// ── shared state ────────────────────────────────────────────────────────────

static std::unordered_map<uint64_t, Time> g_enqueueTime; // UID -> enqueue time
static std::ofstream                       g_csv;

// ── callbacks ────────────────────────────────────────────────────────────────

// Fires when a packet enters the MAC queue on the SENDER
static void
MacEnqueue(Ptr<const WifiMpdu> mpdu)
{
    uint64_t uid = mpdu->GetPacket()->GetUid();
    g_enqueueTime[uid] = Simulator::Now();
}

// Fires when a packet is delivered from Wi-Fi MAC to IP on the RECEIVER
static void
MacRx(Ptr<const Packet> pkt)
{
    uint64_t uid = pkt->GetUid();
    auto it = g_enqueueTime.find(uid);
    if (it == g_enqueueTime.end())
    {
        return; // control frames, beacons, etc. — no entry expected
    }

    double e2eMs = (Simulator::Now() - it->second).GetSeconds() * 1000.0;
    g_enqueueTime.erase(it);

    g_csv << Simulator::Now().GetSeconds() << ',' << uid << ',' << e2eMs << '\n';
    g_csv.flush();
}

// ── main ─────────────────────────────────────────────────────────────────────

int
main(int argc, char* argv[])
{
    double simTime = 5.0; // seconds
    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.Parse(argc, argv);

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

    // --- MAC ---
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

    // --- AP ---
    NodeContainer apNode;
    apNode.Create(1);
    WifiMacHelper macAp;
    Ssid ssid("demo");
    macAp.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    auto apDev = wifi.Install(phy, macAp, apNode);

    // --- STA ---
    NodeContainer staNode;
    staNode.Create(1);
    WifiMacHelper macSta;
    macSta.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid),
                   "ActiveProbing", BooleanValue(false));
    auto staDev = wifi.Install(phy, macSta, staNode);

    // --- mobility ---
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(apNode);
    mob.Install(staNode);
    apNode.Get(0)->GetObject<MobilityModel>()->SetPosition({0, 0, 0});
    staNode.Get(0)->GetObject<MobilityModel>()->SetPosition({10, 0, 0});

    // --- IP ---
    InternetStackHelper inet;
    inet.Install(apNode);
    inet.Install(staNode);
    Ipv4AddressHelper addr;
    addr.SetBase("10.0.0.0", "255.255.255.0");
    auto apIf  = addr.Assign(apDev);
    auto staIf = addr.Assign(staDev);

    // --- traffic: AP -> STA, UDP, 1 Mbps ---
    uint16_t port = 9;
    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address::GetAny(), port));
    sink.Install(staNode).Start(Seconds(0));

    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(staIf.GetAddress(0), port));
    onoff.SetConstantRate(DataRate("1Mbps"), 1024);
    auto apps = onoff.Install(apNode);
    apps.Start(Seconds(1.0));
    apps.Stop(Seconds(simTime));

    // --- open CSV ---
    g_csv.open("wifi_e2e.csv");
    g_csv << "time_s,uid,wifi_e2e_ms\n";

    // --- connect traces AFTER wifi.Install ---
    //
    // "Enqueue" trace: Queue<WifiMpdu>::DoEnqueue fires m_traceEnqueue(item)
    // Path: NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/Queue/Enqueue
    //
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/Queue/Enqueue",
        MakeCallback(&MacEnqueue));

    // "MacRx" trace: WifiMac fires m_rxCallback when packet is delivered to IP
    // Path: NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx
    //
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
        MakeCallback(&MacRx));

    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();
    Simulator::Destroy();

    g_csv.close();
    std::cout << "Done. Results in wifi_e2e.csv\n";
    return 0;
}
