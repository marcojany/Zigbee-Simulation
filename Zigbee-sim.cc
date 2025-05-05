/**
 * Copyright (c) 2024 Tokushima University, Japan
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * This file is a modified version of an original work by:
 *   Alberto Gallegos Ramonet <alramonet@is.tokushima-u.ac.jp>
 *
 * Modifications and extensions by:
 *   Marco Giannatiempo

 * 
 * Mesh routing example with data transmission using a simple topology.
 *
 * This example shows the NWK layer procedure to perform a route request.
 * Prior the route discovery and data transmission, an association-based join is performed.
 * The procedure requires a sequence of primitive calls on a specific order in the indicated
 * devices.
 *
 *
 *  Network Extended PAN id: 0X000000000000CA:FE (based on the PAN coordinator address)
 *
 *  Devices Addresses:
 *
 *  [Coordinator] ZC  (dev0 | Node 0): [00:00:00:00:00:00:CA:FE]  [00:00]
 *  [Router 1]    ZR1 (dev1 | Node 1): [00:00:00:00:00:00:00:01]  [short addr assigned by ZC]
 *  [Router 2]    ZR2 (dev2 | Node 2): [00:00:00:00:00:00:00:02]  [short addr assigned by ZR1]
 *  [Router 3]    ZR3 (dev3 | Node 3): [00:00:00:00:00:00:00:03]  [short addr assigned by ZR2]
 *  [Router 4]    ZR4 (dev4 | Node 4): [00:00:00:00:00:00:00:04]  [short addr assigned by ZR1]
 *
 *  Topology:
 *
 *  ZC--------ZR1------------ZR2----------ZR3
 *             |
 *             |
 *            ZR4
 */

#include "ns3/constant-position-mobility-model.h" //For keeping devices at fixed positions         
#include "ns3/core-module.h"                      //Core ns-3 functionalities      
#include "ns3/log.h"                              //For logging messages      
#include "ns3/lr-wpan-module.h"                   //For the IEEE 802.15.4 (LR-WPAN) standard   
#include "ns3/packet.h"                           //For creating and managing packets
#include "ns3/propagation-delay-model.h"          //For modeling signal propagation delay     
#include "ns3/propagation-loss-model.h"           //For modeling signal propagation loss
#include "ns3/simulator.h"                        //For simulation time management
#include "ns3/single-model-spectrum-channel.h"    //For the wireless channel     
#include "ns3/zigbee-module.h"                    //For the Zigbee stack

#include <iostream>

using namespace ns3;
using namespace ns3::lrwpan;
using namespace ns3::zigbee;

NS_LOG_COMPONENT_DEFINE("ZigbeeRouting"); //Enable logging for the ZigbeeRouting component

ZigbeeStackContainer zigbeeStacks; //A container to hold all the Zigbee stacks in the simulation. This is used to access the stacks later.

//todo Packet Tracking //-NEW-
uint32_t g_totalPacketsSent = 0;
uint32_t g_totalPacketsReceived = 0;
uint32_t g_packetCounter = 0; // Unique packet identifier
//Packet Tag
class PacketIdTag : public Tag
{
public:
    static TypeId GetTypeId(void)
    {
        static TypeId tid = TypeId("PacketIdTag")
                                .SetParent<Tag>()
                                .AddConstructor<PacketIdTag>();
        return tid;
    }
    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return sizeof(uint32_t); }
    void Serialize(TagBuffer i) const override
    {
        i.WriteU32(m_packetId);
    }
    void Deserialize(TagBuffer i) override
    {
        m_packetId = i.ReadU32();
    }
    void Print(std::ostream& os) const override
    {
        os << "PacketId=" << m_packetId;
    }

    void SetPacketId(uint32_t id) { m_packetId = id; }
    uint32_t GetPacketId() const { return m_packetId; }

private:
    uint32_t m_packetId;
};

//* TraceRoute Function
//Purpose: This function is crucial for understanding how routing works. It traces the route from a source (src) to a destination (dst) by querying the routing tables of the intermediate nodes.
//How it works:
//1. It starts at the source node.
//2. It uses zstack->GetNwk()->FindRoute(dst, neighbor) to find the next hop toward the destination.
//3. FindRoute returns the next hop's address and a boolean neighbor indicating if the next hop is a direct neighbor.
//4. It iterates through the hops until it reaches the destination or finds that the destination is unreachable.
//5. It prints the route information to the console.
static void
TraceRoute(Mac16Address src, Mac16Address dst)
{
    std::cout << "\nTime " << Simulator::Now().As(Time::S) << " | "
              << "Traceroute to destination [" << dst << "]:\n";
    Mac16Address target = src;
    uint32_t count = 1;
    while (target != Mac16Address("FF:FF") && target != dst)
    {
        Ptr<ZigbeeStack> zstack;

        for (auto i = zigbeeStacks.Begin(); i != zigbeeStacks.End(); i++)
        {
            zstack = *i;
            if (zstack->GetNwk()->GetNetworkAddress() == target)
            {
                break;
            }
        }

        bool neighbor = false;
        target = zstack->GetNwk()->FindRoute(dst, neighbor);
        if (target == Mac16Address("FF:FF"))
        {
            std::cout << count << ". Node " << zstack->GetNode()->GetId() << " ["
                      << zstack->GetNwk()->GetNetworkAddress() << " | "
                      << zstack->GetNwk()->GetIeeeAddress() << "]: "
                      << " Destination Unreachable\n";
        }
        else
        {
            std::cout << count << ". Node " << zstack->GetNode()->GetId() << " ["
                      << zstack->GetNwk()->GetNetworkAddress() << " | "
                      << zstack->GetNwk()->GetIeeeAddress() << "]: "
                      << "NextHop [" << target << "] ";
            if (neighbor)
            {
                std::cout << "(*Neighbor)\n";
            }
            else
            {
                std::cout << "\n";
            }
            count++;
        }
    }
    std::cout << "\n";
}


//* NwkDataIndication Function
//Purpose: This is a callback function that is invoked when a Zigbee node receives a data packet.
//What it does:
//Prints a message to the console indicating that a packet has been received, the receiving node's ID, and the packet size.
static void                         
NwkDataIndication(Ptr<ZigbeeStack> stack, NldeDataIndicationParams params, Ptr<Packet> p)
{
    // --- Packet Received ---
    PacketIdTag tag;
    if (p->PeekPacketTag(tag))
    {
        // Check if the packet has already been received
        if (tag.GetPacketId() > 0)
        {
            g_totalPacketsReceived++;
            std::cout << Simulator::Now().As(Time::S) << " Node " << stack->GetNode()->GetId() << " | "
                      << "NsdeDataIndication:  Received packet of size " << p->GetSize() << " | Packet ID: " << tag.GetPacketId() << "\n";
        }
    }
    else
    {
        std::cout << Simulator::Now().As(Time::S) << " Node " << stack->GetNode()->GetId() << " | "
                  << "NsdeDataIndication:  Received packet of size " << p->GetSize() << " | Packet ID: UNKNOWN" << "\n";
    }
}

//* NwkNetworkFormationConfirm Function
//Purpose: This is a callback function that is invoked when the network formation process (by the coordinator) is confirmed.
//What it does:
//Prints the status of the network formation (e.g., success or failure).
static void
NwkNetworkFormationConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkFormationConfirmParams params)
{
    std::cout << "NlmeNetworkFormationConfirmStatus = " << params.m_status << "\n";
}


//* NwkNetworkDiscoveryConfirm Function
//Purpose: This is a callback function that is invoked when the network discovery process (by end devices) is confirmed.
//What it does:
//1. Checks if the discovery was successful.
//2. If successful, it prints the details of the discovered networks (Extended PAN ID, channel, PAN ID, stack profile).
//3. It then prepares a NlmeJoinRequestParams to join the discovered network.
//4. It schedules the NlmeJoinRequest to join the network.
static void
NwkNetworkDiscoveryConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkDiscoveryConfirmParams params)
{
    // See Zigbee Specification r22.1.0, 3.6.1.4.1
    // This method implements a simplistic version of the method implemented
    // in a zigbee APL layer. In this layer a candidate Extended PAN Id must
    // be selected and a NLME-JOIN.request must be issued.

    if (params.m_status == NwkStatus::SUCCESS)
    {
        std::cout << " Network discovery confirm Received. Networks found ("
                  << params.m_netDescList.size() << "):\n";

        for (const auto& netDescriptor : params.m_netDescList)
        {
            std::cout << " ExtPanID: 0x" << std::hex << netDescriptor.m_extPanId << "\n"
                      << std::dec << " CH:  " << static_cast<uint32_t>(netDescriptor.m_logCh)
                      << "\n"
                      << std::hex << " Pan ID: 0x" << netDescriptor.m_panId << "\n"
                      << " Stack profile: " << std::dec
                      << static_cast<uint32_t>(netDescriptor.m_stackProfile) << "\n"
                      << "--------------------\n";
        }

        NlmeJoinRequestParams joinParams;

        zigbee::CapabilityInformation capaInfo;
        capaInfo.SetDeviceType(ROUTER);
        capaInfo.SetAllocateAddrOn(true);

        joinParams.m_rejoinNetwork = zigbee::JoiningMethod::ASSOCIATION;
        joinParams.m_capabilityInfo = capaInfo.GetCapability();
        joinParams.m_extendedPanId = params.m_netDescList[0].m_extPanId;

        Simulator::ScheduleNow(&ZigbeeNwk::NlmeJoinRequest, stack->GetNwk(), joinParams);
    }
    else
    {
        NS_ABORT_MSG("Unable to discover networks | status: " << params.m_status);
    }
}


//* NwkJoinConfirm Function
//Purpose: This is a callback function that is invoked when the JOIN procedure is confirmed.
//What it does:
//1. Checks if the JOIN was successful.
//2. If successful, it prints the details of the JOIN (network short address, extended PAN ID).
//3. It then schedules a NlmeStartRouterRequest to start the device as a router
static void
NwkJoinConfirm(Ptr<ZigbeeStack> stack, NlmeJoinConfirmParams params)
{
    if (params.m_status == NwkStatus::SUCCESS)
    {
        std::cout << Simulator::Now().As(Time::S) << " Node " << stack->GetNode()->GetId() << " | "
                  << " The device joined the network SUCCESSFULLY with short address " << std::hex
                  << params.m_networkAddress << " on the Extended PAN Id: " << std::hex
                  << params.m_extendedPanId << "\n"
                  << std::dec;

        // 3 - After dev 1 is associated, it should be started as a router
        //     (i.e. it becomes able to accept request from other devices to join the network)
        NlmeStartRouterRequestParams startRouterParams;
        Simulator::ScheduleNow(&ZigbeeNwk::NlmeStartRouterRequest,
                               stack->GetNwk(),
                               startRouterParams);
    }
    else
    {
        std::cout << " The device FAILED to join the network with status " << params.m_status
                  << "\n";
    }
}


//* NwkRouteDiscoveryConfirm Function
//Purpose: This is a callback function that is invoked when a route discovery process is confirmed.
//What it does:
//Prints the status of the route discovery.
static void
NwkRouteDiscoveryConfirm(Ptr<ZigbeeStack> stack, NlmeRouteDiscoveryConfirmParams params)
{
    std::cout << "NlmeRouteDiscoveryConfirmStatus = " << params.m_status << "\n";
}


//* SendData Function
//Purpose: This function sends a data packet from one Zigbee node (stackSrc) to another (stackDst).
//How it works:
//1. Creates a packet.
//2. Sets the destination address (dataReqParams.m_dstAddr) to the network address of the destination node.
//3. Sets dataReqParams.m_discoverRoute = ENABLE_ROUTE_DISCOVERY; to enable route discovery if a route is not already known.
//4. Schedules the NldeDataRequest to send the packet.
//5. Schedules TraceRoute to print the route used.
//6. Schedules the printing of the neighbor, routing, and route discovery tables.
static void
SendData(Ptr<ZigbeeStack> stackSrc, Ptr<ZigbeeStack> stackDst)
{
    // Send data from a device with stackSrc to device with stackDst.

    // We do not know what network address will be assigned after the JOIN procedure
    // but we can request the network address from stackDst (the destination device) when
    // we intend to send data. If a route do not exist, we will search for a route
    // before transmitting data (Mesh routing).

    // --- Packet Sent ---
    g_totalPacketsSent++;
    g_packetCounter++;

    Ptr<Packet> p = Create<Packet>(5);

    // --- Add Packet Tag ---
    PacketIdTag tag;
    tag.SetPacketId(g_packetCounter);
    p->AddPacketTag(tag);

    NldeDataRequestParams dataReqParams;
    dataReqParams.m_dstAddrMode = UCST_BCST;
    dataReqParams.m_dstAddr = stackDst->GetNwk()->GetNetworkAddress();
    dataReqParams.m_nsduHandle = 1;
    dataReqParams.m_discoverRoute = ENABLE_ROUTE_DISCOVERY;

    Simulator::ScheduleNow(&ZigbeeNwk::NldeDataRequest, stackSrc->GetNwk(), dataReqParams, p);

    // Give a few seconds to allow the creation of the route and
    // then print the route trace and tables from the source
    Simulator::Schedule(Seconds(3),
                        &TraceRoute,
                        stackSrc->GetNwk()->GetNetworkAddress(),
                        stackDst->GetNwk()->GetNetworkAddress());

    Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(&std::cout);
    Simulator::Schedule(Seconds(4), &ZigbeeNwk::PrintNeighborTable, stackSrc->GetNwk(), stream);

    Simulator::Schedule(Seconds(4), &ZigbeeNwk::PrintRoutingTable, stackSrc->GetNwk(), stream);

    Simulator::Schedule(Seconds(4),
                        &ZigbeeNwk::PrintRouteDiscoveryTable,
                        stackSrc->GetNwk(),
                        stream);
}


//* MAIN Function
int
main(int argc, char* argv[])
{
 //Inialization
    LogComponentEnableAll(LogLevel(LOG_PREFIX_TIME | LOG_PREFIX_FUNC | LOG_PREFIX_NODE));
    //Enables logging for all components with time, function, and node prefixes.
    //LogComponentEnable("ZigbeeNwk", LOG_LEVEL_DEBUG);

    RngSeedManager::SetSeed(3);
    RngSeedManager::SetRun(4);
    //Set the seed and run number for the random number generator.

    NodeContainer nodes;
    nodes.Create(5);
    //Create a container to hold the 5 nodes.

 //MAC Configuration
    LrWpanHelper lrWpanHelper; //Creates a helper for LR-WPAN (802.15.4) devices
    
    //Installs LR-WPAN devices on the nodes
    NetDeviceContainer lrwpanDevices = lrWpanHelper.Install(nodes); 
    Ptr<LrWpanNetDevice> dev0 = lrwpanDevices.Get(0)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev1 = lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev2 = lrwpanDevices.Get(2)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev3 = lrwpanDevices.Get(3)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev4 = lrwpanDevices.Get(4)->GetObject<LrWpanNetDevice>();

    //Each device must ALWAYS have unique 64-bit IEEE Address (Extended address) assigned.
    //Network address (short address) are assigned by the the JOIN mechanism
    dev0->GetMac()->SetExtendedAddress("00:00:00:00:00:00:CA:FE");
    dev1->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:01");
    dev2->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:02");
    dev3->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:03");
    dev4->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:04");

    //creates a wireless channel for the devices
    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
    //creates a propagation loss model
    Ptr<LogDistancePropagationLossModel> propModel =
        CreateObject<LogDistancePropagationLossModel>();
    //creates a propagation delay model
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();

    channel->AddPropagationLossModel(propModel);    //Adds the propagation loss model to the channel
    channel->SetPropagationDelayModel(delayModel);  //Sets the propagation delay model for the channel

    //Assigns the channel to each device
    dev0->SetChannel(channel);
    dev1->SetChannel(channel);
    dev2->SetChannel(channel);
    dev3->SetChannel(channel);
    dev4->SetChannel(channel);

 //NWK Configuration
    ZigbeeHelper zigbee; //Creates a helper for Zigbee devices
    
    //Installs the Zigbee stack on all devices
    ZigbeeStackContainer zigbeeStackContainer = zigbee.Install(lrwpanDevices);
    Ptr<ZigbeeStack> zstack0 = zigbeeStackContainer.Get(0)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack1 = zigbeeStackContainer.Get(1)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack2 = zigbeeStackContainer.Get(2)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack3 = zigbeeStackContainer.Get(3)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack4 = zigbeeStackContainer.Get(4)->GetObject<ZigbeeStack>();

    // Add the stacks to a container to later on print routes.
    zigbeeStacks.Add(zstack0);
    zigbeeStacks.Add(zstack1);
    zigbeeStacks.Add(zstack2);
    zigbeeStacks.Add(zstack3);
    zigbeeStacks.Add(zstack4);

    // Assign streams to the zigbee stacks to obtain
    // reprodusable results from random events occurring inside the stack.
    zstack0->GetNwk()->AssignStreams(0);
    zstack1->GetNwk()->AssignStreams(10);
    zstack2->GetNwk()->AssignStreams(20);
    zstack3->GetNwk()->AssignStreams(30);
    zstack4->GetNwk()->AssignStreams(40);

 //Mobility configuration
    // Set the position of the devices amd assign mobility models to each device
    Ptr<ConstantPositionMobilityModel> dev0Mobility = CreateObject<ConstantPositionMobilityModel>();
    dev0Mobility->SetPosition(Vector(0, 0, 0));
    dev0->GetPhy()->SetMobility(dev0Mobility);

    Ptr<ConstantPositionMobilityModel> dev1Mobility = CreateObject<ConstantPositionMobilityModel>();
    dev1Mobility->SetPosition(Vector(90, 0, 0));
    dev1->GetPhy()->SetMobility(dev1Mobility);

    Ptr<ConstantPositionMobilityModel> dev2Mobility = CreateObject<ConstantPositionMobilityModel>();
    dev2Mobility->SetPosition(Vector(170, 0, 0));
    dev2->GetPhy()->SetMobility(dev2Mobility);

    Ptr<ConstantPositionMobilityModel> dev3Mobility = CreateObject<ConstantPositionMobilityModel>();
    dev3Mobility->SetPosition(Vector(250, 0, 0));
    dev3->GetPhy()->SetMobility(dev3Mobility);

    Ptr<ConstantPositionMobilityModel> dev4Mobility = CreateObject<ConstantPositionMobilityModel>();
    dev4Mobility->SetPosition(Vector(90, 50, 0));
    dev4->GetPhy()->SetMobility(dev4Mobility);

 //NWK callbacks hooks
    // These hooks are usually directly connected to the APS layer
    // In this case, there is no APS layer, therefore, we connect the event outputs
    // of all devices directly to our static functions in this example.

    zstack0->GetNwk()->SetNlmeNetworkFormationConfirmCallback(
        MakeBoundCallback(&NwkNetworkFormationConfirm, zstack0));
    zstack0->GetNwk()->SetNlmeRouteDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkRouteDiscoveryConfirm, zstack0));

    zstack0->GetNwk()->SetNldeDataIndicationCallback(
        MakeBoundCallback(&NwkDataIndication, zstack0));
    zstack1->GetNwk()->SetNldeDataIndicationCallback(
        MakeBoundCallback(&NwkDataIndication, zstack1));
    zstack2->GetNwk()->SetNldeDataIndicationCallback(
        MakeBoundCallback(&NwkDataIndication, zstack2));
    zstack3->GetNwk()->SetNldeDataIndicationCallback(
        MakeBoundCallback(&NwkDataIndication, zstack3));
    zstack4->GetNwk()->SetNldeDataIndicationCallback(
        MakeBoundCallback(&NwkDataIndication, zstack4));

    zstack1->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack1));
    zstack2->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack2));
    zstack3->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack3));
    zstack4->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack4));

    zstack1->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack1));
    zstack2->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack2));
    zstack3->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack3));
    zstack4->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack4));

 //Network Formation
    // 1 - Initiate the Zigbee coordinator, start the network
    // ALL_CHANNELS = 0x07FFF800 (Channels to scan [11~26])
    NlmeNetworkFormationRequestParams netFormParams;
    netFormParams.m_scanChannelList.channelPageCount = 1;
    netFormParams.m_scanChannelList.channelsField[0] = ALL_CHANNELS;
    netFormParams.m_scanDuration = 0;
    netFormParams.m_superFrameOrder = 15;
    netFormParams.m_beaconOrder = 15;
    
    //Schedules the network formation request for the coordinator.
    Simulator::ScheduleWithContext(zstack0->GetNode()->GetId(),
                                   Seconds(1),
                                   &ZigbeeNwk::NlmeNetworkFormationRequest,
                                   zstack0->GetNwk(),
                                   netFormParams);

 //Network Discovery and Joining
    // 2- Schedule devices sequentially find and join the network.
    //    After this procedure, each device make a NLME-START-ROUTER.request to become a router

    NlmeNetworkDiscoveryRequestParams netDiscParams;
    netDiscParams.m_scanChannelList.channelPageCount = 1;
    netDiscParams.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
    netDiscParams.m_scanDuration = 2;
    //Schedules the network discovery request for each router, with increasing delays
    Simulator::ScheduleWithContext(zstack1->GetNode()->GetId(),
                                   Seconds(3),
                                   &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                   zstack1->GetNwk(),
                                   netDiscParams);

    NlmeNetworkDiscoveryRequestParams netDiscParams2;
    netDiscParams2.m_scanChannelList.channelPageCount = 1;
    netDiscParams2.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
    netDiscParams2.m_scanDuration = 2;
    Simulator::ScheduleWithContext(zstack2->GetNode()->GetId(),
                                   Seconds(4),
                                   &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                   zstack2->GetNwk(),
                                   netDiscParams2);

    NlmeNetworkDiscoveryRequestParams netDiscParams3;
    netDiscParams2.m_scanChannelList.channelPageCount = 1;
    netDiscParams2.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
    netDiscParams2.m_scanDuration = 2;
    Simulator::ScheduleWithContext(zstack3->GetNode()->GetId(),
                                   Seconds(5),
                                   &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                   zstack3->GetNwk(),
                                   netDiscParams3);

    NlmeNetworkDiscoveryRequestParams netDiscParams4;
    netDiscParams4.m_scanChannelList.channelPageCount = 1;
    netDiscParams4.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
    netDiscParams4.m_scanDuration = 2;
    Simulator::ScheduleWithContext(zstack4->GetNode()->GetId(),
                                   Seconds(6),
                                   &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                   zstack4->GetNwk(),
                                   netDiscParams4);

 //Data Transmission
    // 3- Find Route and Send data from ZR0 to ZR3
    //Simulator::Schedule(Seconds(8), &SendData, zstack0, zstack3);     //OLD
    double startTime = 8.0; // Secondo iniziale
    double interval = 0.5;  // Secondi tra i pacchetti
    int numPacketsToSend = 10; // Numero di pacchetti da inviare
    for (int i = 0; i < numPacketsToSend; ++i) {
        Simulator::Schedule(Seconds(startTime + i * interval), &SendData, zstack0, zstack3);
    }
 // --- PDR Calculation and Output ---
    Simulator::Schedule(Seconds(19), []() {
        std::cout << "\n--- Simulation Results ---\n";
        std::cout << "Total Packets Sent: " << g_totalPacketsSent << "\n";
        std::cout << "Total Packets Received: " << g_totalPacketsReceived << "\n";
        if (g_totalPacketsSent > 0)
        {
            double pdr = (double)g_totalPacketsReceived / g_totalPacketsSent;
            std::cout << "Packet Delivery Ratio (PDR): " << pdr << "\n";
        }
        else
        {
            std::cout << "No packets were sent.\n";
        }
        std::cout << "--- End of Simulation Results ---\n";
    });
 //Simulation Control
    Simulator::Stop(Seconds(20));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
