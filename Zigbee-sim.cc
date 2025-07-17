/**
 *  Copyright (c) 2024 Tokushima University, Japan
 * 
 *  SPDX-License-Identifier: GPL-2.0-only
 * 
 *  This file is a modified version of an original work by:
 *  Alberto Gallegos Ramonet <alramonet@is.tokushima-u.ac.jp>
 *  Original source:
 *  [Zigbee Network Routing in NS-3](https://www.nsnam.org/docs/release/3.44/doxygen/d1/d54/zigbee-nwk-routing_8cc.html) 
 * 
 *  Modifications and extensions by:
 *  Marco Giannatiempo
   
 *  Mesh routing example with data transmission using a simple topology.
 * 
 *  This ns-3 simulation models a Zigbee mesh network with a specific topology consisting of 
 *  one Coordinator (ZC), four Routers (ZR), and five End Devices (ZED).
 * 
 *  1. Initialization: 
 *  It sets up 10 nodes, configures their IEEE 802.15.4 physical and MAC layers with unique extended addresses,
 *  and places them at fixed positions in a 2D space. A wireless channel with propagation loss and delay models is established.
 *  
 *  2. Network Formation & Joining: 
 *  The Coordinator (Node 0) starts the network. 
 *  Then, the Routers (Nodes 1-4) and End Devices (Nodes 5-9) sequentially discover and join the network via association, 
 *  receiving their 16-bit short addresses dynamically from their parent node. Routers subsequently enable their routing capabilities.
 * 
 *  3. Data Transmission:
 *  After the network is established, a designated source node sends a stream of data packets 
 *  to a designated destination node over a set period. Route discovery is enabled, allowing the network to find paths dynamically if needed.
 *  Monitoring & Analysis:
 *  The simulation uses callbacks to monitor network events like joins and data reception.
 *  It tracks each packet using a custom tag to calculate end-to-end delay.
 * 
 *  4. Results:
 *  At the end of the simulation run, it calculates and prints key performance metrics, including Packet Delivery Ratio (PDR),
 *  average, minimum, and maximum end-to-end latency, and jitter. It also prints the Neighbor and Routing tables of a specified node
 *  and performs a TraceRoute between the source and destination to visualize the path used.
 *
 *
 *  Topology:
 *  
 *  Legend:
 *  O = Coordinator (ZC) / Router (ZR)
 *  X = End Device (ZED)
 *  
 *     Y
 *     ^
 *     |                                                   N5(ZED)
 *  100|                                                     X
 *     |
 *     |                  N2(ZR)                           N1(ZR)    N6(ZED)
 *   50|                    O                                O          X
 *     |
 *     |                                N0(ZC)                         N7(ZED)
 *    0+----------------------------------O-----------------------------X----> X
 *     |
 *     |             N4(ZR)
 *  -50|               O
 *     |             
 *     |    N8(ZED)           N9(ZED)    N3(ZR)
 * -100|      X                 X         O
 *     |             
 *     |             
 * -150|
 *          -150     -100      -50        0         50       100       150   meters
 *
 *  
 *             
 *             
 *         
 */

#include "ns3/netanim-module.h"                   //For network animation
#include "ns3/mobility-module.h"                  //For keeping devices at fixed positions         
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

#include <map>          // To map packet ID -> send time
#include <vector>       // To store latencies
#include <numeric>      // For std::accumulate (sum)
#include <algorithm>    // For std::min_element, std::max_element
#include <cmath>        // For std::sqrt (for jitter)

using namespace ns3;
using namespace ns3::lrwpan;
using namespace ns3::zigbee;

NS_LOG_COMPONENT_DEFINE("ZigbeeRouting"); //Enable logging for the ZigbeeRouting component

ZigbeeStackContainer zigbeeStacks; //A container to hold all the Zigbee stacks in the simulation. This is used to access the stacks later.

//Packet Tracking
uint32_t g_totalPacketsSent = 0;
uint32_t g_totalPacketsReceived = 0;
uint32_t g_packetCounter = 0;           // Unique packet identifier
std::map<uint32_t, Time> g_sendTimeMap; // Map to track packet send times
std::vector<Time> g_delayList;          // List of end-to-end delays for received packets
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
//* Purpose:
//* This function traces the route from a source to a destination in a Zigbee network by querying the routing tables of intermediate nodes.
//* It detects and terminates loops in the route.
//*
//* How it works:
//* 1. It starts at the source node.
//* 2. It uses zstack->GetNwk()->FindRoute(dst, neighbor) to find the next hop toward the destination.
//*    FindRoute returns the next hop's address and a boolean 'neighbor' indicating if the next hop is a direct neighbor.
//* 3. It iterates through the hops, printing the route information for each hop.
//* 4. If a loop is detected (a node is visited 3 times), the trace is aborted.
//* 5. The trace also stops if the destination is reached, the destination is unreachable, or a maximum hop limit is exceeded.
static void
TraceRoute(Mac16Address src, Mac16Address dst)
{
    std::cout << "\nTime " << Simulator::Now().As(Time::S) << " | "
              << "Traceroute from [" << src << "] to destination [" << dst << "]:\n";

    Mac16Address currentHopAddr = src;
    uint32_t hopCount = 1;
    const uint32_t MAX_HOPS = 30; // May need to increase if allowing more repetitions
    // Loop if a node becomes the starting point of an hop this many times
    const int MAX_VISITS_PER_NODE_FOR_LOOP_DETECTION = 3; 

    // Map: Node Address -> Visit Count (as currentHopAddr)
    std::map<Mac16Address, int> visitedNodeCounts; 

    while (currentHopAddr != Mac16Address("FF:FF") && currentHopAddr != dst && hopCount <= MAX_HOPS)
    {
        // Increment the visit count for the current node
        visitedNodeCounts[currentHopAddr]++;

        // Check if we have visited this node (as a starting point) too many times (potential loop)
        if (visitedNodeCounts[currentHopAddr] >= MAX_VISITS_PER_NODE_FOR_LOOP_DETECTION) {
            std::cout << hopCount << ". Node " << currentHopAddr << " has been the start of an hop "
                      << visitedNodeCounts[currentHopAddr] << " times. LOOP DETECTED! Aborting trace.\n";
            break; // Exit the while loop
        }

        Ptr<ZigbeeStack> currentHopStack = nullptr;

        // Find the stack for the current hop address
        for (auto i = zigbeeStacks.Begin(); i != zigbeeStacks.End(); i++)
        {
            Ptr<ZigbeeStack> zstack = *i;
            if (zstack->GetNwk()->GetNetworkAddress() == currentHopAddr)
            {
                currentHopStack = zstack;
                break;
            }
        }

        if (!currentHopStack)
        {
            std::cout << hopCount << ". Node with address [" << currentHopAddr << "] not found in zigbeeStacks. Aborting trace.\n";
            break;
        }

        bool neighbor = false;
        Mac16Address nextHopAddr = currentHopStack->GetNwk()->FindRoute(dst, neighbor);

        std::cout << hopCount << ". Node " << currentHopStack->GetNode()->GetId() << " ["
                  << currentHopStack->GetNwk()->GetNetworkAddress() << " | "
                  << currentHopStack->GetNwk()->GetIeeeAddress() << "]: ";

        if (nextHopAddr == Mac16Address("FF:FF"))
        {
            std::cout << "Destination Unreachable\n";
            currentHopAddr = nextHopAddr; // This will terminate the loop
        }
        else if (nextHopAddr == dst)
        {
            std::cout << "NextHop [" << nextHopAddr << "] (Destination Reached) ";
            if (neighbor)
            {
                std::cout << "(*Neighbor)\n";
            }
            else
            {
                std::cout << "\n";
            }
            currentHopAddr = nextHopAddr; // This will terminate the loop
        }
        else
        {
            std::cout << "NextHop [" << nextHopAddr << "] ";
            if (neighbor)
            {
                std::cout << "(*Neighbor)\n";
            }
            else
            {
                std::cout << "\n";
            }
            currentHopAddr = nextHopAddr; // Move to the next hop
        }
        hopCount++;
    }

    // Check if the loop terminated due to MAX_HOPS, but not due to visit count loop detection
    if (hopCount > MAX_HOPS && 
        currentHopAddr != dst && 
        (visitedNodeCounts.empty() || visitedNodeCounts[currentHopAddr] < MAX_VISITS_PER_NODE_FOR_LOOP_DETECTION))
    {
        std::cout << "Traceroute stopped: Exceeded maximum hop count (" << MAX_HOPS << "). Possible very long path.\n";
    }
    std::cout << "\n";
}
//* Wrapper function to call TraceRoute at the scheduled time
static void ScheduleTraceRouteWrapper(Ptr<ZigbeeStack> srcStack, Ptr<ZigbeeStack> dstStack)
{
    // Safety check on pointers
    if (!srcStack || !dstStack) {
        NS_LOG_ERROR("ScheduleTraceRouteWrapper: Received invalid stack pointer.");
        return;
    }

    // Get network addresses AT THE TIME OF EXECUTION
    Mac16Address srcAddr = srcStack->GetNwk()->GetNetworkAddress();
    Mac16Address dstAddr = dstStack->GetNwk()->GetNetworkAddress();

    // Check if addresses are valid (different from FF:FF)
    if (srcAddr == Mac16Address("FF:FF") || dstAddr == Mac16Address("FF:FF")) {
         NS_LOG_WARN("ScheduleTraceRouteWrapper: Source Address [" << srcAddr << "] or Destination [" << dstAddr
                   << "] not valid (FF:FF) at the time of execution T=" << Simulator::Now().As(Time::S) << ". TraceRoute canceled.");
         std::cout << "WARN: TraceRoute canceled at T=" << Simulator::Now().As(Time::S)
                   << "s - Source Address [" << srcAddr << "] or Destination [" << dstAddr << "] not valid (FF:FF).\n";
         return; // Do not call TraceRoute if addresses are not valid
    }

    // Log and execute TraceRoute
    NS_LOG_INFO("Executing TraceRoute from " << srcAddr << " to " << dstAddr << " at T=" << Simulator::Now().As(Time::S));
    std::cout << "INFO: Executing TraceRoute from " << srcAddr << " to " << dstAddr << " (Scheduled for T=" << Simulator::Now().As(Time::S) << "s)\n";
    TraceRoute(srcAddr, dstAddr); // Call the original TraceRoute function with the currently retrieved addresses
}

//* NwkDataIndication Function
//Purpose: This is a callback function that is invoked when a Zigbee node receives a data packet.
//What it does:
//Prints a message to the console indicating that a packet has been received, the receiving node's ID, and the packet size.
static void 
 NwkDataIndication(Ptr<ZigbeeStack> stack, NldeDataIndicationParams params, Ptr<Packet> p)
{
    PacketIdTag tag;
    if (p->PeekPacketTag(tag)) // Check if the packet has our tag
    {
        uint32_t packetId = tag.GetPacketId();
        if (packetId > 0) // Ensure the ID is valid
        {
            auto it = g_sendTimeMap.find(packetId); // Search for the send time in the map
            if (it != g_sendTimeMap.end()) // Found?
            {
                Time sendTime = it->second;          // Recorded send time
                Time currentTime = Simulator::Now(); // Current reception time
                Time delay = currentTime - sendTime; // Calculate latency

                g_delayList.push_back(delay);        // Add latency to the list
                g_totalPacketsReceived++;            // Increment *valid* received packets
                g_sendTimeMap.erase(it);             // Remove the entry from the map (packet handled)

                // More detailed log on reception
                NS_LOG_INFO("Node " << stack->GetNode()->GetId() << " | NwkDataIndication: Received Packet ID: "
                            << packetId << " | Size: " << p->GetSize() << " | Delay: " << delay.GetSeconds() << " s");
                std::cout << Simulator::Now().As(Time::S) << " Node " << stack->GetNode()->GetId() << " | "
                          << "NwkDataIndication: Received Packet ID: " << packetId << " | Delay: " << delay.GetSeconds() << " s\n";
            }
            else
            {
                // Packet received but ID not found in the map (could happen if the packet arrives after a long time or there's an error)
                 NS_LOG_WARN("Node " << stack->GetNode()->GetId() << " | NwkDataIndication: Received Packet ID: " << packetId << " but no send time found!");
                 std::cout << Simulator::Now().As(Time::S) << " Node " << stack->GetNode()->GetId() << " | "
                          << "NwkDataIndication: Received Packet ID: " << packetId << " NO SEND TIME!\n";
            }
        } else {
             NS_LOG_WARN("Node " << stack->GetNode()->GetId() << " | NwkDataIndication: Received packet with invalid ID (0) in tag.");
             std::cout << Simulator::Now().As(Time::S) << " Node " << stack->GetNode()->GetId() << " | "
                          << "NwkDataIndication: Received packet with invalid ID tag.\n";
        }
    }
    else
    {
         NS_LOG_WARN("Node " << stack->GetNode()->GetId() << " | NwkDataIndication: Received packet without PacketIdTag.");
          std::cout << Simulator::Now().As(Time::S) << " Node " << stack->GetNode()->GetId() << " | "
                          << "NwkDataIndication: Received packet NO TAG.\n";
    }
}

//* NwkNetworkFormationConfirm Function
//Purpose: This is a callback function that is invoked when the network formation process (by the coordinator) is confirmed.
//What it does:
//Prints the status of the network formation (e.g., success or failure).
static void
NwkNetworkFormationConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkFormationConfirmParams params)
{
    std::cout << "\nNlmeNetworkFormationConfirmStatus = " << params.m_status << "\n";
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
        // Set device type based on node ID (Nodes 0-4 are Routers, 5-9 are End Devices)
        if (stack->GetNode()->GetId() >= 1 && stack->GetNode()->GetId() <= 4) // Nodes 1 to 4 are routers
        {
            NS_LOG_INFO("Node " << stack->GetNode()->GetId() << " joining as ROUTER");
            capaInfo.SetDeviceType(ROUTER);
        } 
        else if (stack->GetNode()->GetId() >= 5 && stack->GetNode()->GetId() <= 9) // Nodes 5 to 9 are end devices
        {
            NS_LOG_INFO("Node " << stack->GetNode()->GetId() << " joining as END DEVICE");
            capaInfo.SetDeviceType(ENDDEVICE);
        }
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

        // Check if the node is NOT an End Device before starting the router
        if (stack->GetNode()->GetId() >= 1 && stack->GetNode()->GetId() <= 4) // Execute only if NOT an End Device
        {
            NS_LOG_INFO("Node " << stack->GetNode()->GetId() << " starting as ROUTER");
            // Original: Start the device as a router
            NlmeStartRouterRequestParams startRouterParams; 
            Simulator::ScheduleNow(&ZigbeeNwk::NlmeStartRouterRequest,
                                   stack->GetNwk(),
                                   startRouterParams);
        }
        else
        {
             NS_LOG_INFO("Node " << stack->GetNode()->GetId() << " (EndDevice) does NOT start router functionality.");
        }
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
static void
SendData(Ptr<ZigbeeStack> stackSrc, Ptr<ZigbeeStack> stackDst)
{
    // Send data from a device with stackSrc to device with stackDst.

    // We do not know what network address will be assigned after the JOIN procedure
    // but we can request the network address from stackDst (the destination device) when
    // we intend to send data. If a route do not exist, we will search for a route
    // before transmitting data (Mesh routing).

    // --- Packet Sent ---
    NS_LOG_INFO("Node " << stackSrc->GetNode()->GetId() << " sending data to Node " << stackDst->GetNode()->GetId()); // Log send
    g_totalPacketsSent++;
    g_packetCounter++; //Increment to get a unique ID

    Ptr<Packet> p = Create<Packet>(5); // Create a 5-byte packet

    // --- Add Packet Tag --- 
    PacketIdTag tag;
    tag.SetPacketId(g_packetCounter); // Set the unique ID in the tag
    p->AddPacketTag(tag); // Add the tag to the packet

    // --- Record Send Time ---
    g_sendTimeMap[g_packetCounter] = Simulator::Now(); // Associate the packet ID with the current time

    NldeDataRequestParams dataReqParams;
    dataReqParams.m_dstAddrMode = UCST_BCST; 
    dataReqParams.m_dstAddr = stackDst->GetNwk()->GetNetworkAddress();
    dataReqParams.m_nsduHandle = 1; // Puoi usare g_packetCounter se vuoi un handle univoco
    dataReqParams.m_discoverRoute = ENABLE_ROUTE_DISCOVERY; // Enable route discovery if no route is known

    Simulator::ScheduleNow(&ZigbeeNwk::NldeDataRequest, stackSrc->GetNwk(), dataReqParams, p);
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
    nodes.Create(10);
    //Create a container to hold the 10 nodes.

//MAC Configuration
    LrWpanHelper lrWpanHelper; //Creates a helper for LR-WPAN (802.15.4) devices
    
    //Installs LR-WPAN devices on the nodes
    NetDeviceContainer lrwpanDevices = lrWpanHelper.Install(nodes);
    Ptr<LrWpanNetDevice> dev0 = lrwpanDevices.Get(0)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev1 = lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev2 = lrwpanDevices.Get(2)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev3 = lrwpanDevices.Get(3)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev4 = lrwpanDevices.Get(4)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev5 = lrwpanDevices.Get(5)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev6 = lrwpanDevices.Get(6)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev7 = lrwpanDevices.Get(7)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev8 = lrwpanDevices.Get(8)->GetObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev9 = lrwpanDevices.Get(9)->GetObject<LrWpanNetDevice>();

    //Each device must ALWAYS have unique 64-bit IEEE Address (Extended address) assigned.
    //Network address (short address) are assigned by the the JOIN mechanism
    dev0->GetMac()->SetExtendedAddress("00:00:00:00:00:00:CA:FE");
    dev1->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:01");
    dev2->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:02");
    dev3->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:03");
    dev4->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:04");
    dev5->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:05");
    dev6->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:06");
    dev7->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:07");
    dev8->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:08");
    dev9->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:09");

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
    dev5->SetChannel(channel);
    dev6->SetChannel(channel);
    dev7->SetChannel(channel);
    dev8->SetChannel(channel);
    dev9->SetChannel(channel);

//NWK Configuration
    ZigbeeHelper zigbee; //Creates a helper for Zigbee devices
    
    //Installs the Zigbee stack on all devices
    ZigbeeStackContainer zigbeeStackContainer = zigbee.Install(lrwpanDevices);
    Ptr<ZigbeeStack> zstack0 = zigbeeStackContainer.Get(0)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack1 = zigbeeStackContainer.Get(1)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack2 = zigbeeStackContainer.Get(2)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack3 = zigbeeStackContainer.Get(3)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack4 = zigbeeStackContainer.Get(4)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack5 = zigbeeStackContainer.Get(5)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack6 = zigbeeStackContainer.Get(6)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack7 = zigbeeStackContainer.Get(7)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack8 = zigbeeStackContainer.Get(8)->GetObject<ZigbeeStack>();
    Ptr<ZigbeeStack> zstack9 = zigbeeStackContainer.Get(9)->GetObject<ZigbeeStack>();

    // Add the stacks to a container to later on print routes.
    zigbeeStacks.Add(zstack0);
    zigbeeStacks.Add(zstack1);
    zigbeeStacks.Add(zstack2);
    zigbeeStacks.Add(zstack3);
    zigbeeStacks.Add(zstack4);
    zigbeeStacks.Add(zstack5);
    zigbeeStacks.Add(zstack6);
    zigbeeStacks.Add(zstack7);
    zigbeeStacks.Add(zstack8);
    zigbeeStacks.Add(zstack9);

    // Assign streams to the zigbee stacks to obtain
    // reprodusable results from random events occurring inside the stack.
    zstack0->GetNwk()->AssignStreams(0);
    zstack1->GetNwk()->AssignStreams(10);
    zstack2->GetNwk()->AssignStreams(20);
    zstack3->GetNwk()->AssignStreams(30);
    zstack4->GetNwk()->AssignStreams(40);
    zstack5->GetNwk()->AssignStreams(50); 
    zstack6->GetNwk()->AssignStreams(60); 
    zstack7->GetNwk()->AssignStreams(70); 
    zstack8->GetNwk()->AssignStreams(80); 
    zstack9->GetNwk()->AssignStreams(90); 
    
//Mobility configuration
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel"); // Set the type of model to install
    mobility.Install(nodes); // Install the model on ALL nodes in the container

    //get the installed mobility model for each node and set its specific position
    Ptr<ConstantPositionMobilityModel> mob0 = nodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
    mob0->SetPosition(Vector(0, 0, 0));

    Ptr<ConstantPositionMobilityModel> mob1 = nodes.Get(1)->GetObject<ConstantPositionMobilityModel>();
    mob1->SetPosition(Vector(100, 50, 0));

    Ptr<ConstantPositionMobilityModel> mob2 = nodes.Get(2)->GetObject<ConstantPositionMobilityModel>();
    mob2->SetPosition(Vector(-75, 50, 0));

    Ptr<ConstantPositionMobilityModel> mob3 = nodes.Get(3)->GetObject<ConstantPositionMobilityModel>();
    mob3->SetPosition(Vector(0, -100, 0));

    Ptr<ConstantPositionMobilityModel> mob4 = nodes.Get(4)->GetObject<ConstantPositionMobilityModel>();
    mob4->SetPosition(Vector(-100, -50, 0));

    Ptr<ConstantPositionMobilityModel> mob5 = nodes.Get(5)->GetObject<ConstantPositionMobilityModel>();
    mob5->SetPosition(Vector(100, 100, 0));

    Ptr<ConstantPositionMobilityModel> mob6 = nodes.Get(6)->GetObject<ConstantPositionMobilityModel>();
    mob6->SetPosition(Vector(150, 50, 0));

    Ptr<ConstantPositionMobilityModel> mob7 = nodes.Get(7)->GetObject<ConstantPositionMobilityModel>();
    mob7->SetPosition(Vector(150, 0, 0));

    Ptr<ConstantPositionMobilityModel> mob8 = nodes.Get(8)->GetObject<ConstantPositionMobilityModel>();
    mob8->SetPosition(Vector(-150, -100, 0));

    Ptr<ConstantPositionMobilityModel> mob9 = nodes.Get(9)->GetObject<ConstantPositionMobilityModel>();
    mob9->SetPosition(Vector(-50, -100, 0));

    //link the node's mobility model to the PHY layer of the LR-WPAN device
    dev0->GetPhy()->SetMobility(mob0);
    dev1->GetPhy()->SetMobility(mob1);
    dev2->GetPhy()->SetMobility(mob2);
    dev3->GetPhy()->SetMobility(mob3);
    dev4->GetPhy()->SetMobility(mob4);
    dev5->GetPhy()->SetMobility(mob5);
    dev6->GetPhy()->SetMobility(mob6);
    dev7->GetPhy()->SetMobility(mob7);
    dev8->GetPhy()->SetMobility(mob8);
    dev9->GetPhy()->SetMobility(mob9);



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
    zstack5->GetNwk()->SetNldeDataIndicationCallback(
        MakeBoundCallback(&NwkDataIndication, zstack5));
    zstack6->GetNwk()->SetNldeDataIndicationCallback(
        MakeBoundCallback(&NwkDataIndication, zstack6));
    zstack7->GetNwk()->SetNldeDataIndicationCallback(
        MakeBoundCallback(&NwkDataIndication, zstack7));
    zstack8->GetNwk()->SetNldeDataIndicationCallback(
        MakeBoundCallback(&NwkDataIndication, zstack8));
    zstack9->GetNwk()->SetNldeDataIndicationCallback(
        MakeBoundCallback(&NwkDataIndication, zstack9));

    zstack1->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack1));
    zstack2->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack2));
    zstack3->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack3));
    zstack4->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack4));
    zstack5->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack5));
    zstack6->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack6));
    zstack7->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack7));
    zstack8->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack8));
    zstack9->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(
        MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack9));

    zstack1->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack1));
    zstack2->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack2));
    zstack3->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack3));
    zstack4->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack4));
    zstack5->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack5));
    zstack6->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack6));
    zstack7->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack7));
    zstack8->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack8));
    zstack9->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack9));

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
    netDiscParams3.m_scanChannelList.channelPageCount = 1;
    netDiscParams3.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
    netDiscParams3.m_scanDuration = 2;
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
    // End devices
    NlmeNetworkDiscoveryRequestParams netDiscParams5;
    netDiscParams5.m_scanChannelList.channelPageCount = 1;
    netDiscParams5.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
    netDiscParams5.m_scanDuration = 2;
    Simulator::ScheduleWithContext(zstack5->GetNode()->GetId(),
                                   Seconds(7), // Continue staggering
                                   &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                   zstack5->GetNwk(),
                                   netDiscParams5);

    NlmeNetworkDiscoveryRequestParams netDiscParams6;
    netDiscParams6.m_scanChannelList.channelPageCount = 1;
    netDiscParams6.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
    netDiscParams6.m_scanDuration = 2;
    Simulator::ScheduleWithContext(zstack6->GetNode()->GetId(),
                                   Seconds(8),
                                   &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                   zstack6->GetNwk(),
                                   netDiscParams6);

    NlmeNetworkDiscoveryRequestParams netDiscParams7;
    netDiscParams7.m_scanChannelList.channelPageCount = 1;
    netDiscParams7.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
    netDiscParams7.m_scanDuration = 2;
    Simulator::ScheduleWithContext(zstack7->GetNode()->GetId(),
                                   Seconds(9),
                                   &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                   zstack7->GetNwk(),
                                   netDiscParams7);

    NlmeNetworkDiscoveryRequestParams netDiscParams8;
    netDiscParams8.m_scanChannelList.channelPageCount = 1;
    netDiscParams8.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
    netDiscParams8.m_scanDuration = 2;
    Simulator::ScheduleWithContext(zstack8->GetNode()->GetId(),
                                   Seconds(10),
                                   &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                   zstack8->GetNwk(),
                                   netDiscParams8);

    NlmeNetworkDiscoveryRequestParams netDiscParams9;
    netDiscParams9.m_scanChannelList.channelPageCount = 1;
    netDiscParams9.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
    netDiscParams9.m_scanDuration = 2;
    Simulator::ScheduleWithContext(zstack9->GetNode()->GetId(),
                                   Seconds(11),
                                   &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                   zstack9->GetNwk(),
                                   netDiscParams9);

// ---------------------------------------------------------------------
//todo --- Transmission and Inspection Configuration ---
// ---------------------------------------------------------------------
    // Modify these lines to easily change the involved nodes
    // Note: zstackN corresponds to the Zigbee stack of Node N in the simulation (e.g., zstack0 -> Node 0)
    Ptr<ZigbeeStack> sourceStack      = zstack2; // SOURCE NODE: Change here (e.g., zstack5)
    Ptr<ZigbeeStack> destinationStack = zstack8; // DESTINATION NODE: Change here (e.g., zstack3)
    Ptr<ZigbeeStack> inspectStack     = zstack4; // NODE TO INSPECT: Change here (e.g., destinationStack or zstack2)

    // Log/info print to confirm the chosen configuration
    NS_LOG_INFO("--- Simulation Configuration ---");
    NS_LOG_INFO("Source Node:      Node " << sourceStack->GetNode()->GetId() << " (" << sourceStack->GetNwk()->GetIeeeAddress() << ")");
    NS_LOG_INFO("Destination Node: Node " << destinationStack->GetNode()->GetId() << " (" << destinationStack->GetNwk()->GetIeeeAddress() << ")");
    NS_LOG_INFO("Inspecting Node:  Node " << inspectStack->GetNode()->GetId() << " (" << inspectStack->GetNwk()->GetIeeeAddress() << ")");
    std::cout << "\n--------------------------------\n";
    std::cout << "--- Simulation Configuration ---\n";
    std::cout << "Source Node:      Node " << sourceStack->GetNode()->GetId() << "\n";
    std::cout << "Destination Node: Node " << destinationStack->GetNode()->GetId() << "\n";
    std::cout << "Inspecting Node:  Node " << inspectStack->GetNode()->GetId() << "\n";
    std::cout << "--------------------------------\n";
// ---------------------------------------------------------------------

//Data Transmission
    double startTime = 12.0;    // Start sending packets
    double interval = 0.5;      // Interval between packets (seconds)
    int numPacketsToSend = 200; // Total number of packets to send
        
    NS_LOG_INFO("Scheduling " << numPacketsToSend << " packets from Node " << sourceStack->GetNode()->GetId()
                << " to Node " << destinationStack->GetNode()->GetId() << " starting at " << startTime << "s");

    for (int i = 0; i < numPacketsToSend; ++i) {
        // Schedule sending packets at regular intervals from the source node to the destination node
        Simulator::Schedule(Seconds(startTime + i * interval), &SendData, sourceStack, destinationStack);
    }

// ---------------------------------------------------------------------
// --- Calculate and Print Final Results ---
// ---------------------------------------------------------------------
    // MAKE SURE THIS TIME IS AFTER THE LAST PACKET + POSSIBLE MAXIMUM LATENCY
    // Example: if you send 200 packets every 0.5s starting from 12s, the last send is at 12 + 199*0.5 = 111.5s
    // Give it more time to arrive, e.g., 120s or more.
    double calculationTime = startTime + (numPacketsToSend * interval) + 10.0; // Added safety time
    Simulator::Schedule(Seconds(calculationTime), []() {
    std::cout << "\n-----------------------------------------\n";
    std::cout << "---      Simulation Results           ---\n";
    std::cout << "-----------------------------------------\n";
    std::cout << "Total Packets Sent:     " << g_totalPacketsSent << "\n";
    std::cout << "Total Packets Received: " << g_totalPacketsReceived << "\n";

    // Calculate Average PDR
    double avgPdr = 0.0;
    if (g_totalPacketsSent > 0)
    {
        avgPdr = static_cast<double>(g_totalPacketsReceived) / g_totalPacketsSent;
        std::cout << "Packet Delivery Ratio (PDR): " << avgPdr * 100.0 << " %\n";
    }
    else
    {
        std::cout << "PDR: N/A (No packets sent)\n";
    }
    
    // Calculate latency metrics
    std::cout << "--- Latency Metrics (End-to-End) ---\n";
    if (!g_delayList.empty())
    {
        Time totalDelay = Seconds(0);
        Time minDelay = g_delayList[0];
        Time maxDelay = g_delayList[0];

        // Calculate sum, min, max
        for (const auto& delay : g_delayList) {
            totalDelay += delay;
            if (delay < minDelay) minDelay = delay;
            if (delay > maxDelay) maxDelay = delay;
        }

        // Calculate average
        Time avgDelay = totalDelay / g_delayList.size();

        // Calculate Jitter (as standard deviation of latency in seconds)
        double sumSquaredDiff = 0.0;
        double avgDelaySec = avgDelay.GetSeconds();
        for (const auto& delay : g_delayList) {
            double delaySec = delay.GetSeconds();
            sumSquaredDiff += (delaySec - avgDelaySec) * (delaySec - avgDelaySec);
        }
        double variance = sumSquaredDiff / g_delayList.size();
        double jitter = std::sqrt(variance);

        std::cout << "Average Delay: " << avgDelay.GetSeconds() << " s\n";
        std::cout << "Minimum Delay: " << minDelay.GetSeconds() << " s\n";
        std::cout << "Maximum Delay: " << maxDelay.GetSeconds() << " s\n";
        std::cout << "Jitter (StdDev): " << jitter << " s\n";
        std::cout << "(Based on " << g_delayList.size() << " successfully received packets)\n";
    }
    else
    {
        std::cout << "Average Delay: N/A\n";
        std::cout << "Minimum Delay: N/A\n";
        std::cout << "Maximum Delay: N/A\n";
        std::cout << "Jitter (StdDev): N/A\n";
        std::cout << "(No packets received successfully to calculate latency)\n";
    }
    std::cout << "-------------------------------------------\n";
    });
    
    //Print TABLES
    // Choose the node to inspect
    Ptr<ZigbeeStack> nodeToInspect = inspectStack;
    // Choose a time shortly before the final results
    double tablePrintTime = calculationTime - 0.5; // Print half a second before results
    // Make sure the time is not too early if calculationTime is very close to the last send
    if (tablePrintTime < startTime + (numPacketsToSend * interval)) {
        tablePrintTime = calculationTime; // Otherwise, print at the same time as results
    }
    // Log/info print to confirm the chosen configuration before simulation
    NS_LOG_INFO("Scheduling final tables print for Node " << nodeToInspect->GetNode()->GetId()
                << " at T=" << tablePrintTime << " s");
    std::cout << "INFO: Scheduling final tables print for Node " << nodeToInspect->GetNode()->GetId()
              << " at T=" << tablePrintTime << " s\n";
    std::cout << "----------------------------------------------------------\n";

    // Create the output stream wrapper for std::cout (necessary for Print* functions)
    Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(&std::cout);

    // ---Schedule printing a line before printing the tables---
    Simulator::Schedule(Seconds(tablePrintTime), [nodeToInspect]() { 
        std::cout << "----  END TRANSMISSION  ----\n"; 
        std::cout << "\n-----------------------------------------\n"; 
        std::cout << "---         Tables for Node " << nodeToInspect->GetNode()->GetId() 
                  << "         ---\n"; 
        std::cout << "-----------------------------------------\n"; 
    });  

    // Print the NEIGHBOR TABLE at the end of all packet transmissions
    Simulator::Schedule(Seconds(tablePrintTime),
                        &ZigbeeNwk::PrintNeighborTable,
                        nodeToInspect->GetNwk(),
                        stream);
    // Print the ROUTING TABLE at the end of all packet transmissions
    Simulator::Schedule(Seconds(tablePrintTime + 0.01), 
                        &ZigbeeNwk::PrintRoutingTable,
                        nodeToInspect->GetNwk(),
                        stream);
    //!Print the ROUTE DISCOVERY TABLE immediately after sending the first packet
    Simulator::Schedule(Seconds(startTime + 0.72), 
                        &ZigbeeNwk::PrintRouteDiscoveryTable,
                        nodeToInspect->GetNwk(),
                        stream);

    // Schedule TraceRoute via the Wrapper function
    Simulator::Schedule(Seconds(tablePrintTime + 0.03), // Keep the same time or adjust if needed
                       &ScheduleTraceRouteWrapper,      // Call the NEW wrapper function
                       sourceStack,                     // Pass the POINTER to the source stack
                       destinationStack);                // Pass the POINTER to the destination stack

// --------------------------------------------------------------------
// --- Animation & Tracing ---
// ---------------------------------------------------------------------
/*//Animation
    AnimationInterface anim("Zigbee-sim.xml");  // file XML
    anim.UpdateNodeDescription(nodes.Get(0), "ZC-0"); // Coordinator
    anim.UpdateNodeDescription(nodes.Get(1), "ZR-1"); // Router 1
    anim.UpdateNodeDescription(nodes.Get(2), "ZR-2"); // Router 2
    anim.UpdateNodeDescription(nodes.Get(3), "ZR-3"); // Router 3
    anim.UpdateNodeDescription(nodes.Get(4), "ZR-4"); // Router 4
    anim.UpdateNodeDescription(nodes.Get(5), "ZED-5"); // End Device 5
    anim.UpdateNodeDescription(nodes.Get(6), "ZED-6"); // End Device 6
    anim.UpdateNodeDescription(nodes.Get(7), "ZED-7"); // End Device 7
    anim.UpdateNodeDescription(nodes.Get(8), "ZED-8"); // End Device 8
    anim.UpdateNodeDescription(nodes.Get(9), "ZED-9"); // End Device 9
 //ASCII tracing
    AsciiTraceHelper ascii;
    lrWpanHelper.EnableAsciiAll(ascii.CreateFileStream("Zigbee-sim.tr"));
 //PCAP tracing
    lrWpanHelper.EnablePcapAll("Zigbee-sim");
    */
//---------------------------------------------------------------------

// --- Simulation Control ---
    double stopTime = calculationTime + 5.0; // Ensure simulation ends AFTER calculation
    Simulator::Stop(Seconds(stopTime));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
