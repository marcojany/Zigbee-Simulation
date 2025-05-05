/**
 *  Copyright (c) 2024 Tokushima University, Japan
 * 
 *  SPDX-License-Identifier: GPL-2.0-only
 * 
 *  This file is a modified version of an original work by:
 *    Alberto Gallegos Ramonet <alramonet@is.tokushima-u.ac.jp>
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
  
 *  Network Extended PAN id: 0X000000000000CA:FE (based on the PAN coordinator address)
 *
 *  Devices Addresses:
 *
 *  [Coordinator] ZC  (dev0 | Node 0): [00:00:00:00:00:00:CA:FE]  
 *  [Router 1]    ZR1 (dev1 | Node 1): [00:00:00:00:00:00:00:01]
 *  [Router 2]    ZR2 (dev2 | Node 2): [00:00:00:00:00:00:00:02]
 *  [Router 3]    ZR3 (dev3 | Node 3): [00:00:00:00:00:00:00:03]
 *  [Router 4]    ZR4 (dev4 | Node 4): [00:00:00:00:00:00:00:04]
 *  [End Device 5]    ZR5 (dev5 | Node 5): [00:00:00:00:00:00:00:05]
 *  [End Device 6]    ZR6 (dev6 | Node 6): [00:00:00:00:00:00:00:06]
 *  [End Device 7]    ZR7 (dev7 | Node 7): [00:00:00:00:00:00:00:07]
 *  [End Device 8]    ZR8 (dev8 | Node 8): [00:00:00:00:00:00:00:08]
 *  [End Device 9]    ZR9 (dev9 | Node 9): [00:00:00:00:00:00:00:09]
 
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

#include <map>          // Per mappare ID pacchetto -> tempo invio
#include <vector>       // Per memorizzare le latenze
#include <numeric>      // Per std::accumulate (somma)
#include <algorithm>    // Per std::min_element, std::max_element
#include <cmath>        // Per std::sqrt (per jitter)

using namespace ns3;
using namespace ns3::lrwpan;
using namespace ns3::zigbee;

NS_LOG_COMPONENT_DEFINE("ZigbeeRouting"); //Enable logging for the ZigbeeRouting component

ZigbeeStackContainer zigbeeStacks; //A container to hold all the Zigbee stacks in the simulation. This is used to access the stacks later.

//Packet Tracking
uint32_t g_totalPacketsSent = 0;
uint32_t g_totalPacketsReceived = 0;
uint32_t g_packetCounter = 0; // Unique packet identifier
std::map<uint32_t, Time> g_sendTimeMap; // Mappa per tenere traccia del tempo di invio dei pacchetti
std::vector<Time> g_delayList;       // Lista delle latenze end-to-end per i pacchetti ricevuti
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

//* Funzione Wrapper per chiamare TraceRoute al momento schedulato
static void ScheduleTraceRouteWrapper(Ptr<ZigbeeStack> srcStack, Ptr<ZigbeeStack> dstStack)
{
    // Controllo di sicurezza sui puntatori
    if (!srcStack || !dstStack) {
        NS_LOG_ERROR("ScheduleTraceRouteWrapper: Ricevuto puntatore a stack non valido.");
        return;
    }

    // Ottieni gli indirizzi di rete AL MOMENTO DELL'ESECUZIONE
    Mac16Address srcAddr = srcStack->GetNwk()->GetNetworkAddress();
    Mac16Address dstAddr = dstStack->GetNwk()->GetNetworkAddress();

    // Controllo se gli indirizzi sono validi (diversi da FF:FF)
    if (srcAddr == Mac16Address("FF:FF") || dstAddr == Mac16Address("FF:FF")) {
         NS_LOG_WARN("ScheduleTraceRouteWrapper: Indirizzo Sorgente [" << srcAddr << "] o Destinazione [" << dstAddr
                   << "] non valido (FF:FF) al momento dell'esecuzione T=" << Simulator::Now().As(Time::S) << ". TraceRoute annullato.");
         std::cout << "WARN: TraceRoute annullato a T=" << Simulator::Now().As(Time::S)
                   << "s - Indirizzo Sorgente [" << srcAddr << "] o Destinazione [" << dstAddr << "] non valido (FF:FF).\n";
         return; // Non chiamare TraceRoute se gli indirizzi non sono validi
    }

    // Log ed esecuzione di TraceRoute
    NS_LOG_INFO("Executing TraceRoute from " << srcAddr << " to " << dstAddr << " at T=" << Simulator::Now().As(Time::S));
    std::cout << "INFO: Eseguendo TraceRoute da " << srcAddr << " a " << dstAddr << " (Schedulato per T=" << Simulator::Now().As(Time::S) << "s)\n";
    TraceRoute(srcAddr, dstAddr); // Chiama la funzione TraceRoute originale con gli indirizzi recuperati ora
}

//* NwkDataIndication Function
//Purpose: This is a callback function that is invoked when a Zigbee node receives a data packet.
//What it does:
//Prints a message to the console indicating that a packet has been received, the receiving node's ID, and the packet size.
static void 
 NwkDataIndication(Ptr<ZigbeeStack> stack, NldeDataIndicationParams params, Ptr<Packet> p)
{
    PacketIdTag tag;
    if (p->PeekPacketTag(tag)) // Controlla se il pacchetto ha il nostro tag
    {
        uint32_t packetId = tag.GetPacketId();
        if (packetId > 0) // Assicurati che l'ID sia valido
        {
            auto it = g_sendTimeMap.find(packetId); // Cerca il tempo di invio nella mappa
            if (it != g_sendTimeMap.end()) // Trovato?
            {
                Time sendTime = it->second;       // Tempo di invio registrato
                Time currentTime = Simulator::Now(); // Tempo di ricezione corrente
                Time delay = currentTime - sendTime; // Calcola latenza

                g_delayList.push_back(delay);     // Aggiungi la latenza alla lista
                g_totalPacketsReceived++;         // Incrementa pacchetti ricevuti *validi*
                g_sendTimeMap.erase(it);          // Rimuovi l'entry dalla mappa (pacchetto gestito)

                // Log più dettagliato sulla ricezione
                NS_LOG_INFO("Node " << stack->GetNode()->GetId() << " | NwkDataIndication: Received Packet ID: "
                            << packetId << " | Size: " << p->GetSize() << " | Delay: " << delay.GetSeconds() << " s");
                std::cout << Simulator::Now().As(Time::S) << " Node " << stack->GetNode()->GetId() << " | "
                          << "NwkDataIndication: Received Packet ID: " << packetId << " | Delay: " << delay.GetSeconds() << " s\n";

            }
            else
            {
                // Pacchetto ricevuto ma ID non trovato nella mappa (potrebbe succedere se il pacchetto arriva dopo molto tempo o c'è un errore)
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
        // Set device type based on node ID
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

        // Controlla se il nodo NON è l'End Device prima di avviare il router
        if (stack->GetNode()->GetId() >= 1 && stack->GetNode()->GetId() <= 4) // Esegui solo se NON è End Device
        {
            NS_LOG_INFO("Node " << stack->GetNode()->GetId() << " starting as ROUTER");
            // Originale: Avvia il dispositivo come router
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
    NS_LOG_INFO("Node " << stackSrc->GetNode()->GetId() << " sending data to Node " << stackDst->GetNode()->GetId()); // Log invio 
    g_totalPacketsSent++;
    g_packetCounter++; //Incrementa per ottenere un ID univoco

    Ptr<Packet> p = Create<Packet>(5); // Crea un pacchetto di 5 byte

    // --- Add Packet Tag --- 
    PacketIdTag tag;
    tag.SetPacketId(g_packetCounter); // Imposta l'ID univoco nel tag
    p->AddPacketTag(tag); // Aggiungi il tag al pacchetto

    // --- Registra il Tempo di Invio ---
    g_sendTimeMap[g_packetCounter] = Simulator::Now(); // Associa l'ID del pacchetto al tempo corrente

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
    zstack5->GetNwk()->AssignStreams(50); // <-- Aggiunto
    zstack6->GetNwk()->AssignStreams(60); // <-- Aggiunto
    zstack7->GetNwk()->AssignStreams(70); // <-- Aggiunto
    zstack8->GetNwk()->AssignStreams(80); // <-- Aggiunto
    zstack9->GetNwk()->AssignStreams(90); // <-- Aggiunto
    
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

    // --- Added for End Devices 5 to 9 ---
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
//todo --- Configurazione Trasmissione e Ispezione ---
// ---------------------------------------------------------------------
    // Modifica queste righe per cambiare facilmente i nodi coinvolti
    // Nota: zstackN corrisponde allo stack Zigbee del Node N della simulazione (es. zstack0 -> Node 0)
    Ptr<ZigbeeStack> sourceStack      = zstack4; // NODO SORGENTE: Cambia qui (es. zstack5)
    Ptr<ZigbeeStack> destinationStack = zstack6; // NODO DESTINAZIONE: Cambia qui (es. zstack3)
    Ptr<ZigbeeStack> inspectStack     = zstack1; // NODO DA ISPEZIONARE: Cambia qui (es. destinationStack o zstack2)

    // Stampa di log/info per confermare la configurazione scelta
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
    double startTime = 12.0; // Inizio invio pacchetti
    double interval = 0.5;  // Intervallo tra pacchetti (secondi)
    int numPacketsToSend = 200; // Numero totale di pacchetti da inviare
        
    NS_LOG_INFO("Scheduling " << numPacketsToSend << " packets from Node " << sourceStack->GetNode()->GetId()
                << " to Node " << destinationStack->GetNode()->GetId() << " starting at " << startTime << "s");

    for (int i = 0; i < numPacketsToSend; ++i) {
        // Schedula l'invio di pacchetti a intervalli regolari dal nodo sorgente al nodo di destinazione
        Simulator::Schedule(Seconds(startTime + i * interval), &SendData, sourceStack, destinationStack);
    }

// ---------------------------------------------------------------------
// --- Calcolo e Stampa Risultati Finali ---
// ---------------------------------------------------------------------
    // ASSICURATI CHE QUESTO TEMPO SIA DOPO L'ULTIMO PACCHETTO + POSSIBILE LATENZA MASSIMA
    // Esempio: se invii 200 pacchetti ogni 0.5s a partire da 12s, l'ultimo invio è a 12 + 199*0.5 = 111.5s
    // Dagli ancora tempo per arrivare, es. 120s o più.
    double calculationTime = startTime + (numPacketsToSend * interval) + 10.0; // Tempo di sicurezza aggiunto
    Simulator::Schedule(Seconds(calculationTime), []() {
    std::cout << "\n-----------------------------------------\n";
    std::cout << "---      Simulation Results           ---\n";
    std::cout << "-----------------------------------------\n";
    std::cout << "Total Packets Sent:     " << g_totalPacketsSent << "\n";
    std::cout << "Total Packets Received: " << g_totalPacketsReceived << "\n";

    // Calcolo PDR Medio
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
    
    // Calcolo metriche di latenza
    std::cout << "--- Latency Metrics (End-to-End) ---\n";
    if (!g_delayList.empty())
    {
        Time totalDelay = Seconds(0);
        Time minDelay = g_delayList[0];
        Time maxDelay = g_delayList[0];

        // Calcola somma, min, max
        for (const auto& delay : g_delayList) {
            totalDelay += delay;
            if (delay < minDelay) minDelay = delay;
            if (delay > maxDelay) maxDelay = delay;
        }

        // Calcola media
        Time avgDelay = totalDelay / g_delayList.size();

        // Calcola Jitter (come deviazione standard della latenza in secondi)
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
    
    //Stampa TABELLE
    // Scegli il nodo da ispezionare
    Ptr<ZigbeeStack> nodeToInspect = inspectStack;
    // Scegli un tempo poco prima dei risultati finali
    double tablePrintTime = calculationTime - 0.5; // Stampa mezzo secondo prima dei risultati
    // Assicurati che il tempo non sia troppo presto se calculationTime fosse molto vicino all'ultimo invio
    if (tablePrintTime < startTime + (numPacketsToSend * interval)) {
        tablePrintTime = calculationTime; // Altrimenti stampa allo stesso tempo dei risultati
    }
    // Stampa di log/info per confermare la configurazione scelta prima della simulazione
    NS_LOG_INFO("Scheduling final tables print for Node " << nodeToInspect->GetNode()->GetId()
                << " at T=" << tablePrintTime << " s");
    std::cout << "INFO: Scheduling final tables print for Node " << nodeToInspect->GetNode()->GetId()
              << " at T=" << tablePrintTime << " s\n";
    std::cout << "----------------------------------------------------------\n";


    // Crea l'output stream wrapper per std::cout (necessario per le funzioni Print*)
    Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(&std::cout);

    // ---Schedula la stampa di una riga prima della stampa delle tabelle---
    Simulator::Schedule(Seconds(tablePrintTime), [nodeToInspect]() {
        std::cout << "----  END TRANSMISSION  ----\n";
        std::cout << "\n-----------------------------------------\n";
        std::cout << "---         Tables for Node " << nodeToInspect->GetNode()->GetId()
                  << "         ---\n";
        std::cout << "-----------------------------------------\n";
    });

    // stampa la NEIGHBOR TABLE alla fine della trasmissione di tutti i pacchetti
    Simulator::Schedule(Seconds(tablePrintTime),
                        &ZigbeeNwk::PrintNeighborTable,
                        nodeToInspect->GetNwk(),
                        stream);
    // stampa la ROUTING TABLE alla fine della trasmissione di tutti i pacchetti
    Simulator::Schedule(Seconds(tablePrintTime + 0.01), 
                        &ZigbeeNwk::PrintRoutingTable,
                        nodeToInspect->GetNwk(),
                        stream);
    // stampa la ROUTE DISCOVERY TABLE subito dopo l'invio del primo pacchetto
    //Simulator::Schedule(Seconds(startTime + 0.02), 
    //                    &ZigbeeNwk::PrintRouteDiscoveryTable,
    //                    nodeToInspect->GetNwk(),
    //                    stream);

    // Schedula TraceRoute tramite la funzione Wrapper (funziona solo con un nodo router di origine)
    Simulator::Schedule(Seconds(tablePrintTime + 0.03), // Mantieni lo stesso tempo o aggiusta se necessario
                       &ScheduleTraceRouteWrapper,      // Chiama la NUOVA funzione wrapper
                       sourceStack,                     // Passa il PUNTATORE allo stack sorgente
                       destinationStack);                // Passa il PUNTATORE allo stack destinazione

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

// --- Controllo Simulazione ---
    double stopTime = calculationTime + 5.0; // Assicurati che la simulazione finisca DOPO il calcolo
    Simulator::Stop(Seconds(stopTime));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
