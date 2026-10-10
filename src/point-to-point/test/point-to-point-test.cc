/*
 * Copyright (c) 2009 INRIA
 *
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#include "ns3/drop-tail-queue.h"
#include "ns3/error-model.h"
#include "ns3/custom-header.h"
#include "ns3/broadcom-egress-queue.h"
#include "ns3/flow-id-tag.h"
#include "ns3/integer.h"
#include "ns3/ipv4-header.h"
#include "ns3/load-balancing.h"
#include "ns3/net-device-queue-interface.h"
#include "ns3/ppp-header.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/pointer.h"
#include "ns3/qbb-channel.h"
#include "ns3/qbb-header.h"
#include "ns3/qbb-net-device.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rdma-hw.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/switch-node.h"
#include "ns3/node.h"
#include "ns3/nscc-window.h"
#include "ns3/seq-ts-header.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/udp-header.h"

#include <algorithm>
#include <map>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <vector>

using namespace ns3;

/**
 * \brief Test class for PointToPoint model
 *
 * It tries to send one packet from one NetDevice to another, over a
 * PointToPointChannel.
 */
class PointToPointTest : public TestCase
{
  public:
    /**
     * \brief Create the test
     */
    PointToPointTest();

    /**
     * \brief Run the test
     */
    void DoRun() override;

  private:
    Ptr<const Packet> m_recvdPacket; //!< received packet
    /**
     * \brief Send one packet to the device specified
     *
     * \param device NetDevice to send to.
     * \param buffer Payload content of the packet.
     * \param size Size of the payload.
     */
    void SendOnePacket(Ptr<PointToPointNetDevice> device, const uint8_t* buffer, uint32_t size);
    /**
     * \brief Callback function which sets the recvdPacket parameter
     *
     * \param dev The receiving device.
     * \param pkt The received packet.
     * \param mode The protocol mode used.
     * \param sender The sender address.
     *
     * \return A boolean indicating packet handled properly.
     */
    bool RxPacket(Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address& sender);
};

PointToPointTest::PointToPointTest()
    : TestCase("PointToPoint")
{
}

void
PointToPointTest::SendOnePacket(Ptr<PointToPointNetDevice> device,
                                const uint8_t* buffer,
                                uint32_t size)
{
    Ptr<Packet> p = Create<Packet>(buffer, size);
    device->Send(p, device->GetBroadcast(), 0x800);
}

bool
PointToPointTest::RxPacket(Ptr<NetDevice> dev,
                           Ptr<const Packet> pkt,
                           uint16_t mode,
                           const Address& sender)
{
    m_recvdPacket = pkt;
    return true;
}

void
PointToPointTest::DoRun()
{
    Ptr<Node> a = CreateObject<Node>();
    Ptr<Node> b = CreateObject<Node>();
    Ptr<PointToPointNetDevice> devA = CreateObject<PointToPointNetDevice>();
    Ptr<PointToPointNetDevice> devB = CreateObject<PointToPointNetDevice>();
    Ptr<PointToPointChannel> channel = CreateObject<PointToPointChannel>();

    devA->Attach(channel);
    devA->SetAddress(Mac48Address::Allocate());
    devA->SetQueue(CreateObject<DropTailQueue<Packet>>());
    devB->Attach(channel);
    devB->SetAddress(Mac48Address::Allocate());
    devB->SetQueue(CreateObject<DropTailQueue<Packet>>());

    a->AddDevice(devA);
    b->AddDevice(devB);

    devB->SetReceiveCallback(MakeCallback(&PointToPointTest::RxPacket, this));
    uint8_t txBuffer[] = "\"Can you tell me where my country lies?\" \\ said the unifaun to his "
                         "true love's eyes. \\ \"It lies with me!\" cried the Queen of Maybe \\ - "
                         "for her merchandise, he traded in his prize.";
    size_t txBufferSize = sizeof(txBuffer);

    Simulator::Schedule(Seconds(1.0),
                        &PointToPointTest::SendOnePacket,
                        this,
                        devA,
                        txBuffer,
                        txBufferSize);

    Simulator::Run();

    NS_TEST_EXPECT_MSG_EQ(m_recvdPacket->GetSize(), txBufferSize, "trivial");

    uint8_t
        rxBuffer[1500]; // As large as the P2P MTU size, assuming that the user didn't change it.

    m_recvdPacket->CopyData(rxBuffer, txBufferSize);
    NS_TEST_EXPECT_MSG_EQ(memcmp(rxBuffer, txBuffer, txBufferSize), 0, "trivial");

    Simulator::Destroy();
}

class UecTrimHeaderTest : public TestCase
{
  public:
    UecTrimHeaderTest()
        : TestCase("UEC trim control preserves explicit missing-payload identity")
    {
    }

    void DoRun() override
    {
                const IntHeader::Mode savedIntMode = IntHeader::mode;
                IntHeader::mode = IntHeader::TS;

        qbbHeader trim;
        trim.SetSeq(4096);
        trim.SetPG(3);
        trim.SetSport(10000);
        trim.SetDport(10001);
        trim.SetTrimPayloadSize(1000);
        trim.SetTs(123456789);

        Ptr<Packet> packet = Create<Packet>(0);
        packet->AddHeader(trim);
        Ipv4Header ip;
        ip.SetSource(Ipv4Address("11.0.1.1"));
        ip.SetDestination(Ipv4Address("11.0.2.1"));
        ip.SetProtocol(kUecTrimNotificationProtocol);
        ip.SetPayloadSize(packet->GetSize());
        packet->AddHeader(ip);
        PppHeader ppp;
        ppp.SetProtocol(0x0021);
        packet->AddHeader(ppp);

        CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                            CustomHeader::L4_Header);
        parsed.getInt = 1;
        packet->PeekHeader(parsed);
        NS_TEST_EXPECT_MSG_EQ(parsed.l3Prot, kUecTrimNotificationProtocol,
                              "trim must use its dedicated control protocol");
        NS_TEST_EXPECT_MSG_EQ(parsed.ack.sport, 10000,
                              "trim preserves the original source port");
        NS_TEST_EXPECT_MSG_EQ(parsed.ack.dport, 10001,
                              "trim preserves the original destination port");
        NS_TEST_EXPECT_MSG_EQ(parsed.ack.seq, 4096,
                              "trim preserves the missing byte sequence");
        NS_TEST_EXPECT_MSG_EQ(parsed.ack.trim_payload_size, 1000,
                              "trim preserves the missing payload size");

        CustomHeader serialized(CustomHeader::L3_Header | CustomHeader::L4_Header);
        serialized.m_tos = 0;
        serialized.m_ttl = 64;
        serialized.l3Prot = kUecTrimRepairProtocol;
        serialized.sip = Ipv4Address("11.0.2.1").Get();
        serialized.dip = Ipv4Address("11.0.1.1").Get();
        serialized.m_payloadSize = CustomHeader::GetAckSerializedSize();
        serialized.ack.sport = 10001;
        serialized.ack.dport = 10000;
        serialized.ack.flags = 0;
        serialized.ack.pg = 3;
        serialized.ack.seq = 4096;
        serialized.ack.trim_payload_size = 1000;
        serialized.ack.ih.ts = 123456789;

        Ptr<Packet> serializedPacket = Create<Packet>();
        serializedPacket->AddHeader(serialized);
        CustomHeader reparsed(CustomHeader::L3_Header | CustomHeader::L4_Header);
        reparsed.getInt = 1;
        serializedPacket->PeekHeader(reparsed);
        NS_TEST_EXPECT_MSG_EQ(
            reparsed.ack.ih.ts, serialized.ack.ih.ts,
            "ACK and trim controls must serialize their own INT metadata");

        constexpr uint32_t kPayloadSize = 1000;
        Ptr<Packet> dataPacket = Create<Packet>(kPayloadSize);
        SeqTsHeader seqTs;
        seqTs.SetSeq(4096);
        seqTs.SetPG(3);
        dataPacket->AddHeader(seqTs);
        UdpHeader udp;
        udp.SetSourcePort(10000);
        udp.SetDestinationPort(10001);
        dataPacket->AddHeader(udp);
        Ipv4Header dataIp;
        dataIp.SetSource(Ipv4Address("11.0.1.1"));
        dataIp.SetDestination(Ipv4Address("11.0.2.1"));
        dataIp.SetProtocol(0x11);
        dataIp.SetPayloadSize(dataPacket->GetSize());
        dataPacket->AddHeader(dataIp);
        dataPacket->AddHeader(ppp);

        CustomHeader dataHeader(CustomHeader::L2_Header | CustomHeader::L3_Header |
                                CustomHeader::L4_Header);
        dataHeader.getInt = 1;
        dataPacket->PeekHeader(dataHeader);
        NS_TEST_EXPECT_MSG_EQ(
            dataPacket->GetSize() - dataHeader.GetSerializedSize(), kPayloadSize,
            "trim payload accounting must exclude the parsed PPP/IP/RDMA header");

        IntHeader::mode = savedIntMode;
    }
};

class UecTrimSwitchTest : public TestCase
{
  public:
        explicit UecTrimSwitchTest(PacketTrimMode mode, bool trimRouteAvailable = true)
                : TestCase(trimRouteAvailable
                                             ? (mode == PacketTrimMode::ForwardToDestination
                                                            ? "UEC FTD trim converts a rejected RDMA packet"
                                                            : "UEC BTS trim converts a rejected RDMA packet")
                                             : "UEC trim retains a data drop when BTS metadata has no route"),
                    m_mode(mode),
                    m_trimRouteAvailable(trimRouteAvailable)
    {
    }

    void DoRun() override
    {
        const IntHeader::Mode savedIntMode = IntHeader::mode;
        IntHeader::mode = IntHeader::NONE;

        Ptr<SwitchNode> sw = CreateObject<SwitchNode>();
        Ptr<QbbNetDevice> output = CreateObject<QbbNetDevice>();
        Ptr<BEgressQueue> queue = CreateObject<BEgressQueue>();
        queue->SetAttribute("MaxBytes", DoubleValue(100.0));
        output->SetQueue(queue);
        sw->AddDevice(output);
        // A plain Node has node type 0, so this egress port is a host downlink
        // and every trim taken here is a last-hop trim (UEC 1.0.3 section
        // 4.1.4.1).
        Ptr<Node> peerNode = CreateObject<Node>();
        Ptr<QbbNetDevice> peer = CreateObject<QbbNetDevice>();
        peerNode->AddDevice(peer);
        Ptr<QbbChannel> channel = CreateObject<QbbChannel>();
        output->Attach(channel);
        peer->Attach(channel);
        sw->SetAttribute("PacketTrimMode", UintegerValue(static_cast<uint32_t>(m_mode)));

        Ipv4Address sender("11.0.1.1");
        Ipv4Address receiver("11.0.2.1");
        sw->AddTableEntry(receiver, output->GetIfIndex());
        if (m_mode == PacketTrimMode::BackToSender && m_trimRouteAvailable)
        {
            sw->AddTableEntry(sender, output->GetIfIndex());
        }

        constexpr uint32_t kPayloadSize = 1000;
        Ptr<Packet> dataPacket = Create<Packet>(kPayloadSize);
        SeqTsHeader seqTs;
        seqTs.SetSeq(4096);
        seqTs.SetPG(3);
        dataPacket->AddHeader(seqTs);
        UdpHeader udp;
        udp.SetSourcePort(10000);
        udp.SetDestinationPort(10001);
        udp.ForcePayloadSize(CustomHeader::GetUdpHeaderSize() + kPayloadSize);
        dataPacket->AddHeader(udp);
        Ipv4Header ip;
        ip.SetSource(sender);
        ip.SetDestination(receiver);
        ip.SetProtocol(0x11);
        // UEC 1.0.3 section 4.1: only DSCP_TRIMMABLE packets may be trimmed.
        ip.SetDscp(static_cast<Ipv4Header::DscpType>(kUetDscpTrimmable));
        ip.SetPayloadSize(dataPacket->GetSize());
        dataPacket->AddHeader(ip);
        PppHeader ppp;
        ppp.SetProtocol(0x0021);
        dataPacket->AddHeader(ppp);
        dataPacket->AddPacketTag(FlowIdTag(0));

        CustomHeader dataHeader(CustomHeader::L2_Header | CustomHeader::L3_Header |
                                CustomHeader::L4_Header);
        dataHeader.getInt = 1;
        dataPacket->PeekHeader(dataHeader);
        const uint32_t expectedPayload =
            dataPacket->GetSize() - dataHeader.GetSerializedSize();
        const bool expectedForward =
            m_mode == PacketTrimMode::ForwardToDestination;
        const uint32_t dataPacketBytes = dataPacket->GetSize();

        m_trimCount = 0;
        m_dropCount = 0;
        sw->m_traceTrim.ConnectWithoutContext(
            MakeCallback(&UecTrimSwitchTest::RecordTrim, this));
        sw->m_traceDrop.ConnectWithoutContext(
            MakeCallback(&UecTrimSwitchTest::RecordDrop, this));

        sw->SwitchReceiveFromDevice(nullptr, dataPacket, dataHeader);
        if (m_trimRouteAvailable)
        {
            NS_TEST_EXPECT_MSG_EQ(
                m_trimCount, 1, "switch must convert the rejected data packet once");
            NS_TEST_EXPECT_MSG_EQ(
                m_trimPayload,
                expectedPayload,
                "trim metadata must contain the original payload bytes");
            NS_TEST_EXPECT_MSG_EQ(
                m_trimTrigger,
                static_cast<uint32_t>(PacketTrimTrigger::EgressQueueLastHop),
                "queue capacity rejection on a host downlink is a last-hop trim");
            NS_TEST_EXPECT_MSG_EQ(m_trimForward,
                                  expectedForward,
                                  "trim metadata must preserve the configured direction");
            NS_TEST_EXPECT_MSG_EQ(
                m_trimDscp, kUetDscpTrimmedLastHop,
                "a trim on a host downlink carries DSCP_TRIMMED_LAST_HOP");
            if (expectedForward)
            {
                NS_TEST_EXPECT_MSG_LT(
                    m_trimmedPacketBytes, dataPacketBytes,
                    "a trimmed packet must never exceed the original packet size");
            }
            NS_TEST_EXPECT_MSG_EQ(
                m_trimSource,
                (m_mode == PacketTrimMode::ForwardToDestination ? sender.Get()
                                                                 : receiver.Get()),
                "trim IP source must follow the selected notification path");
            NS_TEST_EXPECT_MSG_EQ(
                m_trimDestination,
                (m_mode == PacketTrimMode::ForwardToDestination ? receiver.Get()
                                                                 : sender.Get()),
                "trim IP destination must follow the selected notification path");
        }
        else
        {
            NS_TEST_EXPECT_MSG_EQ(m_trimCount,
                                  0,
                                  "a trim notification without a route must not be recorded");
            NS_TEST_EXPECT_MSG_EQ(m_dropCount,
                                  1,
                                  "trim notification failure must retain the original data drop");
            NS_TEST_EXPECT_MSG_EQ(m_dropProtocol,
                                  0x11,
                                  "trim notification failure must record the lost RDMA data");
            NS_TEST_EXPECT_MSG_EQ(
                m_dropReason,
                static_cast<uint32_t>(SwitchDropReason::EgressQueue),
                "trim notification failure must retain the original drop trigger");
        }
        IntHeader::mode = savedIntMode;
    }

  private:
    void RecordTrim(Ptr<const Packet> packet, uint32_t trigger)
    {
        CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                            CustomHeader::L4_Header);
        parsed.getInt = 1;
        packet->PeekHeader(parsed);
        ++m_trimCount;
        m_trimTrigger = trigger;
        m_trimSource = parsed.sip;
        m_trimDestination = parsed.dip;
        m_trimDscp = parsed.GetIpv4Dscp();
        // A UEC 1.0.3 trim keeps the original UDP packet, truncated and remarked;
        // the non-UET back-to-sender mode emits its own control packet instead.
        m_trimForward = parsed.l3Prot == 0x11;
        if (m_trimForward)
        {
            m_trimPayload = parsed.udp.payload_size > CustomHeader::GetUdpHeaderSize()
                                ? parsed.udp.payload_size -
                                      CustomHeader::GetUdpHeaderSize()
                                : 0;
            m_trimmedPacketBytes = packet->GetSize();
        }
        else
        {
            m_trimPayload = parsed.ack.trim_payload_size;
        }
    }

    void RecordDrop(Ptr<const Packet> packet, uint32_t reason)
    {
        CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                            CustomHeader::L4_Header);
        parsed.getInt = 1;
        packet->PeekHeader(parsed);
        ++m_dropCount;
        m_dropProtocol = parsed.l3Prot;
        m_dropReason = reason;
    }

    PacketTrimMode m_mode;
    bool m_trimRouteAvailable;
    uint32_t m_trimCount = 0;
    uint32_t m_trimPayload = 0;
    uint32_t m_trimTrigger = 0;
    bool m_trimForward = false;
    uint32_t m_trimSource = 0;
    uint32_t m_trimDestination = 0;
    uint32_t m_trimDscp = 0;
    uint32_t m_trimmedPacketBytes = 0;
    uint32_t m_dropCount = 0;
    uint32_t m_dropProtocol = 0;
    uint32_t m_dropReason = 0;
};

class UecTrimRecoveryTest : public TestCase
{
  public:
    UecTrimRecoveryTest()
        : TestCase("UEC trim recovery preserves strict payload delivery")
    {
    }

    void DoRun() override
    {
        Ipv4Address sender("11.0.1.1");
        Ipv4Address receiver("11.0.2.1");
        constexpr uint16_t kSourcePort = 10000;
        constexpr uint16_t kPriorityGroup = 3;
        constexpr uint32_t kTrimmedBytes = 1000;

        Ptr<RdmaHw> senderHw = CreateObject<RdmaHw>();
        senderHw->m_cc_mode = 0;
        senderHw->m_ack_interval = 1;
        senderHw->m_backto0 = false;
        senderHw->m_retransmission_timeout_ns = 0;
        senderHw->m_max_retransmission_retries = 2;
        Ptr<QbbNetDevice> senderDevice = CreateObject<QbbNetDevice>();
        RdmaInterfaceMgr senderInterface;
        senderInterface.dev = senderDevice;
        senderHw->m_nic.push_back(senderInterface);
        senderHw->m_rtTable[receiver.Get()].push_back(0);

        Ptr<RdmaQueuePair> senderQp = CreateObject<RdmaQueuePair>(
            kPriorityGroup, sender, receiver, kSourcePort, 10001);
        senderQp->m_size = 3000;
        senderQp->snd_una = 0;
        senderQp->snd_nxt = 2000;
        senderQp->m_highest_sent = 2000;
        // One silent retransmission timeout already charged, so the trim
        // below shows it leaves the budget alone and the ACK shows the reset.
        senderQp->m_recovery_retries = 1;
        senderHw->m_qpMap[senderHw->GetQpKey(
            receiver.Get(), kSourcePort, kPriorityGroup)] = senderQp;

        CustomHeader trim;
        trim.sip = receiver.Get();
        trim.dip = sender.Get();
        trim.ack.flags = 0;
        trim.ack.dport = kSourcePort;
        trim.ack.sport = 10001;
        trim.ack.pg = kPriorityGroup;
        trim.ack.seq = 0;
        trim.ack.trim_payload_size = kTrimmedBytes;

        senderHw->RecoverTrimmedQueue(senderQp, trim);
        NS_TEST_EXPECT_MSG_EQ(senderQp->snd_nxt,
                              0,
                              "a trim must restart from the last cumulative ACK");
        NS_TEST_EXPECT_MSG_EQ(senderQp->m_trimmed_payload_bytes,
                              kTrimmedBytes,
                              "a trim must account for missing payload bytes");
        NS_TEST_EXPECT_MSG_EQ(senderQp->m_trim_notifications,
                              1,
                              "an actionable trim must be counted");
        NS_TEST_EXPECT_MSG_EQ(senderQp->m_recovery_retries,
                              1,
                              "a trim must not charge the silent-timeout budget");

        CustomHeader ack;
        ack.l3Prot = 0xFC;
        ack.sip = receiver.Get();
        ack.ack.dport = kSourcePort;
        ack.ack.pg = kPriorityGroup;
        ack.ack.seq = kTrimmedBytes;
        senderHw->ReceiveAck(Create<Packet>(), ack);
        NS_TEST_EXPECT_MSG_EQ(senderQp->snd_una,
                              kTrimmedBytes,
                              "payload delivery still requires a cumulative ACK");
        NS_TEST_EXPECT_MSG_EQ(senderQp->m_recovery_retries,
                              0,
                              "cumulative ACK progress must reset the recovery budget");

        senderHw->RecoverTrimmedQueue(senderQp, trim);
        NS_TEST_EXPECT_MSG_EQ(senderQp->m_stale_trim_notifications,
                              1,
                              "trim ranges covered by a cumulative ACK must be stale");
        NS_TEST_EXPECT_MSG_EQ(senderQp->m_trimmed_payload_bytes,
                              kTrimmedBytes,
                              "a stale trim must not manufacture additional lost bytes");

        Ptr<RdmaHw> receiverHw = CreateObject<RdmaHw>();
        Ptr<QbbNetDevice> receiverDevice = CreateObject<QbbNetDevice>();
        RdmaInterfaceMgr receiverInterface;
        receiverInterface.dev = receiverDevice;
        receiverHw->m_nic.push_back(receiverInterface);
        receiverHw->m_rtTable[sender.Get()].push_back(0);
        receiverDevice->m_traceEnqueue.ConnectWithoutContext(
            MakeCallback(&UecTrimRecoveryTest::RecordRepair, this));

        // UEC 1.0.3 section 3.5.15.1: a trimmed data packet is recognized by
        // ip.dscp before any protocol dispatch, MUST NOT advance receiver state,
        // and MUST produce a NACK identifying the original packet.
        CustomHeader trimmed;
        trimmed.l3Prot = 0x11;
        trimmed.m_tos = kUetDscpTrimmed << 2;
        trimmed.m_ttl = 64;
        trimmed.ipid = 0;
        trimmed.sip = sender.Get();
        trimmed.dip = receiver.Get();
        trimmed.udp.sport = kSourcePort;
        trimmed.udp.dport = 10001;
        trimmed.udp.pg = kPriorityGroup;
        trimmed.udp.seq = 0;
        // Trimming leaves the UDP length field unmodified, so it still reports
        // the size of the packet whose payload was discarded.
        trimmed.udp.payload_size =
            kTrimmedBytes + CustomHeader::GetUdpHeaderSize();
        receiverHw->Receive(Create<Packet>(), trimmed);
        NS_TEST_EXPECT_MSG_EQ(receiverHw->m_rxQpMap.empty(),
                              true,
                              "a trimmed packet must not create or advance a receiver QP");
        NS_TEST_EXPECT_MSG_EQ(m_repairCount,
                              1,
                              "a trimmed packet must produce one NACK control packet");
        NS_TEST_EXPECT_MSG_EQ(m_repairProtocol,
                              kUecTrimRepairProtocol,
                              "the trim NACK must use its dedicated control protocol");
        NS_TEST_EXPECT_MSG_EQ(m_repairSequence,
                              0,
                              "the trim NACK must preserve the missing byte range");
        NS_TEST_EXPECT_MSG_EQ(m_repairPayloadBytes,
                              kTrimmedBytes,
                              "the trim NACK must preserve the missing payload length");
    }

  private:
    void RecordRepair(Ptr<const Packet> packet, uint32_t queue)
    {
        CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                            CustomHeader::L4_Header);
        parsed.getInt = 1;
        packet->PeekHeader(parsed);
        ++m_repairCount;
        m_repairProtocol = parsed.l3Prot;
        m_repairSequence = parsed.ack.seq;
        m_repairPayloadBytes = parsed.ack.trim_payload_size;
        NS_TEST_ASSERT_MSG_EQ(queue, 0, "trim repair must use strict-priority control");
    }

    uint32_t m_repairCount = 0;
    uint32_t m_repairProtocol = 0;
    uint32_t m_repairSequence = 0;
    uint32_t m_repairPayloadBytes = 0;
};

class SpineIdentificationTest : public TestCase
{
  public:
    SpineIdentificationTest()
        : TestCase("IPv4 identification carries the requested and the carrying spine")
    {
    }

    void DoRun() override
    {
        for (uint32_t requested = 0; requested <= UINT8_MAX; ++requested)
        {
            for (uint32_t carrying = 0; carrying <= UINT8_MAX; ++carrying)
            {
                const uint16_t identification = SpineIdentification(requested, carrying);
                NS_TEST_ASSERT_MSG_EQ(RequestedSpine(identification),
                                      requested,
                                      "the requested spine must survive encoding");
                NS_TEST_ASSERT_MSG_EQ(CarryingSpine(identification),
                                      carrying,
                                      "the carrying spine must survive encoding");
            }
        }
        NS_TEST_EXPECT_MSG_EQ(SpineIdentification(0x12, 0x34),
                              0x1234,
                              "the requested spine is the high byte");

        Ptr<Packet> packet = Create<Packet>(0);
        Ipv4Header ip;
        ip.SetSource(Ipv4Address("11.0.1.1"));
        ip.SetDestination(Ipv4Address("11.0.2.1"));
        ip.SetProtocol(0x11);
        ip.SetIdentification(SpineIdentification(7, 3));
        packet->AddHeader(ip);
        CustomHeader parsed(CustomHeader::L3_Header);
        packet->PeekHeader(parsed);
        NS_TEST_EXPECT_MSG_EQ(RequestedSpine(parsed.ipid),
                              7,
                              "the wire carries the requested spine");
        NS_TEST_EXPECT_MSG_EQ(CarryingSpine(parsed.ipid),
                              3,
                              "the wire carries the carrying spine");
    }
};

/**
 * A leaf with one host port and four spine uplinks, its routing entry for the
 * remote host listing the uplinks out of spine order.
 */
class LoadBalancingSwitchTest : public TestCase
{
  public:
    LoadBalancingSwitchTest()
        : TestCase("A leaf routes data by the requested spine or by the entropy value")
    {
    }

    void DoRun() override
    {
        RouteBySpine();
        RouteByEntropy(LoadBalancingMode::Ecmp);
        RouteByEntropy(LoadBalancingMode::EntropyHash);
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = 4;
    Ipv4Address m_sender{"11.0.1.1"};
    Ipv4Address m_localHost{"11.0.1.2"};
    Ipv4Address m_remoteHost{"11.0.2.1"};
    // Behind a leaf that spine 2 has lost its link to, so the routing reaches
    // it through the other three spines only.
    Ipv4Address m_cutOffHost{"11.0.3.1"};

    struct Egress
    {
        uint32_t port;
        uint16_t identification;
    };

    Ptr<SwitchNode> m_leaf;
    uint32_t m_hostPort = 0;
    std::vector<uint32_t> m_spinePorts;
    std::vector<Egress> m_egress;

    void BuildLeaf(LoadBalancingMode mode)
    {
        m_leaf = CreateObject<SwitchNode>();
        m_leaf->SetAttribute("LoadBalancing", UintegerValue(static_cast<uint32_t>(mode)));
        m_leaf->SetAttribute("PfcEnabled", BooleanValue(false));
        m_hostPort = AttachPort(CreateObject<Node>());
        m_spinePorts.clear();
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            m_spinePorts.push_back(AttachPort(CreateObject<SwitchNode>()));
        }
        m_leaf->SetSpinePorts(m_spinePorts);
        for (uint32_t spine : {2, 0, 3, 1})
        {
            m_leaf->AddTableEntry(m_remoteHost, m_spinePorts[spine]);
        }
        for (uint32_t spine : {3, 0, 1})
        {
            m_leaf->AddTableEntry(m_cutOffHost, m_spinePorts[spine]);
        }
        m_leaf->AddTableEntry(m_localHost, m_hostPort);
        m_egress.clear();
    }

    uint32_t AttachPort(Ptr<Node> peerNode)
    {
        Ptr<QbbNetDevice> port = CreateObject<QbbNetDevice>();
        port->SetQueue(CreateObject<BEgressQueue>());
        m_leaf->AddDevice(port);
        Ptr<QbbNetDevice> peer = CreateObject<QbbNetDevice>();
        peerNode->AddDevice(peer);
        Ptr<QbbChannel> channel = CreateObject<QbbChannel>();
        port->Attach(channel);
        peer->Attach(channel);
        const uint32_t index = port->GetIfIndex();
        port->m_traceEnqueue.ConnectWithoutContext(
            Callback<void, Ptr<const Packet>, uint32_t>(
                [this, index](Ptr<const Packet> packet, uint32_t) {
                    CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header);
                    packet->PeekHeader(parsed);
                    m_egress.push_back({index, parsed.ipid});
                }));
        return index;
    }

    Egress Send(Ipv4Address destination, uint16_t sourcePort, uint16_t identification)
    {
        Ptr<Packet> packet = Create<Packet>(1000);
        SeqTsHeader seqTs;
        // Priority group 0 bypasses buffer admission, which is not under test.
        seqTs.SetPG(0);
        packet->AddHeader(seqTs);
        UdpHeader udp;
        udp.SetSourcePort(sourcePort);
        udp.SetDestinationPort(10001);
        packet->AddHeader(udp);
        Ipv4Header ip;
        ip.SetSource(m_sender);
        ip.SetDestination(destination);
        ip.SetProtocol(0x11);
        ip.SetIdentification(identification);
        ip.SetPayloadSize(packet->GetSize());
        packet->AddHeader(ip);
        PppHeader ppp;
        ppp.SetProtocol(0x0021);
        packet->AddHeader(ppp);
        packet->AddPacketTag(FlowIdTag(m_hostPort));
        CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                            CustomHeader::L4_Header);
        parsed.getInt = 1;
        packet->PeekHeader(parsed);
        const size_t sent = m_egress.size();
        m_leaf->SwitchReceiveFromDevice(nullptr, packet, parsed);
        NS_TEST_EXPECT_MSG_EQ(m_egress.size(), sent + 1, "the leaf forwards every packet once");
        return m_egress.size() > sent ? m_egress.back() : Egress{UINT32_MAX, 0};
    }

    void RouteBySpine()
    {
        BuildLeaf(LoadBalancingMode::SprayUniform);
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            const Egress egress = Send(m_remoteHost, 10000, SpineIdentification(spine, spine));
            NS_TEST_EXPECT_MSG_EQ(egress.port,
                                  m_spinePorts[spine],
                                  "a request leaves on the requested spine's uplink");
            NS_TEST_EXPECT_MSG_EQ(egress.identification,
                                  SpineIdentification(spine, spine),
                                  "an honoured request is not rewritten");
        }

        const Egress local = Send(m_localHost, 10000, SpineIdentification(3, 3));
        NS_TEST_EXPECT_MSG_EQ(local.port, m_hostPort, "a local destination ignores the request");

        const Egress unrouted = Send(m_cutOffHost, 10000, SpineIdentification(2, 2));
        NS_TEST_EXPECT_MSG_NE(unrouted.port,
                              m_spinePorts[2],
                              "a request for a spine with no route onward is moved, its "
                              "uplink up or not");
        NS_TEST_EXPECT_MSG_EQ(RequestedSpine(unrouted.identification), 2, "the request survives");
        NS_TEST_EXPECT_MSG_EQ(unrouted.port,
                              m_spinePorts[CarryingSpine(unrouted.identification)],
                              "the leaf records the spine that carries the packet");
        for (uint32_t spine : {0, 1, 3})
        {
            NS_TEST_EXPECT_MSG_EQ(Send(m_cutOffHost, 10000, SpineIdentification(spine, spine)).port,
                                  m_spinePorts[spine],
                                  "requests for routed spines are honoured");
        }

        DynamicCast<QbbNetDevice>(m_leaf->GetDevice(m_spinePorts[1]))->TakeDown();
        const Egress moved = Send(m_remoteHost, 10000, SpineIdentification(1, 1));
        NS_TEST_EXPECT_MSG_NE(moved.port,
                              m_spinePorts[1],
                              "a request for a dead uplink is moved to a live one");
        NS_TEST_EXPECT_MSG_EQ(RequestedSpine(moved.identification),
                              1,
                              "the request survives the move");
        NS_TEST_EXPECT_MSG_EQ(moved.port,
                              m_spinePorts[CarryingSpine(moved.identification)],
                              "the leaf records the spine that carries the packet");
        const Egress again = Send(m_remoteHost, 10000, moved.identification);
        NS_TEST_EXPECT_MSG_EQ(again.port,
                              moved.port,
                              "a packet that passes the leaf again keeps its spine");
        const Egress live = Send(m_remoteHost, 10000, SpineIdentification(3, 3));
        NS_TEST_EXPECT_MSG_EQ(live.port,
                              m_spinePorts[3],
                              "requests for live uplinks are unaffected by a dead one");
        FoldByFlow();
    }

    // With spine 1 dead, the spine a flow's request for it lands on: the
    // same on every send while the live spines stay the same, and across
    // flows each live spine equally often.
    void FoldByFlow()
    {
        constexpr uint32_t kFlows = 6000;
        std::map<uint32_t, uint32_t> perPort;
        for (uint32_t flow = 0; flow < kFlows; ++flow)
        {
            const uint16_t sourcePort = 10000 + flow;
            const uint32_t port = Send(m_remoteHost, sourcePort, SpineIdentification(1, 1)).port;
            ++perPort[port];
            for (uint32_t repeat = 0; repeat < 3; ++repeat)
            {
                NS_TEST_ASSERT_MSG_EQ(Send(m_remoteHost, sourcePort, SpineIdentification(1, 1)).port,
                                      port,
                                      "a flow's request keeps one spine while the live ones stay");
            }
        }
        NS_TEST_ASSERT_MSG_EQ(perPort.size(), kSpines - 1, "every live spine takes a share");
        NS_TEST_EXPECT_MSG_EQ(perPort.count(m_spinePorts[1]), 0, "the dead spine takes none");
        // 2000 expected per live spine, with a binomial standard deviation of
        // 36.5; four of them either way.
        for (const auto& [port, flows] : perPort)
        {
            NS_TEST_EXPECT_MSG_GT(flows, 1854, "the dead spine's share spreads evenly");
            NS_TEST_EXPECT_MSG_LT(flows, 2146, "the dead spine's share spreads evenly");
        }
    }

    void RouteByEntropy(LoadBalancingMode mode)
    {
        BuildLeaf(mode);
        std::set<uint32_t> ports;
        for (uint32_t identification = 0; identification <= UINT8_MAX; ++identification)
        {
            ports.insert(Send(m_remoteHost, 10000, identification).port);
        }
        if (mode == LoadBalancingMode::Ecmp)
        {
            NS_TEST_EXPECT_MSG_EQ(ports.size(), 1, "ECMP keeps a flow on one path");
        }
        else
        {
            NS_TEST_EXPECT_MSG_EQ(ports.size(),
                                  kSpines,
                                  "the entropy value spreads one flow over every uplink");
            const uint32_t port = Send(m_remoteHost, 10000, 0x1234).port;
            NS_TEST_EXPECT_MSG_EQ(Send(m_remoteHost, 10000, 0x1234).port,
                                  port,
                                  "an entropy value always takes the same path");
        }
    }
};

/**
 * A packet arriving at a switch on a port: data of priority group pg, or a
 * control packet of the given protocol, carried in the same headers.
 */
void ArriveAtSwitch(Ptr<SwitchNode> sw,
                    Ptr<NetDevice> port,
                    Ipv4Address from,
                    Ipv4Address to,
                    uint16_t pg,
                    uint8_t protocol = 0x11)
{
    Ptr<Packet> packet = Create<Packet>(1000);
    SeqTsHeader seqTs;
    seqTs.SetPG(pg);
    packet->AddHeader(seqTs);
    UdpHeader udp;
    udp.SetSourcePort(10000);
    udp.SetDestinationPort(10001);
    packet->AddHeader(udp);
    Ipv4Header ip;
    ip.SetSource(from);
    ip.SetDestination(to);
    ip.SetProtocol(protocol);
    ip.SetDscp(static_cast<Ipv4Header::DscpType>(kUetDscpTrimmable));
    ip.SetPayloadSize(packet->GetSize());
    packet->AddHeader(ip);
    PppHeader ppp;
    ppp.SetProtocol(0x0021);
    packet->AddHeader(ppp);
    packet->AddPacketTag(FlowIdTag(port->GetIfIndex()));
    CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                        CustomHeader::L4_Header);
    parsed.getInt = 1;
    packet->PeekHeader(parsed);
    sw->SwitchReceiveFromDevice(port, packet, parsed);
}

/**
 * A port of sw linked to a device of peerNode; returns the switch's device.
 */
Ptr<QbbNetDevice> AttachSwitchPort(Ptr<SwitchNode> sw, Ptr<Node> peerNode)
{
    Ptr<QbbNetDevice> port = CreateObject<QbbNetDevice>();
    port->SetQueue(CreateObject<BEgressQueue>());
    sw->AddDevice(port);
    Ptr<QbbNetDevice> peer = CreateObject<QbbNetDevice>();
    peerNode->AddDevice(peer);
    Ptr<QbbChannel> channel = CreateObject<QbbChannel>();
    port->Attach(channel);
    peer->Attach(channel);
    return port;
}

/**
 * A switch with a host port and an uplink that queues data, its buffer
 * configured as common.h configures a best-effort switch.
 */
class PortDownBufferTest : public TestCase
{
  public:
    PortDownBufferTest()
        : TestCase("A port taken down returns its queued packets' buffer")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kPriorityGroup = 3;
        constexpr uint32_t kPackets = 5;
        Ptr<SwitchNode> sw = CreateObject<SwitchNode>();
        sw->SetAttribute("PfcEnabled", BooleanValue(false));
        Ptr<QbbNetDevice> hostDevice = AttachSwitchPort(sw, CreateObject<Node>());
        const uint32_t hostPort = hostDevice->GetIfIndex();
        const uint32_t uplink = AttachSwitchPort(sw, CreateObject<SwitchNode>())->GetIfIndex();
        for (uint32_t port : {hostPort, uplink})
        {
            sw->m_mmu->ConfigHdrm(port, 0);
            sw->m_mmu->pfc_a_shift[port] = 3;
        }
        sw->m_mmu->ConfigNPort(2);
        sw->m_mmu->ConfigBufferSize(32 * 1024 * 1024);
        Ipv4Address remote("11.0.2.1");
        sw->AddTableEntry(remote, uplink);

        uint32_t packetBytes = 0;
        DynamicCast<QbbNetDevice>(sw->GetDevice(uplink))
            ->m_traceEnqueue.ConnectWithoutContext(Callback<void, Ptr<const Packet>, uint32_t>(
                [&packetBytes](Ptr<const Packet> packet, uint32_t) {
                    packetBytes = packet->GetSize();
                }));
        for (uint32_t i = 0; i < kPackets; ++i)
        {
            ArriveAtSwitch(sw, hostDevice, Ipv4Address("11.0.1.1"), remote, kPriorityGroup);
        }
        // The first packet went onto the wire and released its buffer then.
        const uint32_t queued = (kPackets - 1) * packetBytes;
        NS_TEST_ASSERT_MSG_EQ(sw->m_mmu->egress_bytes[uplink][kPriorityGroup],
                              queued,
                              "the uplink holds every packet behind the one on the wire");
        NS_TEST_ASSERT_MSG_EQ(sw->m_mmu->ingress_bytes[hostPort][kPriorityGroup],
                              queued,
                              "the host port is charged for what the uplink holds");

        DynamicCast<QbbNetDevice>(sw->GetDevice(uplink))->TakeDown();
        NS_TEST_EXPECT_MSG_EQ(sw->m_mmu->egress_bytes[uplink][kPriorityGroup],
                              0,
                              "a downed port holds no buffer");
        NS_TEST_EXPECT_MSG_EQ(sw->m_mmu->ingress_bytes[hostPort][kPriorityGroup],
                              0,
                              "the ports that fed it are charged nothing for discarded packets");
        Simulator::Destroy();
    }
};

/**
 * A host's device on a lossy link: whatever arrives either reaches the
 * transport or is dropped by the link's error model, whatever it carries.
 */
class LinkErrorTest : public TestCase
{
  public:
    LinkErrorTest()
        : TestCase("A lossy link drops data and control at its configured rate")
    {
    }

    void DoRun() override
    {
        RngSeedManager::SetSeed(1);
        RngSeedManager::SetRun(1);
        Ptr<QbbNetDevice> device = CreateObject<QbbNetDevice>();
        CreateObject<Node>()->AddDevice(device);
        Ptr<QbbNetDevice> peer = CreateObject<QbbNetDevice>();
        CreateObject<Node>()->AddDevice(peer);
        Ptr<QbbChannel> channel = CreateObject<QbbChannel>();
        device->Attach(channel);
        peer->Attach(channel);
        uint32_t delivered = 0;
        uint32_t dropped = 0;
        device->m_rdmaReceiveCb = Callback<int, Ptr<Packet>, CustomHeader&>(
            [&delivered](Ptr<Packet>, CustomHeader&) {
                ++delivered;
                return 0;
            });
        device->TraceConnectWithoutContext(
            "LinkErrorDrop",
            Callback<void, Ptr<const Packet>, uint32_t>(
                [&dropped](Ptr<const Packet>, uint32_t) { ++dropped; }));

        constexpr uint32_t kPackets = 4000;
        Receive(device, kPackets);
        NS_TEST_EXPECT_MSG_EQ(delivered, kPackets, "a link without an error model loses nothing");
        NS_TEST_EXPECT_MSG_EQ(dropped, 0, "a link without an error model reports no loss");

        delivered = 0;
        Ptr<RateErrorModel> model = CreateObject<RateErrorModel>();
        model->SetAttribute("ErrorRate", DoubleValue(0.25));
        model->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
        model->SetRandomVariable(
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(7)));
        device->SetAttribute("LinkErrorModel", PointerValue(model));
        Receive(device, kPackets);
        NS_TEST_EXPECT_MSG_EQ(delivered + dropped, kPackets, "every packet is delivered or dropped");
        // 1000 expected with a standard deviation of 27.
        NS_TEST_EXPECT_MSG_GT(dropped, 900, "the link drops at its configured rate");
        NS_TEST_EXPECT_MSG_LT(dropped, 1100, "the link drops at its configured rate");

        delivered = dropped = 0;
        model->SetAttribute("ErrorRate", DoubleValue(1.0));
        Receive(device, 10, 0xFC);
        NS_TEST_EXPECT_MSG_EQ(dropped, 10, "a link loses acknowledgements as it loses data");
        NS_TEST_EXPECT_MSG_EQ(delivered, 0, "a link that loses everything delivers nothing");
        Simulator::Destroy();
    }

  private:
    static void Receive(Ptr<QbbNetDevice> device, uint32_t packets, uint8_t protocol = 0x11)
    {
        for (uint32_t i = 0; i < packets; ++i)
        {
            Ptr<Packet> packet = Create<Packet>(1000);
            Ipv4Header ip;
            ip.SetSource(Ipv4Address("11.0.1.1"));
            ip.SetDestination(Ipv4Address("11.0.2.1"));
            ip.SetProtocol(protocol);
            ip.SetPayloadSize(packet->GetSize());
            packet->AddHeader(ip);
            PppHeader ppp;
            ppp.SetProtocol(0x0021);
            packet->AddHeader(ppp);
            device->Receive(packet);
        }
    }
};

/**
 * A spine with two leaf ports, one of which has stopped forwarding.
 */
class BlackholeTest : public TestCase
{
  public:
    BlackholeTest()
        : TestCase("A black-holed port drops the data arriving on it without notice")
    {
    }

    void DoRun() override
    {
        Ptr<SwitchNode> spine = CreateObject<SwitchNode>();
        spine->SetAttribute("PfcEnabled", BooleanValue(false));
        spine->SetAttribute("PacketTrimMode",
                            UintegerValue(static_cast<uint32_t>(PacketTrimMode::ForwardToDestination)));
        Ptr<QbbNetDevice> fromA = AttachSwitchPort(spine, CreateObject<SwitchNode>());
        Ptr<QbbNetDevice> fromB = AttachSwitchPort(spine, CreateObject<SwitchNode>());
        Ipv4Address hostA("11.0.1.1");
        Ipv4Address hostB("11.0.2.1");
        spine->AddTableEntry(hostA, fromA->GetIfIndex());
        spine->AddTableEntry(hostB, fromB->GetIfIndex());
        std::vector<uint32_t> drops;
        uint32_t trims = 0;
        spine->m_traceDrop.ConnectWithoutContext(Callback<void, Ptr<const Packet>, uint32_t>(
            [&drops](Ptr<const Packet>, uint32_t reason) { drops.push_back(reason); }));
        spine->m_traceTrim.ConnectWithoutContext(Callback<void, Ptr<const Packet>, uint32_t>(
            [&trims](Ptr<const Packet>, uint32_t) { ++trims; }));
        uint32_t sentToB = 0;
        uint32_t sentToA = 0;
        fromB->m_traceEnqueue.ConnectWithoutContext(Callback<void, Ptr<const Packet>, uint32_t>(
            [&sentToB](Ptr<const Packet>, uint32_t) { ++sentToB; }));
        fromA->m_traceEnqueue.ConnectWithoutContext(Callback<void, Ptr<const Packet>, uint32_t>(
            [&sentToA](Ptr<const Packet>, uint32_t) { ++sentToA; }));

        // Priority group 0 bypasses buffer admission, which is not under test.
        ArriveAtSwitch(spine, fromA, hostA, hostB, 0);
        ArriveAtSwitch(spine, fromB, hostB, hostA, 0);
        NS_TEST_EXPECT_MSG_EQ(sentToB, 1, "a working spine forwards from A to B");
        NS_TEST_EXPECT_MSG_EQ(sentToA, 1, "a working spine forwards from B to A");

        spine->SetBlackhole(fromA->GetIfIndex());
        ArriveAtSwitch(spine, fromA, hostA, hostB, 0);
        ArriveAtSwitch(spine, fromA, hostA, hostB, 0);
        NS_TEST_EXPECT_MSG_EQ(sentToB, 1, "no data arriving from A leaves the spine");
        NS_TEST_ASSERT_MSG_EQ(drops.size(), 2, "every data packet is dropped");
        for (uint32_t reason : drops)
        {
            NS_TEST_EXPECT_MSG_EQ(reason,
                                  static_cast<uint32_t>(SwitchDropReason::Blackhole),
                                  "the drop is the port's, not admission's");
        }
        NS_TEST_EXPECT_MSG_EQ(trims, 0, "a black hole sends nobody a trimmed packet");
        ArriveAtSwitch(spine, fromA, hostA, hostB, 0, 0xFC);
        NS_TEST_EXPECT_MSG_EQ(sentToB, 2, "an acknowledgement from A still crosses the spine");
        NS_TEST_EXPECT_MSG_EQ(fromA->IsLinkUp(), true, "the link stays up");

        ArriveAtSwitch(spine, fromB, hostB, hostA, 0);
        NS_TEST_EXPECT_MSG_EQ(sentToA, 2, "the spine still forwards toward A");
        Simulator::Destroy();
    }
};

/**
 * A switch port toward a host whose link is slowed to half its rate while
 * the run goes on.
 */
class LinkRateChangeTest : public TestCase
{
  public:
    LinkRateChangeTest()
        : TestCase("A link slowed mid-run serializes later packets at its new rate")
    {
    }

    void DoRun() override
    {
        Ptr<SwitchNode> sw = CreateObject<SwitchNode>();
        sw->SetAttribute("PfcEnabled", BooleanValue(false));
        Ptr<QbbNetDevice> port = CreateObject<QbbNetDevice>();
        port->SetQueue(CreateObject<BEgressQueue>());
        port->SetDataRate(DataRate("400Gbps"));
        sw->AddDevice(port);
        Ptr<QbbNetDevice> host = CreateObject<QbbNetDevice>();
        host->SetDataRate(DataRate("400Gbps"));
        CreateObject<Node>()->AddDevice(host);
        Ptr<QbbChannel> channel = CreateObject<QbbChannel>();
        port->Attach(channel);
        host->Attach(channel);
        Ipv4Address sender("11.0.1.1");
        Ipv4Address receiver("11.0.2.1");
        sw->AddTableEntry(receiver, port->GetIfIndex());
        std::vector<int64_t> arrivals;
        host->m_rdmaReceiveCb = Callback<int, Ptr<Packet>, CustomHeader&>(
            [&arrivals](Ptr<Packet>, CustomHeader&) {
                arrivals.push_back(Simulator::Now().GetNanoSeconds());
                return 0;
            });

        uint32_t wireBytes = 0;
        port->m_traceDequeue.ConnectWithoutContext(Callback<void, Ptr<const Packet>, uint32_t>(
            [&wireBytes](Ptr<const Packet> packet, uint32_t) { wireBytes = packet->GetSize(); }));
        // Priority group 0 bypasses buffer admission, which is not under test.
        Simulator::Schedule(NanoSeconds(0), [&]() { ArriveAtSwitch(sw, port, sender, receiver, 0); });
        Simulator::Schedule(NanoSeconds(1000), [&]() {
            port->SetDataRate(DataRate("200Gbps"));
            host->SetDataRate(DataRate("200Gbps"));
        });
        Simulator::Schedule(NanoSeconds(2000), [&]() { ArriveAtSwitch(sw, port, sender, receiver, 0); });
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(arrivals.size(), 2, "both packets arrive");
        NS_TEST_EXPECT_MSG_EQ(
            arrivals[0],
            DataRate("400Gbps").CalculateBytesTxTime(wireBytes).GetNanoSeconds(),
            "the first packet is serialized at 400 Gb/s");
        NS_TEST_EXPECT_MSG_EQ(
            arrivals[1] - 2000,
            DataRate("200Gbps").CalculateBytesTxTime(wireBytes).GetNanoSeconds(),
            "the second is serialized at 200 Gb/s");
        Simulator::Destroy();
    }
};

/**
 * A switch that marks every data packet it sends while more wait behind it,
 * sending five data packets and an acknowledgement up one port.
 */
class PortCountersTest : public TestCase
{
  public:
    PortCountersTest()
        : TestCase("A switch port counts what it sends, the data among it and its marks")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kData = 5;
        Ptr<SwitchNode> sw = CreateObject<SwitchNode>();
        sw->SetAttribute("PfcEnabled", BooleanValue(false));
        sw->SetAttribute("EcnEnabled", BooleanValue(true));
        sw->SetAttribute("AckHighPrio", UintegerValue(1));
        Ptr<QbbNetDevice> hostPort = AttachSwitchPort(sw, CreateObject<Node>());
        Ptr<QbbNetDevice> uplink = AttachSwitchPort(sw, CreateObject<SwitchNode>());
        for (Ptr<QbbNetDevice> port : {hostPort, uplink})
        {
            sw->m_mmu->ConfigHdrm(port->GetIfIndex(), 0);
            sw->m_mmu->pfc_a_shift[port->GetIfIndex()] = 3;
            sw->m_mmu->ConfigEcn(port->GetIfIndex(), 0, 0, 1.0);
        }
        sw->m_mmu->ConfigNPort(2);
        sw->m_mmu->ConfigBufferSize(32 * 1024 * 1024);
        Ipv4Address sender("11.0.1.1");
        Ipv4Address remote("11.0.2.1");
        sw->AddTableEntry(remote, uplink->GetIfIndex());
        uint64_t sentBytes = 0;
        uplink->m_traceDequeue.ConnectWithoutContext(Callback<void, Ptr<const Packet>, uint32_t>(
            [&sentBytes](Ptr<const Packet> packet, uint32_t) { sentBytes += packet->GetSize(); }));

        for (uint32_t i = 0; i < kData; ++i)
        {
            ArriveAtSwitch(sw, hostPort, sender, remote, 3);
        }
        ArriveAtSwitch(sw, hostPort, sender, remote, 3, 0xFC);
        Simulator::Run();

        const SwitchPortCounters& sent = sw->GetPortCounters(uplink->GetIfIndex());
        NS_TEST_EXPECT_MSG_EQ(sent.txPackets, kData + 1, "every packet sent is counted");
        NS_TEST_EXPECT_MSG_EQ(sent.txBytes, sentBytes, "every byte sent is counted");
        NS_TEST_EXPECT_MSG_EQ(sent.dataPackets, kData, "the acknowledgement is not data");
        // The first leaves at once and the last with nothing behind it; the
        // acknowledgement rides queue 0, which is never marked.
        NS_TEST_EXPECT_MSG_EQ(sent.ecnMarks, kData - 2, "each mark the port sets is counted");
        const SwitchPortCounters& idle = sw->GetPortCounters(hostPort->GetIfIndex());
        NS_TEST_EXPECT_MSG_EQ(idle.txPackets + idle.txBytes + idle.dataPackets + idle.ecnMarks,
                              0,
                              "a port that sent nothing counts nothing");
        Simulator::Destroy();
    }
};

class LoadBalancingSenderTest : public TestCase
{
  public:
    LoadBalancingSenderTest()
        : TestCase("A sender writes the path draw into the IPv4 identification")
    {
    }

    void DoRun() override
    {
        const std::vector<uint16_t> counter = Identifications(LoadBalancingMode::Ecmp, 4);
        for (uint32_t i = 0; i < counter.size(); ++i)
        {
            NS_TEST_ASSERT_MSG_EQ(counter[i], i, "ECMP keeps the per-QP counter");
        }

        constexpr uint32_t kSpines = 8;
        constexpr uint32_t kPackets = 8000;
        std::vector<uint32_t> perSpine(kSpines, 0);
        for (uint16_t identification :
             Identifications(LoadBalancingMode::SprayUniform, kPackets, kSpines))
        {
            NS_TEST_ASSERT_MSG_EQ(RequestedSpine(identification),
                                  CarryingSpine(identification),
                                  "the sender carries the spine it requests");
            NS_TEST_ASSERT_MSG_LT(RequestedSpine(identification),
                                  kSpines,
                                  "the sender names an existing spine");
            ++perSpine[RequestedSpine(identification)];
        }
        // 1000 expected per spine with a standard deviation of 30.
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_GT(perSpine[spine], 850, "the spine draw is uniform");
            NS_TEST_EXPECT_MSG_LT(perSpine[spine], 1150, "the spine draw is uniform");
        }

        const std::vector<uint16_t> entropy =
            Identifications(LoadBalancingMode::EntropyHash, 1000);
        const std::set<uint16_t> distinct(entropy.begin(), entropy.end());
        // 1000 draws from 65536 values repeat about 8 times.
        NS_TEST_EXPECT_MSG_GT(distinct.size(), 950, "the entropy draw spans 16 bits");
        NS_TEST_EXPECT_MSG_GT(*distinct.rbegin(), UINT8_MAX, "the entropy draw uses the high byte");

        // A fixed stream must not take an automatic one, or turning a mode on
        // would move every later automatic draw of the run.
        const uint64_t before = RngSeedManager::GetNextStreamIndex();
        PathVariable();
        NS_TEST_EXPECT_MSG_EQ(RngSeedManager::GetNextStreamIndex(),
                              before + 1,
                              "the path variable allocates no automatic stream");
        const bool repeated = Identifications(LoadBalancingMode::EntropyHash, 1000) == entropy;
        NS_TEST_EXPECT_MSG_EQ(repeated, true, "the fixed stream repeats its draws");
    }

  private:
    static Ptr<UniformRandomVariable> PathVariable()
    {
        return CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(0));
    }

    static std::vector<uint16_t> Identifications(LoadBalancingMode mode,
                                                 uint32_t packets,
                                                 uint32_t spines = 0)
    {
        constexpr uint32_t kMtu = 1000;
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        hw->SetAttribute("Mtu", UintegerValue(kMtu));
        hw->SetAttribute("LoadBalancing", UintegerValue(static_cast<uint32_t>(mode)));
        hw->SetAttribute("SpineCount", UintegerValue(spines));
        hw->SetAttribute("PathRandomVariable", PointerValue(PathVariable()));
        Ptr<RdmaQueuePair> qp = CreateObject<RdmaQueuePair>(
            3, Ipv4Address("11.0.1.1"), Ipv4Address("11.0.2.1"), 10000, 10001);
        qp->m_size = static_cast<uint64_t>(packets) * kMtu;
        if (mode != LoadBalancingMode::Ecmp)
        {
            qp->m_outstanding.SetPacketSize(kMtu);
        }
        std::vector<uint16_t> identifications;
        for (uint32_t i = 0; i < packets; ++i)
        {
            CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header);
            hw->GetNxtPacket(qp)->PeekHeader(parsed);
            identifications.push_back(parsed.ipid);
        }
        return identifications;
    }
};

class AckPacketSeqHeaderTest : public TestCase
{
  public:
    AckPacketSeqHeaderTest()
        : TestCase("An acknowledgement header carries the packet sequence only when asked")
    {
    }

    void DoRun() override
    {
        const IntHeader::Mode savedIntMode = IntHeader::mode;
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        IntHeader::mode = IntHeader::NONE;

        CustomHeader::ackCarriesPacketSeq = false;
        NS_TEST_EXPECT_MSG_EQ(qbbHeader().GetSerializedSize(),
                              16,
                              "without the packet sequence the header keeps its size");
        NS_TEST_EXPECT_MSG_EQ(CustomHeader::GetAckSerializedSize(),
                              16,
                              "the parser agrees on the size without the packet sequence");

        CustomHeader::ackCarriesPacketSeq = true;
        NS_TEST_EXPECT_MSG_EQ(qbbHeader().GetSerializedSize(),
                              20,
                              "the packet sequence adds four bytes");
        NS_TEST_EXPECT_MSG_EQ(CustomHeader::GetAckSerializedSize(),
                              20,
                              "the parser agrees on the size with the packet sequence");

        qbbHeader ack;
        ack.SetSeq(3000);
        ack.SetPacketSeq(7000);
        ack.SetPG(3);
        ack.SetSport(10001);
        ack.SetDport(10000);
        ack.SetCnp();
        Ptr<Packet> packet = Create<Packet>(0);
        packet->AddHeader(ack);
        Ipv4Header ip;
        ip.SetSource(Ipv4Address("11.0.2.1"));
        ip.SetDestination(Ipv4Address("11.0.1.1"));
        ip.SetProtocol(0xFC);
        ip.SetPayloadSize(packet->GetSize());
        packet->AddHeader(ip);
        PppHeader ppp;
        ppp.SetProtocol(0x0021);
        packet->AddHeader(ppp);
        CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                            CustomHeader::L4_Header);
        packet->PeekHeader(parsed);
        NS_TEST_EXPECT_MSG_EQ(parsed.ack.seq, 3000, "the cumulative sequence keeps its place");
        NS_TEST_EXPECT_MSG_EQ(parsed.ack.packet_seq, 7000, "the packet sequence survives the wire");
        const bool marked = (parsed.ack.flags >> qbbHeader::FLAG_CNP) & 1;
        NS_TEST_EXPECT_MSG_EQ(marked,
                              true,
                              "the flags precede the packet sequence");
        NS_TEST_EXPECT_MSG_EQ(parsed.GetSerializedSize(),
                              packet->GetSize(),
                              "the parser consumes exactly the header written");

        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        IntHeader::mode = savedIntMode;
    }
};

/**
 * A host transport with one NIC that is never attached, so nothing it emits
 * leaves the host. Control packets are parsed off the NIC's enqueue trace in
 * the order they are emitted.
 */
class IsolatedHost
{
  public:
    static constexpr uint32_t kMtu = 1000;
    static constexpr uint16_t kSenderPort = 10000;
    static constexpr uint16_t kReceiverPort = 10001;
    static constexpr uint16_t kPriorityGroup = 3;
    static constexpr uint64_t kTimeoutNs = 1000;

    IsolatedHost(LoadBalancingMode mode, Ipv4Address peer, uint32_t spines = 0)
    {
        hw = CreateObject<RdmaHw>();
        hw->m_mtu = kMtu;
        hw->m_cc_mode = 0;
        hw->m_ack_interval = 1;
        hw->m_chunk = 4000;
        hw->m_backto0 = false;
        hw->m_selective_retransmission = true;
        hw->m_retransmission_timeout_ns = kTimeoutNs;
        hw->m_max_retransmission_retries = 4;
        hw->m_no_progress_timeout_ns = 0;
        hw->m_loadBalancing = static_cast<uint32_t>(mode);
        hw->m_spineCount = spines;
        hw->m_pathRandom =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(0));
        Ptr<QbbNetDevice> device = CreateObject<QbbNetDevice>();
        device->m_traceEnqueue.ConnectWithoutContext(
            Callback<void, Ptr<const Packet>, uint32_t>(
                [this](Ptr<const Packet> packet, uint32_t) {
                    CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                                        CustomHeader::L4_Header);
                    packet->PeekHeader(parsed);
                    emitted.push_back(parsed);
                }));
        RdmaInterfaceMgr nic;
        nic.dev = device;
        hw->m_nic.push_back(nic);
        hw->m_rtTable[peer.Get()].push_back(0);
    }

    // A sender queue pair as AddQueuePair registers it, without the NIC's
    // scheduler, which no test here drives.
    Ptr<RdmaQueuePair> AddSender(Ipv4Address self, Ipv4Address peer, uint64_t size)
    {
        Ptr<RdmaQueuePair> qp =
            CreateObject<RdmaQueuePair>(kPriorityGroup, self, peer, kSenderPort, kReceiverPort);
        qp->m_size = size;
        if (hw->IsPathPerPacket())
        {
            qp->m_outstanding.SetPacketSize(kMtu);
        }
        hw->m_qpMap[RdmaHw::GetQpKey(peer.Get(), kSenderPort, kPriorityGroup)] = qp;
        return qp;
    }

    // A data packet from the sender, parsed as the receiving NIC parses it,
    // handed to this host's transport.
    void ReceiveData(Ipv4Address from,
                     Ipv4Address to,
                     uint32_t seq,
                     uint16_t identification,
                     bool marked = false)
    {
        Ptr<Packet> packet = Create<Packet>(kMtu);
        SeqTsHeader seqTs;
        seqTs.SetSeq(seq);
        seqTs.SetPG(kPriorityGroup);
        packet->AddHeader(seqTs);
        UdpHeader udp;
        udp.SetSourcePort(kSenderPort);
        udp.SetDestinationPort(kReceiverPort);
        udp.ForcePayloadSize(CustomHeader::GetUdpHeaderSize() + kMtu);
        packet->AddHeader(udp);
        Ipv4Header ip;
        ip.SetSource(from);
        ip.SetDestination(to);
        ip.SetProtocol(0x11);
        ip.SetDscp(static_cast<Ipv4Header::DscpType>(kUetDscpTrimmable));
        ip.SetEcn(marked ? Ipv4Header::ECN_CE : Ipv4Header::ECN_NotECT);
        ip.SetIdentification(identification);
        ip.SetPayloadSize(packet->GetSize());
        packet->AddHeader(ip);
        PppHeader ppp;
        ppp.SetProtocol(0x0021);
        packet->AddHeader(ppp);
        CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                            CustomHeader::L4_Header);
        packet->PeekHeader(parsed);
        hw->Receive(packet, parsed);
    }

    // The acknowledgement of the send of seq along the path that
    // identification names, with the receiver's cumulative sequence.
    void ReceiveAck(Ipv4Address from,
                    uint32_t cumulative,
                    uint32_t seq,
                    uint16_t identification,
                    bool marked = false)
    {
        CustomHeader ack;
        ack.l3Prot = 0xFC;
        ack.sip = from.Get();
        ack.ipid = identification;
        ack.ack.flags = marked ? 1 << qbbHeader::FLAG_CNP : 0;
        ack.ack.sport = kReceiverPort;
        ack.ack.dport = kSenderPort;
        ack.ack.pg = kPriorityGroup;
        ack.ack.seq = cumulative;
        ack.ack.packet_seq = seq;
        hw->ReceiveAck(Create<Packet>(), ack);
    }

    // The receiver's repair request for the trimmed send of seq.
    void ReceiveTrimNack(Ptr<RdmaQueuePair> qp, Ipv4Address from, uint32_t seq, uint16_t identification)
    {
        CustomHeader trim;
        trim.l3Prot = kUecTrimRepairProtocol;
        trim.sip = from.Get();
        trim.ipid = identification;
        trim.ack.flags = 0;
        trim.ack.sport = kReceiverPort;
        trim.ack.dport = kSenderPort;
        trim.ack.pg = kPriorityGroup;
        trim.ack.seq = seq;
        trim.ack.trim_payload_size = kMtu;
        hw->RecoverTrimmedQueue(qp, trim);
    }

    Ptr<RdmaHw> hw;
    std::vector<CustomHeader> emitted;
};

const Ipv4Address kTestSender("11.0.1.1");
const Ipv4Address kTestReceiver("11.0.2.1");

class AckNamesPacketTest : public TestCase
{
  public:
    AckNamesPacketTest()
        : TestCase("An acknowledgement names the data packet it answers and its path")
    {
    }

    void DoRun() override
    {
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        CustomHeader::ackCarriesPacketSeq = true;
        IsolatedHost receiver(LoadBalancingMode::SprayUniform, kTestSender, 8);
        // Moved by its leaf from spine 2 to spine 5, then two packets out of
        // order, the first of them marked.
        receiver.ReceiveData(kTestSender, kTestReceiver, 0, SpineIdentification(2, 5));
        receiver.ReceiveData(kTestSender, kTestReceiver, 2000, SpineIdentification(6, 6), true);
        receiver.ReceiveData(kTestSender, kTestReceiver, 1000, SpineIdentification(1, 1));
        NS_TEST_ASSERT_MSG_EQ(receiver.emitted.size(), 3, "every data packet is acknowledged");
        const uint32_t seqs[] = {0, 2000, 1000};
        const uint32_t cumulative[] = {1000, 1000, 3000};
        const uint16_t identifications[] = {SpineIdentification(2, 5),
                                            SpineIdentification(6, 6),
                                            SpineIdentification(1, 1)};
        for (uint32_t i = 0; i < 3; ++i)
        {
            const CustomHeader& ack = receiver.emitted[i];
            NS_TEST_EXPECT_MSG_EQ(ack.l3Prot, 0xFC, "each answer is an acknowledgement");
            NS_TEST_EXPECT_MSG_EQ(ack.ack.packet_seq, seqs[i], "the answer names its packet");
            NS_TEST_EXPECT_MSG_EQ(ack.ipid,
                                  identifications[i],
                                  "the answer returns the packet's requested and carrying spine");
            NS_TEST_EXPECT_MSG_EQ(ack.ack.seq,
                                  cumulative[i],
                                  "the sequence stays the cumulative acknowledgement");
            const bool marked = (ack.ack.flags >> qbbHeader::FLAG_CNP) & 1;
            NS_TEST_EXPECT_MSG_EQ(marked,
                                  (i == 1),
                                  "the answer echoes its own packet's mark");
        }
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;

        IsolatedHost ecmp(LoadBalancingMode::Ecmp, kTestSender);
        ecmp.ReceiveData(kTestSender, kTestReceiver, 0, 0x1234);
        ecmp.ReceiveData(kTestSender, kTestReceiver, 1000, 0x1234);
        NS_TEST_ASSERT_MSG_EQ(ecmp.emitted.size(), 2, "ECMP acknowledges in-order packets");
        NS_TEST_EXPECT_MSG_EQ(ecmp.emitted[0].ipid, 0, "ECMP keeps the receive counter");
        NS_TEST_EXPECT_MSG_EQ(ecmp.emitted[1].ipid, 1, "ECMP keeps the receive counter");
        Simulator::Destroy();
    }
};

class SpineArrivalsTest : public TestCase
{
  public:
    SpineArrivalsTest()
        : TestCase("A receiver counts arrivals by carrying spine and the folded ones apart")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        CustomHeader::ackCarriesPacketSeq = true;
        IsolatedHost receiver(LoadBalancingMode::SprayUniform, kTestSender, 8);
        // Requested spine 2 and carried by 5, then spine 6 twice, the second
        // a duplicate, and spine 1.
        receiver.ReceiveData(kTestSender, kTestReceiver, 0, SpineIdentification(2, 5));
        receiver.ReceiveData(kTestSender, kTestReceiver, 2 * kMtu, SpineIdentification(6, 6));
        receiver.ReceiveData(kTestSender, kTestReceiver, kMtu, SpineIdentification(1, 1));
        receiver.ReceiveData(kTestSender, kTestReceiver, 2 * kMtu, SpineIdentification(6, 6));
        const std::vector<SpineArrivals>& spines = receiver.hw->m_spineArrivals;
        NS_TEST_ASSERT_MSG_GT(spines.size(), 6, "every carrying spine has a count");
        const std::map<uint32_t, std::tuple<uint64_t, uint64_t, uint64_t>> expected = {
            {1, {1, kMtu, 0}},
            {5, {1, kMtu, 1}},
            {6, {2, 2 * kMtu, 0}},
        };
        for (uint32_t spine = 0; spine < spines.size(); ++spine)
        {
            const auto found = expected.find(spine);
            const auto [packets, bytes, folded] =
                found == expected.end() ? std::tuple<uint64_t, uint64_t, uint64_t>{} : found->second;
            NS_TEST_EXPECT_MSG_EQ(spines[spine].packets, packets, "arrivals by carrying spine");
            NS_TEST_EXPECT_MSG_EQ(spines[spine].payloadBytes, bytes, "payload by carrying spine");
            NS_TEST_EXPECT_MSG_EQ(spines[spine].folded, folded, "folded arrivals by carrying spine");
        }
        Ptr<RdmaRxQueuePair> flow = receiver.hw->GetRxQp(kTestReceiver.Get(),
                                                         kTestSender.Get(),
                                                         IsolatedHost::kReceiverPort,
                                                         IsolatedHost::kSenderPort,
                                                         IsolatedHost::kPriorityGroup,
                                                         false);
        NS_TEST_ASSERT_MSG_NE(flow, nullptr, "the flow has a receive queue pair");
        NS_TEST_EXPECT_MSG_EQ(flow->m_data_arrivals, 4, "the flow counts every arrival");
        NS_TEST_EXPECT_MSG_EQ(flow->m_folded_arrivals, 1, "the flow counts the folded arrival");
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;

        IsolatedHost ecmp(LoadBalancingMode::Ecmp, kTestSender);
        ecmp.ReceiveData(kTestSender, kTestReceiver, 0, SpineIdentification(2, 5));
        NS_TEST_EXPECT_MSG_EQ(ecmp.hw->m_spineArrivals.size(),
                              0,
                              "where the identification names no spine nothing is counted by one");
        NS_TEST_EXPECT_MSG_EQ(ecmp.hw->GetRxQp(kTestReceiver.Get(),
                                               kTestSender.Get(),
                                               IsolatedHost::kReceiverPort,
                                               IsolatedHost::kSenderPort,
                                               IsolatedHost::kPriorityGroup,
                                               false)
                                  ->m_data_arrivals,
                              1,
                              "the flow still counts its arrival");
        Simulator::Destroy();
    }
};

class ReorderGapTest : public TestCase
{
  public:
    ReorderGapTest()
        : TestCase("A reorder gap is acknowledged with per-packet paths and NACKed under ECMP")
    {
    }

    void DoRun() override
    {
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        CustomHeader::ackCarriesPacketSeq = true;
        IsolatedHost spray(LoadBalancingMode::SprayUniform, kTestSender, 8);
        Deliver(spray);
        NS_TEST_ASSERT_MSG_EQ(spray.emitted.size(), 4, "every packet is acknowledged");
        for (const CustomHeader& answer : spray.emitted)
        {
            NS_TEST_EXPECT_MSG_EQ(answer.l3Prot, 0xFC, "a gap draws no NACK");
        }
        NS_TEST_EXPECT_MSG_EQ(spray.emitted.back().ack.seq,
                              4000,
                              "the packets beyond the gap were held, not dropped");
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;

        IsolatedHost ecmp(LoadBalancingMode::Ecmp, kTestSender);
        Deliver(ecmp);
        NS_TEST_ASSERT_MSG_EQ(ecmp.emitted.size(),
                              3,
                              "the limiter silences the second packet beyond one gap");
        NS_TEST_EXPECT_MSG_EQ(ecmp.emitted[0].l3Prot, 0xFC, "in order is acknowledged");
        NS_TEST_EXPECT_MSG_EQ(ecmp.emitted[1].l3Prot, 0xFD, "ECMP NACKs a gap");
        NS_TEST_EXPECT_MSG_EQ(ecmp.emitted[2].ack.seq, 4000, "the filled gap releases the rest");
        Simulator::Destroy();
    }

  private:
    static void Deliver(IsolatedHost& receiver)
    {
        receiver.ReceiveData(kTestSender, kTestReceiver, 0, SpineIdentification(0, 0));
        receiver.ReceiveData(kTestSender, kTestReceiver, 2000, SpineIdentification(2, 2));
        receiver.ReceiveData(kTestSender, kTestReceiver, 3000, SpineIdentification(3, 3));
        receiver.ReceiveData(kTestSender, kTestReceiver, 1000, SpineIdentification(1, 1));
    }
};

class OutstandingPacketsModelTest : public TestCase
{
  public:
    OutstandingPacketsModelTest()
        : TestCase("Send records agree with a plain list under random sends and removals")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kSize = 100;
        constexpr uint16_t kPaths = 4;
        OutstandingPackets records;
        records.SetPacketSize(kSize);
        // The model: outstanding packets in send order, each with its path.
        std::vector<std::pair<uint64_t, uint16_t>> sent;
        std::set<uint64_t> lost;
        // Whether the latest send of each packet resent a lost send.
        std::map<uint64_t, bool> resends;
        uint64_t next = 0;
        uint64_t first = 0;
        uint64_t clock = 0;
        std::mt19937 random(1);
        auto modelRemove = [&sent](uint64_t packet) {
            for (auto it = sent.begin(); it != sent.end(); ++it)
            {
                if (it->first == packet)
                {
                    sent.erase(it);
                    return;
                }
            }
        };
        for (uint32_t step = 0; step < 20000; ++step)
        {
            const uint32_t action = random() % 10;
            if (action < 5 || sent.empty())
            {
                // A resend of a lost packet or new data, on a random path.
                uint64_t packet = next;
                const bool resend = !lost.empty() && action == 0;
                if (resend)
                {
                    packet = *lost.begin();
                    lost.erase(lost.begin());
                }
                else
                {
                    next++;
                }
                const uint16_t path = random() % kPaths;
                records.Add(packet * kSize, kSize, path, clock++);
                sent.emplace_back(packet, path);
                resends[packet] = resend;
            }
            else if (action < 9)
            {
                // Any outstanding send except, mostly, the oldest, so that a
                // hole holds the cumulative acknowledgement back and the ring
                // has to grow.
                const size_t index = 1 + random() % sent.size();
                const uint64_t packet = sent[index % sent.size()].first;
                modelRemove(packet);
                if (random() % 2)
                {
                    records.RemoveLost(packet);
                    lost.insert(packet);
                }
                else
                {
                    records.Remove(packet);
                }
            }
            else
            {
                // A cumulative advance below the oldest lost packet and the
                // next new one.
                uint64_t limit = lost.empty() ? next : std::min(*lost.begin(), next);
                if (limit > first)
                {
                    first += 1 + random() % (limit - first);
                }
                records.RemoveBelow(first * kSize);
                std::vector<std::pair<uint64_t, uint16_t>> kept;
                for (const auto& entry : sent)
                {
                    if (entry.first >= first)
                    {
                        kept.push_back(entry);
                    }
                }
                sent = kept;
            }
            NS_TEST_ASSERT_MSG_EQ(records.Bytes(), sent.size() * kSize, "bytes outstanding");
            NS_TEST_ASSERT_MSG_EQ(records.Oldest(),
                                  sent.empty() ? OutstandingPackets::kNone : sent.front().first,
                                  "the oldest send");
            if (sent.empty())
            {
                continue;
            }
            const size_t probe = random() % sent.size();
            const auto [packet, path] = sent[probe];
            NS_TEST_ASSERT_MSG_EQ(records.Find(packet * kSize, path),
                                  packet,
                                  "a send is found by sequence and path");
            NS_TEST_ASSERT_MSG_EQ(records.Find(packet * kSize, (path + 1) % kPaths),
                                  OutstandingPackets::kNone,
                                  "a send is not found along another path");
            NS_TEST_ASSERT_MSG_EQ(records.ResendsLost(packet),
                                  resends[packet],
                                  "a send is known to resend a lost send");
            uint64_t olderOnPath = OutstandingPackets::kNone;
            for (size_t i = 0; i < probe; ++i)
            {
                if (sent[i].second == path)
                {
                    olderOnPath = sent[i].first;
                }
            }
            NS_TEST_ASSERT_MSG_EQ(records.OlderOnPath(packet),
                                  olderOnPath,
                                  "the next older send on the same path");
        }
    }
};

class SelectiveTimeoutTest : public TestCase
{
  public:
    SelectiveTimeoutTest()
        : TestCase("A timeout repairs only the outstanding sends that waited it out")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, 8);
        Ptr<RdmaQueuePair> qp = sender.AddSender(kTestSender, kTestReceiver, 10 * kMtu);
        qp->snd_nxt = 3 * kMtu;
        qp->m_highest_sent = 3 * kMtu;
        for (uint16_t spine = 0; spine < 3; ++spine)
        {
            qp->m_outstanding.Add(spine * kMtu, kMtu, spine, 0);
        }
        qp->AcknowledgePacket(kMtu, 1);
        sender.hw->ArmRetransmissionTimeout(qp);
        // A later send re-arms the timer, which must still measure from the
        // oldest outstanding send.
        Simulator::Schedule(NanoSeconds(600), [&sender, qp]() {
            qp->snd_nxt = 4 * kMtu;
            qp->m_outstanding.Add(3 * kMtu, kMtu, 3, 600);
            sender.hw->ArmRetransmissionTimeout(qp);
        });
        Simulator::Stop(NanoSeconds(1500));
        Simulator::Run();

        NS_TEST_EXPECT_MSG_EQ(qp->m_timeouts, 1, "the oldest send times out once");
        NS_TEST_EXPECT_MSG_EQ(qp->m_repair_ranges.size(), 2, "two ranges are repaired");
        NS_TEST_EXPECT_MSG_EQ(qp->m_repair_ranges.count(0), 1, "the first send is repaired");
        NS_TEST_EXPECT_MSG_EQ(qp->m_repair_ranges.count(2 * kMtu),
                              1,
                              "the third send is repaired");
        NS_TEST_EXPECT_MSG_EQ(qp->RepairBytesLeft(),
                              2 * kMtu,
                              "neither the acknowledged nor the younger send is repaired");
        NS_TEST_EXPECT_MSG_EQ(qp->m_outstanding.Bytes(), kMtu, "the younger send stays outstanding");
        NS_TEST_EXPECT_MSG_EQ(qp->snd_nxt, 4 * kMtu, "a selective timeout does not rewind");
        NS_TEST_EXPECT_MSG_EQ(Simulator::GetDelayLeft(qp->m_retransmissionTimer),
                              NanoSeconds(100),
                              "the timer follows the younger send");
        Simulator::Destroy();
    }
};

class OutstandingWindowTest : public TestCase
{
  public:
    OutstandingWindowTest()
        : TestCase("The window admits new data while a head hole is outstanding")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        IsolatedHost spray(LoadBalancingMode::SprayUniform, kTestReceiver, 8);
        Ptr<RdmaQueuePair> qp = Sent(spray);
        NS_TEST_EXPECT_MSG_EQ(qp->IsWinBound(), true, "three sends fill the window");
        qp->AcknowledgePacket(kMtu, 1);
        qp->AcknowledgePacket(2 * kMtu, 2);
        NS_TEST_EXPECT_MSG_EQ(qp->IsWinBound(),
                              false,
                              "acknowledged sends free the window behind a hole");
        NS_TEST_EXPECT_MSG_EQ(qp->GetOnTheFly(), kMtu, "only the hole is in flight");
        qp->m_outstanding.Add(3 * kMtu, kMtu, 0, 0);
        qp->Acknowledge(4 * kMtu);
        NS_TEST_EXPECT_MSG_EQ(qp->GetOnTheFly(),
                              0,
                              "the cumulative acknowledgement releases what it covers");
        NS_TEST_EXPECT_MSG_EQ(qp->m_outstanding.Oldest(),
                              OutstandingPackets::kNone,
                              "no record survives below the cumulative acknowledgement");

        IsolatedHost ecmp(LoadBalancingMode::Ecmp, kTestReceiver);
        Ptr<RdmaQueuePair> cumulative = Sent(ecmp);
        NS_TEST_EXPECT_MSG_EQ(cumulative->IsWinBound(),
                              true,
                              "ECMP counts the window from the cumulative acknowledgement");
        Simulator::Destroy();
    }

  private:
    static Ptr<RdmaQueuePair> Sent(IsolatedHost& sender)
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        Ptr<RdmaQueuePair> qp = sender.AddSender(kTestSender, kTestReceiver, 10 * kMtu);
        qp->SetWin(3 * kMtu);
        qp->snd_nxt = 3 * kMtu;
        if (qp->m_outstanding.IsKept())
        {
            for (uint16_t spine = 0; spine < 3; ++spine)
            {
                qp->m_outstanding.Add(spine * kMtu, kMtu, spine, 0);
            }
        }
        return qp;
    }
};

class TrimRepairedOnceTest : public TestCase
{
  public:
    TrimRepairedOnceTest()
        : TestCase("A trimmed send is repaired once, by a fresh send with a fresh record")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        constexpr uint16_t kSpine = 3;
        const uint16_t identification = SpineIdentification(kSpine, kSpine);

        // The trim notification arrives first: the trimmed send's record goes
        // with it, so the later acknowledgement on its spine finds nothing
        // older to declare lost.
        IsolatedHost first(LoadBalancingMode::SprayUniform, kTestReceiver, 8);
        Ptr<RdmaQueuePair> qp = TwoSends(first, kSpine);
        first.ReceiveTrimNack(qp, kTestReceiver, 0, identification);
        first.ReceiveAck(kTestReceiver, 0, kMtu, identification);
        NS_TEST_EXPECT_MSG_EQ(qp->m_recovery_events, 1, "the trimmed send is repaired once");
        NS_TEST_EXPECT_MSG_EQ(qp->RepairBytesLeft(), kMtu, "one packet awaits repair");
        NS_TEST_EXPECT_MSG_EQ(qp->m_outstanding.Bytes(), 0, "neither send is outstanding");
        CustomHeader repair(CustomHeader::L2_Header | CustomHeader::L3_Header |
                            CustomHeader::L4_Header);
        first.hw->GetNxtPacket(qp)->PeekHeader(repair);
        NS_TEST_EXPECT_MSG_EQ(repair.udp.seq, 0, "the repair resends the trimmed range");
        const uint16_t path = PathOf(LoadBalancingMode::SprayUniform, repair.ipid);
        NS_TEST_EXPECT_MSG_EQ(qp->m_outstanding.Find(0, path),
                              0,
                              "the repair is recorded along the path it drew");
        NS_TEST_EXPECT_MSG_EQ(qp->RepairBytesLeft(), 0, "nothing is left to repair");

        // The send is declared lost before its trim notification arrives;
        // once its repair has left on another spine, the notification for the
        // first send asks for nothing.
        IsolatedHost second(LoadBalancingMode::SprayUniform, kTestReceiver, 8);
        qp = TwoSends(second, kSpine);
        NS_TEST_EXPECT_MSG_EQ(qp->DeclareLostSentBy(0), 2 * kMtu, "both sends are declared lost");
        uint64_t start = 0;
        NS_TEST_EXPECT_MSG_EQ(qp->TakeRepairSegment(kMtu, start), kMtu, "the loss is repaired");
        qp->m_outstanding.Add(0, kMtu, kSpine + 1, 10);
        second.ReceiveTrimNack(qp, kTestReceiver, 0, identification);
        NS_TEST_EXPECT_MSG_EQ(qp->RepairBytesLeft(),
                              kMtu,
                              "only the second send awaits repair; the first is not repaired again");
        NS_TEST_EXPECT_MSG_EQ(qp->m_recovery_events, 2, "the trim adds no recovery");
        NS_TEST_EXPECT_MSG_EQ(qp->m_trim_notifications, 1, "the trim is still counted");
        NS_TEST_EXPECT_MSG_EQ(qp->m_outstanding.Find(0, kSpine + 1),
                              0,
                              "the repair stays outstanding");
        Simulator::Destroy();
    }

  private:
    static Ptr<RdmaQueuePair> TwoSends(IsolatedHost& sender, uint16_t spine)
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        Ptr<RdmaQueuePair> qp = sender.AddSender(kTestSender, kTestReceiver, 10 * kMtu);
        qp->snd_nxt = 2 * kMtu;
        qp->m_highest_sent = 2 * kMtu;
        qp->m_outstanding.Add(0, kMtu, spine, 0);
        qp->m_outstanding.Add(kMtu, kMtu, spine, 0);
        return qp;
    }
};

class PathLossRuleTest : public TestCase
{
  public:
    PathLossRuleTest()
        : TestCase("An acknowledgement declares the older sends on its path lost, and only those")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        constexpr uint32_t kSpines = 4;
        constexpr uint32_t kPackets = 16;
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, kSpines);
        Ptr<RdmaQueuePair> qp = sender.AddSender(kTestSender, kTestReceiver, kPackets * kMtu);
        std::vector<uint16_t> identifications;
        for (uint32_t i = 0; i < kPackets; ++i)
        {
            CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header);
            sender.hw->GetNxtPacket(qp)->PeekHeader(parsed);
            identifications.push_back(parsed.ipid);
        }
        // The third send along the first packet's spine, so that two older
        // sends share its spine and later ones may too.
        const uint8_t spine = RequestedSpine(identifications[0]);
        std::vector<uint32_t> onSpine;
        for (uint32_t i = 0; i < kPackets; ++i)
        {
            if (RequestedSpine(identifications[i]) == spine)
            {
                onSpine.push_back(i);
            }
        }
        NS_TEST_ASSERT_MSG_GT(onSpine.size(), 2, "the draw puts three sends on one spine");
        const uint32_t acknowledged = onSpine[2];

        // The same packet named along another spine is a different send.
        sender.ReceiveAck(kTestReceiver,
                          0,
                          acknowledged * kMtu,
                          SpineIdentification((spine + 1) % kSpines, (spine + 1) % kSpines));
        NS_TEST_EXPECT_MSG_EQ(qp->m_recovery_events, 0, "another spine's answer declares nothing");

        // Moved by the leaf: the requested spine names the queue sequence.
        sender.ReceiveAck(kTestReceiver,
                          0,
                          acknowledged * kMtu,
                          SpineIdentification(spine, (spine + 1) % kSpines));
        NS_TEST_EXPECT_MSG_EQ(qp->m_recovery_events, 2, "the two older sends on the spine are lost");
        NS_TEST_EXPECT_MSG_EQ(qp->RepairBytesLeft(), 2 * kMtu, "exactly two sends are repaired");
        for (uint32_t i : {onSpine[0], onSpine[1]})
        {
            auto range = qp->m_repair_ranges.upper_bound(i * kMtu);
            const bool repaired = range != qp->m_repair_ranges.begin() &&
                                  std::prev(range)->second > i * kMtu;
            NS_TEST_EXPECT_MSG_EQ(repaired, true, "an older send on the spine is repaired");
        }
        NS_TEST_EXPECT_MSG_EQ(qp->m_outstanding.Bytes(),
                              (kPackets - 3) * kMtu,
                              "every other send stays outstanding");

        // A repeat of the answer finds its send gone and changes nothing.
        sender.ReceiveAck(kTestReceiver, 0, acknowledged * kMtu, identifications[acknowledged]);
        NS_TEST_EXPECT_MSG_EQ(qp->m_recovery_events, 2, "a repeated answer declares nothing");
        Simulator::Destroy();
    }
};

/**
 * An NSCC window on a path of 1000-byte packets with a 10 us base RTT and a
 * bandwidth-delay product of 1500 packets, which makes scaling_a 10: fi is
 * 50000 bytes, fi_scale 2.5, eta 1500 bytes and alpha 10/3 bytes per byte and
 * nanosecond below the target. The parameters are the transport's defaults
 * unless a test says otherwise.
 */
class NsccPath
{
  public:
    static constexpr uint32_t kMtu = 1000;
    static constexpr uint64_t kBdp = 1500 * kMtu;
    static constexpr uint64_t kBaseRtt = 10000;
    // target_qdelay, 0.75 base RTTs, and MaxWnd, 1.5 BDPs.
    static constexpr uint64_t kTarget = 7500;
    static constexpr uint64_t kCeiling = 2250000;
    // The QuickAdapt period, base RTT plus target_qdelay.
    static constexpr uint64_t kQuickAdaptPeriod = kBaseRtt + kTarget;

    static NsccWindow::Parameters Defaults()
    {
        return CreateObject<RdmaHw>()->NsccParameters();
    }

    // Defaults with no adjustment period inside a test, so that eta, which is
    // added per period whatever the acknowledgements say, moves nothing.
    static NsccWindow::Parameters WithoutEta()
    {
        NsccWindow::Parameters parameters = Defaults();
        parameters.adjust_period = 1000;
        return parameters;
    }

    static NsccWindow Started(const NsccWindow::Parameters& parameters)
    {
        NsccWindow window;
        window.Start(parameters, kMtu, kBdp, kBaseRtt, 0);
        return window;
    }

    // Acknowledgements of count packets at one instant, each with the given
    // mark and queueing delay.
    static void Acknowledge(NsccWindow& window,
                            uint32_t count,
                            bool marked,
                            uint64_t delay,
                            uint64_t now,
                            uint64_t inflight = 0)
    {
        for (uint32_t i = 0; i < count; ++i)
        {
            window.OnAck(kMtu, marked, kBaseRtt + delay, inflight, now);
        }
    }
};

class NsccCaseTableTest : public TestCase
{
  public:
    NsccCaseTableTest()
        : TestCase("NSCC moves the window by the case of mark and queueing delay")
    {
    }

    void DoRun() override
    {
        constexpr uint64_t kWindow = 1000000;
        // Nine acknowledgements, so that the ninth applies what the cases
        // accumulated, two base RTTs in, so that a decrease is allowed.
        constexpr uint64_t kNow = 2 * NsccPath::kBaseRtt;
        auto after = [](bool marked, uint64_t delay) {
            NsccWindow window = NsccPath::Started(NsccPath::WithoutEta());
            window.OnLoss(NsccPath::kCeiling - kWindow);
            NsccPath::Acknowledge(window, 9, marked, delay, kNow);
            return window.Cwnd();
        };
        constexpr uint64_t kLow = 2000;
        constexpr uint64_t kHigh = 2 * NsccPath::kTarget;
        // Proportional increase: 9 * (10/3) * 1000 * (7500 - 2000) over the window.
        NS_TEST_EXPECT_MSG_EQ_TOL(after(false, kLow),
                                  kWindow + 165,
                                  1,
                                  "no mark at low delay increases in proportion to the headroom");
        // Fair increase: 9 * 50000 * 1000 over the window.
        NS_TEST_EXPECT_MSG_EQ_TOL(after(false, kHigh),
                                  kWindow + 450,
                                  1,
                                  "no mark at high delay increases by the fair constant");
        // Multiplicative decrease once: 1 - 0.8 * (15000 - 7500) / 15000.
        NS_TEST_EXPECT_MSG_EQ_TOL(after(true, kHigh),
                                  kWindow * 6 / 10,
                                  1,
                                  "a mark at high delay decreases by the delay's excess");
        NS_TEST_EXPECT_MSG_EQ(after(true, kLow), kWindow, "a mark at low delay changes nothing");
    }
};

class NsccLightMarkTest : public TestCase
{
  public:
    NsccLightMarkTest()
        : TestCase("NSCC leaves the window alone on a mark below the target delay")
    {
    }

    void DoRun() override
    {
        NsccWindow window = NsccPath::Started(NsccPath::WithoutEta());
        window.OnLoss(NsccPath::kCeiling / 2);
        // An unmarked packet three targets late raises the average delay well
        // past the target, so only the answered packet's own delay can spare
        // the window from the marks that follow it.
        uint64_t now = 2 * NsccPath::kBaseRtt;
        window.OnAck(NsccPath::kMtu, false, NsccPath::kBaseRtt + 3 * NsccPath::kTarget, 0, now);
        const uint64_t before = window.Cwnd();
        for (uint32_t i = 0; i < 6; ++i)
        {
            window.OnAck(NsccPath::kMtu,
                         true,
                         NsccPath::kBaseRtt + NsccPath::kTarget - 1,
                         0,
                         ++now);
            NS_TEST_EXPECT_MSG_EQ(window.Cwnd(),
                                  before,
                                  "a mark one nanosecond under the target is left to load balancing");
        }
        window.OnAck(NsccPath::kMtu, true, NsccPath::kBaseRtt + NsccPath::kTarget, 0, ++now);
        NS_TEST_EXPECT_MSG_LT(window.Cwnd(), before, "a mark at the target decreases");
    }
};

class NsccQuickAdaptTest : public TestCase
{
  public:
    NsccQuickAdaptTest()
        : TestCase("NSCC QuickAdapt sets the window once its period has run and its gate is open")
    {
    }

    void DoRun() override
    {
        constexpr uint64_t kMtu = NsccPath::kMtu;
        constexpr uint64_t kTrim = 1000;
        constexpr uint64_t kEnd = kTrim + NsccPath::kQuickAdaptPeriod;
        constexpr uint64_t kInflight = 40 * kMtu;
        // The first trim opens the first period and is cut by its own size.
        NsccWindow window = NsccPath::Started(NsccPath::WithoutEta());
        window.OnTrim(kMtu, NsccWindow::kNoRtt, kInflight, kTrim);
        const uint64_t trimmed = NsccPath::kCeiling - kMtu;
        NS_TEST_ASSERT_MSG_EQ(window.Cwnd(), trimmed, "the trim cuts its own bytes");
        // Marks at low delay leave the window alone while the period runs.
        for (uint64_t now = 2000; now <= 11000; now += 1000)
        {
            NsccPath::Acknowledge(window, 1, true, 2000, now, kInflight);
        }
        NsccPath::Acknowledge(window, 1, true, 2000, kEnd - 1, kInflight);
        NS_TEST_EXPECT_MSG_EQ(window.Cwnd(), trimmed, "QuickAdapt waits out its period");
        NsccPath::Acknowledge(window, 1, true, 2000, kEnd, kInflight);
        NS_TEST_EXPECT_MSG_EQ(window.Cwnd(),
                              12 * kMtu,
                              "QuickAdapt sets the window to the bytes acknowledged in the period");
        // The marks on the bytes in flight when the window was set are
        // ignored: a decrease is otherwise due, three targets late and well
        // past a base RTT since the last.
        constexpr uint64_t kLate = 3 * NsccPath::kTarget;
        for (uint64_t i = 0; i + 1 < kInflight / kMtu; ++i)
        {
            NsccPath::Acknowledge(window, 1, true, kLate, 30000 + i, kInflight);
        }
        NS_TEST_EXPECT_MSG_EQ(window.Cwnd(), 12 * kMtu, "the bytes in flight are ignored");
        NsccPath::Acknowledge(window, 1, true, kLate, 30000 + kInflight / kMtu, kInflight);
        NS_TEST_EXPECT_MSG_EQ(window.Cwnd(),
                              6 * kMtu,
                              "past them, a mark at high delay decreases again");

        // MaxWnd / 2^3 bytes acknowledged in the period close the gate.
        NsccWindow open = NsccPath::Started(NsccPath::WithoutEta());
        open.OnTrim(kMtu, NsccWindow::kNoRtt, kInflight, kTrim);
        for (uint64_t i = 0; i < (NsccPath::kCeiling >> 3) / kMtu + 1; ++i)
        {
            NsccPath::Acknowledge(open, 1, true, 2000, 2000 + i * 50, kInflight);
        }
        NsccPath::Acknowledge(open, 1, true, 2000, kEnd, kInflight);
        NS_TEST_EXPECT_MSG_EQ(open.Cwnd(),
                              trimmed,
                              "a flow that delivered an eighth of MaxWnd keeps its window");

        // Without a trim, a delay past qa_threshold target delays triggers it.
        NsccWindow::Parameters delayed = NsccPath::WithoutEta();
        delayed.qa_threshold = 4;
        constexpr uint64_t kThreshold = 4 * NsccPath::kTarget;
        for (const uint64_t delay : {kThreshold, kThreshold + 1})
        {
            // The first acknowledgement opens the period, and three more
            // arrive inside it.
            NsccWindow late = NsccPath::Started(delayed);
            NsccPath::Acknowledge(late, 1, true, 2000, kTrim, kInflight);
            NsccPath::Acknowledge(late, 3, true, 2000, kTrim + 1, kInflight);
            NsccPath::Acknowledge(late, 1, false, delay, kEnd, kInflight);
            const bool adapted = late.Cwnd() == 4 * kMtu;
            const bool past = delay > kThreshold;
            NS_TEST_EXPECT_MSG_EQ(adapted, past, "a delay past qa_threshold alone sets the window");
        }
    }
};

class NsccFastIncreaseTest : public TestCase
{
  public:
    NsccFastIncreaseTest()
        : TestCase("NSCC fast increase takes over after a window of clean acknowledgements")
    {
    }

    void DoRun() override
    {
        constexpr uint64_t kMtu = NsccPath::kMtu;
        constexpr uint64_t kWindow = 3 * kMtu;
        // fi_scale is 2.5, so each clean packet adds two and a half.
        constexpr uint64_t kStep = 2500;
        NsccWindow window = NsccPath::Started(NsccPath::WithoutEta());
        window.OnLoss(NsccPath::kCeiling - kWindow);
        uint64_t now = 2 * NsccPath::kBaseRtt;
        for (uint32_t i = 0; i < 3; ++i)
        {
            NsccPath::Acknowledge(window, 1, false, 0, ++now);
            NS_TEST_EXPECT_MSG_EQ(window.Cwnd(), kWindow, "up to a window of clean bytes waits");
        }
        NsccPath::Acknowledge(window, 1, false, 0, ++now);
        NS_TEST_EXPECT_MSG_EQ(window.Cwnd(), kWindow + kStep, "more than a window starts it");
        NsccPath::Acknowledge(window, 1, false, 0, ++now);
        NS_TEST_EXPECT_MSG_EQ(window.Cwnd(), kWindow + 2 * kStep, "once started it continues");
        // A microsecond of queueing ends it, and the count starts over.
        NsccPath::Acknowledge(window, 1, false, 1000, ++now);
        NS_TEST_EXPECT_MSG_EQ(window.Cwnd(), kWindow + 2 * kStep, "a queue ends it");
        NsccPath::Acknowledge(window, 1, false, 0, ++now);
        NS_TEST_EXPECT_MSG_EQ(window.Cwnd(),
                              kWindow + 2 * kStep,
                              "after a queue a window of clean bytes is needed again");
        // Enough clean packets to climb by fast increase to MaxWnd and stay.
        for (uint32_t i = 0; i < NsccPath::kCeiling / kStep + 100; ++i)
        {
            NsccPath::Acknowledge(window, 1, false, 0, ++now);
            NS_TEST_ASSERT_MSG_LT_OR_EQ(window.Cwnd(),
                                        NsccPath::kCeiling,
                                        "fast increase stops at MaxWnd");
        }
        NS_TEST_EXPECT_MSG_EQ(window.Cwnd(), NsccPath::kCeiling, "fast increase reaches MaxWnd");
    }
};

class NsccCutTest : public TestCase
{
  public:
    NsccCutTest()
        : TestCase("NSCC cuts the window by a trimmed send and by each send declared lost, "
                   "and reports each change once")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        const uint16_t spine = SpineIdentification(3, 3);
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, 8);
        sender.hw->m_retransmission_timeout_ns = 0;
        // Every change of the window, as the NsccWindow trace source reports it.
        Changes changes;
        NS_TEST_ASSERT_MSG_EQ(
            sender.hw->TraceConnectWithoutContext(
                "NsccWindow",
                MakeBoundCallback(&NsccCutTest::Record, &changes)),
            true,
            "RdmaHw offers the NsccWindow trace source");
        Ptr<RdmaQueuePair> qp = Started(sender, 4);
        for (uint32_t i = 0; i < 3; ++i)
        {
            qp->m_outstanding.Add(i * kMtu, kMtu, 3, 0);
        }
        qp->m_outstanding.Add(3 * kMtu, kMtu, 4, 0);
        Simulator::Schedule(NanoSeconds(12000), [this, &sender, qp, spine]() {
            sender.ReceiveTrimNack(qp, kTestReceiver, 0, spine);
            NS_TEST_EXPECT_MSG_EQ(qp->GetWin(),
                                  NsccPath::kCeiling - kMtu,
                                  "a trim cuts the window the NIC enforces by its bytes");
        });
        // Marked, three microseconds late: the acknowledgement itself changes
        // nothing, and the send before it on its spine is declared lost.
        Simulator::Schedule(NanoSeconds(13000), [this, &sender, qp, spine]() {
            sender.ReceiveAck(kTestReceiver, 0, 2 * kMtu, spine, true);
            NS_TEST_EXPECT_MSG_EQ(qp->m_recovery_events, 2, "the trim and one loss are repaired");
            NS_TEST_EXPECT_MSG_EQ(qp->GetWin(),
                                  NsccPath::kCeiling - 2 * kMtu,
                                  "a send declared lost cuts the window by its bytes");
            // The same acknowledgement again matches no send, and a marked
            // acknowledgement below the target delay on another spine is
            // read but moves nothing: neither is reported.
            sender.ReceiveAck(kTestReceiver, 0, 2 * kMtu, spine, true);
            sender.ReceiveAck(kTestReceiver, 0, 3 * kMtu, SpineIdentification(4, 4), true);
            NS_TEST_EXPECT_MSG_EQ(qp->m_outstanding.Bytes(), 0, "every send is answered");
            NS_TEST_EXPECT_MSG_EQ(qp->GetWin(),
                                  NsccPath::kCeiling - 2 * kMtu,
                                  "a mark below the target leaves the window");
        });
        Simulator::Run();
        Simulator::Destroy();
        const Changes expected{{qp, 0, NsccPath::kCeiling},
                               {qp, NsccPath::kCeiling, NsccPath::kCeiling - kMtu},
                               {qp, NsccPath::kCeiling - kMtu, NsccPath::kCeiling - 2 * kMtu}};
        NS_TEST_EXPECT_MSG_EQ((changes == expected),
                              true,
                              "the trace reports the start, the trim and the loss, once each");

        IsolatedHost timed(LoadBalancingMode::SprayUniform, kTestReceiver, 8);
        Ptr<RdmaQueuePair> waiting = Started(timed, 1);
        waiting->m_outstanding.Add(0, kMtu, 3, 0);
        timed.hw->ArmRetransmissionTimeout(waiting);
        Simulator::Stop(NanoSeconds(IsolatedHost::kTimeoutNs + 1));
        Simulator::Run();
        NS_TEST_EXPECT_MSG_EQ(waiting->m_timeouts, 1, "the send times out");
        NS_TEST_EXPECT_MSG_EQ(waiting->GetWin(),
                              NsccPath::kCeiling - kMtu,
                              "a send lost to the timeout cuts the window by its bytes");
        Simulator::Destroy();
    }

  private:
    using Changes = std::vector<std::tuple<Ptr<RdmaQueuePair>, uint64_t, uint64_t>>;

    static void Record(Changes* changes, Ptr<RdmaQueuePair> qp, uint64_t from, uint64_t to)
    {
        changes->emplace_back(qp, from, to);
    }

    static Ptr<RdmaQueuePair> Started(IsolatedHost& sender, uint32_t packets)
    {
        sender.hw->m_cc_mode = 11;
        sender.hw->m_nscc_adjust_period = 1000;
        Ptr<RdmaQueuePair> qp =
            sender.AddSender(kTestSender, kTestReceiver, 10 * IsolatedHost::kMtu);
        qp->snd_nxt = packets * IsolatedHost::kMtu;
        qp->m_highest_sent = qp->snd_nxt;
        qp->nscc.Start(sender.hw->NsccParameters(),
                       IsolatedHost::kMtu,
                       NsccPath::kBdp,
                       NsccPath::kBaseRtt,
                       0);
        sender.hw->ApplyNsccWindow(qp);
        return qp;
    }
};

class NsccBoundsTest : public TestCase
{
  public:
    NsccBoundsTest()
        : TestCase("NSCC keeps the window between one packet and MaxWnd")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kMtu = NsccPath::kMtu;
        NsccWindow window = NsccPath::Started(NsccPath::Defaults());
        std::mt19937 random(1);
        uint64_t now = 0;
        uint64_t atFloor = 0;
        uint64_t atCeiling = 0;
        for (uint32_t step = 0; step < 50000; ++step)
        {
            // Alternating stretches of a clear path and of congestion, so that
            // both bounds are pressed.
            const bool congested = step / 2000 % 2 == 1;
            now += random() % 2000;
            const uint64_t inflight = random() % (2 * NsccPath::kBdp);
            const uint32_t event = random() % 100;
            if (congested && event < 10)
            {
                window.OnTrim(kMtu, NsccPath::kBaseRtt + random() % 30000, inflight, now);
            }
            else if (congested && event < 15)
            {
                window.OnLoss((1 + random() % 50) * kMtu);
            }
            else
            {
                const bool marked = congested ? random() % 2 == 0 : random() % 50 == 0;
                const uint64_t delay = congested ? random() % 30000 : random() % 1500;
                window.OnAck(kMtu, marked, NsccPath::kBaseRtt + delay, inflight, now);
            }
            NS_TEST_ASSERT_MSG_GT_OR_EQ(window.Cwnd(), kMtu, "the window holds a packet");
            NS_TEST_ASSERT_MSG_LT_OR_EQ(window.Cwnd(), window.MaxWnd(), "the window stays under MaxWnd");
            atFloor += window.Cwnd() == kMtu;
            atCeiling += window.Cwnd() == window.MaxWnd();
        }
        NS_TEST_EXPECT_MSG_GT(atFloor, 0, "the floor was pressed");
        NS_TEST_EXPECT_MSG_GT(atCeiling, 0, "the ceiling was pressed");
    }
};

class NsccRttTest : public TestCase
{
  public:
    NsccRttTest()
        : TestCase("NSCC times an acknowledgement from the answered send's record")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, 8);
        sender.hw->m_cc_mode = 11;
        sender.hw->m_retransmission_timeout_ns = 0;
        Ptr<RdmaQueuePair> qp = sender.AddSender(kTestSender, kTestReceiver, 10 * kMtu);
        qp->snd_nxt = 4 * kMtu;
        qp->m_highest_sent = 4 * kMtu;
        qp->nscc.Start(sender.hw->NsccParameters(), kMtu, NsccPath::kBdp, NsccPath::kBaseRtt, 0);
        // Sends on four spines at 50, 60, 100 and 400 ns. The first is declared
        // lost and the second trimmed, and each is sent again on its spine.
        qp->m_outstanding.Add(2 * kMtu, kMtu, 4, 50);
        qp->m_outstanding.Add(3 * kMtu, kMtu, 5, 60);
        qp->m_outstanding.Add(0, kMtu, 1, 100);
        qp->m_outstanding.Add(kMtu, kMtu, 2, 400);
        NS_TEST_ASSERT_MSG_EQ(qp->DeclareLostSentBy(50), kMtu, "the first send is lost");
        qp->m_outstanding.Add(2 * kMtu, kMtu, 4, 4900);
        NS_TEST_ASSERT_MSG_EQ(qp->ReleasePacket(3 * kMtu, 5), true, "the second send is trimmed");
        qp->m_outstanding.Add(3 * kMtu, kMtu, 5, 5000);
        Simulator::Schedule(NanoSeconds(5000), [this, &sender, qp]() {
            sender.ReceiveAck(kTestReceiver, 0, kMtu, SpineIdentification(2, 2));
            NS_TEST_EXPECT_MSG_EQ(qp->nscc.BaseRtt(),
                                  4600,
                                  "the round trip runs from the answered send's own send time");
            sender.ReceiveAck(kTestReceiver, 0, 2 * kMtu, SpineIdentification(4, 4));
            NS_TEST_EXPECT_MSG_EQ(qp->nscc.BaseRtt(),
                                  4600,
                                  "the resend of a lost send, which the lost send's late "
                                  "answer would also name, is not timed");
        });
        Simulator::Schedule(NanoSeconds(5100), [this, &sender, qp]() {
            sender.ReceiveAck(kTestReceiver, 0, 0, SpineIdentification(1, 1));
            NS_TEST_EXPECT_MSG_EQ(qp->nscc.BaseRtt(), 4600, "a longer round trip keeps the base");
            NS_TEST_EXPECT_MSG_EQ(qp->nscc.MaxWnd(),
                                  NsccPath::kCeiling * 4600 / NsccPath::kBaseRtt,
                                  "MaxWnd follows the base RTT");
        });
        Simulator::Schedule(NanoSeconds(5200), [this, &sender, qp]() {
            sender.ReceiveAck(kTestReceiver, 0, 3 * kMtu, SpineIdentification(5, 5));
            NS_TEST_EXPECT_MSG_EQ(qp->nscc.BaseRtt(),
                                  200,
                                  "the resend of a trimmed send, which only it can answer, is timed");
        });
        Simulator::Run();
        Simulator::Destroy();
    }
};

/**
 * \brief TestSuite for PointToPoint module
 */
class PointToPointTestSuite : public TestSuite
{
  public:
    /**
     * \brief Constructor
     */
    PointToPointTestSuite();
};

PointToPointTestSuite::PointToPointTestSuite()
    : TestSuite("devices-point-to-point", Type::UNIT)
{
    AddTestCase(new PointToPointTest, TestCase::Duration::QUICK);
    AddTestCase(new UecTrimHeaderTest, TestCase::Duration::QUICK);
    AddTestCase(new UecTrimSwitchTest(PacketTrimMode::ForwardToDestination),
                TestCase::Duration::QUICK);
    AddTestCase(new UecTrimSwitchTest(PacketTrimMode::BackToSender),
                TestCase::Duration::QUICK);
    AddTestCase(new UecTrimSwitchTest(PacketTrimMode::BackToSender, false),
                TestCase::Duration::QUICK);
    AddTestCase(new UecTrimRecoveryTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineIdentificationTest, TestCase::Duration::QUICK);
    AddTestCase(new LoadBalancingSwitchTest, TestCase::Duration::QUICK);
    AddTestCase(new PortDownBufferTest, TestCase::Duration::QUICK);
    AddTestCase(new LinkErrorTest, TestCase::Duration::QUICK);
    AddTestCase(new BlackholeTest, TestCase::Duration::QUICK);
    AddTestCase(new LinkRateChangeTest, TestCase::Duration::QUICK);
    AddTestCase(new PortCountersTest, TestCase::Duration::QUICK);
    AddTestCase(new LoadBalancingSenderTest, TestCase::Duration::QUICK);
    AddTestCase(new AckPacketSeqHeaderTest, TestCase::Duration::QUICK);
    AddTestCase(new AckNamesPacketTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineArrivalsTest, TestCase::Duration::QUICK);
    AddTestCase(new ReorderGapTest, TestCase::Duration::QUICK);
    AddTestCase(new OutstandingPacketsModelTest, TestCase::Duration::QUICK);
    AddTestCase(new SelectiveTimeoutTest, TestCase::Duration::QUICK);
    AddTestCase(new OutstandingWindowTest, TestCase::Duration::QUICK);
    AddTestCase(new TrimRepairedOnceTest, TestCase::Duration::QUICK);
    AddTestCase(new PathLossRuleTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccCaseTableTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccLightMarkTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccQuickAdaptTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccFastIncreaseTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccCutTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccBoundsTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccRttTest, TestCase::Duration::QUICK);
}

static PointToPointTestSuite g_pointToPointTestSuite; //!< The testsuite
