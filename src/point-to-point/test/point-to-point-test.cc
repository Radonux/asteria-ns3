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
#include "ns3/seq-ts-header.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/udp-header.h"

#include <algorithm>
#include <random>
#include <set>
#include <string>
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

        // With spine 1 down the live spines are 0, 2 and 3; request 1 folds
        // onto the second of them.
        DynamicCast<QbbNetDevice>(m_leaf->GetDevice(m_spinePorts[1]))->TakeDown();
        const Egress moved = Send(m_remoteHost, 10000, SpineIdentification(1, 1));
        NS_TEST_EXPECT_MSG_EQ(moved.port,
                              m_spinePorts[2],
                              "a request for a dead uplink is moved to a live one");
        NS_TEST_EXPECT_MSG_EQ(RequestedSpine(moved.identification),
                              1,
                              "the request survives the move");
        NS_TEST_EXPECT_MSG_EQ(CarryingSpine(moved.identification),
                              2,
                              "the leaf records the spine that carries the packet");
        const Egress again = Send(m_remoteHost, 10000, moved.identification);
        NS_TEST_EXPECT_MSG_EQ(again.port,
                              m_spinePorts[2],
                              "a packet that passes the leaf again keeps its spine");
        const Egress live = Send(m_remoteHost, 10000, SpineIdentification(3, 3));
        NS_TEST_EXPECT_MSG_EQ(live.port,
                              m_spinePorts[3],
                              "requests for live uplinks are unaffected by a dead one");
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
    void ReceiveAck(Ipv4Address from, uint32_t cumulative, uint32_t seq, uint16_t identification)
    {
        CustomHeader ack;
        ack.l3Prot = 0xFC;
        ack.sip = from.Get();
        ack.ipid = identification;
        ack.ack.flags = 0;
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
                // A resend of a removed packet or new data, on a random path.
                uint64_t packet = next;
                if (!lost.empty() && action == 0)
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
            }
            else if (action < 9)
            {
                // Any outstanding send except, mostly, the oldest, so that a
                // hole holds the cumulative acknowledgement back and the ring
                // has to grow.
                const size_t index = 1 + random() % sent.size();
                const uint64_t packet = sent[index % sent.size()].first;
                records.Remove(packet);
                modelRemove(packet);
                if (random() % 2)
                {
                    lost.insert(packet);
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
        NS_TEST_EXPECT_MSG_EQ(qp->DeclareLostSentBy(0), 2, "both sends are declared lost");
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
    AddTestCase(new LoadBalancingSenderTest, TestCase::Duration::QUICK);
    AddTestCase(new AckPacketSeqHeaderTest, TestCase::Duration::QUICK);
    AddTestCase(new AckNamesPacketTest, TestCase::Duration::QUICK);
    AddTestCase(new ReorderGapTest, TestCase::Duration::QUICK);
    AddTestCase(new OutstandingPacketsModelTest, TestCase::Duration::QUICK);
    AddTestCase(new SelectiveTimeoutTest, TestCase::Duration::QUICK);
    AddTestCase(new OutstandingWindowTest, TestCase::Duration::QUICK);
    AddTestCase(new TrimRepairedOnceTest, TestCase::Duration::QUICK);
}

static PointToPointTestSuite g_pointToPointTestSuite; //!< The testsuite
