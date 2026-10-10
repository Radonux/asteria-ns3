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
#include "ns3/switch-mmu.h"
#include "ns3/switch-node.h"
#include "ns3/node.h"
#include "ns3/nscc-window.h"
#include "ns3/path-selector.h"
#include "ns3/seq-ts-header.h"
#include "ns3/simulator.h"
#include "ns3/spine-grader.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/udp-header.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
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
        NS_TEST_EXPECT_MSG_EQ(m_repairMarked, false, "an unmarked trim's NACK carries no mark");

        trimmed.m_tos = kUetDscpTrimmed << 2 | Ipv4Header::ECN_CE;
        receiverHw->Receive(Create<Packet>(), trimmed);
        NS_TEST_EXPECT_MSG_EQ(m_repairCount, 2, "every trimmed packet produces a NACK");
        NS_TEST_EXPECT_MSG_EQ(m_repairMarked, true, "a NACK echoes the trimmed packet's CE");
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
        m_repairMarked = (parsed.ack.flags >> qbbHeader::FLAG_CNP) & 1;
        NS_TEST_ASSERT_MSG_EQ(queue, 0, "trim repair must use strict-priority control");
    }

    bool m_repairMarked = false;
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
        for (LoadBalancingMode mode :
             {LoadBalancingMode::SprayUniform, LoadBalancingMode::SprayPolicy})
        {
            NS_TEST_EXPECT_MSG_EQ(
                PathOf(mode, SpineIdentification(7, 3)),
                7,
                "where the identification names a spine, the path is the request");
        }
        NS_TEST_EXPECT_MSG_EQ(PathOf(LoadBalancingMode::EntropyHash, 0x0703),
                              0x0703,
                              "an entropy value is its own path");
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
        : TestCase("A leaf routes data, and the answers to it, by the requested spine or by "
                   "the entropy value")
    {
    }

    void DoRun() override
    {
        for (LoadBalancingMode mode :
             {LoadBalancingMode::SprayUniform, LoadBalancingMode::SprayPolicy})
        {
            RouteBySpine(mode);
            ReturnOverCarryingSpine(mode);
        }
        RouteByEntropy(LoadBalancingMode::Ecmp);
        RouteByEntropy(LoadBalancingMode::EntropyHash);
        ProbeAsData();
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = 4;
    // An acknowledgement, a NACK and a repair request, which answer a data
    // packet and return its identification.
    static constexpr uint8_t kAnswers[] = {0xFC, 0xFD, kUecTrimRepairProtocol};
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
        uint32_t queue;
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
                [this, index](Ptr<const Packet> packet, uint32_t queue) {
                    CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header);
                    packet->PeekHeader(parsed);
                    m_egress.push_back({index, parsed.ipid, queue});
                }));
        return index;
    }

    // A data packet, a path probe, which travels as its flow's data does, or
    // under another protocol the control packet a host behind this leaf sends
    // to answer one, with its ports the other way round. Priority group 0
    // bypasses buffer admission, which is not under test.
    Egress Send(Ipv4Address destination,
                uint16_t sourcePort,
                uint16_t identification,
                uint8_t protocol = 0x11,
                uint16_t pg = 0)
    {
        Ptr<Packet> packet = Create<Packet>(protocol == 0x11 ? 1000 : 0);
        if (protocol == 0x11)
        {
            SeqTsHeader seqTs;
            seqTs.SetPG(pg);
            packet->AddHeader(seqTs);
            UdpHeader udp;
            udp.SetSourcePort(sourcePort);
            udp.SetDestinationPort(10001);
            packet->AddHeader(udp);
        }
        else
        {
            const bool forward = protocol == kPathProbeProtocol;
            qbbHeader control;
            control.SetPG(pg);
            control.SetSport(forward ? sourcePort : 10001);
            control.SetDport(forward ? 10001 : sourcePort);
            packet->AddHeader(control);
        }
        Ipv4Header ip;
        ip.SetSource(m_sender);
        ip.SetDestination(destination);
        ip.SetProtocol(protocol);
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

    void RouteBySpine(LoadBalancingMode mode)
    {
        BuildLeaf(mode);
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

    // The same leaf stands for the destination leaf of the data it routes: its
    // spine ports are indexed as every leaf's are, so the control packet a host
    // behind it sends in answer leaves on the port of the spine that carried
    // the data.
    void ReturnOverCarryingSpine(LoadBalancingMode mode)
    {
        BuildLeaf(mode);
        m_leaf->SetAttribute("AckHighPrio", UintegerValue(1));
        for (uint8_t protocol : kAnswers)
        {
            for (uint32_t spine = 0; spine < kSpines; ++spine)
            {
                const Egress data = Send(m_remoteHost, 10000, SpineIdentification(spine, spine));
                const Egress answer = Send(m_remoteHost, 10000, data.identification, protocol);
                NS_TEST_EXPECT_MSG_EQ(answer.port,
                                      data.port,
                                      "an answer leaves on the carrying spine's uplink");
                NS_TEST_EXPECT_MSG_EQ(answer.identification,
                                      data.identification,
                                      "an answer's identification is not rewritten");
            }
        }

        // Spine 2 cannot reach this host, so its request was carried by another
        // spine, which the answer takes although spine 2 reaches the remote host.
        const Egress folded = Send(m_cutOffHost, 10000, SpineIdentification(2, 2));
        const Egress answer = Send(m_remoteHost, 10000, folded.identification, 0xFC);
        NS_TEST_EXPECT_MSG_EQ(answer.port,
                              folded.port,
                              "an answer follows the carrying spine, not the requested one");
        NS_TEST_EXPECT_MSG_EQ(answer.identification,
                              folded.identification,
                              "a folded data packet's answer keeps both spines");

        DynamicCast<QbbNetDevice>(m_leaf->GetDevice(m_spinePorts[3]))->TakeDown();
        const Egress moved = Send(m_remoteHost, 10000, SpineIdentification(3, 3), 0xFC);
        NS_TEST_EXPECT_MSG_NE(moved.port,
                              m_spinePorts[3],
                              "an answer whose carrying spine's uplink is down is moved");
        NS_TEST_EXPECT_MSG_EQ(moved.identification,
                              SpineIdentification(3, 3),
                              "a moved answer keeps the identification it echoes");
        NS_TEST_EXPECT_MSG_EQ(Send(m_remoteHost, 10000, SpineIdentification(3, 3), 0xFC).port,
                              moved.port,
                              "a moved answer keeps one spine while the live ones stay");
    }

    // A probe of an entropy value takes the path data with that value takes,
    // and waits in the queue of its priority group.
    void ProbeAsData()
    {
        BuildLeaf(LoadBalancingMode::EntropyHash);
        for (uint32_t identification = 0; identification <= UINT8_MAX; ++identification)
        {
            NS_TEST_EXPECT_MSG_EQ(
                Send(m_remoteHost, 10000, identification, kPathProbeProtocol).port,
                Send(m_remoteHost, 10000, identification).port,
                "a probe takes its data's path");
        }
        m_leaf->m_mmu->ConfigNPort(m_leaf->GetNDevices() - 1);
        m_leaf->m_mmu->ConfigBufferSize(32 * 1024 * 1024);
        NS_TEST_EXPECT_MSG_EQ(Send(m_remoteHost, 10000, 5, kPathProbeProtocol, 3).queue,
                              3,
                              "a probe waits with its priority group's data");
    }

    void RouteByEntropy(LoadBalancingMode mode)
    {
        BuildLeaf(mode);
        for (uint8_t protocol : {uint8_t{0x11}, kAnswers[0], kAnswers[1], kAnswers[2]})
        {
            std::set<uint32_t> ports;
            for (uint32_t identification = 0; identification <= UINT8_MAX; ++identification)
            {
                ports.insert(Send(m_remoteHost, 10000, identification, protocol).port);
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
                const uint32_t port = Send(m_remoteHost, 10000, 0x1234, protocol).port;
                NS_TEST_EXPECT_MSG_EQ(Send(m_remoteHost, 10000, 0x1234, protocol).port,
                                      port,
                                      "an entropy value always takes the same path");
            }
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
        : TestCase("A black-holed port drops the packets arriving on it without notice")
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
        NS_TEST_EXPECT_MSG_EQ(sentToB, 1, "an acknowledgement from A is lost with the data");
        NS_TEST_ASSERT_MSG_EQ(drops.size(), 3, "the acknowledgement is dropped");
        NS_TEST_EXPECT_MSG_EQ(drops.back(),
                              static_cast<uint32_t>(SwitchDropReason::Blackhole),
                              "the acknowledgement's drop is the port's");
        NS_TEST_EXPECT_MSG_EQ(fromA->IsLinkUp(), true, "the link stays up");

        ArriveAtSwitch(spine, fromB, hostB, hostA, 0);
        ArriveAtSwitch(spine, fromB, hostB, hostA, 0, 0xFC);
        NS_TEST_EXPECT_MSG_EQ(sentToA, 3, "the spine still forwards toward A");
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
            sw->m_mmu->ConfigEcn(port->GetIfIndex(), 0, 0, 1.0, port->GetIfIndex());
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

/**
 * One switch port held halfway between its ECN thresholds, where it marks with
 * probability one half, asked the same sequence of marking decisions alone and
 * amid the draws of another port and of automatically streamed variables.
 */
class MarkingStreamTest : public TestCase
{
  public:
    MarkingStreamTest()
        : TestCase("A port draws its marks from its own stream, whatever else is drawn")
    {
    }

    void DoRun() override
    {
        RngSeedManager::SetSeed(1);
        RngSeedManager::SetRun(1);
        // Reading the next automatic stream takes it, so with no automatic
        // stream taken between them two readings differ by one.
        const uint64_t before = RngSeedManager::GetNextStreamIndex();
        const std::vector<bool> alone = Marks(false);
        const uint64_t after = RngSeedManager::GetNextStreamIndex();
        NS_TEST_EXPECT_MSG_EQ(after, before + 1, "marking takes no automatic stream");
        const std::vector<bool> amid = Marks(true);
        NS_TEST_EXPECT_MSG_EQ((alone == amid), true, "the same occupancies draw the same marks");
        // 1000 expected with a standard deviation of 22.
        const auto marked = std::count(alone.begin(), alone.end(), true);
        NS_TEST_EXPECT_MSG_GT(marked, 900, "the port marks at its probability");
        NS_TEST_EXPECT_MSG_LT(marked, 1100, "the port marks at its probability");
        Simulator::Destroy();
    }

  private:
    static std::vector<bool> Marks(bool amidOtherDraws)
    {
        constexpr uint32_t kPort = 1;
        constexpr uint32_t kOtherPort = 2;
        constexpr uint32_t kQueue = 3;
        Ptr<SwitchMmu> mmu = CreateObject<SwitchMmu>();
        mmu->ConfigEcn(kPort, 0, 100, 1.0, 11);
        mmu->ConfigEcn(kOtherPort, 0, 100, 1.0, 12);
        mmu->egress_bytes[kPort][kQueue] = 50000;
        mmu->egress_bytes[kOtherPort][kQueue] = 50000;
        std::vector<bool> marks;
        for (uint32_t i = 0; i < 2000; ++i)
        {
            if (amidOtherDraws)
            {
                mmu->ShouldSendCN(kOtherPort, kQueue);
                CreateObject<UniformRandomVariable>()->GetValue();
            }
            marks.push_back(mmu->ShouldSendCN(kPort, kQueue));
        }
        return marks;
    }
};

class PfcIdentificationTest : public TestCase
{
  public:
    PfcIdentificationTest()
        : TestCase("A device draws its PFC frame identifications from its fixed stream")
    {
    }

    void DoRun() override
    {
        RngSeedManager::SetSeed(1);
        RngSeedManager::SetRun(1);
        // Reading the next automatic stream takes it, so with no automatic
        // stream taken between them two readings differ by one.
        const uint64_t before = RngSeedManager::GetNextStreamIndex();
        const std::vector<uint16_t> first = Identifications();
        const uint64_t after = RngSeedManager::GetNextStreamIndex();
        NS_TEST_EXPECT_MSG_EQ(after, before + 1, "the draw takes no automatic stream");
        NS_TEST_EXPECT_MSG_EQ((Identifications() == first),
                              true,
                              "another device on the stream draws the same frames");
        const std::set<uint16_t> distinct(first.begin(), first.end());
        // 1000 draws from 65536 values repeat about 8 times.
        NS_TEST_EXPECT_MSG_GT(distinct.size(), 950, "the draw spans 16 bits");
        Simulator::Destroy();
    }

  private:
    static std::vector<uint16_t> Identifications()
    {
        Ptr<QbbNetDevice> device = CreateObject<QbbNetDevice>();
        device->SetAttribute("PfcIdentificationStream", IntegerValue(21));
        std::vector<uint16_t> identifications;
        for (uint32_t i = 0; i < 1000; ++i)
        {
            identifications.push_back(device->DrawPfcIdentification());
        }
        return identifications;
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
            hw->StartPathSelection(qp, 0, 0);
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
            hw->StartPathSelection(qp, 0, 0);
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

    // A probe of the path identification names, from the sender's queue pair.
    void ReceiveProbe(Ipv4Address from, Ipv4Address to, uint16_t identification, bool marked)
    {
        CustomHeader probe;
        probe.l3Prot = kPathProbeProtocol;
        probe.m_tos = marked ? Ipv4Header::ECN_CE : Ipv4Header::ECN_NotECT;
        probe.sip = from.Get();
        probe.dip = to.Get();
        probe.ipid = identification;
        probe.ack.flags = 0;
        probe.ack.sport = kSenderPort;
        probe.ack.dport = kReceiverPort;
        probe.ack.pg = kPriorityGroup;
        hw->Receive(Create<Packet>(), probe);
    }

    // The acknowledgement of the send of seq along the path that
    // identification names, with the receiver's cumulative sequence; or with
    // probeAnswer the answer to a probe of that path.
    void ReceiveAck(Ipv4Address from,
                    uint32_t cumulative,
                    uint32_t seq,
                    uint16_t identification,
                    bool marked = false,
                    bool probeAnswer = false)
    {
        CustomHeader ack;
        ack.l3Prot = 0xFC;
        ack.sip = from.Get();
        ack.ipid = identification;
        ack.ack.flags = (marked ? 1 << qbbHeader::FLAG_CNP : 0) |
                        (probeAnswer ? 1 << qbbHeader::FLAG_PROBE_ANSWER : 0);
        ack.ack.sport = kReceiverPort;
        ack.ack.dport = kSenderPort;
        ack.ack.pg = kPriorityGroup;
        ack.ack.seq = cumulative;
        ack.ack.packet_seq = seq;
        hw->ReceiveAck(Create<Packet>(), ack);
    }

    // The receiver's repair request for the trimmed send of seq, trimmed at the
    // last hop or before it, marked when the trimmed packet carried CE.
    void ReceiveTrimNack(Ptr<RdmaQueuePair> qp,
                         Ipv4Address from,
                         uint32_t seq,
                         uint16_t identification,
                         bool lastHop = false,
                         bool marked = false)
    {
        CustomHeader trim;
        trim.l3Prot = kUecTrimRepairProtocol;
        trim.sip = from.Get();
        trim.ipid = identification;
        trim.ack.flags = (lastHop ? 1 << qbbHeader::FLAG_TRIM_LASTHOP : 0) |
                         (marked ? 1 << qbbHeader::FLAG_CNP : 0);
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

/**
 * A selector that hands out the paths it was given in turn and writes down,
 * in order, what the transport tells it.
 */
class ScriptedSelector : public PathSelector
{
  public:
    explicit ScriptedSelector(std::vector<uint16_t> paths, std::vector<uint16_t> probes = {})
        : m_paths(std::move(paths)),
          m_probes(std::move(probes))
    {
    }

    // The probes it was given, one per send, then none.
    bool TakeProbe(uint64_t, uint16_t& path) override
    {
        if (m_probed == m_probes.size())
        {
            return false;
        }
        path = m_probes[m_probed++];
        return true;
    }

    void OnProbeAnswer(uint16_t path, bool marked, uint64_t) override
    {
        told.push_back("probe answer " + std::to_string(path) + (marked ? " marked" : ""));
    }

    uint16_t Choose(uint64_t) override
    {
        return m_paths[m_chosen++ % m_paths.size()];
    }

    void OnAck(uint16_t path, bool marked, uint64_t) override
    {
        told.push_back("ack " + std::to_string(path) + (marked ? " marked" : ""));
    }

    void OnTrim(uint16_t path, bool lastHop, bool marked, uint64_t) override
    {
        told.push_back("trim " + std::to_string(path) + (lastHop ? " last hop" : "") +
                       (marked ? " marked" : ""));
    }

    void OnLoss(uint16_t path, uint64_t) override
    {
        told.push_back("loss " + std::to_string(path));
    }

    void OnTimeout(uint64_t nowNs) override
    {
        told.push_back("timeout at " + std::to_string(nowNs));
    }

    std::vector<std::string> told;

  private:
    std::vector<uint16_t> m_paths;
    uint32_t m_chosen = 0;
    std::vector<uint16_t> m_probes;
    uint32_t m_probed = 0;
};

class PathSelectorHooksTest : public TestCase
{
  public:
    PathSelectorHooksTest()
        : TestCase("A queue pair's selector names every send's path and hears every answer")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        CustomHeader::ackCarriesPacketSeq = true;
        IsolatedHost sender(LoadBalancingMode::EntropyHash, kTestReceiver);
        Ptr<RdmaQueuePair> qp = sender.AddSender(kTestSender, kTestReceiver, 10 * kMtu);
        auto selector = std::make_unique<ScriptedSelector>(std::vector<uint16_t>{7, 7, 9, 11, 13});
        ScriptedSelector* told = selector.get();
        qp->m_pathSelector = std::move(selector);
        for (uint16_t path : {7, 7, 9, 11, 13})
        {
            CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header);
            sender.hw->GetNxtPacket(qp)->PeekHeader(parsed);
            NS_TEST_EXPECT_MSG_EQ(parsed.ipid, path, "a data packet takes the path chosen for it");
        }

        // The second send on path 7 is acknowledged, marked, which declares
        // the first lost; the send on path 13 is acknowledged unmarked.
        sender.ReceiveAck(kTestReceiver, 0, kMtu, 7, true);
        sender.ReceiveAck(kTestReceiver, 0, 4 * kMtu, 13);
        sender.ReceiveTrimNack(qp, kTestReceiver, 2 * kMtu, 9, true, true);
        sender.ReceiveTrimNack(qp, kTestReceiver, 2 * kMtu, 9);
        // The send on path 11 waits out the timeout.
        Simulator::Stop(NanoSeconds(IsolatedHost::kTimeoutNs + 1));
        Simulator::Run();

        const std::vector<std::string> expected{"ack 7 marked",
                                                "loss 7",
                                                "ack 13",
                                                "trim 9 last hop marked",
                                                "trim 9",
                                                "loss 11",
                                                "timeout at 1000"};
        NS_TEST_EXPECT_MSG_EQ(told->told.size(),
                              expected.size(),
                              "the selector hears each event once");
        for (uint32_t i = 0; i < std::min(expected.size(), told->told.size()); ++i)
        {
            NS_TEST_EXPECT_MSG_EQ(told->told[i], expected[i], "the selector hears it in order");
        }
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        Simulator::Destroy();
    }
};

class PathProbeTest : public TestCase
{
  public:
    PathProbeTest()
        : TestCase("A probe due at a send leaves with it and its answer acknowledges no send")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        CustomHeader::ackCarriesPacketSeq = true;

        IsolatedHost sender(LoadBalancingMode::EntropyHash, kTestReceiver);
        Ptr<RdmaQueuePair> qp = sender.AddSender(kTestSender, kTestReceiver, 10 * kMtu);
        auto selector =
            std::make_unique<ScriptedSelector>(std::vector<uint16_t>{7}, std::vector<uint16_t>{21});
        ScriptedSelector* told = selector.get();
        qp->m_pathSelector = std::move(selector);
        sender.hw->GetNxtPacket(qp);
        sender.hw->GetNxtPacket(qp);
        NS_TEST_ASSERT_MSG_EQ(sender.emitted.size(), 1, "the one probe due leaves, once");
        const CustomHeader& probe = sender.emitted[0];
        NS_TEST_EXPECT_MSG_EQ(probe.l3Prot, kPathProbeProtocol, "it is a probe");
        NS_TEST_EXPECT_MSG_EQ(probe.ipid, 21, "it takes the probed path");
        NS_TEST_EXPECT_MSG_EQ(probe.ack.sport, IsolatedHost::kSenderPort, "it names the flow");
        NS_TEST_EXPECT_MSG_EQ(probe.ack.dport, IsolatedHost::kReceiverPort, "it names the flow");
        NS_TEST_EXPECT_MSG_EQ(probe.ack.pg,
                              IsolatedHost::kPriorityGroup,
                              "it rides the flow's priority group");
        NS_TEST_EXPECT_MSG_EQ(IsUetTrimmableDscp(probe.GetIpv4Dscp()),
                              false,
                              "it is not trimmable");

        IsolatedHost receiver(LoadBalancingMode::EntropyHash, kTestSender);
        receiver.ReceiveProbe(kTestSender, kTestReceiver, 21, false);
        NS_TEST_EXPECT_MSG_EQ(receiver.emitted.size(), 0, "a probe for no flow goes unanswered");
        receiver.ReceiveData(kTestSender, kTestReceiver, 0, 7);
        receiver.ReceiveProbe(kTestSender, kTestReceiver, 21, true);
        NS_TEST_ASSERT_MSG_EQ(receiver.emitted.size(), 2, "a probe for a flow is answered");
        const CustomHeader& answer = receiver.emitted[1];
        const auto answersProbe = [](const CustomHeader& ack) {
            return static_cast<bool>((ack.ack.flags >> qbbHeader::FLAG_PROBE_ANSWER) & 1);
        };
        const bool marked = (answer.ack.flags >> qbbHeader::FLAG_CNP) & 1;
        NS_TEST_EXPECT_MSG_EQ(answer.l3Prot, 0xFC, "the answer is an acknowledgement");
        NS_TEST_EXPECT_MSG_EQ(answersProbe(answer), true, "that says it answers a probe");
        NS_TEST_EXPECT_MSG_EQ(marked, true, "with the probe's mark");
        NS_TEST_EXPECT_MSG_EQ(answer.ipid, 21, "and the probe's path");
        NS_TEST_EXPECT_MSG_EQ(answer.ack.seq, kMtu, "and the cumulative acknowledgement");
        NS_TEST_EXPECT_MSG_EQ(answersProbe(receiver.emitted[0]),
                              false,
                              "a data packet's acknowledgement answers no probe");
        Ptr<RdmaRxQueuePair> flow = receiver.hw->m_rxQpMap.begin()->second;
        NS_TEST_EXPECT_MSG_EQ(flow->ReceiverNextExpectedSeq,
                              kMtu,
                              "a probe moves no receive state");
        NS_TEST_EXPECT_MSG_EQ(flow->m_data_arrivals, 1, "a probe is no data arrival");

        const uint64_t outstanding = qp->m_outstanding.Bytes();
        sender.ReceiveAck(kTestReceiver, 0, 0, 21, true, true);
        NS_TEST_EXPECT_MSG_EQ(qp->m_outstanding.Bytes(),
                              outstanding,
                              "the answer acknowledges no send");
        NS_TEST_ASSERT_MSG_EQ(told->told.size(), 1, "the selector hears the answer alone");
        NS_TEST_EXPECT_MSG_EQ(told->told[0], "probe answer 21 marked", "as a probe's answer");
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        Simulator::Destroy();
    }
};

// Two variables on one fixed stream draw the same sequence, so a selector's
// fresh draws can be read off a second selector that draws nothing else.
Ptr<UniformRandomVariable> SelectorStream()
{
    return CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(5));
}

class RepsSelectorTest : public TestCase
{
  public:
    RepsSelectorTest()
        : TestCase("REPS reuses each unmarked acknowledgement's value once and freezes on a "
                   "timeout")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kWindow = 20;
        constexpr uint64_t kFreezingNs = 1000;
        RepsSelector reps(SelectorStream(), 8, kFreezingNs, kWindow);
        ObliviousSelector fresh(SelectorStream());
        // How many of the next sends draw afresh: take what OPS on the same
        // stream draws next.
        const auto freshSends = [&reps, &fresh](uint32_t sends, uint64_t nowNs) {
            uint32_t drawn = 0;
            for (uint32_t i = 0; i < sends; ++i)
            {
                drawn += reps.Choose(nowNs) == fresh.Choose(0);
            }
            return drawn;
        };
        NS_TEST_EXPECT_MSG_EQ(freshSends(kWindow, 0), kWindow, "the first window explores");

        reps.OnAck(0x1234, false, 0);
        const uint16_t reused = reps.Choose(0);
        NS_TEST_EXPECT_MSG_EQ(reused, 0x1234, "an unmarked value is reused");
        NS_TEST_EXPECT_MSG_EQ(freshSends(50, 0), 50, "and only once");
        reps.OnAck(0x4321, true, 0);
        NS_TEST_EXPECT_MSG_EQ(freshSends(1, 0), 1, "a marked value is not reused");

        // Ten values into eight entries: the two oldest are overwritten, and
        // the rest are reused oldest first.
        for (uint16_t ev = 1; ev <= 10; ++ev)
        {
            reps.OnAck(ev, false, 0);
        }
        std::vector<uint16_t> cached;
        for (uint32_t i = 0; i < 8; ++i)
        {
            cached.push_back(reps.Choose(0));
        }
        NS_TEST_EXPECT_MSG_EQ((cached == std::vector<uint16_t>{3, 4, 5, 6, 7, 8, 9, 10}),
                              true,
                              "cached values are reused oldest first");
        NS_TEST_EXPECT_MSG_EQ(freshSends(1, 0), 1, "a spent buffer draws afresh");

        // Every cached value used, a timeout freezes: nothing is drawn, and the
        // eight held values come round in turn.
        reps.OnTimeout(100);
        std::vector<uint16_t> frozen;
        for (uint32_t i = 0; i < 16; ++i)
        {
            frozen.push_back(reps.Choose(200));
        }
        std::vector<uint16_t> held(frozen.begin(), frozen.begin() + 8);
        std::sort(held.begin(), held.end());
        NS_TEST_EXPECT_MSG_EQ((held == std::vector<uint16_t>{3, 4, 5, 6, 7, 8, 9, 10}),
                              true,
                              "freezing mode cycles through the held values");
        NS_TEST_EXPECT_MSG_EQ((std::equal(frozen.begin(), frozen.begin() + 8, frozen.begin() + 8)),
                              true,
                              "freezing mode cycles in a fixed order");

        // An unmarked value before the timeout runs out is reused first, and
        // the cycle goes on.
        const std::set<uint16_t> stillHeld{4, 5, 6, 7, 8, 9, 10, 77};
        const auto heldSends = [&reps, &stillHeld](uint32_t sends, uint64_t nowNs) {
            uint32_t fromHeld = 0;
            for (uint32_t i = 0; i < sends; ++i)
            {
                fromHeld += stillHeld.count(reps.Choose(nowNs));
            }
            return fromHeld;
        };
        reps.OnAck(77, false, 500);
        const uint16_t acknowledged = reps.Choose(600);
        NS_TEST_EXPECT_MSG_EQ(acknowledged, 77, "a fresh acknowledgement's value comes first");
        NS_TEST_EXPECT_MSG_EQ(heldSends(8, 600), 8, "freezing mode lasts its timeout");
        reps.OnAck(88, true, 2000);
        NS_TEST_EXPECT_MSG_EQ(heldSends(1, 2000),
                              1,
                              "a marked acknowledgement does not end freezing mode");

        // The first unmarked acknowledgement after the timeout ends it, and a
        // window of fresh values follows, which a timeout does not interrupt.
        reps.OnAck(99, false, 2000);
        reps.OnTimeout(2100);
        NS_TEST_EXPECT_MSG_EQ(freshSends(kWindow, 2200),
                              kWindow,
                              "leaving freezing mode explores a window");
        const uint16_t afterExploring = reps.Choose(2200);
        NS_TEST_EXPECT_MSG_EQ(afterExploring, 99, "then the cached value is reused");
        NS_TEST_EXPECT_MSG_EQ(freshSends(1, 2200), 1, "a timeout while exploring does not freeze");
    }
};

class UeSelectorTest : public TestCase
{
  public:
    UeSelectorTest()
        : TestCase("UEC spraying uses every value once per pass and skips a marked one once")
    {
    }

    void DoRun() override
    {
        Rotation();
        SkipOnce();
        Saturation(4, 4);
        Saturation(5, 0);
    }

  private:
    static constexpr uint32_t kValues = 16;

    // Oblivious spraying: each pass is a permutation of the space, drawn anew.
    void Rotation()
    {
        UeObliviousSelector oblivious(SelectorStream(), kValues);
        std::vector<std::vector<uint16_t>> passes(4);
        for (auto& pass : passes)
        {
            for (uint32_t i = 0; i < kValues; ++i)
            {
                pass.push_back(oblivious.Choose(0));
            }
            std::vector<uint16_t> values(pass);
            std::sort(values.begin(), values.end());
            for (uint32_t ev = 0; ev < kValues; ++ev)
            {
                NS_TEST_EXPECT_MSG_EQ(values[ev], ev, "a pass uses every value once");
            }
        }
        std::set<std::vector<uint16_t>> orders(passes.begin(), passes.end());
        NS_TEST_EXPECT_MSG_EQ(orders.size(), passes.size(), "every pass has its own order");
    }

    // Path-aware spraying rotates as oblivious spraying does, on the same
    // stream, and leaves out a marked value the first time it comes round.
    void SkipOnce()
    {
        UeAwareSelector aware(SelectorStream(), kValues, 0.5);
        EntropyRotation order(SelectorStream(), kValues);
        std::vector<uint16_t> expected;
        for (uint32_t i = 0; i < 3 * kValues; ++i)
        {
            expected.push_back(order.Next());
        }
        // Marked by an acknowledgement, a trim before the last hop and a
        // marked last-hop trim; an unmarked acknowledgement and an unmarked
        // last-hop trim say nothing.
        const uint16_t skipped[] = {expected[3], expected[5], expected[9]};
        aware.OnAck(skipped[0], true, 0);
        aware.OnTrim(skipped[1], false, false, 0);
        aware.OnTrim(skipped[2], true, true, 0);
        aware.OnAck(expected[1], false, 0);
        aware.OnTrim(expected[2], true, false, 0);
        for (uint32_t position : {9, 5, 3})
        {
            expected.erase(expected.begin() + position);
        }
        std::vector<uint16_t> sent;
        for (uint32_t i = 0; i < expected.size(); ++i)
        {
            sent.push_back(aware.Choose(0));
        }
        NS_TEST_EXPECT_MSG_EQ((sent == expected),
                              true,
                              "a marked value is skipped once, and nothing else is");
    }

    // Eight values, half of them the saturation point, the marked ones the
    // first the rotation reaches. The first send takes the first unmarked
    // value unless more than half are marked, when it takes the first marked
    // one; with that one's mark spent, the second send skips the rest.
    void Saturation(uint32_t marked, uint32_t firstSent)
    {
        constexpr uint32_t kSmall = 8;
        UeAwareSelector aware(SelectorStream(), kSmall, 0.5);
        EntropyRotation order(SelectorStream(), kSmall);
        std::vector<uint16_t> pass;
        for (uint32_t i = 0; i < kSmall; ++i)
        {
            pass.push_back(order.Next());
        }
        for (uint32_t i = 0; i < marked; ++i)
        {
            aware.OnAck(pass[i], true, 0);
        }
        // Marking a marked value again marks nothing more.
        aware.OnAck(pass[0], true, 0);
        const uint16_t first = aware.Choose(0);
        const uint16_t second = aware.Choose(0);
        NS_TEST_EXPECT_MSG_EQ(first, pass[firstSent], "skipping stops above saturation");
        NS_TEST_EXPECT_MSG_EQ(second, pass[5], "and resumes at saturation");
    }
};

class MrcSelectorTest : public TestCase
{
  public:
    MrcSelectorTest()
        : TestCase("MRC moves each value between GOOD, SKIP and ASSUMED_BAD on its evidence")
    {
    }

    void DoRun() override
    {
        SkipOnMark();
        OneResetPerSend();
        SkipLapses();
        AssumedBadUntilProbed();
        DeadPathDrained();
        AllAssumedBad();
    }

  private:
    static constexpr uint32_t kValues = 16;
    static constexpr uint64_t kSkipNs = 1000;
    static constexpr uint64_t kProbeNs = 10000;

    // A selector and the order its rotation will take, read off a rotation on
    // the same stream.
    struct Fixture
    {
        explicit Fixture(uint32_t values = kValues)
            : mrc(SelectorStream(), values, kSkipNs, kProbeNs)
        {
            EntropyRotation order(SelectorStream(), values);
            for (uint32_t i = 0; i < 4 * values; ++i)
            {
                rotation.push_back(order.Next());
            }
        }

        // Sends at nowNs, one per position from first to last but those left
        // out, compared with the rotation's values there: the first that
        // differs, or nothing.
        std::string Sends(uint32_t first,
                          uint32_t last,
                          const std::set<uint32_t>& leftOut,
                          uint64_t nowNs)
        {
            for (uint32_t position = first; position < last; ++position)
            {
                if (leftOut.count(position) > 0)
                {
                    continue;
                }
                const uint16_t sent = mrc.Choose(nowNs);
                if (sent != rotation[position])
                {
                    return "position " + std::to_string(position) + " sent " +
                           std::to_string(sent) + ", not " + std::to_string(rotation[position]);
                }
            }
            return "";
        }

        // The probes due at nowNs, in order; no more than one per value.
        std::vector<uint16_t> Probes(uint64_t nowNs)
        {
            std::vector<uint16_t> due;
            uint16_t probed;
            while (due.size() <= rotation.size() && mrc.TakeProbe(nowNs, probed))
            {
                due.push_back(probed);
            }
            return due;
        }

        MrcSelector mrc;
        std::vector<uint16_t> rotation;
    };

    // A marked acknowledgement and a trim before the last hop each move a value
    // to SKIP, passed over once; a last-hop trim, marked or not, and an
    // unmarked acknowledgement move nothing.
    void SkipOnMark()
    {
        Fixture f;
        f.mrc.OnAck(f.rotation[2], true, 0);
        f.mrc.OnTrim(f.rotation[5], false, false, 0);
        f.mrc.OnTrim(f.rotation[8], true, true, 0);
        f.mrc.OnAck(f.rotation[10], false, 0);
        const std::string differs = f.Sends(0, 2 * kValues, {2, 5}, 0);
        NS_TEST_EXPECT_MSG_EQ(differs, "", "a SKIP value is passed over once");
    }

    // A send resets the first SKIP value it passes over and only that one, so
    // of two adjacent SKIP values the second is passed over again next pass.
    void OneResetPerSend()
    {
        Fixture f;
        f.mrc.OnAck(f.rotation[3], true, 0);
        f.mrc.OnAck(f.rotation[4], true, 0);
        uint32_t second = kValues;
        while (f.rotation[second] != f.rotation[4])
        {
            ++second;
        }
        const std::string differs = f.Sends(0, 3 * kValues, {3, 4, second}, 0);
        NS_TEST_EXPECT_MSG_EQ(differs, "", "one send resets one SKIP value");
    }

    void SkipLapses()
    {
        Fixture f;
        f.mrc.OnAck(f.rotation[2], true, 0);
        const std::string differs = f.Sends(0, kValues, {}, kSkipNs);
        NS_TEST_EXPECT_MSG_EQ(differs, "", "a SKIP value is GOOD again after its time");
    }

    // A declared loss takes a value out of service, a probe of it falls due
    // every interval until one is answered, and an unmarked answer brings it
    // back; the answer to data sent on it does not.
    void AssumedBadUntilProbed()
    {
        Fixture f;
        f.mrc.OnLoss(f.rotation[1], 0);
        f.mrc.OnAck(f.rotation[1], false, 100);
        std::set<uint32_t> bad;
        for (uint32_t position = 0; position < 4 * kValues; ++position)
        {
            if (f.rotation[position] == f.rotation[1])
            {
                bad.insert(position);
            }
        }
        const std::string withoutBad = f.Sends(0, 2 * kValues, bad, 100);
        NS_TEST_EXPECT_MSG_EQ(withoutBad, "", "an ASSUMED_BAD value is not sent on");
        const std::vector<uint16_t> condemned{f.rotation[1]};
        const std::vector<uint16_t> early = f.Probes(kProbeNs - 1);
        const std::vector<uint16_t> first = f.Probes(kProbeNs);
        const std::vector<uint16_t> again = f.Probes(kProbeNs);
        const std::vector<uint16_t> second = f.Probes(2 * kProbeNs);
        f.mrc.OnProbeAnswer(f.rotation[1], false, 2 * kProbeNs);
        const std::vector<uint16_t> answered = f.Probes(3 * kProbeNs);
        NS_TEST_EXPECT_MSG_EQ(early.empty(), true, "no probe is due before its interval");
        NS_TEST_EXPECT_MSG_EQ((first == condemned), true, "then the value assumed bad is probed");
        NS_TEST_EXPECT_MSG_EQ(again.empty(), true, "once");
        NS_TEST_EXPECT_MSG_EQ((second == condemned), true, "and again while unanswered");
        NS_TEST_EXPECT_MSG_EQ(answered.empty(), true, "an answered value is not probed");
        const std::string restored = f.Sends(2 * kValues, 4 * kValues, {}, 2 * kProbeNs);
        NS_TEST_EXPECT_MSG_EQ(restored, "", "an unmarked answer restores GOOD");

        // A marked answer moves the value to SKIP: passed over once.
        Fixture g;
        g.mrc.OnLoss(g.rotation[6], 0);
        g.mrc.OnProbeAnswer(g.rotation[6], true, 0);
        const std::string skippedOnce = g.Sends(0, 2 * kValues, {6}, 0);
        NS_TEST_EXPECT_MSG_EQ(skippedOnce, "", "a marked answer moves to SKIP");
    }

    // Every value of a dead path assumed bad as its sends are lost: none is
    // sent on again, and each is probed once per interval, in the order they
    // were condemned.
    void DeadPathDrained()
    {
        Fixture f;
        const std::vector<uint32_t> dead{0, 3, 7, 8, 12};
        std::set<uint16_t> deadValues;
        for (uint32_t i = 0; i < dead.size(); ++i)
        {
            f.mrc.OnLoss(f.rotation[dead[i]], i);
            deadValues.insert(f.rotation[dead[i]]);
        }
        uint32_t sentOnDead = 0;
        for (uint32_t i = 0; i < 4 * kValues; ++i)
        {
            sentOnDead += deadValues.count(f.mrc.Choose(100));
        }
        NS_TEST_EXPECT_MSG_EQ(sentOnDead, 0, "a dead path is drained");
        std::vector<uint16_t> condemned;
        for (uint32_t position : dead)
        {
            condemned.push_back(f.rotation[position]);
        }
        for (uint64_t round = 1; round <= 2; ++round)
        {
            const std::vector<uint16_t> due = f.Probes(round * kProbeNs + dead.size());
            NS_TEST_EXPECT_MSG_EQ((due == condemned),
                                  true,
                                  "each dead value is probed once per interval, in the order "
                                  "condemned");
        }
    }

    // With every value assumed bad, a send still takes one.
    void AllAssumedBad()
    {
        Fixture f(4);
        for (uint32_t position = 0; position < 4; ++position)
        {
            f.mrc.OnLoss(f.rotation[position], 0);
        }
        const uint16_t sent = f.mrc.Choose(0);
        NS_TEST_EXPECT_MSG_EQ(sent, f.rotation[0], "the first value reached is sent");
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

class DuplicateRepairTest : public TestCase
{
  public:
    DuplicateRepairTest()
        : TestCase("A repair of a packet already repaired is counted as a duplicate")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, 8);
        Ptr<RdmaQueuePair> qp = sender.AddSender(kTestSender, kTestReceiver, 4 * kMtu);
        for (uint32_t i = 0; i < 2; ++i)
        {
            sender.hw->GetNxtPacket(qp);
        }
        NS_TEST_EXPECT_MSG_EQ(qp->DeclareLostSentBy(0), 2 * kMtu, "both sends are lost");
        for (uint32_t i = 0; i < 2; ++i)
        {
            sender.hw->GetNxtPacket(qp);
        }
        NS_TEST_EXPECT_MSG_EQ(qp->m_duplicate_repairs, 0, "a first repair is no duplicate");
        NS_TEST_EXPECT_MSG_EQ(qp->DeclareLostSentBy(0), 2 * kMtu, "both repairs are lost");
        CustomHeader again(CustomHeader::L2_Header | CustomHeader::L3_Header |
                           CustomHeader::L4_Header);
        sender.hw->GetNxtPacket(qp)->PeekHeader(again);
        NS_TEST_EXPECT_MSG_EQ(again.udp.seq, 0, "the first packet is repaired again");
        NS_TEST_EXPECT_MSG_EQ(qp->m_duplicate_repairs, 1, "its second repair is a duplicate");

        // Once the acknowledgement passes both packets the next send is new
        // data, which repairs nothing.
        sender.ReceiveAck(kTestReceiver, 2 * kMtu, 0, again.ipid);
        sender.hw->GetNxtPacket(qp);
        NS_TEST_EXPECT_MSG_EQ(qp->m_duplicate_repairs, 1, "new data repairs nothing");
        Simulator::Destroy();
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
            window.OnAck(kMtu, marked, kBaseRtt + delay, inflight, now, 0);
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
        window.OnAck(NsccPath::kMtu, false, NsccPath::kBaseRtt + 3 * NsccPath::kTarget, 0, now, 0);
        const uint64_t before = window.Cwnd();
        for (uint32_t i = 0; i < 6; ++i)
        {
            window.OnAck(NsccPath::kMtu,
                         true,
                         NsccPath::kBaseRtt + NsccPath::kTarget - 1,
                         0,
                         ++now,
                         0);
            NS_TEST_EXPECT_MSG_EQ(window.Cwnd(),
                                  before,
                                  "a mark one nanosecond under the target is left to load balancing");
        }
        window.OnAck(NsccPath::kMtu, true, NsccPath::kBaseRtt + NsccPath::kTarget, 0, ++now, 0);
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
                window.OnAck(kMtu, marked, NsccPath::kBaseRtt + delay, inflight, now, 0);
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
class SpineReportHeaderTest : public TestCase
{
  public:
    SpineReportHeaderTest()
        : TestCase("An acknowledgement carries a spine report only under spray_policy, in the "
                   "padding of its frame")
    {
    }

    void DoRun() override
    {
        const IntHeader::Mode savedIntMode = IntHeader::mode;
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        const uint32_t savedReportBytes = CustomHeader::ackReportBytes;
        IntHeader::mode = IntHeader::NONE;
        CustomHeader::ackCarriesPacketSeq = true;
        CustomHeader::ackReportBytes = 0;
        NS_TEST_EXPECT_MSG_EQ(qbbHeader().GetSerializedSize(),
                              20,
                              "without a report the header keeps its size");

        // Eight spines: a sequence byte and two bytes of grades.
        CustomHeader::ackReportBytes = 1 + SpineReport::GradeBytes(8);
        NS_TEST_EXPECT_MSG_EQ(qbbHeader().GetSerializedSize(), 23, "the report adds three bytes");
        NS_TEST_EXPECT_MSG_EQ(CustomHeader::GetAckSerializedSize(),
                              23,
                              "the parser agrees on the size with the report");
        NS_TEST_EXPECT_MSG_LT_OR_EQ(qbbHeader().GetSerializedSize(),
                                    60 - 14 - 20,
                                    "the report fits the padding of a minimum frame");
        NS_TEST_EXPECT_MSG_EQ(SpineReport::GradeBytes(32), 8, "32 spines take eight bytes");

        SpineReport report;
        report.sequence = 77;
        report.edgeCongested = true;
        report.SetGrade(2, 0);
        report.SetGrade(5, 1);
        report.SetGrade(6, 2);
        qbbHeader ack;
        ack.SetSeq(3000);
        ack.SetPacketSeq(7000);
        ack.SetCnp();
        ack.SetSpineReport(report);
        Ptr<Packet> packet = Create<Packet>(0);
        packet->AddHeader(ack);
        qbbHeader copy;
        packet->PeekHeader(copy);
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
        NS_TEST_EXPECT_MSG_EQ(parsed.ack.packet_seq, 7000, "the packet sequence keeps its place");
        NS_TEST_EXPECT_MSG_EQ(parsed.ack.spine_report[0], 77, "the report's sequence is parsed");
        NS_TEST_EXPECT_MSG_EQ(((parsed.ack.flags >> qbbHeader::FLAG_EDGE_CONGESTED) & 1),
                              1,
                              "the edge bit rides the flags");
        NS_TEST_EXPECT_MSG_EQ(((parsed.ack.flags >> qbbHeader::FLAG_CNP) & 1),
                              1,
                              "the edge bit leaves the mark alone");
        NS_TEST_EXPECT_MSG_EQ(parsed.GetSerializedSize(),
                              packet->GetSize(),
                              "the parser consumes exactly the header written");
        SpineReport wire;
        std::copy_n(parsed.ack.spine_report + 1, CustomHeader::ackReportBytes - 1, wire.grades);
        const SpineReport& read = copy.GetSpineReport();
        NS_TEST_EXPECT_MSG_EQ(read.sequence, 77, "the header reads its own report back");
        NS_TEST_EXPECT_MSG_EQ(read.edgeCongested, true, "the header reads its own edge bit back");
        const uint8_t expected[8] = {3, 3, 0, 3, 3, 1, 2, 3};
        for (uint32_t spine = 0; spine < 8; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(uint32_t(wire.Grade(spine)),
                                  uint32_t(expected[spine]),
                                  "each spine's grade survives the wire");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(read.Grade(spine)),
                                  uint32_t(expected[spine]),
                                  "each spine's grade survives the header");
        }

        CustomHeader::ackReportBytes = savedReportBytes;
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        IntHeader::mode = savedIntMode;
    }
};

class SupervisedEwmaTest : public TestCase
{
  public:
    SupervisedEwmaTest()
        : TestCase("A supervised average follows its gain and restarts at a step")
    {
    }

    void DoRun() override
    {
        const SupervisedEwma::Parameters parameters{1.0 / 16, 0.125, 0.5};
        SupervisedEwma average;
        NS_TEST_EXPECT_MSG_EQ(average.Value(), 0, "an average starts at zero");
        // Samples within the slack of the average move it by the gain alone.
        for (uint32_t i = 0; i < 100; ++i)
        {
            average.Add(0.1, parameters);
        }
        NS_TEST_EXPECT_MSG_EQ_TOL(average.Value(),
                                  0.1 * (1 - std::pow(15.0 / 16, 100)),
                                  1e-12,
                                  "a sample inside the slack moves the average by the gain");
        // A step of 0.5: the excess net of the slack is 0.275 per sample, so
        // the second sample passes the threshold and restarts the average.
        const double before = average.Value();
        average.Add(0.6, parameters);
        NS_TEST_EXPECT_MSG_EQ_TOL(average.Value(),
                                  before + (0.6 - before) / 16,
                                  1e-12,
                                  "one sample of a step moves the average by the gain");
        average.Add(0.6, parameters);
        NS_TEST_EXPECT_MSG_EQ(average.Value(), 0.6, "the second sample of a step restarts it");
        // And back down.
        average.Add(0.0, parameters);
        average.Add(0.0, parameters);
        NS_TEST_EXPECT_MSG_EQ(average.Value(), 0.0, "a fall restarts it the same way");
    }
};

/**
 * A receiver's grader of eight spines with the RdmaHw defaults, over 10 us
 * intervals starting at zero, and the intervals it is fed.
 */
class GraderFixture
{
  public:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint64_t kInterval = 10000;

    static SpineGrader::Parameters Defaults(bool oneWayDelay = false)
    {
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        SpineGrader::Parameters parameters;
        parameters.spines = kSpines;
        parameters.intervalNs = kInterval;
        parameters.phaseNs = 0;
        parameters.marks = {hw->m_sprayEstimatorGain,
                            hw->m_sprayMarkCusumSlack,
                            hw->m_sprayMarkCusumThreshold};
        parameters.markThresholds[0] = hw->m_sprayMarkThreshold1;
        parameters.markThresholds[1] = hw->m_sprayMarkThreshold2;
        parameters.markThresholds[2] = hw->m_sprayMarkThreshold3;
        parameters.oneWayDelay = oneWayDelay;
        // A base RTT of 10 us.
        parameters.delayNs = {hw->m_sprayEstimatorGain,
                              hw->m_sprayDelayCusumSlackBaseRtts * 10000,
                              hw->m_sprayDelayCusumThresholdBaseRtts * 10000};
        parameters.delayThresholdsNs[0] = hw->m_sprayDelayThreshold1BaseRtts * 10000;
        parameters.delayThresholdsNs[1] = hw->m_sprayDelayThreshold2BaseRtts * 10000;
        parameters.delayThresholdsNs[2] = hw->m_sprayDelayThreshold3BaseRtts * 10000;
        parameters.holdDownIntervals = hw->m_sprayHoldDownIntervals;
        return parameters;
    }

    // One interval of arrivals on each spine, marked at the fraction given,
    // then the report the next interval's first event issues.
    static void Interval(SpineGrader& grader,
                         uint32_t interval,
                         const std::vector<double>& markFractions,
                         uint32_t arrivals = 100)
    {
        grader.Advance(interval * kInterval);
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            const uint32_t marked = std::lround(markFractions[spine] * arrivals);
            for (uint32_t i = 0; i < arrivals; ++i)
            {
                grader.OnArrival(spine, spine, i < marked, 0);
            }
        }
        NS_ASSERT(grader.Advance((interval + 1) * kInterval));
    }
};

class SpineAttributionTest : public TestCase
{
  public:
    SpineAttributionTest()
        : TestCase("A receiver attributes marks to the spine that carried them, and to it alone")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kDegraded = 3;
        constexpr uint32_t kIntervals = 40;
        SpineGrader grader(GraderFixture::Defaults());
        // Every spine marks two packets in a hundred at random; from interval
        // 20 spine 3 marks sixty.
        std::mt19937 random(7);
        std::uniform_real_distribution<double> uniform;
        for (uint32_t interval = 0; interval < kIntervals; ++interval)
        {
            grader.Advance(interval * GraderFixture::kInterval);
            for (uint32_t packet = 0; packet < 800; ++packet)
            {
                const uint8_t carrying = packet % GraderFixture::kSpines;
                const double rate = carrying == kDegraded && interval >= 20 ? 0.6 : 0.02;
                grader.OnArrival(carrying, carrying, uniform(random) < rate, 0);
            }
            grader.Advance((interval + 1) * GraderFixture::kInterval);
            const std::vector<SpineGrader::SpineInterval>& closed = grader.LastInterval();
            for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
            {
                NS_TEST_EXPECT_MSG_EQ(closed[spine].arrivals, 100, "every arrival is counted once");
                const bool degraded = spine == kDegraded && interval >= 21;
                if (degraded)
                {
                    NS_TEST_EXPECT_MSG_GT(closed[spine].markFraction,
                                          0.5,
                                          "the degraded spine's average rises within two "
                                          "intervals");
                }
                else if (spine != kDegraded || interval < 20)
                {
                    NS_TEST_EXPECT_MSG_LT(closed[spine].markFraction,
                                          0.06,
                                          "a healthy spine's average stays at its level");
                }
                NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(spine)),
                                      degraded ? 1u : 3u,
                                      "only the degraded spine's grade falls");
            }
            NS_TEST_EXPECT_MSG_EQ(grader.Report().edgeCongested,
                                  false,
                                  "one degraded spine is not the receiver's downlink");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().sequence),
                                  interval + 1,
                                  "each interval issues one report");
        }

        // Marks on packets one spine carried for another are the carrier's.
        SpineGrader moved(GraderFixture::Defaults());
        for (uint32_t i = 0; i < 100; ++i)
        {
            moved.OnArrival(4, 5, true, 0);
        }
        moved.Advance(GraderFixture::kInterval);
        NS_TEST_EXPECT_MSG_EQ(moved.LastInterval()[5].marked, 100, "the carrier counts the marks");
        NS_TEST_EXPECT_MSG_EQ(moved.LastInterval()[4].arrivals, 0, "the requested spine none");
        NS_TEST_EXPECT_MSG_EQ(moved.LastInterval()[4].moved, 100, "but counts the moves");
        Simulator::Destroy();
    }
};

class SpineGradeTest : public TestCase
{
  public:
    SpineGradeTest()
        : TestCase("A spine's grade follows absolute thresholds, marks before delay, a trim "
                   "holds it down for a while, and the edge bit needs every spine")
    {
    }

    void DoRun() override
    {
        Thresholds();
        DelayAfterMarks();
        HoldDown();
        Edge();
        Simulator::Destroy();
    }

  private:
    void Thresholds()
    {
        SpineGrader grader(GraderFixture::Defaults());
        const std::vector<double> fractions{0.0, 0.2, 0.3, 0.45, 0.55, 0.7, 0.8, 1.0};
        for (uint32_t interval = 0; interval < 4; ++interval)
        {
            GraderFixture::Interval(grader, interval, fractions);
        }
        const uint32_t expected[] = {3, 3, 2, 2, 1, 1, 0, 0};
        for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(spine)),
                                  expected[spine],
                                  "a steady mark fraction earns the grade of its band");
        }
        NS_TEST_EXPECT_MSG_EQ(grader.Report().edgeCongested,
                              false,
                              "spines graded apart are not the downlink");
    }

    // With one-way delay, a spine without marks grades by its delay above the
    // least it has shown, and a spine with marks by its marks whatever its
    // delay. Base RTT 10 us: the delay bands start at 2.5, 5 and 7.5 us.
    void DelayAfterMarks()
    {
        SpineGrader grader(GraderFixture::Defaults(true));
        const uint64_t kLeast = 3000;
        const uint64_t extra[GraderFixture::kSpines] = {0, 2000, 3000, 6000, 9000, 9000, 0, 0};
        const bool marks[GraderFixture::kSpines] =
            {false, false, false, false, false, true, true, false};
        for (uint32_t interval = 0; interval < 4; ++interval)
        {
            grader.Advance(interval * GraderFixture::kInterval);
            for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
            {
                // One packet at the least delay, so that the excess is what the
                // others carry.
                grader.OnArrival(spine, spine, false, kLeast);
                for (uint32_t i = 0; i < 99; ++i)
                {
                    grader.OnArrival(spine, spine, marks[spine] && i < 40, kLeast + extra[spine]);
                }
            }
            grader.Advance((interval + 1) * GraderFixture::kInterval);
        }
        // Spine 5 is marked at 0.4 and delayed into grade 0; spine 6 is
        // marked at 0.4 without delay.
        const uint32_t expected[] = {3, 3, 2, 1, 0, 2, 2, 3};
        for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(spine)),
                                  expected[spine],
                                  "marks grade a spine first and delay grades the rest");
        }
        NS_TEST_EXPECT_MSG_EQ_TOL(grader.LastInterval()[3].delayNs,
                                  6000 * 0.99,
                                  1,
                                  "the delay is the mean excess over the spine's least");
    }

    void HoldDown()
    {
        const SpineGrader::Parameters parameters = GraderFixture::Defaults();
        const uint32_t hold = parameters.holdDownIntervals;
        NS_TEST_ASSERT_MSG_GT(hold, 1, "the hold lasts more than one interval");
        SpineGrader grader(parameters);
        const std::vector<double> clean(GraderFixture::kSpines, 0.0);
        GraderFixture::Interval(grader, 0, clean);
        // Interval 1: a trim on spine 4 before the last hop, one at the last
        // hop on spine 6, and a packet requested on spine 5 carried by 7.
        grader.Advance(GraderFixture::kInterval);
        grader.OnTrim(4, false);
        grader.OnTrim(6, true);
        grader.OnArrival(5, 7, false, 0);
        for (uint32_t interval = 1; interval < 1 + hold + 2; ++interval)
        {
            if (interval > 1)
            {
                GraderFixture::Interval(grader, interval, clean);
            }
            else
            {
                grader.Advance(2 * GraderFixture::kInterval);
            }
            const bool held = interval < 1 + hold;
            for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
            {
                const bool heldSpine = held && (spine == 4 || spine == 5);
                NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(spine)),
                                      heldSpine ? 0u : 3u,
                                      "a trim or a move holds its spine at 0 for the hold's "
                                      "intervals and no other");
                NS_TEST_EXPECT_MSG_EQ(grader.LastInterval()[spine].held,
                                      heldSpine,
                                      "the interval records the hold");
            }
            NS_TEST_EXPECT_MSG_EQ(grader.Report().edgeCongested,
                                  (interval == 1),
                                  "a last-hop trim sets the edge bit for its interval");
        }
        // A trim during the hold extends it from its own interval.
        SpineGrader again(parameters);
        again.OnTrim(2, false);
        again.Advance(GraderFixture::kInterval);
        again.Advance(2 * GraderFixture::kInterval - 1);
        again.OnTrim(2, false);
        for (uint32_t interval = 1; interval < hold + 1; ++interval)
        {
            again.Advance((interval + 1) * GraderFixture::kInterval);
            NS_TEST_EXPECT_MSG_EQ(uint32_t(again.Report().Grade(2)),
                                  0u,
                                  "a second trim extends the hold");
        }
        again.Advance((hold + 2) * GraderFixture::kInterval);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(again.Report().Grade(2)), 3u, "and the hold ends");
        // An idle stretch counts against the hold like any other interval.
        SpineGrader idle(parameters);
        idle.OnTrim(1, false);
        idle.Advance(GraderFixture::kInterval);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(idle.Report().Grade(1)), 0u, "the trim holds its spine");
        idle.Advance((hold + 1) * GraderFixture::kInterval);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(idle.Report().Grade(1)), 3u, "idle intervals run it out");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(idle.Report().sequence), 2u, "one report per closing");
    }

    void Edge()
    {
        SpineGrader all(GraderFixture::Defaults());
        SpineGrader one(GraderFixture::Defaults());
        std::vector<double> oneMarked(GraderFixture::kSpines, 0.0);
        oneMarked[2] = 0.4;
        for (uint32_t interval = 0; interval < 4; ++interval)
        {
            GraderFixture::Interval(all,
                                    interval,
                                    std::vector<double>(GraderFixture::kSpines, 0.4));
            GraderFixture::Interval(one, interval, oneMarked);
        }
        NS_TEST_EXPECT_MSG_EQ(all.Report().edgeCongested,
                              true,
                              "every spine marked alike sets the edge bit");
        NS_TEST_EXPECT_MSG_EQ(one.Report().edgeCongested, false, "one spine marked does not");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(one.Report().Grade(2)), 2u, "it grades that spine down");
    }
};

/**
 * A sender's scores of eight spines towards one host with the RdmaHw
 * defaults, and the reports it is fed.
 */
class ScoresFixture
{
  public:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint64_t kInterval = 10000;

    static SpineScores::Parameters Defaults()
    {
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        return SpineScores::Parameters{
            hw->m_sprayGamma,
            hw->m_sprayEpsilon,
            hw->m_sprayCandidates,
            static_cast<SpineScores::CandidateDraw>(hw->m_sprayCandidateDraw),
            kInterval};
    }

    static SpineReport Report(uint8_t sequence, const std::vector<uint8_t>& grades)
    {
        SpineReport report;
        report.sequence = sequence;
        for (uint32_t spine = 0; spine < grades.size(); ++spine)
        {
            report.SetGrade(spine, grades[spine]);
        }
        return report;
    }

    // Reports 1, 2, ... one interval apart, spine 3 graded as given and the
    // others 3; the share of spine 3 after each.
    static std::vector<double> Feed(SpineScores& scores,
                                    const std::vector<uint8_t>& spine3,
                                    uint32_t firstSequence = 1)
    {
        std::vector<double> shares;
        for (uint32_t i = 0; i < spine3.size(); ++i)
        {
            std::vector<uint8_t> grades(kSpines, 3);
            grades[3] = spine3[i];
            const uint32_t sequence = firstSequence + i;
            scores.OnReport(Report(sequence, grades), sequence * kInterval);
            shares.push_back(scores.Share(3));
        }
        return shares;
    }
};

class SpineScoresDrainTest : public TestCase
{
  public:
    SpineScoresDrainTest()
        : TestCase("Scenario 12a: a spine graded 0 drains geometrically, 1/gamma reports to a "
                   "time constant, without oscillation")
    {
    }

    void DoRun() override
    {
        const SpineScores::Parameters defaults = ScoresFixture::Defaults();
        NS_TEST_ASSERT_MSG_EQ(defaults.gamma, 0.25, "the default gamma is the design's");
        SpineScores scores(ScoresFixture::kSpines, defaults);
        const double uniform = 1.0 / ScoresFixture::kSpines;
        NS_TEST_EXPECT_MSG_EQ_TOL(scores.Share(3), uniform, 1e-12, "scores start uniform");
        const uint32_t start = scores.Score(3);
        ScoresFixture::Feed(scores, std::vector<uint8_t>(5, 3));
        NS_TEST_EXPECT_MSG_EQ(scores.Score(3),
                              start,
                              "the top grade holds a score where it starts");

        // Graded 0 from report 6: s_3 falls by 1 - gamma per report.
        std::vector<double> shares = ScoresFixture::Feed(scores, std::vector<uint8_t>(40, 0), 6);
        uint32_t timeConstant = 0;
        double previous = uniform;
        SpineScores replay(ScoresFixture::kSpines, defaults);
        // Until the score reaches zero, after which the share stays at the floor.
        for (uint32_t n = 1; n <= shares.size(); ++n)
        {
            if (n <= 20)
            {
                NS_TEST_EXPECT_MSG_LT(shares[n - 1], previous, "the share falls with every report");
            }
            NS_TEST_EXPECT_MSG_LT_OR_EQ(shares[n - 1], previous, "the share never rises");
            previous = shares[n - 1];
        }
        for (uint32_t n = 1; n <= 12; ++n)
        {
            std::vector<uint8_t> grades(ScoresFixture::kSpines, 3);
            grades[3] = 0;
            replay.OnReport(ScoresFixture::Report(n, grades), n * ScoresFixture::kInterval);
            const double expected = start * std::pow(1 - defaults.gamma, n);
            NS_TEST_EXPECT_MSG_EQ_TOL(replay.Score(3) / expected,
                                      1.0,
                                      0.02,
                                      "the score is (1 - gamma)^n of its start");
            if (timeConstant == 0 && replay.Score(3) <= start / std::exp(1.0))
            {
                timeConstant = n;
            }
        }
        // ln(1 / e) / ln(0.75) is 3.48 reports.
        NS_TEST_EXPECT_MSG_EQ(timeConstant, 4, "the score reaches 1/e in about 1/gamma reports");
        NS_TEST_EXPECT_MSG_EQ_TOL(shares.back(),
                                  defaults.epsilon / ScoresFixture::kSpines,
                                  1e-9,
                                  "the share drains to the exploration floor");
        NS_TEST_EXPECT_MSG_EQ(scores.Score(3), 0, "the score reaches zero");

        // At gamma 0.5 a drain and a recovery are both monotone.
        SpineScores::Parameters half = defaults;
        half.gamma = 0.5;
        SpineScores fast(ScoresFixture::kSpines, half);
        std::vector<uint8_t> grades(20, 0);
        grades.insert(grades.end(), 20, 3);
        shares = ScoresFixture::Feed(fast, grades);
        for (uint32_t n = 1; n < shares.size(); ++n)
        {
            if (n < 20)
            {
                NS_TEST_EXPECT_MSG_LT_OR_EQ(shares[n], shares[n - 1], "the drain never turns back");
            }
            else
            {
                NS_TEST_EXPECT_MSG_GT_OR_EQ(shares[n],
                                            shares[n - 1],
                                            "the recovery never turns back");
                NS_TEST_EXPECT_MSG_LT_OR_EQ(shares[n], uniform + 1e-12, "nor overshoots");
            }
        }
        NS_TEST_EXPECT_MSG_EQ_TOL(shares.back(), uniform, 1e-3, "and returns to uniform");
    }
};

class SpineScoresFloorTest : public TestCase
{
  public:
    SpineScoresFloorTest()
        : TestCase("Scenario 12b: all-zero grades, or no reports, fall back to the uniform floor")
    {
    }

    void DoRun() override
    {
        SpineScores scores(ScoresFixture::kSpines, ScoresFixture::Defaults());
        ScoresFixture::Feed(scores, std::vector<uint8_t>(20, 0));
        NS_TEST_ASSERT_MSG_LT(scores.Share(3), 0.01, "spine 3 starts drained");
        Ptr<UniformRandomVariable> random =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(0));
        uint32_t sequence = 21;
        for (; sequence < 100; ++sequence)
        {
            scores.OnReport(ScoresFixture::Report(sequence, std::vector<uint8_t>(8, 0)),
                            sequence * ScoresFixture::kInterval);
            double sum = 0;
            for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
            {
                const double share = scores.Share(spine);
                NS_TEST_EXPECT_MSG_EQ(std::isfinite(share), true, "a share is always a number");
                sum += share;
            }
            NS_TEST_EXPECT_MSG_EQ_TOL(sum, 1.0, 1e-9, "the shares always sum to one");
            NS_TEST_EXPECT_MSG_LT(uint32_t(scores.Choose(*random, sequence * 10000)),
                                  ScoresFixture::kSpines,
                                  "a draw always names a spine");
        }
        for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(scores.Score(spine),
                                  0,
                                  "without deposits every score reaches zero");
            NS_TEST_EXPECT_MSG_EQ(scores.Share(spine), 1.0 / 8, "then every share is uniform");
        }
        NS_TEST_EXPECT_MSG_EQ(Spread(scores, *random, sequence * 10000),
                              true,
                              "and the draws spread uniformly");

        // Reports that stop: two intervals after the last, each interval
        // decays the scores once, to zero and the uniform floor.
        SpineScores stale(ScoresFixture::kSpines, ScoresFixture::Defaults());
        ScoresFixture::Feed(stale, std::vector<uint8_t>(10, 0));
        const uint64_t last = 10 * ScoresFixture::kInterval;
        const uint32_t before = stale.Score(0);
        stale.Choose(*random, last + 2 * ScoresFixture::kInterval - 1);
        NS_TEST_EXPECT_MSG_EQ(stale.Score(0), before, "a report one interval late decays nothing");
        stale.Choose(*random, last + 2 * ScoresFixture::kInterval);
        NS_TEST_EXPECT_MSG_EQ(stale.Score(0),
                              before - before / 4,
                              "then each interval decays once");
        stale.Choose(*random, last + 200 * ScoresFixture::kInterval);
        NS_TEST_EXPECT_MSG_EQ(stale.Score(0), 0, "until the scores reach zero");
        NS_TEST_EXPECT_MSG_EQ(stale.Share(3), 1.0 / 8, "and the shares are uniform");
        NS_TEST_EXPECT_MSG_EQ(Spread(stale, *random, last + 201 * ScoresFixture::kInterval),
                              true,
                              "so are the draws");
        Simulator::Destroy();
    }

  private:
    // 80000 draws at one instant, each spine within five binomial standard
    // deviations of an eighth.
    static bool Spread(SpineScores& scores, UniformRandomVariable& random, uint64_t now)
    {
        std::vector<uint32_t> counts(ScoresFixture::kSpines, 0);
        for (uint32_t i = 0; i < 80000; ++i)
        {
            ++counts[scores.Choose(random, now)];
        }
        return std::all_of(counts.begin(), counts.end(), [](uint32_t count) {
            return count > 10000 - 5 * 93.5 && count < 10000 + 5 * 93.5;
        });
    }
};

class SpineScoresFlapTest : public TestCase
{
  public:
    SpineScoresFlapTest()
        : TestCase("Scenario 12c: a grade flapping between 2 and 3 holds the share at an "
                   "intermediate ratio")
    {
    }

    void DoRun() override
    {
        SpineScores scores(ScoresFixture::kSpines, ScoresFixture::Defaults());
        // At gamma 0.25 the flapping score settles between 4.25 / 0.4375 and
        // 4.5 / 0.4375 grades, against 12 for a healthy spine: shares of 0.81
        // and 0.86 of a healthy spine's, where the grades are 2/3 and 1 of it.
        double low = 1, high = 0;
        for (uint32_t sequence = 1; sequence <= 60; ++sequence)
        {
            std::vector<uint8_t> grades(ScoresFixture::kSpines, 3);
            grades[3] = sequence % 2 == 0 ? 3 : 2;
            scores.OnReport(ScoresFixture::Report(sequence, grades),
                            sequence * ScoresFixture::kInterval);
            const double ratio = scores.Share(3) / scores.Share(0);
            if (sequence > 40)
            {
                low = std::min(low, ratio);
                high = std::max(high, ratio);
            }
        }
        NS_TEST_EXPECT_MSG_GT(low, 0.78, "the share settles above grade 2's ratio");
        NS_TEST_EXPECT_MSG_LT(high, 0.89, "and below grade 3's");
        NS_TEST_EXPECT_MSG_LT(high - low,
                              (1 - 2.0 / 3) / 4,
                              "its swing is a fraction of the grades'");
    }
};

class SpineScoresDrawTest : public TestCase
{
  public:
    SpineScoresDrawTest()
        : TestCase("A spine is drawn in proportion to its share, the best of k candidates "
                   "under k > 1, and a report is taken once and not under the edge bit")
    {
    }

    void DoRun() override
    {
        Proportional();
        Candidates();
        Reports();
        Selector();
        Simulator::Destroy();
    }

  private:
    static Ptr<UniformRandomVariable> Stream()
    {
        return CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(0));
    }

    // Spines 0 to 7 graded 0 to 3 twice over, until the scores settle.
    static SpineScores Graded(const SpineScores::Parameters& parameters)
    {
        SpineScores scores(ScoresFixture::kSpines, parameters);
        for (uint32_t sequence = 1; sequence < 40; ++sequence)
        {
            scores.OnReport(ScoresFixture::Report(sequence, {0, 1, 2, 3, 0, 1, 2, 3}),
                            sequence * ScoresFixture::kInterval);
        }
        return scores;
    }

    // k = 1 is the inverse of p's distribution at one uniform draw per packet.
    void Proportional()
    {
        SpineScores scores = Graded(ScoresFixture::Defaults());
        Ptr<UniformRandomVariable> drawn = Stream();
        Ptr<UniformRandomVariable> replayed = Stream();
        // The floor's draws spread evenly over the spines and the others in
        // proportion to the scores, which as one distribution is p.
        std::vector<double> byScore;
        double sum = 0;
        for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
        {
            sum += scores.Score(spine);
            byScore.push_back(sum);
        }
        std::vector<uint32_t> counts(ScoresFixture::kSpines, 0);
        const uint32_t kDraws = 100000;
        uint32_t disagree = 0;
        for (uint32_t i = 0; i < kDraws; ++i)
        {
            const uint8_t spine = scores.Choose(*drawn, 400000);
            ++counts[spine];
            const double u = replayed->GetValue();
            const uint32_t expected =
                u < 0.02
                    ? uint32_t(u / 0.02 * 8)
                    : std::upper_bound(byScore.begin(), byScore.end(), (u - 0.02) / 0.98 * sum) -
                          byScore.begin();
            disagree += expected != spine;
        }
        NS_TEST_EXPECT_MSG_LT(disagree, 10, "one draw per packet, mapped through p");
        NS_TEST_EXPECT_MSG_EQ(drawn->GetValue(), replayed->GetValue(), "and no draw more");
        for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
        {
            const double p = scores.Share(spine);
            const double sd = std::sqrt(kDraws * p * (1 - p));
            NS_TEST_EXPECT_MSG_EQ_TOL(double(counts[spine]),
                                      kDraws * p,
                                      5 * sd + 1,
                                      "each spine is drawn at its share");
        }
        NS_TEST_EXPECT_MSG_GT(counts[0], 0, "a spine graded 0 keeps the floor");
    }

    void Candidates()
    {
        // One spine scored zero among eight healthy ones.
        auto oneDown = [](SpineScores::CandidateDraw draw, uint32_t k) {
            SpineScores::Parameters parameters = ScoresFixture::Defaults();
            parameters.candidates = k;
            parameters.candidateDraw = draw;
            SpineScores scores(ScoresFixture::kSpines, parameters);
            ScoresFixture::Feed(scores, std::vector<uint8_t>(60, 0));
            Ptr<UniformRandomVariable> random = Stream();
            uint32_t onDown = 0;
            for (uint32_t i = 0; i < 64000; ++i)
            {
                onDown += scores.Choose(*random, 600000) == 3;
            }
            return onDown;
        };
        // Uniform candidates: k = 1 is uniform spraying, and at k = 2 spine 3
        // wins only when both candidates are spine 3, one draw in 64.
        NS_TEST_EXPECT_MSG_EQ_TOL(oneDown(SpineScores::CandidateDraw::Uniform, 1),
                                  8000,
                                  5 * 83.7,
                                  "one uniform candidate is uniform spraying");
        NS_TEST_EXPECT_MSG_EQ_TOL(oneDown(SpineScores::CandidateDraw::Uniform, 2),
                                  1000,
                                  5 * 31.4,
                                  "a spine scored below the rest wins only as both candidates");
        // Proportional candidates: spine 3's floor of 0.0025 to the power k.
        NS_TEST_EXPECT_MSG_EQ_TOL(oneDown(SpineScores::CandidateDraw::Proportional, 1),
                                  160,
                                  5 * 12.6,
                                  "one proportional candidate is the floor");
        NS_TEST_EXPECT_MSG_LT(oneDown(SpineScores::CandidateDraw::Proportional, 2),
                              5,
                              "two proportional candidates square it");

        // Among graded spines, more candidates favour the best.
        auto bestShare = [](uint32_t k) {
            SpineScores::Parameters parameters = ScoresFixture::Defaults();
            parameters.candidates = k;
            SpineScores scores = Graded(parameters);
            Ptr<UniformRandomVariable> random = Stream();
            uint32_t best = 0;
            for (uint32_t i = 0; i < 40000; ++i)
            {
                const uint8_t spine = scores.Choose(*random, 400000);
                best += spine == 3 || spine == 7;
            }
            return best / 40000.0;
        };
        const double one = bestShare(1);
        const double two = bestShare(2);
        const double three = bestShare(3);
        NS_TEST_EXPECT_MSG_EQ_TOL(one,
                                  0.02 * 2 / 8 + 0.98 * 2 * 3 / 12.0,
                                  0.01,
                                  "one candidate draws the best spines at their share");
        NS_TEST_EXPECT_MSG_GT(two, one + 0.1, "two candidates favour them");
        NS_TEST_EXPECT_MSG_GT(three, two + 0.05, "three more so");
    }

    void Reports()
    {
        SpineScores scores(ScoresFixture::kSpines, ScoresFixture::Defaults());
        const uint32_t start = scores.Score(0);
        std::vector<uint8_t> grades(8, 3);
        grades[0] = 0;
        scores.OnReport(ScoresFixture::Report(10, grades), 100000);
        const uint32_t once = scores.Score(0);
        NS_TEST_EXPECT_MSG_LT(once, start, "a report is taken");
        scores.OnReport(ScoresFixture::Report(10, grades), 100100);
        NS_TEST_EXPECT_MSG_EQ(scores.Score(0), once, "the same report is taken once");
        scores.OnReport(ScoresFixture::Report(9, grades), 100200);
        NS_TEST_EXPECT_MSG_EQ(scores.Score(0), once, "an older one is not taken");
        SpineReport edge = ScoresFixture::Report(11, grades);
        edge.edgeCongested = true;
        scores.OnReport(edge, 110000);
        NS_TEST_EXPECT_MSG_EQ(scores.Score(0), once, "a report with the edge bit moves no score");
        scores.OnReport(ScoresFixture::Report(12, grades), 120000);
        NS_TEST_EXPECT_MSG_LT(scores.Score(0), once, "the next report without it is taken");
        // The sequence wraps.
        for (uint32_t sequence = 13; sequence < 13 + 300; ++sequence)
        {
            const uint32_t previous = scores.Score(1);
            grades[1] = sequence % 2 == 0 ? 3 : 0;
            scores.OnReport(ScoresFixture::Report(sequence % 256, grades), sequence * 10000);
            NS_TEST_EXPECT_MSG_NE(scores.Score(1), previous, "each newer report is taken");
        }
        // After half the sequence's range without a report, any report is the
        // latest.
        const uint32_t before = scores.Score(1);
        grades[1] = 3;
        scores.OnReport(ScoresFixture::Report(0, grades), (313 + 128) * 10000);
        NS_TEST_EXPECT_MSG_NE(scores.Score(1), before, "a report after a long silence is taken");
    }

    // Two queue pairs to one host draw from one set of scores, which a report
    // to either moves; a queue pair to another host draws from its own.
    void Selector()
    {
        SpineScores toFirst(ScoresFixture::kSpines, ScoresFixture::Defaults());
        SpineScores toSecond(ScoresFixture::kSpines, ScoresFixture::Defaults());
        PolicySpineSelector a(Stream(), toFirst);
        PolicySpineSelector b(Stream(), toFirst);
        PolicySpineSelector c(Stream(), toSecond);
        std::vector<uint8_t> grades(8, 0);
        grades[6] = 3;
        for (uint32_t sequence = 1; sequence < 40; ++sequence)
        {
            a.OnReport(ScoresFixture::Report(sequence, grades), sequence * 10000);
        }
        uint32_t onSix = 0;
        for (uint32_t i = 0; i < 1000; ++i)
        {
            const uint16_t identification = b.Choose(400000);
            NS_TEST_ASSERT_MSG_EQ(RequestedSpine(identification),
                                  CarryingSpine(identification),
                                  "the request names its spine in both bytes");
            onSix += RequestedSpine(identification) == 6;
        }
        NS_TEST_EXPECT_MSG_GT(onSix, 950, "a report to one queue pair steers the other");
        uint32_t cOnSix = 0;
        for (uint32_t i = 0; i < 1000; ++i)
        {
            cOnSix += RequestedSpine(c.Choose(400000)) == 6;
        }
        NS_TEST_EXPECT_MSG_LT(cOnSix, 200, "a queue pair to another host keeps its own scores");
    }
};

class SprayPolicyTransportTest : public TestCase
{
  public:
    SprayPolicyTransportTest()
        : TestCase("Under spray_policy every answer carries the receiver's latest report and the "
                   "sender's scores and window follow it")
    {
    }

    void DoRun() override
    {
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        const uint32_t savedReportBytes = CustomHeader::ackReportBytes;
        CustomHeader::ackCarriesPacketSeq = true;
        CustomHeader::ackReportBytes = 1 + SpineReport::GradeBytes(8);
        Receiver();
        Sender();
        EdgePenalty();
        CustomHeader::ackReportBytes = savedReportBytes;
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
    }

  private:
    static constexpr uint64_t kBaseRtt = 10000;

    // A receiver with two spines graded apart: spine 2 marked on every packet,
    // spine 5 trimmed once, the rest clean.
    void Receiver()
    {
        IsolatedHost receiver(LoadBalancingMode::SprayPolicy, kTestSender, 8);
        Ptr<Node> node = CreateObject<Node>();
        receiver.hw->SetNode(node);
        receiver.hw->m_sprayBaseRttNs = kBaseRtt;
        const uint64_t interval = receiver.hw->m_sprayReportIntervalBaseRtts * kBaseRtt;
        // The receiver's first interval ends a fraction of the golden ratio per
        // node id into the second.
        const uint64_t phase = std::fmod(node->GetId() * 0.6180339887498949, 1.0) * interval;
        uint32_t seq = 0;
        auto deliver = [&](uint8_t spine, bool marked) {
            receiver.ReceiveData(kTestSender,
                                 kTestReceiver,
                                 seq,
                                 SpineIdentification(spine, spine),
                                 marked);
            seq += IsolatedHost::kMtu;
        };
        // Three intervals, eight packets per spine each, then one packet.
        for (uint32_t i = 0; i < 3; ++i)
        {
            Simulator::Schedule(NanoSeconds(phase + i * interval + 1), [&]() {
                for (uint8_t spine = 0; spine < 8; ++spine)
                {
                    for (uint32_t j = 0; j < 8; ++j)
                    {
                        deliver(spine, spine == 2);
                    }
                }
            });
        }
        Simulator::Schedule(NanoSeconds(phase + interval + 2), [&]() {
            Trim(receiver, seq, 5);
            seq += IsolatedHost::kMtu;
        });
        Simulator::Schedule(NanoSeconds(phase + 3 * interval + 1), [&]() { deliver(0, false); });
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(receiver.emitted.size(), 3 * 64 + 2, "every packet is answered");
        const CustomHeader& last = receiver.emitted.back();
        NS_TEST_EXPECT_MSG_EQ(last.l3Prot, 0xFC, "the last answer is an acknowledgement");
        const SpineReport& report = receiver.hw->Grader().Report();
        NS_TEST_EXPECT_MSG_EQ(uint32_t(report.sequence), 3u, "three intervals closed");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(2)), 0u, "the marked spine is graded down");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(5)), 0u, "the trimmed spine is held down");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(1)), 3u, "a clean spine keeps the top grade");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(last.ack.spine_report[0]),
                              uint32_t(report.sequence),
                              "the acknowledgement carries the latest report");
        SpineReport wire;
        std::copy_n(last.ack.spine_report + 1, 2, wire.grades);
        for (uint32_t spine = 0; spine < 8; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(uint32_t(wire.Grade(spine)),
                                  uint32_t(report.Grade(spine)),
                                  "and its grades");
        }
        NS_TEST_EXPECT_MSG_EQ(((last.ack.flags >> qbbHeader::FLAG_EDGE_CONGESTED) & 1),
                              0,
                              "two spines graded down are not the downlink");
        // The answers of an interval carry the report of the one before.
        const CustomHeader& first = receiver.emitted.front();
        NS_TEST_EXPECT_MSG_EQ(uint32_t(first.ack.spine_report[0]),
                              0u,
                              "the first interval's answers carry the starting report");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(first.ack.spine_report[1]),
                              0xffu,
                              "which grades every spine at the top");
        const CustomHeader& repair = receiver.emitted[2 * 64];
        NS_TEST_EXPECT_MSG_EQ(repair.l3Prot, kUecTrimRepairProtocol, "the trim is answered");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(repair.ack.spine_report[0]),
                              1u,
                              "the repair request carries the latest report");
        std::copy_n(repair.ack.spine_report + 1, 2, wire.grades);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(wire.Grade(2)), 0u, "with the marked spine graded down");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(wire.Grade(5)), 3u, "before its own trim is graded");
        Simulator::Destroy();
    }

    // A data packet trimmed on spine before the last hop, as its destination
    // parses it.
    static void Trim(IsolatedHost& receiver, uint32_t seq, uint8_t spine)
    {
        CustomHeader trimmed;
        trimmed.l3Prot = 0x11;
        trimmed.m_tos = kUetDscpTrimmed << 2;
        trimmed.sip = kTestSender.Get();
        trimmed.dip = kTestReceiver.Get();
        trimmed.ipid = SpineIdentification(spine, spine);
        trimmed.udp.sport = IsolatedHost::kSenderPort;
        trimmed.udp.dport = IsolatedHost::kReceiverPort;
        trimmed.udp.pg = IsolatedHost::kPriorityGroup;
        trimmed.udp.seq = seq;
        trimmed.udp.payload_size = CustomHeader::GetUdpHeaderSize() + IsolatedHost::kMtu;
        receiver.hw->Receive(Create<Packet>(), trimmed);
    }

    void Sender()
    {
        IsolatedHost sender(LoadBalancingMode::SprayPolicy, kTestReceiver, 8);
        sender.hw->m_sprayBaseRttNs = kBaseRtt;
        Ptr<RdmaQueuePair> first =
            sender.AddSender(kTestSender, kTestReceiver, 100 * IsolatedHost::kMtu);
        for (uint32_t i = 0; i < 4; ++i)
        {
            sender.hw->GetNxtPacket(first);
        }
        NS_TEST_ASSERT_MSG_EQ(sender.hw->m_spineScores.size(),
                              1,
                              "one destination, one set of scores");
        const SpineScores& scores = sender.hw->m_spineScores.begin()->second;
        const uint32_t start = scores.Score(4);
        CustomHeader ack;
        ack.l3Prot = 0xFC;
        ack.sip = kTestReceiver.Get();
        ack.ack.sport = IsolatedHost::kReceiverPort;
        ack.ack.dport = IsolatedHost::kSenderPort;
        ack.ack.pg = IsolatedHost::kPriorityGroup;
        ack.ack.flags = 0;
        ack.ack.seq = 0;
        ack.ack.packet_seq = 0;
        SpineReport report;
        report.sequence = 1;
        report.SetGrade(4, 0);
        ack.ack.spine_report[0] = report.sequence;
        std::copy_n(report.grades, 2, ack.ack.spine_report + 1);
        sender.hw->ReceiveAck(Create<Packet>(), ack);
        NS_TEST_EXPECT_MSG_LT(scores.Score(4),
                              start,
                              "an acknowledgement's report moves the scores");
        NS_TEST_EXPECT_MSG_EQ(scores.Score(3), start, "the top grade keeps a score");
        const uint32_t after = scores.Score(4);
        ack.ack.trim_payload_size = IsolatedHost::kMtu;
        ack.l3Prot = kUecTrimRepairProtocol;
        ack.ack.spine_report[0] = 2;
        sender.hw->RecoverTrimmedQueue(first, ack);
        NS_TEST_EXPECT_MSG_LT(scores.Score(4), after, "so does a repair request's");
        Simulator::Destroy();
    }

    // Under NSCC, an acknowledgement whose report has the edge bit set hands
    // the window SprayEdgeWindowPenalty as the destination's Rcv_Cwnd_Pend.
    void EdgePenalty()
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        auto windowAfter = [](bool edge) {
            IsolatedHost sender(LoadBalancingMode::SprayPolicy, kTestReceiver, 8);
            sender.hw->m_sprayBaseRttNs = kBaseRtt;
            sender.hw->m_cc_mode = 11;
            Ptr<RdmaQueuePair> qp = sender.AddSender(kTestSender, kTestReceiver, 100 * kMtu);
            qp->nscc.Start(sender.hw->NsccParameters(), kMtu, 50 * kMtu, kBaseRtt, 0);
            CustomHeader sent(CustomHeader::L2_Header | CustomHeader::L3_Header);
            sender.hw->GetNxtPacket(qp)->PeekHeader(sent);
            for (uint32_t i = 0; i < 9; ++i)
            {
                sender.hw->GetNxtPacket(qp);
            }
            CustomHeader ack;
            ack.l3Prot = 0xFC;
            ack.sip = kTestReceiver.Get();
            ack.ipid = sent.ipid;
            ack.ack.sport = IsolatedHost::kReceiverPort;
            ack.ack.dport = IsolatedHost::kSenderPort;
            ack.ack.pg = IsolatedHost::kPriorityGroup;
            ack.ack.flags = edge ? 1 << qbbHeader::FLAG_EDGE_CONGESTED : 0;
            ack.ack.seq = kMtu;
            ack.ack.packet_seq = 0;
            ack.ack.spine_report[0] = 1;
            std::fill_n(ack.ack.spine_report + 1, 2, 0xff);
            sender.hw->ReceiveAck(Create<Packet>(), ack);
            const uint64_t window = qp->nscc.Cwnd();
            Simulator::Destroy();
            return window;
        };
        // Nine packets in flight after the answer, less 64/128 of the one it
        // answers.
        NS_TEST_EXPECT_MSG_EQ(windowAfter(true),
                              9 * kMtu - kMtu / 2,
                              "the edge bit holds the window down");
        NS_TEST_EXPECT_MSG_GT(windowAfter(false), 9 * kMtu, "without it the window is left alone");
    }
};

class NsccReceiverPenaltyTest : public TestCase
{
  public:
    NsccReceiverPenaltyTest()
        : TestCase("NSCC follows a destination's Rcv_Cwnd_Pend: the window falls with every "
                   "acknowledgement and does not grow")
    {
    }

    void DoRun() override
    {
        constexpr uint64_t kNow = 2 * NsccPath::kBaseRtt;
        constexpr uint64_t kWindow = 1000000;
        constexpr uint64_t kInflight = 800000;
        NsccWindow window = NsccPath::Started(NsccPath::WithoutEta());
        window.OnLoss(NsccPath::kCeiling - kWindow);
        // Half a packet per packet acknowledged, from the bytes in flight.
        window.OnAck(NsccPath::kMtu, false, NsccPath::kBaseRtt + 2000, kInflight, kNow, 64);
        NS_TEST_EXPECT_MSG_EQ(window.Cwnd(),
                              kInflight - NsccPath::kMtu / 2,
                              "the window drops to what is in flight less pend/128 of the bytes");
        for (uint32_t i = 0; i < 8; ++i)
        {
            window.OnAck(NsccPath::kMtu, false, NsccPath::kBaseRtt + 2000, kInflight, kNow, 64);
        }
        NS_TEST_EXPECT_MSG_EQ(window.Cwnd(),
                              kInflight - 9 * NsccPath::kMtu / 2,
                              "and grows by neither increase while held down");
        const uint64_t held = window.Cwnd();
        for (uint32_t i = 0; i < 9; ++i)
        {
            window.OnAck(NsccPath::kMtu, false, NsccPath::kBaseRtt + 2000, kInflight, kNow, 0);
        }
        NS_TEST_EXPECT_MSG_GT(window.Cwnd(), held, "with the penalty gone it grows again");
        NsccWindow floor = NsccPath::Started(NsccPath::WithoutEta());
        floor.OnAck(NsccPath::kMtu, false, NsccPath::kBaseRtt, 0, kNow, 127);
        NS_TEST_EXPECT_MSG_EQ(floor.Cwnd(), NsccPath::kMtu, "the window keeps one packet");
    }
};

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
    AddTestCase(new MarkingStreamTest, TestCase::Duration::QUICK);
    AddTestCase(new PfcIdentificationTest, TestCase::Duration::QUICK);
    AddTestCase(new LoadBalancingSenderTest, TestCase::Duration::QUICK);
    AddTestCase(new AckPacketSeqHeaderTest, TestCase::Duration::QUICK);
    AddTestCase(new AckNamesPacketTest, TestCase::Duration::QUICK);
    AddTestCase(new PathSelectorHooksTest, TestCase::Duration::QUICK);
    AddTestCase(new PathProbeTest, TestCase::Duration::QUICK);
    AddTestCase(new RepsSelectorTest, TestCase::Duration::QUICK);
    AddTestCase(new UeSelectorTest, TestCase::Duration::QUICK);
    AddTestCase(new MrcSelectorTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineArrivalsTest, TestCase::Duration::QUICK);
    AddTestCase(new ReorderGapTest, TestCase::Duration::QUICK);
    AddTestCase(new OutstandingPacketsModelTest, TestCase::Duration::QUICK);
    AddTestCase(new SelectiveTimeoutTest, TestCase::Duration::QUICK);
    AddTestCase(new OutstandingWindowTest, TestCase::Duration::QUICK);
    AddTestCase(new TrimRepairedOnceTest, TestCase::Duration::QUICK);
    AddTestCase(new DuplicateRepairTest, TestCase::Duration::QUICK);
    AddTestCase(new PathLossRuleTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccCaseTableTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccLightMarkTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccQuickAdaptTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccFastIncreaseTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccCutTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccBoundsTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccRttTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineReportHeaderTest, TestCase::Duration::QUICK);
    AddTestCase(new SupervisedEwmaTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineAttributionTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineGradeTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineScoresDrainTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineScoresFloorTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineScoresFlapTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineScoresDrawTest, TestCase::Duration::QUICK);
    AddTestCase(new SprayPolicyTransportTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccReceiverPenaltyTest, TestCase::Duration::QUICK);
}

static PointToPointTestSuite g_pointToPointTestSuite; //!< The testsuite
