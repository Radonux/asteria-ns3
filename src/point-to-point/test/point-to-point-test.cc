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
#include "ns3/link-events.h"
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
#include "ns3/spine-redirect.h"
#include "ns3/switch-queue-state.h"
#include "ns3/error-model.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/udp-header.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
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
 * A link from a host to a switch port, failed and restored by link events.
 */
class LinkRestoreTest : public TestCase
{
  public:
    LinkRestoreTest()
        : TestCase("A link restore puts back the rate, loss or black hole its latest failure "
                   "replaced, refuses a downed link, and applies before a failure at its time")
    {
    }

    void DoRun() override
    {
        Order();
        Check();
        Restores();
        Schedule();
        Simulator::Destroy();
    }

  private:
    static LinkEvent Event(uint64_t timeNs, LinkEventKind kind, DataRate rate = DataRate())
    {
        LinkEvent event;
        event.timeNs = timeNs;
        event.kind = kind;
        event.a = 0;
        event.b = 1;
        event.rate = rate;
        return event;
    }

    struct Link
    {
        Ptr<SwitchNode> sw;
        LinkEnds ends;
    };

    // Node 0 a host at 400 Gb/s, node 1 a switch at 200 Gb/s.
    static Link MakeLink()
    {
        Link link;
        link.sw = CreateObject<SwitchNode>();
        link.sw->SetAttribute("PfcEnabled", BooleanValue(false));
        Ptr<QbbNetDevice> port = AttachSwitchPort(link.sw, CreateObject<Node>());
        Ptr<QbbChannel> channel = DynamicCast<QbbChannel>(port->GetChannel());
        link.ends.atB = port;
        link.ends.atA = DynamicCast<QbbNetDevice>(
            channel->GetDevice(0) == port ? channel->GetDevice(1) : channel->GetDevice(0));
        link.ends.switchAtB = link.sw;
        link.ends.atA->SetDataRate(DataRate("400Gbps"));
        link.ends.atB->SetDataRate(DataRate("200Gbps"));
        return link;
    }

    static LinkEventApplier Applier(uint32_t* downs = nullptr)
    {
        return LinkEventApplier(
            [](Ptr<QbbNetDevice> device, double rate) {
                PointerValue current;
                device->GetAttribute("LinkErrorModel", current);
                Ptr<RateErrorModel> model = current.Get<RateErrorModel>();
                if (!model)
                {
                    model = CreateObject<RateErrorModel>();
                    model->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
                    device->SetAttribute("LinkErrorModel", PointerValue(model));
                }
                model->SetRate(rate);
            },
            [downs](uint32_t, uint32_t) {
                if (downs != nullptr)
                {
                    ++*downs;
                }
            });
    }

    static Ptr<RateErrorModel> ErrorModelOf(Ptr<QbbNetDevice> device)
    {
        PointerValue current;
        device->GetAttribute("LinkErrorModel", current);
        return current.Get<RateErrorModel>();
    }

    void Order()
    {
        std::vector<LinkEvent> events = {Event(20, LinkEventKind::Rate),
                                         Event(20, LinkEventKind::Restore),
                                         Event(10, LinkEventKind::Loss),
                                         Event(30, LinkEventKind::Restore),
                                         Event(20, LinkEventKind::Blackhole)};
        OrderLinkEvents(events);
        const std::vector<std::pair<uint64_t, LinkEventKind>> expected = {
            {10, LinkEventKind::Loss},
            {20, LinkEventKind::Restore},
            {20, LinkEventKind::Rate},
            {20, LinkEventKind::Blackhole},
            {30, LinkEventKind::Restore}};
        for (uint32_t i = 0; i < expected.size(); ++i)
        {
            NS_TEST_EXPECT_MSG_EQ(events[i].timeNs, expected[i].first, "by time");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(events[i].kind),
                                  uint32_t(expected[i].second),
                                  "a restore first at its time, the failures as given");
        }
    }

    void Check()
    {
        NS_TEST_EXPECT_MSG_EQ(
            CheckLinkEvents({Event(10, LinkEventKind::Rate), Event(20, LinkEventKind::Restore)}),
            "",
            "a restore after a failure applies");
        NS_TEST_EXPECT_MSG_NE(
            CheckLinkEvents({Event(10, LinkEventKind::Down), Event(20, LinkEventKind::Restore)}),
            "",
            "a downed link is not restored");
        NS_TEST_EXPECT_MSG_NE(CheckLinkEvents({Event(10, LinkEventKind::Down),
                                               Event(15, LinkEventKind::Rate),
                                               Event(20, LinkEventKind::Restore)}),
                              "",
                              "nor after a later failure");
        NS_TEST_EXPECT_MSG_NE(CheckLinkEvents({Event(20, LinkEventKind::Restore)}),
                              "",
                              "a restore needs a failure before it");
        LinkEvent reversed = Event(20, LinkEventKind::Restore);
        std::swap(reversed.a, reversed.b);
        NS_TEST_EXPECT_MSG_EQ(CheckLinkEvents({Event(10, LinkEventKind::Loss), reversed}),
                              "",
                              "a link is named in either order");
    }

    void Restores()
    {
        Link link = MakeLink();
        uint32_t downs = 0;
        LinkEventApplier applier = Applier(&downs);

        applier.Apply(Event(0, LinkEventKind::Rate, DataRate("100Gbps")), link.ends);
        NS_TEST_EXPECT_MSG_EQ(link.ends.atA->GetDataRate(), DataRate("100Gbps"), "slowed at a");
        NS_TEST_EXPECT_MSG_EQ(link.ends.atB->GetDataRate(), DataRate("100Gbps"), "slowed at b");
        applier.Apply(Event(0, LinkEventKind::Restore), link.ends);
        NS_TEST_EXPECT_MSG_EQ(link.ends.atA->GetDataRate(), DataRate("400Gbps"), "a's rate back");
        NS_TEST_EXPECT_MSG_EQ(link.ends.atB->GetDataRate(), DataRate("200Gbps"), "b's rate back");

        LinkEvent loss = Event(0, LinkEventKind::Loss);
        loss.errorRate = 0.5;
        applier.Apply(loss, link.ends);
        NS_TEST_ASSERT_MSG_NE(ErrorModelOf(link.ends.atA), nullptr, "a loss adds a model");
        NS_TEST_EXPECT_MSG_EQ(ErrorModelOf(link.ends.atB)->GetRate(), 0.5, "at its rate");
        applier.Apply(Event(0, LinkEventKind::Restore), link.ends);
        NS_TEST_EXPECT_MSG_EQ(ErrorModelOf(link.ends.atA), nullptr, "the added model is removed");
        NS_TEST_EXPECT_MSG_EQ(ErrorModelOf(link.ends.atB), nullptr, "at both ends");

        Ptr<RateErrorModel> standing = CreateObject<RateErrorModel>();
        standing->SetRate(0.01);
        link.ends.atA->SetAttribute("LinkErrorModel", PointerValue(standing));
        loss.errorRate = 0.25;
        applier.Apply(loss, link.ends);
        NS_TEST_EXPECT_MSG_EQ(standing->GetRate(), 0.25, "a standing model takes the loss");
        applier.Apply(Event(0, LinkEventKind::Restore), link.ends);
        NS_TEST_EXPECT_MSG_EQ(ErrorModelOf(link.ends.atA), standing, "the standing model stays");
        NS_TEST_EXPECT_MSG_EQ(standing->GetRate(), 0.01, "at its former rate");
        NS_TEST_EXPECT_MSG_EQ(ErrorModelOf(link.ends.atB), nullptr, "b had none");

        const uint32_t port = link.ends.atB->GetIfIndex();
        applier.Apply(Event(0, LinkEventKind::Blackhole), link.ends);
        NS_TEST_EXPECT_MSG_EQ(link.sw->IsBlackholed(port), true, "the port is a black hole");
        applier.Apply(Event(0, LinkEventKind::Restore), link.ends);
        NS_TEST_EXPECT_MSG_EQ(link.sw->IsBlackholed(port), false, "and no longer");

        applier.Apply(Event(0, LinkEventKind::Down), link.ends);
        NS_TEST_EXPECT_MSG_EQ(downs, 1, "a down takes the link down");
    }

    // A repeat schedule given out of order, with a failure at a restore's
    // time listed before it.
    void Schedule()
    {
        Link link = MakeLink();
        LinkEventApplier applier = Applier();
        std::vector<LinkEvent> events = {Event(400, LinkEventKind::Rate, DataRate("25Gbps")),
                                         Event(100, LinkEventKind::Rate, DataRate("100Gbps")),
                                         Event(200, LinkEventKind::Rate, DataRate("50Gbps")),
                                         Event(200, LinkEventKind::Restore),
                                         Event(300, LinkEventKind::Restore)};
        OrderLinkEvents(events);
        NS_TEST_ASSERT_MSG_EQ(CheckLinkEvents(events), "", "the schedule applies");
        for (const LinkEvent& event : events)
        {
            Simulator::Schedule(NanoSeconds(event.timeNs),
                                [&applier, &link, event]() { applier.Apply(event, link.ends); });
        }
        std::vector<uint64_t> rates;
        for (uint64_t at : {50, 150, 250, 350, 450})
        {
            Simulator::Schedule(NanoSeconds(at), [&rates, &link]() {
                rates.push_back(link.ends.atA->GetDataRate().GetBitRate());
            });
        }
        Simulator::Run();
        const std::vector<uint64_t> expected = {400000000000,
                                                100000000000,
                                                50000000000,
                                                400000000000,
                                                25000000000};
        NS_TEST_ASSERT_MSG_EQ(rates.size(), expected.size(), "every probe ran");
        for (uint32_t i = 0; i < expected.size(); ++i)
        {
            NS_TEST_EXPECT_MSG_EQ(rates[i], expected[i], "the rate follows the schedule in order");
        }
        Simulator::Destroy();
    }
};

/**
 * Two switches, a and b, linked to each other, each with a host behind it,
 * failed in the direction from a to b alone.
 */
class DirectionalLinkFaultTest : public TestCase
{
  public:
    DirectionalLinkFaultTest()
        : TestCase("A forward link loss or rate applies in the direction from the first node "
                   "to the second alone, its restore puts back the end it set, and failures of "
                   "the two directions of a link restore independently")
    {
    }

    void DoRun() override
    {
        Loss();
        Rate();
        Blackholes();
        Check();
    }

  private:
    struct Fabric
    {
        Ptr<SwitchNode> a, b;
        // a's port toward b and b's toward a.
        Ptr<QbbNetDevice> atA, atB;
        // The ports toward the hosts behind a and b.
        Ptr<QbbNetDevice> hostPortA, hostPortB;
        Ipv4Address hostA{"11.0.1.1"}, hostB{"11.0.2.1"};
        LinkEnds ends;
    };

    static Ptr<QbbNetDevice> SwitchPort(Ptr<SwitchNode> sw)
    {
        Ptr<QbbNetDevice> port = CreateObject<QbbNetDevice>();
        port->SetQueue(CreateObject<BEgressQueue>());
        port->SetDataRate(DataRate("400Gbps"));
        sw->AddDevice(port);
        return port;
    }

    static Ptr<QbbNetDevice> HostPort(Ptr<SwitchNode> sw)
    {
        Ptr<QbbNetDevice> port = SwitchPort(sw);
        Ptr<QbbNetDevice> host = CreateObject<QbbNetDevice>();
        host->SetDataRate(DataRate("400Gbps"));
        CreateObject<Node>()->AddDevice(host);
        host->m_rdmaReceiveCb =
            Callback<int, Ptr<Packet>, CustomHeader&>([](Ptr<Packet>, CustomHeader&) { return 0; });
        Ptr<QbbChannel> channel = CreateObject<QbbChannel>();
        port->Attach(channel);
        host->Attach(channel);
        return port;
    }

    static Fabric MakeFabric()
    {
        Fabric fabric;
        for (Ptr<SwitchNode>* sw : {&fabric.a, &fabric.b})
        {
            *sw = CreateObject<SwitchNode>();
            (*sw)->SetAttribute("PfcEnabled", BooleanValue(false));
        }
        fabric.atA = SwitchPort(fabric.a);
        fabric.atB = SwitchPort(fabric.b);
        Ptr<QbbChannel> channel = CreateObject<QbbChannel>();
        fabric.atA->Attach(channel);
        fabric.atB->Attach(channel);
        fabric.hostPortA = HostPort(fabric.a);
        fabric.hostPortB = HostPort(fabric.b);
        fabric.a->AddTableEntry(fabric.hostB, fabric.atA->GetIfIndex());
        fabric.a->AddTableEntry(fabric.hostA, fabric.hostPortA->GetIfIndex());
        fabric.b->AddTableEntry(fabric.hostA, fabric.atB->GetIfIndex());
        fabric.b->AddTableEntry(fabric.hostB, fabric.hostPortB->GetIfIndex());
        fabric.ends.atA = fabric.atA;
        fabric.ends.atB = fabric.atB;
        fabric.ends.switchAtB = fabric.b;
        return fabric;
    }

    static LinkEventApplier Applier()
    {
        return LinkEventApplier(
            [](Ptr<QbbNetDevice> device, double rate) {
                PointerValue current;
                device->GetAttribute("LinkErrorModel", current);
                Ptr<RateErrorModel> model = current.Get<RateErrorModel>();
                if (!model)
                {
                    model = CreateObject<RateErrorModel>();
                    model->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
                    device->SetAttribute("LinkErrorModel", PointerValue(model));
                }
                model->SetRate(rate);
            },
            [](uint32_t, uint32_t) {});
    }

    static LinkEvent Event(LinkEventKind kind, LinkDirection direction)
    {
        LinkEvent event;
        event.kind = kind;
        event.a = 0;
        event.b = 1;
        event.direction = direction;
        return event;
    }

    // Priority group 0 bypasses buffer admission, which is not under test.
    static void Send(Fabric& fabric, bool fromA, uint32_t packets)
    {
        for (uint32_t i = 0; i < packets; ++i)
        {
            if (fromA)
            {
                ArriveAtSwitch(fabric.a, fabric.hostPortA, fabric.hostA, fabric.hostB, 0);
            }
            else
            {
                ArriveAtSwitch(fabric.b, fabric.hostPortB, fabric.hostB, fabric.hostA, 0);
            }
        }
    }

    static Ptr<RateErrorModel> ErrorModelOf(Ptr<QbbNetDevice> device)
    {
        PointerValue current;
        device->GetAttribute("LinkErrorModel", current);
        return current.Get<RateErrorModel>();
    }

    void Loss()
    {
        constexpr uint32_t kPackets = 10;
        Fabric fabric = MakeFabric();
        LinkEventApplier applier = Applier();
        uint32_t deliveredAtA = 0, deliveredAtB = 0, droppedAtA = 0, droppedAtB = 0;
        auto count = [](uint32_t* counter) {
            return Callback<void, Ptr<const Packet>, uint32_t>(
                [counter](Ptr<const Packet>, uint32_t) { ++*counter; });
        };
        fabric.atA->m_traceDataPlaneDeliver.ConnectWithoutContext(count(&deliveredAtA));
        fabric.atB->m_traceDataPlaneDeliver.ConnectWithoutContext(count(&deliveredAtB));
        fabric.atA->m_traceLinkErrorDrop.ConnectWithoutContext(count(&droppedAtA));
        fabric.atB->m_traceLinkErrorDrop.ConnectWithoutContext(count(&droppedAtB));

        LinkEvent loss = Event(LinkEventKind::Loss, LinkDirection::Forward);
        loss.errorRate = 1.0;
        Simulator::Schedule(NanoSeconds(0), [&]() {
            applier.Apply(loss, fabric.ends);
            Send(fabric, true, kPackets);
            Send(fabric, false, kPackets);
        });
        Simulator::Schedule(MicroSeconds(10), [&]() {
            NS_TEST_EXPECT_MSG_EQ(ErrorModelOf(fabric.atA), nullptr, "a's end has no model");
            NS_TEST_EXPECT_MSG_EQ(droppedAtB, kPackets, "b drops all a sends it");
            NS_TEST_EXPECT_MSG_EQ(deliveredAtB, 0, "and delivers none");
            NS_TEST_EXPECT_MSG_EQ(droppedAtA, 0, "a drops nothing b sends it");
            NS_TEST_EXPECT_MSG_EQ(deliveredAtA, kPackets, "and delivers all of it");
            applier.Apply(Event(LinkEventKind::Restore, LinkDirection::Both), fabric.ends);
            NS_TEST_EXPECT_MSG_EQ(ErrorModelOf(fabric.atB), nullptr, "the restore removes b's model");
            Send(fabric, true, kPackets);
        });
        Simulator::Run();
        NS_TEST_EXPECT_MSG_EQ(droppedAtB, kPackets, "after the restore b drops nothing more");
        NS_TEST_EXPECT_MSG_EQ(deliveredAtB, kPackets, "and delivers what a sends");
        Simulator::Destroy();
    }

    // A black hole at each end of the link, each restored by a restore naming
    // its nodes in its order.
    void Blackholes()
    {
        Fabric fabric = MakeFabric();
        LinkEventApplier applier = Applier();
        LinkEnds reversedEnds{fabric.atB, fabric.atA, fabric.a};
        LinkEvent toB = Event(LinkEventKind::Blackhole, LinkDirection::Both);
        LinkEvent toA = toB;
        std::swap(toA.a, toA.b);
        LinkEvent restoreToB = Event(LinkEventKind::Restore, LinkDirection::Both);
        LinkEvent restoreToA = restoreToB;
        std::swap(restoreToA.a, restoreToA.b);
        const uint32_t portB = fabric.atB->GetIfIndex();
        const uint32_t portA = fabric.atA->GetIfIndex();

        applier.Apply(toB, fabric.ends);
        applier.Apply(toA, reversedEnds);
        NS_TEST_EXPECT_MSG_EQ(fabric.b->IsBlackholed(portB), true, "b drops what a sends");
        NS_TEST_EXPECT_MSG_EQ(fabric.a->IsBlackholed(portA), true, "a drops what b sends");
        applier.Apply(restoreToB, fabric.ends);
        NS_TEST_EXPECT_MSG_EQ(fabric.b->IsBlackholed(portB), false, "the restore of a b clears b");
        NS_TEST_EXPECT_MSG_EQ(fabric.a->IsBlackholed(portA), true, "and leaves a");
        applier.Apply(restoreToA, reversedEnds);
        NS_TEST_EXPECT_MSG_EQ(fabric.a->IsBlackholed(portA), false, "the restore of b a clears a");
        NS_TEST_EXPECT_MSG_EQ(fabric.b->IsBlackholed(portB), false, "and b stays clear");

        auto count = [](uint32_t* counter) {
            return Callback<void, Ptr<const Packet>, uint32_t>(
                [counter](Ptr<const Packet>, uint32_t) { ++*counter; });
        };
        uint32_t forwardedToHostA = 0, forwardedToHostB = 0;
        fabric.hostPortA->m_traceEnqueue.ConnectWithoutContext(count(&forwardedToHostA));
        fabric.hostPortB->m_traceEnqueue.ConnectWithoutContext(count(&forwardedToHostB));
        Simulator::Schedule(NanoSeconds(0), [&]() {
            Send(fabric, true, 3);
            Send(fabric, false, 3);
        });
        Simulator::Run();
        NS_TEST_EXPECT_MSG_EQ(forwardedToHostB, 3, "the link carries a's packets to b's host");
        NS_TEST_EXPECT_MSG_EQ(forwardedToHostA, 3, "and b's to a's host");
        Simulator::Destroy();
    }

    void Check()
    {
        LinkEvent toB = Event(LinkEventKind::Blackhole, LinkDirection::Both);
        LinkEvent toA = toB;
        std::swap(toA.a, toA.b);
        LinkEvent restoreToB = Event(LinkEventKind::Restore, LinkDirection::Both);
        LinkEvent restoreToA = restoreToB;
        std::swap(restoreToA.a, restoreToA.b);
        NS_TEST_EXPECT_MSG_EQ(CheckLinkEvents({toB, toA, restoreToB, restoreToA}),
                              "",
                              "each direction's failure has its restore");
        NS_TEST_EXPECT_MSG_EQ(CheckLinkEvents({toB, restoreToA}),
                              "",
                              "a restore names the link in either order");
        NS_TEST_EXPECT_MSG_NE(CheckLinkEvents({toB, toA, restoreToB, restoreToA, restoreToA}),
                              "",
                              "a third restore has no failure left to put back");
    }

    void Rate()
    {
        Fabric fabric = MakeFabric();
        LinkEventApplier applier = Applier();
        std::vector<int64_t> arrivalsAtA, arrivalsAtB;
        uint32_t bytesFromA = 0, bytesFromB = 0;
        auto arrival = [](std::vector<int64_t>* arrivals) {
            return Callback<void, Ptr<const Packet>, uint32_t>(
                [arrivals](Ptr<const Packet>, uint32_t) {
                    arrivals->push_back(Simulator::Now().GetNanoSeconds());
                });
        };
        auto size = [](uint32_t* bytes) {
            return Callback<void, Ptr<const Packet>, uint32_t>(
                [bytes](Ptr<const Packet> packet, uint32_t) { *bytes = packet->GetSize(); });
        };
        fabric.atA->m_traceDataPlaneDeliver.ConnectWithoutContext(arrival(&arrivalsAtA));
        fabric.atB->m_traceDataPlaneDeliver.ConnectWithoutContext(arrival(&arrivalsAtB));
        fabric.atA->m_traceDequeue.ConnectWithoutContext(size(&bytesFromA));
        fabric.atB->m_traceDequeue.ConnectWithoutContext(size(&bytesFromB));

        LinkEvent rate = Event(LinkEventKind::Rate, LinkDirection::Forward);
        rate.rate = DataRate("100Gbps");
        applier.Apply(rate, fabric.ends);
        NS_TEST_EXPECT_MSG_EQ(fabric.atA->GetDataRate(), DataRate("100Gbps"), "a sends slower");
        NS_TEST_EXPECT_MSG_EQ(fabric.atB->GetDataRate(), DataRate("400Gbps"), "b does not");
        Simulator::Schedule(NanoSeconds(0), [&]() {
            Send(fabric, true, 1);
            Send(fabric, false, 1);
        });
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(arrivalsAtB.size(), 1, "a's packet arrives at b");
        NS_TEST_ASSERT_MSG_EQ(arrivalsAtA.size(), 1, "b's packet arrives at a");
        NS_TEST_EXPECT_MSG_EQ(arrivalsAtB[0],
                              DataRate("100Gbps").CalculateBytesTxTime(bytesFromA).GetNanoSeconds(),
                              "from a to b at the new rate");
        NS_TEST_EXPECT_MSG_EQ(arrivalsAtA[0],
                              DataRate("400Gbps").CalculateBytesTxTime(bytesFromB).GetNanoSeconds(),
                              "from b to a at the former rate");

        fabric.atB->SetDataRate(DataRate("200Gbps"));
        applier.Apply(Event(LinkEventKind::Restore, LinkDirection::Both), fabric.ends);
        NS_TEST_EXPECT_MSG_EQ(fabric.atA->GetDataRate(), DataRate("400Gbps"), "a's rate back");
        NS_TEST_EXPECT_MSG_EQ(fabric.atB->GetDataRate(),
                              DataRate("200Gbps"),
                              "the restore leaves b's end, which the failure did not set");
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
                    emittedSizes.push_back(packet->GetSize());
                }));
        RdmaInterfaceMgr nic;
        nic.dev = device;
        hw->m_nic.push_back(nic);
        hw->m_rtTable[peer.Get()].push_back(0);
    }

    // A sender queue pair as AddQueuePair registers it, without the NIC's
    // scheduler, which no test here drives.
    Ptr<RdmaQueuePair> AddSender(Ipv4Address self,
                                 Ipv4Address peer,
                                 uint64_t size,
                                 uint16_t port = kSenderPort)
    {
        Ptr<RdmaQueuePair> qp =
            CreateObject<RdmaQueuePair>(kPriorityGroup, self, peer, port, kReceiverPort);
        qp->m_size = size;
        if (hw->IsPathPerPacket())
        {
            hw->StartPathSelection(qp, 0, 0);
        }
        hw->m_qpMap[RdmaHw::GetQpKey(peer.Get(), port, kPriorityGroup)] = qp;
        return qp;
    }

    // A data packet from the sender, parsed as the receiving NIC parses it,
    // handed to this host's transport.
    void ReceiveData(Ipv4Address from,
                     Ipv4Address to,
                     uint32_t seq,
                     uint16_t identification,
                     bool marked = false,
                     uint16_t senderPort = kSenderPort,
                     uint64_t delayNs = 0)
    {
        Ptr<Packet> packet = Create<Packet>(kMtu);
        SeqTsHeader seqTs;
        seqTs.SetSeq(seq);
        seqTs.SetPG(kPriorityGroup);
        // Sent delayNs ago, where the header carries the send time.
        if (IntHeader::mode == IntHeader::TS)
        {
            seqTs.ih.ts = (Simulator::Now() - NanoSeconds(delayNs)).GetTimeStep();
        }
        packet->AddHeader(seqTs);
        UdpHeader udp;
        udp.SetSourcePort(senderPort);
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
    std::vector<uint32_t> emittedSizes;
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
        const SupervisedEwma::Parameters parameters{1.0 / 16, 0.125, 0.5, 0};
        SupervisedEwma average;
        NS_TEST_EXPECT_MSG_EQ(average.Value(), 0, "an average starts at zero");
        // Samples within the slack of the average move it by the gain alone.
        for (uint32_t i = 0; i < 100; ++i)
        {
            average.Add(0.1, 100, parameters);
        }
        NS_TEST_EXPECT_MSG_EQ_TOL(average.Value(),
                                  0.1 * (1 - std::pow(15.0 / 16, 100)),
                                  1e-12,
                                  "a sample inside the slack moves the average by the gain");
        // A step of 0.5: the excess net of the slack is 0.275 per sample, so
        // the second sample passes the threshold and restarts the average.
        const double before = average.Value();
        average.Add(0.6, 100, parameters);
        NS_TEST_EXPECT_MSG_EQ_TOL(average.Value(),
                                  before + (0.6 - before) / 16,
                                  1e-12,
                                  "one sample of a step moves the average by the gain");
        average.Add(0.6, 100, parameters);
        NS_TEST_EXPECT_MSG_EQ(average.Value(), 0.6, "the second sample of a step restarts it");
        // And back down.
        average.Add(0.0, 100, parameters);
        average.Add(0.0, 100, parameters);
        NS_TEST_EXPECT_MSG_EQ(average.Value(), 0.0, "a fall restarts it the same way");

        // With interval samples, a sample of that many packets replaces the
        // average and one of fewer moves it by the gain.
        const SupervisedEwma::Parameters sampled{1.0 / 16, 0.125, 0.5, 32};
        SupervisedEwma replaced;
        replaced.Add(0.2, 31, sampled);
        NS_TEST_EXPECT_MSG_EQ_TOL(replaced.Value(),
                                  0.2 / 16,
                                  1e-12,
                                  "a sample of fewer packets moves the average by the gain");
        replaced.Add(0.2, 32, sampled);
        NS_TEST_EXPECT_MSG_EQ(replaced.Value(), 0.2, "one of that many replaces it");
        replaced.Add(0.3, 31, sampled);
        NS_TEST_EXPECT_MSG_EQ_TOL(replaced.Value(),
                                  0.2 + 0.1 / 16,
                                  1e-12,
                                  "and a later sample of fewer moves it by the gain again");
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
        parameters.fractions = {hw->m_sprayEstimatorGain,
                                hw->m_sprayFractionCusumSlack,
                                hw->m_sprayFractionCusumThreshold,
                                hw->m_sprayEstimatorIntervalSamples};
        parameters.congestionThresholds[0] = hw->m_sprayCongestionThreshold1;
        parameters.congestionThresholds[1] = hw->m_sprayCongestionThreshold2;
        parameters.congestionThresholds[2] = hw->m_sprayCongestionThreshold3;
        parameters.oneWayDelay = oneWayDelay;
        // A base RTT of 10 us.
        parameters.delayNs = {hw->m_sprayEstimatorGain,
                              hw->m_sprayDelayCusumSlackBaseRtts * 10000,
                              hw->m_sprayDelayCusumThresholdBaseRtts * 10000,
                              hw->m_sprayEstimatorIntervalSamples};
        parameters.delayThresholdsNs[0] = hw->m_sprayDelayThreshold1BaseRtts * 10000;
        parameters.delayThresholdsNs[1] = hw->m_sprayDelayThreshold2BaseRtts * 10000;
        parameters.delayThresholdsNs[2] = hw->m_sprayDelayThreshold3BaseRtts * 10000;
        parameters.reference = static_cast<SpineGrader::GradeReference>(hw->m_sprayGradeReference);
        parameters.absenceFractionOfMedian = hw->m_sprayAbsenceFractionOfMedian;
        parameters.absenceMinimumMedian = hw->m_sprayAbsenceMinimumMedian;
        parameters.releaseFractionOfFloor = hw->m_sprayReleaseFractionOfFloor;
        parameters.floorShareOfMedian = hw->m_sprayEpsilon;
        return parameters;
    }

    // The defaults with each spine graded against no reference.
    static SpineGrader::Parameters Absolute(bool oneWayDelay = false)
    {
        SpineGrader::Parameters parameters = Defaults(oneWayDelay);
        parameters.reference = SpineGrader::GradeReference::Absolute;
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
        // Advance outside the check, which NS_ASSERT drops without asserts.
        const bool reported = grader.Advance((interval + 1) * kInterval);
        NS_ABORT_MSG_UNLESS(reported, "the next interval's first event issues the report");
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
        Attribute(SpineGrader::GradeReference::Absolute, 0);
        Attribute(SpineGrader::GradeReference::Median, 32);
        Moved();
        Simulator::Destroy();
    }

  private:
    // Every spine marks two packets in a hundred at random; from interval 20
    // spine 3 marks sixty. With the supervised average alone the CUSUM moves
    // spine 3's average within two intervals; with intervals of 100 packets
    // setting the average, within one.
    void Attribute(SpineGrader::GradeReference reference, uint32_t intervalSamples)
    {
        constexpr uint32_t kDegraded = 3;
        constexpr uint32_t kIntervals = 40;
        SpineGrader::Parameters parameters = GraderFixture::Defaults();
        parameters.reference = reference;
        parameters.fractions.intervalSamples = intervalSamples;
        const bool sampled = intervalSamples > 0;
        SpineGrader grader(parameters);
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
                const bool degraded = spine == kDegraded && interval >= (sampled ? 20 : 21);
                if (degraded)
                {
                    NS_TEST_EXPECT_MSG_GT(closed[spine].markFraction,
                                          sampled ? 0.4 : 0.5,
                                          "the degraded spine's average rises within the "
                                          "intervals its estimator takes");
                    NS_TEST_EXPECT_MSG_LT(uint32_t(grader.Report().Grade(spine)),
                                          3u,
                                          "and its grade falls");
                    if (!sampled)
                    {
                        NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(spine)),
                                              1u,
                                              "to the band of its average");
                    }
                }
                else if (spine != kDegraded || interval < 20)
                {
                    NS_TEST_EXPECT_MSG_LT(closed[spine].markFraction,
                                          sampled ? 0.1 : 0.06,
                                          "a healthy spine's average stays at its level");
                    NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(spine)),
                                          3u,
                                          "and its grade at the top");
                }
            }
            NS_TEST_EXPECT_MSG_EQ(grader.Report().edgeCongested,
                                  false,
                                  "one degraded spine is not the receiver's downlink");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().sequence),
                                  interval + 1,
                                  "each interval issues one report");
        }
    }

    // Marks on packets one spine carried for another are the carrier's.
    void Moved()
    {
        SpineGrader moved(GraderFixture::Defaults());
        for (uint32_t i = 0; i < 100; ++i)
        {
            moved.OnArrival(4, 5, true, 0);
        }
        moved.Advance(GraderFixture::kInterval);
        NS_TEST_EXPECT_MSG_EQ(moved.LastInterval()[5].marked, 100, "the carrier counts the marks");
        NS_TEST_EXPECT_MSG_EQ(moved.LastInterval()[4].arrivals, 0, "the requested spine none");
        NS_TEST_EXPECT_MSG_EQ(moved.LastInterval()[4].moved, 100, "but counts the moves");
    }
};

class SpineGradeTest : public TestCase
{
  public:
    SpineGradeTest()
        : TestCase("A spine's grade follows absolute thresholds, marks before delay, a move "
                   "holds it until its arrivals return, and the edge bit needs every spine")
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
        SpineGrader grader(GraderFixture::Absolute());
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
        SpineGrader grader(GraderFixture::Absolute(true));
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

    // A moved packet holds its requested spine until arrivals on that spine
    // release it; a trim grades its spine for its interval and holds nothing.
    void HoldDown()
    {
        SpineGrader grader(GraderFixture::Defaults());
        const std::vector<double> clean(GraderFixture::kSpines, 0.0);
        GraderFixture::Interval(grader, 0, clean);
        // Interval 1: a trim on spine 4 before the last hop, one at the last
        // hop on spine 6, and a packet requested on spine 5 carried by 7.
        grader.Advance(GraderFixture::kInterval);
        grader.OnTrim(4, false);
        grader.OnTrim(6, true);
        grader.OnArrival(5, 7, false, 0);
        grader.Advance(2 * GraderFixture::kInterval);
        for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(spine)),
                                  (spine == 4 || spine == 5) ? 0u : 3u,
                                  "a move holds its spine and a trim grades its own");
            NS_TEST_EXPECT_MSG_EQ(grader.LastInterval()[spine].held,
                                  (spine == 5),
                                  "the interval records the hold");
        }
        NS_TEST_EXPECT_MSG_EQ(grader.Report().edgeCongested,
                              true,
                              "a last-hop trim sets the edge bit for its interval");
        // A move in an interval the spine also had arrivals in holds it all
        // the same: arrivals before the hold do not count towards its release.
        SpineGrader busy(GraderFixture::Defaults());
        busy.Advance(0);
        for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
        {
            for (uint32_t i = 0; i < 100; ++i)
            {
                busy.OnArrival(spine, spine, false, 0);
            }
        }
        busy.OnArrival(5, 7, false, 0);
        busy.Advance(GraderFixture::kInterval);
        NS_TEST_EXPECT_MSG_EQ(busy.LastInterval()[5].held, true, "a move holds a busy spine");
        // Spine 5 silent while the others arrive stays held; its arrivals
        // returning release it.
        for (uint32_t interval = 2; interval < 12; ++interval)
        {
            grader.Advance(interval * GraderFixture::kInterval);
            for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
            {
                for (uint32_t i = 0; i < (spine == 5 && interval < 10 ? 0 : 100); ++i)
                {
                    grader.OnArrival(spine, spine, false, 0);
                }
            }
            grader.Advance((interval + 1) * GraderFixture::kInterval);
            NS_TEST_EXPECT_MSG_EQ(grader.LastInterval()[5].held,
                                  (interval < 10),
                                  "the hold lasts until the spine's arrivals return");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(4)),
                                  3u,
                                  "the trimmed spine grades 3 again without trims");
            NS_TEST_EXPECT_MSG_EQ(grader.Report().edgeCongested, false, "and the edge clears");
        }
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

class SpineTrimCostTest : public TestCase
{
  public:
    SpineTrimCostTest()
        : TestCase("Trims before the last hop grade a spine through its trimmed fraction, the "
                   "larger of it and the marked fraction, and hold nothing")
    {
    }

    void DoRun() override
    {
        // Spine 3's arrivals, marked arrivals and trims per interval among
        // spines with 100 clean arrivals each: trimming 20 percent of its
        // packets, 30 percent, marking and trimming 30 percent each, and
        // trimming 60 percent.
        struct Case
        {
            uint32_t arrivals;
            uint32_t marked;
            uint32_t trims;
            uint32_t grade;
        };

        const Case cases[] = {{100, 0, 25, 3}, {70, 0, 30, 2}, {70, 30, 30, 2}, {40, 0, 60, 1}};
        for (const Case& c : cases)
        {
            for (auto reference :
                 {SpineGrader::GradeReference::Absolute, SpineGrader::GradeReference::Median})
            {
                SpineGrader::Parameters parameters = GraderFixture::Defaults();
                parameters.reference = reference;
                SpineGrader grader(parameters);
                for (uint32_t interval = 0; interval < 4; ++interval)
                {
                    grader.Advance(interval * GraderFixture::kInterval);
                    for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
                    {
                        const bool tested = spine == 3;
                        for (uint32_t i = 0; i < (tested ? c.arrivals : 100); ++i)
                        {
                            grader.OnArrival(spine, spine, tested && i < c.marked, 0);
                        }
                        for (uint32_t i = 0; i < (tested ? c.trims : 0); ++i)
                        {
                            grader.OnTrim(spine, false);
                        }
                    }
                    grader.Advance((interval + 1) * GraderFixture::kInterval);
                    for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
                    {
                        NS_TEST_EXPECT_MSG_EQ(grader.LastInterval()[spine].held,
                                              false,
                                              "a trim holds no spine");
                    }
                }
                NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(3)),
                                      c.grade,
                                      "a spine grades by the larger of its fractions");
                NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(0)),
                                      3u,
                                      "and the others keep the top grade");
                NS_TEST_EXPECT_MSG_EQ_TOL(grader.LastInterval()[3].trimFraction,
                                          static_cast<double>(c.trims) / (c.arrivals + c.trims),
                                          1e-9,
                                          "the trimmed fraction is of arrivals and trims together");
            }
        }
    }
};

class SpineTrimThresholdsTest : public TestCase
{
  public:
    SpineTrimThresholdsTest()
        : TestCase("Trim thresholds grade the trimmed fraction alone, unset they are the "
                   "congestion thresholds, and the edge bit and absence keep theirs")
    {
    }

    void DoRun() override
    {
        Unset();
        Lower();
        EdgeAndAbsence();
    }

  private:
    // Trim thresholds below the default congestion thresholds.
    static constexpr std::array<double, 3> kLow = {0.05, 0.1, 0.15};

    static SpineGrader::SpineValues Values(double marked, double trimmed, double lost = 0)
    {
        return {marked, trimmed, 0, lost};
    }

    // Four intervals of 100 arrivals per spine, spine k marking and trimming
    // its share of the pattern; the report after each interval.
    static std::vector<SpineReport> Run(const SpineGrader::Parameters& parameters)
    {
        const uint32_t marked[GraderFixture::kSpines] = {0, 30, 0, 30, 60, 10, 80, 20};
        const uint32_t trims[GraderFixture::kSpines] = {0, 0, 30, 60, 30, 80, 10, 20};
        SpineGrader grader(parameters);
        std::vector<SpineReport> reports;
        for (uint32_t interval = 0; interval < 4; ++interval)
        {
            grader.Advance(interval * GraderFixture::kInterval);
            for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
            {
                for (uint32_t i = 0; i < 100; ++i)
                {
                    grader.OnArrival(spine, spine, i < marked[spine], 0);
                }
                for (uint32_t i = 0; i < trims[spine]; ++i)
                {
                    grader.OnTrim(spine, false);
                }
            }
            grader.Advance((interval + 1) * GraderFixture::kInterval);
            reports.push_back(grader.Report());
        }
        return reports;
    }

    // Unset, a spine grades by the larger of its marked and trimmed
    // fractions through the congestion thresholds, as with trim thresholds
    // equal to them.
    void Unset()
    {
        SpineGrader grader(GraderFixture::Absolute());
        const std::vector<SpineGrader::SpineValues> values = {Values(0, 0),
                                                              Values(0.3, 0),
                                                              Values(0, 0.3),
                                                              Values(0.3, 0.6),
                                                              Values(0.6, 0.3),
                                                              Values(0.1, 0.8),
                                                              Values(0.8, 0.1),
                                                              Values(0.2, 0.2)};
        const uint32_t expected[GraderFixture::kSpines] = {3, 2, 2, 1, 1, 0, 0, 3};
        const std::vector<bool> held(GraderFixture::kSpines, false);
        const SpineReport report = grader.Graded(values, held, std::nullopt);
        for (uint32_t k = 0; k < GraderFixture::kSpines; ++k)
        {
            NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(k)),
                                  expected[k],
                                  "unset, the larger fraction grades through the congestion "
                                  "thresholds");
        }
        for (auto reference :
             {SpineGrader::GradeReference::Absolute, SpineGrader::GradeReference::Median})
        {
            SpineGrader::Parameters unset = GraderFixture::Defaults();
            unset.reference = reference;
            SpineGrader::Parameters same = unset;
            same.trimThresholds = std::array<double, 3>{same.congestionThresholds[0],
                                                        same.congestionThresholds[1],
                                                        same.congestionThresholds[2]};
            const std::vector<SpineReport> a = Run(unset);
            const std::vector<SpineReport> b = Run(same);
            for (uint32_t i = 0; i < a.size(); ++i)
            {
                NS_TEST_EXPECT_MSG_EQ(a[i].edgeCongested,
                                      b[i].edgeCongested,
                                      "the same edge bit");
                for (uint32_t k = 0; k < GraderFixture::kSpines; ++k)
                {
                    NS_TEST_EXPECT_MSG_EQ(uint32_t(a[i].Grade(k)),
                                          uint32_t(b[i].Grade(k)),
                                          "trim thresholds equal to the congestion thresholds "
                                          "are the unset rule");
                }
            }
        }
    }

    // Lower trim thresholds grade a small trimmed fraction down, and leave
    // the marked fraction, and a loss fraction, on the congestion thresholds.
    void Lower()
    {
        const std::vector<SpineGrader::SpineValues> values = {Values(0, 0),
                                                              Values(0, 0.08),
                                                              Values(0, 0.12),
                                                              Values(0, 0.2),
                                                              Values(0.3, 0),
                                                              Values(0.6, 0),
                                                              Values(0.3, 0.12),
                                                              Values(0, 0, 0.08)};
        const uint32_t before[GraderFixture::kSpines] = {3, 3, 3, 3, 2, 1, 2, 3};
        const uint32_t after[GraderFixture::kSpines] = {3, 2, 1, 0, 2, 1, 1, 3};
        const std::vector<bool> held(GraderFixture::kSpines, false);
        SpineGrader::Parameters parameters = GraderFixture::Absolute();
        const SpineReport unset =
            SpineGrader(parameters).Graded(values, held, std::optional(0.875));
        parameters.trimThresholds = kLow;
        const SpineReport low = SpineGrader(parameters).Graded(values, held, std::optional(0.875));
        for (uint32_t k = 0; k < GraderFixture::kSpines; ++k)
        {
            NS_TEST_EXPECT_MSG_EQ(uint32_t(unset.Grade(k)), before[k], "the congestion thresholds");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(low.Grade(k)),
                                  after[k],
                                  "the trim thresholds grade the trimmed fraction alone");
        }
    }

    // Lower trim thresholds leave the edge bit on the congestion thresholds
    // and on last-hop trims, and a silent spine held.
    void EdgeAndAbsence()
    {
        SpineGrader::Parameters parameters = GraderFixture::Absolute();
        parameters.trimThresholds = kLow;
        const std::vector<SpineGrader::SpineValues> values(GraderFixture::kSpines,
                                                           Values(0, 0.08));
        const std::vector<bool> held(GraderFixture::kSpines, false);
        const SpineReport graded = SpineGrader(parameters).Graded(values, held, std::nullopt);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(graded.Grade(0)), 2u, "every spine grades down");
        NS_TEST_EXPECT_MSG_EQ(graded.edgeCongested,
                              false,
                              "while the congestion thresholds give each the top grade");

        SpineGrader grader(parameters);
        GraderFixture::Interval(grader, 0, std::vector<double>(GraderFixture::kSpines, 0));
        NS_TEST_EXPECT_MSG_EQ(grader.Report().edgeCongested, false, "a clean interval");
        grader.OnArrival(0, 0, false, 0);
        grader.OnTrim(0, true);
        NS_TEST_ASSERT_MSG_EQ(grader.Advance(2 * GraderFixture::kInterval), true, "a report");
        NS_TEST_EXPECT_MSG_EQ(grader.Report().edgeCongested, true, "a last-hop trim");

        SpineGrader absent(parameters);
        std::mt19937 random(11);
        std::uniform_int_distribution<uint32_t> spine(0, GraderFixture::kSpines - 1);
        for (uint32_t interval = 0; interval < 20; ++interval)
        {
            absent.Advance(interval * GraderFixture::kInterval);
            for (uint32_t i = 0; i < 300; ++i)
            {
                const uint8_t k = spine(random);
                if (k != 5 || interval < 10)
                {
                    absent.OnArrival(k, k, false, 0);
                }
            }
            absent.Advance((interval + 1) * GraderFixture::kInterval);
            NS_TEST_EXPECT_MSG_EQ(uint32_t(absent.Report().Grade(5)),
                                  (interval < 10) ? 3u : 0u,
                                  "a silent spine is held");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(absent.Report().Grade(4)), 3u, "and no other");
        }
    }
};

class SpineDelayReferenceTest : public TestCase
{
  public:
    SpineDelayReferenceTest()
        : TestCase("With one-way delay, a spine's delay above its least is graded against the "
                   "median spine's, after its marks and trims")
    {
    }

    void DoRun() override
    {
        // Base RTT 10 us: delay bands from 2.5, 5 and 7.5 us of excess. Every
        // spine queues 6 us; spine 3 queues 6 us more, spine 6 too but is
        // marked at 0.4.
        SpineGrader grader(GraderFixture::Defaults(true));
        NS_TEST_ASSERT_MSG_EQ(uint32_t(GraderFixture::Defaults(true).reference),
                              uint32_t(SpineGrader::GradeReference::Median),
                              "the default reference is the median");
        for (uint32_t interval = 0; interval < 4; ++interval)
        {
            grader.Advance(interval * GraderFixture::kInterval);
            for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
            {
                const uint64_t queued = spine == 3 || spine == 6 ? 12000 : 6000;
                // One packet at the least delay, so that the excess is what the
                // others carry.
                grader.OnArrival(spine, spine, false, 3000);
                for (uint32_t i = 0; i < 99; ++i)
                {
                    grader.OnArrival(spine, spine, spine == 6 && i < 40, 3000 + queued);
                }
            }
            grader.Advance((interval + 1) * GraderFixture::kInterval);
        }
        const uint32_t expected[GraderFixture::kSpines] = {3, 3, 3, 1, 3, 3, 2, 3};
        for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(spine)),
                                  expected[spine],
                                  "a delay alike earns the top grade, one standing out loses");
        }
    }
};

class SpineGradeReferenceTest : public TestCase
{
  public:
    SpineGradeReferenceTest()
        : TestCase("Against the median, spines marked alike all earn the top grade and one "
                   "standing out grades down")
    {
    }

    void DoRun() override
    {
        // Every spine at 0.3; one at 0.3 and the rest at 0; one at 0.6 and the
        // rest at 0.3; three at 0.6, which the median of 0 sets apart by more
        // than the mean of 0.225 would.
        const std::vector<std::vector<double>> fractions{
            std::vector<double>(GraderFixture::kSpines, 0.3),
            {0, 0, 0, 0.3, 0, 0, 0, 0},
            {0.3, 0.3, 0.3, 0.6, 0.3, 0.3, 0.3, 0.3},
            {0, 0, 0, 0.6, 0, 0.6, 0.6, 0},
        };
        // The grades of spine 3 and of spine 0, absolute then median.
        const uint32_t expected[4][2][2] = {
            {{2, 2}, {3, 3}},
            {{2, 3}, {2, 3}},
            {{1, 2}, {2, 3}},
            {{1, 3}, {1, 3}},
        };
        for (uint32_t c = 0; c < fractions.size(); ++c)
        {
            for (auto reference :
                 {SpineGrader::GradeReference::Absolute, SpineGrader::GradeReference::Median})
            {
                SpineGrader::Parameters parameters = GraderFixture::Defaults();
                parameters.reference = reference;
                SpineGrader grader(parameters);
                for (uint32_t interval = 0; interval < 4; ++interval)
                {
                    GraderFixture::Interval(grader, interval, fractions[c]);
                }
                const uint32_t r = reference == SpineGrader::GradeReference::Median;
                NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(3)),
                                      expected[c][r][0],
                                      "spine 3 earns its grade against the reference");
                NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(0)),
                                      expected[c][r][1],
                                      "and spine 0 its");
                NS_TEST_EXPECT_MSG_EQ(grader.Report().edgeCongested,
                                      (c == 0 || c == 2),
                                      "the edge bit is judged on the averages themselves");
            }
        }
    }
};

class SpineAbsenceTest : public TestCase
{
  public:
    SpineAbsenceTest()
        : TestCase("A spine whose arrivals stop while the others' continue is held until its "
                   "floor's arrivals return, judged over enough arrivals, and binomial "
                   "variation never holds one")
    {
    }

    void DoRun() override
    {
        Dead();
        Returns();
        Healthy();
        Sparse();
        GradedDown();
        EvenMedian();
    }

  private:
    // perInterval arrivals per interval, each over a spine drawn uniformly;
    // spine 5 gets none in [from, to).
    static void Interval(SpineGrader& grader,
                         std::mt19937& random,
                         uint32_t interval,
                         uint32_t perInterval,
                         uint32_t from = UINT32_MAX,
                         uint32_t to = UINT32_MAX)
    {
        std::uniform_int_distribution<uint32_t> spine(0, GraderFixture::kSpines - 1);
        grader.Advance(interval * GraderFixture::kInterval);
        for (uint32_t i = 0; i < perInterval; ++i)
        {
            const uint8_t k = spine(random);
            if (k == 5 && interval >= from && interval < to)
            {
                continue;
            }
            grader.OnArrival(k, k, false, 0);
        }
        grader.Advance((interval + 1) * GraderFixture::kInterval);
    }

    // 300 arrivals per interval, the kill fixture's rate at a report interval
    // of two base RTTs; spine 5 silent from interval 10 for 50 intervals.
    void Dead()
    {
        SpineGrader grader(GraderFixture::Defaults());
        std::mt19937 random(11);
        uint32_t holds = 0;
        bool held = false;
        for (uint32_t interval = 0; interval < 60; ++interval)
        {
            Interval(grader, random, interval, 300, 10, 60);
            const bool now = grader.LastInterval()[5].held;
            holds += now && !held;
            held = now;
            NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(5)),
                                  (interval < 10) ? 3u : 0u,
                                  "a silent spine is held from its first silent interval on");
            for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
            {
                if (spine != 5)
                {
                    NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(spine)),
                                          3u,
                                          "only the silent spine is held");
                }
            }
        }
        NS_TEST_EXPECT_MSG_EQ(holds, 1, "and held once");
    }

    // Arrivals that return release the spine within two intervals, whether at
    // a top-graded spine's rate or at about the exploration floor's, one per
    // interval against a median of 43.
    void Returns()
    {
        for (const uint32_t returned : {300u, 6u})
        {
            SpineGrader grader(GraderFixture::Defaults());
            std::mt19937 random(14);
            std::uniform_int_distribution<uint32_t> spine(0, GraderFixture::kSpines - 2);
            uint32_t released = UINT32_MAX;
            for (uint32_t interval = 0; interval < 40; ++interval)
            {
                grader.Advance(interval * GraderFixture::kInterval);
                // 300 arrivals over the other seven spines, and spine 7's share
                // of returned arrivals on spine 7 from interval 20.
                for (uint32_t i = 0; i < 300; ++i)
                {
                    const uint8_t k = spine(random);
                    grader.OnArrival(k, k, false, 0);
                }
                if (interval >= 20)
                {
                    for (uint32_t i = 0; i < returned / 8 + (returned < 8 ? 1 : 0); ++i)
                    {
                        grader.OnArrival(7, 7, false, 0);
                    }
                }
                grader.Advance((interval + 1) * GraderFixture::kInterval);
                if (interval >= 20 && released == UINT32_MAX && !grader.LastInterval()[7].held)
                {
                    released = interval;
                }
                if (interval > 0 && interval < 20)
                {
                    NS_TEST_EXPECT_MSG_EQ(grader.LastInterval()[7].held,
                                          true,
                                          "a silent spine stays held");
                }
            }
            NS_TEST_EXPECT_MSG_LT(released,
                                  22,
                                  "returned arrivals release it within two intervals");
            // At the floor's rate it is judged absent again, as a sender that
            // never raised its share would leave it.
            if (returned == 300)
            {
                NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(7)),
                                      3u,
                                      "and at a top-graded spine's rate it stays released");
            }
        }
    }

    // 10000 intervals of 300 uniform arrivals hold no spine: a spine's count is
    // binomial with mean 37.5, and falling below an eighth of the median, 4 or
    // fewer with the median at its mean, has probability 6.1e-13 per spine and
    // interval.
    void Healthy()
    {
        SpineGrader grader(GraderFixture::Defaults());
        std::mt19937 random(12);
        uint32_t held = 0;
        for (uint32_t interval = 0; interval < 10000; ++interval)
        {
            Interval(grader, random, interval, 300);
            for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
            {
                held += grader.LastInterval()[spine].held;
            }
        }
        NS_TEST_EXPECT_MSG_EQ(held, 0, "binomial variation never holds a spine");
    }

    // At a median of 5 arrivals per interval the counts accumulate for four
    // intervals before the median of 20 clears the minimum of 16 and a silent
    // spine is judged; a sparse healthy stream holds nothing.
    void Sparse()
    {
        SpineGrader grader(GraderFixture::Defaults());
        for (uint32_t interval = 0; interval < 8; ++interval)
        {
            grader.Advance(interval * GraderFixture::kInterval);
            for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
            {
                for (uint32_t i = 0; i < (spine == 5 ? 0 : 5); ++i)
                {
                    grader.OnArrival(spine, spine, false, 0);
                }
            }
            grader.Advance((interval + 1) * GraderFixture::kInterval);
            NS_TEST_EXPECT_MSG_EQ(grader.LastInterval()[5].held,
                                  (interval >= 3),
                                  "a silent spine is judged once four intervals hold enough");
        }
        SpineGrader healthy(GraderFixture::Defaults());
        std::mt19937 random(13);
        uint32_t held = 0;
        for (uint32_t interval = 0; interval < 10000; ++interval)
        {
            Interval(healthy, random, interval, 40);
            for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
            {
                held += healthy.LastInterval()[spine].held;
            }
        }
        NS_TEST_EXPECT_MSG_EQ(held, 0, "a sparse healthy stream holds nothing");
    }

    // With eight spines the median is the mean of the two middle counts: of
    // 40, 40, 40, 40, 8, 8, 8 and spine 7's, it is 24, so 2 arrivals are
    // absent and 4 are not; the lower middle, 8, would hold neither and the
    // upper, 40, both.
    void EvenMedian()
    {
        for (const uint32_t tested : {2u, 4u})
        {
            SpineGrader grader(GraderFixture::Defaults());
            grader.Advance(0);
            const uint32_t counts[GraderFixture::kSpines] = {40, 40, 40, 40, 8, 8, 8, tested};
            for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
            {
                for (uint32_t i = 0; i < counts[spine]; ++i)
                {
                    grader.OnArrival(spine, spine, false, 0);
                }
            }
            grader.Advance(GraderFixture::kInterval);
            NS_TEST_EXPECT_MSG_EQ(grader.LastInterval()[7].held,
                                  (tested == 2),
                                  "absence is judged against the mean of the middle counts");
        }
    }

    // A spine its marks grade 0 is sent next to nothing and may be held for
    // it, but the hold lets go once its arrivals return, and the spine earns
    // the top grade back when its marks stop.
    void GradedDown()
    {
        SpineGrader grader(GraderFixture::Defaults());
        for (uint32_t interval = 0; interval < 30; ++interval)
        {
            grader.Advance(interval * GraderFixture::kInterval);
            for (uint32_t spine = 0; spine < GraderFixture::kSpines; ++spine)
            {
                const uint32_t arrivals = spine == 2 && interval >= 4 && interval < 20 ? 1 : 40;
                for (uint32_t i = 0; i < arrivals; ++i)
                {
                    grader.OnArrival(spine, spine, spine == 2 && interval < 20, 0);
                }
            }
            grader.Advance((interval + 1) * GraderFixture::kInterval);
        }
        NS_TEST_EXPECT_MSG_EQ(grader.LastInterval()[2].held, false, "the hold lets go");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(2)), 3u, "and the spine earns 3 back");
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

class SpineCongestionGradeFloorTest : public TestCase
{
  public:
    SpineCongestionGradeFloorTest()
        : TestCase("A congestion grade floor raises the grades the marked and trimmed fractions "
                   "and the delay earn, and leaves the loss grade and a held spine at 0")
    {
    }

    void DoRun() override
    {
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        UintegerValue attribute;
        hw->GetAttribute("SprayCongestionGradeFloor", attribute);
        NS_TEST_EXPECT_MSG_EQ(attribute.Get(), 0u, "SprayCongestionGradeFloor is 0 by default");
        hw->SetNode(CreateObject<Node>());
        hw->m_sprayBaseRttNs = 10000;
        hw->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayPolicy);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(hw->GraderParameters().congestionGradeFloor),
                              0u,
                              "the grader takes the default");
        hw->SetAttribute("SprayCongestionGradeFloor", UintegerValue(1));
        NS_TEST_EXPECT_MSG_EQ(uint32_t(hw->GraderParameters().congestionGradeFloor),
                              1u,
                              "and the floor set");

        Arrivals();
        PathValues();
        SenderLoss();
    }

  private:
    static constexpr uint32_t kSpines = GraderFixture::kSpines;

    // Spine 2 marks nine in ten of its arrivals, above the top threshold.
    void Arrivals()
    {
        std::vector<double> marks(kSpines, 0);
        marks[2] = 0.9;
        for (const uint8_t floor : {0, 1, 2})
        {
            SpineGrader::Parameters parameters = GraderFixture::Absolute();
            parameters.congestionGradeFloor = floor;
            SpineGrader grader(parameters);
            GraderFixture::Interval(grader, 0, marks);
            NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(2)),
                                  uint32_t(floor),
                                  "the marked spine grades the floor");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Report().Grade(0)),
                                  3u,
                                  "an unmarked spine keeps the top grade");
        }
    }

    // The switch_queue path's values: the marked fraction and the delay are
    // floored, the loss fraction's grade and the loss hold are not.
    void PathValues()
    {
        std::vector<SpineGrader::SpineValues> values(kSpines);
        values[1].markFraction = 0.9;
        values[2].lossFraction = 0.8;
        values[3].lossFraction = 0.9;
        values[4].delayNs = 9000;
        std::vector<bool> held(kSpines, false);
        held[5] = true;
        for (const uint8_t floor : {0, 1})
        {
            SpineGrader::Parameters parameters = GraderFixture::Absolute(true);
            parameters.congestionGradeFloor = floor;
            const SpineReport report = SpineGrader(parameters).Graded(values, held, 0.875);
            NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(1)),
                                  uint32_t(floor),
                                  "the marked fraction's grade is floored");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(2)),
                                  0u,
                                  "the loss fraction's grade is not");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(3)), 0u, "nor the loss hold");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(4)),
                                  uint32_t(floor),
                                  "the delay's grade is floored");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(5)), 0u, "a held spine grades 0");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(0)), 3u, "a clean spine grades 3");
        }
    }

    // A report floored at 1 meets the sender's loss grade of 0, and the
    // lower stands.
    void SenderLoss()
    {
        SpineScores::Parameters parameters = ScoresFixture::Defaults();
        parameters.lossThresholds = std::array<double, 3>{0.1, 0.25, 0.5};
        SpineScores scores(kSpines, parameters);
        for (uint32_t i = 0; i < 10; ++i)
        {
            scores.OnSend(3, 1000);
            scores.OnSend(4, 1000);
            if (i < 6)
            {
                scores.OnLoss(3, 1000);
            }
        }
        std::vector<uint8_t> grades(kSpines, 3);
        grades[3] = 1;
        grades[4] = 1;
        scores.OnReport(ScoresFixture::Report(1, grades), ScoresFixture::kInterval);
        NS_TEST_EXPECT_MSG_EQ(scores.Held(3), true, "the loss grade of 0 holds the spine");
        NS_TEST_EXPECT_MSG_EQ(scores.Held(4), false, "the floored grade alone does not");
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

class SpineScoresEdgeFreezeTest : public TestCase
{
  public:
    SpineScoresEdgeFreezeTest()
        : TestCase("Under edge freeze a report with the edge bit moves no score; without it "
                   "the report moves the scores as one without the bit")
    {
    }

    void DoRun() override
    {
        NS_TEST_EXPECT_MSG_EQ(ScoresFixture::Defaults().edgeFreeze,
                              true,
                              "the scores freeze under the edge bit by default");
        BooleanValue attribute;
        CreateObject<RdmaHw>()->GetAttribute("SprayEdgeFreeze", attribute);
        NS_TEST_EXPECT_MSG_EQ(attribute.Get(), true, "SprayEdgeFreeze is true by default");

        std::vector<uint8_t> grades(ScoresFixture::kSpines, 3);
        grades[2] = 0;
        grades[5] = 1;
        SpineReport edge = ScoresFixture::Report(1, grades);
        edge.edgeCongested = true;
        const SpineReport plain = ScoresFixture::Report(1, grades);

        SpineScores::Parameters frozen = ScoresFixture::Defaults();
        frozen.edgeFreeze = true;
        SpineScores::Parameters moving = ScoresFixture::Defaults();
        moving.edgeFreeze = false;
        SpineScores held(ScoresFixture::kSpines, frozen);
        SpineScores taken(ScoresFixture::kSpines, moving);
        SpineScores reference(ScoresFixture::kSpines, moving);
        const uint32_t start = held.Score(2);
        held.OnReport(edge, 10000);
        taken.OnReport(edge, 10000);
        reference.OnReport(plain, 10000);
        for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(held.Score(spine), start, "edge freeze holds every score");
            NS_TEST_EXPECT_MSG_EQ(taken.Score(spine),
                                  reference.Score(spine),
                                  "without edge freeze the edge bit changes nothing");
        }
        NS_TEST_EXPECT_MSG_LT(taken.Score(2), start, "a spine graded 0 loses score");
        NS_TEST_EXPECT_MSG_LT(taken.Share(2), held.Share(2), "and share");
        // The bit still dates the report: the same sequence is not taken again.
        const uint32_t once = taken.Score(2);
        taken.OnReport(edge, 10100);
        NS_TEST_EXPECT_MSG_EQ(taken.Score(2), once, "a report is taken once");
    }
};

class SpineScoresPoolTest : public TestCase
{
  public:
    SpineScoresPoolTest()
        : TestCase("Pooled scores: at weight 0 the draws and shares are unchanged, at 1 every "
                   "destination follows the pool, at 0.5 their arithmetic mixture; with a pool "
                   "decay interval the pool decays once per interval by the mean grade")
    {
    }

    void DoRun() override
    {
        DoubleValue attribute;
        CreateObject<RdmaHw>()->GetAttribute("SpraySenderPoolWeight", attribute);
        NS_TEST_EXPECT_MSG_EQ(attribute.Get(), 0.0, "SpraySenderPoolWeight is 0 by default");
        UintegerValue interval;
        CreateObject<RdmaHw>()->GetAttribute("SprayPoolDecayIntervalNs", interval);
        NS_TEST_EXPECT_MSG_EQ(interval.Get(), 0u, "SprayPoolDecayIntervalNs is 0 by default");
        NS_TEST_EXPECT_MSG_EQ(ScoresFixture::Defaults().poolDecayIntervalNs,
                              0u,
                              "and so is the pool decay interval");
        WeightZero();
        WeightOne();
        WeightHalf();
        EdgeBit();
        DecayIntervalIndependentOfDestinations();
        PerReportDecayDependsOnDestinations();
        DecayIntervalMean();
        DecayIntervalEdgeBit();
        DecayIntervalIdle();
        Simulator::Destroy();
    }

  private:
    static Ptr<UniformRandomVariable> Stream()
    {
        return CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(0));
    }

    // Reports with varied grades, some with the edge bit, some out of order
    // and some after a silence; a draw after each.
    static std::vector<uint8_t> Run(SpineScores& scores, UniformRandomVariable& random)
    {
        std::mt19937 grades(7);
        std::vector<uint8_t> draws;
        uint64_t now = 0;
        for (uint32_t i = 1; i < 400; ++i)
        {
            now += i % 50 == 0 ? 7 * ScoresFixture::kInterval : ScoresFixture::kInterval;
            std::vector<uint8_t> graded(ScoresFixture::kSpines);
            for (uint8_t& grade : graded)
            {
                grade = grades() % 4;
            }
            SpineReport report = ScoresFixture::Report(i % 3 == 0 ? i - 2 : i, graded);
            report.edgeCongested = i % 11 == 0;
            scores.OnReport(report, now);
            for (uint32_t k = 0; k < 4; ++k)
            {
                draws.push_back(scores.Choose(random, now + k * 1000));
            }
        }
        return draws;
    }

    void WeightZero()
    {
        for (uint32_t candidates : {1u, 2u})
        {
            SpineScores::Parameters parameters = ScoresFixture::Defaults();
            parameters.candidates = candidates;
            SpineScores alone(ScoresFixture::kSpines, parameters);
            SpineScores pool(ScoresFixture::kSpines, parameters);
            SpineScores pooled(ScoresFixture::kSpines, parameters);
            pooled.Pool(pool, 0);
            const uint32_t start = pool.Score(0);
            Ptr<UniformRandomVariable> first = Stream();
            Ptr<UniformRandomVariable> second = Stream();
            const std::vector<uint8_t> drawn = Run(alone, *first);
            NS_TEST_EXPECT_MSG_EQ((Run(pooled, *second) == drawn),
                                  true,
                                  "at weight 0 every draw is the unpooled one");
            NS_TEST_EXPECT_MSG_EQ(first->GetValue(), second->GetValue(), "and no draw more");
            for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
            {
                NS_TEST_EXPECT_MSG_EQ(pooled.Score(spine), alone.Score(spine), "scores match");
                NS_TEST_EXPECT_MSG_EQ(pooled.Share(spine), alone.Share(spine), "shares match");
            }
            bool moved = false;
            for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
            {
                moved |= pool.Score(spine) != start;
            }
            NS_TEST_EXPECT_MSG_EQ(moved, true, "the pool takes the reports all the same");
        }
    }

    // A report towards one destination moves the other's p, which follows the
    // pool exactly: the pool's own p, and a reference that took every report.
    void WeightOne()
    {
        const SpineScores::Parameters parameters = ScoresFixture::Defaults();
        SpineScores pool(ScoresFixture::kSpines, parameters);
        SpineScores toFirst(ScoresFixture::kSpines, parameters);
        SpineScores toSecond(ScoresFixture::kSpines, parameters);
        SpineScores reference(ScoresFixture::kSpines, parameters);
        toFirst.Pool(pool, 1);
        toSecond.Pool(pool, 1);
        const double before = toSecond.Share(3);
        std::vector<uint8_t> grades(ScoresFixture::kSpines, 3);
        grades[3] = 0;
        for (uint32_t sequence = 1; sequence <= 6; ++sequence)
        {
            const SpineReport report = ScoresFixture::Report(sequence, grades);
            toFirst.OnReport(report, sequence * ScoresFixture::kInterval);
            reference.OnReport(report, sequence * ScoresFixture::kInterval);
        }
        NS_TEST_EXPECT_MSG_LT(toSecond.Share(3), before / 2, "a report to one moves the other");
        for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ_TOL(toSecond.Share(spine),
                                      reference.Share(spine),
                                      1e-12,
                                      "the other's p is the pool's");
        }
        Ptr<UniformRandomVariable> drawn = Stream();
        Ptr<UniformRandomVariable> replayed = Stream();
        for (uint32_t i = 0; i < 5000; ++i)
        {
            NS_TEST_ASSERT_MSG_EQ(uint32_t(toSecond.Choose(*drawn, 65000)),
                                  uint32_t(reference.Choose(*replayed, 65000)),
                                  "the other draws as the pool's scores do");
        }
    }

    // gamma 0.25 from 3072 per spine: a decay takes 768 from a full score and
    // 576 from 2304, and grade 3 deposits 768.
    void WeightHalf()
    {
        const SpineScores::Parameters parameters = ScoresFixture::Defaults();
        SpineScores pool(ScoresFixture::kSpines, parameters);
        SpineScores toFirst(ScoresFixture::kSpines, parameters);
        SpineScores toSecond(ScoresFixture::kSpines, parameters);
        toFirst.Pool(pool, 0.5);
        toSecond.Pool(pool, 0.5);
        NS_TEST_ASSERT_MSG_EQ(pool.Score(0), 3072u, "every score starts at 3072");
        auto expectShares = [&](const std::vector<double>& mixed, const std::string& when) {
            double sum = 0;
            for (double score : mixed)
            {
                sum += score;
            }
            for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
            {
                NS_TEST_EXPECT_MSG_EQ_TOL(toSecond.Share(spine),
                                          0.98 * mixed[spine] / sum + 0.02 / 8,
                                          1e-12,
                                          "p is that of the mixed scores " + when);
            }
        };
        std::vector<uint8_t> grades(ScoresFixture::kSpines, 3);
        grades[0] = 0;
        toFirst.OnReport(ScoresFixture::Report(1, grades), 10000);
        // The pool and the first: 2304 on spine 0, 3072 elsewhere; the second
        // untouched at 3072.
        NS_TEST_EXPECT_MSG_EQ(pool.Score(0), 2304u, "the pool takes the first's report");
        NS_TEST_EXPECT_MSG_EQ(toSecond.Score(0), 3072u, "the second's own scores stand");
        expectShares({2688, 3072, 3072, 3072, 3072, 3072, 3072, 3072}, "after one report");
        grades[0] = 3;
        grades[1] = 0;
        toSecond.OnReport(ScoresFixture::Report(1, grades), 20000);
        // The pool: 2304 - 576 + 768 = 2496 on spine 0, 2304 on spine 1;
        // the second: 2304 on spine 1.
        NS_TEST_EXPECT_MSG_EQ(pool.Score(0), 2496u, "the pool decays and deposits again");
        NS_TEST_EXPECT_MSG_EQ(pool.Score(1), 2304u, "the pool takes the second's report");
        expectShares({2784, 2304, 3072, 3072, 3072, 3072, 3072, 3072}, "after two");

        // A draw is p's inverse over the mixed scores.
        const std::vector<double> byMixed = {2784, 5088, 8160, 11232, 14304, 17376, 20448, 23520};
        Ptr<UniformRandomVariable> drawn = Stream();
        Ptr<UniformRandomVariable> replayed = Stream();
        for (uint32_t i = 0; i < 5000; ++i)
        {
            const double u = replayed->GetValue();
            const uint32_t expected =
                u < 0.02 ? uint32_t(u / 0.02 * 8)
                         : std::upper_bound(byMixed.begin(),
                                            byMixed.end(),
                                            (u - 0.02) / 0.98 * byMixed.back()) -
                               byMixed.begin();
            NS_TEST_ASSERT_MSG_EQ(uint32_t(toSecond.Choose(*drawn, 25000)),
                                  expected,
                                  "the draw maps one uniform value through the mixed p");
        }
    }

    // A report the edge bit holds the scores under holds the pool too.
    void EdgeBit()
    {
        for (bool freeze : {true, false})
        {
            SpineScores::Parameters parameters = ScoresFixture::Defaults();
            parameters.edgeFreeze = freeze;
            SpineScores pool(ScoresFixture::kSpines, parameters);
            SpineScores local(ScoresFixture::kSpines, parameters);
            local.Pool(pool, 0.5);
            std::vector<uint8_t> grades(ScoresFixture::kSpines, 3);
            grades[4] = 0;
            SpineReport edge = ScoresFixture::Report(1, grades);
            edge.edgeCongested = true;
            local.OnReport(edge, 10000);
            const uint32_t expected = freeze ? 3072 : 2304;
            NS_TEST_EXPECT_MSG_EQ(pool.Score(4),
                                  expected,
                                  "the pool is held exactly when the scores are");
            NS_TEST_EXPECT_MSG_EQ(local.Score(4), pool.Score(4), "as the scores are");
        }
    }

    static SpineScores::Parameters PoolParameters(uint64_t decayIntervalNs)
    {
        SpineScores::Parameters parameters = ScoresFixture::Defaults();
        parameters.poolDecayIntervalNs = decayIntervalNs;
        return parameters;
    }

    // Pool A takes one destination's reports and pool B 63 destinations',
    // every destination sending one report per interval, A's late in it and
    // B's spread over it, all spines graded 3 but spine 3, graded 0 from
    // interval 4. The pools' scores after each interval, A's then B's.
    static std::vector<std::pair<std::vector<uint32_t>, std::vector<uint32_t>>> RunPools(
        uint64_t decayIntervalNs)
    {
        constexpr uint32_t kDestinations = 63;
        const SpineScores::Parameters parameters = ScoresFixture::Defaults();
        SpineScores poolA(ScoresFixture::kSpines, PoolParameters(decayIntervalNs));
        SpineScores poolB(ScoresFixture::kSpines, PoolParameters(decayIntervalNs));
        SpineScores toA(ScoresFixture::kSpines, parameters);
        toA.Pool(poolA, 0.5);
        std::deque<SpineScores> toB;
        for (uint32_t d = 0; d < kDestinations; ++d)
        {
            toB.emplace_back(ScoresFixture::kSpines, parameters);
            toB.back().Pool(poolB, 0.5);
        }
        std::vector<std::pair<std::vector<uint32_t>, std::vector<uint32_t>>> scores;
        for (uint32_t n = 0; n < 10; ++n)
        {
            std::vector<uint8_t> grades(ScoresFixture::kSpines, 3);
            grades[3] = n >= 4 ? 0 : 3;
            const SpineReport report = ScoresFixture::Report(n + 1, grades);
            const uint64_t start = n * ScoresFixture::kInterval;
            for (uint32_t d = 0; d < kDestinations; ++d)
            {
                toB[d].OnReport(report, start + 100 + 140 * d);
            }
            toA.OnReport(report, start + 9000);
            std::vector<uint32_t> a;
            std::vector<uint32_t> b;
            for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
            {
                a.push_back(poolA.Score(spine));
                b.push_back(poolB.Score(spine));
            }
            scores.emplace_back(a, b);
        }
        return scores;
    }

    // With a pool decay interval, the pool's scores depend on the grades
    // within each interval and not on how many destinations report them.
    void DecayIntervalIndependentOfDestinations()
    {
        const auto scores = RunPools(ScoresFixture::kInterval);
        // Spine 3: 3072 through interval 3, then decayed once per interval.
        const std::vector<uint32_t> spine3 = {3072, 3072, 3072, 3072, 2304,
                                              1728, 1296, 972,  729,  546};
        for (uint32_t n = 0; n < scores.size(); ++n)
        {
            for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
            {
                NS_TEST_EXPECT_MSG_EQ(scores[n].first[spine],
                                      scores[n].second[spine],
                                      "1 and 63 destinations give the same pooled score");
                NS_TEST_EXPECT_MSG_EQ(scores[n].first[spine],
                                      (spine == 3 ? spine3[n] : 3072u),
                                      "the pool decays once per interval");
            }
        }
    }

    // Without one, the pool decays once per report taken, so 63 destinations
    // take spine 3 to zero within the interval in which one takes it to 2304.
    void PerReportDecayDependsOnDestinations()
    {
        const auto scores = RunPools(0);
        for (uint32_t n = 0; n < 4; ++n)
        {
            NS_TEST_EXPECT_MSG_EQ((scores[n].first == scores[n].second),
                                  true,
                                  "the top grade holds both pools at 3072");
        }
        NS_TEST_EXPECT_MSG_EQ(scores[4].first[3], 2304u, "one report decays spine 3 once");
        NS_TEST_EXPECT_MSG_EQ(scores[4].second[3], 0u, "63 reports decay it to zero");
        NS_TEST_EXPECT_MSG_EQ(scores[9].first[3], 546u, "one report per interval: 546");
        NS_TEST_EXPECT_MSG_EQ(scores[9].second[3], 0u, "63 per interval: zero");
    }

    // Two destinations grading differently in one interval: the decayed
    // base, 2304 from 3072, plus 256 times the mean of their grades.
    void DecayIntervalMean()
    {
        const SpineScores::Parameters parameters = ScoresFixture::Defaults();
        SpineScores pool(ScoresFixture::kSpines, PoolParameters(ScoresFixture::kInterval));
        SpineScores toFirst(ScoresFixture::kSpines, parameters);
        SpineScores toSecond(ScoresFixture::kSpines, parameters);
        toFirst.Pool(pool, 0.5);
        toSecond.Pool(pool, 0.5);
        std::vector<uint8_t> first(ScoresFixture::kSpines, 3);
        std::vector<uint8_t> second(ScoresFixture::kSpines, 3);
        first[0] = 0;
        second[0] = 1;
        first[1] = 0;
        second[1] = 0;
        first[2] = 2;
        second[2] = 3;
        second[4] = 0;
        toFirst.OnReport(ScoresFixture::Report(1, first), 2000);
        NS_TEST_EXPECT_MSG_EQ(pool.Score(0), 2304u, "one report: the base plus its grade");
        NS_TEST_EXPECT_MSG_EQ(toFirst.Held(0), true, "and spine 0 is held at its grade 0");
        toSecond.OnReport(ScoresFixture::Report(1, second), 6000);
        const std::vector<uint32_t> expected = {2432, 2304, 2944, 3072, 2688, 3072, 3072, 3072};
        for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(pool.Score(spine), expected[spine], "2304 plus 256 x the mean");
        }
        NS_TEST_EXPECT_MSG_EQ(toFirst.Held(0), false, "a mean of 1/2 rounds to 1 and releases");
        NS_TEST_EXPECT_MSG_EQ(toFirst.Held(1), true, "a mean of 0 holds");
        NS_TEST_EXPECT_MSG_EQ(toFirst.Held(4), false, "a mean of 3/2 does not");
        // The next interval decays the scores once: 2432 - 608 = 1824, and
        // the one report's grade 3 adds 768.
        std::vector<uint8_t> top(ScoresFixture::kSpines, 3);
        toFirst.OnReport(ScoresFixture::Report(2, top), 12000);
        NS_TEST_EXPECT_MSG_EQ(pool.Score(0), 2592u, "the next interval's base is decayed once");
        NS_TEST_EXPECT_MSG_EQ(pool.Score(1), 2496u, "1728 plus 768 on spine 1");
        NS_TEST_EXPECT_MSG_EQ(toFirst.Held(1), false, "and grade 3 releases it");
    }

    // A report the edge bit holds the scores under adds no grade to the
    // interval's mean.
    void DecayIntervalEdgeBit()
    {
        for (bool freeze : {true, false})
        {
            SpineScores::Parameters parameters = ScoresFixture::Defaults();
            parameters.edgeFreeze = freeze;
            SpineScores::Parameters poolParameters = parameters;
            poolParameters.poolDecayIntervalNs = ScoresFixture::kInterval;
            SpineScores pool(ScoresFixture::kSpines, poolParameters);
            SpineScores toFirst(ScoresFixture::kSpines, parameters);
            SpineScores toSecond(ScoresFixture::kSpines, parameters);
            toFirst.Pool(pool, 0.5);
            toSecond.Pool(pool, 0.5);
            std::vector<uint8_t> grades(ScoresFixture::kSpines, 3);
            grades[5] = 0;
            SpineReport edge = ScoresFixture::Report(1, grades);
            edge.edgeCongested = true;
            toFirst.OnReport(edge, 2000);
            NS_TEST_EXPECT_MSG_EQ(pool.Score(5),
                                  2304u,
                                  "frozen, nothing counted: the base; else grade 0");
            NS_TEST_EXPECT_MSG_EQ(pool.Score(0),
                                  (freeze ? 2304u : 3072u),
                                  "frozen, nothing counted: the base; else grade 3");
            NS_TEST_EXPECT_MSG_EQ(toFirst.Held(5), !freeze, "only a counted grade 0 holds");
            grades[5] = 3;
            toSecond.OnReport(ScoresFixture::Report(1, grades), 6000);
            NS_TEST_EXPECT_MSG_EQ(pool.Score(5),
                                  (freeze ? 3072u : 2688u),
                                  "the frozen report is not in the mean");
            NS_TEST_EXPECT_MSG_EQ(toFirst.Held(5), false, "and the mean releases");
        }
    }

    // Without reports a pool with a decay interval ages as one without.
    void DecayIntervalIdle()
    {
        const SpineScores::Parameters parameters = ScoresFixture::Defaults();
        SpineScores interval(ScoresFixture::kSpines, PoolParameters(ScoresFixture::kInterval));
        SpineScores perReport(ScoresFixture::kSpines, PoolParameters(0));
        SpineScores toInterval(ScoresFixture::kSpines, parameters);
        SpineScores toPerReport(ScoresFixture::kSpines, parameters);
        toInterval.Pool(interval, 0.5);
        toPerReport.Pool(perReport, 0.5);
        std::vector<uint8_t> grades(ScoresFixture::kSpines, 3);
        grades[2] = 1;
        grades[6] = 0;
        const SpineReport report = ScoresFixture::Report(1, grades);
        toInterval.OnReport(report, 3000);
        toPerReport.OnReport(report, 3000);
        Ptr<UniformRandomVariable> first = Stream();
        Ptr<UniformRandomVariable> second = Stream();
        for (uint64_t now = 3000; now < 30 * ScoresFixture::kInterval; now += 7000)
        {
            NS_TEST_EXPECT_MSG_EQ(uint32_t(toInterval.Choose(*first, now)),
                                  uint32_t(toPerReport.Choose(*second, now)),
                                  "the same draws");
            for (uint32_t spine = 0; spine < ScoresFixture::kSpines; ++spine)
            {
                NS_TEST_EXPECT_MSG_EQ(interval.Score(spine),
                                      perReport.Score(spine),
                                      "idle, both pools age alike");
            }
        }
        NS_TEST_EXPECT_MSG_LT(interval.Score(0), 3072u, "and they did age");
    }
};

class SpineLossThresholdsTest : public TestCase
{
  public:
    SpineLossThresholdsTest()
        : TestCase("Loss thresholds grade the lost fraction of a spine at the sender, per "
                   "destination between reports and host-wide per loss interval in the pool, "
                   "which the host traces as each interval closes; under switch_queue they "
                   "grade the loss of the paths instead; with loss hold clean sentinels a "
                   "spine they grade 0 stays at 0 until that many clean sends")
    {
    }

    void DoRun() override
    {
        Defaults();
        Unset();
        Local();
        Pool();
        PoolTrace();
        LossHoldLocal(0);
        LossHoldLocal(4);
        LossHoldPool(0);
        LossHoldPool(4);
        LossHoldUnset();
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        const uint32_t savedReportBytes = CustomHeader::ackReportBytes;
        CustomHeader::ackCarriesPacketSeq = true;
        CustomHeader::ackReportBytes = 1 + SpineReport::GradeBytes(kSpines);
        Transport(GradeCostSource::Estimate);
        Transport(GradeCostSource::SwitchQueue);
        CustomHeader::ackReportBytes = savedReportBytes;
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        Oracle();
        Validation();
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = ScoresFixture::kSpines;
    static constexpr uint64_t kInterval = ScoresFixture::kInterval;
    static constexpr std::array<double, 3> kThresholds = {0.1, 0.25, 0.5};
    static constexpr uint16_t kOtherPort = 10002;

    static SpineScores::Parameters WithThresholds()
    {
        SpineScores::Parameters parameters = ScoresFixture::Defaults();
        parameters.lossThresholds = kThresholds;
        return parameters;
    }

    static void Send(SpineScores& scores,
                     uint32_t spine,
                     uint32_t sent,
                     uint32_t lost,
                     uint64_t now)
    {
        for (uint32_t i = 0; i < sent; ++i)
        {
            scores.OnSend(spine, now);
        }
        for (uint32_t i = 0; i < lost; ++i)
        {
            scores.OnLoss(spine, now);
        }
    }

    void ExpectScores(const SpineScores& scores,
                      const SpineScores& reference,
                      const std::string& message)
    {
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(scores.Score(spine), reference.Score(spine), message);
        }
    }

    void Defaults()
    {
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        NS_TEST_EXPECT_MSG_EQ(hw->m_sprayLossThreshold1, 0.0, "the loss thresholds are unset");
        NS_TEST_EXPECT_MSG_EQ(hw->m_sprayLossIntervalNs, 0u, "the loss interval is the report's");
        NS_TEST_EXPECT_MSG_EQ(hw->SprayLossThresholds().has_value(), false, "unset");
        NS_TEST_EXPECT_MSG_EQ(hw->m_sprayLossHoldCleanSentinels, 0u, "no loss hold");
        NS_TEST_EXPECT_MSG_EQ(ScoresFixture::Defaults().lossHoldCleanSentinels,
                              0u,
                              "the scores hold nothing by default");
    }

    // One report period or loss interval of the loss hold sequence: the
    // sends and losses on spine 2, and its loss grade without and with a
    // loss hold of 4 clean sends.
    struct HoldStep
    {
        uint32_t sent, lost;
        uint8_t unheld, held;
    };

    static std::vector<HoldStep> HoldSteps()
    {
        return {
            {10, 6, 0, 0}, // 6 of 10 lost grades 0: the loss hold starts
            {0, 0, 3, 0},  // nothing sent: held at 0
            {1, 1, 0, 0},  // a lost sentinel
            {2, 0, 3, 0},  // 2 clean
            {0, 0, 3, 0},  // nothing sent: the clean count is kept
            {1, 0, 3, 0},  // 3 clean
            {1, 1, 0, 0},  // a loss restarts the count
            {3, 0, 3, 0},  // 3 clean
            {1, 0, 3, 3},  // 4 clean: the loss hold ends
            {0, 0, 3, 3},  // nothing sent and not held: the top grade
            {10, 6, 0, 0}, // the next loss holds it again
            {4, 0, 3, 3},  // and 4 clean in one period end it
        };
    }

    static SpineScores::Parameters WithHold(uint32_t clean)
    {
        SpineScores::Parameters parameters = WithThresholds();
        parameters.lossHoldCleanSentinels = clean;
        return parameters;
    }

    // Per destination: the report's grade of spine 2 is 3, except 2 at the
    // ninth report, where the loss hold ends and the lower of the two holds;
    // spine 5 loses 3 of 10 in the first period, graded 1 and not held.
    void LossHoldLocal(uint32_t clean)
    {
        SpineScores scores(kSpines, WithHold(clean));
        SpineScores reference(kSpines, ScoresFixture::Defaults());
        const std::vector<HoldStep> steps = HoldSteps();
        const std::string label = clean > 0 ? "held: " : "unheld: ";
        for (uint32_t i = 0; i < steps.size(); ++i)
        {
            const uint8_t sequence = i + 1;
            const uint64_t now = sequence * kInterval;
            Send(scores, 2, steps[i].sent, steps[i].lost, now - 100);
            if (i == 0)
            {
                Send(scores, 5, 10, 3, now - 100);
            }
            std::vector<uint8_t> grades(kSpines, 3);
            if (i == 8)
            {
                grades[2] = 2;
            }
            scores.OnReport(ScoresFixture::Report(sequence, grades), now);
            std::vector<uint8_t> expected = grades;
            const uint8_t loss = clean > 0 ? steps[i].held : steps[i].unheld;
            expected[2] = std::min(grades[2], loss);
            if (i == 0)
            {
                expected[5] = 1;
            }
            reference.OnReport(ScoresFixture::Report(sequence, expected), now);
            ExpectScores(scores, reference, label + "report " + std::to_string(sequence));
            NS_TEST_EXPECT_MSG_EQ(scores.Held(2),
                                  (expected[2] == 0),
                                  label + "held while graded 0, report " +
                                      std::to_string(sequence));
        }
    }

    // In the pool, whose loss interval is the report interval: the first
    // destination sends, and the second, which sends nothing, reports every
    // spine 3 once per interval, so the pool takes its own loss grade. The
    // destinations hold nothing themselves.
    void LossHoldPool(uint32_t clean)
    {
        SpineScores pool(kSpines, WithHold(clean));
        SpineScores toFirst(kSpines, WithThresholds());
        SpineScores toSecond(kSpines, WithThresholds());
        toFirst.Pool(pool, 0.5);
        toSecond.Pool(pool, 0.5);
        SpineScores referencePool(kSpines, ScoresFixture::Defaults());
        std::vector<Closed> closed;
        pool.SetLossIntervalCallback(Recorder(closed));
        const std::vector<uint8_t> top(kSpines, 3);
        const std::vector<HoldStep> steps = HoldSteps();
        const std::string label = clean > 0 ? "held: " : "unheld: ";
        std::vector<Closed> expectedClosed;
        for (uint32_t i = 0; i < steps.size(); ++i)
        {
            const uint8_t sequence = i + 1;
            // The sends fall in interval i, and the report early in the next.
            Send(toFirst, 2, steps[i].sent, steps[i].lost, i * kInterval + 6000);
            const uint64_t now = sequence * kInterval + 5000;
            toSecond.OnReport(ScoresFixture::Report(sequence, top), now);
            std::vector<uint8_t> expected = top;
            expected[2] = clean > 0 ? steps[i].held : steps[i].unheld;
            referencePool.OnReport(ScoresFixture::Report(sequence, expected), now);
            ExpectScores(pool, referencePool, label + "interval " + std::to_string(i));
            Closed interval;
            interval.endNs = sequence * kInterval;
            interval.sent.assign(kSpines, 0);
            interval.lost.assign(kSpines, 0);
            interval.sent[2] = steps[i].sent;
            interval.lost[2] = steps[i].lost;
            interval.grades = expected;
            expectedClosed.push_back(interval);
        }
        ExpectClosed(closed, expectedClosed);

        // Held again, then three intervals with nothing counted: the one
        // closed and the two not passed leave it at 0, where it was the top
        // grade before.
        const uint32_t next = steps.size();
        Send(toFirst, 2, 10, 6, next * kInterval + 6000);
        toSecond.OnReport(ScoresFixture::Report(next + 1, top), (next + 1) * kInterval + 5000);
        std::vector<uint8_t> expected = top;
        expected[2] = 0;
        referencePool.OnReport(ScoresFixture::Report(next + 1, expected),
                               (next + 1) * kInterval + 5000);
        toSecond.OnReport(ScoresFixture::Report(next + 2, top), (next + 4) * kInterval + 5000);
        expected[2] = clean > 0 ? 0 : 3;
        referencePool.OnReport(ScoresFixture::Report(next + 2, expected),
                               (next + 4) * kInterval + 5000);
        ExpectScores(pool, referencePool, label + "intervals with nothing counted");
        NS_TEST_EXPECT_MSG_EQ(pool.Held(2), (clean > 0), label + "the pool holds the spine");
    }

    // Without loss thresholds nothing is counted, so a loss hold set holds
    // nothing: the scores deposit the report's grades.
    void LossHoldUnset()
    {
        SpineScores::Parameters parameters = ScoresFixture::Defaults();
        parameters.lossHoldCleanSentinels = 4;
        SpineScores pool(kSpines, parameters);
        SpineScores scores(kSpines, parameters);
        scores.Pool(pool, 0.5);
        SpineScores reference(kSpines, ScoresFixture::Defaults());
        SpineScores referencePool(kSpines, ScoresFixture::Defaults());
        reference.Pool(referencePool, 0.5);
        const std::vector<uint8_t> top(kSpines, 3);
        for (uint8_t sequence = 1; sequence <= 3; ++sequence)
        {
            const uint32_t lost = sequence == 1 ? 10 : 0;
            Send(scores, 2, lost, lost, sequence * kInterval - 100);
            scores.OnReport(ScoresFixture::Report(sequence, top), sequence * kInterval);
            reference.OnReport(ScoresFixture::Report(sequence, top), sequence * kInterval);
        }
        ExpectScores(scores, reference, "without thresholds nothing is held");
        ExpectScores(pool, referencePool, "nor in the pool");
    }

    // Unset, sends and losses leave every deposit as the report has it.
    void Unset()
    {
        const SpineScores::Parameters parameters = ScoresFixture::Defaults();
        SpineScores pool(kSpines, parameters);
        SpineScores counted(kSpines, parameters);
        SpineScores reference(kSpines, parameters);
        SpineScores referencePool(kSpines, parameters);
        counted.Pool(pool, 0.5);
        reference.Pool(referencePool, 0.5);
        std::vector<uint8_t> grades(kSpines, 3);
        grades[5] = 2;
        for (uint8_t sequence = 1; sequence <= 3; ++sequence)
        {
            Send(counted, 3, 10, 7, sequence * kInterval - 1);
            counted.OnReport(ScoresFixture::Report(sequence, grades), sequence * kInterval);
            reference.OnReport(ScoresFixture::Report(sequence, grades), sequence * kInterval);
        }
        ExpectScores(counted, reference, "unset, the scores deposit the report's grades");
        ExpectScores(pool, referencePool, "and so does the pool");
    }

    // gamma 0.25 from 3072: a decay leaves 2304, and grade g deposits 256 g.
    void Local()
    {
        SpineScores scores(kSpines, WithThresholds());
        SpineScores reference(kSpines, ScoresFixture::Defaults());
        std::vector<uint8_t> grades(kSpines, 3);
        grades[4] = 0;
        grades[6] = 2;
        Send(scores, 0, 10, 0, 100);
        Send(scores, 3, 10, 3, 100);
        Send(scores, 4, 10, 3, 100);
        // Lost sends counted on a spine nothing was sent on since the last
        // report.
        Send(scores, 6, 0, 2, 100);
        scores.OnReport(ScoresFixture::Report(1, grades), kInterval);
        NS_TEST_EXPECT_MSG_EQ(scores.Score(0), 3072u, "no loss keeps the top grade");
        NS_TEST_EXPECT_MSG_EQ(scores.Score(3), 2304u + 256u, "3 of 10 lost grades the spine 1");
        NS_TEST_EXPECT_MSG_EQ(scores.Score(4), 2304u, "the receiver's lower grade wins");
        NS_TEST_EXPECT_MSG_EQ(scores.Score(6), 2304u, "losses with no sends grade the spine 0");
        std::vector<uint8_t> combined = grades;
        combined[3] = 1;
        combined[6] = 0;
        reference.OnReport(ScoresFixture::Report(1, combined), kInterval);
        ExpectScores(scores, reference, "the first report, combined");

        // The counts restarted at the report: the next one stands alone.
        std::vector<uint8_t> top(kSpines, 3);
        scores.OnReport(ScoresFixture::Report(2, top), 2 * kInterval);
        reference.OnReport(ScoresFixture::Report(2, top), 2 * kInterval);
        ExpectScores(scores, reference, "the counts restart at every report taken");

        // A stale report is not taken and leaves the counts to the next.
        Send(scores, 2, 4, 4, 2 * kInterval + 100);
        scores.OnReport(ScoresFixture::Report(1, top), 2 * kInterval + 200);
        ExpectScores(scores, reference, "a stale report deposits nothing");
        scores.OnReport(ScoresFixture::Report(3, top), 3 * kInterval);
        std::vector<uint8_t> lost = top;
        lost[2] = 0;
        reference.OnReport(ScoresFixture::Report(3, lost), 3 * kInterval);
        ExpectScores(scores, reference, "the counts reach the next report taken");
    }

    // Two destinations pooled; the pool's loss interval is the report
    // interval, kInterval.
    void Pool()
    {
        const SpineScores::Parameters parameters = WithThresholds();
        SpineScores pool(kSpines, parameters);
        SpineScores toFirst(kSpines, parameters);
        SpineScores toSecond(kSpines, parameters);
        toFirst.Pool(pool, 0.5);
        toSecond.Pool(pool, 0.5);
        SpineScores referencePool(kSpines, ScoresFixture::Defaults());
        const std::vector<uint8_t> top(kSpines, 3);
        std::vector<Closed> closed;
        pool.SetLossIntervalCallback(Recorder(closed));

        // First interval: spine 3 loses none of 10 towards the first and 6 of
        // 10 towards the second, 6 of 20 from the host; spine 5 loses a send
        // but carries none, which grades it 0.
        Send(toFirst, 3, 10, 0, 1000);
        Send(toSecond, 3, 10, 6, 2000);
        Send(toFirst, 5, 0, 1, 3000);
        Send(toSecond, 0, 10, 0, 4000);
        toFirst.OnReport(ScoresFixture::Report(1, top), kInterval + 5000);
        NS_TEST_EXPECT_MSG_EQ(toFirst.Score(3), 3072u, "the first lost nothing on spine 3");
        NS_TEST_EXPECT_MSG_EQ(pool.Score(3),
                              2304u + 256u,
                              "the pool grades spine 3 by the host's 6 of 20, grade 1");
        std::vector<uint8_t> expected = top;
        expected[3] = 1;
        expected[5] = 0;
        referencePool.OnReport(ScoresFixture::Report(1, expected), kInterval + 5000);
        ExpectScores(pool, referencePool, "spine 5, lost on and not sent on, is graded 0");

        // The second destination's own loss, 6 of 10, grades spine 3 0, below
        // the pool's grade.
        toSecond.OnReport(ScoresFixture::Report(1, top), kInterval + 6000);
        NS_TEST_EXPECT_MSG_EQ(toSecond.Score(3), 2304u, "6 of 10 lost grades the spine 0");
        expected[3] = 0;
        referencePool.OnReport(ScoresFixture::Report(2, expected), kInterval + 6000);
        ExpectScores(pool, referencePool, "the pool takes the lower of the two grades");

        // Second interval: the second destination loses all 4 sends on spine
        // 4. The first destination's next report, after the interval ends,
        // grades spine 4 by the host's counts, and spine 3, whose counts
        // restarted, at the top.
        Send(toSecond, 4, 4, 4, kInterval + 7000);
        toFirst.OnReport(ScoresFixture::Report(2, top), 2 * kInterval + 5000);
        NS_TEST_EXPECT_MSG_EQ(toFirst.Score(4), 3072u, "the first sent nothing on spine 4");
        expected = top;
        expected[4] = 0;
        referencePool.OnReport(ScoresFixture::Report(3, expected), 2 * kInterval + 5000);
        ExpectScores(pool, referencePool, "the pool's counts restart at each interval's end");

        // Two intervals later with nothing counted in between: every spine at
        // the top.
        Send(toSecond, 6, 2, 2, 2 * kInterval + 6000);
        toFirst.OnReport(ScoresFixture::Report(3, top), 4 * kInterval + 5000);
        referencePool.OnReport(ScoresFixture::Report(4, top), 4 * kInterval + 5000);
        ExpectScores(pool, referencePool, "an interval with nothing counted grades no spine down");

        // One record per interval closed, with the counts above: the first
        // interval's, the second's, and the third's, whose 2 of 2 lost on
        // spine 6 grade it 0 although the fourth, which counted nothing and
        // is not passed, is the one the pool grades by.
        NS_TEST_ASSERT_MSG_EQ(closed.size(), 3u, "a record per interval closed");
        std::vector<Closed> expectedClosed(3);
        expectedClosed[0].endNs = kInterval;
        expectedClosed[1].endNs = 2 * kInterval;
        expectedClosed[2].endNs = 3 * kInterval;
        for (Closed& interval : expectedClosed)
        {
            interval.sent.assign(kSpines, 0);
            interval.lost.assign(kSpines, 0);
            interval.grades = top;
        }
        expectedClosed[0].sent[3] = 20;
        expectedClosed[0].lost[3] = 6;
        expectedClosed[0].grades[3] = 1;
        expectedClosed[0].lost[5] = 1;
        expectedClosed[0].grades[5] = 0;
        expectedClosed[0].sent[0] = 10;
        expectedClosed[1].sent[4] = 4;
        expectedClosed[1].lost[4] = 4;
        expectedClosed[1].grades[4] = 0;
        expectedClosed[2].sent[6] = 2;
        expectedClosed[2].lost[6] = 2;
        expectedClosed[2].grades[6] = 0;
        ExpectClosed(closed, expectedClosed);
    }

    // A loss interval the pool closed, as its callback passes it.
    struct Closed
    {
        uint64_t endNs = 0;
        std::vector<uint32_t> sent, lost;
        std::vector<uint8_t> grades;
    };

    static SpineScores::LossIntervalCallback Recorder(std::vector<Closed>& closed)
    {
        return [&closed](uint64_t endNs,
                         const std::vector<uint32_t>& sent,
                         const std::vector<uint32_t>& lost,
                         const std::vector<uint8_t>& grades) {
            closed.push_back(Closed{endNs, sent, lost, grades});
        };
    }

    void ExpectClosed(const std::vector<Closed>& closed, const std::vector<Closed>& expected)
    {
        NS_TEST_ASSERT_MSG_EQ(closed.size(), expected.size(), "a record per interval closed");
        for (uint32_t i = 0; i < expected.size(); ++i)
        {
            NS_TEST_EXPECT_MSG_EQ(closed[i].endNs, expected[i].endNs, "the interval's end");
            NS_TEST_ASSERT_MSG_EQ(closed[i].sent.size(), uint32_t(kSpines), "every spine");
            for (uint32_t spine = 0; spine < kSpines; ++spine)
            {
                NS_TEST_EXPECT_MSG_EQ(closed[i].sent[spine], expected[i].sent[spine], "sent");
                NS_TEST_EXPECT_MSG_EQ(closed[i].lost[spine], expected[i].lost[spine], "lost");
                NS_TEST_EXPECT_MSG_EQ(uint32_t(closed[i].grades[spine]),
                                      uint32_t(expected[i].grades[spine]),
                                      "the grade the interval's lost fraction earns");
            }
        }
    }

    // A host whose queue pairs' scores are pooled traces each loss interval
    // its pool closes through SpineLossInterval.
    void PoolTrace()
    {
        IsolatedHost sender(LoadBalancingMode::SprayPolicy, kTestReceiver, kSpines);
        sender.hw->m_sprayBaseRttNs = kInterval / 2;
        sender.hw->m_sprayLossThreshold1 = kThresholds[0];
        sender.hw->m_sprayLossThreshold2 = kThresholds[1];
        sender.hw->m_sprayLossThreshold3 = kThresholds[2];
        sender.hw->m_sprayLossIntervalNs = kInterval;
        sender.hw->m_spraySenderPoolWeight = 0.5;
        sender.AddSender(kTestSender, kTestReceiver, 16 * IsolatedHost::kMtu);
        NS_TEST_ASSERT_MSG_NE(sender.hw->m_senderSpineScores, nullptr, "the host pools");
        NS_TEST_ASSERT_MSG_EQ(sender.hw->m_spineScores.size(), 1u, "one host's scores");
        SpineScores& scores = sender.hw->m_spineScores.begin()->second;
        std::vector<Closed> closed;
        const SpineScores::LossIntervalCallback record = Recorder(closed);
        sender.hw->TraceConnectWithoutContext(
            "SpineLossInterval",
            Callback<void,
                     uint64_t,
                     const std::vector<uint32_t>&,
                     const std::vector<uint32_t>&,
                     const std::vector<uint8_t>&>(record));

        Send(scores, 2, 8, 3, 1000);
        Send(scores, 1, 4, 0, 2000);
        Send(scores, 2, 1, 0, kInterval + 1000);
        NS_TEST_EXPECT_MSG_EQ(closed.size(), 1u, "the first send after its end closes it");
        Send(scores, 0, 1, 0, 3 * kInterval + 1000);
        std::vector<Closed> expected(2);
        expected[0].endNs = kInterval;
        expected[1].endNs = 2 * kInterval;
        for (Closed& interval : expected)
        {
            interval.sent.assign(kSpines, 0);
            interval.lost.assign(kSpines, 0);
            interval.grades.assign(kSpines, 3);
        }
        expected[0].sent[2] = 8;
        expected[0].lost[2] = 3;
        expected[0].grades[2] = 1;
        expected[0].sent[1] = 4;
        expected[1].sent[2] = 1;
        ExpectClosed(closed, expected);
        Simulator::Destroy();
    }

    // The acknowledgement of the send of seq along identification's spine,
    // carrying a report that grades every spine 3.
    static void Acknowledge(IsolatedHost& sender,
                            uint32_t seq,
                            uint16_t identification,
                            uint8_t sequence)
    {
        CustomHeader ack;
        ack.l3Prot = 0xFC;
        ack.sip = kTestReceiver.Get();
        ack.ipid = identification;
        ack.ack.sport = IsolatedHost::kReceiverPort;
        ack.ack.dport = IsolatedHost::kSenderPort;
        ack.ack.pg = IsolatedHost::kPriorityGroup;
        ack.ack.flags = 0;
        ack.ack.seq = 0;
        ack.ack.packet_seq = seq;
        SpineReport report;
        report.sequence = sequence;
        ack.ack.spine_report[0] = report.sequence;
        std::copy_n(report.grades, SpineReport::GradeBytes(kSpines), ack.ack.spine_report + 1);
        sender.hw->ReceiveAck(Create<Packet>(), ack);
    }

    // Two queue pairs to one host send; an acknowledgement declares two of
    // the first's sends lost on one spine, and the next report is graded by
    // the sends and losses of both. Under switch_queue the sender counts
    // nothing and deposits the report as it stands.
    void Transport(GradeCostSource source)
    {
        constexpr uint32_t kMtu = IsolatedHost::kMtu;
        constexpr uint32_t kPackets = 16;
        IsolatedHost sender(LoadBalancingMode::SprayPolicy, kTestReceiver, kSpines);
        sender.hw->m_sprayBaseRttNs = kInterval / 2;
        sender.hw->m_sprayGradeCostSource = static_cast<uint32_t>(source);
        sender.hw->m_sprayLossThreshold1 = kThresholds[0];
        sender.hw->m_sprayLossThreshold2 = kThresholds[1];
        sender.hw->m_sprayLossThreshold3 = kThresholds[2];
        const bool counting = source == GradeCostSource::Estimate;
        NS_TEST_EXPECT_MSG_EQ(sender.hw->CountsSprayLoss(),
                              counting,
                              "the sender counts loss outside switch_queue only");
        Ptr<RdmaQueuePair> first = sender.AddSender(kTestSender, kTestReceiver, kPackets * kMtu);
        Ptr<RdmaQueuePair> second =
            sender.AddSender(kTestSender, kTestReceiver, kPackets * kMtu, kOtherPort);
        NS_TEST_ASSERT_MSG_EQ(sender.hw->m_spineScores.size(), 1u, "one host's scores");
        SpineScores& scores = sender.hw->m_spineScores.begin()->second;
        SpineScores reference(kSpines, ScoresFixture::Defaults());
        const std::vector<uint8_t> top(kSpines, 3);
        scores.OnReport(ScoresFixture::Report(1, top), 0);
        reference.OnReport(ScoresFixture::Report(1, top), 0);

        std::vector<uint32_t> sent(kSpines, 0);
        std::vector<uint16_t> identifications;
        for (uint32_t i = 0; i < kPackets; ++i)
        {
            for (Ptr<RdmaQueuePair> qp : {first, second})
            {
                CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header);
                sender.hw->GetNxtPacket(qp)->PeekHeader(parsed);
                sent[RequestedSpine(parsed.ipid)]++;
                if (qp == first)
                {
                    identifications.push_back(parsed.ipid);
                }
            }
        }
        // The first queue pair's sends on the spine it sent most on.
        std::vector<std::vector<uint32_t>> bySpine(kSpines);
        for (uint32_t i = 0; i < kPackets; ++i)
        {
            bySpine[RequestedSpine(identifications[i])].push_back(i);
        }
        const uint8_t spine =
            std::max_element(bySpine.begin(),
                             bySpine.end(),
                             [](const auto& a, const auto& b) { return a.size() < b.size(); }) -
            bySpine.begin();
        const std::vector<uint32_t>& onSpine = bySpine[spine];
        NS_TEST_ASSERT_MSG_GT(onSpine.size(), 2, "the draw puts three sends on one spine");
        // The report it carries is the one already taken.
        Acknowledge(sender, onSpine[2] * kMtu, identifications[onSpine[2]], 1);
        NS_TEST_EXPECT_MSG_EQ(first->m_recovery_events, 2, "two sends declared lost");
        Acknowledge(sender, onSpine[2] * kMtu, identifications[onSpine[2]], 2);

        std::vector<uint8_t> expected = top;
        if (counting)
        {
            expected[spine] = QuantizeGrade(std::min(1.0, 2.0 / sent[spine]), kThresholds);
            NS_TEST_EXPECT_MSG_LT(uint32_t(expected[spine]), 3u, "the loss costs a grade");
        }
        reference.OnReport(ScoresFixture::Report(2, expected), 0);
        ExpectScores(scores,
                     reference,
                     counting ? "both queue pairs' sends and the losses grade the host's scores"
                              : "under switch_queue the sender's losses are not counted");
        Simulator::Destroy();
    }

    // Under switch_queue the grader grades the paths' loss through the loss
    // thresholds, and through the congestion thresholds while they are unset.
    void Oracle()
    {
        std::vector<SpineGrader::SpineValues> values(GraderFixture::kSpines);
        values[6].lossFraction = 0.3;
        const std::vector<bool> held(GraderFixture::kSpines, false);
        SpineGrader::Parameters parameters = GraderFixture::Absolute();
        NS_TEST_EXPECT_MSG_EQ(
            uint32_t(SpineGrader(parameters).Graded(values, held, 0.875).Grade(6)),
            2u,
            "unset, a loss of 0.3 grades 2 through the congestion thresholds");
        parameters.lossThresholds = kThresholds;
        const SpineReport report = SpineGrader(parameters).Graded(values, held, 0.875);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(6)), 1u, "and 1 through the loss thresholds");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(0)), 3u, "a spine without loss keeps 3");
        values[6].lossFraction = 0;
        values[6].markFraction = 0.3;
        NS_TEST_EXPECT_MSG_EQ(
            uint32_t(SpineGrader(parameters).Graded(values, held, 0.875).Grade(6)),
            2u,
            "the marked fraction keeps the congestion thresholds");

        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        hw->SetNode(CreateObject<Node>());
        hw->m_sprayBaseRttNs = 10000;
        hw->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayPolicy);
        hw->m_sprayGradeCostSource = static_cast<uint32_t>(GradeCostSource::SwitchQueue);
        NS_TEST_EXPECT_MSG_EQ(hw->GraderParameters().lossThresholds.has_value(),
                              false,
                              "unset, the grader has no loss thresholds");
        hw->m_sprayLossThreshold1 = kThresholds[0];
        hw->m_sprayLossThreshold2 = kThresholds[1];
        hw->m_sprayLossThreshold3 = kThresholds[2];
        const std::optional<std::array<double, 3>> filled = hw->GraderParameters().lossThresholds;
        NS_TEST_ASSERT_MSG_EQ(filled.has_value(), true, "the grader takes the loss thresholds");
        NS_TEST_EXPECT_MSG_EQ((*filled == kThresholds), true, "as set");
    }

    // The sender's counting needs the send records selective retransmission
    // keeps; under switch_queue it counts nothing.
    void Validation()
    {
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        hw->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayPolicy);
        hw->m_selective_retransmission = false;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidSprayLoss(), true, "unset thresholds need nothing");
        hw->m_sprayLossThreshold1 = kThresholds[0];
        hw->m_sprayLossThreshold2 = kThresholds[1];
        hw->m_sprayLossThreshold3 = kThresholds[2];
        NS_TEST_EXPECT_MSG_EQ(hw->ValidSprayLoss(),
                              false,
                              "thresholds without selective retransmission are refused");
        hw->m_selective_retransmission = true;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidSprayLoss(), true, "with it they are accepted");
        hw->m_selective_retransmission = false;
        hw->m_sprayGradeCostSource = static_cast<uint32_t>(GradeCostSource::SwitchQueue);
        NS_TEST_EXPECT_MSG_EQ(hw->ValidSprayLoss(),
                              true,
                              "under switch_queue the sender counts nothing to need them for");
    }
};

class SpineHeldExplorationTest : public TestCase
{
  public:
    SpineHeldExplorationTest()
        : TestCase("Under the sentinel held exploration a spine graded 0 has no share of the "
                   "epsilon term and is sent one sentinel per interval, alone or pooled; none "
                   "is refused outside switch_queue")
    {
    }

    void DoRun() override
    {
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(hw->m_sprayHeldExploration),
                              static_cast<uint32_t>(SpineScores::HeldExploration::Epsilon),
                              "the default keeps epsilon / N for a held spine");
        hw->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayPolicy);
        hw->m_sprayHeldExploration = SpineScores::HeldExploration::None;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidHeldExploration(), false, "none under the estimate");
        hw->m_sprayGradeCostSource = static_cast<uint32_t>(GradeCostSource::SwitchQueue);
        NS_TEST_EXPECT_MSG_EQ(hw->ValidHeldExploration(), true, "none under switch_queue");

        SpineScores::Parameters parameters = ScoresFixture::Defaults();
        parameters.heldExploration = SpineScores::HeldExploration::Sentinel;
        const double epsilon = parameters.epsilon;
        SpineScores scores(kSpines, parameters);
        ScoresFixture::Feed(scores, std::vector<uint8_t>(40, 0));
        NS_TEST_EXPECT_MSG_EQ(scores.Score(3), 0u, "grade 0 drains the score");
        NS_TEST_EXPECT_MSG_EQ(scores.Share(3), 0.0, "a held spine has no epsilon term");
        ExpectShares(scores, epsilon / 7, "held");
        Ptr<UniformRandomVariable> random =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(0));
        PolicySpineSelector selector(random, scores);
        uint32_t sentinels = 0;
        for (uint64_t now = 41 * kInterval; now < 51 * kInterval; now += 100)
        {
            sentinels += RequestedSpine(selector.Choose(now)) == 3;
        }
        NS_TEST_EXPECT_MSG_EQ(sentinels, 10u, "one draw per sentinel interval on the held spine");
        ScoresFixture::Feed(scores, {3}, 52);
        NS_TEST_EXPECT_MSG_EQ(scores.Held(3), false, "a grade-3 report clears the hold");
        ExpectShares(scores, epsilon / kSpines, "released");

        // Pooled: the pool's grade holds the spine, and a lost sentinel keeps
        // the pool's loss grade, and so the hold, at 0.
        parameters.lossThresholds = std::array<double, 3>{0.1, 0.25, 0.5};
        SpineScores pool(kSpines, parameters);
        SpineScores toFirst(kSpines, parameters);
        SpineScores toSecond(kSpines, parameters);
        toFirst.Pool(pool, 0.5);
        toSecond.Pool(pool, 0.5);
        ScoresFixture::Feed(toFirst, std::vector<uint8_t>(40, 0));
        NS_TEST_EXPECT_MSG_EQ(toSecond.Held(3), true, "the pooled grade holds the spine");
        double pooledSum = 0;
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            pooledSum += toFirst.Share(spine);
        }
        NS_TEST_EXPECT_MSG_EQ_TOL(pooledSum, 1.0, 1e-9, "the pooled shares sum to one");
        NS_TEST_EXPECT_MSG_EQ(toFirst.Share(3), 0.0, "and the held spine's is 0");
        PolicySpineSelector pooled(random, toSecond);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(RequestedSpine(pooled.Choose(40 * kInterval + 100))),
                              3u,
                              "the sentinel is due");
        pooled.OnLoss(3, 40 * kInterval + 200);
        std::vector<uint8_t> top(kSpines, 3);
        toFirst.OnReport(ScoresFixture::Report(41, top), 41 * kInterval + 100);
        NS_TEST_EXPECT_MSG_EQ(toSecond.Held(3), true, "a lost sentinel keeps the hold");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(RequestedSpine(pooled.Choose(41 * kInterval + 200))),
                              3u,
                              "the next sentinel is due an interval later");
        toFirst.OnReport(ScoresFixture::Report(42, top), 42 * kInterval + 100);
        NS_TEST_EXPECT_MSG_EQ(toSecond.Held(3), false, "a delivered one lets grade 3 clear it");

        // A sentinel in the window after spine 5 fell silent, against a
        // median of about 143 arrivals: below the floor's share, 1.43, which
        // only releaseOnOneArrival lets one arrival pass.
        for (const bool oneArrival : {false, true})
        {
            SpineGrader::Parameters graderParameters = GraderFixture::Defaults();
            graderParameters.releaseOnOneArrival = oneArrival;
            SpineGrader grader(graderParameters);
            for (uint32_t interval = 0; interval < 5; ++interval)
            {
                grader.Advance(interval * kInterval);
                for (uint32_t i = 0; i < 1000; ++i)
                {
                    const uint8_t k = i % 7 < 5 ? i % 7 : i % 7 + 1;
                    grader.OnArrival(k, k, false, 0);
                }
                if (interval == 4)
                {
                    grader.OnArrival(5, 5, false, 0);
                }
            }
            grader.Advance(5 * kInterval);
            NS_TEST_EXPECT_MSG_EQ(grader.LastInterval()[5].held,
                                  !oneArrival,
                                  "one arrival per window releases only with the flag");
        }
    }

  private:
    static constexpr uint32_t kSpines = ScoresFixture::kSpines;
    static constexpr uint64_t kInterval = ScoresFixture::kInterval;

    // Each spine's share is its score's part and the epsilon term, which is
    // 0 for held spine 3 and unheld for the others; the shares sum to one.
    void ExpectShares(const SpineScores& scores, double unheld, const std::string& message)
    {
        const double epsilon = ScoresFixture::Defaults().epsilon;
        double scoreSum = 0;
        double sum = 0;
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            scoreSum += scores.Score(spine);
            sum += scores.Share(spine);
        }
        NS_TEST_EXPECT_MSG_EQ_TOL(sum, 1.0, 1e-9, message + ": the shares sum to one");
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            const double term = scores.Held(spine) ? 0 : unheld;
            NS_TEST_EXPECT_MSG_EQ_TOL(scores.Share(spine),
                                      (1 - epsilon) * scores.Score(spine) / scoreSum + term,
                                      1e-12,
                                      message);
        }
    }
};

class SpinePoolHoldPropagationTest : public TestCase
{
  public:
    SpinePoolHoldPropagationTest()
        : TestCase("With pool hold propagation a spine the pool holds weighs nothing in every "
                   "destination's draw and is sent the pool's sentinels, one per interval for "
                   "the host; a spine held only by a destination keeps its own treatment")
    {
    }

    void DoRun() override
    {
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        BooleanValue attribute;
        hw->GetAttribute("SprayPoolHoldPropagates", attribute);
        NS_TEST_EXPECT_MSG_EQ(attribute.Get(), false, "SprayPoolHoldPropagates is false by default");
        NS_TEST_EXPECT_MSG_EQ(ScoresFixture::Defaults().poolHoldPropagates,
                              false,
                              "and so is the scores' parameter");
        hw->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayPolicy);
        hw->m_sprayPoolHoldPropagates = true;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidPoolHoldPropagates(), false, "refused without a pool");
        hw->m_spraySenderPoolWeight = 0.75;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidPoolHoldPropagates(), true, "read with a pool weight");

        Shares();
        Draws();
        EpsilonExploration();
        Release();
        Sentinels();
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = ScoresFixture::kSpines;
    static constexpr uint64_t kInterval = ScoresFixture::kInterval;
    static constexpr double kWeight = 0.75;

    // A pool counting loss with a loss hold of 4 clean sends and a pool
    // decay interval of one report interval, and two destinations pooled
    // with it at weight 0.75 that count nothing themselves. The first sends
    // spine 3 ten packets, six lost, and both report: the first grades spine
    // 5 at 0, the second every spine 3. The pool holds spine 3, by its loss
    // grade and its held flag, and not spine 5, whose mean grade is 1.5; the
    // first destination holds spine 5 and the second nothing.
    struct Setup
    {
        std::unique_ptr<SpineScores> pool, toFirst, toSecond;
    };

    static Setup Make(bool propagates,
                      SpineScores::HeldExploration exploration =
                          SpineScores::HeldExploration::Sentinel)
    {
        SpineScores::Parameters local = ScoresFixture::Defaults();
        local.heldExploration = exploration;
        local.poolHoldPropagates = propagates;
        SpineScores::Parameters pooled = local;
        pooled.lossThresholds = std::array<double, 3>{0.1, 0.25, 0.5};
        pooled.lossHoldCleanSentinels = 4;
        pooled.poolDecayIntervalNs = kInterval;
        Setup setup;
        setup.pool = std::make_unique<SpineScores>(kSpines, pooled);
        setup.toFirst = std::make_unique<SpineScores>(kSpines, local);
        setup.toSecond = std::make_unique<SpineScores>(kSpines, local);
        setup.toFirst->Pool(*setup.pool, kWeight);
        setup.toSecond->Pool(*setup.pool, kWeight);
        for (uint32_t i = 0; i < 10; ++i)
        {
            setup.toFirst->OnSend(3, 6000);
            if (i < 6)
            {
                setup.toFirst->OnLoss(3, 6000);
            }
        }
        std::vector<uint8_t> grades(kSpines, 3);
        grades[5] = 0;
        setup.toFirst->OnReport(ScoresFixture::Report(1, grades), 15000);
        setup.toSecond->OnReport(ScoresFixture::Report(1, std::vector<uint8_t>(kSpines, 3)),
                                 16000);
        return setup;
    }

    static double Mixed(const Setup& setup, const SpineScores& scores, uint32_t spine)
    {
        return kWeight * setup.pool->Score(spine) + (1 - kWeight) * scores.Score(spine);
    }

    // Each spine's share: the part of 1 - epsilon its mixed score earns over
    // the spines not excluded, plus its epsilon term, which is spread over
    // the spines held for neither scores; the shares sum to one.
    void ExpectShares(const Setup& setup,
                      const SpineScores& scores,
                      const std::vector<uint32_t>& excluded,
                      const std::vector<uint32_t>& held,
                      const std::string& message)
    {
        const double epsilon = ScoresFixture::Defaults().epsilon;
        auto in = [](const std::vector<uint32_t>& set, uint32_t spine) {
            return std::find(set.begin(), set.end(), spine) != set.end();
        };
        double mixedSum = 0;
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            mixedSum += in(excluded, spine) ? 0 : Mixed(setup, scores, spine);
        }
        double sum = 0;
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            sum += scores.Share(spine);
            const double part =
                in(excluded, spine) ? 0 : (1 - epsilon) * Mixed(setup, scores, spine) / mixedSum;
            const double term = in(held, spine) ? 0 : epsilon / (kSpines - held.size());
            NS_TEST_EXPECT_MSG_EQ_TOL(scores.Share(spine),
                                      part + term,
                                      1e-12,
                                      message + ", spine " + std::to_string(spine));
            NS_TEST_EXPECT_MSG_EQ(scores.Held(spine),
                                  in(held, spine),
                                  message + ": held, spine " + std::to_string(spine));
        }
        NS_TEST_EXPECT_MSG_EQ_TOL(sum, 1.0, 1e-9, message + ": the shares sum to one");
    }

    void Shares()
    {
        Setup today = Make(false);
        NS_TEST_EXPECT_MSG_EQ(today.pool->Held(3), true, "the pool holds spine 3");
        NS_TEST_EXPECT_MSG_EQ(today.pool->Held(5), false, "and not spine 5");
        NS_TEST_EXPECT_MSG_GT(today.toFirst->Score(3), 0u, "the destinations' scores do not");
        // Unset: the pool's flag alone holds, and spine 3 keeps (1 - w) of
        // the destination's score.
        ExpectShares(today, *today.toFirst, {}, {3}, "unset, first");
        ExpectShares(today, *today.toSecond, {}, {3}, "unset, second");
        NS_TEST_EXPECT_MSG_GT(today.toFirst->Share(3), 0.0, "unset, the pool's hold leaks");

        Setup set = Make(true);
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(set.pool->Score(spine),
                                  today.pool->Score(spine),
                                  "the pool's own scores do not depend on propagation");
        }
        ExpectShares(set, *set.toFirst, {3}, {3, 5}, "set, first");
        ExpectShares(set, *set.toSecond, {3}, {3}, "set, second");
        NS_TEST_EXPECT_MSG_EQ(set.toFirst->Share(3), 0.0, "the pool-held spine has no share");
        NS_TEST_EXPECT_MSG_EQ(set.toSecond->Share(3), 0.0, "in either destination");
        NS_TEST_EXPECT_MSG_GT(set.toFirst->Share(5),
                              0.0,
                              "a spine held only locally keeps the pool's part of its share");

        // The local hold ends by the local rule.
        set.toFirst->OnReport(ScoresFixture::Report(2, std::vector<uint8_t>(kSpines, 3)), 17000);
        NS_TEST_EXPECT_MSG_EQ(set.toFirst->Held(5), false, "a grade-3 report clears it");
        NS_TEST_EXPECT_MSG_EQ(set.toFirst->Held(3), true, "the pool's hold stands");
    }

    void Draws()
    {
        for (const bool propagates : {false, true})
        {
            Setup setup = Make(propagates);
            Ptr<UniformRandomVariable> random =
                CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(0));
            uint32_t onHeld = 0;
            for (uint32_t i = 0; i < 2000; ++i)
            {
                SpineScores& scores = i % 2 == 0 ? *setup.toFirst : *setup.toSecond;
                onHeld += scores.Choose(*random, 16000 + i) == 3;
            }
            if (propagates)
            {
                NS_TEST_EXPECT_MSG_EQ(onHeld, 0u, "set, no draw lands on the pool-held spine");
            }
            else
            {
                NS_TEST_EXPECT_MSG_GT(onHeld, 0u, "unset, draws land on it");
            }
        }
    }

    // Under the epsilon held exploration the pool-held spine keeps epsilon
    // / N and nothing of its mixed score.
    void EpsilonExploration()
    {
        Setup setup = Make(true, SpineScores::HeldExploration::Epsilon);
        const double epsilon = ScoresFixture::Defaults().epsilon;
        NS_TEST_EXPECT_MSG_EQ_TOL(setup.toSecond->Share(3),
                                  epsilon / kSpines,
                                  1e-12,
                                  "epsilon / N for the pool-held spine");
        double sum = 0;
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            sum += setup.toSecond->Share(spine);
        }
        NS_TEST_EXPECT_MSG_EQ_TOL(sum, 1.0, 1e-9, "the shares sum to one");
    }

    // Four clean sends on spine 3 and a report closing their loss interval
    // end the pool's loss hold and, the pooled grade now 3, its flag: the
    // share comes back in both destinations.
    void Release()
    {
        Setup setup = Make(true);
        for (uint32_t i = 0; i < 4; ++i)
        {
            setup.toSecond->OnSend(3, 16500);
        }
        NS_TEST_EXPECT_MSG_EQ(setup.toSecond->Share(3), 0.0, "held until the interval closes");
        setup.toSecond->OnReport(ScoresFixture::Report(2, std::vector<uint8_t>(kSpines, 3)),
                                 25000);
        NS_TEST_EXPECT_MSG_EQ(setup.pool->Held(3), false, "the pool releases spine 3");
        ExpectShares(setup, *setup.toFirst, {}, {5}, "released, first");
        ExpectShares(setup, *setup.toSecond, {}, {}, "released, second");
        NS_TEST_EXPECT_MSG_GT(setup.toSecond->Share(3), 0.0, "the share is restored");
    }

    // Two destinations drawing in turn for ten sentinel intervals: unset,
    // each sends spine 3 its own sentinels, besides the draws that land on
    // it; set, the host sends it the pool's, one per interval.
    void Sentinels()
    {
        for (const bool propagates : {false, true})
        {
            Setup setup = Make(propagates);
            Ptr<UniformRandomVariable> random =
                CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(0));
            PolicySpineSelector first(random, *setup.toFirst);
            PolicySpineSelector second(random, *setup.toSecond);
            uint32_t onHeld = 0;
            uint32_t onLocal = 0;
            uint32_t step = 0;
            for (uint64_t now = 16100; now < 16100 + 10 * kInterval; now += 100, ++step)
            {
                PolicySpineSelector& selector = step % 2 == 0 ? first : second;
                const uint8_t spine = RequestedSpine(selector.Choose(now));
                onHeld += spine == 3;
                onLocal += step % 2 == 0 && spine == 5;
            }
            if (propagates)
            {
                NS_TEST_EXPECT_MSG_EQ(onHeld, 10u, "set, one sentinel per interval for the host");
                NS_TEST_EXPECT_MSG_GT_OR_EQ(onLocal,
                                            10u,
                                            "the locally held spine keeps its own sentinels");
            }
            else
            {
                NS_TEST_EXPECT_MSG_GT_OR_EQ(onHeld, 20u, "unset, one per interval per destination");
            }
        }
    }
};

/**
 * Scores used as a loss hold alone, as RdmaHw makes them under PathSelector
 * RepsHold: eight spines, a loss interval and sentinel interval of 10 us, the
 * loss thresholds 0.1, 0.25 and 0.5, and a hold of three clean sentinels.
 */
class LossHoldFixture
{
  public:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint64_t kInterval = 10000;
    static constexpr uint32_t kClean = 3;

    static Ptr<RdmaHw> Hw()
    {
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        hw->m_spineCount = kSpines;
        hw->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
        hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::RepsHold);
        hw->m_sprayLossThreshold1 = 0.1;
        hw->m_sprayLossThreshold2 = 0.25;
        hw->m_sprayLossThreshold3 = 0.5;
        hw->m_sprayLossIntervalNs = kInterval;
        hw->m_sprayLossHoldCleanSentinels = kClean;
        hw->m_sprayHeldExploration = SpineScores::HeldExploration::Sentinel;
        hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Host);
        return hw;
    }

    static std::unique_ptr<RepsHoldSelector> Selector(SpineScores& hold)
    {
        return std::make_unique<RepsHoldSelector>(SelectorStream(), 8, 1000000, 20, kSpines, hold);
    }

    // One send of selector at nowNs, as the transport answers it: a send on a
    // spine in lossy is declared lost, and of the others every second one is
    // acknowledged unmarked and the rest marked, so that the buffer drains
    // and fresh spines are drawn too. The spine sent on.
    static uint8_t Send(PathSelector& selector,
                        uint64_t nowNs,
                        uint32_t step,
                        const std::set<uint8_t>& lossy)
    {
        const uint8_t spine = RequestedSpine(selector.Choose(nowNs));
        if (lossy.count(spine) > 0)
        {
            selector.OnLoss(spine, nowNs);
        }
        else
        {
            selector.OnAck(spine, step % 2 == 1, nowNs);
        }
        return spine;
    }
};

class RepsHoldTest : public TestCase
{
  public:
    RepsHoldTest()
        : TestCase("Under reps_hold a spine whose losses grade it 0 leaves every flow's REPS "
                   "choices, is sent one sentinel per interval for the host, and returns after "
                   "the clean sentinels; the policy is refused without its loss hold")
    {
    }

    void DoRun() override
    {
        Validation();
        Hold();
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = LossHoldFixture::kSpines;
    static constexpr uint64_t kInterval = LossHoldFixture::kInterval;

    void Validation()
    {
        NS_TEST_EXPECT_MSG_EQ(LossHoldFixture::Hw()->ValidRepsHold(), true, "a full loss hold");
        NS_TEST_EXPECT_MSG_EQ(CreateObject<RdmaHw>()->ValidRepsHold(), true, "not reps_hold");
        const std::vector<std::pair<std::function<void(RdmaHw&)>, std::string>> missing = {
            {[](RdmaHw& hw) {
                 hw.m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::EntropyHash);
             },
             "entropy values name no spine"},
            {[](RdmaHw& hw) { hw.m_sprayLossThreshold1 = 0; }, "no loss thresholds"},
            {[](RdmaHw& hw) { hw.m_sprayLossIntervalNs = 0; }, "no loss interval"},
            {[](RdmaHw& hw) { hw.m_sprayLossHoldCleanSentinels = 0; }, "no loss hold"},
            {[](RdmaHw& hw) { hw.m_sprayHeldExploration = SpineScores::HeldExploration::Epsilon; },
             "no sentinels"},
        };
        for (const auto& [unset, message] : missing)
        {
            Ptr<RdmaHw> hw = LossHoldFixture::Hw();
            unset(*hw);
            NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsHold(), false, "refused: " + message);
        }
        Ptr<RdmaHw> hw = LossHoldFixture::Hw();
        NS_TEST_EXPECT_MSG_EQ(hw->CountsSprayLoss(), true, "the hold counts the sender's losses");
        hw->m_selective_retransmission = false;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidSprayLoss(), false, "from selective retransmission's records");
        const SpineScores::Parameters parameters = hw->LossHoldParameters();
        NS_TEST_EXPECT_MSG_EQ(parameters.lossIntervalNs, kInterval, "the loss interval");
        NS_TEST_EXPECT_MSG_EQ(parameters.lossHoldCleanSentinels, 3u, "the clean sentinels");
    }

    // Two flows of one host share the hold. Over the first interval every
    // send on spine 3 is lost; over the next three each is sent its
    // sentinel, which arrives; after the third the spine is chosen again.
    void Hold()
    {
        Ptr<RdmaHw> hw = LossHoldFixture::Hw();
        SpineScores hold(kSpines, hw->LossHoldParameters());
        std::unique_ptr<RepsHoldSelector> flows[2] = {LossHoldFixture::Selector(hold),
                                                      LossHoldFixture::Selector(hold)};
        uint32_t step = 0;
        std::vector<uint32_t> onSpine3(5);
        std::vector<uint32_t> sentinelsByFlow(2);
        for (uint32_t interval = 0; interval < 5; ++interval)
        {
            for (uint32_t i = 0; i < 100; ++i, ++step)
            {
                const uint64_t now = interval * kInterval + 100 * i;
                const std::set<uint8_t> lossy =
                    interval == 0 ? std::set<uint8_t>{3} : std::set<uint8_t>{};
                const uint8_t spine =
                    LossHoldFixture::Send(*flows[step % 2], now, step, lossy);
                onSpine3[interval] += spine == 3;
                if (interval >= 1 && interval <= 3 && spine == 3)
                {
                    sentinelsByFlow[step % 2]++;
                }
            }
            if (interval == 0)
            {
                NS_TEST_EXPECT_MSG_GT(onSpine3[0], 0u, "spine 3 is sent on, and loses");
            }
        }
        NS_TEST_EXPECT_MSG_EQ(onSpine3[1], 1u, "held: one sentinel in the first interval");
        NS_TEST_EXPECT_MSG_EQ(onSpine3[2], 1u, "and in the second");
        NS_TEST_EXPECT_MSG_EQ(onSpine3[3], 1u, "and in the third, for the whole host");
        NS_TEST_EXPECT_MSG_EQ(sentinelsByFlow[0] + sentinelsByFlow[1], 3u, "taken by either flow");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), false, "three clean sentinels release the spine");
        NS_TEST_EXPECT_MSG_GT(onSpine3[4], 1u, "and REPS chooses it again");
    }
};

class MrcHoldTest : public TestCase
{
  public:
    MrcHoldTest()
        : TestCase("Under mrc_hold a spine whose losses grade it 0 leaves every flow's MRC "
                   "choices, is sent one sentinel per interval for the host, and returns after "
                   "the clean sentinels; the policy is refused without its loss hold")
    {
    }

    void DoRun() override
    {
        Validation();
        Hold();
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = LossHoldFixture::kSpines;
    static constexpr uint64_t kInterval = LossHoldFixture::kInterval;

    static Ptr<RdmaHw> Hw()
    {
        Ptr<RdmaHw> hw = LossHoldFixture::Hw();
        hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::MrcHold);
        return hw;
    }

    void Validation()
    {
        Ptr<RdmaHw> full = Hw();
        NS_TEST_EXPECT_MSG_EQ(full->ValidRepsHold(), true, "a full loss hold");
        NS_TEST_EXPECT_MSG_EQ(full->IsRepsHold(), true, "mrc_hold keeps reps_hold's loss hold");
        NS_TEST_EXPECT_MSG_EQ(full->CountsSprayLoss(), true, "and counts the sender's losses");
        const std::vector<std::pair<std::function<void(RdmaHw&)>, std::string>> missing = {
            {[](RdmaHw& hw) {
                 hw.m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::EntropyHash);
             },
             "entropy values name no spine"},
            {[](RdmaHw& hw) { hw.m_sprayLossThreshold1 = 0; }, "no loss thresholds"},
            {[](RdmaHw& hw) { hw.m_sprayLossIntervalNs = 0; }, "no loss interval"},
            {[](RdmaHw& hw) { hw.m_sprayLossHoldCleanSentinels = 0; }, "no loss hold"},
            {[](RdmaHw& hw) { hw.m_sprayHeldExploration = SpineScores::HeldExploration::Epsilon; },
             "no sentinels"},
            {[](RdmaHw& hw) { hw.m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Leaf); },
             "leaf scope without the hosts' leaves"},
            {[](RdmaHw& hw) { hw.m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Auto); },
             "auto scope without the hosts' leaves"},
        };
        for (const auto& [unset, message] : missing)
        {
            Ptr<RdmaHw> hw = Hw();
            unset(*hw);
            NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsHold(), false, "refused: " + message);
        }
    }

    // Two flows of one host share the host's hold, made by StartPathSelection.
    // Over the first interval every send on spine 3, data or probe, is lost,
    // which also takes spine 3 out of MRC's service; from the second on
    // spine 3's probes are answered, which returns it to MRC, but the hold
    // keeps it out of every choice but its sentinel, one per interval for the
    // host, until three clean sentinels release it.
    void Hold()
    {
        Ptr<RdmaHw> hw = Hw();
        hw->SetAttribute("Mtu", UintegerValue(1000));
        hw->SetAttribute("PathRandomVariable", PointerValue(SelectorStream()));
        hw->m_retransmission_timeout_ns = 2000;
        std::vector<Ptr<RdmaQueuePair>> qps;
        for (const char* destination : {"11.0.2.1", "11.0.3.1"})
        {
            qps.push_back(CreateObject<RdmaQueuePair>(
                3, Ipv4Address("11.0.1.1"), Ipv4Address(destination), 10000, 100));
            hw->StartPathSelection(qps.back(), 0, 1000);
            NS_TEST_EXPECT_MSG_NE(dynamic_cast<MrcHoldSelector*>(qps.back()->m_pathSelector.get()),
                                  nullptr,
                                  "MRC over spines with the hold");
        }
        NS_TEST_EXPECT_MSG_NE(hw->m_senderSpineScores.get(), nullptr, "one hold for the host");
        SpineScores& hold = *hw->m_senderSpineScores;
        uint32_t step = 0;
        std::vector<uint32_t> onSpine3(5);
        std::vector<uint32_t> sentinelsByFlow(2);
        uint32_t probesAnswered = 0;
        bool probesNameSpines = true;
        for (uint32_t interval = 0; interval < 5; ++interval)
        {
            for (uint32_t i = 0; i < 100; ++i, ++step)
            {
                const uint64_t now = interval * kInterval + 100 * i;
                PathSelector& flow = *qps[step % 2]->m_pathSelector;
                uint16_t probed;
                if (flow.TakeProbe(now, probed))
                {
                    const uint8_t spine = RequestedSpine(probed);
                    probesNameSpines = probesNameSpines && spine < kSpines &&
                                       CarryingSpine(probed) == spine;
                    if (interval > 0)
                    {
                        flow.OnProbeAnswer(spine, false, now);
                        probesAnswered += spine == 3;
                    }
                }
                const std::set<uint8_t> lossy =
                    interval == 0 ? std::set<uint8_t>{3} : std::set<uint8_t>{};
                const uint8_t spine = LossHoldFixture::Send(flow, now, step / 2, lossy);
                onSpine3[interval] += spine == 3;
                if (interval >= 1 && interval <= 3 && spine == 3)
                {
                    sentinelsByFlow[step % 2]++;
                }
            }
            if (interval == 0)
            {
                NS_TEST_EXPECT_MSG_GT(onSpine3[0], 0u, "spine 3 is sent on, and loses");
            }
        }
        NS_TEST_EXPECT_MSG_EQ(probesNameSpines, true, "a probe names the spine it probes");
        NS_TEST_EXPECT_MSG_GT(probesAnswered, 0u, "spine 3's probes are answered while held");
        NS_TEST_EXPECT_MSG_EQ(onSpine3[1], 1u, "held: one sentinel in the first interval");
        NS_TEST_EXPECT_MSG_EQ(onSpine3[2], 1u, "and in the second");
        NS_TEST_EXPECT_MSG_EQ(onSpine3[3], 1u, "and in the third, for the whole host");
        NS_TEST_EXPECT_MSG_EQ(sentinelsByFlow[0] + sentinelsByFlow[1], 3u, "taken by either flow");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), false, "three clean sentinels release the spine");
        NS_TEST_EXPECT_MSG_GT(onSpine3[4], 1u, "and MRC chooses it again");
    }
};

class MrcHoldExplorationTest : public TestCase
{
  public:
    MrcHoldExplorationTest()
        : TestCase("Under mrc_hold a receiver-held spine leaves MRC's choices and is sent the "
                   "receivers' sentinel, a copy under SpraySentinelMode duplicate; a copy of a "
                   "data packet goes up the held spine with that spine's own path sequence")
    {
    }

    void DoRun() override
    {
        ReceiverHold(SentinelMode::Data);
        ReceiverHold(SentinelMode::Duplicate);
        Duplicate();
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = LossHoldFixture::kSpines;
    static constexpr uint64_t kInterval = LossHoldFixture::kInterval;
    static constexpr uint64_t kSentinelNs = 100000;
    static constexpr uint32_t kReceiverLeaf = 2;

    static Ptr<RdmaHw> Hw(SentinelMode mode)
    {
        Ptr<RdmaHw> hw = LossHoldFixture::Hw();
        hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::MrcHold);
        hw->m_spraySentinelMode = static_cast<uint32_t>(mode);
        return hw;
    }

    struct Sends
    {
        std::vector<uint32_t> live = std::vector<uint32_t>(kSpines, 0);
        std::vector<uint32_t> copies = std::vector<uint32_t>(kSpines, 0);
        uint32_t explorations = 0;
    };

    // count sends from nowNs, each answered clean, its copy too.
    static Sends Draw(PathSelector& selector, uint64_t nowNs, uint32_t count)
    {
        Sends sends;
        for (uint32_t i = 0; i < count; ++i)
        {
            const uint64_t now = nowNs + 10 * i;
            const uint8_t spine = RequestedSpine(selector.Choose(now));
            sends.live[spine]++;
            sends.explorations += selector.ChoseExploration();
            uint8_t copySpine;
            bool trickle;
            if (selector.TakeCopy(copySpine, trickle))
            {
                sends.copies[copySpine]++;
                selector.OnAck(copySpine, false, now);
            }
            selector.OnAck(spine, false, now);
        }
        return sends;
    }

    // A receiver on kReceiverLeaf holds spine 3; the loss hold holds nothing.
    void ReceiverHold(SentinelMode mode)
    {
        const bool duplicate = mode == SentinelMode::Duplicate;
        const std::string which = duplicate ? "duplicate: " : "data: ";
        Ptr<RdmaHw> hw = Hw(mode);
        SpineScores hold(kSpines, hw->LossHoldParameters());
        MrcHoldSelector selector(SelectorStream(), kSpines, 1000, 2000, hold);
        selector.SetReceiverHold(std::make_shared<ReceiverHoldMarks>(kSpines, kSentinelNs),
                                 kReceiverLeaf);
        selector.OnReceiverHold(100, 0x08, 0);
        Sends sends = Draw(selector, 0, 400);
        NS_TEST_EXPECT_MSG_EQ(sends.live[3], 0u, which + "the held spine leaves MRC's choices");
        NS_TEST_EXPECT_MSG_EQ(sends.explorations, 0u, which + "no sentinel before its interval");
        sends = Draw(selector, kSentinelNs, 400);
        // One receivers' sentinel in the interval: the data packet itself,
        // or a copy, beside a data packet that is no exploration send.
        NS_TEST_EXPECT_MSG_EQ(sends.live[3], duplicate ? 0u : 1u, which + "one sentinel on 3");
        NS_TEST_EXPECT_MSG_EQ(sends.copies[3], duplicate ? 1u : 0u, which + "a copy under duplicate");
        NS_TEST_EXPECT_MSG_EQ(sends.explorations, duplicate ? 0u : 1u, which + "the data's flag");
        selector.OnReceiverHold(100, 0x00, 2 * kSentinelNs);
        sends = Draw(selector, 2 * kSentinelNs, 400);
        NS_TEST_EXPECT_MSG_GT(sends.live[3], 10u, which + "a clear bit returns it to MRC");
    }

    // Through the transport: spine 3 is held by the host's loss hold, so each
    // interval's sentinel is a copy of a data packet sent on MRC's choice.
    void Duplicate()
    {
        const IntHeader::Mode savedIntMode = IntHeader::mode;
        const bool savedPathSeq = CustomHeader::udpCarriesPathSeq;
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        IntHeader::mode = IntHeader::NONE;
        CustomHeader::udpCarriesPathSeq = true;
        CustomHeader::ackCarriesPacketSeq = true;

        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, kSpines);
        Ptr<RdmaHw> hw = sender.hw;
        hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::MrcHold);
        hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Host);
        hw->m_sprayLossThreshold1 = 0.1;
        hw->m_sprayLossThreshold2 = 0.25;
        hw->m_sprayLossThreshold3 = 0.5;
        hw->m_sprayLossIntervalNs = kInterval;
        hw->m_sprayLossHoldCleanSentinels = LossHoldFixture::kClean;
        hw->m_sprayHeldExploration = SpineScores::HeldExploration::Sentinel;
        hw->m_spraySentinelMode = static_cast<uint32_t>(SentinelMode::Duplicate);
        hw->m_sprayPathSeq = true;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsHold(), true, "a valid hold");
        NS_TEST_EXPECT_MSG_EQ(hw->ValidPathSeq(), true, "path sequences under mrc_hold");
        Ptr<RdmaQueuePair> qp =
            sender.AddSender(kTestSender, kTestReceiver, 100 * IsolatedHost::kMtu);
        NS_TEST_EXPECT_MSG_NE(dynamic_cast<MrcHoldSelector*>(qp->m_pathSelector.get()),
                              nullptr,
                              "MRC over spines with the hold");
        SpineScores& hold = *hw->m_senderSpineScores;
        for (uint32_t i = 0; i < 20; ++i)
        {
            hold.CountIntervalSend(3, 100 + i);
        }
        for (uint32_t i = 0; i < 10; ++i)
        {
            hold.CountIntervalLoss(3, 200 + i);
        }

        std::vector<uint16_t> nextLive(kSpines, 0);
        bool liveInOrder = true;
        uint32_t liveOn3 = 0;
        std::vector<uint16_t> copyPathSeqs;
        bool copiesMatch = true;
        for (uint32_t interval = 1; interval <= 3; ++interval)
        {
            for (uint32_t i = 0; i < 5; ++i)
            {
                Simulator::Schedule(NanoSeconds(interval * kInterval + 100 * i), [&]() {
                    const size_t before = sender.emitted.size();
                    CustomHeader live(CustomHeader::L2_Header | CustomHeader::L3_Header |
                                      CustomHeader::L4_Header);
                    hw->GetNxtPacket(qp)->PeekHeader(live);
                    const uint8_t spine = RequestedSpine(live.ipid);
                    liveOn3 += spine == 3;
                    liveInOrder = liveInOrder && live.udp.path_seq == nextLive[spine]++;
                    if (sender.emitted.size() > before)
                    {
                        const CustomHeader& copy = sender.emitted.back();
                        copiesMatch = copiesMatch && copy.udp.seq == live.udp.seq &&
                                      RequestedSpine(copy.ipid) == 3;
                        copyPathSeqs.push_back(copy.udp.path_seq);
                    }
                });
            }
        }
        Simulator::Run();
        NS_TEST_EXPECT_MSG_EQ(liveOn3, 0u, "no data packet goes up the held spine");
        NS_TEST_EXPECT_MSG_EQ(liveInOrder, true, "the data packets' path sequences skip nothing");
        NS_TEST_EXPECT_MSG_EQ(copiesMatch, true, "each copy is its data packet on spine 3");
        NS_TEST_ASSERT_MSG_EQ(copyPathSeqs.size(), 3u, "one copy per interval for the host");
        for (uint16_t i = 0; i < 3; ++i)
        {
            NS_TEST_EXPECT_MSG_EQ(copyPathSeqs[i], i, "each copy carries spine 3's path sequence");
        }
        NS_TEST_EXPECT_MSG_EQ(qp->m_exploration_copies, 3u, "each copy with its record");

        CustomHeader::udpCarriesPathSeq = savedPathSeq;
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        IntHeader::mode = savedIntMode;
        Simulator::Destroy();
    }
};

class RepsFlowHoldTest : public TestCase
{
  public:
    RepsFlowHoldTest()
        : TestCase("Under reps_flow_hold each queue pair keeps its own loss hold: a spine one "
                   "flow loses on leaves that flow's choices alone, and is sent that flow's "
                   "sentinels; under reps_hold the queue pairs share the host's")
    {
    }

    void DoRun() override
    {
        Ptr<RdmaHw> shared = Hw(PathSelectorKind::RepsHold);
        Ptr<RdmaQueuePair> first = Qp(shared, 10000);
        Ptr<RdmaQueuePair> second = Qp(shared, 10001);
        NS_TEST_EXPECT_MSG_NE(shared->m_senderSpineScores.get(),
                              nullptr,
                              "reps_hold makes the host's loss hold");

        Ptr<RdmaHw> hw = Hw(PathSelectorKind::RepsFlowHold);
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsHold(), true, "the loss hold is set");
        hw->m_sprayLossHoldCleanSentinels = 0;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsHold(), false, "refused without the loss hold");
        hw->m_sprayLossHoldCleanSentinels = LossHoldFixture::kClean;
        hw->m_sprayHeldExploration = SpineScores::HeldExploration::None;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsHold(), false, "refused without sentinels");
        hw->m_sprayHeldExploration = SpineScores::HeldExploration::Sentinel;
        Ptr<RdmaQueuePair> lossy = Qp(hw, 10000);
        Ptr<RdmaQueuePair> clean = Qp(hw, 10001);
        NS_TEST_EXPECT_MSG_EQ(hw->m_senderSpineScores.get(),
                              nullptr,
                              "reps_flow_hold makes no host-wide hold");

        // Over the first interval every send of the first flow on spine 3 is
        // lost; the second flow loses nothing.
        uint32_t step = 0;
        std::vector<uint32_t> lossyOn3(4);
        std::vector<uint32_t> cleanOn3(4);
        for (uint32_t interval = 0; interval < 4; ++interval)
        {
            for (uint32_t i = 0; i < 200; ++i, ++step)
            {
                const uint64_t now = interval * kInterval + 50 * i;
                const bool firstFlow = step % 2 == 0;
                PathSelector& selector =
                    *(firstFlow ? lossy : clean)->m_pathSelector;
                const std::set<uint8_t> lost =
                    firstFlow && interval == 0 ? std::set<uint8_t>{3} : std::set<uint8_t>{};
                const uint8_t spine = LossHoldFixture::Send(selector, now, step / 2, lost);
                (firstFlow ? lossyOn3 : cleanOn3)[interval] += spine == 3;
            }
        }
        NS_TEST_EXPECT_MSG_GT(lossyOn3[0], 0u, "the first flow loses on spine 3");
        for (uint32_t interval = 1; interval < 4; ++interval)
        {
            NS_TEST_EXPECT_MSG_EQ(lossyOn3[interval],
                                  1u,
                                  "the first flow sends spine 3 its sentinel alone");
            NS_TEST_EXPECT_MSG_GT(cleanOn3[interval], 1u, "the second flow keeps spine 3");
        }
        Simulator::Destroy();
    }

  private:
    static constexpr uint64_t kInterval = LossHoldFixture::kInterval;

    static Ptr<RdmaHw> Hw(PathSelectorKind kind)
    {
        Ptr<RdmaHw> hw = LossHoldFixture::Hw();
        hw->m_pathSelectorKind = static_cast<uint32_t>(kind);
        hw->SetAttribute("Mtu", UintegerValue(1000));
        hw->SetAttribute("PathRandomVariable", PointerValue(SelectorStream()));
        return hw;
    }

    static Ptr<RdmaQueuePair> Qp(Ptr<RdmaHw> hw, uint16_t sport)
    {
        Ptr<RdmaQueuePair> qp = CreateObject<RdmaQueuePair>(
            3, Ipv4Address("11.0.1.1"), Ipv4Address("11.0.2.1"), sport, 100);
        hw->StartPathSelection(qp, 0, 0);
        return qp;
    }
};

class RepsHoldScopeTest : public TestCase
{
  public:
    RepsHoldScopeTest()
        : TestCase("Under reps_hold with SprayHoldScope leaf a spine lost on toward one leaf is "
                   "held for that leaf's destinations alone, with one sentinel per interval for "
                   "the leaf; the scope needs the hosts' leaves")
    {
    }

    void DoRun() override
    {
        Ptr<RdmaHw> unset = CreateObject<RdmaHw>();
        UintegerValue scope;
        unset->GetAttribute("SprayHoldScope", scope);
        NS_TEST_EXPECT_MSG_EQ(scope.Get(),
                              static_cast<uint32_t>(HoldScope::Auto),
                              "SprayHoldScope is auto by default");

        Ptr<RdmaHw> hw = LossHoldFixture::Hw();
        hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Leaf);
        hw->SetAttribute("Mtu", UintegerValue(1000));
        hw->SetAttribute("PathRandomVariable", PointerValue(SelectorStream()));
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsHold(), false, "refused without the hosts' leaves");
        hw->m_switchQueues = Create<SwitchQueueState>(2, LossHoldFixture::kSpines);
        hw->m_switchQueues->SetLeafOf(Ipv4Address("11.0.2.1").Get(), 0);
        hw->m_switchQueues->SetLeafOf(Ipv4Address("11.0.3.1").Get(), 0);
        hw->m_switchQueues->SetLeafOf(Ipv4Address("11.0.4.1").Get(), 1);
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsHold(), true, "accepted with them");
        // Two destinations on leaf 0, the first lossy on spine 3 over the
        // first interval, and one on leaf 1.
        std::vector<Ptr<RdmaQueuePair>> qps;
        for (const char* destination : {"11.0.2.1", "11.0.3.1", "11.0.4.1"})
        {
            qps.push_back(CreateObject<RdmaQueuePair>(
                3, Ipv4Address("11.0.1.1"), Ipv4Address(destination), 10000, 100));
            hw->StartPathSelection(qps.back(), 0, 0);
        }
        NS_TEST_EXPECT_MSG_EQ(hw->m_leafHolds.size(), 2u, "one hold per destination leaf");
        NS_TEST_EXPECT_MSG_EQ(hw->m_senderSpineScores.get(), nullptr, "and none for the host");

        uint32_t step = 0;
        std::vector<std::vector<uint32_t>> on3(3, std::vector<uint32_t>(4));
        for (uint32_t interval = 0; interval < 4; ++interval)
        {
            for (uint32_t i = 0; i < 300; ++i, ++step)
            {
                const uint32_t flow = step % 3;
                const uint64_t now = interval * kInterval + 30 * i;
                const std::set<uint8_t> lost =
                    flow == 0 && interval == 0 ? std::set<uint8_t>{3} : std::set<uint8_t>{};
                const uint8_t spine =
                    LossHoldFixture::Send(*qps[flow]->m_pathSelector, now, step / 3, lost);
                on3[flow][interval] += spine == 3;
            }
        }
        NS_TEST_EXPECT_MSG_GT(on3[0][0], 0u, "the flow to leaf 0 loses on spine 3");
        NS_TEST_EXPECT_MSG_EQ(hw->m_leafHolds.at(0).Holds(3), true, "leaf 0's hold holds it");
        NS_TEST_EXPECT_MSG_EQ(hw->m_leafHolds.at(1).Holds(3), false, "leaf 1's does not");
        for (uint32_t interval = 1; interval < 4; ++interval)
        {
            NS_TEST_EXPECT_MSG_EQ(on3[0][interval] + on3[1][interval],
                                  1u,
                                  "leaf 0's destinations send spine 3 one sentinel");
            NS_TEST_EXPECT_MSG_GT(on3[2][interval], 1u, "leaf 1's destination keeps spine 3");
        }
        Simulator::Destroy();
    }

  private:
    static constexpr uint64_t kInterval = LossHoldFixture::kInterval;
};

class RepsHoldAutoScopeTest : public TestCase
{
  public:
    RepsHoldAutoScopeTest()
        : TestCase("Under reps_hold with SprayHoldScope auto a spine lost on toward one leaf is "
                   "held for that leaf alone, widens to every destination at a loss toward a "
                   "second leaf, is sent one sentinel per interval for the host, and returns "
                   "after the clean sentinels")
    {
    }

    void DoRun() override
    {
        Ptr<RdmaHw> unset = CreateObject<RdmaHw>();
        UintegerValue leaves;
        unset->GetAttribute("SprayHoldAutoLeaves", leaves);
        NS_TEST_EXPECT_MSG_EQ(leaves.Get(), 2u, "SprayHoldAutoLeaves is 2 by default");

        Ptr<RdmaHw> hw = LossHoldFixture::Hw();
        hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Auto);
        hw->m_sprayHoldAutoLeaves = 2;
        hw->SetAttribute("Mtu", UintegerValue(1000));
        hw->SetAttribute("PathRandomVariable", PointerValue(SelectorStream()));
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsHold(), false, "refused without the hosts' leaves");
        hw->m_switchQueues = Create<SwitchQueueState>(2, LossHoldFixture::kSpines);
        hw->m_switchQueues->SetLeafOf(Ipv4Address("11.0.2.1").Get(), 0);
        hw->m_switchQueues->SetLeafOf(Ipv4Address("11.0.4.1").Get(), 1);
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsHold(), true, "accepted with them");
        std::map<uint32_t, uint32_t> intervals;
        std::map<uint32_t, uint32_t> hostHeldRows;
        hw->TraceConnectWithoutContext(
            "LeafSpineLossInterval",
            Callback<void,
                     uint32_t,
                     uint64_t,
                     const std::vector<uint32_t>&,
                     const std::vector<uint32_t>&,
                     const std::vector<uint8_t>&>(
                [&](uint32_t leaf,
                    uint64_t,
                    const std::vector<uint32_t>&,
                    const std::vector<uint32_t>&,
                    const std::vector<uint8_t>& grades) {
                    intervals[leaf]++;
                    hostHeldRows[leaf] += hw->m_autoHold->HostHolds(3) && grades[3] == 0;
                }));
        // One destination on leaf 0, lossy on spine 3 over interval 0, and
        // one on leaf 1, whose first send on spine 3 in interval 2 is lost.
        std::vector<Ptr<RdmaQueuePair>> qps;
        for (const char* destination : {"11.0.2.1", "11.0.4.1"})
        {
            qps.push_back(CreateObject<RdmaQueuePair>(
                3, Ipv4Address("11.0.1.1"), Ipv4Address(destination), 10000, 100));
            hw->StartPathSelection(qps.back(), 0, 0);
        }
        NS_TEST_EXPECT_MSG_NE(hw->m_autoHold.get(), nullptr, "one hold for the host");
        NS_TEST_EXPECT_MSG_EQ(hw->m_leafHolds.size(), 0u, "and no leaf's own");
        AutoLossHold& hold = *hw->m_autoHold;

        uint32_t step = 0;
        bool loseToLeaf1 = false;
        std::vector<std::vector<uint32_t>> on3(2, std::vector<uint32_t>(kIntervals));
        for (uint32_t interval = 0; interval < kIntervals; ++interval)
        {
            loseToLeaf1 = interval == 2;
            for (uint32_t i = 0; i < 300; ++i, ++step)
            {
                const uint32_t flow = step % 2;
                const uint64_t now = interval * kInterval + 30 * i;
                const bool lossy = flow == 0 ? interval == 0 : loseToLeaf1;
                const uint8_t spine = LossHoldFixture::Send(
                    *qps[flow]->m_pathSelector,
                    now,
                    step / 2,
                    lossy ? std::set<uint8_t>{3} : std::set<uint8_t>{});
                on3[flow][interval] += spine == 3;
                if (flow == 1 && spine == 3 && loseToLeaf1)
                {
                    loseToLeaf1 = false;
                    NS_TEST_EXPECT_MSG_EQ(hold.HostHolds(3),
                                          true,
                                          "the loss toward leaf 1 widens the hold at once");
                }
            }
            if (interval == 1)
            {
                NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 3), true, "spine 3 is held toward leaf 0");
                NS_TEST_EXPECT_MSG_EQ(hold.Holds(1, 3), false, "and not toward leaf 1");
                NS_TEST_EXPECT_MSG_EQ(hold.HostHolds(3), false, "nor for the host");
                NS_TEST_EXPECT_MSG_EQ(hold.LossLeaves(3), 1u, "one leaf has a loss on it");
            }
            if (interval == 4)
            {
                NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 3), true, "held toward leaf 0");
                NS_TEST_EXPECT_MSG_EQ(hold.Holds(1, 3), true, "and toward leaf 1");
                NS_TEST_EXPECT_MSG_EQ(hold.LossLeaves(3), 2u, "two leaves have a loss on it");
            }
        }
        NS_TEST_EXPECT_MSG_GT(on3[0][0], 0u, "the flow to leaf 0 loses on spine 3");
        NS_TEST_EXPECT_MSG_EQ(on3[0][1], 1u, "leaf 0 sends spine 3 one sentinel");
        NS_TEST_EXPECT_MSG_GT(on3[1][1], 1u, "leaf 1 keeps spine 3");
        NS_TEST_EXPECT_MSG_GT(on3[1][2], 0u, "leaf 1 sends on spine 3 and loses");
        for (uint32_t interval = 3; interval < 6; ++interval)
        {
            NS_TEST_EXPECT_MSG_EQ(on3[0][interval] + on3[1][interval],
                                  1u,
                                  "the host sends spine 3 one sentinel");
        }
        // Interval 2 saw a loss; the sentinels of 3 to 5 are the clean sends,
        // and interval 5's close releases spine 3 everywhere.
        NS_TEST_EXPECT_MSG_EQ(hold.HostHolds(3), false, "the clean sentinels release it");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 3), false, "toward leaf 0");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(1, 3), false, "and toward leaf 1");
        NS_TEST_EXPECT_MSG_EQ(hold.LossLeaves(3), 0u, "and clears its leaves");
        NS_TEST_EXPECT_MSG_GT(on3[0][6], 1u, "leaf 0 chooses spine 3 again");
        NS_TEST_EXPECT_MSG_GT(on3[1][6], 1u, "and so does leaf 1");
        NS_TEST_EXPECT_MSG_GT(intervals[0], 0u, "leaf 0's intervals are traced");
        NS_TEST_EXPECT_MSG_EQ(intervals[0], intervals[1], "as many as leaf 1's");
        NS_TEST_EXPECT_MSG_GT(hostHeldRows[1], 0u, "leaf 1's rows grade the host hold 0");
        Simulator::Destroy();
    }

  private:
    static constexpr uint64_t kInterval = LossHoldFixture::kInterval;
    static constexpr uint32_t kIntervals = 7;
};

class LossWithoutSendsTest : public TestCase
{
  public:
    LossWithoutSendsTest()
        : TestCase("A loss declared in a loss interval with no send on its spine grades the "
                   "spine 0 and holds it, in host scope and in auto scope; an interval with "
                   "neither sends nor losses keeps the top grade")
    {
    }

    void DoRun() override
    {
        HostScope();
        AutoScope();
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = LossHoldFixture::kSpines;
    static constexpr uint64_t kInterval = LossHoldFixture::kInterval;

    // Spine 3 is sent on in interval 0, which closes clean; its loss is
    // declared in interval 1, with no send on it. Spine 2 is sent on in
    // every interval and spine 5 never.
    void HostScope()
    {
        SpineScores hold(kSpines, LossHoldFixture::Hw()->LossHoldParameters());
        std::vector<std::vector<uint8_t>> closed;
        hold.SetLossIntervalCallback([&](uint64_t,
                                         const std::vector<uint32_t>&,
                                         const std::vector<uint32_t>&,
                                         const std::vector<uint8_t>& grades) {
            closed.push_back(grades);
        });
        hold.CountIntervalSend(3, 0);
        hold.CountIntervalSend(2, 10);
        hold.CountIntervalSend(2, kInterval + 10);
        NS_TEST_EXPECT_MSG_EQ(closed.size(), 1u, "interval 0 is closed");
        NS_TEST_EXPECT_MSG_EQ(+closed[0][3], +SpineReport::kTopGrade, "spine 3 closes it clean");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), false, "and is not held");
        hold.CountIntervalLoss(3, kInterval + 20);
        hold.CountIntervalSend(2, 2 * kInterval + 10);
        NS_TEST_EXPECT_MSG_EQ(closed.size(), 2u, "interval 1 is closed");
        NS_TEST_EXPECT_MSG_EQ(+closed[1][3], 0, "its loss without sends grades spine 3 0");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), true, "and holds it");
        for (const auto& grades : closed)
        {
            NS_TEST_EXPECT_MSG_EQ(+grades[5],
                                  +SpineReport::kTopGrade,
                                  "no sends and no losses keep the top grade");
            NS_TEST_EXPECT_MSG_EQ(+grades[2], +SpineReport::kTopGrade, "as do clean sends");
        }
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(5), false, "spine 5 is not held");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(2), false, "nor spine 2");
    }

    // The same toward leaf 0, with leaf 1 sending on spine 2 alone.
    void AutoScope()
    {
        AutoLossHold hold(kSpines, LossHoldFixture::Hw()->LossHoldParameters(), 2);
        hold.AddLeaf(0);
        hold.AddLeaf(1);
        std::map<uint32_t, std::vector<std::vector<uint8_t>>> closed;
        hold.SetLossIntervalCallback([&](uint32_t leaf,
                                         uint64_t,
                                         const std::vector<uint32_t>&,
                                         const std::vector<uint32_t>&,
                                         const std::vector<uint8_t>& grades) {
            closed[leaf].push_back(grades);
        });
        hold.CountSend(0, 3, 0);
        hold.CountSend(1, 2, 10);
        hold.CountSend(1, 2, kInterval + 10);
        NS_TEST_EXPECT_MSG_EQ(closed[0].size(), 1u, "interval 0 is closed");
        NS_TEST_EXPECT_MSG_EQ(+closed[0][0][3], +SpineReport::kTopGrade, "spine 3 closes it clean");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 3), false, "and is not held");
        hold.CountLoss(0, 3, kInterval + 20);
        hold.CountSend(1, 2, 2 * kInterval + 10);
        NS_TEST_EXPECT_MSG_EQ(closed[0].size(), 2u, "interval 1 is closed");
        NS_TEST_EXPECT_MSG_EQ(+closed[0][1][3], 0, "its loss without sends grades spine 3 0");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 3), true, "and holds it toward leaf 0");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(1, 3), false, "and not toward leaf 1");
        NS_TEST_EXPECT_MSG_EQ(hold.HostHolds(3), false, "nor for the host");
        for (uint32_t leaf : {0u, 1u})
        {
            for (const auto& grades : closed[leaf])
            {
                NS_TEST_EXPECT_MSG_EQ(+grades[5],
                                      +SpineReport::kTopGrade,
                                      "no sends and no losses keep the top grade");
            }
        }
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 5), false, "spine 5 is not held");
    }
};

class RepsHoldLlrTest : public TestCase
{
  public:
    RepsHoldLlrTest()
        : TestCase("Under SprayHoldRule llr a hold key's likelihood ratio holds a spine at the "
                   "loss that reaches its threshold: with SprayLossP0 zero at the first loss, "
                   "with it above zero after enough losses close together, and a clean stretch "
                   "decays the ratio to zero; the auto scope's leaves still choose the scope")
    {
    }

    void DoRun() override
    {
        Validation();
        FirstLoss();
        Threshold();
        AutoScope();
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = LossHoldFixture::kSpines;
    static constexpr uint64_t kInterval = LossHoldFixture::kInterval;

    static Ptr<RdmaHw> Hw(double p0, double p1)
    {
        Ptr<RdmaHw> hw = LossHoldFixture::Hw();
        hw->m_sprayHoldRule = static_cast<uint32_t>(HoldRule::Llr);
        hw->m_sprayLossP0 = p0;
        hw->m_sprayLossP1 = p1;
        return hw;
    }

    void Validation()
    {
        Ptr<RdmaHw> unset = CreateObject<RdmaHw>();
        UintegerValue rule;
        unset->GetAttribute("SprayHoldRule", rule);
        NS_TEST_EXPECT_MSG_EQ(rule.Get(),
                              static_cast<uint32_t>(HoldRule::Count),
                              "SprayHoldRule is count by default");
        DoubleValue value;
        unset->GetAttribute("SprayLossP0", value);
        NS_TEST_EXPECT_MSG_EQ(value.Get(), 0.0, "SprayLossP0 is 0 by default");
        unset->GetAttribute("SprayLossP1", value);
        NS_TEST_EXPECT_MSG_EQ(value.Get(), 0.001, "SprayLossP1 is 0.001 by default");
        unset->GetAttribute("SprayHoldAlpha", value);
        NS_TEST_EXPECT_MSG_EQ(value.Get(), 0.001, "SprayHoldAlpha is 0.001 by default");
        unset->GetAttribute("SprayHoldBeta", value);
        NS_TEST_EXPECT_MSG_EQ(value.Get(), 0.001, "SprayHoldBeta is 0.001 by default");
        NS_TEST_EXPECT_MSG_EQ(unset->ValidHoldRule(), true, "count needs nothing");
        NS_TEST_EXPECT_MSG_EQ(Hw(0, 0.001)->ValidHoldRule(), true, "llr under reps_hold");
        NS_TEST_EXPECT_MSG_EQ((Hw(0, 0.001)->LossHoldParameters().holdRule == HoldRule::Llr),
                              true,
                              "the hold's parameters carry the rule");
        const std::vector<std::pair<std::function<void(RdmaHw&)>, std::string>> refused = {
            {[](RdmaHw& hw) {
                 hw.m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::Reps);
             },
             "not reps_hold"},
            {[](RdmaHw& hw) { hw.m_sprayLossP0 = hw.m_sprayLossP1; }, "p0 not below p1"},
            {[](RdmaHw& hw) { hw.m_sprayLossP1 = 1; }, "p1 at 1"},
            {[](RdmaHw& hw) { hw.m_sprayHoldAlpha = 0.5; }, "alpha at 0.5"},
            {[](RdmaHw& hw) { hw.m_sprayHoldBeta = 0; }, "beta at 0"},
        };
        for (const auto& [set, message] : refused)
        {
            Ptr<RdmaHw> hw = Hw(0, 0.001);
            set(*hw);
            NS_TEST_EXPECT_MSG_EQ(hw->ValidHoldRule(), false, "refused: " + message);
        }
    }

    // With p0 zero a loss adds +infinity: the first loss holds the spine at
    // once, a send adds nothing, and the clean sentinels release it as under
    // count, restarting the test.
    void FirstLoss()
    {
        Ptr<RdmaHw> hw = Hw(0, 0.001);
        SpineScores hold(kSpines, hw->LossHoldParameters());
        for (uint32_t i = 0; i < 50; ++i)
        {
            hold.CountIntervalSend(3, 10 * i);
        }
        NS_TEST_EXPECT_MSG_EQ(hold.LossRatio(3), 0.0, "a send adds nothing");
        hold.CountIntervalSend(3, 600);
        hold.CountIntervalLoss(3, 600);
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), true, "the first loss holds at once");
        NS_TEST_EXPECT_MSG_EQ(std::isinf(hold.LossRatio(3)), true, "L is +infinity");
        // Interval 0 saw the loss; one clean sentinel, acknowledged, in each
        // of the next three releases the spine at the close of the third.
        uint8_t spine = kSpines;
        for (uint32_t interval = 1; interval <= 3; ++interval)
        {
            const uint64_t now = interval * kInterval + 1;
            NS_TEST_EXPECT_MSG_EQ(hold.TakeHoldSentinel(now, spine), true, "a sentinel is due");
            NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(spine), 3u, "on spine 3");
            hold.CountIntervalSend(spine, now);
            hold.CountIntervalAnswer(spine, now + 1);
        }
        hold.TakeHoldSentinel(4 * kInterval + 1, spine);
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), false, "three clean sentinels release it");
        NS_TEST_EXPECT_MSG_EQ(hold.LossRatio(3), 0.0, "and restart its test");
    }

    // p0 = 1e-4 and p1 = 1e-2, alpha = beta = 0.001: a send adds
    // ln(0.99 / 0.9999), a loss ln(100), and the threshold is ln(999).
    void Threshold()
    {
        const double p0 = 1e-4;
        const double p1 = 1e-2;
        Ptr<RdmaHw> hw = Hw(p0, p1);
        SpineScores hold(kSpines, hw->LossHoldParameters());
        const double send = std::log((1 - p1) / (1 - p0));
        const double loss = std::log(p1 / p0);
        const double threshold = std::log(0.999 / 0.001);
        // Ten clean sends, then a lost one: L is clamped at 0 until the loss.
        for (uint32_t i = 0; i < 11; ++i)
        {
            hold.CountIntervalSend(3, i);
        }
        hold.CountIntervalLoss(3, 11);
        double expected = loss;
        NS_TEST_EXPECT_MSG_EQ_TOL(hold.LossRatio(3), expected, 1e-9, "L after one loss");
        NS_TEST_EXPECT_MSG_LT(expected, threshold, "below the threshold");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), false, "one loss does not hold");
        // Twenty clean sends and a lost one shortly after.
        for (uint32_t i = 0; i < 21; ++i)
        {
            hold.CountIntervalSend(3, 20 + i);
        }
        expected += 21 * send;
        NS_TEST_EXPECT_MSG_EQ_TOL(hold.LossRatio(3), expected, 1e-9, "L after the sends");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), false, "still below the threshold");
        hold.CountIntervalLoss(3, 50);
        expected += loss;
        NS_TEST_EXPECT_MSG_GT_OR_EQ(expected, threshold, "the second loss reaches it");
        NS_TEST_EXPECT_MSG_EQ_TOL(hold.LossRatio(3), expected, 1e-9, "L after two losses");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), true, "and holds the spine");

        // On spine 5 one loss, then a long clean stretch: L falls by -send
        // per send and reaches 0 after ceil(loss / -send) of them.
        hold.CountIntervalSend(5, 100);
        hold.CountIntervalLoss(5, 100);
        const uint32_t decay = static_cast<uint32_t>(std::ceil(loss / -send));
        for (uint32_t i = 0; i + 1 < decay; ++i)
        {
            hold.CountIntervalSend(5, 200);
        }
        NS_TEST_EXPECT_MSG_GT(hold.LossRatio(5), 0.0, "one send short of the decay");
        hold.CountIntervalSend(5, 200);
        NS_TEST_EXPECT_MSG_EQ(hold.LossRatio(5), 0.0, "decayed to 0");
        for (uint32_t i = 0; i < 1000; ++i)
        {
            hold.CountIntervalSend(5, 300);
        }
        NS_TEST_EXPECT_MSG_EQ(hold.LossRatio(5), 0.0, "and clamped there");
        hold.CountIntervalLoss(5, 400);
        NS_TEST_EXPECT_MSG_EQ_TOL(hold.LossRatio(5), loss, 1e-9, "no credit against a loss");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(5), false, "which does not hold");
    }

    // Under auto with p0 zero the first loss toward leaf 0 holds spine 3
    // toward it alone, and a loss toward leaf 1 widens the hold.
    void AutoScope()
    {
        Ptr<RdmaHw> hw = Hw(0, 0.001);
        AutoLossHold hold(kSpines, hw->LossHoldParameters(), 2);
        hold.AddLeaf(0);
        hold.AddLeaf(1);
        hold.CountSend(0, 3, 10);
        hold.CountLoss(0, 3, 20);
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 3), true, "held toward leaf 0 at the loss");
        NS_TEST_EXPECT_MSG_EQ(hold.HostHolds(3), false, "in leaf scope");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(1, 3), false, "not toward leaf 1");
        NS_TEST_EXPECT_MSG_EQ(hold.LossRatio(1, 3), 0.0, "whose test is apart");
        hold.CountSend(1, 3, 30);
        hold.CountLoss(1, 3, 40);
        NS_TEST_EXPECT_MSG_EQ(hold.HostHolds(3), true, "a second leaf widens it");
    }
};

class RepsHoldTrickleTest : public TestCase
{
  public:
    RepsHoldTrickleTest()
        : TestCase("Under SprayHoldRelease trickle a held spine receives one in round(1 / "
                   "SprayHoldTrickle) of a flow's sends, the answers to them reach the hold "
                   "through the send records, a lost one restarts R, and the expected number of "
                   "clean acknowledgements releases the spine")
    {
    }

    void DoRun() override
    {
        Validation();
        Release();
        AutoScope();
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = LossHoldFixture::kSpines;
    static constexpr uint64_t kInterval = LossHoldFixture::kInterval;
    static constexpr uint32_t kSize = 1000;

    // Count rule with thresholds that grade a lost fraction of 0.04 0, and
    // the trickle release in place of the clean sentinels.
    static Ptr<RdmaHw> Hw()
    {
        Ptr<RdmaHw> hw = LossHoldFixture::Hw();
        hw->m_sprayLossThreshold1 = 0.01;
        hw->m_sprayLossThreshold2 = 0.02;
        hw->m_sprayLossThreshold3 = 0.04;
        hw->m_sprayLossHoldCleanSentinels = 0;
        hw->m_sprayHoldRelease = static_cast<uint32_t>(HoldRelease::Trickle);
        return hw;
    }

    // ceil(ln(1 / alpha) / -ln(1 - p1)) with p0 zero and alpha 0.001.
    static uint32_t ExpectedAcks(double p1)
    {
        return static_cast<uint32_t>(std::ceil(std::log(1000.0) / -std::log(1 - p1)));
    }

    // Twenty sends on spine 3 in interval 0, one of them declared lost: the
    // interval's lost fraction 0.05 grades it 0.
    template <typename Count, typename Lose>
    static void Onset(Count count, Lose lose)
    {
        for (uint32_t i = 0; i < 20; ++i)
        {
            count(100 + i);
        }
        lose(200);
    }

    void Validation()
    {
        Ptr<RdmaHw> unset = CreateObject<RdmaHw>();
        UintegerValue release;
        unset->GetAttribute("SprayHoldRelease", release);
        NS_TEST_EXPECT_MSG_EQ(release.Get(),
                              static_cast<uint32_t>(HoldRelease::Sentinels),
                              "SprayHoldRelease is sentinels by default");
        DoubleValue trickle;
        unset->GetAttribute("SprayHoldTrickle", trickle);
        NS_TEST_EXPECT_MSG_EQ(trickle.Get(), 0.01, "SprayHoldTrickle is 0.01 by default");
        NS_TEST_EXPECT_MSG_EQ(unset->ValidHoldRelease(), true, "sentinels need nothing");
        NS_TEST_EXPECT_MSG_EQ(Hw()->ValidHoldRelease(), true, "trickle under reps_hold");
        NS_TEST_EXPECT_MSG_EQ(Hw()->ValidRepsHold(), true, "without clean sentinels");
        NS_TEST_EXPECT_MSG_EQ(TrickleEvery(0.01), 100u, "one in 100 sends");
        const std::vector<std::pair<std::function<void(RdmaHw&)>, std::string>> refused = {
            {[](RdmaHw& hw) {
                 hw.m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::Reps);
             },
             "not reps_hold"},
            {[](RdmaHw& hw) { hw.m_sprayLossHoldCleanSentinels = 3; }, "clean sentinels set"},
            {[](RdmaHw& hw) { hw.m_sprayHeldSentinelIntervalNs = 1000; },
             "sentinel interval set"},
            {[](RdmaHw& hw) { hw.m_sprayHoldTrickle = 0; }, "no trickle"},
            {[](RdmaHw& hw) { hw.m_sprayLossP0 = hw.m_sprayLossP1; }, "p0 not below p1"},
        };
        for (const auto& [set, message] : refused)
        {
            Ptr<RdmaHw> hw = Hw();
            set(*hw);
            NS_TEST_EXPECT_MSG_EQ(hw->ValidHoldRelease(), false, "refused: " + message);
        }
        Ptr<RdmaHw> sentinels = Hw();
        sentinels->m_sprayHoldRelease = static_cast<uint32_t>(HoldRelease::Sentinels);
        NS_TEST_EXPECT_MSG_EQ(sentinels->ValidRepsHold(),
                              false,
                              "the sentinels' release needs clean sentinels");
    }

    // One flow of the host through its send records.
    void Release()
    {
        Ptr<RdmaHw> hw = Hw();
        SpineScores hold(kSpines, hw->LossHoldParameters());
        Ptr<RdmaQueuePair> qp = CreateObject<RdmaQueuePair>(
            3, Ipv4Address("11.0.1.1"), Ipv4Address("11.0.2.1"), 10000, 100);
        qp->m_outstanding.SetPacketSize(kSize);
        qp->m_pathSelector = std::make_unique<RepsHoldSelector>(SelectorStream(),
                                                                8,
                                                                1000000,
                                                                20,
                                                                kSpines,
                                                                hold);
        Onset([&](uint64_t now) { hold.CountIntervalSend(3, now); },
              [&](uint64_t now) { hold.CountIntervalLoss(3, now); });

        uint64_t now = kInterval;
        uint64_t seq = 0;
        // One send at now, answered at once: acknowledged, or declared lost.
        auto send = [&](bool lose) {
            const uint16_t identification = qp->m_pathSelector->Choose(now);
            const uint8_t spine = RequestedSpine(identification);
            const bool trickle = qp->m_pathSelector->ChoseTrickle();
            qp->m_outstanding.Add(seq, kSize, spine, now, trickle);
            if (lose)
            {
                qp->DeclareLostSentBy(now);
            }
            else
            {
                qp->m_pathSelector->OnAck(spine, false, now);
                qp->AcknowledgePacket(seq, spine);
            }
            seq += kSize;
            qp->m_outstanding.RemoveBelow(seq);
            now += 10;
            NS_TEST_EXPECT_MSG_EQ((!trickle || spine == 3), true, "a trickle send is on 3");
            return std::make_pair(spine, trickle);
        };

        // 1000 sends: spine 3 is held from the first, and is sent exactly
        // the 100th, 200th, ... of them, each a trickle send.
        uint32_t onSpine3 = 0;
        bool everyHundredth = true;
        for (uint32_t i = 1; i <= 1000; ++i)
        {
            const auto [spine, trickle] = send(false);
            if (spine == 3)
            {
                onSpine3++;
                everyHundredth &= trickle && i % 100 == 0;
            }
        }
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), true, "the interval's losses hold spine 3");
        NS_TEST_EXPECT_MSG_EQ(onSpine3, 10u, "one in 100 of the flow's sends");
        NS_TEST_EXPECT_MSG_EQ(everyHundredth, true, "every 100th, each a trickle send");
        const double p1 = 0.05;
        const double step = -std::log(1 - p1);
        NS_TEST_EXPECT_MSG_EQ_TOL(hold.ReleaseP1(3), p1, 1e-12, "p1 is the interval's 1 / 20");
        NS_TEST_EXPECT_MSG_EQ_TOL(hold.ReleaseValue(3),
                                  10 * step,
                                  1e-9,
                                  "ten clean trickle acknowledgements");

        // The next trickle send, the 1100th, is lost: R restarts.
        for (uint32_t i = 1001; i < 1100; ++i)
        {
            send(false);
        }
        const auto [lostSpine, lostTrickle] = send(true);
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(lostSpine), 3u, "the 1100th is on 3");
        NS_TEST_EXPECT_MSG_EQ(lostTrickle, true, "a trickle send");
        NS_TEST_EXPECT_MSG_EQ(hold.ReleaseValue(3), 0.0, "whose loss restarts R");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), true, "and keeps the hold");

        // ceil(ln(1000) / -ln(0.95)) = 135 clean trickle acknowledgements
        // release it.
        const uint32_t expected = ExpectedAcks(p1);
        NS_TEST_EXPECT_MSG_EQ(expected, 135u, "135 clean trickle acknowledgements");
        uint32_t acks = 0;
        while (hold.Holds(3) && acks <= expected)
        {
            if (send(false).second)
            {
                acks++;
            }
        }
        NS_TEST_EXPECT_MSG_EQ(acks, expected, "released at the expected acknowledgement");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), false, "spine 3 is released");
        NS_TEST_EXPECT_MSG_GT_OR_EQ(hold.ReleaseValue(3), std::log(1000.0), "R reached ln(1000)");
        uint32_t chosen = 0;
        for (uint32_t i = 0; i < 400; ++i)
        {
            const auto [spine, trickle] = send(false);
            chosen += spine == 3 && !trickle;
        }
        NS_TEST_EXPECT_MSG_GT(chosen, 0u, "REPS chooses spine 3 again");
    }

    // Under auto a leaf-scope hold records its interval's rate and releases
    // by the same count; a host-scope hold that ends it records the larger.
    void AutoScope()
    {
        Ptr<RdmaHw> hw = Hw();
        AutoLossHold hold(kSpines, hw->LossHoldParameters(), 2);
        hold.AddLeaf(0);
        hold.AddLeaf(1);
        NS_TEST_EXPECT_MSG_EQ(hold.TrickleEvery(), 100u, "one in 100 sends");
        Onset([&](uint64_t now) { hold.CountSend(0, 3, now); },
              [&](uint64_t now) { hold.CountLoss(0, 3, now); });
        uint8_t spine;
        NS_TEST_EXPECT_MSG_EQ(hold.TakeSentinel(0, kInterval, spine), false, "no sentinel");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 3), true, "held toward leaf 0");
        NS_TEST_EXPECT_MSG_EQ_TOL(hold.ReleaseP1(0, 3), 0.05, 1e-12, "on the interval's rate");
        const uint32_t expected = ExpectedAcks(0.05);
        for (uint32_t i = 1; i < expected; ++i)
        {
            hold.TrickleAnswer(0, 3, false, kInterval + i);
        }
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 3), true, "one acknowledgement short");
        hold.TrickleAnswer(0, 3, false, 2 * kInterval - 1);
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 3), false, "released");
        NS_TEST_EXPECT_MSG_EQ(hold.LossLeaves(3), 0u, "clearing its leaves");

        // Held again toward leaf 0 at 0.05; a loss toward leaf 1 widens it.
        Onset([&](uint64_t now) { hold.CountSend(0, 3, 2 * kInterval + now); },
              [&](uint64_t now) { hold.CountLoss(0, 3, 2 * kInterval + now); });
        hold.TakeSentinel(0, 3 * kInterval, spine);
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 3), true, "held toward leaf 0 again");
        hold.CountSend(1, 3, 3 * kInterval + 10);
        hold.CountLoss(1, 3, 3 * kInterval + 20);
        NS_TEST_EXPECT_MSG_EQ(hold.HostHolds(3), true, "widened");
        NS_TEST_EXPECT_MSG_EQ_TOL(hold.ReleaseP1(1, 3), 0.05, 1e-12, "keeping leaf 0's p1");
    }
};

class RepsHoldDuplicateTest : public TestCase
{
  public:
    RepsHoldDuplicateTest()
        : TestCase("Under SpraySentinelMode duplicate each trickle send onto a held spine is a "
                   "copy of a data packet sent on REPS's choice: losing every copy causes no "
                   "retransmission timeout and no repair, the hold still counts the copies' "
                   "losses, and the copies' clean acknowledgements release the spine")
    {
    }

    void DoRun() override
    {
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        CustomHeader::ackCarriesPacketSeq = true;
        Flow();
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint64_t kInterval = 10000;
    static constexpr uint32_t kMtu = IsolatedHost::kMtu;
    static constexpr uint32_t kPackets = 1500;
    static constexpr uint32_t kLosingSends = 400;
    static constexpr uint64_t kStepNs = 100;

    void Flow()
    {
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, kSpines);
        Ptr<RdmaHw> hw = sender.hw;
        hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::RepsHold);
        hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Host);
        hw->m_sprayLossThreshold1 = 0.1;
        hw->m_sprayLossThreshold2 = 0.25;
        hw->m_sprayLossThreshold3 = 0.5;
        hw->m_sprayLossIntervalNs = kInterval;
        hw->m_sprayHeldExploration = SpineScores::HeldExploration::Sentinel;
        hw->m_sprayLossHoldCleanSentinels = 0;
        hw->m_sprayHoldRelease = static_cast<uint32_t>(HoldRelease::Trickle);
        hw->m_spraySentinelMode = static_cast<uint32_t>(SentinelMode::Duplicate);
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsHold(), true, "a valid hold");
        NS_TEST_EXPECT_MSG_EQ((hw->LossHoldParameters().sentinelMode == SentinelMode::Duplicate),
                              true,
                              "the hold's parameters carry the mode");
        bool completed = false;
        hw->m_qpCompleteCallback =
            Callback<void, Ptr<RdmaQueuePair>>([&](Ptr<RdmaQueuePair>) { completed = true; });
        Ptr<RdmaQueuePair> qp =
            sender.AddSender(kTestSender, kTestReceiver, uint64_t{kPackets} * kMtu);
        qp->m_baseRtt = 2000;
        qp->m_notifyAppFinish = Callback<void>([]() {});
        SpineScores& hold = *hw->m_senderSpineScores;
        uint32_t intervalLosses = 0;
        hw->TraceConnectWithoutContext(
            "SpineLossInterval",
            Callback<void,
                     uint64_t,
                     const std::vector<uint32_t>&,
                     const std::vector<uint32_t>&,
                     const std::vector<uint8_t>&>([&](uint64_t endNs,
                                                      const std::vector<uint32_t>&,
                                                      const std::vector<uint32_t>& lost,
                                                      const std::vector<uint8_t>&) {
                if (endNs > kInterval)
                {
                    intervalLosses += lost[3];
                }
            }));
        // Interval 0: half of 20 sends on spine 3 lost; p1 is 0.5.
        for (uint32_t i = 0; i < 20; ++i)
        {
            hold.CountIntervalSend(3, 100 + i);
        }
        for (uint32_t i = 0; i < 10; ++i)
        {
            hold.CountIntervalLoss(3, 200 + i);
        }

        uint32_t sends = 0;
        uint32_t copies = 0;
        uint32_t copyAcks = 0;
        uint32_t liveOnHeld = 0;
        uint32_t chosenAfterRelease = 0;
        uint32_t copyAcksAtRelease = 0;
        bool copiesMatch = true;
        bool heldAfterLosses = false;
        double releaseAfterLosses = -1;
        uint32_t copiesLostAfterLosses = 0;
        std::function<void()> step = [&]() {
            if (qp->IsFinished())
            {
                return;
            }
            const size_t before = sender.emitted.size();
            Ptr<Packet> packet = hw->GetNxtPacket(qp);
            sends++;
            hw->ArmRetransmissionTimeout(qp);
            CustomHeader live(CustomHeader::L2_Header | CustomHeader::L3_Header |
                              CustomHeader::L4_Header);
            packet->PeekHeader(live);
            const bool held = hold.Holds(3);
            liveOnHeld += held && RequestedSpine(live.ipid) == 3;
            if (!held && RequestedSpine(live.ipid) == 3)
            {
                chosenAfterRelease++;
            }
            sender.ReceiveAck(kTestReceiver, live.udp.seq + kMtu, live.udp.seq, live.ipid);
            if (sender.emitted.size() > before)
            {
                const CustomHeader& copy = sender.emitted.back();
                copies++;
                copiesMatch &= copy.udp.seq == live.udp.seq && RequestedSpine(copy.ipid) == 3 &&
                               copy.l3Prot == 0x11;
                if (sends > kLosingSends)
                {
                    sender.ReceiveAck(kTestReceiver,
                                      live.udp.seq + kMtu,
                                      copy.udp.seq,
                                      copy.ipid);
                    copyAcks++;
                    if (!hold.Holds(3) && copyAcksAtRelease == 0)
                    {
                        copyAcksAtRelease = copyAcks;
                    }
                }
            }
            if (sends == kLosingSends + 50)
            {
                heldAfterLosses = hold.Holds(3);
                releaseAfterLosses = hold.ReleaseValue(3);
                copiesLostAfterLosses = qp->m_exploration_copies_lost;
            }
            Simulator::Schedule(NanoSeconds(kStepNs), step);
        };
        Simulator::Schedule(NanoSeconds(kInterval), step);
        Simulator::Run();

        NS_TEST_EXPECT_MSG_EQ(completed, true, "the flow completes");
        NS_TEST_EXPECT_MSG_EQ(sends, kPackets, "in one send per packet");
        NS_TEST_EXPECT_MSG_EQ(qp->m_retransmitted_bytes, 0u, "with no repair");
        NS_TEST_EXPECT_MSG_EQ(qp->m_timeouts, 0u, "and no retransmission timeout");
        NS_TEST_EXPECT_MSG_EQ(qp->m_recovery_events, 0u, "nor a data send declared lost");
        NS_TEST_EXPECT_MSG_EQ(liveOnHeld, 0u, "no data send goes up the held spine");
        NS_TEST_EXPECT_MSG_EQ(copiesMatch, true, "each copy is the data packet on spine 3");
        // One copy per 100 sends: 4 lost over the first 400, then 10 clean
        // ones, ceil(ln(1000) / ln(2)), release the spine.
        const uint32_t expected =
            static_cast<uint32_t>(std::ceil(std::log(1000.0) / -std::log(1 - 0.5)));
        NS_TEST_EXPECT_MSG_EQ(expected, 10u, "ten clean copy acknowledgements");
        NS_TEST_EXPECT_MSG_EQ(copies, 4 + expected, "the copies, besides the sends");
        NS_TEST_EXPECT_MSG_EQ(qp->m_exploration_copies, copies, "each with a record");
        NS_TEST_EXPECT_MSG_EQ(heldAfterLosses, true, "the lost copies keep the hold");
        NS_TEST_EXPECT_MSG_EQ(releaseAfterLosses, 0.0, "and R at 0");
        NS_TEST_EXPECT_MSG_EQ(copiesLostAfterLosses, 4u, "four copies declared lost");
        NS_TEST_EXPECT_MSG_EQ(qp->m_exploration_copies_lost, 4u, "and no other");
        NS_TEST_EXPECT_MSG_EQ(intervalLosses, 4u, "the hold's intervals count them");
        NS_TEST_EXPECT_MSG_EQ(copyAcksAtRelease, expected, "released at the tenth copy ack");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(3), false, "spine 3 is released");
        NS_TEST_EXPECT_MSG_GT(chosenAfterRelease, 0u, "and REPS chooses it again");
    }
};

class HeldSpineLeakTest : public TestCase
{
  public:
    HeldSpineLeakTest()
        : TestCase("Under reps_hold and mrc_hold with SpraySentinelMode duplicate, over many "
                   "messages from a host whose hold holds spine 3, which delivers nothing, at "
                   "host and at leaf scope, with answers that take longer than a loss interval: "
                   "a held spine's clean sends are its acknowledged sends, so the hold never "
                   "releases, no data send that is not an exploration send goes up spine 3 and "
                   "no retransmission timeout fires")
    {
    }

    void DoRun() override
    {
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        CustomHeader::ackCarriesPacketSeq = true;
        for (const bool leafScope : {false, true})
        {
            Flow(PathSelectorKind::RepsHold, leafScope);
            Flow(PathSelectorKind::MrcHold, leafScope);
        }
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
    }

  private:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint64_t kInterval = 10000;
    static constexpr uint32_t kMtu = IsolatedHost::kMtu;
    static constexpr uint32_t kMessages = 20;
    static constexpr uint32_t kPackets = 400;
    static constexpr uint64_t kStepNs = 100;
    // Longer than the loss interval: a copy's loss is declared one base RTT
    // after its data send's acknowledgement, intervals after the copy was
    // counted as a send.
    static constexpr uint64_t kRttNs = 15000;

    // Messages of kPackets, one queue pair each, 5 loss intervals apart; one
    // send per kStepNs. Every send on spine 3, data or copy, is dropped; every
    // other send is acknowledged kRttNs after it, with the receiver's
    // cumulative sequence. Spine 3 is held toward leaf 1 from the start, in
    // the host's SpineScores or, at leaf scope, in an AutoLossHold.
    void Flow(PathSelectorKind kind, bool leafScope)
    {
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, kSpines);
        Ptr<RdmaHw> hw = sender.hw;
        hw->m_pathSelectorKind = static_cast<uint32_t>(kind);
        hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Host);
        hw->m_sprayLossThreshold1 = 0.1;
        hw->m_sprayLossThreshold2 = 0.25;
        hw->m_sprayLossThreshold3 = 0.5;
        hw->m_sprayLossIntervalNs = kInterval;
        hw->m_sprayHeldExploration = SpineScores::HeldExploration::Sentinel;
        hw->m_sprayHeldSentinelIntervalNs = kInterval / 4;
        hw->m_sprayLossHoldCleanSentinels = 3;
        hw->m_spraySentinelMode = static_cast<uint32_t>(SentinelMode::Duplicate);
        hw->m_retransmission_timeout_ns = 10 * kRttNs;
        hw->m_max_retransmission_retries = 100;
        const std::string which = std::string(kind == PathSelectorKind::RepsHold
                                                  ? "reps_hold"
                                                  : "mrc_hold") +
                                  (leafScope ? " leaf: " : " host: ");
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsHold(), true, which + "a valid hold");
        uint32_t completed = 0;
        hw->m_qpCompleteCallback =
            Callback<void, Ptr<RdmaQueuePair>>([&](Ptr<RdmaQueuePair>) { completed++; });
        AutoLossHold autoHold(kSpines, hw->LossHoldParameters(), 2);
        autoHold.AddLeaf(1);
        SpineScores& hostHold = hw->HostScores(hw->LossHoldParameters());
        auto holds = [&]() { return leafScope ? autoHold.Holds(1, 3) : hostHold.Holds(3); };
        for (uint32_t i = 0; i < 20; ++i)
        {
            if (leafScope)
            {
                autoHold.CountSend(1, 3, 100 + i);
                autoHold.CountLoss(1, 3, 200 + i);
            }
            else
            {
                hostHold.CountIntervalSend(3, 100 + i);
                hostHold.CountIntervalLoss(3, 200 + i);
            }
        }
        uint32_t timeouts = 0;
        hw->TraceConnectWithoutContext(
            "RetransmissionTimeout",
            Callback<void, uint64_t, Ptr<RdmaQueuePair>, int32_t, bool, uint64_t>(
                [&](uint64_t, Ptr<RdmaQueuePair>, int32_t, bool, uint64_t) { timeouts++; }));
        uint32_t sends = 0;
        uint32_t heldAtSend = 0;
        uint32_t dataOnSpine3 = 0;
        Ptr<RdmaQueuePair> qp;
        uint64_t received = 0;
        std::set<uint64_t> arrived;
        std::function<void()> start;
        auto answer = [&](Ptr<RdmaQueuePair> to, uint32_t seq, uint16_t identification) {
            if (to != qp || to->IsFinished())
            {
                return;
            }
            arrived.insert(seq);
            while (arrived.count(received) > 0)
            {
                received += kMtu;
            }
            sender.ReceiveAck(kTestReceiver, received, seq, identification);
        };
        std::function<void()> step = [&]() {
            if (qp->IsFinished())
            {
                if (completed < kMessages)
                {
                    Simulator::Schedule(NanoSeconds(5 * kInterval), start);
                }
                return;
            }
            if (qp->snd_nxt >= qp->m_size && qp->RepairBytesLeft() == 0)
            {
                Simulator::Schedule(NanoSeconds(kStepNs), step);
                return;
            }
            const size_t before = sender.emitted.size();
            Ptr<Packet> packet = hw->GetNxtPacket(qp);
            hw->ArmRetransmissionTimeout(qp);
            CustomHeader live(CustomHeader::L2_Header | CustomHeader::L3_Header |
                              CustomHeader::L4_Header);
            packet->PeekHeader(live);
            const bool held = holds();
            sends++;
            heldAtSend += held;
            Ptr<RdmaQueuePair> to = qp;
            if (RequestedSpine(live.ipid) == 3)
            {
                dataOnSpine3 += !qp->m_pathSelector->ChoseExploration();
            }
            else
            {
                const uint32_t seq = live.udp.seq;
                const uint16_t ipid = live.ipid;
                Simulator::Schedule(NanoSeconds(kRttNs),
                                    [&, to, seq, ipid]() { answer(to, seq, ipid); });
            }
            if (sender.emitted.size() > before &&
                RequestedSpine(sender.emitted.back().ipid) != 3)
            {
                const uint32_t seq = sender.emitted.back().udp.seq;
                const uint16_t ipid = sender.emitted.back().ipid;
                Simulator::Schedule(NanoSeconds(kRttNs),
                                    [&, to, seq, ipid]() { answer(to, seq, ipid); });
            }
            Simulator::Schedule(NanoSeconds(kStepNs), step);
        };
        start = [&]() {
            received = 0;
            arrived.clear();
            qp = sender.AddSender(kTestSender, kTestReceiver, uint64_t{kPackets} * kMtu);
            qp->m_baseRtt = kRttNs;
            qp->m_notifyAppFinish = Callback<void>([]() {});
            if (leafScope && kind == PathSelectorKind::RepsHold)
            {
                qp->m_pathSelector = std::make_unique<RepsHoldSelector>(
                    SelectorStream(), 8, 1000000, 20, kSpines, autoHold, 1);
            }
            else if (leafScope)
            {
                qp->m_pathSelector = std::make_unique<MrcHoldSelector>(
                    SelectorStream(), kSpines, kRttNs, 10 * kRttNs, autoHold, 1);
            }
            step();
        };
        Simulator::Schedule(NanoSeconds(kInterval), start);
        Simulator::Stop(Seconds(1));
        Simulator::Run();
        NS_TEST_EXPECT_MSG_EQ(completed, kMessages, which + "every message completes");
        NS_TEST_EXPECT_MSG_EQ(heldAtSend, sends, which + "spine 3 is held at every send");
        NS_TEST_EXPECT_MSG_EQ(dataOnSpine3, 0u, which + "no data send but an exploration send on 3");
        NS_TEST_EXPECT_MSG_EQ(timeouts, 0u, which + "and no retransmission timeout fires");
        Simulator::Destroy();
    }
};

class HeldSpineFastLossTest : public TestCase
{
  public:
    HeldSpineFastLossTest()
        : TestCase("Under reps_hold and mrc_hold a data send up a spine held as it is chosen, "
                   "here the sentinel under SpraySentinelMode data, still unacknowledged "
                   "SprayHeldLossRtts base RTTs after it is declared lost and repaired, with "
                   "no retransmission timeout; with SprayHeldLossRtts 0 the timeout declares it "
                   "lost")
    {
    }

    void DoRun() override
    {
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        CustomHeader::ackCarriesPacketSeq = true;
        Ptr<RdmaHw> unset = CreateObject<RdmaHw>();
        DoubleValue rtts;
        unset->GetAttribute("SprayHeldLossRtts", rtts);
        NS_TEST_EXPECT_MSG_EQ(rtts.Get(), 2.0, "SprayHeldLossRtts is 2 by default");
        Flow(PathSelectorKind::RepsHold, 2);
        Flow(PathSelectorKind::MrcHold, 2);
        Flow(PathSelectorKind::RepsHold, 0);
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
    }

  private:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint64_t kInterval = 10000;
    static constexpr uint32_t kMtu = IsolatedHost::kMtu;
    static constexpr uint32_t kPackets = 300;
    static constexpr uint64_t kStepNs = 100;
    static constexpr uint64_t kRttNs = 2000;
    static constexpr uint64_t kTimeoutNs = 50 * kRttNs;

    // One message, one send per kStepNs, spine 3 held from the start with a
    // sentinel per loss interval. Every send on spine 3 is dropped, every
    // other one acknowledged kRttNs after it.
    void Flow(PathSelectorKind kind, double heldLossRtts)
    {
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, kSpines);
        Ptr<RdmaHw> hw = sender.hw;
        hw->m_pathSelectorKind = static_cast<uint32_t>(kind);
        hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Host);
        hw->m_sprayLossThreshold1 = 0.1;
        hw->m_sprayLossThreshold2 = 0.25;
        hw->m_sprayLossThreshold3 = 0.5;
        hw->m_sprayLossIntervalNs = kInterval;
        hw->m_sprayHeldExploration = SpineScores::HeldExploration::Sentinel;
        hw->m_sprayLossHoldCleanSentinels = 3;
        hw->m_sprayHeldLossRtts = heldLossRtts;
        hw->m_retransmission_timeout_ns = kTimeoutNs;
        const std::string which = std::string(kind == PathSelectorKind::RepsHold
                                                  ? "reps_hold"
                                                  : "mrc_hold") +
                                  (heldLossRtts > 0 ? ": " : " without: ");
        bool completed = false;
        hw->m_qpCompleteCallback =
            Callback<void, Ptr<RdmaQueuePair>>([&](Ptr<RdmaQueuePair>) { completed = true; });
        uint32_t failure = 0;
        hw->m_qpFailureCallback = Callback<void, Ptr<RdmaQueuePair>, uint32_t>(
            [&](Ptr<RdmaQueuePair>, uint32_t reason) { failure = reason + 1; });
        std::vector<bool> timeoutExploration;
        hw->TraceConnectWithoutContext(
            "RetransmissionTimeout",
            Callback<void, uint64_t, Ptr<RdmaQueuePair>, int32_t, bool, uint64_t>(
                [&](uint64_t, Ptr<RdmaQueuePair>, int32_t, bool exploration, uint64_t) {
                    timeoutExploration.push_back(exploration);
                }));
        Ptr<RdmaQueuePair> qp =
            sender.AddSender(kTestSender, kTestReceiver, uint64_t{kPackets} * kMtu);
        qp->m_baseRtt = kRttNs;
        qp->m_notifyAppFinish = Callback<void>([]() {});
        SpineScores& hold = *hw->m_senderSpineScores;
        hold.CountIntervalSend(3, 100);
        hold.CountIntervalLoss(3, 200);
        uint64_t received = 0;
        std::set<uint64_t> arrived;
        // Per sequence, when it was last sent up the held spine 3; and the
        // delays from such a send to the repair of its sequence.
        std::map<uint32_t, uint64_t> onHeld;
        std::vector<uint64_t> repairDelays;
        uint32_t heldSends = 0;
        std::function<void()> step = [&]() {
            if (qp->IsFinished())
            {
                return;
            }
            if (qp->snd_nxt >= qp->m_size && qp->RepairBytesLeft() == 0)
            {
                Simulator::Schedule(NanoSeconds(kStepNs), step);
                return;
            }
            const uint64_t now = Simulator::Now().GetNanoSeconds();
            Ptr<Packet> packet = hw->GetNxtPacket(qp);
            hw->ArmRetransmissionTimeout(qp);
            CustomHeader live(CustomHeader::L2_Header | CustomHeader::L3_Header |
                              CustomHeader::L4_Header);
            packet->PeekHeader(live);
            const uint32_t seq = live.udp.seq;
            const auto earlier = onHeld.find(seq);
            if (earlier != onHeld.end())
            {
                repairDelays.push_back(now - earlier->second);
                onHeld.erase(earlier);
            }
            if (RequestedSpine(live.ipid) == 3)
            {
                NS_TEST_EXPECT_MSG_EQ(qp->m_pathSelector->ChoseHeld(), true,
                                      which + "a send up spine 3 is one up a held spine");
                heldSends++;
                onHeld[seq] = now;
            }
            else
            {
                const uint16_t ipid = live.ipid;
                Simulator::Schedule(NanoSeconds(kRttNs), [&, seq, ipid]() {
                    if (qp->IsFinished())
                    {
                        return;
                    }
                    arrived.insert(seq);
                    while (arrived.count(received) > 0)
                    {
                        received += kMtu;
                    }
                    sender.ReceiveAck(kTestReceiver, received, seq, ipid);
                });
            }
            Simulator::Schedule(NanoSeconds(kStepNs), step);
        };
        Simulator::Schedule(NanoSeconds(kInterval), step);
        // Without the early loss, up to the first timeout only: once nothing
        // but repairs is left to send, each repair a loss interval after the
        // last sentinel is the next sentinel.
        Simulator::Stop(heldLossRtts > 0 ? Seconds(1)
                                         : NanoSeconds(kInterval + kPackets * kStepNs +
                                                       kTimeoutNs));
        Simulator::Run();

        NS_TEST_EXPECT_MSG_GT(heldSends, 1u, which + "sentinels go up spine 3");
        if (heldLossRtts > 0)
        {
            NS_TEST_EXPECT_MSG_EQ(failure, 0u, which + "the queue pair does not fail");
            NS_TEST_EXPECT_MSG_EQ(completed, true, which + "the message completes");
            NS_TEST_EXPECT_MSG_EQ(repairDelays.size(), heldSends, which + "each one is repaired");
            NS_TEST_EXPECT_MSG_EQ(qp->m_held_declared_lost, heldSends,
                                  which + "each declared lost as a send on a held spine");
            NS_TEST_EXPECT_MSG_EQ(timeoutExploration.size(), 0u,
                                  which + "and no retransmission timeout fires");
            for (const uint64_t delay : repairDelays)
            {
                NS_TEST_EXPECT_MSG_GT_OR_EQ(delay, 2 * kRttNs, which + "after two base RTTs");
                NS_TEST_EXPECT_MSG_LT_OR_EQ(delay, 2 * kRttNs + kStepNs,
                                            which + "at the next send after them");
            }
        }
        else
        {
            NS_TEST_EXPECT_MSG_EQ(qp->m_held_declared_lost, 0u, which + "none declared early");
            NS_TEST_EXPECT_MSG_GT(timeoutExploration.size(), 0u, which + "the timeout fires");
            NS_TEST_EXPECT_MSG_EQ(timeoutExploration.empty() || timeoutExploration[0],
                                  true,
                                  which + "for a sentinel");
        }
        Simulator::Destroy();
    }
};

class SprayAckLossTest : public TestCase
{
  public:
    enum class Answer
    {
        Dropped,
        Late,
        Off,
    };

    explicit SprayAckLossTest(Answer answer)
        : TestCase(Name(answer)),
          m_answer(answer)
    {
    }

    void DoRun() override
    {
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        CustomHeader::ackCarriesPacketSeq = true;
        Flow();
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
    }

  private:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint64_t kInterval = 10000;
    static constexpr uint32_t kMtu = IsolatedHost::kMtu;
    static constexpr uint32_t kPackets = 100;
    static constexpr uint64_t kStepNs = 100;
    static constexpr uint64_t kRttNs = 2000;

    static std::string Name(Answer answer)
    {
        switch (answer)
        {
        case Answer::Dropped:
            return "Under SprayAckLossRtts 2 a send whose acknowledgement is lost and that the "
                   "cumulative acknowledgement covers is declared lost on its spine two base "
                   "RTTs after, for the selector alone, with no repair";
        case Answer::Late:
            return "Under SprayAckLossRtts 2 a covered send whose own acknowledgement arrives "
                   "before two base RTTs is not declared lost";
        default:
            return "With SprayAckLossRtts 0, the default, a covered send whose acknowledgement "
                   "is lost is not declared lost";
        }
    }

    // One message under reps_hold, one send per kStepNs, every send
    // acknowledged kRttNs after it but one: its data arrives, and its
    // acknowledgement is dropped, or under Late delivered 1.5 base RTTs late,
    // while the cumulative acknowledgement of the next send, up another
    // spine, covers it.
    void Flow()
    {
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, kSpines);
        Ptr<RdmaHw> hw = sender.hw;
        DoubleValue rtts;
        hw->GetAttribute("SprayAckLossRtts", rtts);
        NS_TEST_EXPECT_MSG_EQ(rtts.Get(), 0.0, "SprayAckLossRtts is 0 by default");
        hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::RepsHold);
        hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Host);
        hw->m_sprayLossThreshold1 = 0.1;
        hw->m_sprayLossThreshold2 = 0.25;
        hw->m_sprayLossThreshold3 = 0.5;
        hw->m_sprayLossIntervalNs = kInterval;
        hw->m_sprayHeldExploration = SpineScores::HeldExploration::Sentinel;
        hw->m_sprayLossHoldCleanSentinels = 3;
        hw->m_retransmission_timeout_ns = 50 * kRttNs;
        if (m_answer != Answer::Off)
        {
            hw->m_sprayAckLossRtts = 2;
        }
        NS_TEST_EXPECT_MSG_EQ(hw->ValidAckLoss(), true, "accepted under spray_uniform");
        bool completed = false;
        hw->m_qpCompleteCallback =
            Callback<void, Ptr<RdmaQueuePair>>([&](Ptr<RdmaQueuePair>) { completed = true; });
        uint32_t events = 0;
        uint64_t declaredNs = 0;
        hw->m_transportEventCallback =
            Callback<void, const char*, uint64_t>([&](const char* event, uint64_t bytes) {
                if (std::string(event) == "ack_loss_declared")
                {
                    events++;
                    declaredNs = Simulator::Now().GetNanoSeconds();
                    NS_TEST_EXPECT_MSG_EQ(bytes, 0u, "the event carries no bytes");
                }
            });
        std::vector<uint32_t> lost(kSpines, 0);
        hw->TraceConnectWithoutContext(
            "SpineLossInterval",
            Callback<void,
                     uint64_t,
                     const std::vector<uint32_t>&,
                     const std::vector<uint32_t>&,
                     const std::vector<uint8_t>&>([&](uint64_t,
                                                      const std::vector<uint32_t>&,
                                                      const std::vector<uint32_t>& counted,
                                                      const std::vector<uint8_t>&) {
                for (uint32_t k = 0; k < kSpines; ++k)
                {
                    lost[k] += counted[k];
                }
            }));
        Ptr<RdmaQueuePair> qp =
            sender.AddSender(kTestSender, kTestReceiver, uint64_t{kPackets} * kMtu);
        qp->m_baseRtt = kRttNs;
        qp->m_notifyAppFinish = Callback<void>([]() {});
        uint64_t received = 0;
        std::set<uint64_t> arrived;
        std::map<uint32_t, uint32_t> sends;
        std::vector<uint16_t> ipids;
        std::vector<uint64_t> sentNs;
        // The first send whose next send goes up another spine, so that no
        // later send up its spine is answered before the cumulative
        // acknowledgement covers it.
        uint32_t dropped = kPackets;
        uint64_t coveredNs = 0;
        uint32_t coveredAtCover = 0;
        auto acknowledge = [&](uint32_t packet) {
            if (qp->IsFinished())
            {
                return;
            }
            const uint32_t seq = packet * kMtu;
            arrived.insert(seq);
            while (arrived.count(received) > 0)
            {
                received += kMtu;
            }
            if (packet == dropped)
            {
                return;
            }
            sender.ReceiveAck(kTestReceiver, received, seq, ipids[packet]);
            if (packet == dropped + 1)
            {
                coveredNs = Simulator::Now().GetNanoSeconds();
                coveredAtCover = qp->m_covered.size();
            }
        };
        std::function<void()> step = [&]() {
            if (qp->IsFinished() || qp->snd_nxt >= qp->m_size)
            {
                return;
            }
            Ptr<Packet> packet = hw->GetNxtPacket(qp);
            hw->ArmRetransmissionTimeout(qp);
            CustomHeader live(CustomHeader::L2_Header | CustomHeader::L3_Header |
                              CustomHeader::L4_Header);
            packet->PeekHeader(live);
            const uint32_t seq = live.udp.seq;
            sends[seq]++;
            const uint32_t index = seq / kMtu;
            ipids.push_back(live.ipid);
            sentNs.push_back(Simulator::Now().GetNanoSeconds());
            if (dropped == kPackets && index > 0 &&
                RequestedSpine(ipids[index - 1]) != RequestedSpine(live.ipid))
            {
                dropped = index - 1;
                if (m_answer == Answer::Late)
                {
                    const uint64_t lateNs = sentNs[dropped] + kRttNs + 3 * kRttNs / 2;
                    Simulator::Schedule(
                        NanoSeconds(lateNs - Simulator::Now().GetNanoSeconds()), [&]() {
                            sender.ReceiveAck(kTestReceiver, received, dropped * kMtu,
                                              ipids[dropped]);
                        });
                }
            }
            Simulator::Schedule(NanoSeconds(kRttNs), [&, index]() { acknowledge(index); });
            Simulator::Schedule(NanoSeconds(kStepNs), step);
        };
        Simulator::Schedule(NanoSeconds(kInterval), step);
        Simulator::Stop(Seconds(1));
        Simulator::Run();
        // The loss intervals still open are closed by a later count.
        hw->m_senderSpineScores->CountIntervalSend(0, Seconds(2).GetNanoSeconds());

        const std::string which = m_answer == Answer::Dropped ? "dropped: "
                                  : m_answer == Answer::Late  ? "late: "
                                                              : "off: ";
        NS_TEST_EXPECT_MSG_EQ(completed, true, which + "the message completes");
        NS_TEST_ASSERT_MSG_LT(dropped, kPackets, which + "a send's answer is dropped");
        const uint8_t first = RequestedSpine(ipids[dropped]);
        NS_TEST_EXPECT_MSG_EQ(coveredNs, sentNs[dropped] + kStepNs + kRttNs,
                              which + "the next send's acknowledgement covers it");
        const uint32_t kept = m_answer == Answer::Off ? 0 : 1;
        NS_TEST_EXPECT_MSG_EQ(coveredAtCover, kept,
                              which + "and keeps it as covered only when the key is set");
        uint32_t lostSends = 0;
        for (uint32_t k = 0; k < kSpines; ++k)
        {
            lostSends += lost[k];
        }
        const uint32_t declared = m_answer == Answer::Dropped ? 1 : 0;
        NS_TEST_EXPECT_MSG_EQ(qp->m_ack_loss_declared, declared, which + "the counter");
        NS_TEST_EXPECT_MSG_EQ(events, declared, which + "the transport event");
        if (declared > 0)
        {
            NS_TEST_EXPECT_MSG_EQ(declaredNs, coveredNs + 2 * kRttNs,
                                  which + "two base RTTs after it was covered");
        }
        NS_TEST_EXPECT_MSG_EQ(lostSends, declared, which + "the losses the hold counts");
        NS_TEST_EXPECT_MSG_EQ(lost[first], declared, which + "on that send's spine");
        NS_TEST_EXPECT_MSG_EQ(sends[dropped * kMtu], 1u, which + "that send is not retransmitted");
        NS_TEST_EXPECT_MSG_EQ(sends.size(), kPackets, which + "every packet is sent");
        NS_TEST_EXPECT_MSG_EQ(qp->m_recovery_events, 0u, which + "no send is declared lost");
        NS_TEST_EXPECT_MSG_EQ(qp->m_repair_ranges.empty(), true, which + "no repair is queued");
        NS_TEST_EXPECT_MSG_EQ(qp->m_timeouts, 0u, which + "no retransmission timeout");
        NS_TEST_EXPECT_MSG_EQ(qp->m_covered.empty(), true, which + "nothing is left covered");
        Simulator::Destroy();
    }

    Answer m_answer;
};

class SprayAckLossValidTest : public TestCase
{
  public:
    SprayAckLossValidTest()
        : TestCase("SprayAckLossRtts is refused under every LoadBalancing but spray_uniform")
    {
    }

    void DoRun() override
    {
        for (const LoadBalancingMode mode : {LoadBalancingMode::Ecmp,
                                             LoadBalancingMode::EntropyHash,
                                             LoadBalancingMode::SprayPolicy})
        {
            Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
            hw->m_loadBalancing = static_cast<uint32_t>(mode);
            NS_TEST_EXPECT_MSG_EQ(hw->ValidAckLoss(), true, "unset, accepted under any mode");
            hw->SetAttribute("SprayAckLossRtts", DoubleValue(2.0));
            NS_TEST_EXPECT_MSG_EQ(hw->ValidAckLoss(), false, "set, refused");
        }
        Ptr<RdmaHw> uniform = CreateObject<RdmaHw>();
        uniform->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
        uniform->SetAttribute("SprayAckLossRtts", DoubleValue(2.0));
        NS_TEST_EXPECT_MSG_EQ(uniform->ValidAckLoss(), true, "accepted under spray_uniform");
    }
};

class RetransmissionTimeoutTraceTest : public TestCase
{
  public:
    RetransmissionTimeoutTraceTest()
        : TestCase("A retransmission timeout taken is traced once, with the requested spine of "
                   "the oldest outstanding send and whether it was an exploration send")
    {
    }

    void DoRun() override
    {
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        CustomHeader::ackCarriesPacketSeq = true;
        Timeout(true);
        Timeout(false);
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
    }

  private:
    struct Row
    {
        uint64_t nowNs;
        int32_t spine;
        bool exploration;
        uint64_t bytes;
    };

    // Two sends at 10 us, unanswered; with heldSpine3 spine 3 is held, so
    // the first of them is its sentinel.
    void Timeout(bool heldSpine3)
    {
        constexpr uint64_t kStart = 10000;
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, 8);
        Ptr<RdmaHw> hw = sender.hw;
        hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::RepsHold);
        hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Host);
        hw->m_sprayLossThreshold1 = 0.1;
        hw->m_sprayLossThreshold2 = 0.25;
        hw->m_sprayLossThreshold3 = 0.5;
        hw->m_sprayLossIntervalNs = kStart;
        hw->m_sprayLossHoldCleanSentinels = 3;
        hw->m_sprayHeldExploration = SpineScores::HeldExploration::Sentinel;
        std::vector<Row> rows;
        hw->TraceConnectWithoutContext(
            "RetransmissionTimeout",
            Callback<void, uint64_t, Ptr<RdmaQueuePair>, int32_t, bool, uint64_t>(
                [&](uint64_t nowNs, Ptr<RdmaQueuePair>, int32_t spine, bool exploration,
                    uint64_t bytes) { rows.push_back(Row{nowNs, spine, exploration, bytes}); }));
        Ptr<RdmaQueuePair> qp =
            sender.AddSender(kTestSender, kTestReceiver, 10 * IsolatedHost::kMtu);
        SpineScores& hold = *hw->m_senderSpineScores;
        if (heldSpine3)
        {
            hold.CountIntervalSend(3, 0);
            hold.CountIntervalLoss(3, 0);
        }
        uint8_t first = 0;
        Simulator::Schedule(NanoSeconds(kStart), [&]() {
            CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header);
            hw->GetNxtPacket(qp)->PeekHeader(parsed);
            first = RequestedSpine(parsed.ipid);
            hw->GetNxtPacket(qp);
            hw->ArmRetransmissionTimeout(qp);
        });
        Simulator::Stop(NanoSeconds(kStart + IsolatedHost::kTimeoutNs + 1));
        Simulator::Run();
        const std::string which = heldSpine3 ? "sentinel: " : "data: ";
        NS_TEST_EXPECT_MSG_EQ(rows.size(), 1u, which + "one timeout, one row");
        if (!rows.empty())
        {
            NS_TEST_EXPECT_MSG_EQ(rows[0].nowNs, kStart + IsolatedHost::kTimeoutNs,
                                  which + "at the oldest send's timeout");
            NS_TEST_EXPECT_MSG_EQ(rows[0].spine, static_cast<int32_t>(first),
                                  which + "the oldest send's spine");
            NS_TEST_EXPECT_MSG_EQ(rows[0].exploration, heldSpine3, which + "the sentinel flag");
            NS_TEST_EXPECT_MSG_EQ(rows[0].bytes, 2u * IsolatedHost::kMtu,
                                  which + "two packets outstanding");
        }
        if (heldSpine3)
        {
            NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(first), 3u, "the sentinel is on spine 3");
        }
        NS_TEST_EXPECT_MSG_EQ(qp->m_timeouts, 1u, which + "the queue pair counts it");
        Simulator::Destroy();
    }
};


class SprayPathSeqTest : public TestCase
{
  public:
    SprayPathSeqTest()
        : TestCase("Under SprayPathSeq every data packet carries its path sequence and the "
                   "receiver counts the packets missing per carrying spine")
    {
    }

    void DoRun() override
    {
        const IntHeader::Mode savedIntMode = IntHeader::mode;
        const bool savedPathSeq = CustomHeader::udpCarriesPathSeq;
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        IntHeader::mode = IntHeader::NONE;
        CustomHeader::ackCarriesPacketSeq = true;

        CustomHeader::udpCarriesPathSeq = false;
        const uint32_t without = CustomHeader::GetUdpHeaderSize();
        NS_TEST_EXPECT_MSG_EQ(SeqTsHeader().GetSerializedSize(),
                              6u,
                              "without the path sequence the data header keeps its size");
        CustomHeader::udpCarriesPathSeq = true;
        NS_TEST_EXPECT_MSG_EQ(CustomHeader::GetUdpHeaderSize(),
                              without + 2,
                              "the path sequence adds two bytes");
        NS_TEST_EXPECT_MSG_EQ(SeqTsHeader().GetSerializedSize(),
                              8u,
                              "the writer agrees on the size");
        Unit();
        Transport();

        CustomHeader::udpCarriesPathSeq = savedPathSeq;
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        IntHeader::mode = savedIntMode;
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint8_t kLossy = 3;

    void Unit()
    {
        SpineSequenceSender sender;
        NS_TEST_EXPECT_MSG_EQ(sender.Next(2), 0u, "a spine's first packet carries 0");
        NS_TEST_EXPECT_MSG_EQ(sender.Next(2), 1u, "and the next 1");
        NS_TEST_EXPECT_MSG_EQ(sender.Next(5), 0u, "each spine counts apart");

        SpineSequenceReceiver receiver;
        auto arrive = [&](uint8_t requested, uint8_t carrying, uint16_t seq) {
            return receiver.OnArrival(requested, carrying, seq);
        };
        NS_TEST_EXPECT_MSG_EQ(arrive(1, 1, 0).gaps, 0u, "in order");
        NS_TEST_EXPECT_MSG_EQ(arrive(1, 1, 3).gaps, 2u, "1 and 2 are missing");
        const SpineSequenceReceiver::Arrival late = arrive(1, 1, 2);
        NS_TEST_EXPECT_MSG_EQ(late.late, true, "2 arrives late");
        NS_TEST_EXPECT_MSG_EQ(late.gaps, 0u, "and counts nothing");
        NS_TEST_EXPECT_MSG_EQ(arrive(1, 1, 4).gaps, 0u, "4 follows 3");
        NS_TEST_EXPECT_MSG_EQ(arrive(4, 4, 1).gaps, 1u, "a spine's first packet is expected");
        // A swap of two packets on one spine shows one missing, then one late.
        NS_TEST_EXPECT_MSG_EQ(arrive(6, 6, 1).gaps, 1u, "the second of a swap shows a gap");
        NS_TEST_EXPECT_MSG_EQ(arrive(6, 6, 0).late, true, "the first is late");
        // Moved from spine 2 to spine 7 by a source leaf: tracked apart from
        // spine 7's own packets, from its first arrival.
        NS_TEST_EXPECT_MSG_EQ(arrive(7, 7, 0).gaps, 0u, "spine 7's own packet");
        NS_TEST_EXPECT_MSG_EQ(arrive(2, 7, 40).gaps, 0u, "a moved path starts at its first arrival");
        NS_TEST_EXPECT_MSG_EQ(arrive(2, 7, 41).gaps, 0u, "and follows on");
        NS_TEST_EXPECT_MSG_EQ(arrive(7, 7, 1).gaps, 0u, "spine 7's own count is not disturbed");
        // The counter wraps at 2^16.
        for (uint32_t seq = 2; seq <= 0xFFFF; ++seq)
        {
            arrive(7, 7, static_cast<uint16_t>(seq));
        }
        NS_TEST_EXPECT_MSG_EQ(arrive(7, 7, 0).gaps, 0u, "the sequence wraps");
        NS_TEST_EXPECT_MSG_EQ(arrive(7, 7, 2).gaps, 1u, "and counts across the wrap");
    }

    // A queue pair under reps_hold sends through a fabric that drops 1% of
    // the packets up spine 3 and delays every packet up spine 1 behind the
    // next few, to a receiver that counts the gaps.
    void Transport()
    {
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, kSpines);
        IsolatedHost receiver(LoadBalancingMode::SprayUniform, kTestSender, kSpines);
        for (IsolatedHost* host : {&sender, &receiver})
        {
            Ptr<RdmaHw> hw = host->hw;
            hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::RepsHold);
            hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Host);
            hw->m_sprayLossThreshold1 = 0.1;
            hw->m_sprayLossThreshold2 = 0.25;
            hw->m_sprayLossThreshold3 = 0.5;
            hw->m_sprayLossIntervalNs = 10000;
            hw->m_sprayLossHoldCleanSentinels = 3;
            hw->m_sprayHeldExploration = SpineScores::HeldExploration::Sentinel;
            hw->m_sprayPathSeq = true;
            NS_TEST_EXPECT_MSG_EQ(hw->ValidPathSeq(), true, "accepted under reps_hold");
        }
        Ptr<RdmaHw> off = CreateObject<RdmaHw>();
        NS_TEST_EXPECT_MSG_EQ(off->m_sprayPathSeq, false, "SprayPathSeq is off by default");
        off->m_sprayPathSeq = true;
        NS_TEST_EXPECT_MSG_EQ(off->ValidPathSeq(), false, "refused outside spray_uniform");
        Ptr<RdmaHw> uniform = CreateObject<RdmaHw>();
        uniform->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
        uniform->m_sprayPathSeq = true;
        NS_TEST_EXPECT_MSG_EQ(uniform->ValidPathSeq(),
                              true,
                              "accepted under spray_uniform with any selector");
        CustomHeader::udpCarriesPathSeq = false;
        NS_TEST_EXPECT_MSG_EQ(sender.hw->ValidPathSeq(),
                              false,
                              "refused unless the data header carries it");
        CustomHeader::udpCarriesPathSeq = true;

        constexpr uint32_t kPackets = 20000;
        Ptr<RdmaQueuePair> qp = sender.AddSender(kTestSender, kTestReceiver,
                                                 static_cast<uint64_t>(kPackets) * IsolatedHost::kMtu);
        Ptr<UniformRandomVariable> loss =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(7));
        std::vector<uint32_t> sent(kSpines, 0);
        std::vector<uint16_t> lastSeq(kSpines, 0);
        uint32_t dropped = 0;
        std::deque<Ptr<Packet>> delayed;
        auto deliver = [&](Ptr<Packet> packet) {
            CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                                CustomHeader::L4_Header);
            packet->PeekHeader(parsed);
            receiver.hw->Receive(packet, parsed);
        };
        for (uint32_t i = 0; i < kPackets; ++i)
        {
            Ptr<Packet> packet = sender.hw->GetNxtPacket(qp);
            CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                                CustomHeader::L4_Header);
            packet->PeekHeader(parsed);
            const uint8_t spine = RequestedSpine(parsed.ipid);
            NS_TEST_ASSERT_MSG_EQ(parsed.udp.path_seq,
                                  static_cast<uint16_t>(sent[spine]),
                                  "each spine's packets carry its count");
            NS_TEST_ASSERT_MSG_EQ(parsed.udp.seq,
                                  i * IsolatedHost::kMtu,
                                  "the sequence keeps its place");
            ++sent[spine];
            if (spine == kLossy && loss->GetValue() < 0.01)
            {
                ++dropped;
                continue;
            }
            if (spine == 1)
            {
                delayed.push_back(packet);
                continue;
            }
            deliver(packet);
            if (delayed.size() > 4)
            {
                deliver(delayed.front());
                delayed.pop_front();
            }
        }
        while (!delayed.empty())
        {
            deliver(delayed.front());
            delayed.pop_front();
        }
        NS_TEST_EXPECT_MSG_GT(dropped, 10u, "some packets up spine 3 are dropped");
        NS_TEST_ASSERT_MSG_EQ(receiver.hw->m_spineGaps.size(), kSpines, "every spine arrived");
        NS_TEST_EXPECT_MSG_EQ(receiver.hw->m_spineGaps[kLossy],
                              dropped,
                              "the gaps on spine 3 are its dropped packets");
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_GT(sent[spine], 0u, "every spine is sent on");
            if (spine != kLossy)
            {
                NS_TEST_EXPECT_MSG_EQ(receiver.hw->m_spineGaps[spine],
                                      0u,
                                      "no other spine shows a gap");
            }
        }
    }
};

class SprayReceiverHoldTest : public TestCase
{
  public:
    SprayReceiverHoldTest()
        : TestCase("Under SprayReceiverHold the receiver holds a spine for one sender leaf or "
                   "every one by the leaves losing on it, reports it in each acknowledgement, "
                   "and the senders hold it toward its leaf until the bit clears")
    {
    }

    void DoRun() override
    {
        const IntHeader::Mode savedIntMode = IntHeader::mode;
        const bool savedPathSeq = CustomHeader::udpCarriesPathSeq;
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        const uint32_t savedHoldBytes = CustomHeader::ackReceiverHoldBytes;
        IntHeader::mode = IntHeader::NONE;
        CustomHeader::ackCarriesPacketSeq = true;
        CustomHeader::udpCarriesPathSeq = true;

        CustomHeader::ackReceiverHoldBytes = 0;
        const uint32_t without = qbbHeader().GetSerializedSize();
        CustomHeader::ackReceiverHoldBytes = 1;
        NS_TEST_EXPECT_MSG_EQ(qbbHeader().GetSerializedSize(),
                              without + 1,
                              "the bitmap adds one byte");
        NS_TEST_EXPECT_MSG_EQ(CustomHeader::GetAckSerializedSize(),
                              without + 1,
                              "the parser agrees on the size");

        Unit();
        Transport();

        CustomHeader::ackReceiverHoldBytes = savedHoldBytes;
        CustomHeader::udpCarriesPathSeq = savedPathSeq;
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        IntHeader::mode = savedIntMode;
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint32_t kClean = 20;
    static constexpr uint64_t kSentinelNs = 1000;

    void Unit()
    {
        ReceiverHold hold(kSpines, 2, 3, 0);
        hold.OnArrival(0, 3, 1, false, 0);
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 3), true, "a loss holds the spine for its leaf");
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(1, 3), false, "and not for another");
        NS_TEST_EXPECT_MSG_EQ(hold.Bitmap(0), 0x08, "bit 3 for leaf 0");
        hold.OnArrival(0, 3, 0, false, 0);
        hold.OnArrival(0, 3, 0, true, 0);
        NS_TEST_EXPECT_MSG_EQ(hold.Clean(0, 3), 1u, "a late arrival counts nothing");
        hold.OnArrival(0, 3, 2, false, 0);
        NS_TEST_EXPECT_MSG_EQ(hold.Clean(0, 3), 0u, "a gap restarts the count");
        hold.OnArrival(1, 3, 0, false, 0);
        NS_TEST_EXPECT_MSG_EQ(hold.Clean(0, 3), 0u, "another leaf's arrival is not leaf 0's");
        for (uint32_t i = 0; i < 3; ++i)
        {
            hold.OnArrival(0, 3, 0, false, 0);
        }
        NS_TEST_EXPECT_MSG_EQ(hold.Holds(0, 3), false, "the clean arrivals release it");
        NS_TEST_EXPECT_MSG_EQ(hold.LossLeaves(3), 0u, "and take leaf 0 out");

        hold.OnArrival(0, 5, 1, false, 0);
        hold.OnArrival(2, 5, 1, false, 0);
        NS_TEST_EXPECT_MSG_EQ(hold.HostHolds(5), true, "a second leaf widens to every leaf");
        NS_TEST_EXPECT_MSG_EQ(hold.Bitmap(7), 0x20, "even toward a leaf that lost nothing");
        hold.OnArrival(7, 5, 0, false, 0);
        hold.OnArrival(0, 5, 0, false, 0);
        hold.OnArrival(2, 5, 0, false, 0);
        NS_TEST_EXPECT_MSG_EQ(hold.HostHolds(5), false, "arrivals from any leaf release it");
        NS_TEST_EXPECT_MSG_EQ(hold.LossLeaves(5), 0u, "and clear its leaves");

        ReceiverHoldMarks marks(kSpines, kSentinelNs);
        marks.OnBitmap(4, 100, 0x08);
        marks.OnBitmap(4, 101, 0x08);
        marks.OnBitmap(4, 100, 0x00);
        NS_TEST_EXPECT_MSG_EQ(marks.Holds(4, 3), true, "held while one receiver holds it");
        marks.OnBitmap(4, 101, 0x00);
        NS_TEST_EXPECT_MSG_EQ(marks.Holds(4, 3), false, "and released by the last of them");
    }

    struct Host
    {
        Host(Ipv4Address self, const std::vector<Ipv4Address>& peers, Ptr<SwitchQueueState> leaves)
            : address(self),
              host(LoadBalancingMode::SprayUniform, peers.front(), kSpines)
        {
            Ptr<RdmaHw> hw = host.hw;
            for (const Ipv4Address& peer : peers)
            {
                hw->m_rtTable[peer.Get()] = {0};
            }
            hw->m_retransmission_timeout_ns = 0;
            hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::RepsHold);
            hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Auto);
            hw->m_sprayHoldAutoLeaves = 2;
            hw->m_sprayLossThreshold1 = 0.1;
            hw->m_sprayLossThreshold2 = 0.25;
            hw->m_sprayLossThreshold3 = 0.5;
            hw->m_sprayLossIntervalNs = 100000;
            hw->m_sprayLossHoldCleanSentinels = 3;
            hw->m_sprayHeldExploration = SpineScores::HeldExploration::Sentinel;
            hw->m_sprayHeldSentinelIntervalNs = kSentinelNs;
            hw->m_sprayPathSeq = true;
            hw->m_sprayReceiverHold = true;
            hw->m_sprayReceiverHoldClean = kClean;
            hw->m_switchQueues = leaves;
        }

        Ipv4Address address;
        IsolatedHost host;
    };

    // A data packet from sender carried by spine with path sequence pathSeq,
    // handed to receiver; the receiver_hold bitmap of its acknowledgement.
    static uint8_t Deliver(Host& receiver, Host& sender, uint32_t seq, uint8_t spine, uint16_t pathSeq)
    {
        Ptr<Packet> packet = Create<Packet>(IsolatedHost::kMtu);
        SeqTsHeader seqTs;
        seqTs.SetSeq(seq);
        seqTs.SetPG(IsolatedHost::kPriorityGroup);
        seqTs.SetPathSeq(pathSeq);
        packet->AddHeader(seqTs);
        UdpHeader udp;
        udp.SetSourcePort(IsolatedHost::kSenderPort);
        udp.SetDestinationPort(IsolatedHost::kReceiverPort);
        udp.ForcePayloadSize(CustomHeader::GetUdpHeaderSize() + IsolatedHost::kMtu);
        packet->AddHeader(udp);
        Ipv4Header ip;
        ip.SetSource(sender.address);
        ip.SetDestination(receiver.address);
        ip.SetProtocol(0x11);
        ip.SetDscp(static_cast<Ipv4Header::DscpType>(kUetDscpTrimmable));
        ip.SetIdentification(SpineIdentification(spine, spine));
        ip.SetPayloadSize(packet->GetSize());
        packet->AddHeader(ip);
        PppHeader ppp;
        ppp.SetProtocol(0x0021);
        packet->AddHeader(ppp);
        CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                            CustomHeader::L4_Header);
        packet->PeekHeader(parsed);
        const size_t before = receiver.host.emitted.size();
        receiver.host.hw->Receive(packet, parsed);
        NS_ASSERT_MSG(receiver.host.emitted.size() == before + 1, "every arrival is acknowledged");
        const CustomHeader& ack = receiver.host.emitted.back();
        // The acknowledgement reaches its sender, whose queue pair follows it.
        CustomHeader answer = ack;
        answer.ack.seq = 0;
        sender.host.hw->ReceiveAck(Create<Packet>(), answer);
        return ack.ack.receiver_hold;
    }

    // Spines chosen by the queue pair at times from nowNs, one per 2 ns, so
    // that the draws span less than a sentinel interval.
    static std::vector<uint32_t> Draws(Ptr<RdmaQueuePair> qp, uint64_t nowNs, uint32_t sends)
    {
        std::vector<uint32_t> perSpine(kSpines, 0);
        for (uint32_t i = 0; i < sends; ++i)
        {
            const uint16_t id = qp->m_pathSelector->Choose(nowNs + 2 * i);
            ++perSpine[RequestedSpine(id)];
            // Every second answer marked, so that REPS draws fresh spines too.
            qp->m_pathSelector->OnAck(RequestedSpine(id), i % 2 == 1, nowNs + 2 * i);
        }
        return perSpine;
    }

    void Transport()
    {
        // Senders a1 and a2 on leaf 0, b on leaf 1, receivers r on leaf 2 and
        // q on leaf 3.
        const Ipv4Address a1("11.0.1.1"), a2("11.0.1.2"), b("11.0.3.1"), r("11.0.2.1"),
            q("11.0.4.1");
        Ptr<SwitchQueueState> leaves = Create<SwitchQueueState>(4, kSpines);
        leaves->SetLeafOf(a1.Get(), 0);
        leaves->SetLeafOf(a2.Get(), 0);
        leaves->SetLeafOf(b.Get(), 1);
        leaves->SetLeafOf(r.Get(), 2);
        leaves->SetLeafOf(q.Get(), 3);
        Host receiver(r, {a1, a2, b}, leaves);
        Host sa1(a1, {r, q}, leaves), sa2(a2, {r}, leaves), sb(b, {r}, leaves);
        NS_TEST_EXPECT_MSG_EQ(receiver.host.hw->ValidReceiverHold(), true, "accepted");
        Ptr<RdmaHw> unset = CreateObject<RdmaHw>();
        NS_TEST_EXPECT_MSG_EQ(unset->m_sprayReceiverHold, false, "off by default");
        UintegerValue clean;
        unset->GetAttribute("SprayReceiverHoldClean", clean);
        NS_TEST_EXPECT_MSG_EQ(clean.Get(), 1024u, "SprayReceiverHoldClean is 1024 by default");
        receiver.host.hw->m_sprayPathSeq = false;
        NS_TEST_EXPECT_MSG_EQ(receiver.host.hw->ValidReceiverHold(),
                              false,
                              "refused without the path sequences");
        receiver.host.hw->m_sprayPathSeq = true;

        std::map<Host*, Ptr<RdmaQueuePair>> qps;
        for (Host* sender : {&sa1, &sa2, &sb})
        {
            qps[sender] = sender->host.AddSender(sender->address, r, 1000000000);
            sender->host.hw->AttachReceiverHold(qps[sender]);
        }
        Ptr<RdmaQueuePair> toQ = sa1.host.AddSender(a1, q, 1000000000, 10002);
        sa1.host.hw->AttachReceiverHold(toQ);
        std::map<std::pair<Host*, uint8_t>, uint16_t> pathSeq;
        uint32_t seq = 0;
        auto deliver = [&](Host& sender, uint8_t spine, uint16_t skip = 0) {
            uint16_t& next = pathSeq[{&sender, spine}];
            next += skip;
            seq += IsolatedHost::kMtu;
            return Deliver(receiver, sender, seq, spine, next++);
        };
        auto spine3 = [&](Host& sender) { return deliver(sender, 3) & 0x08; };

        // (b) Sender leaf 0 loses on spine 3: only its senders see the bit.
        deliver(sa1, 3);
        NS_TEST_EXPECT_MSG_EQ(deliver(sa1, 3, 1), 0x08, "a loss from leaf 0 holds spine 3 for it");
        NS_TEST_EXPECT_MSG_EQ(receiver.host.hw->m_receiverHold->HostHolds(3),
                              false,
                              "for leaf 0 alone");
        NS_TEST_EXPECT_MSG_EQ(deliver(sa2, 1), 0x08, "the other sender on leaf 0 sees it");
        NS_TEST_EXPECT_MSG_EQ(deliver(sb, 1), 0x00, "the sender on leaf 1 does not");
        std::vector<uint32_t> drawn = Draws(qps[&sa2], 0, 400);
        NS_TEST_EXPECT_MSG_EQ(drawn[3], 0u, "leaf 0's sender stops drawing spine 3");
        drawn = Draws(qps[&sa1], 0, 400);
        NS_TEST_EXPECT_MSG_EQ(drawn[3], 0u, "as does the sender that lost");
        drawn = Draws(qps[&sb], 0, 400);
        NS_TEST_EXPECT_MSG_GT(drawn[3], 10u, "leaf 1's sender keeps it");
        drawn = Draws(toQ, 0, 400);
        NS_TEST_EXPECT_MSG_GT(drawn[3], 10u, "and toward another leaf spine 3 is kept");

        // (a) Sender leaf 1 loses on spine 3 too: the receiver holds it for
        // every leaf and both leaves' senders see the bit.
        deliver(sb, 3);
        NS_TEST_EXPECT_MSG_EQ(deliver(sb, 3, 2), 0x08, "a loss from leaf 1 holds spine 3");
        NS_TEST_EXPECT_MSG_EQ(receiver.host.hw->m_receiverHold->HostHolds(3),
                              true,
                              "for every sender leaf");
        NS_TEST_EXPECT_MSG_EQ(deliver(sa1, 1), 0x08, "leaf 0's sender sees it");
        for (Host* sender : {&sa1, &sa2, &sb})
        {
            drawn = Draws(qps[sender], 1000000, 400);
            NS_TEST_EXPECT_MSG_EQ(drawn[3],
                                  1u,
                                  "toward the receiver's leaf spine 3 is sent one sentinel");
        }
        drawn = Draws(toQ, 1000000, 400);
        NS_TEST_EXPECT_MSG_GT(drawn[3], 10u, "toward another leaf it is kept");

        // (c) The clean arrivals release it; the next acknowledgements clear
        // the bit and the senders draw spine 3 again.
        for (uint32_t i = 0; i + 1 < kClean; ++i)
        {
            NS_TEST_EXPECT_MSG_EQ(spine3(i % 2 == 0 ? sa1 : sb), 0x08, "held until the count");
        }
        NS_TEST_EXPECT_MSG_EQ(spine3(sb), 0x00, "released after the clean arrivals");
        deliver(sa1, 1);
        deliver(sa2, 1);
        for (Host* sender : {&sa1, &sa2, &sb})
        {
            drawn = Draws(qps[sender], 2000000, 400);
            NS_TEST_EXPECT_MSG_GT(drawn[3], 10u, "the senders re-admit spine 3");
        }

        // Every scope of reps_hold, and reps_flow_hold, follows the bitmap.
        for (uint32_t scope : {static_cast<uint32_t>(HoldScope::Host),
                               static_cast<uint32_t>(HoldScope::Leaf),
                               kFlowHold})
        {
            Host sender(a1, {r}, leaves);
            Ptr<RdmaHw> hw = sender.host.hw;
            if (scope == kFlowHold)
            {
                hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::RepsFlowHold);
            }
            else
            {
                hw->m_sprayHoldScope = scope;
            }
            Ptr<RdmaQueuePair> flow = sender.host.AddSender(a1, r, 1000000000);
            hw->AttachReceiverHold(flow);
            flow->m_pathSelector->OnReceiverHold(r.Get(), 0x08, 0);
            drawn = Draws(flow, 0, 400);
            NS_TEST_EXPECT_MSG_EQ(drawn[3], 0u, "a set bit holds the spine under every scope");
            drawn = Draws(flow, 1000000, 400);
            NS_TEST_EXPECT_MSG_EQ(drawn[3], 1u, "and sends it one sentinel per interval");
            flow->m_pathSelector->OnReceiverHold(r.Get(), 0x00, 2000000);
            drawn = Draws(flow, 2000000, 400);
            NS_TEST_EXPECT_MSG_GT(drawn[3], 10u, "a clear bit releases it");
        }
    }

    static constexpr uint32_t kFlowHold = 99;
};

class SprayReceiverHoldTraceTest : public TestCase
{
  public:
    SprayReceiverHoldTraceTest()
        : TestCase("Under SprayReceiverHold the receiving host traces each closed interval per "
                   "sender leaf with the arrivals, the missing packets and each spine's hold")
    {
    }

    void DoRun() override
    {
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        hw->m_spineCount = 8;
        hw->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
        hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::RepsHold);
        hw->m_sprayLossIntervalNs = 1000;
        hw->m_sprayHoldAutoLeaves = 2;
        hw->m_sprayReceiverHoldClean = 3;
        hw->m_sprayPathSeq = true;
        hw->m_sprayReceiverHold = true;
        struct Row
        {
            uint32_t leaf;
            uint64_t endNs;
            std::vector<uint32_t> arrivals;
            std::vector<uint32_t> lost;
            std::vector<uint8_t> holds;
        };
        std::vector<Row> rows;
        hw->TraceConnectWithoutContext(
            "ReceiverHoldInterval",
            Callback<void,
                     uint32_t,
                     uint64_t,
                     const std::vector<uint32_t>&,
                     const std::vector<uint32_t>&,
                     const std::vector<uint8_t>&>(
                [&](uint32_t leaf,
                    uint64_t endNs,
                    const std::vector<uint32_t>& arrivals,
                    const std::vector<uint32_t>& lost,
                    const std::vector<uint8_t>& holds) {
                    rows.push_back(Row{leaf, endNs, arrivals, lost, holds});
                }));
        ReceiverHold& hold = hw->ReceiverHoldOf();
        hold.OnArrival(0, 3, 2, false, 100);
        hold.OnArrival(1, 2, 0, false, 500);
        NS_TEST_EXPECT_MSG_EQ(rows.size(), 0u, "nothing is traced before the interval ends");
        hold.OnArrival(1, 3, 1, false, 2500);
        NS_TEST_ASSERT_MSG_EQ(rows.size(), 2u, "one row set per sender leaf seen");
        NS_TEST_EXPECT_MSG_EQ(rows[0].leaf, 0u, "in leaf order");
        NS_TEST_EXPECT_MSG_EQ(rows[0].endNs, 1000u, "at the end of the interval counted");
        NS_TEST_EXPECT_MSG_EQ(rows[0].arrivals[3], 1u, "leaf 0's arrival on spine 3");
        NS_TEST_EXPECT_MSG_EQ(rows[0].lost[3], 2u, "and its missing packets");
        NS_TEST_EXPECT_MSG_EQ(rows[0].holds[3],
                              static_cast<uint8_t>(ReceiverHold::Scope::Leaf),
                              "spine 3 held for leaf 0");
        NS_TEST_EXPECT_MSG_EQ(rows[1].arrivals[2], 1u, "leaf 1's arrival on spine 2");
        NS_TEST_EXPECT_MSG_EQ(rows[1].holds[3],
                              static_cast<uint8_t>(ReceiverHold::Scope::None),
                              "and spine 3 not held for leaf 1");
        hold.OnArrival(0, 1, 0, false, 3100);
        NS_TEST_ASSERT_MSG_EQ(rows.size(), 4u, "the next interval closes");
        NS_TEST_EXPECT_MSG_EQ(rows[2].endNs, 3000u, "the empty interval between is skipped");
        NS_TEST_EXPECT_MSG_EQ(rows[3].lost[3], 1u, "leaf 1's missing packet");
        NS_TEST_EXPECT_MSG_EQ(rows[3].holds[3],
                              static_cast<uint8_t>(ReceiverHold::Scope::Host),
                              "widened spine 3 to every sender leaf");
        NS_TEST_EXPECT_MSG_EQ(rows[2].holds[3],
                              static_cast<uint8_t>(ReceiverHold::Scope::Host),
                              "for leaf 0 too");
        Simulator::Destroy();
    }
};

class SprayPathSeqTrimTest : public TestCase
{
  public:
    SprayPathSeqTrimTest()
        : TestCase("Under SprayPathSeq a trimmed header accounts its path sequence without "
                   "moving the receiver's expected one on, so trims alone, also ahead of "
                   "the data they overtake, hold no spine")
    {
    }

    void DoRun() override
    {
        const IntHeader::Mode savedIntMode = IntHeader::mode;
        const bool savedPathSeq = CustomHeader::udpCarriesPathSeq;
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        const uint32_t savedHoldBytes = CustomHeader::ackReceiverHoldBytes;
        IntHeader::mode = IntHeader::NONE;
        CustomHeader::ackCarriesPacketSeq = true;
        CustomHeader::udpCarriesPathSeq = true;
        CustomHeader::ackReceiverHoldBytes = 1;

        Overtaking();

        // Trims on spines 2 and 3, no link loss.
        Outcome trimsOnly = Run(false);
        NS_TEST_EXPECT_MSG_GT(trimsOnly.trims, 100u, "the congested spines trim");
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(trimsOnly.gaps[spine], 0u, "a trim is not a missing packet");
            NS_TEST_EXPECT_MSG_EQ(trimsOnly.lossLeaves[spine], 0u, "nor a loss of the hold");
        }
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(trimsOnly.bitmap),
                              0u,
                              "trims alone hold no spine");

        // The same trims and a 1% link loss on spine 3.
        Outcome lossy = Run(true);
        NS_TEST_EXPECT_MSG_GT(lossy.drops, 10u, "spine 3 drops packets");
        NS_TEST_EXPECT_MSG_EQ(lossy.gaps[kLossy],
                              lossy.drops,
                              "the missing packets on spine 3 are its link drops");
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            if (spine != kLossy)
            {
                NS_TEST_EXPECT_MSG_EQ(lossy.gaps[spine], 0u, "no other spine misses a packet");
            }
        }
        NS_TEST_EXPECT_MSG_EQ(lossy.lossLeaves[kLossy], 1u, "the link drops hold spine 3");

        // The trimmed headers of path sequences 10 to 20 on spine 5 overtake
        // the data of 5 to 9, as the switches' high-priority queue lets them.
        Outcome overtaken = Run(false, true);
        NS_TEST_EXPECT_MSG_EQ(overtaken.overtaking, 11u, "eleven trimmed headers overtake");
        NS_TEST_EXPECT_MSG_EQ(overtaken.overtaken, 5u, "five data packets");
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            NS_TEST_EXPECT_MSG_EQ(overtaken.gaps[spine], 0u, "no packet is missing");
            NS_TEST_EXPECT_MSG_EQ(overtaken.lossLeaves[spine], 0u, "nor a loss of the hold");
        }
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(overtaken.bitmap),
                              0u,
                              "the overtaking trims hold no spine");

        CustomHeader::ackReceiverHoldBytes = savedHoldBytes;
        CustomHeader::udpCarriesPathSeq = savedPathSeq;
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        IntHeader::mode = savedIntMode;
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint8_t kLossy = 3;

    static constexpr uint8_t kOvertaken = 5;

    // Trimmed headers of sequences 10 to 20 arrive before the data of 5 to 9
    // on one path: nothing is missing, and the data arrive in order.
    void Overtaking()
    {
        SpineSequenceReceiver receiver;
        for (uint16_t seq = 0; seq < 5; ++seq)
        {
            NS_TEST_EXPECT_MSG_EQ(receiver.OnArrival(kOvertaken, kOvertaken, seq).gaps,
                                  0u,
                                  "the data before the trims in order");
        }
        for (uint16_t seq = 10; seq <= 20; ++seq)
        {
            receiver.OnTrim(kOvertaken, kOvertaken, seq);
        }
        for (uint16_t seq = 5; seq < 10; ++seq)
        {
            const SpineSequenceReceiver::Arrival arrival =
                receiver.OnArrival(kOvertaken, kOvertaken, seq);
            NS_TEST_EXPECT_MSG_EQ(arrival.gaps, 0u, "the overtaken data shows no gap");
            NS_TEST_EXPECT_MSG_EQ(arrival.late, false, "and is consumed in order");
        }
        NS_TEST_EXPECT_MSG_EQ(receiver.OnArrival(kOvertaken, kOvertaken, 21).gaps,
                              0u,
                              "the trims accounted 10 to 20");
        NS_TEST_EXPECT_MSG_EQ(receiver.Accounted(kOvertaken, 15),
                              false,
                              "and are dropped once passed");
        NS_TEST_EXPECT_MSG_EQ(receiver.OnArrival(kOvertaken, kOvertaken, 23).gaps,
                              1u,
                              "a data packet lost after them is missing");
        // A trimmed header behind the expected sequence, or too far ahead of
        // it, is not accounted.
        receiver.OnTrim(kOvertaken, kOvertaken, 2);
        NS_TEST_EXPECT_MSG_EQ(receiver.Accounted(kOvertaken, 2), false, "behind");
        receiver.OnTrim(kOvertaken,
                        kOvertaken,
                        24 + SpineSequenceReceiver::kAccountedWindow);
        NS_TEST_EXPECT_MSG_EQ(receiver.Accounted(kOvertaken,
                                                 24 + SpineSequenceReceiver::kAccountedWindow),
                              false,
                              "beyond the window");
        receiver.OnTrim(kOvertaken,
                        kOvertaken,
                        23 + SpineSequenceReceiver::kAccountedWindow);
        NS_TEST_EXPECT_MSG_EQ(receiver.Accounted(kOvertaken,
                                                 23 + SpineSequenceReceiver::kAccountedWindow),
                              true,
                              "the window's last");
        // A moved path's trims before its first data arrival.
        receiver.OnTrim(2, 6, 30);
        receiver.OnTrim(2, 6, 12);
        NS_TEST_EXPECT_MSG_EQ(receiver.OnArrival(2, 6, 28).gaps, 0u, "a moved path starts");
        NS_TEST_EXPECT_MSG_EQ(receiver.Accounted(2, 12), false, "the trims behind it dropped");
        NS_TEST_EXPECT_MSG_EQ(receiver.OnArrival(2, 6, 31).gaps, 1u, "29 missing, 30 accounted");
    }

    struct Outcome
    {
        uint32_t trims = 0;
        uint32_t drops = 0;
        uint32_t overtaking = 0;
        uint32_t overtaken = 0;
        std::vector<uint64_t> gaps;
        std::vector<uint32_t> lossLeaves;
        uint8_t bitmap = 0;
    };

    // Trim packet as SwitchNode::TrimInPlace does with MIN_TRIM_SIZE 24: keep
    // the larger of it and the data headers of IP payload, and mark it.
    static void Trim(Ptr<Packet> packet)
    {
        const uint32_t kept = std::max<uint32_t>(std::max<uint32_t>(24, CustomHeader::GetUdpHeaderSize()),
                                                 60 - 14 - 20);
        packet->RemoveAtEnd(packet->GetSize() - PppHeader::GetStaticSize() - 20 - kept);
        PppHeader ppp;
        Ipv4Header ip;
        packet->RemoveHeader(ppp);
        packet->RemoveHeader(ip);
        ip.SetDscp(static_cast<Ipv4Header::DscpType>(kUetDscpTrimmed));
        ip.SetPayloadSize(kept);
        packet->AddHeader(ip);
        packet->AddHeader(ppp);
    }

    // With overtake, no other trim or loss: spine kOvertaken's data packets
    // of path sequences 5 to 9 are held back, its packets of 10 to 20 are
    // trimmed and delivered at once, and the held ones follow the last trim.
    Outcome Run(bool linkLoss, bool overtake = false)
    {
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, kSpines);
        IsolatedHost receiver(LoadBalancingMode::SprayUniform, kTestSender, kSpines);
        Ptr<SwitchQueueState> leaves = Create<SwitchQueueState>(2, kSpines);
        leaves->SetLeafOf(kTestSender.Get(), 0);
        leaves->SetLeafOf(kTestReceiver.Get(), 1);
        for (IsolatedHost* host : {&sender, &receiver})
        {
            Ptr<RdmaHw> hw = host->hw;
            hw->m_retransmission_timeout_ns = 0;
            hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::RepsHold);
            hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Host);
            hw->m_sprayLossThreshold1 = 0.1;
            hw->m_sprayLossThreshold2 = 0.25;
            hw->m_sprayLossThreshold3 = 0.5;
            hw->m_sprayLossIntervalNs = 10000;
            hw->m_sprayLossHoldCleanSentinels = 3;
            hw->m_sprayHeldExploration = SpineScores::HeldExploration::Sentinel;
            hw->m_sprayPathSeq = true;
            hw->m_sprayReceiverHold = true;
            hw->m_sprayHoldAutoLeaves = 2;
            hw->m_switchQueues = leaves;
        }
        constexpr uint32_t kPackets = 20000;
        Ptr<RdmaQueuePair> qp = sender.AddSender(kTestSender, kTestReceiver,
                                                 static_cast<uint64_t>(kPackets) * IsolatedHost::kMtu);
        Ptr<UniformRandomVariable> draw =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(11));
        Outcome outcome;
        std::vector<Ptr<Packet>> heldBack;
        for (uint32_t i = 0; i < kPackets; ++i)
        {
            Ptr<Packet> packet = sender.hw->GetNxtPacket(qp);
            CustomHeader sent(CustomHeader::L2_Header | CustomHeader::L3_Header |
                              CustomHeader::L4_Header);
            packet->PeekHeader(sent);
            const uint8_t spine = RequestedSpine(sent.ipid);
            const double u = draw->GetValue();
            if (linkLoss && spine == kLossy && u < 0.01)
            {
                ++outcome.drops;
                continue;
            }
            const bool onOvertaken = overtake && spine == kOvertaken;
            if (onOvertaken && sent.udp.path_seq >= 5 && sent.udp.path_seq <= 9)
            {
                heldBack.push_back(packet);
                continue;
            }
            const bool overtaking =
                onOvertaken && sent.udp.path_seq >= 10 && sent.udp.path_seq <= 20;
            const bool trimmed = overtake
                ? overtaking
                : (spine == 2 && u < 0.3) || (spine == kLossy && u > 0.9);
            if (trimmed)
            {
                Trim(packet);
                ++outcome.trims;
            }
            CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                                CustomHeader::L4_Header);
            packet->PeekHeader(parsed);
            NS_TEST_EXPECT_MSG_EQ(parsed.udp.path_seq,
                                  sent.udp.path_seq,
                                  "the trimmed header keeps the path sequence");
            receiver.hw->Receive(packet, parsed);
            outcome.overtaking += overtaking;
            if (overtaking && sent.udp.path_seq == 20)
            {
                for (Ptr<Packet> held : heldBack)
                {
                    CustomHeader late(CustomHeader::L2_Header | CustomHeader::L3_Header |
                                      CustomHeader::L4_Header);
                    held->PeekHeader(late);
                    receiver.hw->Receive(held, late);
                    ++outcome.overtaken;
                }
                heldBack.clear();
            }
        }
        Ptr<RdmaHw> hw = receiver.hw;
        outcome.gaps = hw->m_spineGaps;
        outcome.gaps.resize(kSpines, 0);
        ReceiverHold& hold = hw->ReceiverHoldOf();
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            outcome.lossLeaves.push_back(hold.LossLeaves(spine));
        }
        outcome.bitmap = hold.Bitmap(0);
        return outcome;
    }
};

class SprayTailRecoveryTest : public TestCase
{
  public:
    SprayTailRecoveryTest()
        : TestCase("Under SprayTailRecovery the packets a queue pair sends once it has no new "
                   "data left close their spines and carry each spine's count, the receiver asks "
                   "for the path sequences still missing after its delay, and the sender repairs "
                   "a lost tail without a retransmission timeout")
    {
    }

    void DoRun() override
    {
        const IntHeader::Mode savedIntMode = IntHeader::mode;
        const bool savedPathSeq = CustomHeader::udpCarriesPathSeq;
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        const uint32_t savedCounts = CustomHeader::udpTailCounts;
        const uint32_t savedNack = CustomHeader::ackTailNackBytes;
        IntHeader::mode = IntHeader::NONE;
        CustomHeader::ackCarriesPacketSeq = true;
        CustomHeader::udpCarriesPathSeq = true;

        Unit();
        Refusals();
        TailLoss();
        LostClosingPacket();
        OvertakenClosingPacket();
        TrimmedAfterRequest();
        TrimmedAfterCompletion();
        LostAfterCompletion();
        LossReportedAtOnce();

        CustomHeader::ackTailNackBytes = savedNack;
        CustomHeader::udpTailCounts = savedCounts;
        CustomHeader::udpCarriesPathSeq = savedPathSeq;
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        IntHeader::mode = savedIntMode;
        Simulator::Destroy();
    }

  private:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint8_t kLossy = 3;
    static constexpr uint32_t kMtu = IsolatedHost::kMtu;
    static constexpr uint64_t kOneWayNs = 1000;
    static constexpr uint64_t kBaseRttNs = 2 * kOneWayNs;
    static constexpr uint64_t kStepNs = 100;
    static constexpr uint64_t kTimeoutNs = 20000;
    // The receiver's delay, the base RTT by default.
    static constexpr uint64_t kDelayNs = kBaseRttNs;
    // How long after the last packet a slower spine delivers its own last.
    static constexpr uint64_t kOvertakeNs = kDelayNs / 2;
    // The new data packets up spine 3 among the message's last ones that a
    // tail loss drops.
    static constexpr uint32_t kTailPackets = 16;
    // The hold's loss interval under RepsHold.
    static constexpr uint64_t kLossIntervalNs = 10000;

    static void SetHeaders(bool on)
    {
        CustomHeader::udpTailCounts = on ? kSpines : 0;
        CustomHeader::ackTailNackBytes = on ? 5 : 0;
    }

    // Trim packet as SwitchNode::TrimInPlace does with MIN_TRIM_SIZE 24: keep
    // the larger of it and the data headers of IP payload, and mark it.
    static void Trim(Ptr<Packet> packet)
    {
        const uint32_t kept =
            std::max<uint32_t>(std::max<uint32_t>(24, CustomHeader::GetUdpHeaderSize()),
                               60 - 14 - 20);
        packet->RemoveAtEnd(packet->GetSize() - PppHeader::GetStaticSize() - 20 - kept);
        PppHeader ppp;
        Ipv4Header ip;
        packet->RemoveHeader(ppp);
        packet->RemoveHeader(ip);
        ip.SetDscp(static_cast<Ipv4Header::DscpType>(kUetDscpTrimmed));
        ip.SetPayloadSize(kept);
        packet->AddHeader(ip);
        packet->AddHeader(ppp);
    }

    static CustomHeader Parse(Ptr<const Packet> packet)
    {
        CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                            CustomHeader::L4_Header);
        packet->PeekHeader(parsed);
        return parsed;
    }

    void Unit()
    {
        SetHeaders(false);
        const uint32_t udp = CustomHeader::GetUdpHeaderSize();
        const uint32_t ack = CustomHeader::GetAckSerializedSize();
        const uint32_t qbb = qbbHeader().GetSerializedSize();
        SetHeaders(true);
        NS_TEST_EXPECT_MSG_EQ(CustomHeader::GetUdpHeaderSize(),
                              udp + 1 + 2 * kSpines,
                              "the data header grows by a flag byte and a count per spine");
        NS_TEST_EXPECT_MSG_EQ(SeqTsHeader().GetSerializedSize(),
                              8u + 1 + 2 * kSpines,
                              "the writer agrees on the size");
        NS_TEST_EXPECT_MSG_EQ(CustomHeader::GetAckSerializedSize(),
                              ack + 5,
                              "the acknowledgement header grows by the spine, sequence and count");
        NS_TEST_EXPECT_MSG_EQ(qbbHeader().GetSerializedSize(), qbb + 5, "the writer agrees");

        // A data header's tail fields, written and parsed.
        uint16_t counts[kSpines] = {1, 2, 3, 65535, 0, 7, 8, 9};
        SeqTsHeader seqTs;
        seqTs.SetSeq(5000);
        seqTs.SetPG(IsolatedHost::kPriorityGroup);
        seqTs.SetPathSeq(41);
        seqTs.SetTail(kTailCloses, counts, kSpines);
        Ptr<Packet> data = Create<Packet>(kMtu);
        data->AddHeader(seqTs);
        UdpHeader udpHeader;
        udpHeader.ForcePayloadSize(CustomHeader::GetUdpHeaderSize() + kMtu);
        data->AddHeader(udpHeader);
        Ipv4Header ip;
        ip.SetProtocol(0x11);
        ip.SetPayloadSize(data->GetSize());
        data->AddHeader(ip);
        PppHeader ppp;
        ppp.SetProtocol(0x0021);
        data->AddHeader(ppp);
        const CustomHeader parsed = Parse(data);
        NS_TEST_EXPECT_MSG_EQ(parsed.udp.path_seq, 41u, "the path sequence before the fields");
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(parsed.udp.tail_flags),
                              static_cast<uint32_t>(kTailCloses),
                              "the closing flag");
        for (uint32_t k = 0; k < kSpines; ++k)
        {
            NS_TEST_EXPECT_MSG_EQ(parsed.udp.tail_counts[k], counts[k], "each spine's count");
        }
        NS_TEST_EXPECT_MSG_EQ(data->GetSize() - parsed.GetSerializedSize(),
                              kMtu,
                              "the payload follows the fields");

        // A tail repair request, written and parsed.
        qbbHeader request;
        request.SetTailNack(kLossy, 65530, 7);
        NS_TEST_EXPECT_MSG_EQ(request.IsTailNack(), true, "the flag is set");
        Ptr<Packet> nack = Create<Packet>();
        nack->AddHeader(request);
        Ipv4Header nackIp;
        nackIp.SetProtocol(kUecTrimRepairProtocol);
        nackIp.SetPayloadSize(nack->GetSize());
        nack->AddHeader(nackIp);
        nack->AddHeader(ppp);
        const CustomHeader parsedNack = Parse(nack);
        NS_TEST_EXPECT_MSG_EQ(((parsedNack.ack.flags >> qbbHeader::FLAG_TAIL_NACK) & 1), 1,
                              "the parser reads the flag");
        NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(parsedNack.ack.tail_spine), 3u, "the spine");
        NS_TEST_EXPECT_MSG_EQ(parsedNack.ack.tail_seq, 65530u, "the first missing sequence");
        NS_TEST_EXPECT_MSG_EQ(parsedNack.ack.tail_count, 7u, "and the count");
        NS_TEST_EXPECT_MSG_EQ(((qbbHeader().GetSerializedSize() > 0) &&
                               !qbbHeader().IsTailNack()),
                              true,
                              "an acknowledgement is no tail repair request");

        SpineSequenceSender sender;
        NS_TEST_EXPECT_MSG_EQ(sender.Count(2), 0u, "nothing sent up spine 2");
        sender.Next(2);
        sender.Next(2);
        NS_TEST_EXPECT_MSG_EQ(sender.Count(2), 2u, "two sent up spine 2");
        NS_TEST_EXPECT_MSG_EQ(sender.Count(5), 0u, "each spine counts apart");

        SpineSequenceReceiver receiver;
        // The single run TakeMissing returns, or an empty one.
        auto take = [&](uint8_t spine, uint16_t owed) {
            const std::vector<SpineSequenceReceiver::Missing> runs =
                receiver.TakeMissing(spine, owed);
            NS_TEST_EXPECT_MSG_LT(runs.size(), 2u, "one run at most without trims");
            return runs.empty() ? SpineSequenceReceiver::Missing{0, 0} : runs.front();
        };
        SpineSequenceReceiver::Missing missing = take(1, 3);
        NS_TEST_EXPECT_MSG_EQ(missing.first, 0u, "nothing arrived: from 0");
        NS_TEST_EXPECT_MSG_EQ(missing.count, 3u, "three owed");
        NS_TEST_EXPECT_MSG_EQ(take(1, 3).count, 0u, "a run is taken once");
        missing = take(1, 5);
        NS_TEST_EXPECT_MSG_EQ(missing.first, 3u, "a later count takes the rest");
        NS_TEST_EXPECT_MSG_EQ(missing.count, 2u, "two more");
        receiver.OnArrival(2, 2, 0);
        receiver.OnArrival(2, 2, 1);
        NS_TEST_EXPECT_MSG_EQ(take(2, 2).count, 0u, "both arrived");
        missing = take(2, 4);
        NS_TEST_EXPECT_MSG_EQ(missing.first, 2u, "the tail after them");
        NS_TEST_EXPECT_MSG_EQ(missing.count, 2u, "is missing");
        // A packet requested up 4 that a source leaf moved onto 6 is
        // furthest on the path (4, 6).
        receiver.OnArrival(4, 4, 0);
        receiver.OnArrival(4, 6, 5);
        missing = take(4, 8);
        NS_TEST_EXPECT_MSG_EQ(missing.first, 6u, "from the moved packet on");
        NS_TEST_EXPECT_MSG_EQ(missing.count, 2u, "two owed");
        NS_TEST_EXPECT_MSG_EQ(take(7, 0).count, 0u, "nothing owed");
        receiver.OnArrival(5, 6, 65534);
        missing = take(5, 1);
        NS_TEST_EXPECT_MSG_EQ(missing.first, 65535u, "across the wrap");
        NS_TEST_EXPECT_MSG_EQ(missing.count, 2u, "65535 and 0");

        TrimmedNotRequested();

        PendingTailRuns runs;
        NS_TEST_EXPECT_MSG_EQ(runs.NextDue(), PendingTailRuns::kNone, "nothing pending");
        runs.Add(3, 10, 4, 100);
        runs.Add(5, 0, 1, 100);
        runs.Add(6, 7, 2, 200);
        runs.OnArrival(3, 11);
        runs.OnArrival(5, 0);
        runs.OnArrival(4, 12);
        NS_TEST_EXPECT_MSG_EQ(runs.NextDue(), 100u, "the earliest due");
        NS_TEST_EXPECT_MSG_EQ(runs.TakeDue(99).size(), 0u, "nothing due before it");
        const std::vector<PendingTailRuns::Request> due = runs.TakeDue(100);
        NS_TEST_EXPECT_MSG_EQ(due.size(), 2u, "spine 3's run split by its arrival, 5's gone");
        if (due.size() == 2)
        {
            NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(due[0].spine), 3u, "spine 3");
            NS_TEST_EXPECT_MSG_EQ(due[0].first, 10u, "10 alone");
            NS_TEST_EXPECT_MSG_EQ(due[0].count, 1u, "11 arrived");
            NS_TEST_EXPECT_MSG_EQ(due[1].first, 12u, "12 and 13");
            NS_TEST_EXPECT_MSG_EQ(due[1].count, 2u, "an arrival on spine 4 takes nothing");
        }
        NS_TEST_EXPECT_MSG_EQ(runs.NextDue(), 200u, "spine 6's run waits");
        runs.OnArrival(6, 7);
        runs.OnArrival(6, 8);
        NS_TEST_EXPECT_MSG_EQ(runs.TakeDue(200).size(), 0u, "all of it arrived");
        NS_TEST_EXPECT_MSG_EQ(runs.Empty(), true, "and is gone");
    }

    // A trimmed header accounts its path sequence: tail recovery treats it
    // as present and does not request it, while the data behind it on a
    // slower path is still owed.
    void TrimmedNotRequested()
    {
        SpineSequenceReceiver receiver;
        receiver.OnArrival(3, 3, 0);
        receiver.OnArrival(3, 3, 1);
        // The trimmed headers of 3 and 5 overtake the data of 2 and 4.
        receiver.OnTrim(3, 3, 3);
        receiver.OnTrim(3, 3, 5);
        NS_TEST_EXPECT_MSG_EQ(receiver.Accounted(3, 3), true, "3 is accounted");
        NS_TEST_EXPECT_MSG_EQ(receiver.Accounted(3, 2), false, "2 is not");
        NS_TEST_EXPECT_MSG_EQ(receiver.Accounted(4, 3), false, "per requested spine");
        std::vector<SpineSequenceReceiver::Missing> runs = receiver.TakeMissing(3, 7);
        NS_TEST_EXPECT_MSG_EQ(runs.size(), 3u, "2, 4 and 6, the trims left out");
        if (runs.size() == 3)
        {
            NS_TEST_EXPECT_MSG_EQ(runs[0].first, 2u, "2");
            NS_TEST_EXPECT_MSG_EQ(runs[0].count, 1u, "alone");
            NS_TEST_EXPECT_MSG_EQ(runs[1].first, 4u, "4");
            NS_TEST_EXPECT_MSG_EQ(runs[1].count, 1u, "alone");
            NS_TEST_EXPECT_MSG_EQ(runs[2].first, 6u, "6");
            NS_TEST_EXPECT_MSG_EQ(runs[2].count, 1u, "alone");
        }
        NS_TEST_EXPECT_MSG_EQ(receiver.TakeMissing(3, 7).size(), 0u, "taken once");
        // A trimmed sequence ahead of the expected one on a moved path is
        // accounted for its requested spine too.
        receiver.OnArrival(5, 5, 0);
        receiver.OnArrival(5, 1, 8);
        receiver.OnTrim(5, 1, 10);
        runs = receiver.TakeMissing(5, 12);
        NS_TEST_EXPECT_MSG_EQ(runs.size(), 2u, "9 and 11");
        if (runs.size() == 2)
        {
            NS_TEST_EXPECT_MSG_EQ(runs[0].first, 9u, "9");
            NS_TEST_EXPECT_MSG_EQ(runs[0].count, 1u, "alone");
            NS_TEST_EXPECT_MSG_EQ(runs[1].first, 11u, "11");
            NS_TEST_EXPECT_MSG_EQ(runs[1].count, 1u, "alone");
        }
        // Every owed sequence trimmed: nothing is requested.
        receiver.OnTrim(6, 6, 0);
        receiver.OnTrim(6, 6, 1);
        NS_TEST_EXPECT_MSG_EQ(receiver.TakeMissing(6, 2).size(), 0u, "both trimmed");
    }

    void Refusals()
    {
        NS_TEST_EXPECT_MSG_EQ(CreateObject<RdmaHw>()->m_sprayTailRecovery,
                              false,
                              "SprayTailRecovery is off by default");
        Ptr<RdmaHw> delayed = CreateObject<RdmaHw>();
        NS_TEST_EXPECT_MSG_EQ(delayed->m_sprayTailRecoveryDelayNs,
                              RdmaHw::kTailDelayBaseRtt,
                              "the delay is the base RTT by default");
        delayed->m_sprayBaseRttNs = 1234;
        NS_TEST_EXPECT_MSG_EQ(delayed->TailRecoveryDelayNs(), 1234u, "SprayBaseRttNs");
        delayed->m_sprayTailRecoveryDelayNs = 0;
        NS_TEST_EXPECT_MSG_EQ(delayed->TailRecoveryDelayNs(), 0u, "or as set");
        SetHeaders(false);
        NS_TEST_EXPECT_MSG_EQ(CreateObject<RdmaHw>()->ValidTailRecovery(),
                              true,
                              "off, with the headers as they are");
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        hw->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
        hw->m_spineCount = kSpines;
        hw->m_selective_retransmission = true;
        hw->m_sprayPathSeq = true;
        hw->m_sprayTailRecovery = true;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidTailRecovery(), false, "refused unless the headers grow");
        SetHeaders(true);
        for (PathSelectorKind kind : {PathSelectorKind::Ops,
                                      PathSelectorKind::Reps,
                                      PathSelectorKind::AckRedirect,
                                      PathSelectorKind::RepsHold,
                                      PathSelectorKind::RepsFlowHold,
                                      PathSelectorKind::RepsOracle,
                                      PathSelectorKind::MrcHold})
        {
            hw->m_pathSelectorKind = static_cast<uint32_t>(kind);
            NS_TEST_EXPECT_MSG_EQ(hw->ValidTailRecovery(),
                                  true,
                                  "accepted under every selector of spray_uniform");
        }
        hw->m_sprayPathSeq = false;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidTailRecovery(), false, "refused without SprayPathSeq");
        hw->m_sprayPathSeq = true;
        hw->m_selective_retransmission = false;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidTailRecovery(),
                              false,
                              "refused without SelectiveRetransmission");
        hw->m_selective_retransmission = true;
        hw->m_spineCount = kMaxTailCounts + 1;
        CustomHeader::udpTailCounts = kMaxTailCounts + 1;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidTailRecovery(), false, "refused above 8 spines");
        hw->m_spineCount = kSpines;
        CustomHeader::udpTailCounts = kSpines - 1;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidTailRecovery(),
                              false,
                              "refused unless the data header counts every spine");
        CustomHeader::udpTailCounts = kSpines;
        CustomHeader::ackTailNackBytes = 0;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidTailRecovery(),
                              false,
                              "refused unless the repair request carries the fields");
        CustomHeader::ackTailNackBytes = 5;
        hw->m_sprayTailRecovery = false;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidTailRecovery(), false, "off, the headers do not grow");
        hw->m_sprayTailRecovery = true;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidTailRecovery(), true, "accepted again");

        Ptr<RdmaHw> guarded = CreateObject<RdmaHw>();
        DoubleValue rtts;
        guarded->GetAttribute("SprayTailLossRtts", rtts);
        NS_TEST_EXPECT_MSG_EQ(rtts.Get(), 1.0, "SprayTailLossRtts is 1 by default");
        NS_TEST_EXPECT_MSG_EQ(guarded->ValidTailLoss(), true, "unset, accepted without tail recovery");
        for (double value : {0.0, 2.0})
        {
            guarded->SetAttribute("SprayTailLossRtts", DoubleValue(value));
            guarded->m_sprayTailRecovery = false;
            NS_TEST_EXPECT_MSG_EQ(guarded->ValidTailLoss(),
                                  false,
                                  "set, refused without SprayTailRecovery");
            guarded->m_sprayTailRecovery = true;
            NS_TEST_EXPECT_MSG_EQ(guarded->ValidTailLoss(),
                                  true,
                                  "accepted under SprayTailRecovery");
        }
    }

    enum class Loss
    {
        // 1% of the new data packets up spine 3 and every one among the
        // message's last kTailPackets, the last packet itself excepted.
        TailOnSpine3,
        // The newest new data packet still in flight when the last one is
        // sent, on a spine other than the last one's.
        ClosingPacket,
        // The same packet delayed instead, to arrive kOvertakeNs after the
        // last one.
        Overtaken,
        // The same packet trimmed instead: its header arrives trimLateNs
        // after the packet would have, and its payload is dropped.
        Trimmed,
    };

    struct Nack
    {
        uint8_t spine;
        uint16_t first;
        uint16_t count;
    };

    struct Outcome
    {
        bool completed = false;
        // When the queue pair completed, and its losses then waiting on the
        // guard.
        uint64_t completedNs = 0;
        uint32_t waitingAtCompletion = 0;
        uint32_t timeouts = 0;
        uint32_t qpTimeouts = 0;
        // When the message's last new data packet was sent, and up which
        // spine, and when the receiver held every byte.
        uint64_t endNs = 0;
        uint8_t lastSpine = 0;
        uint64_t receivedNs = 0;
        uint32_t drops = 0;
        uint32_t tailDrops = 0;
        uint8_t droppedSpine = 0;
        uint16_t droppedPathSeq = 0;
        uint64_t droppedSeq = UINT64_MAX;
        // The sends of droppedSeq after its first.
        uint32_t repairs = 0;
        uint32_t trimNotifications = 0;
        // When the first tail repair request reached the sender, when the
        // first repair of droppedSeq left, and the losses then waiting on
        // the guard; the losses still waiting just before and just after the
        // guard of the first request ends.
        uint64_t nackArrivalNs = 0;
        uint64_t repairNs = 0;
        uint32_t waitingAtRepair = 0;
        uint32_t waitingBeforeDue = 0;
        uint32_t waitingAfterDue = 0;
        uint32_t lossesCancelled = 0;
        // The transport events tail_loss_cancelled reported.
        uint32_t cancelledEvents = 0;
        // When the first trim notification reached the sender.
        uint64_t trimArrivalNs = 0;
        uint32_t tailDeclared = 0;
        uint32_t tailIgnored = 0;
        uint32_t closingSends = 0;
        std::vector<Nack> nacks;
        std::vector<uint32_t> lossesTraced = std::vector<uint32_t>(kSpines, 0);
        // Each loss interval the hold traced: its end and its losses.
        std::vector<std::pair<uint64_t, std::vector<uint32_t>>> lossIntervals;
        uint32_t declaredLost = 0;
        uint64_t overtakenNs = 0;
    };

    static void Configure(Ptr<RdmaHw> hw,
                          bool tailRecovery,
                          PathSelectorKind kind,
                          uint64_t delayNs,
                          double tailLossRtts)
    {
        hw->m_sprayBaseRttNs = kBaseRttNs;
        hw->m_sprayTailRecoveryDelayNs = delayNs;
        hw->m_sprayPathSeq = true;
        hw->m_sprayTailRecovery = tailRecovery;
        hw->m_sprayTailLossRtts = tailLossRtts;
        hw->m_pathSelectorKind = static_cast<uint32_t>(kind);
        if (kind == PathSelectorKind::RepsHold)
        {
            hw->m_sprayHoldScope = static_cast<uint32_t>(HoldScope::Host);
            hw->m_sprayLossThreshold1 = 0.1;
            hw->m_sprayLossThreshold2 = 0.25;
            hw->m_sprayLossThreshold3 = 0.5;
            hw->m_sprayLossIntervalNs = kLossIntervalNs;
            hw->m_sprayLossHoldCleanSentinels = 3;
            hw->m_sprayHeldExploration = SpineScores::HeldExploration::Sentinel;
        }
    }

    // One queue pair of packets data packets, one send per kStepNs, to a
    // receiver kOneWayNs away each way, with loss.
    Outcome Flow(bool tailRecovery,
                 Loss loss,
                 PathSelectorKind kind,
                 uint32_t packets,
                 uint64_t delayNs = RdmaHw::kTailDelayBaseRtt,
                 uint64_t trimLateNs = 0,
                 double tailLossRtts = 1)
    {
        SetHeaders(tailRecovery);
        Outcome outcome;
        {
            IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, kSpines);
            IsolatedHost receiver(LoadBalancingMode::SprayUniform, kTestSender, kSpines);
            Configure(sender.hw, tailRecovery, kind, delayNs, tailLossRtts);
            Configure(receiver.hw, tailRecovery, kind, delayNs, tailLossRtts);
            Ptr<RdmaHw> hw = sender.hw;
            hw->m_retransmission_timeout_ns = kTimeoutNs;
            NS_TEST_EXPECT_MSG_EQ(hw->ValidTailRecovery(), true, "a valid configuration");
            NS_TEST_EXPECT_MSG_EQ(hw->ValidPathSeq(), true, "with path sequences");
            hw->m_qpCompleteCallback =
                Callback<void, Ptr<RdmaQueuePair>>([&](Ptr<RdmaQueuePair> done) {
                    outcome.completed = true;
                    outcome.completedNs = Simulator::Now().GetNanoSeconds();
                    outcome.waitingAtCompletion = hw->TailLossesWaiting(done);
                });
            hw->m_transportEventCallback =
                Callback<void, const char*, uint64_t>([&](const char* event, uint64_t) {
                    outcome.cancelledEvents += std::string(event) == "tail_loss_cancelled";
                });
            hw->TraceConnectWithoutContext(
                "RetransmissionTimeout",
                Callback<void, uint64_t, Ptr<RdmaQueuePair>, int32_t, bool, uint64_t>(
                    [&](uint64_t, Ptr<RdmaQueuePair>, int32_t, bool, uint64_t) {
                        outcome.timeouts++;
                    }));
            const uint64_t size = uint64_t{packets} * kMtu;
            Ptr<RdmaQueuePair> qp = sender.AddSender(kTestSender, kTestReceiver, size);
            qp->m_baseRtt = kBaseRttNs;
            qp->m_notifyAppFinish = Callback<void>([]() {});
            if (kind == PathSelectorKind::RepsHold)
            {
                hw->TraceConnectWithoutContext(
                    "SpineLossInterval",
                    Callback<void,
                             uint64_t,
                             const std::vector<uint32_t>&,
                             const std::vector<uint32_t>&,
                             const std::vector<uint8_t>&>(
                        [&](uint64_t endNs,
                            const std::vector<uint32_t>&,
                            const std::vector<uint32_t>& lost,
                            const std::vector<uint8_t>&) {
                            for (uint32_t k = 0; k < kSpines && k < lost.size(); ++k)
                            {
                                outcome.lossesTraced[k] += lost[k];
                            }
                            outcome.lossIntervals.emplace_back(endNs, lost);
                        }));
            }

            // The receiver's answers reach the sender kOneWayNs later.
            receiver.hw->m_nic[0].dev->m_traceEnqueue.ConnectWithoutContext(
                Callback<void, Ptr<const Packet>, uint32_t>([&](Ptr<const Packet> sent, uint32_t) {
                    Ptr<Packet> packet = sent->Copy();
                    const CustomHeader parsed = Parse(packet);
                    if (parsed.l3Prot == kUecTrimRepairProtocol &&
                        !((parsed.ack.flags >> qbbHeader::FLAG_TAIL_NACK) & 1) &&
                        outcome.trimArrivalNs == 0)
                    {
                        outcome.trimArrivalNs = Simulator::Now().GetNanoSeconds() + kOneWayNs;
                    }
                    if (parsed.l3Prot == kUecTrimRepairProtocol &&
                        ((parsed.ack.flags >> qbbHeader::FLAG_TAIL_NACK) & 1))
                    {
                        outcome.nacks.push_back(
                            Nack{parsed.ack.tail_spine, parsed.ack.tail_seq, parsed.ack.tail_count});
                        if (outcome.nackArrivalNs == 0)
                        {
                            outcome.nackArrivalNs = Simulator::Now().GetNanoSeconds() + kOneWayNs;
                            const uint64_t guardNs =
                                static_cast<uint64_t>(tailLossRtts * kBaseRttNs);
                            Simulator::Schedule(NanoSeconds(kOneWayNs + guardNs - 1), [&]() {
                                outcome.waitingBeforeDue = hw->TailLossesWaiting(qp);
                            });
                            Simulator::Schedule(NanoSeconds(kOneWayNs + guardNs + 1), [&]() {
                                outcome.waitingAfterDue = hw->TailLossesWaiting(qp);
                            });
                        }
                    }
                    Simulator::Schedule(NanoSeconds(kOneWayNs), [&sender, packet]() {
                        CustomHeader header = Parse(packet);
                        sender.hw->Receive(packet, header);
                    });
                }));
            auto deliver = [&](Ptr<Packet> packet) {
                CustomHeader header = Parse(packet);
                receiver.hw->Receive(packet, header);
                const Ptr<RdmaRxQueuePair> rx =
                    receiver.hw->GetRxQp(kTestReceiver.Get(),
                                         kTestSender.Get(),
                                         IsolatedHost::kReceiverPort,
                                         IsolatedHost::kSenderPort,
                                         IsolatedHost::kPriorityGroup,
                                         false);
                if (rx && outcome.receivedNs == 0 && rx->ReceiverNextExpectedSeq >= size)
                {
                    outcome.receivedNs = Simulator::Now().GetNanoSeconds();
                }
            };

            Ptr<UniformRandomVariable> draw =
                CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(23));
            std::set<uint64_t> sent;
            std::vector<EventId> inFlight(kSpines);
            std::vector<uint16_t> inFlightPathSeq(kSpines, 0);
            std::vector<uint64_t> inFlightSeq(kSpines, 0);
            std::vector<uint64_t> inFlightSentNs(kSpines, 0);
            std::vector<Ptr<Packet>> inFlightPacket(kSpines);
            std::function<void()> step = [&]() {
                if (outcome.completed)
                {
                    return;
                }
                if (qp->GetBytesLeft() > 0)
                {
                    Ptr<Packet> packet = hw->GetNxtPacket(qp);
                    hw->ArmRetransmissionTimeout(qp);
                    const CustomHeader header = Parse(packet);
                    const uint8_t spine = RequestedSpine(header.ipid);
                    const uint64_t now = Simulator::Now().GetNanoSeconds();
                    const bool fresh = sent.insert(header.udp.seq).second;
                    const bool last = fresh && header.udp.seq + kMtu >= size;
                    if (!fresh && header.udp.seq == outcome.droppedSeq && outcome.repairs++ == 0)
                    {
                        outcome.repairNs = now;
                        outcome.waitingAtRepair = hw->TailLossesWaiting(qp);
                    }
                    outcome.closingSends +=
                        tailRecovery && (header.udp.tail_flags & kTailCloses) != 0;
                    if (last)
                    {
                        outcome.endNs = now;
                        outcome.lastSpine = spine;
                    }
                    bool drop = false;
                    if (loss == Loss::TailOnSpine3 && fresh && !last && spine == kLossy)
                    {
                        const bool tail = header.udp.seq >= size - uint64_t{kTailPackets} * kMtu;
                        drop = draw->GetValue() < 0.01 || tail;
                        outcome.tailDrops += tail;
                    }
                    if (last && loss != Loss::TailOnSpine3)
                    {
                        // The newest packet in flight on another spine is its
                        // spine's last before the end.
                        int32_t newest = -1;
                        for (uint32_t k = 0; k < kSpines; ++k)
                        {
                            if (k != spine && inFlight[k].IsPending() &&
                                (newest < 0 || inFlightSentNs[k] > inFlightSentNs[newest]))
                            {
                                newest = static_cast<int32_t>(k);
                            }
                        }
                        NS_TEST_EXPECT_MSG_GT(newest, -1, "a packet is in flight on another spine");
                        if (newest >= 0)
                        {
                            Simulator::Cancel(inFlight[newest]);
                            outcome.droppedSpine = static_cast<uint8_t>(newest);
                            outcome.droppedPathSeq = inFlightPathSeq[newest];
                            outcome.droppedSeq = inFlightSeq[newest];
                            if (loss == Loss::ClosingPacket)
                            {
                                outcome.drops++;
                            }
                            else if (loss == Loss::Trimmed)
                            {
                                const Ptr<Packet> trimmed = inFlightPacket[newest]->Copy();
                                Trim(trimmed);
                                const uint64_t left = inFlightSentNs[newest] + kOneWayNs - now;
                                Simulator::Schedule(NanoSeconds(left + trimLateNs),
                                                    [&deliver, trimmed]() { deliver(trimmed); });
                            }
                            else
                            {
                                // Still undelivered, so its delivery moves
                                // behind the last packet's.
                                const Ptr<Packet> late = inFlightPacket[newest];
                                outcome.overtakenNs = kOneWayNs + kOvertakeNs;
                                Simulator::Schedule(NanoSeconds(kOneWayNs + kOvertakeNs),
                                                    [&deliver, late]() { deliver(late); });
                            }
                        }
                    }
                    if (drop)
                    {
                        outcome.drops++;
                    }
                    else
                    {
                        const EventId arrival =
                            Simulator::Schedule(NanoSeconds(kOneWayNs), [&deliver, packet]() {
                                deliver(packet);
                            });
                        if (fresh)
                        {
                            inFlight[spine] = arrival;
                            inFlightPathSeq[spine] = header.udp.path_seq;
                            inFlightSeq[spine] = header.udp.seq;
                            inFlightSentNs[spine] = now;
                            inFlightPacket[spine] = packet;
                        }
                    }
                }
                Simulator::Schedule(NanoSeconds(kStepNs), step);
            };
            Simulator::Schedule(NanoSeconds(kStepNs), step);
            Simulator::Stop(NanoSeconds(uint64_t{packets} * kStepNs + 10 * kTimeoutNs));
            Simulator::Run();
            if (kind == PathSelectorKind::RepsHold)
            {
                // A send long after closes the loss intervals counted.
                hw->m_senderSpineScores->CountIntervalSend(
                    0,
                    Simulator::Now().GetNanoSeconds() + 100 * hw->m_sprayLossIntervalNs);
            }
            outcome.qpTimeouts = qp->m_timeouts;
            outcome.tailDeclared = qp->m_tail_declared_lost;
            outcome.tailIgnored = qp->m_tail_nacks_ignored;
            outcome.declaredLost = qp->m_recovery_events;
            outcome.trimNotifications = qp->m_trim_notifications;
            outcome.lossesCancelled = hw->m_tailLossesCancelled;
            Simulator::Destroy();
        }
        return outcome;
    }

    // Test (a): spine 3 loses 1% of its packets and every one among the
    // message's last ones.
    void TailLoss()
    {
        constexpr uint32_t kPackets = 4000;
        const Outcome on = Flow(true, Loss::TailOnSpine3, PathSelectorKind::Ops, kPackets);
        NS_TEST_EXPECT_MSG_EQ(on.completed, true, "on: the flow completes");
        NS_TEST_EXPECT_MSG_NE(static_cast<uint32_t>(on.lastSpine),
                              static_cast<uint32_t>(kLossy),
                              "the last packet goes up another spine");
        NS_TEST_EXPECT_MSG_GT(on.tailDrops, 0u, "spine 3 loses packets at the tail");
        NS_TEST_EXPECT_MSG_GT(on.drops, on.tailDrops, "and before it");
        NS_TEST_EXPECT_MSG_GT(on.closingSends, 0u, "closing packets are sent");
        NS_TEST_EXPECT_MSG_EQ(on.timeouts, 0u, "on: no retransmission timeout is traced");
        NS_TEST_EXPECT_MSG_EQ(on.qpTimeouts, 0u, "nor counted");
        NS_TEST_EXPECT_MSG_GT(on.nacks.size(), 0u, "the receiver asks for the tail");
        for (const Nack& nack : on.nacks)
        {
            NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(nack.spine),
                                  static_cast<uint32_t>(kLossy),
                                  "only spine 3 misses packets");
        }
        NS_TEST_EXPECT_MSG_GT_OR_EQ(on.tailDeclared, on.tailDrops, "every tail loss is repaired");
        NS_TEST_EXPECT_MSG_LT_OR_EQ(on.tailDeclared, on.drops, "and nothing that arrived");
        NS_TEST_EXPECT_MSG_GT(on.receivedNs, on.endNs, "received after the end");
        NS_TEST_EXPECT_MSG_LT_OR_EQ(on.receivedNs - on.endNs,
                                    kBaseRttNs * 3 / 2 + kDelayNs + (on.tailDeclared + 1) * kStepNs,
                                    "within 1.5 base RTT and the delay of the end, and the "
                                    "repairs' sends");

        const Outcome off = Flow(false, Loss::TailOnSpine3, PathSelectorKind::Ops, kPackets);
        NS_TEST_EXPECT_MSG_EQ(off.tailDrops, on.tailDrops, "off: the same tail losses");
        NS_TEST_EXPECT_MSG_EQ(off.completed, true, "off: the flow completes");
        NS_TEST_EXPECT_MSG_GT(off.timeouts, 0u, "off: a retransmission timeout is traced");
        NS_TEST_EXPECT_MSG_EQ(off.qpTimeouts, off.timeouts, "and counted");
        NS_TEST_EXPECT_MSG_EQ(off.nacks.size(), 0u, "off: no tail repair request");
        NS_TEST_EXPECT_MSG_GT(off.receivedNs, off.endNs + kTimeoutNs - kTailPackets * kStepNs,
                              "off: the tail waits out the timeout");
    }

    // Test (b): spine k's last packet before the end is lost, and only the
    // last packet's counts show it.
    void LostClosingPacket()
    {
        constexpr uint32_t kPackets = 400;
        const Outcome on = Flow(true, Loss::ClosingPacket, PathSelectorKind::RepsHold, kPackets);
        NS_TEST_EXPECT_MSG_EQ(on.completed, true, "on: the flow completes");
        NS_TEST_EXPECT_MSG_EQ(on.drops, 1u, "one packet lost");
        NS_TEST_EXPECT_MSG_EQ(on.timeouts, 0u, "on: no retransmission timeout");
        NS_TEST_EXPECT_MSG_EQ(on.nacks.size(), 1u, "one tail repair request");
        if (!on.nacks.empty())
        {
            NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(on.nacks[0].spine),
                                  static_cast<uint32_t>(on.droppedSpine),
                                  "for the spine that lost it");
            NS_TEST_EXPECT_MSG_EQ(on.nacks[0].first, on.droppedPathSeq, "its path sequence");
            NS_TEST_EXPECT_MSG_EQ(on.nacks[0].count, 1u, "alone");
        }
        NS_TEST_EXPECT_MSG_EQ(on.tailDeclared, 1u, "declared lost by the request");
        NS_TEST_EXPECT_MSG_EQ(on.tailIgnored, 0u, "which names an outstanding send");
        NS_TEST_EXPECT_MSG_EQ(on.lossesTraced[on.droppedSpine],
                              1u,
                              "the hold's loss trace counts it on its spine");
        NS_TEST_EXPECT_MSG_EQ(on.repairs, 1u, "it is repaired once");
        NS_TEST_EXPECT_MSG_LT_OR_EQ(on.repairNs - on.nackArrivalNs,
                                    kStepNs,
                                    "at once on the request");
        NS_TEST_EXPECT_MSG_EQ(on.waitingAtRepair, 1u, "while its loss waits on the guard");
        NS_TEST_EXPECT_MSG_EQ(on.waitingBeforeDue, 1u, "until the guard ends");
        NS_TEST_EXPECT_MSG_EQ(on.waitingAfterDue, 0u, "and is told then");
        NS_TEST_EXPECT_MSG_EQ(on.lossesCancelled, 0u, "no trim drops it");
        NS_TEST_EXPECT_MSG_LT_OR_EQ(on.receivedNs - on.endNs,
                                    kBaseRttNs * 3 / 2 + kDelayNs + kStepNs,
                                    "within 1.5 base RTT and the delay of the end");
        NS_TEST_EXPECT_MSG_GT(on.receivedNs - on.endNs,
                              kBaseRttNs * 3 / 2 + kDelayNs - kStepNs,
                              "the request waits out the delay");

        const Outcome off = Flow(false, Loss::ClosingPacket, PathSelectorKind::RepsHold, kPackets);
        NS_TEST_EXPECT_MSG_EQ(off.completed, true, "off: the flow completes");
        NS_TEST_EXPECT_MSG_GT(off.timeouts, 0u, "off: the loss waits out the timeout");
    }

    // The trimmed header of spine k's last packet before the end reaches the
    // receiver after its request for the packet: the request declares it
    // lost and it is repaired at once, and its trim notification, inside the
    // guard, keeps the loss from the selector.
    void TrimmedAfterRequest()
    {
        constexpr uint32_t kPackets = 400;
        constexpr uint64_t kLateNs = 4000;
        const Outcome on = Flow(true,
                                Loss::Trimmed,
                                PathSelectorKind::RepsHold,
                                kPackets,
                                RdmaHw::kTailDelayBaseRtt,
                                kLateNs);
        NS_TEST_EXPECT_MSG_EQ(on.completed, true, "the flow completes");
        NS_TEST_EXPECT_MSG_EQ(on.timeouts, 0u, "without a retransmission timeout");
        NS_TEST_EXPECT_MSG_EQ(on.nacks.size(), 1u, "the trimmed packet is requested");
        if (!on.nacks.empty())
        {
            NS_TEST_EXPECT_MSG_EQ(static_cast<uint32_t>(on.nacks[0].spine),
                                  static_cast<uint32_t>(on.droppedSpine),
                                  "on its spine");
            NS_TEST_EXPECT_MSG_EQ(on.nacks[0].first, on.droppedPathSeq, "by its path sequence");
            NS_TEST_EXPECT_MSG_EQ(on.nacks[0].count, 1u, "alone");
        }
        NS_TEST_EXPECT_MSG_EQ(on.tailDeclared, 1u, "declared lost by the request");
        NS_TEST_EXPECT_MSG_EQ(on.declaredLost, 1u, "once");
        NS_TEST_EXPECT_MSG_EQ(on.repairs, 1u, "repaired once");
        NS_TEST_EXPECT_MSG_LT_OR_EQ(on.repairNs - on.nackArrivalNs,
                                    kStepNs,
                                    "at once on the request");
        NS_TEST_EXPECT_MSG_EQ(on.waitingAtRepair, 1u, "while its loss waits on the guard");
        NS_TEST_EXPECT_MSG_EQ(on.trimNotifications, 1u, "its trim notification arrives");
        NS_TEST_EXPECT_MSG_EQ(on.lossesCancelled, 1u, "inside the guard, and drops the loss");
        NS_TEST_EXPECT_MSG_EQ(on.waitingBeforeDue, 0u, "before the guard ends");
        NS_TEST_EXPECT_MSG_EQ(on.lossesTraced[on.droppedSpine],
                              0u,
                              "no loss reaches the selector on its spine");
    }

    // The same trimmed packet with a guard of three base RTTs, its header
    // late enough that the queue pair completes, its repair acknowledged,
    // before the trim notification arrives: the loss waits on the host past
    // the completion, and the trim notification still drops it.
    void TrimmedAfterCompletion()
    {
        constexpr uint32_t kPackets = 400;
        constexpr uint64_t kLateNs = 6000;
        constexpr double kRtts = 3;
        const uint64_t guardNs = static_cast<uint64_t>(kRtts * kBaseRttNs);
        const Outcome on = Flow(true,
                                Loss::Trimmed,
                                PathSelectorKind::RepsHold,
                                kPackets,
                                RdmaHw::kTailDelayBaseRtt,
                                kLateNs,
                                kRtts);
        NS_TEST_EXPECT_MSG_EQ(on.completed, true, "the flow completes");
        NS_TEST_EXPECT_MSG_EQ(on.timeouts, 0u, "without a retransmission timeout");
        NS_TEST_EXPECT_MSG_EQ(on.nacks.size(), 1u, "the trimmed packet is requested");
        NS_TEST_EXPECT_MSG_EQ(on.tailDeclared, 1u, "declared lost by the request");
        NS_TEST_EXPECT_MSG_EQ(on.repairs, 1u, "repaired once");
        NS_TEST_EXPECT_MSG_EQ(on.waitingAtCompletion,
                              1u,
                              "its loss waits as the queue pair completes");
        NS_TEST_EXPECT_MSG_GT(on.trimArrivalNs,
                              on.completedNs,
                              "the trim notification arrives after");
        NS_TEST_EXPECT_MSG_LT(on.trimArrivalNs, on.nackArrivalNs + guardNs, "inside the guard");
        NS_TEST_EXPECT_MSG_EQ(on.trimNotifications, 0u, "for no open queue pair");
        NS_TEST_EXPECT_MSG_EQ(on.lossesCancelled, 1u, "and drops the loss all the same");
        NS_TEST_EXPECT_MSG_EQ(on.cancelledEvents, 1u, "reported as tail_loss_cancelled");
        NS_TEST_EXPECT_MSG_EQ(on.waitingBeforeDue, 0u, "before the guard ends");
        NS_TEST_EXPECT_MSG_EQ(on.lossesTraced[on.droppedSpine],
                              0u,
                              "no loss reaches the hold on its spine");
    }

    // A silently lost closing packet with a guard of two base RTTs: its
    // repair is acknowledged and the queue pair completes before the guard
    // ends, and the loss is told once, when the guard ends, at the time it is
    // told.
    void LostAfterCompletion()
    {
        constexpr uint32_t kPackets = 400;
        constexpr double kRtts = 2;
        const uint64_t guardNs = static_cast<uint64_t>(kRtts * kBaseRttNs);
        const Outcome on = Flow(true,
                                Loss::ClosingPacket,
                                PathSelectorKind::RepsHold,
                                kPackets,
                                RdmaHw::kTailDelayBaseRtt,
                                0,
                                kRtts);
        NS_TEST_EXPECT_MSG_EQ(on.completed, true, "the flow completes");
        NS_TEST_EXPECT_MSG_EQ(on.tailDeclared, 1u, "declared lost by the request");
        NS_TEST_EXPECT_MSG_EQ(on.repairs, 1u, "repaired once");
        NS_TEST_EXPECT_MSG_LT(on.completedNs,
                              on.nackArrivalNs + guardNs,
                              "the queue pair completes inside the guard");
        NS_TEST_EXPECT_MSG_EQ(on.waitingAtCompletion, 1u, "with its loss waiting");
        NS_TEST_EXPECT_MSG_EQ(on.waitingBeforeDue, 1u, "until the guard ends");
        NS_TEST_EXPECT_MSG_EQ(on.waitingAfterDue, 0u, "and told then");
        NS_TEST_EXPECT_MSG_EQ(on.lossesCancelled, 0u, "no trim drops it");
        NS_TEST_EXPECT_MSG_EQ(on.cancelledEvents, 0u, "nor is reported dropped");
        NS_TEST_EXPECT_MSG_EQ(on.lossesTraced[on.droppedSpine],
                              1u,
                              "the hold counts it once on its spine");
        const uint64_t reportNs = on.nackArrivalNs + guardNs;
        uint32_t rows = 0;
        uint64_t lastEndNs = 0;
        for (const auto& [endNs, lost] : on.lossIntervals)
        {
            NS_TEST_EXPECT_MSG_EQ(endNs % kLossIntervalNs, 0u, "every interval ends on the grid");
            NS_TEST_EXPECT_MSG_GT(endNs, lastEndNs, "after the one before");
            lastEndNs = endNs;
            if (lost[on.droppedSpine] == 0)
            {
                continue;
            }
            rows++;
            NS_TEST_EXPECT_MSG_GT(endNs, reportNs, "the loss's interval ends after the report");
            NS_TEST_EXPECT_MSG_LT_OR_EQ(endNs - kLossIntervalNs,
                                        reportNs,
                                        "and is the interval of the report");
        }
        NS_TEST_EXPECT_MSG_EQ(rows, 1u, "one interval counts the loss");
    }

    // With SprayTailLossRtts 0 the selector is told of a send a tail repair
    // request declares lost at once, trimmed or not.
    void LossReportedAtOnce()
    {
        constexpr uint32_t kPackets = 400;
        const Outcome trimmed = Flow(true,
                                     Loss::Trimmed,
                                     PathSelectorKind::RepsHold,
                                     kPackets,
                                     RdmaHw::kTailDelayBaseRtt,
                                     4000,
                                     0);
        NS_TEST_EXPECT_MSG_EQ(trimmed.completed, true, "trimmed: the flow completes");
        NS_TEST_EXPECT_MSG_EQ(trimmed.tailDeclared, 1u, "trimmed: declared lost by the request");
        NS_TEST_EXPECT_MSG_EQ(trimmed.repairs, 1u, "trimmed: repaired once");
        NS_TEST_EXPECT_MSG_EQ(trimmed.waitingAtRepair, 0u, "trimmed: no loss waits");
        NS_TEST_EXPECT_MSG_EQ(trimmed.lossesCancelled, 0u, "trimmed: none is dropped");
        NS_TEST_EXPECT_MSG_EQ(trimmed.lossesTraced[trimmed.droppedSpine],
                              1u,
                              "trimmed: the loss reaches the selector");

        const Outcome lost = Flow(true,
                                  Loss::ClosingPacket,
                                  PathSelectorKind::RepsHold,
                                  kPackets,
                                  RdmaHw::kTailDelayBaseRtt,
                                  0,
                                  0);
        NS_TEST_EXPECT_MSG_EQ(lost.completed, true, "lost: the flow completes");
        NS_TEST_EXPECT_MSG_EQ(lost.tailDeclared, 1u, "lost: declared lost by the request");
        NS_TEST_EXPECT_MSG_EQ(lost.repairs, 1u, "lost: repaired once");
        NS_TEST_EXPECT_MSG_EQ(lost.waitingAtRepair, 0u, "lost: told at once");
        NS_TEST_EXPECT_MSG_EQ(lost.lossesTraced[lost.droppedSpine],
                              1u,
                              "lost: the loss reaches the selector");
    }

    // The last packet overtakes the last one of a slower spine by less than
    // the delay: nothing is requested and nothing declared lost.
    void OvertakenClosingPacket()
    {
        constexpr uint32_t kPackets = 400;
        const Outcome on = Flow(true, Loss::Overtaken, PathSelectorKind::RepsHold, kPackets);
        NS_TEST_EXPECT_MSG_GT(on.overtakenNs, 0u, "a packet is overtaken");
        NS_TEST_EXPECT_MSG_LT(kOvertakeNs, kDelayNs, "by less than the delay");
        NS_TEST_EXPECT_MSG_EQ(on.completed, true, "the flow completes");
        NS_TEST_EXPECT_MSG_EQ(on.drops, 0u, "nothing is lost");
        NS_TEST_EXPECT_MSG_EQ(on.nacks.size(), 0u, "no tail repair request");
        NS_TEST_EXPECT_MSG_EQ(on.tailDeclared, 0u, "no send declared lost by one");
        NS_TEST_EXPECT_MSG_EQ(on.declaredLost, 0u, "nor at all");
        NS_TEST_EXPECT_MSG_EQ(on.lossesTraced[on.droppedSpine], 0u, "nor traced as a loss");
        NS_TEST_EXPECT_MSG_EQ(on.timeouts, 0u, "and no retransmission timeout");

        // Requested at once, the same packet is asked for before it arrives.
        const Outcome atOnce = Flow(true, Loss::Overtaken, PathSelectorKind::RepsHold, kPackets, 0);
        NS_TEST_EXPECT_MSG_EQ(atOnce.completed, true, "at once: the flow completes");
        NS_TEST_EXPECT_MSG_EQ(atOnce.nacks.size(), 1u, "at once: the packet is requested");
        NS_TEST_EXPECT_MSG_EQ(atOnce.tailDeclared, 1u, "and declared lost");
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
        // Three intervals, eight packets per spine each and eight more on
        // spine 5 trimmed, then one packet.
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
                for (uint32_t j = 0; j < 8; ++j)
                {
                    Trim(receiver, seq, 5);
                    seq += IsolatedHost::kMtu;
                }
            });
        }
        Simulator::Schedule(NanoSeconds(phase + 3 * interval + 1), [&]() { deliver(0, false); });
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(receiver.emitted.size(), 3 * 72 + 1, "every packet is answered");
        const CustomHeader& last = receiver.emitted.back();
        NS_TEST_EXPECT_MSG_EQ(last.l3Prot, 0xFC, "the last answer is an acknowledgement");
        const SpineReport& report = receiver.hw->Grader().Report();
        NS_TEST_EXPECT_MSG_EQ(uint32_t(report.sequence), 3u, "three intervals closed");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(2)), 0u, "the marked spine is graded down");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(report.Grade(5)),
                              1u,
                              "the spine trimming half its packets grades by its trimmed fraction");
        NS_TEST_EXPECT_MSG_EQ(receiver.hw->Grader().LastInterval()[5].held,
                              false,
                              "and is not held");
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
        const CustomHeader& repair = receiver.emitted[72 + 64];
        NS_TEST_EXPECT_MSG_EQ(repair.l3Prot, kUecTrimRepairProtocol, "the trim is answered");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(repair.ack.spine_report[0]),
                              1u,
                              "the repair request carries the latest report");
        std::copy_n(repair.ack.spine_report + 1, 2, wire.grades);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(wire.Grade(2)), 0u, "with the marked spine graded down");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(wire.Grade(5)),
                              3u,
                              "and the trimmed spine as one interval of trims left it");
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

class SprayPolicyDefaultsTest : public TestCase
{
  public:
    SprayPolicyDefaultsTest()
        : TestCase("spray_policy's defaults: a report per two base RTTs, the median reference, "
                   "intervals of 32 packets setting an average, gamma 0.25, epsilon 0.02, one "
                   "proportional candidate")
    {
    }

    void DoRun() override
    {
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        NS_TEST_EXPECT_MSG_EQ(hw->m_sprayReportIntervalBaseRtts, 2.0, "two base RTTs");
        NS_TEST_EXPECT_MSG_EQ(hw->m_sprayGradeReference,
                              static_cast<uint32_t>(SpineGrader::GradeReference::Median),
                              "the median reference");
        NS_TEST_EXPECT_MSG_EQ(hw->m_sprayEstimatorIntervalSamples, 32, "32 packets");
        NS_TEST_EXPECT_MSG_EQ(hw->m_sprayGamma, 0.25, "gamma 0.25");
        NS_TEST_EXPECT_MSG_EQ(hw->m_sprayEpsilon, 0.02, "epsilon 0.02");
        NS_TEST_EXPECT_MSG_EQ(hw->m_sprayCandidates, 1, "one candidate");
        NS_TEST_EXPECT_MSG_EQ(hw->m_sprayCandidateDraw,
                              static_cast<uint32_t>(SpineScores::CandidateDraw::Proportional),
                              "drawn in proportion");
        NS_TEST_EXPECT_MSG_EQ(hw->m_sprayOneWayDelay, false, "no one-way delay");
        NS_TEST_EXPECT_MSG_EQ(hw->m_sprayGradeScope,
                              static_cast<uint32_t>(GradeScope::PerReceiver),
                              "one grader per receiving host");
    }
};

class SpineRedirectTest : public TestCase
{
  public:
    SpineRedirectTest()
        : TestCase("A receive queue pair scores each arrival by its one-way delay and mark, keeps "
                   "the best spine for a validity cycle and redirects past the flag")
    {
    }

    void DoRun() override
    {
        Score();
        BestAndCycle();
        Flag();
    }

  private:
    static constexpr uint64_t kBase = 6000;
    static constexpr uint64_t kCycle = 20000;

    static SpineRedirect::Parameters Defaults(double flag = 0.1)
    {
        return SpineRedirect::Parameters{0.5, flag, kCycle, kBase};
    }

    // omega / (L / L0 + 0.001) + (1 - omega)(1 - m), worked by hand at the
    // base delay and twice it.
    void Score()
    {
        struct Case
        {
            double omega;
            uint64_t delayNs;
            bool marked;
            double score;
        };
        const Case cases[] = {
            {0.3, kBase, false, 0.9997002997002997},
            {0.3, kBase, true, 0.2997002997002997},
            {0.3, 2 * kBase, false, 0.8499250374812594},
            {0.3, 2 * kBase, true, 0.1499250374812594},
            {0.5, kBase, false, 0.9995004995004995},
            {0.5, kBase, true, 0.4995004995004995},
            {0.5, 2 * kBase, false, 0.7498750624687656},
            {0.5, 2 * kBase, true, 0.2498750624687656},
            {0.7, kBase, false, 0.9993006993006993},
            {0.7, kBase, true, 0.6993006993006993},
            {0.7, 2 * kBase, false, 0.6498250874562719},
            {0.7, 2 * kBase, true, 0.3498250874562719},
        };
        for (const Case& c : cases)
        {
            NS_TEST_EXPECT_MSG_EQ_TOL(SpineRedirect::Score(c.omega, c.delayNs, kBase, c.marked),
                                      c.score,
                                      1e-12,
                                      "equation 1 at omega " << c.omega);
        }
        NS_TEST_EXPECT_MSG_EQ_TOL(SpineRedirect::Score(0.5, 0, kBase, false),
                                  0.5 / 0.001 + 0.5,
                                  1e-9,
                                  "no delay at all is bounded by epsilon");
    }

    void BestAndCycle()
    {
        const SpineRedirect::Parameters parameters = Defaults();
        SpineRedirect redirect;
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.Best()), uint32_t(kNoRedirect), "no best at first");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.OnArrival(4, 2 * kBase, false, 1000, parameters)),
                              uint32_t(kNoRedirect),
                              "the first arrival starts a cycle and redirects nothing");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.Best()), 4u, "its spine is the best");
        NS_TEST_EXPECT_MSG_EQ(redirect.CycleEndNs(), 1000 + kCycle, "for one validity cycle");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.OnArrival(6, kBase, false, 2000, parameters)),
                              uint32_t(kNoRedirect),
                              "a higher score redirects nothing");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.Best()), 6u, "and replaces the best");
        NS_TEST_EXPECT_MSG_EQ(redirect.CycleEndNs(), 1000 + kCycle, "within the same cycle");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.OnArrival(6, 8 * kBase, true, 3000, parameters)),
                              uint32_t(kNoRedirect),
                              "an arrival on the best redirects nothing, however low it scores");
        NS_TEST_EXPECT_MSG_EQ_TOL(redirect.BestScore(),
                                  SpineRedirect::Score(0.5, 8 * kBase, kBase, true),
                                  1e-12,
                                  "and refreshes the best's score");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.OnArrival(2, 4 * kBase, true, 4000, parameters)),
                              uint32_t(kNoRedirect),
                              "a score above the refreshed best's replaces it");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.Best()), 2u, "so spine 2 is the best");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.OnArrival(5,
                                                          8 * kBase,
                                                          true,
                                                          1000 + kCycle - 1,
                                                          parameters)),
                              uint32_t(2),
                              "just before the cycle ends a low score is redirected to it");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.OnArrival(5,
                                                          8 * kBase,
                                                          true,
                                                          1000 + kCycle,
                                                          parameters)),
                              uint32_t(kNoRedirect),
                              "the first arrival at the cycle's end starts a new cycle");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.Best()), 5u, "with its own spine as the best");
        NS_TEST_EXPECT_MSG_EQ(redirect.CycleEndNs(), 1000 + 2 * kCycle, "for a cycle from then");
    }

    // An arrival scoring a fraction g below the best is redirected when the
    // flag is below g and not when it is above.
    void Flag()
    {
        const double best = SpineRedirect::Score(0.5, kBase, kBase, false);
        const double worse = SpineRedirect::Score(0.5, 3 * kBase / 2, kBase, false);
        const double gap = (best - worse) / best;
        auto redirectAt = [&](double flag) {
            SpineRedirect redirect;
            redirect.OnArrival(1, kBase, false, 0, Defaults(flag));
            return redirect.OnArrival(3, 3 * kBase / 2, false, 10, Defaults(flag));
        };
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirectAt(gap - 1e-9)),
                              1u,
                              "a gap just over the flag redirects to the best");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirectAt(gap + 1e-9)),
                              uint32_t(kNoRedirect),
                              "a gap just under it does not");
        NS_TEST_EXPECT_MSG_GT(gap, 0.1, "half again the delay is past the default flag");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirectAt(0.1)), 1u, "so the default redirects it");
    }
};

class AckRedirectSelectorTest : public TestCase
{
  public:
    AckRedirectSelectorTest()
        : TestCase("ack_redirect sends on the latest acknowledgement's redirect, else on the "
                   "spine its send crossed, else on a uniform draw")
    {
    }

    void DoRun() override
    {
        Ptr<UniformRandomVariable> drawn =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(0));
        Ptr<UniformRandomVariable> reference =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(0));
        AckRedirectSelector selector(drawn, 8);
        UniformSpineSelector uniform(reference, 8);
        bool alike = true;
        for (uint32_t i = 0; i < 100; ++i)
        {
            alike &= selector.Choose(0) == uniform.Choose(0);
        }
        NS_TEST_EXPECT_MSG_EQ(alike, true, "before any acknowledgement each spine is drawn");
        selector.OnRedirect(kNoRedirect, 5, 10);
        NS_TEST_EXPECT_MSG_EQ(selector.Choose(20),
                              SpineIdentification(5, 5),
                              "without a redirect, the spine the answered send crossed");
        selector.OnRedirect(3, 5, 30);
        NS_TEST_EXPECT_MSG_EQ(selector.Choose(40),
                              SpineIdentification(3, 3),
                              "with one, the spine it names");
        NS_TEST_EXPECT_MSG_EQ(selector.Choose(50), SpineIdentification(3, 3), "until the next");
        selector.OnRedirect(kNoRedirect, 6, 60);
        NS_TEST_EXPECT_MSG_EQ(selector.Choose(70),
                              SpineIdentification(6, 6),
                              "the latest acknowledgement wins");
        NS_TEST_EXPECT_MSG_EQ(drawn->GetInteger(0, 1000),
                              reference->GetInteger(0, 1000),
                              "following acknowledgements draws nothing");
    }
};

class AckRedirectTransportTest : public TestCase
{
  public:
    AckRedirectTransportTest()
        : TestCase("Under ack_redirect every acknowledgement carries its receive queue pair's "
                   "redirect after the packet sequence, and the sender follows it")
    {
    }

    void DoRun() override
    {
        const IntHeader::Mode savedIntMode = IntHeader::mode;
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        const uint32_t savedRedirectBytes = CustomHeader::ackRedirectBytes;
        IntHeader::mode = IntHeader::NONE;
        CustomHeader::ackCarriesPacketSeq = true;
        Wire();
        IntHeader::mode = IntHeader::TS;
        CustomHeader::ackRedirectBytes = 1;
        Receiver(TrimScoring::Mark);
        Receiver(TrimScoring::Ignore);
        Sender();
        CustomHeader::ackRedirectBytes = savedRedirectBytes;
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        IntHeader::mode = savedIntMode;
    }

  private:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint64_t kBaseRtt = 10000;
    static constexpr uint64_t kBase = kBaseRtt / 2;

    // A serialized acknowledgement's bytes from the start of its header.
    static std::vector<uint8_t> Bytes(const qbbHeader& ack)
    {
        Ptr<Packet> packet = Create<Packet>(0);
        packet->AddHeader(ack);
        std::vector<uint8_t> bytes(packet->GetSize());
        packet->CopyData(bytes.data(), bytes.size());
        return bytes;
    }

    void Wire()
    {
        CustomHeader::ackRedirectBytes = 0;
        qbbHeader plain;
        plain.SetSeq(3000);
        plain.SetPacketSeq(7000);
        qbbHeader redirected = plain;
        redirected.SetRedirect(4);
        NS_TEST_EXPECT_MSG_EQ(plain.GetSerializedSize(), 20, "without the field 20 bytes");
        NS_TEST_EXPECT_MSG_EQ((Bytes(plain) == Bytes(redirected)),
                              true,
                              "and a redirect set outside ack_redirect is not on the wire");

        CustomHeader::ackRedirectBytes = 1;
        NS_TEST_EXPECT_MSG_EQ(redirected.GetSerializedSize(), 21, "the field adds one byte");
        NS_TEST_EXPECT_MSG_EQ(CustomHeader::GetAckSerializedSize(), 21, "the parser agrees");
        const std::vector<uint8_t> bytes = Bytes(redirected);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(bytes[20]), 4u, "the redirect is at offset 20");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(Bytes(plain)[20]),
                              uint32_t(kNoRedirect),
                              "and names no spine unless set");
        Ptr<Packet> packet = Create<Packet>(0);
        packet->AddHeader(redirected);
        Ipv4Header ip;
        ip.SetProtocol(0xFC);
        ip.SetPayloadSize(packet->GetSize());
        packet->AddHeader(ip);
        PppHeader ppp;
        ppp.SetProtocol(0x0021);
        packet->AddHeader(ppp);
        CustomHeader parsed(CustomHeader::L2_Header | CustomHeader::L3_Header |
                            CustomHeader::L4_Header);
        packet->PeekHeader(parsed);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(parsed.ack.redirect), 4u, "the redirect is parsed");
        NS_TEST_EXPECT_MSG_EQ(parsed.ack.packet_seq, 7000, "the packet sequence keeps its place");
        qbbHeader copy;
        packet->RemoveHeader(ppp);
        packet->RemoveHeader(ip);
        packet->PeekHeader(copy);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(copy.GetRedirect()), 4u, "the header reads it back");

        NS_TEST_EXPECT_MSG_EQ(uint32_t(DecodeRedirect(0, kSpines)), 0u, "spine 0 is a spine");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(DecodeRedirect(kSpines - 1, kSpines)),
                              kSpines - 1,
                              "and so is the last");
        for (uint32_t field = kSpines; field <= kNoRedirect; ++field)
        {
            NS_TEST_EXPECT_MSG_EQ(uint32_t(DecodeRedirect(field, kSpines)),
                                  uint32_t(kNoRedirect),
                                  "no value from the spine count up names a spine");
        }

        // An answer still fits the minimum frame without a timestamp.
        IsolatedHost receiver(LoadBalancingMode::SprayUniform, kTestSender, kSpines);
        receiver.ReceiveData(kTestSender, kTestReceiver, 0, SpineIdentification(1, 1));
        NS_TEST_ASSERT_MSG_EQ(receiver.emittedSizes.size(), 1, "the packet is answered");
        NS_TEST_EXPECT_MSG_EQ(receiver.emittedSizes[0],
                              PppHeader().GetSerializedSize() + 20 + (60 - 14 - 20),
                              "in a minimum frame");
        CustomHeader::ackRedirectBytes = 0;
        Simulator::Destroy();
    }

    static void Redirect(IsolatedHost& host, TrimScoring trims)
    {
        host.hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::AckRedirect);
        host.hw->m_sprayBaseRttNs = kBaseRtt;
        host.hw->m_ackRedirectTrimScoring = static_cast<uint32_t>(trims);
    }

    // A data packet trimmed on spine, before the last hop or at it, sent
    // delayNs ago.
    static void Trim(IsolatedHost& receiver,
                     uint32_t seq,
                     uint8_t spine,
                     bool lastHop,
                     uint64_t delayNs)
    {
        CustomHeader trimmed;
        trimmed.l3Prot = 0x11;
        trimmed.m_tos = (lastHop ? kUetDscpTrimmedLastHop : kUetDscpTrimmed) << 2;
        trimmed.sip = kTestSender.Get();
        trimmed.dip = kTestReceiver.Get();
        trimmed.ipid = SpineIdentification(spine, spine);
        trimmed.udp.sport = IsolatedHost::kSenderPort;
        trimmed.udp.dport = IsolatedHost::kReceiverPort;
        trimmed.udp.pg = IsolatedHost::kPriorityGroup;
        trimmed.udp.seq = seq;
        trimmed.udp.payload_size = CustomHeader::GetUdpHeaderSize() + IsolatedHost::kMtu;
        trimmed.udp.ih.ts = (Simulator::Now() - NanoSeconds(delayNs)).GetTimeStep();
        receiver.hw->Receive(Create<Packet>(), trimmed);
    }

    void Receiver(TrimScoring trims)
    {
        IsolatedHost receiver(LoadBalancingMode::SprayUniform, kTestSender, kSpines);
        Redirect(receiver, trims);
        uint32_t seq = 0;
        auto deliver = [&](uint16_t identification, uint64_t delayNs, bool marked = false) {
            receiver.ReceiveData(kTestSender,
                                 kTestReceiver,
                                 seq,
                                 identification,
                                 marked,
                                 IsolatedHost::kSenderPort,
                                 delayNs);
            seq += IsolatedHost::kMtu;
            return uint32_t(receiver.emitted.back().ack.redirect);
        };
        auto state = [&]() {
            return receiver.hw->GetRxQp(kTestReceiver.Get(),
                                        kTestSender.Get(),
                                        IsolatedHost::kReceiverPort,
                                        IsolatedHost::kSenderPort,
                                        IsolatedHost::kPriorityGroup,
                                        false)
                ->m_spineRedirect;
        };
        const uint32_t none = kNoRedirect;
        Simulator::Schedule(NanoSeconds(100000), [&]() {
            NS_TEST_EXPECT_MSG_EQ(deliver(SpineIdentification(1, 1), kBase), none, "a cycle starts");
            NS_TEST_EXPECT_MSG_EQ(deliver(SpineIdentification(2, 2), 3 * kBase),
                                  1u,
                                  "a slow spine's answer redirects to the best");
            NS_TEST_EXPECT_MSG_EQ(deliver(SpineIdentification(1, 1), kBase),
                                  none,
                                  "an answer on the best does not");
            NS_TEST_EXPECT_MSG_EQ(deliver(SpineIdentification(4, 4), kBase, true),
                                  1u,
                                  "a marked answer on another spine redirects to the best");
            // Moved by its leaf from spine 1 to spine 6, so scored as spine 6.
            NS_TEST_EXPECT_MSG_EQ(deliver(SpineIdentification(1, 6), kBase / 2),
                                  none,
                                  "a packet moved off its spine is scored on the one it crossed");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(state().Best()), 6u, "and replaces the best");
            const uint64_t cycleEnd = state().CycleEndNs();
            const double best = state().BestScore();
            receiver.hw->m_ownLeafHosts = {kTestSender.Get()};
            NS_TEST_EXPECT_MSG_EQ(deliver(SpineIdentification(3, 3), 10 * kBase, true),
                                  none,
                                  "a sender on this leaf crossed no spine and is not redirected");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(state().Best()), 6u, "nor scored");
            NS_TEST_EXPECT_MSG_EQ(state().BestScore(), best, "the best keeps its score");
            receiver.hw->m_ownLeafHosts.clear();
            Trim(receiver, seq, 5, false, 10 * kBase);
            seq += IsolatedHost::kMtu;
            const CustomHeader& repair = receiver.emitted.back();
            NS_TEST_EXPECT_MSG_EQ(repair.l3Prot, kUecTrimRepairProtocol, "a trim is answered");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(repair.ack.redirect), none, "with no redirect");
            NS_TEST_EXPECT_MSG_EQ(state().CycleEndNs(), cycleEnd, "within the cycle");
        });
        // The first event of the next cycle is a trim, which starts it only
        // if trims are scored, and a last-hop trim never.
        Simulator::Schedule(NanoSeconds(100000 + 2 * kBaseRtt), [&]() {
            Trim(receiver, seq, 7, true, 0);
            seq += IsolatedHost::kMtu;
            NS_TEST_EXPECT_MSG_EQ(uint32_t(state().Best()), 6u, "a last-hop trim is not scored");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(receiver.emitted.back().ack.redirect),
                                  none,
                                  "and its repair request carries no redirect");
            Trim(receiver, seq, 5, false, 0);
            seq += IsolatedHost::kMtu;
            NS_TEST_EXPECT_MSG_EQ(uint32_t(receiver.emitted.back().ack.redirect),
                                  none,
                                  "a repair request carries no redirect");
            if (trims == TrimScoring::Mark)
            {
                NS_TEST_EXPECT_MSG_EQ(uint32_t(state().Best()),
                                      5u,
                                      "a trim before the last hop is an arrival on its spine");
                NS_TEST_EXPECT_MSG_EQ_TOL(state().BestScore(),
                                          SpineRedirect::Score(0.5, 0, kBase, true),
                                          1e-9,
                                          "marked");
            }
            else
            {
                NS_TEST_EXPECT_MSG_EQ(uint32_t(state().Best()), 6u, "or is ignored");
            }
        });
        Simulator::Run();
        Simulator::Destroy();
    }

    void Sender()
    {
        IsolatedHost sender(LoadBalancingMode::SprayUniform, kTestReceiver, kSpines);
        Redirect(sender, TrimScoring::Mark);
        Ptr<RdmaQueuePair> qp =
            sender.AddSender(kTestSender, kTestReceiver, 100 * IsolatedHost::kMtu);
        Ptr<UniformRandomVariable> reference =
            CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(0));
        UniformSpineSelector uniform(reference, kSpines);
        auto sent = [&]() {
            CustomHeader header(CustomHeader::L2_Header | CustomHeader::L3_Header);
            sender.hw->GetNxtPacket(qp)->PeekHeader(header);
            return header.ipid;
        };
        std::vector<uint16_t> window;
        bool drawn = true;
        for (uint32_t i = 0; i < 4; ++i)
        {
            window.push_back(sent());
            drawn &= window.back() == uniform.Choose(0);
        }
        NS_TEST_EXPECT_MSG_EQ(drawn, true, "the first window's spines are drawn");
        auto answer = [&](uint32_t packetSeq, uint16_t identification, uint8_t redirect) {
            CustomHeader ack;
            ack.l3Prot = 0xFC;
            ack.sip = kTestReceiver.Get();
            ack.ipid = identification;
            ack.ack.sport = IsolatedHost::kReceiverPort;
            ack.ack.dport = IsolatedHost::kSenderPort;
            ack.ack.pg = IsolatedHost::kPriorityGroup;
            ack.ack.flags = 0;
            ack.ack.seq = 0;
            ack.ack.packet_seq = packetSeq;
            ack.ack.redirect = redirect;
            sender.hw->ReceiveAck(Create<Packet>(), ack);
        };
        // The first send, moved by its leaf onto the next spine.
        const uint8_t asked = RequestedSpine(window[0]);
        const uint8_t crossed = (asked + 1) % kSpines;
        answer(0, SpineIdentification(asked, crossed), kNoRedirect);
        NS_TEST_EXPECT_MSG_EQ(sent(),
                              SpineIdentification(crossed, crossed),
                              "without a redirect the spine the send crossed, not the one asked");
        answer(1000, window[1], 3);
        NS_TEST_EXPECT_MSG_EQ(sent(), SpineIdentification(3, 3), "a redirect is followed");
        answer(2000, window[2], 200);
        NS_TEST_EXPECT_MSG_EQ(sent(), window[2], "a field naming no spine is no redirect");
        answer(3000, window[3], 4);
        qp->AddRepairRange(0, IsolatedHost::kMtu);
        CustomHeader repair(CustomHeader::L2_Header | CustomHeader::L3_Header |
                            CustomHeader::L4_Header);
        sender.hw->GetNxtPacket(qp)->PeekHeader(repair);
        NS_TEST_EXPECT_MSG_EQ(repair.udp.seq, 0, "the repair is sent next");
        NS_TEST_EXPECT_MSG_EQ(repair.ipid, SpineIdentification(4, 4), "on the redirect too");
        Simulator::Destroy();
    }
};

class HostSpineRedirectTest : public TestCase
{
  public:
    HostSpineRedirectTest()
        : TestCase("A host's redirect scores each spine from its grader's averages at each close "
                   "and redirects every connection to the best")
    {
    }

    void DoRun() override
    {
        Score();
        Best();
        Redirect();
    }

  private:
    static constexpr uint64_t kBase = 6000;

    static HostSpineRedirect::Parameters Defaults(double omega = 0.5,
                                                  bool oneWayDelay = false,
                                                  double flag = 0.1)
    {
        return HostSpineRedirect::Parameters{omega, flag, kBase, oneWayDelay};
    }

    // Four spines: clean, marked 0.2 with a delay average of the base delay,
    // trimmed 0.2, and the second's averages held.
    static std::vector<SpineGrader::SpineInterval> Averages()
    {
        std::vector<SpineGrader::SpineInterval> spines(4);
        spines[1].markFraction = 0.2;
        spines[1].delayNs = kBase;
        spines[2].trimFraction = 0.2;
        spines[2].markFraction = 0.1;
        spines[3] = spines[1];
        spines[3].held = true;
        return spines;
    }

    // omega / (D / L0 + 0.001) + (1 - omega)(1 - C), worked by hand for
    // C = 0.2 with D = L0, and D = 2 L0 under one-way delay.
    void Score()
    {
        struct Case
        {
            double omega;
            bool oneWayDelay;
            double score;
        };
        const Case cases[] = {
            {0.3, false, 0.8597002997002997},
            {0.3, true, 0.7099250374812594},
            {0.5, false, 0.8995004995004995},
            {0.5, true, 0.6498750624687656},
            {0.7, false, 0.9393006993006993},
            {0.7, true, 0.5898250874562719},
        };
        for (const Case& c : cases)
        {
            HostSpineRedirect redirect;
            redirect.OnClose(Averages(), Defaults(c.omega, c.oneWayDelay));
            NS_TEST_EXPECT_MSG_EQ_TOL(redirect.SpineScore(1),
                                      c.score,
                                      1e-12,
                                      "equation 1 on the averages at omega " << c.omega);
            NS_TEST_EXPECT_MSG_EQ_TOL(redirect.SpineScore(2),
                                      SpineRedirect::Score(c.omega, kBase, kBase, 0.2),
                                      1e-12,
                                      "the congestion average is the larger fraction");
            NS_TEST_EXPECT_MSG_EQ(redirect.SpineScore(3), 0.0, "a held spine scores 0");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.Best()), 0u, "the clean spine is best");
        }
    }

    void Best()
    {
        HostSpineRedirect redirect;
        std::vector<SpineGrader::SpineInterval> spines(4);
        spines[0].markFraction = 0.5;
        redirect.OnClose(spines, Defaults());
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.Best()),
                              1u,
                              "of tied highest, the lowest index at the first close");
        spines[0].markFraction = 0;
        redirect.OnClose(spines, Defaults());
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.Best()), 1u, "a tie keeps the current best");
        spines[1].markFraction = 0.3;
        redirect.OnArrival(2, Defaults());
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.Best()), 1u, "only a close moves the best");
        redirect.OnClose(spines, Defaults());
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.Best()), 0u, "a higher score wins at the close");
    }

    void Redirect()
    {
        HostSpineRedirect redirect;
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.OnArrival(2, Defaults())),
                              uint32_t(kNoRedirect),
                              "nothing is redirected before the first close");
        std::vector<SpineGrader::SpineInterval> spines(4);
        spines[2].markFraction = 0.4;
        const double best = SpineRedirect::Score(0.5, kBase, kBase, 0);
        const double worse = SpineRedirect::Score(0.5, kBase, kBase, 0.4);
        const double gap = (best - worse) / best;
        redirect.OnClose(spines, Defaults());
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.OnArrival(2, Defaults(0.5, false, gap - 1e-9))),
                              0u,
                              "a gap just over the flag redirects to the best");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.OnArrival(2, Defaults(0.5, false, gap + 1e-9))),
                              uint32_t(kNoRedirect),
                              "a gap just under it does not");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(redirect.OnArrival(0, Defaults(0.5, false, 0))),
                              uint32_t(kNoRedirect),
                              "an arrival on the best is not redirected");
    }
};

class AckRedirectScopeTest : public TestCase
{
  public:
    AckRedirectScopeTest()
        : TestCase("Under ack_redirect's per_receiver scope every connection into a host is "
                   "redirected by the host's averages, under per_connection by its own arrivals")
    {
    }

    void DoRun() override
    {
        const IntHeader::Mode savedIntMode = IntHeader::mode;
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        const uint32_t savedRedirectBytes = CustomHeader::ackRedirectBytes;
        IntHeader::mode = IntHeader::TS;
        CustomHeader::ackCarriesPacketSeq = true;
        CustomHeader::ackRedirectBytes = 1;
        const auto [hostMarked, hostClean] = Redirects(GradeScope::PerReceiver);
        NS_TEST_EXPECT_MSG_EQ(hostMarked, 0u, "the marked connection is moved off spine 3");
        NS_TEST_EXPECT_MSG_EQ(hostClean, 0u, "and so is the other, by the host's averages");
        const auto [ownMarked, ownClean] = Redirects(GradeScope::PerConnection);
        NS_TEST_EXPECT_MSG_EQ(ownMarked, 0u, "the marked connection is moved off spine 3");
        NS_TEST_EXPECT_MSG_EQ(ownClean,
                              uint32_t(kNoRedirect),
                              "the other, whose arrivals over spine 3 are clean, is not");
        CustomHeader::ackRedirectBytes = savedRedirectBytes;
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
        IntHeader::mode = savedIntMode;
    }

  private:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint64_t kBaseRtt = 10000;
    static constexpr uint16_t kOtherPort = 10002;

    // Two connections into node 0, whose grader's first interval ends at two
    // base RTTs: within it 16 packets per spine on each, those of the first
    // over spine 3 marked; after it one packet over spine 3 on each. The
    // redirect fields the two last answers carry.
    static std::pair<uint32_t, uint32_t> Redirects(GradeScope scope)
    {
        IsolatedHost receiver(LoadBalancingMode::SprayUniform, kTestSender, kSpines);
        receiver.hw->SetNode(CreateObject<Node>());
        receiver.hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::AckRedirect);
        receiver.hw->m_ackRedirectScope = static_cast<uint32_t>(scope);
        receiver.hw->m_sprayBaseRttNs = kBaseRtt;
        const std::vector<uint16_t> ports{IsolatedHost::kSenderPort, kOtherPort};
        std::map<uint16_t, uint32_t> seq;
        std::map<uint16_t, uint32_t> last;
        auto deliver = [&](uint16_t port, uint8_t spine, bool marked) {
            receiver.ReceiveData(kTestSender,
                                 kTestReceiver,
                                 seq[port],
                                 SpineIdentification(spine, spine),
                                 marked,
                                 port,
                                 kBaseRtt / 2);
            seq[port] += IsolatedHost::kMtu;
            last[port] = receiver.emitted.back().ack.redirect;
        };
        Simulator::Schedule(NanoSeconds(kBaseRtt), [&]() {
            for (uint16_t port : ports)
            {
                for (uint8_t spine = 0; spine < kSpines; ++spine)
                {
                    for (uint32_t j = 0; j < 16; ++j)
                    {
                        deliver(port, spine, spine == 3 && port == ports.front());
                    }
                }
            }
        });
        Simulator::Schedule(NanoSeconds(2 * kBaseRtt + 1000), [&]() {
            deliver(ports.front(), 3, true);
            deliver(ports.back(), 3, false);
        });
        Simulator::Run();
        Simulator::Destroy();
        return {last[ports.front()], last[ports.back()]};
    }
};

/**
 * Three leaves and four spines, every leaf linked to every spine at 400 Gbps
 * with ECN from 100 KB to 400 KB at a pmax of 0.2, and the SwitchQueueState
 * that reads their paths. Leaves 0 and 1 hold one host each, leaf 2 the
 * receiver.
 */
class SwitchQueueFabric
{
  public:
    static constexpr uint32_t kLeaves = 3;
    static constexpr uint32_t kSpines = 4;
    static constexpr uint32_t kPriorityGroup = 3;

    SwitchQueueFabric()
        : state(Create<SwitchQueueState>(kLeaves, kSpines))
    {
        for (uint32_t i = 0; i < kLeaves; ++i)
        {
            leaves.push_back(Switch());
        }
        for (uint32_t k = 0; k < kSpines; ++k)
        {
            spines.push_back(Switch());
        }
        for (uint32_t s = 0; s < kLeaves; ++s)
        {
            for (uint32_t k = 0; k < kSpines; ++k)
            {
                const uint32_t up = Port(leaves[s]);
                const uint32_t down = Port(spines[k]);
                Ptr<QbbChannel> channel = CreateObject<QbbChannel>();
                Device(leaves[s], up)->Attach(channel);
                Device(spines[k], down)->Attach(channel);
                uplink[s][k] = up;
                downlink[k][s] = down;
                state->SetUplink(s, k, SwitchQueueState::Hop{leaves[s], up, spines[k], down});
                state->SetDownlink(k, s, SwitchQueueState::Hop{spines[k], down, leaves[s], up});
            }
        }
        state->SetLeafOf(kTestSender.Get(), 0);
        state->SetLeafOf(kOtherSender.Get(), 1);
        state->SetLeafOf(kTestReceiver.Get(), 2);
    }

    static Ptr<QbbNetDevice> Device(Ptr<SwitchNode> sw, uint32_t port)
    {
        return DynamicCast<QbbNetDevice>(sw->GetDevice(port));
    }

    // The data bytes queued on spine k's downlink to the receiver's leaf, or
    // on leaf s's uplink to spine k.
    void QueueDown(uint32_t k, uint32_t bytes)
    {
        spines[k]->m_mmu->egress_bytes[downlink[k][2]][kPriorityGroup] = bytes;
    }

    void QueueUp(uint32_t s, uint32_t k, uint32_t bytes)
    {
        leaves[s]->m_mmu->egress_bytes[uplink[s][k]][kPriorityGroup] = bytes;
    }

    static inline const Ipv4Address kOtherSender{"11.0.3.1"};
    std::vector<Ptr<SwitchNode>> leaves;
    std::vector<Ptr<SwitchNode>> spines;
    uint32_t uplink[kLeaves][kSpines];
    uint32_t downlink[kSpines][kLeaves];
    Ptr<SwitchQueueState> state;

  private:
    static Ptr<SwitchNode> Switch()
    {
        Ptr<SwitchNode> sw = CreateObject<SwitchNode>();
        sw->SetAttribute("EcnEnabled", BooleanValue(true));
        // Port 0 is the loopback every node holds.
        sw->AddDevice(CreateObject<QbbNetDevice>());
        return sw;
    }

    static uint32_t Port(Ptr<SwitchNode> sw)
    {
        Ptr<QbbNetDevice> device = CreateObject<QbbNetDevice>();
        device->SetQueue(CreateObject<BEgressQueue>());
        device->SetDataRate(DataRate("400Gbps"));
        const uint32_t port = sw->AddDevice(device);
        sw->m_mmu->ConfigEcn(port, 100, 400, 0.2, port);
        return port;
    }
};

class SwitchQueueStateTest : public TestCase
{
  public:
    SwitchQueueStateTest()
        : TestCase("Simulator state read per path: the marking probability of each hop's queue, "
                   "their queueing delay and their configured loss")
    {
    }

    void DoRun() override
    {
        Marking();
        TwoHops();
        Loss();
        ReverseLoss();
        Simulator::Destroy();
    }

  private:
    // The probability ShouldSendCN marks with at each occupancy.
    void Marking()
    {
        SwitchQueueFabric fabric;
        Ptr<SwitchMmu> mmu = fabric.leaves[0]->m_mmu;
        const uint32_t port = fabric.uplink[0][0];
        NS_TEST_EXPECT_MSG_EQ(mmu->MarkingProbability(port, 100000), 0.0, "0 at kmin");
        NS_TEST_EXPECT_MSG_EQ_TOL(mmu->MarkingProbability(port, 250000),
                                  0.1,
                                  1e-12,
                                  "linear between kmin and kmax");
        NS_TEST_EXPECT_MSG_EQ_TOL(mmu->MarkingProbability(port, 400000), 0.2, 1e-12, "pmax at kmax");
        NS_TEST_EXPECT_MSG_EQ(mmu->MarkingProbability(port, 400001), 1.0, "1 above kmax");
        mmu->egress_bytes[port][3] = 400001;
        NS_TEST_EXPECT_MSG_EQ(mmu->ShouldSendCN(port, 3), true, "as ShouldSendCN marks above kmax");
        mmu->egress_bytes[port][3] = 100000;
        NS_TEST_EXPECT_MSG_EQ(mmu->ShouldSendCN(port, 3), false, "and not at kmin");
    }

    // Leaf 0's uplink to spine 1 halfway between the thresholds, spine 1's
    // downlink at kmax and at half rate.
    void TwoHops()
    {
        SwitchQueueFabric fabric;
        fabric.QueueUp(0, 1, 250000);
        fabric.QueueDown(1, 400000);
        SwitchQueueFabric::Device(fabric.spines[1], fabric.downlink[1][2])
            ->SetDataRate(DataRate("200Gbps"));
        const auto fromZero = fabric.state->PathValues(0, 2, true);
        NS_TEST_EXPECT_MSG_EQ_TOL(fromZero[1].markFraction,
                                  1 - 0.9 * 0.8,
                                  1e-12,
                                  "a packet crossing both hops arrives marked");
        NS_TEST_EXPECT_MSG_EQ_TOL(fromZero[1].delayNs,
                                  8.0 * 250000 / 400 + 8.0 * 400000 / 200,
                                  1e-6,
                                  "each hop delays it by its queue over its rate");
        NS_TEST_EXPECT_MSG_EQ(fromZero[1].trimFraction, 0.0, "no trimmed fraction is read");
        NS_TEST_EXPECT_MSG_EQ(fromZero[0].markFraction, 0.0, "the other spines are clean");
        NS_TEST_EXPECT_MSG_EQ(fabric.state->PathValues(0, 2, false)[1].delayNs,
                              0.0,
                              "no delay unless asked for");
        const auto fromOne = fabric.state->PathValues(1, 2, true);
        NS_TEST_EXPECT_MSG_EQ_TOL(fromOne[1].markFraction,
                                  0.2,
                                  1e-12,
                                  "leaf 1 shares only the downlink");
        NS_TEST_EXPECT_MSG_EQ(fabric.state->LeafOf(SwitchQueueFabric::kOtherSender.Get()),
                              1u,
                              "a host's leaf is found by its address");
        NS_TEST_EXPECT_MSG_EQ(fabric.state->LeafOf(0),
                              SwitchQueueState::kNoLeaf,
                              "an unknown address has none");
    }

    void Loss()
    {
        SwitchQueueFabric fabric;
        for (uint32_t s = 0; s < 2; ++s)
        {
            for (const auto& values : fabric.state->PathValues(s, 2, false))
            {
                NS_TEST_EXPECT_MSG_EQ(values.lossFraction, 0.0, "no loss configured, none read");
            }
        }
        // One percent lost where leaf 0's uplink to spine 2 is received.
        Ptr<RateErrorModel> gray = CreateObject<RateErrorModel>();
        gray->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
        gray->SetRate(0.01);
        SwitchQueueFabric::Device(fabric.spines[2], fabric.downlink[2][0])
            ->SetAttribute("LinkErrorModel", PointerValue(gray));
        // Spine 3's downlink to the receiver's leaf silent.
        fabric.leaves[2]->SetBlackhole(fabric.uplink[2][3]);
        const auto fromZero = fabric.state->PathValues(0, 2, false);
        const auto fromOne = fabric.state->PathValues(1, 2, false);
        NS_TEST_EXPECT_MSG_EQ_TOL(fromZero[2].lossFraction, 0.01, 1e-12, "the gray hop's path");
        NS_TEST_EXPECT_MSG_EQ(fromOne[2].lossFraction, 0.0, "and no other path");
        NS_TEST_EXPECT_MSG_EQ(fromZero[3].lossFraction, 1.0, "a silent port loses everything");
        NS_TEST_EXPECT_MSG_EQ(fromOne[3].lossFraction, 1.0, "on every path through it");
        NS_TEST_EXPECT_MSG_EQ(fromZero[0].lossFraction, 0.0, "the others lose nothing");
        // From the receiver's leaf both faults sit on the hops back.
        const auto fromTwo = fabric.state->PathValues(2, 0, false);
        NS_TEST_EXPECT_MSG_EQ_TOL(fromTwo[2].lossFraction,
                                  0.01,
                                  1e-12,
                                  "the gray hop drops the ACK coming back");
        NS_TEST_EXPECT_MSG_EQ(fromTwo[3].lossFraction, 1.0, "and the silent port does too");
        NS_TEST_EXPECT_MSG_EQ(fromTwo[0].lossFraction, 0.0, "the others lose nothing back");
    }

    // The ACK returns through the spine that carried the packet, over the
    // destination leaf's uplink and the spine's downlink to the source leaf.
    void ReverseLoss()
    {
        {
            SwitchQueueFabric fabric;
            // The receiver's leaf's uplink to spine 1 silent.
            fabric.spines[1]->SetBlackhole(fabric.downlink[1][2]);
            // Its uplink to spine 1 queued above kmax.
            fabric.QueueUp(2, 1, 400001);
            const auto fromZero = fabric.state->PathValues(0, 2, false);
            NS_TEST_EXPECT_MSG_EQ(fromZero[1].lossFraction,
                                  1.0,
                                  "a silent reverse hop loses every ACK");
            NS_TEST_EXPECT_MSG_EQ(fromZero[1].markFraction,
                                  0.0,
                                  "the marked fraction is the forward hops' only");
            for (uint32_t k : {0u, 2u, 3u})
            {
                NS_TEST_EXPECT_MSG_EQ(fromZero[k].lossFraction, 0.0, "the other spines lose nothing");
            }
        }
        {
            SwitchQueueFabric fabric;
            // Five percent lost each way on the sender's leaf's link to spine 3.
            const double e = 0.05;
            for (const auto& [sw, port] :
                 {std::make_pair(fabric.spines[3], fabric.downlink[3][0]),
                  std::make_pair(fabric.leaves[0], fabric.uplink[0][3])})
            {
                Ptr<RateErrorModel> gray = CreateObject<RateErrorModel>();
                gray->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
                gray->SetRate(e);
                SwitchQueueFabric::Device(sw, port)->SetAttribute("LinkErrorModel",
                                                                  PointerValue(gray));
            }
            const auto fromZero = fabric.state->PathValues(0, 2, false);
            NS_TEST_EXPECT_MSG_EQ_TOL(fromZero[3].lossFraction,
                                      1 - (1 - e) * (1 - e),
                                      1e-12,
                                      "the packet and its ACK each cross the lossy link");
            NS_TEST_EXPECT_MSG_EQ(fromZero[0].lossFraction, 0.0, "the other spines lose nothing");
        }
    }
};

class RepsOracleTest : public TestCase
{
  public:
    RepsOracleTest()
        : TestCase("Under reps_oracle a spine the switches' state shows a loss or a slow port on "
                   "toward a destination leaf leaves the REPS choices of the flows to that leaf "
                   "for the run; the policy needs the switch_queue cost source")
    {
    }

    void DoRun() override
    {
        SwitchQueueFabric fabric;
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        hw->SetAttribute("Mtu", UintegerValue(1000));
        hw->SetAttribute("PathRandomVariable", PointerValue(SelectorStream()));
        hw->m_spineCount = SwitchQueueFabric::kSpines;
        hw->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
        hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::RepsOracle);
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsOracle(), false, "refused without the cost source");
        hw->m_sprayGradeCostSource = static_cast<uint32_t>(GradeCostSource::SwitchQueue);
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsOracle(), false, "and without the switches' state");
        hw->m_switchQueues = fabric.state;
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsOracle(), true, "accepted with both");
        hw->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::EntropyHash);
        NS_TEST_EXPECT_MSG_EQ(hw->ValidRepsOracle(), false, "refused without named spines");
        hw->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
        hw->m_sprayBaseRttNs = 1000;
        // The sending host is on leaf 1.
        hw->m_leaf = fabric.state->LeafOf(SwitchQueueFabric::kOtherSender.Get());

        // One percent lost where spine 3's downlink to leaf 0 is received,
        // and spine 2's downlink to leaf 2 at half rate.
        Ptr<RateErrorModel> gray = CreateObject<RateErrorModel>();
        gray->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
        gray->SetRate(0.01);
        SwitchQueueFabric::Device(fabric.leaves[0], fabric.uplink[0][3])
            ->SetAttribute("LinkErrorModel", PointerValue(gray));
        Ptr<QbbNetDevice> slow = SwitchQueueFabric::Device(fabric.spines[2], fabric.downlink[2][2]);
        slow->SetDataRate(DataRate(slow->GetDataRate().GetBitRate() / 2));
        NS_TEST_EXPECT_MSG_EQ((fabric.state->Faulted(1, 0) ==
                               std::vector<bool>{false, false, false, true}),
                              true,
                              "toward leaf 0 the lossy spine is faulted");
        NS_TEST_EXPECT_MSG_EQ((fabric.state->Faulted(1, 2) ==
                               std::vector<bool>{false, false, true, false}),
                              true,
                              "toward leaf 2 the slow spine is");

        Ptr<RdmaQueuePair> toZero = CreateObject<RdmaQueuePair>(
            3, SwitchQueueFabric::kOtherSender, kTestSender, 10000, 100);
        Ptr<RdmaQueuePair> toTwo = CreateObject<RdmaQueuePair>(
            3, SwitchQueueFabric::kOtherSender, kTestReceiver, 10001, 100);
        hw->StartPathSelection(toZero, 0, 0);
        hw->StartPathSelection(toTwo, 0, 0);
        std::vector<uint32_t> zeroOn(SwitchQueueFabric::kSpines);
        std::vector<uint32_t> twoOn(SwitchQueueFabric::kSpines);
        for (uint32_t step = 0; step < 800; ++step)
        {
            // Halfway the faults are cleared; the spines stay out.
            if (step == 400)
            {
                gray->Disable();
                slow->SetDataRate(DataRate(slow->GetDataRate().GetBitRate() * 2));
            }
            const bool zero = step % 2 == 0;
            const uint8_t spine = LossHoldFixture::Send(
                *(zero ? toZero : toTwo)->m_pathSelector, 100 * step, step / 2, {});
            (zero ? zeroOn : twoOn)[spine]++;
        }
        NS_TEST_EXPECT_MSG_EQ(zeroOn[3], 0u, "the flow to leaf 0 never sends on spine 3");
        NS_TEST_EXPECT_MSG_GT(twoOn[3], 0u, "the flow to leaf 2 does");
        NS_TEST_EXPECT_MSG_EQ(twoOn[2], 0u, "the flow to leaf 2 never sends on spine 2");
        NS_TEST_EXPECT_MSG_GT(zeroOn[2], 0u, "the flow to leaf 0 does");
        Simulator::Destroy();
    }
};

class RepsSpineTest : public TestCase
{
  public:
    RepsSpineTest()
        : TestCase("Under reps_spine REPS draws spine indices, reuses each unmarked "
                   "acknowledgement's spine once and caches no marked one")
    {
    }

    void DoRun() override
    {
        constexpr uint32_t kSpines = 4;
        Ptr<RdmaHw> hw = CreateObject<RdmaHw>();
        hw->SetAttribute("Mtu", UintegerValue(1000));
        hw->SetAttribute("PathRandomVariable", PointerValue(SelectorStream()));
        hw->m_spineCount = kSpines;
        hw->m_loadBalancing = static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
        hw->m_pathSelectorKind = static_cast<uint32_t>(PathSelectorKind::RepsSpine);
        NS_TEST_EXPECT_MSG_EQ(hw->IsRepsHold(), false, "no loss hold");
        NS_TEST_EXPECT_MSG_EQ(hw->CountsSprayLoss(), false, "no loss counted");
        Ptr<RdmaQueuePair> qp =
            CreateObject<RdmaQueuePair>(3, kTestSender, kTestReceiver, 10000, 100);
        hw->StartPathSelection(qp, 0, 0);
        PathSelector& reps = *qp->m_pathSelector;
        // A uniform spine draw on the same stream: what a fresh draw is.
        UniformSpineSelector fresh(SelectorStream(), kSpines);
        const auto freshSends = [&reps, &fresh](uint32_t sends) {
            uint32_t drawn = 0;
            for (uint32_t i = 0; i < sends; ++i)
            {
                const uint16_t identification = reps.Choose(0);
                drawn += identification == fresh.Choose(0) &&
                         RequestedSpine(identification) < kSpines &&
                         RequestedSpine(identification) == CarryingSpine(identification);
            }
            return drawn;
        };
        NS_TEST_EXPECT_MSG_EQ(freshSends(50), 50u, "fresh draws are spine indices");

        reps.OnAck(2, false, 0);
        const uint16_t reused = reps.Choose(0);
        NS_TEST_EXPECT_MSG_EQ(reused, SpineIdentification(2, 2), "an unmarked spine is reused");
        NS_TEST_EXPECT_MSG_EQ(freshSends(20), 20u, "and only once");
        reps.OnAck(3, true, 0);
        NS_TEST_EXPECT_MSG_EQ(freshSends(20), 20u, "a marked spine is not cached");
        Simulator::Destroy();
    }
};

class SwitchQueueGradeTest : public TestCase
{
  public:
    SwitchQueueGradeTest()
        : TestCase("Under grade cost source switch_queue each source leaf's paths are graded "
                   "from the switches' state at the grader's close, by the grade rule")
    {
    }

    void DoRun() override
    {
        Substitution();
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        const uint32_t savedReportBytes = CustomHeader::ackReportBytes;
        CustomHeader::ackCarriesPacketSeq = true;
        CustomHeader::ackReportBytes = 1 + SpineReport::GradeBytes(SwitchQueueFabric::kSpines);
        Transport();
        CustomHeader::ackReportBytes = savedReportBytes;
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
    }

  private:
    static constexpr uint64_t kBaseRtt = 10000;

    // The grade rule on the grader's own averages gives the grader's report;
    // with zero loss graded it gives the same.
    void Substitution()
    {
        SpineGrader::Parameters parameters = GraderFixture::Defaults();
        SpineGrader grader(parameters);
        for (uint32_t i = 0; i < 3; ++i)
        {
            for (uint8_t spine = 0; spine < 8; ++spine)
            {
                for (uint32_t j = 0; j < 40; ++j)
                {
                    grader.OnArrival(spine, spine, spine == 2 || (spine == 5 && j % 3 == 0), 0);
                }
            }
            grader.Advance((i + 1) * parameters.intervalNs);
        }
        std::vector<SpineGrader::SpineValues> values(8);
        std::vector<bool> held(8);
        for (uint32_t k = 0; k < 8; ++k)
        {
            const SpineGrader::SpineInterval& closed = grader.LastInterval()[k];
            values[k] = {closed.markFraction, closed.trimFraction, closed.delayNs, 0};
            held[k] = closed.held;
        }
        const SpineReport& report = grader.Report();
        NS_TEST_ASSERT_MSG_LT(uint32_t(report.Grade(2)), 3u, "the grader graded spine 2 down");
        for (const std::optional<double> lossHold : {std::optional<double>(), std::optional(0.875)})
        {
            const SpineReport graded = grader.Graded(values, held, lossHold);
            NS_TEST_EXPECT_MSG_EQ(graded.sequence, report.sequence, "the latest sequence");
            for (uint32_t k = 0; k < 8; ++k)
            {
                NS_TEST_EXPECT_MSG_EQ(uint32_t(graded.Grade(k)),
                                      uint32_t(report.Grade(k)),
                                      "the grader's own averages earn its own grades");
            }
        }
        values[6].lossFraction = 0.875;
        NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Graded(values, held, 0.875).Grade(6)),
                              0u,
                              "a loss reaching the hold level holds the spine");
        values[6].lossFraction = 0.6;
        NS_TEST_EXPECT_MSG_EQ(uint32_t(grader.Graded(values, held, 0.875).Grade(6)),
                              1u,
                              "a smaller loss costs grades over the median's");
        // Every spine losing alike costs none of them a grade against the
        // median, but each is held all the same.
        for (SpineGrader::SpineValues& spine : values)
        {
            spine.lossFraction = 0.9;
        }
        const SpineReport lossy = grader.Graded(values, held, 0.875);
        for (uint32_t k = 0; k < 8; ++k)
        {
            NS_TEST_EXPECT_MSG_EQ(uint32_t(lossy.Grade(k)), 0u, "a loss at the hold level holds");
        }
    }

    // The receiver on leaf 2; one data packet from each sender just after
    // each of three interval closes. The grades of spine k in the answers to
    // each sender after each close.
    struct Answers
    {
        std::vector<SpineReport> fromZero;
        std::vector<SpineReport> fromOne;
    };

    static SpineReport Carried(const CustomHeader& answer)
    {
        SpineReport report;
        report.sequence = answer.ack.spine_report[0];
        std::copy_n(answer.ack.spine_report + 1,
                    SpineReport::GradeBytes(SwitchQueueFabric::kSpines),
                    report.grades);
        return report;
    }

    void Transport()
    {
        SwitchQueueFabric fabric;
        IsolatedHost receiver(LoadBalancingMode::SprayPolicy,
                              kTestSender,
                              SwitchQueueFabric::kSpines);
        receiver.hw->m_rtTable[SwitchQueueFabric::kOtherSender.Get()].push_back(0);
        Ptr<Node> node = CreateObject<Node>();
        receiver.hw->SetNode(node);
        receiver.hw->m_sprayBaseRttNs = kBaseRtt;
        receiver.hw->m_sprayGradeCostSource =
            static_cast<uint32_t>(GradeCostSource::SwitchQueue);
        receiver.hw->m_switchQueues = fabric.state;
        receiver.hw->m_leaf = 2;
        const uint64_t interval = receiver.hw->SprayReportIntervalNs();
        // Where the receiver's intervals start.
        const uint64_t phase = std::fmod(node->GetId() * 0.6180339887498949, 1.0) * interval;
        Answers answers;
        std::map<uint32_t, uint32_t> seq;
        auto deliver = [&](Ipv4Address from) {
            receiver.ReceiveData(from, kTestReceiver, seq[from.Get()], SpineIdentification(0, 0));
            seq[from.Get()] += IsolatedHost::kMtu;
            return Carried(receiver.emitted.back());
        };
        auto both = [&]() {
            answers.fromZero.push_back(deliver(kTestSender));
            answers.fromOne.push_back(deliver(SwitchQueueFabric::kOtherSender));
        };
        // Within the first interval spine 1's downlink fills and drains again;
        // at its close only leaf 0's uplink to spine 3 is full.
        Simulator::Schedule(NanoSeconds(phase + interval / 4), [&]() {
            both();
            fabric.QueueDown(1, 500000);
        });
        Simulator::Schedule(NanoSeconds(phase + interval / 2), [&]() {
            fabric.QueueDown(1, 0);
            fabric.QueueUp(0, 3, 500000);
        });
        Simulator::Schedule(NanoSeconds(phase + interval + 1), both);
        // At the second close spine 1's downlink is silent.
        Simulator::Schedule(NanoSeconds(phase + interval + 2), [&]() {
            fabric.leaves[2]->SetBlackhole(fabric.uplink[2][1]);
        });
        Simulator::Schedule(NanoSeconds(phase + 2 * interval + 1), both);
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(answers.fromZero.size(), 3u, "three answers to each sender");
        for (uint32_t k = 0; k < SwitchQueueFabric::kSpines; ++k)
        {
            NS_TEST_EXPECT_MSG_EQ(uint32_t(answers.fromZero[0].Grade(k)),
                                  3u,
                                  "before the first close every spine is at the top");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(answers.fromZero[1].Grade(k)),
                                  (k == 3 ? 0u : 3u),
                                  "at the first close only what is queued then counts");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(answers.fromOne[1].Grade(k)),
                                  3u,
                                  "and leaf 0's uplink is not leaf 1's path");
            NS_TEST_EXPECT_MSG_EQ(uint32_t(answers.fromOne[2].Grade(k)),
                                  (k == 1 ? 0u : 3u),
                                  "a silent downlink holds its spine at the next close");
        }
        NS_TEST_EXPECT_MSG_EQ(uint32_t(answers.fromZero[1].sequence),
                              1u,
                              "the reports keep the grader's sequence");
    }
};

class GradeScopeTest : public TestCase
{
  public:
    GradeScopeTest()
        : TestCase("Under grade scope per_connection a receive queue pair grades the spines from "
                   "its own arrivals and a sending queue pair keeps its own scores")
    {
    }

    void DoRun() override
    {
        const bool savedPacketSeq = CustomHeader::ackCarriesPacketSeq;
        const uint32_t savedReportBytes = CustomHeader::ackReportBytes;
        CustomHeader::ackCarriesPacketSeq = true;
        CustomHeader::ackReportBytes = 1 + SpineReport::GradeBytes(kSpines);
        OneConnection();
        TwoConnections();
        SenderScores();
        CustomHeader::ackReportBytes = savedReportBytes;
        CustomHeader::ackCarriesPacketSeq = savedPacketSeq;
    }

  private:
    static constexpr uint32_t kSpines = 8;
    static constexpr uint64_t kBaseRtt = 10000;
    static constexpr uint16_t kOtherPort = 10002;

    // What a receiver under scope emits for four intervals of arrivals over
    // every spine from each of ports, spine 2 marked on the first port's
    // packets, spine 5 trimmed once per interval on each port's.
    static std::vector<CustomHeader> Answers(GradeScope scope, std::vector<uint16_t> ports)
    {
        IsolatedHost receiver(LoadBalancingMode::SprayPolicy, kTestSender, kSpines);
        // Node 1, whose intervals start at a phase of their own.
        CreateObject<Node>();
        Ptr<Node> node = CreateObject<Node>();
        receiver.hw->SetNode(node);
        receiver.hw->m_sprayBaseRttNs = kBaseRtt;
        receiver.hw->m_sprayGradeScope = static_cast<uint32_t>(scope);
        const uint64_t interval = receiver.hw->SprayReportIntervalNs();
        const uint64_t phase = std::fmod(node->GetId() * 0.6180339887498949, 1.0) * interval;
        std::map<uint16_t, uint32_t> seq;
        // Spread over each interval, so that an interval closing at another
        // phase would split them differently.
        for (uint32_t i = 0; i < 4; ++i)
        {
            for (uint32_t j = 0; j < 8; ++j)
            {
                Simulator::Schedule(NanoSeconds(phase + i * interval + 1 + j * interval / 9),
                                    [&, i, j]() {
                                        for (uint16_t port : ports)
                                        {
                                            Deliver(receiver, seq[port], port, i, j, ports);
                                        }
                                    });
            }
        }
        Simulator::Run();
        Simulator::Destroy();
        return receiver.emitted;
    }

    // The j-th of eight packets per spine in interval i on the connection from
    // port, spine 2 marked on the first port's; in the first three intervals
    // every spine and a trim on spine 5, in the fourth spine 0 only.
    static void Deliver(IsolatedHost& receiver,
                        uint32_t& seq,
                        uint16_t port,
                        uint32_t i,
                        uint32_t j,
                        const std::vector<uint16_t>& ports)
    {
        for (uint8_t spine = 0; spine < kSpines && (i < 3 || spine == 0); ++spine)
        {
            receiver.ReceiveData(kTestSender,
                                 kTestReceiver,
                                 seq,
                                 SpineIdentification(spine, spine),
                                 spine == 2 && port == ports.front(),
                                 port);
            seq += IsolatedHost::kMtu;
        }
        if (i < 3 && j == 0)
        {
            Trim(receiver, port, seq, 5);
            seq += IsolatedHost::kMtu;
        }
    }

    // A data packet of the connection from port trimmed on spine before the
    // last hop, as its destination parses it.
    static void Trim(IsolatedHost& receiver, uint16_t port, uint32_t seq, uint8_t spine)
    {
        CustomHeader trimmed;
        trimmed.l3Prot = 0x11;
        trimmed.m_tos = kUetDscpTrimmed << 2;
        trimmed.sip = kTestSender.Get();
        trimmed.dip = kTestReceiver.Get();
        trimmed.ipid = SpineIdentification(spine, spine);
        trimmed.udp.sport = port;
        trimmed.udp.dport = IsolatedHost::kReceiverPort;
        trimmed.udp.pg = IsolatedHost::kPriorityGroup;
        trimmed.udp.seq = seq;
        trimmed.udp.payload_size = CustomHeader::GetUdpHeaderSize() + IsolatedHost::kMtu;
        receiver.hw->Receive(Create<Packet>(), trimmed);
    }

    static SpineReport LastReportTo(const std::vector<CustomHeader>& answers, uint16_t port)
    {
        SpineReport report;
        for (const CustomHeader& answer : answers)
        {
            if (answer.ack.dport == port)
            {
                report.sequence = answer.ack.spine_report[0];
                std::copy_n(answer.ack.spine_report + 1,
                            SpineReport::GradeBytes(kSpines),
                            report.grades);
            }
        }
        return report;
    }

    // With one connection into the receiver its arrivals are all the host's,
    // so the two scopes answer alike, report for report.
    void OneConnection()
    {
        const std::vector<CustomHeader> host =
            Answers(GradeScope::PerReceiver, {IsolatedHost::kSenderPort});
        const std::vector<CustomHeader> own =
            Answers(GradeScope::PerConnection, {IsolatedHost::kSenderPort});
        NS_TEST_ASSERT_MSG_EQ(own.size(), host.size(), "every packet is answered in both");
        bool alike = true;
        for (size_t i = 0; i < host.size(); ++i)
        {
            alike &= own[i].l3Prot == host[i].l3Prot && own[i].ack.seq == host[i].ack.seq &&
                     own[i].ack.flags == host[i].ack.flags &&
                     std::equal(own[i].ack.spine_report,
                                own[i].ack.spine_report + CustomHeader::ackReportBytes,
                                host[i].ack.spine_report);
        }
        NS_TEST_EXPECT_MSG_EQ(alike, true, "one connection's grades are its host's");
        const SpineReport last = LastReportTo(host, IsolatedHost::kSenderPort);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(last.sequence), 3u, "three intervals were reported");
        NS_TEST_EXPECT_MSG_LT(uint32_t(last.Grade(2)), 3u, "and the marked spine graded down");
    }

    // Marks on one connection's arrivals over spine 2 lower that spine for
    // both connections when the host grades, and for the marked one alone
    // when each connection grades its own.
    void TwoConnections()
    {
        const std::vector<uint16_t> ports{IsolatedHost::kSenderPort, kOtherPort};
        const std::vector<CustomHeader> host = Answers(GradeScope::PerReceiver, ports);
        const std::vector<CustomHeader> own = Answers(GradeScope::PerConnection, ports);
        const SpineReport hostMarked = LastReportTo(host, IsolatedHost::kSenderPort);
        const SpineReport hostClean = LastReportTo(host, kOtherPort);
        NS_TEST_EXPECT_MSG_LT(uint32_t(hostMarked.Grade(2)), 3u, "the host grades spine 2 down");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(hostClean.Grade(2)),
                              uint32_t(hostMarked.Grade(2)),
                              "for both connections");
        const SpineReport ownMarked = LastReportTo(own, IsolatedHost::kSenderPort);
        const SpineReport ownClean = LastReportTo(own, kOtherPort);
        NS_TEST_EXPECT_MSG_EQ(uint32_t(ownMarked.sequence), 3u, "each connection reports");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(ownClean.sequence), 3u, "on the host's intervals");
        NS_TEST_EXPECT_MSG_LT(uint32_t(ownMarked.Grade(2)),
                              uint32_t(hostMarked.Grade(2)),
                              "the marked connection sees spine 2 alone and grades it lower");
        NS_TEST_EXPECT_MSG_EQ(uint32_t(ownClean.Grade(2)),
                              3u,
                              "the other connection's spine 2 keeps the top grade");
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            if (spine != 2)
            {
                NS_TEST_EXPECT_MSG_EQ(uint32_t(ownMarked.Grade(spine)),
                                      uint32_t(ownClean.Grade(spine)),
                                      "the connections agree on every other spine");
            }
        }
    }

    // The acknowledgement of a queue pair's first send, carrying a report
    // that grades every spine 0 but spine 6.
    static void Report(IsolatedHost& sender, uint16_t port, uint8_t sequence)
    {
        CustomHeader ack;
        ack.l3Prot = 0xFC;
        ack.sip = kTestReceiver.Get();
        ack.ack.sport = IsolatedHost::kReceiverPort;
        ack.ack.dport = port;
        ack.ack.pg = IsolatedHost::kPriorityGroup;
        ack.ack.flags = 0;
        ack.ack.seq = 0;
        ack.ack.packet_seq = 0;
        SpineReport report;
        report.sequence = sequence;
        for (uint32_t spine = 0; spine < kSpines; ++spine)
        {
            report.SetGrade(spine, spine == 6 ? 3 : 0);
        }
        ack.ack.spine_report[0] = report.sequence;
        std::copy_n(report.grades, SpineReport::GradeBytes(kSpines), ack.ack.spine_report + 1);
        sender.hw->ReceiveAck(Create<Packet>(), ack);
    }

    // The sends on spine 6 out of a thousand each queue pair to one host draws
    // after reports reach the first of them only, and the sets of scores
    // shared per destination host.
    static std::tuple<uint32_t, uint32_t, size_t> OnSix(GradeScope scope)
    {
        IsolatedHost sender(LoadBalancingMode::SprayPolicy, kTestReceiver, kSpines);
        sender.hw->m_sprayBaseRttNs = kBaseRtt;
        sender.hw->m_sprayGradeScope = static_cast<uint32_t>(scope);
        Ptr<RdmaQueuePair> first =
            sender.AddSender(kTestSender, kTestReceiver, 100 * IsolatedHost::kMtu);
        Ptr<RdmaQueuePair> second =
            sender.AddSender(kTestSender, kTestReceiver, 100 * IsolatedHost::kMtu, kOtherPort);
        for (uint8_t sequence = 1; sequence < 40; ++sequence)
        {
            Report(sender, IsolatedHost::kSenderPort, sequence);
        }
        uint32_t firstOnSix = 0;
        uint32_t secondOnSix = 0;
        for (uint32_t i = 0; i < 1000; ++i)
        {
            firstOnSix += RequestedSpine(first->m_pathSelector->Choose(0)) == 6;
            secondOnSix += RequestedSpine(second->m_pathSelector->Choose(0)) == 6;
        }
        const size_t shared = sender.hw->m_spineScores.size();
        Simulator::Destroy();
        return {firstOnSix, secondOnSix, shared};
    }

    void SenderScores()
    {
        const auto [hostFirst, hostSecond, hostShared] = OnSix(GradeScope::PerReceiver);
        NS_TEST_EXPECT_MSG_EQ(hostShared, 1u, "one set of scores per destination host");
        NS_TEST_EXPECT_MSG_GT(hostFirst, 950u, "the reported queue pair follows its reports");
        NS_TEST_EXPECT_MSG_GT(hostSecond, 950u, "and so does the other, sharing its scores");
        const auto [ownFirst, ownSecond, ownShared] = OnSix(GradeScope::PerConnection);
        NS_TEST_EXPECT_MSG_EQ(ownShared, 0u, "no scores shared per destination host");
        NS_TEST_EXPECT_MSG_GT(ownFirst, 950u, "the reported queue pair follows its reports");
        NS_TEST_EXPECT_MSG_LT(ownSecond, 200u, "the other keeps scores of its own");
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
    AddTestCase(new LinkRestoreTest, TestCase::Duration::QUICK);
    AddTestCase(new DirectionalLinkFaultTest, TestCase::Duration::QUICK);
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
    AddTestCase(new SpineTrimCostTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineTrimThresholdsTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineGradeReferenceTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineDelayReferenceTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineAbsenceTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineCongestionGradeFloorTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineScoresDrainTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineScoresFloorTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineScoresFlapTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineScoresDrawTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineScoresEdgeFreezeTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineScoresPoolTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineLossThresholdsTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineHeldExplorationTest, TestCase::Duration::QUICK);
    AddTestCase(new SpinePoolHoldPropagationTest, TestCase::Duration::QUICK);
    AddTestCase(new RepsHoldTest, TestCase::Duration::QUICK);
    AddTestCase(new MrcHoldTest, TestCase::Duration::QUICK);
    AddTestCase(new MrcHoldExplorationTest, TestCase::Duration::QUICK);
    AddTestCase(new RepsFlowHoldTest, TestCase::Duration::QUICK);
    AddTestCase(new RepsHoldScopeTest, TestCase::Duration::QUICK);
    AddTestCase(new RepsHoldAutoScopeTest, TestCase::Duration::QUICK);
    AddTestCase(new LossWithoutSendsTest, TestCase::Duration::QUICK);
    AddTestCase(new RepsHoldLlrTest, TestCase::Duration::QUICK);
    AddTestCase(new RepsHoldTrickleTest, TestCase::Duration::QUICK);
    AddTestCase(new RepsHoldDuplicateTest, TestCase::Duration::QUICK);
    AddTestCase(new RetransmissionTimeoutTraceTest, TestCase::Duration::QUICK);
    AddTestCase(new HeldSpineLeakTest, TestCase::Duration::QUICK);
    AddTestCase(new HeldSpineFastLossTest, TestCase::Duration::QUICK);
    AddTestCase(new SprayAckLossTest(SprayAckLossTest::Answer::Dropped),
                TestCase::Duration::QUICK);
    AddTestCase(new SprayAckLossTest(SprayAckLossTest::Answer::Late), TestCase::Duration::QUICK);
    AddTestCase(new SprayAckLossTest(SprayAckLossTest::Answer::Off), TestCase::Duration::QUICK);
    AddTestCase(new SprayAckLossValidTest, TestCase::Duration::QUICK);
    AddTestCase(new SprayTailRecoveryTest, TestCase::Duration::QUICK);
    AddTestCase(new SprayPathSeqTest, TestCase::Duration::QUICK);
    AddTestCase(new SprayReceiverHoldTest, TestCase::Duration::QUICK);
    AddTestCase(new SprayReceiverHoldTraceTest, TestCase::Duration::QUICK);
    AddTestCase(new SprayPathSeqTrimTest, TestCase::Duration::QUICK);
    AddTestCase(new SprayPolicyTransportTest, TestCase::Duration::QUICK);
    AddTestCase(new NsccReceiverPenaltyTest, TestCase::Duration::QUICK);
    AddTestCase(new SprayPolicyDefaultsTest, TestCase::Duration::QUICK);
    AddTestCase(new GradeScopeTest, TestCase::Duration::QUICK);
    AddTestCase(new SpineRedirectTest, TestCase::Duration::QUICK);
    AddTestCase(new AckRedirectSelectorTest, TestCase::Duration::QUICK);
    AddTestCase(new AckRedirectTransportTest, TestCase::Duration::QUICK);
    AddTestCase(new HostSpineRedirectTest, TestCase::Duration::QUICK);
    AddTestCase(new AckRedirectScopeTest, TestCase::Duration::QUICK);
    AddTestCase(new SwitchQueueStateTest, TestCase::Duration::QUICK);
    AddTestCase(new SwitchQueueGradeTest, TestCase::Duration::QUICK);
    AddTestCase(new RepsOracleTest, TestCase::Duration::QUICK);
    AddTestCase(new RepsSpineTest, TestCase::Duration::QUICK);
}

static PointToPointTestSuite g_pointToPointTestSuite; //!< The testsuite
