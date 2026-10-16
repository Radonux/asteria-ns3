// RdmaHw's per-connection answer routes, AckRoute ReverseSet and Repath, kept
// apart from rdma-hw.cc, which calls them from its answer and data paths.

#include "rdma-hw.h"
#include "load-balancing.h"
#include "ns3/ppp-header.h"
#include <ns3/ipv4-header.h>
#include <ns3/simulator.h>
#include <algorithm>

namespace ns3 {

bool RdmaHw::ValidAnswerRoutes() const{
	if (m_ackRoute == static_cast<uint32_t>(AckRoute::ReverseSet))
		return m_ackRouteSetSize >= 1 && m_ackRouteSetSize <= m_spineCount;
	if (m_ackRoute == static_cast<uint32_t>(AckRoute::Repath))
		return m_sprayTailRecovery;
	return true;
}

RdmaHw::AnswerRoutes &RdmaHw::AnswerRoutesOf(uint32_t senderIp, uint16_t pg){
	return m_answerRoutes[{senderIp, pg}];
}

uint32_t RdmaHw::DrawAnswerRoute(uint32_t n){
	return m_ackRouteRandom->GetInteger(0, n - 1);
}

bool RdmaHw::AnswersCrossSpine(uint32_t senderIp) const{
	return !std::binary_search(m_ownLeafHosts.begin(), m_ownLeafHosts.end(),
		senderIp);
}

void RdmaHw::ProbeAnswerSet(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch){
	if (!AnswersCrossSpine(ch.sip))
		return;
	NS_ABORT_MSG_IF(m_sprayBaseRttNs == 0,
		"AckRoute 2 probes its sets once per SprayBaseRttNs");
	std::vector<uint8_t> probes;
	std::vector<std::pair<uint8_t, uint8_t>> replaced;
	if (!AnswerRoutesOf(ch.sip, ch.udp.pg).set.TakeRound(
			Simulator::Now().GetNanoSeconds(), m_sprayBaseRttNs,
			static_cast<uint64_t>(kAnswerSetProbeTimeoutRtts * m_sprayBaseRttNs),
			m_spineCount, m_ackRouteSetSize,
			[this](uint32_t n){ return DrawAnswerRoute(n); }, probes, replaced))
		return;
	for (size_t k = 0; k < replaced.size(); k++)
		ReportTransportEvent("answer_set_replace", 0);
	for (const uint8_t spine : probes)
		SendRouteProbe(q, spine);
}

void RdmaHw::SendRouteProbe(Ptr<RdmaRxQueuePair> q, uint8_t spine){
	// The receive queue pair's addresses and ports are the data's
	// destination's first, as an answer's are.
	qbbHeader probe;
	probe.SetPG(q->m_ecn_source.qIndex);
	probe.SetSport(q->sport);
	probe.SetDport(q->dport);
	probe.SetRouteProbe();
	probe.SetRouteSpine(spine);
	Ptr<Packet> packet = Create<Packet>(
		std::max(60 - 14 - 20 - static_cast<int>(probe.GetSerializedSize()), 0));
	packet->AddHeader(probe);
	Ipv4Header ip;
	ip.SetSource(Ipv4Address(q->sip));
	ip.SetDestination(Ipv4Address(q->dip));
	ip.SetProtocol(0xFC);
	ip.SetTtl(64);
	ip.SetDscp(static_cast<Ipv4Header::DscpType>(kUetDscpControl));
	ip.SetPayloadSize(packet->GetSize());
	ip.SetIdentification(SpineIdentification(spine, spine));
	packet->AddHeader(ip);
	AddHeader(packet, 0x800);
	const uint32_t nic = GetNicIdxOfRxQp(q);
	m_nic[nic].dev->RdmaEnqueueHighPrioQ(packet);
	m_nic[nic].dev->TriggerTransmit();
	ReportTransportEvent("answer_set_probe", 0);
}

void RdmaHw::ReplyRouteProbe(const CustomHeader &ch){
	if (ch.ack.route_spine >= m_spineCount)
		return;
	// Back to the probing receiver as its sender's path probes reach it:
	// the data's ports, the data's addresses.
	qbbHeader reply;
	reply.SetPG(ch.ack.pg);
	reply.SetSport(ch.ack.dport);
	reply.SetDport(ch.ack.sport);
	reply.SetRouteProbeReply();
	reply.SetRouteSpine(ch.ack.route_spine);
	Ptr<Packet> packet = Create<Packet>(
		std::max(60 - 14 - 20 - static_cast<int>(reply.GetSerializedSize()), 0));
	packet->AddHeader(reply);
	Ipv4Header ip;
	ip.SetSource(Ipv4Address(ch.dip));
	ip.SetDestination(Ipv4Address(ch.sip));
	ip.SetProtocol(kPathProbeProtocol);
	ip.SetTtl(64);
	ip.SetPayloadSize(packet->GetSize());
	ip.SetIdentification(
		SpineIdentification(ch.ack.route_spine, ch.ack.route_spine));
	packet->AddHeader(ip);
	AddHeader(packet, 0x800);
	auto route = m_rtTable.find(ch.sip);
	if (route == m_rtTable.end() || route->second.empty())
		return;
	m_nic[route->second[0]].dev->RdmaEnqueueHighPrioQ(packet);
	m_nic[route->second[0]].dev->TriggerTransmit();
}

void RdmaHw::RepathAnswers(uint32_t senderIp, uint16_t pg){
	if (!AnswersCrossSpine(senderIp))
		return;
	AnswerRoutesOf(senderIp, pg).repath.Repath(m_spineCount,
		[this](uint32_t n){ return DrawAnswerRoute(n); });
	ReportTransportEvent("answer_repath", 0);
}

} // namespace ns3
