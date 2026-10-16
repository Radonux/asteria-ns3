// RdmaHw's coverage-first loss rules (SprayCoverageFirst) and answer path
// sequences (SprayAnswerPathSeq), kept apart from rdma-hw.cc, which calls them
// from its answer and data paths.

#include "rdma-hw.h"
#include "load-balancing.h"
#include <ns3/simulator.h>
#include <string>

namespace ns3 {

bool RdmaHw::ValidCoverageFirst() const{
	return !m_sprayCoverageFirst || m_loadBalancing ==
		static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
}

void RdmaHw::ReportCoveredAnswerLoss(){
	ReportTransportEvent("answer_loss_covered", 0);
}

bool RdmaHw::ValidDuplicateReport() const{
	if (m_sprayDuplicateReport == static_cast<uint32_t>(DuplicateReport::Off))
		return true;
	return m_sprayCoverageFirst &&
		(m_sprayDuplicateReport != static_cast<uint32_t>(DuplicateReport::Retract) ||
			(IsAutoHold() &&
				m_sprayHoldRule == static_cast<uint32_t>(HoldRule::Count)));
}

void RdmaHw::ReportTimedOutResolved(bool duplicate){
	ReportTransportEvent(duplicate ? "answer_loss_duplicate"
		: "timeout_loss_data", 0);
}

void RdmaHw::ReportTimeoutLossRetracted(uint16_t route, bool released){
	if (route == kNoRouteSpine)
		ReportTransportEvent("answer_loss_duplicate", 0);
	else
		ReportTailMissing("answer_loss_duplicate", static_cast<uint8_t>(route));
	if (released)
		ReportTransportEvent("timeout_retract_release", 0);
}

bool RdmaHw::ValidAnswerPathSeq() const{
	if (CustomHeader::ackAnswerSeqBytes != (m_sprayAnswerPathSeq ? 5u : 0u) ||
			CustomHeader::udpAnswerHoldBytes != (m_sprayAnswerPathSeq ? 1u : 0u))
		return false;
	return !m_sprayAnswerPathSeq ||
		(m_ackRoute == static_cast<uint32_t>(AckRoute::Spray) &&
			m_sprayPathSeq && IsRepsHold() && m_switchQueues &&
			m_leaf != SwitchQueueState::kNoLeaf && m_spineCount <= 8);
}

AnswerRouteHold &RdmaHw::AnswerRouteHoldTo(uint32_t leaf){
	auto hold = m_answerRouteHolds.find(leaf);
	if (hold != m_answerRouteHolds.end())
		return hold->second;
	hold = m_answerRouteHolds.try_emplace(leaf, m_spineCount,
		m_sprayLossIntervalNs, m_sprayLossThreshold3,
		m_sprayAnswerHoldClean).first;
	hold->second.SetChangeCallback(
		[this, leaf](uint32_t route, bool held, uint64_t nowNs){
			ReportTransportEvent(held ? "answer_route_hold"
				: "answer_route_release", 0);
			m_traceAnswerHold(false, leaf, route, held, nowNs);
		});
	return hold->second;
}

AnswerHoldReports &RdmaHw::AnswerHoldReportsOf(){
	if (m_answerHoldReports)
		return *m_answerHoldReports;
	m_answerHoldReports = std::make_unique<AnswerHoldReports>(m_spineCount);
	m_answerHoldReports->SetChangeCallback(
		[this](uint32_t leaf, uint32_t spine, bool held, uint64_t nowNs){
			m_traceAnswerHold(true, leaf, spine, held, nowNs);
		});
	return *m_answerHoldReports;
}

void RdmaHw::NumberAnswer(Ptr<RdmaRxQueuePair> q, qbbHeader &header,
		uint8_t route){
	if (!q || route == kNoRouteSpine)
		return;
	const AnswerSequencer::Number number = q->m_answerSequencer.Next(route);
	header.SetAnswerSeq(number.seq, number.announceRoute, number.announceSeq);
}

void RdmaHw::ReadAnswerSeq(Ptr<RdmaQueuePair> qp, const CustomHeader &ch){
	if (!((ch.ack.flags >> qbbHeader::FLAG_ANSWER_NUMBERED) & 1) ||
			ch.ack.route_spine == kNoRouteSpine ||
			ch.ack.route_spine >= m_spineCount)
		return;
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	std::vector<AnswerSequenceTracker::Outcome> outcomes;
	qp->m_answerTracker.OnAnswer(ch.ack.route_spine, ch.ack.answer_seq,
		ch.ack.announce_route < m_spineCount ? ch.ack.announce_route
			: kNoAnswerRoute,
		ch.ack.announce_seq, now,
		static_cast<uint64_t>(kAnswerGuardRtts * qp->m_baseRtt), outcomes);
	AnswerRouteHold &hold = AnswerRouteHoldTo(m_switchQueues->LeafOf(ch.sip));
	for (const AnswerSequenceTracker::Outcome &outcome : outcomes){
		if (outcome.lost > 0){
			hold.OnLost(outcome.route, outcome.lost, now);
			for (uint16_t i = 0; i < outcome.lost; i++)
				ReportTailMissing("answer_route_lost", outcome.route);
		}
		if (outcome.received)
			hold.OnReceived(outcome.route, now);
	}
}

} // namespace ns3
