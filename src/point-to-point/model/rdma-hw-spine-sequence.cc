// RdmaHw's path sequences (SprayPathSeq), the tail recovery (SprayTailRecovery)
// and the receiver's hold (SprayReceiverHold) built on them, kept apart from rdma-hw.cc, which calls them from
// its data and acknowledgement paths.

#include "rdma-hw.h"
#include "load-balancing.h"
#include "path-selector.h"
#include <ns3/simulator.h>
#include <string>

namespace ns3 {

bool RdmaHw::ValidPathSeq() const{
	return CustomHeader::udpCarriesPathSeq == m_sprayPathSeq &&
		(!m_sprayPathSeq || m_loadBalancing ==
			static_cast<uint32_t>(LoadBalancingMode::SprayUniform));
}

void RdmaHw::CountPathSeq(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch,
		bool trimmed){
	if (q->m_stray){
		ReadStrayPathSeq(q, ch);
		return;
	}
	if (m_sprayTailRecovery && (ch.udp.tail_flags & kTailCloses))
		AskTailMissing(q, ch);
	const uint8_t requested = RequestedSpine(ch.ipid);
	const uint8_t carrying = CarryingSpine(ch.ipid);
	// An arrival, or a trimmed header, whose request is not yet due needs
	// none: the data arrived or its trim is repaired as any trim. One whose
	// request's guard has not ended is not missing to the receiver's hold:
	// the data arrived, or the trim is no loss.
	if (m_sprayTailRecovery && !q->m_tailRuns.Empty())
		q->m_tailRuns.OnArrival(requested, ch.udp.path_seq);
	if (!q->m_tailMissingRuns.Empty())
		q->m_tailMissingRuns.OnArrival(requested, ch.udp.path_seq);
	if (trimmed){
		// A trimmed header can overtake earlier data of its path in the
		// switches' high-priority queue: it only accounts its sequence.
		if (q->m_pathSeqReceiver.OnTrim(requested, carrying, ch.udp.path_seq))
			ReportTailMissing("receiver_tail_missing_late", requested);
		if (IsReceiverHold())
			ReceiverHoldOf().OnTrim(m_switchQueues->LeafOf(ch.sip), carrying,
				Simulator::Now().GetNanoSeconds());
		return;
	}
	const SpineSequenceReceiver::Arrival arrival =
		q->m_pathSeqReceiver.OnArrival(requested, carrying, ch.udp.path_seq);
	if (arrival.counted)
		ReportTailMissing("receiver_tail_missing_late", requested);
	if (carrying >= m_spineGaps.size())
		m_spineGaps.resize(carrying + 1, 0);
	m_spineGaps[carrying] += arrival.gaps;
	if (!IsReceiverHold())
		return;
	ReceiverHoldOf().OnArrival(m_switchQueues->LeafOf(ch.sip), carrying,
		arrival.gaps, arrival.late, Simulator::Now().GetNanoSeconds());
}

bool RdmaHw::ValidStrayPathSeq() const{
	return m_sprayStrayPathSeq == static_cast<uint32_t>(StrayPathSeq::Fresh) ||
		m_sprayPathSeq;
}

void RdmaHw::ReadStrayPathSeq(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch){
	// The stray queue pair's addresses and ports are the data's destination's
	// first, as the removed one's were.
	const auto closed = m_closedRxQps.find(
		GetRxQpKey(q->dip, q->m_ecn_source.qIndex, q->dport));
	if (closed == m_closedRxQps.end() || !closed->second)
		return;
	const Ptr<RdmaRxQueuePair> flow = closed->second;
	const uint8_t requested = RequestedSpine(ch.ipid);
	if (!flow->m_tailRuns.Empty())
		flow->m_tailRuns.OnArrival(requested, ch.udp.path_seq);
	if (!flow->m_tailMissingRuns.Empty())
		flow->m_tailMissingRuns.OnArrival(requested, ch.udp.path_seq);
	// A send is trimmed once and a data packet never carries a trimmed
	// header's sequence, so a sequence the flow accounted, whether this is
	// the data packet or its trimmed header, is one its runs counted missing
	// (SpineSequenceReceiver::OnArrival, OnTrim).
	if (flow->m_pathSeqReceiver.Accounted(requested, ch.udp.path_seq))
		ReportTailMissing("receiver_tail_missing_late", requested);
	if (flow->m_tailRuns.Empty() && flow->m_tailMissingRuns.Empty() &&
			!flow->m_countedTailMissing)
		closed->second = nullptr;
}

bool RdmaHw::ValidTailRecovery() const{
	return CustomHeader::udpTailCounts ==
			(m_sprayTailRecovery ? m_spineCount : 0) &&
		CustomHeader::ackTailNackBytes == (m_sprayTailRecovery ? 5u : 0u) &&
		(!m_sprayTailRecovery ||
			(m_sprayPathSeq && m_selective_retransmission &&
				m_spineCount > 0 && m_spineCount <= kMaxTailCounts));
}

void RdmaHw::SetTailFields(Ptr<RdmaQueuePair> qp, SeqTsHeader &seqTs,
		bool closing) const{
	if (!closing){
		seqTs.SetTail(0, nullptr, 0);
		return;
	}
	uint16_t counts[kMaxTailCounts] = {};
	for (uint32_t k = 0; k < m_spineCount && k < kMaxTailCounts; k++)
		counts[k] = qp->m_pathSeqSender.Count(static_cast<uint8_t>(k));
	seqTs.SetTail(kTailCloses, counts, m_spineCount);
}

void RdmaHw::AskTailMissing(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch){
	const uint8_t requested = RequestedSpine(ch.ipid);
	for (uint32_t k = 0; k < CustomHeader::udpTailCounts; k++){
		const uint8_t spine = static_cast<uint8_t>(k);
		// The packet's own spine owes the packets sent up it before this
		// one, which its path sequence counts; it is counted next.
		const uint16_t owed = spine == requested
			? ch.udp.path_seq : ch.udp.tail_counts[k];
		// Accounted (trimmed) sequences are left out: a trim is repaired as
		// any trim.
		const std::vector<SpineSequenceReceiver::Missing> runs =
			q->m_pathSeqReceiver.TakeMissing(spine, owed);
		if (runs.empty())
			continue;
		const uint64_t delay = TailRecoveryDelayNs();
		for (const SpineSequenceReceiver::Missing &missing : runs){
			if (delay == 0){
				SendTailNack(q, spine, missing.first, missing.count);
				continue;
			}
			// Every run waits the same delay, so a run added later is never
			// due before the timer already set.
			q->m_tailRuns.Add(spine, missing.first, missing.count,
				Simulator::Now().GetNanoSeconds() + delay);
		}
		if (delay > 0 && !q->m_tailTimer.IsPending())
			q->m_tailTimer = Simulator::Schedule(NanoSeconds(delay),
				&RdmaHw::RequestTailRuns, this, q);
	}
}

uint64_t RdmaHw::TailRecoveryDelayNs() const{
	if (m_sprayTailRecoveryDelayNs != kTailDelayBaseRtt)
		return m_sprayTailRecoveryDelayNs;
	NS_ABORT_MSG_IF(m_sprayBaseRttNs == 0, "SprayTailRecoveryDelayNs defaults "
		"to SprayBaseRttNs, which is not set");
	return m_sprayBaseRttNs;
}

void RdmaHw::RequestTailRuns(Ptr<RdmaRxQueuePair> q){
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	for (const PendingTailRuns::Request &request : q->m_tailRuns.TakeDue(now))
		SendTailNack(q, request.spine, request.first, request.count);
	const uint64_t next = q->m_tailRuns.NextDue();
	if (next != PendingTailRuns::kNone)
		q->m_tailTimer = Simulator::Schedule(NanoSeconds(next - now),
			&RdmaHw::RequestTailRuns, this, q);
}

void RdmaHw::SendTailNack(Ptr<RdmaRxQueuePair> q, uint8_t spine,
		uint16_t first, uint16_t count){
	// The receive queue pair's addresses and ports are the data's
	// destination's first; its priority group is the one it was made with.
	qbbHeader request;
	request.SetPG(q->m_ecn_source.qIndex);
	request.SetSport(q->sport);
	request.SetDport(q->dport);
	request.SetTailNack(spine, first, count);
	SendRepairRequest(request, q->sip, q->dip,
		SpineIdentification(spine, spine), q);
	if (m_sprayReceiverTailMissing)
		WaitTailMissing(q, spine, first, count);
}

bool RdmaHw::ValidReceiverTailMissing() const{
	return !m_sprayReceiverTailMissing ||
		(m_sprayReceiverHold && m_sprayTailRecovery);
}

void RdmaHw::WaitTailMissing(Ptr<RdmaRxQueuePair> q, uint8_t spine,
		uint16_t first, uint16_t count){
	NS_ABORT_MSG_IF(m_sprayTailLossRtts > 0 && m_sprayBaseRttNs == 0,
		"SprayReceiverTailMissing counts its guard in SprayBaseRttNs, which "
		"is not set");
	const uint64_t guardNs =
		static_cast<uint64_t>(m_sprayTailLossRtts * m_sprayBaseRttNs);
	// Every run waits the same guard, so a run added later is never due
	// before the timer already set.
	q->m_tailMissingRuns.Add(spine, first, count,
		Simulator::Now().GetNanoSeconds() + guardNs);
	if (!q->m_tailMissingTimer.IsPending())
		q->m_tailMissingTimer = Simulator::Schedule(NanoSeconds(guardNs),
			&RdmaHw::CountTailMissing, this, q);
}

void RdmaHw::CountTailMissing(Ptr<RdmaRxQueuePair> q){
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	// The receive queue pair's addresses are the data's destination's first.
	const uint32_t leaf = m_switchQueues->LeafOf(q->dip);
	for (const PendingTailRuns::Request &run : q->m_tailMissingRuns.TakeDue(now)){
		const uint16_t missing = q->m_pathSeqReceiver.TakeTailMissing(run.spine,
			run.first, run.count);
		if (missing == 0)
			continue;
		q->m_countedTailMissing = true;
		ReceiverHoldOf().OnTailMissing(leaf, run.spine, missing, now);
		for (uint16_t i = 0; i < missing; i++)
			ReportTailMissing("receiver_tail_missing", run.spine);
	}
	const uint64_t next = q->m_tailMissingRuns.NextDue();
	if (next != PendingTailRuns::kNone)
		q->m_tailMissingTimer = Simulator::Schedule(NanoSeconds(next - now),
			&RdmaHw::CountTailMissing, this, q);
}

void RdmaHw::ReportTailMissing(const char *event, uint8_t spine){
	ReportTransportEvent(event, 0);
	ReportTransportEvent(
		(std::string(event) + "_spine" + std::to_string(spine)).c_str(), 0);
}

void RdmaHw::RecoverTail(Ptr<RdmaQueuePair> qp, const CustomHeader &ch){
	if (!qp->m_outstanding.IsKept() || EnforceProgressDeadline(qp))
		return;
	const uint64_t guardNs =
		static_cast<uint64_t>(m_sprayTailLossRtts * qp->m_baseRtt);
	std::vector<RdmaQueuePair::DeclaredSend> deferred;
	const uint64_t lost = qp->DeclareLostOnPathSeqs(ch.ack.tail_spine,
		ch.ack.tail_seq, ch.ack.tail_count, guardNs > 0 ? &deferred : nullptr);
	DeferTailLosses(qp, deferred, guardNs);
	if (lost == 0)
		return;
	if (m_cc_mode == 11 && DeliverCongestionSignal(qp)){
		qp->Nscc().OnLoss(lost);
		ApplyNsccWindow(qp);
	}
	m_nic[GetNicIdxOfQp(qp)].dev->TriggerTransmit();
	ArmRetransmissionTimeout(qp);
}

bool RdmaHw::IsReceiverHold() const{
	return m_sprayReceiverHold && IsRepsHold();
}

bool RdmaHw::ValidReceiverHold() const{
	return CustomHeader::ackReceiverHoldBytes ==
			(m_sprayReceiverHold ? SpineBitmapBytes(m_spineCount) : 0u) &&
		(!m_sprayReceiverHold ||
			(m_sprayPathSeq && IsRepsHold() && m_switchQueues &&
				m_spineCount > 0 && m_spineCount <= kReceiverHoldSpines));
}

ReceiverHold &RdmaHw::ReceiverHoldOf(){
	if (m_receiverHold)
		return *m_receiverHold;
	m_receiverHold = std::make_unique<ReceiverHold>(m_spineCount,
		m_sprayHoldAutoLeaves, m_sprayReceiverHoldClean, m_sprayLossIntervalNs);
	m_receiverHold->SetIntervalCallback(
		[this](uint32_t leaf, uint64_t endNs,
				const std::vector<uint32_t> &arrivals,
				const std::vector<uint32_t> &lost,
				const std::vector<ReceiverHold::Scope> &holds){
			std::vector<uint8_t> scopes(holds.size());
			for (uint32_t k = 0; k < holds.size(); k++)
				scopes[k] = static_cast<uint8_t>(holds[k]);
			m_traceReceiverHoldInterval(leaf, endNs, arrivals, lost, scopes);
		});
	return *m_receiverHold;
}

SpineBitmap RdmaHw::ReceiverHoldBitmap(uint32_t senderIp) const{
	if (!IsReceiverHold() || !m_receiverHold)
		return SpineBitmap();
	return m_receiverHold->Bitmap(m_switchQueues->LeafOf(senderIp));
}

void RdmaHw::AttachReceiverHold(Ptr<RdmaQueuePair> qp){
	// A sentinel interval of zero is the loss interval, as the loss hold's.
	const uint64_t sentinelNs = m_sprayHeldSentinelIntervalNs > 0
		? m_sprayHeldSentinelIntervalNs : m_sprayLossIntervalNs;
	std::shared_ptr<ReceiverHoldMarks> marks;
	if (m_pathSelectorKind ==
			static_cast<uint32_t>(PathSelectorKind::RepsFlowHold))
		marks = std::make_shared<ReceiverHoldMarks>(m_spineCount, sentinelNs);
	else {
		if (!m_receiverHoldMarks)
			m_receiverHoldMarks =
				std::make_shared<ReceiverHoldMarks>(m_spineCount, sentinelNs);
		marks = m_receiverHoldMarks;
	}
	const uint32_t leaf = m_switchQueues->LeafOf(qp->dip.Get());
	qp->m_pathSelector->SetReceiverHold(std::move(marks), leaf);
}

} // namespace ns3
