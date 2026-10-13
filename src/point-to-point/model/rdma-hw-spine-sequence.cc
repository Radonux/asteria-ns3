// RdmaHw's path sequences (SprayPathSeq), the tail recovery (SprayTailRecovery)
// and the receiver's hold (SprayReceiverHold) built on them, kept apart from rdma-hw.cc, which calls them from
// its data and acknowledgement paths.

#include "rdma-hw.h"
#include "load-balancing.h"
#include "path-selector.h"
#include <ns3/simulator.h>

namespace ns3 {

bool RdmaHw::ValidPathSeq() const{
	return CustomHeader::udpCarriesPathSeq == m_sprayPathSeq &&
		(!m_sprayPathSeq || m_loadBalancing ==
			static_cast<uint32_t>(LoadBalancingMode::SprayUniform));
}

void RdmaHw::CountPathSeq(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch,
		bool trimmed){
	if (m_sprayTailRecovery && (ch.udp.tail_flags & kTailCloses))
		AskTailMissing(q, ch);
	const uint8_t requested = RequestedSpine(ch.ipid);
	const uint8_t carrying = CarryingSpine(ch.ipid);
	const SpineSequenceReceiver::Arrival arrival =
		q->m_pathSeqReceiver.OnArrival(requested, carrying, ch.udp.path_seq);
	if (carrying >= m_spineGaps.size())
		m_spineGaps.resize(carrying + 1, 0);
	m_spineGaps[carrying] += arrival.gaps;
	if (!IsReceiverHold())
		return;
	const uint32_t leaf = m_switchQueues->LeafOf(ch.sip);
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	if (trimmed)
		ReceiverHoldOf().OnTrim(leaf, carrying, arrival.gaps, now);
	else
		ReceiverHoldOf().OnArrival(leaf, carrying, arrival.gaps, arrival.late,
			now);
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
		const SpineSequenceReceiver::Missing missing =
			q->m_pathSeqReceiver.TakeMissing(spine, owed);
		if (missing.count > 0)
			SendTailNack(ch, spine, missing.first, missing.count);
	}
}

void RdmaHw::SendTailNack(const CustomHeader &ch, uint8_t spine,
		uint16_t first, uint16_t count){
	qbbHeader request;
	request.SetPG(ch.udp.pg);
	request.SetSport(ch.udp.dport);
	request.SetDport(ch.udp.sport);
	request.SetTailNack(spine, first, count);
	SendRepairRequest(request, ch.dip, ch.sip,
		SpineIdentification(spine, spine));
}

void RdmaHw::RecoverTail(Ptr<RdmaQueuePair> qp, const CustomHeader &ch){
	if (!qp->m_outstanding.IsKept() || EnforceProgressDeadline(qp))
		return;
	const uint64_t lost = qp->DeclareLostOnPathSeqs(ch.ack.tail_spine,
		ch.ack.tail_seq, ch.ack.tail_count);
	if (lost == 0)
		return;
	if (m_cc_mode == 11 && DeliverCongestionSignal(qp)){
		qp->nscc.OnLoss(lost);
		ApplyNsccWindow(qp);
	}
	m_nic[GetNicIdxOfQp(qp)].dev->TriggerTransmit();
	ArmRetransmissionTimeout(qp);
}

bool RdmaHw::IsReceiverHold() const{
	return m_sprayReceiverHold && IsRepsHold();
}

bool RdmaHw::ValidReceiverHold() const{
	return CustomHeader::ackReceiverHoldBytes == (m_sprayReceiverHold ? 1u : 0u) &&
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

uint8_t RdmaHw::ReceiverHoldBitmap(uint32_t senderIp) const{
	if (!IsReceiverHold() || !m_receiverHold)
		return 0;
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
