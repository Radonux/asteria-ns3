// RdmaHw's path sequences (SprayPathSeq) and the receiver's hold built on
// them (SprayReceiverHold), kept apart from rdma-hw.cc, which calls them from
// its data and acknowledgement paths.

#include "rdma-hw.h"
#include "load-balancing.h"
#include "path-selector.h"
#include <ns3/simulator.h>

namespace ns3 {

bool RdmaHw::ValidPathSeq() const{
	return CustomHeader::udpCarriesPathSeq == m_sprayPathSeq &&
		(!m_sprayPathSeq || IsRepsHold());
}

void RdmaHw::CountPathSeq(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch,
		bool trimmed){
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
	PathSelector *selector = qp->m_pathSelector.get();
	if (auto *hold = dynamic_cast<RepsHoldSelector *>(selector))
		hold->SetReceiverHold(std::move(marks), leaf);
	else if (auto *autoHold = dynamic_cast<RepsAutoHoldSelector *>(selector))
		autoHold->SetReceiverHold(std::move(marks), leaf);
}

} // namespace ns3
