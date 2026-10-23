// Themis (Liu et al., "Enabling Packet Spraying over Commodity RNICs with
// In-Network Support", EuroSys 2026) as PathSelector 11: the source sprays
// by packet sequence number and avoids a path a Path Avoidance Signal names
// (ThemisSelector); the destination, the receiving host standing in for its
// top-of-rack switch, judges each queue pair from its own arrivals.

#include "rdma-hw.h"
#include "load-balancing.h"
#include "path-selector.h"
#include <ns3/simulator.h>
#include <algorithm>
#include <string>

namespace ns3 {

bool RdmaHw::IsThemis() const{
	return m_pathSelectorKind ==
		static_cast<uint32_t>(PathSelectorKind::Themis);
}

uint32_t RdmaHw::ThemisPaths() const{
	return m_sprayThemisPaths > 0 ? m_sprayThemisPaths : m_spineCount;
}

bool RdmaHw::ValidThemis() const{
	if (!IsThemis())
		return m_sprayThemisPaths == 0 && m_sprayThemisOooThreshold == 448 &&
			m_sprayThemisStashMinNs == 0 &&
			m_sprayThemisAvoidanceWindow == 2000000 && m_sprayThemisRetxReroute;
	const uint32_t paths = ThemisPaths();
	return m_loadBalancing ==
			static_cast<uint32_t>(LoadBalancingMode::SprayUniform) &&
		m_selective_retransmission && m_spineCount > 0 && paths <= m_spineCount &&
		m_spineCount % paths == 0;
}

// Algorithm 1's onDataPacket and section 4.4, packets numbered from q's first
// and paths indexed by packet mod N, which groups q's packets as the source's
// paths do. A repair request the receiving NIC would send for the head of a
// gap is confirmed when a later packet of the head's path has arrived, and is
// otherwise stashed; the stash ends when the head arrives or a later packet of
// its path does, and a packet more than the threshold past it, once the stash
// is SprayThemisStashMinNs old, turns it into a Path Avoidance Signal on this
// arrival's acknowledgement, whose cumulative sequence is the head's.
bool RdmaHw::ThemisArrival(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch){
	if (q->m_stray || m_sprayThemisOooThreshold == 0)
		return false;
	RdmaRxQueuePair::ThemisDestination &state = q->m_themis;
	const uint32_t paths = ThemisPaths();
	if (state.pathMax.empty())
		state.pathMax.assign(paths, -1);
	const uint64_t nowNs = Simulator::Now().GetNanoSeconds();
	const uint64_t packet = ch.udp.seq / m_mtu;
	const uint64_t head = q->ReceiverNextExpectedSeq / m_mtu;
	int64_t &seen = state.pathMax[packet % paths];
	seen = std::max(seen, static_cast<int64_t>(packet));
	bool signal = false;
	if (state.valid){
		if (head > state.sePsn){
			state.valid = false;
			ReportTransportEvent("themis_stash_arrived", 0);
		}else if (packet % paths == state.sePsn % paths && packet > state.sePsn){
			state.valid = false;
			ReportTransportEvent("themis_stash_confirmed", 0);
		}else if (packet > state.sePsn + m_sprayThemisOooThreshold &&
				nowNs - state.stashNs >= m_sprayThemisStashMinNs){
			state.valid = false;
			signal = true;
			m_themisSignalsSent++;
			ReportTransportEvent("themis_signal_sent", 0);
		}
	}
	if (!state.valid && !signal && packet > head && head != state.requestedHead){
		state.requestedHead = head;
		if (state.pathMax[head % paths] > static_cast<int64_t>(head))
			ReportTransportEvent("themis_request_confirmed", 0);
		else{
			state.valid = true;
			state.sePsn = head;
			state.stashNs = nowNs;
			ReportTransportEvent("themis_stash", 0);
		}
	}
	return signal;
}

void RdmaHw::ThemisPathAvoid(Ptr<RdmaQueuePair> qp, uint64_t seq){
	const uint64_t nowNs = Simulator::Now().GetNanoSeconds();
	const int avoided = qp->m_pathSelector->OnPathAvoid(qp->m_psnBase + seq / m_mtu, nowNs);
	m_themisSignalsReceived++;
	ReportTransportEvent("themis_signal_received", 0);
	// The path the source avoids, through a connection's shared selector too.
	if (avoided >= 0)
		ReportTransportEvent(("themis_signal_spine" + std::to_string(avoided)).c_str(), 0);
	// The signal names the send the destination's stash waits for. A resend
	// of it left after the sender's own rule declared that send lost, so the
	// destination has not judged it: avoid the path, repair nothing more.
	const uint64_t resend = qp->m_outstanding.FindPacket(seq);
	if (resend != OutstandingPackets::kNone && qp->m_outstanding.ResendsLost(resend))
		return;
	const uint64_t lost = qp->DeclareLostSeq(seq);
	if (lost == 0)
		return;
	if (m_cc_mode == 11 && DeliverCongestionSignal(qp)){
		qp->Nscc().OnLoss(lost);
		ApplyNsccWindow(qp);
	}
	m_nic[GetNicIdxOfQp(qp)].dev->TriggerTransmit();
	ArmRetransmissionTimeout(qp);
}

} // namespace ns3
