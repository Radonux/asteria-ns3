// RdmaHw's path sequences (SprayPathSeq), kept apart from rdma-hw.cc, which
// calls them from its data path.

#include "rdma-hw.h"
#include "load-balancing.h"

namespace ns3 {

bool RdmaHw::ValidPathSeq() const{
	return CustomHeader::udpCarriesPathSeq == m_sprayPathSeq &&
		(!m_sprayPathSeq || IsRepsHold());
}

void RdmaHw::CountPathSeq(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch){
	const uint8_t requested = RequestedSpine(ch.ipid);
	const uint8_t carrying = CarryingSpine(ch.ipid);
	const SpineSequenceReceiver::Arrival arrival =
		q->m_pathSeqReceiver.OnArrival(requested, carrying, ch.udp.path_seq);
	if (carrying >= m_spineGaps.size())
		m_spineGaps.resize(carrying + 1, 0);
	m_spineGaps[carrying] += arrival.gaps;
}

} // namespace ns3
