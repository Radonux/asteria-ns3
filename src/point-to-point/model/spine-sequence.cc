#include "spine-sequence.h"
#include "load-balancing.h"

namespace ns3 {

uint16_t SpineSequenceSender::Next(uint8_t spine){
	if (spine >= m_next.size())
		m_next.resize(spine + 1, 0);
	return m_next[spine]++;
}

SpineSequenceReceiver::Arrival SpineSequenceReceiver::OnArrival(
		uint8_t requested, uint8_t carrying, uint16_t pathSeq){
	Expected &expected = m_expected.try_emplace(
		SpineIdentification(requested, carrying),
		Expected{0, requested == carrying}).first->second;
	if (!expected.started){
		expected.started = true;
		expected.next = static_cast<uint16_t>(pathSeq + 1);
		return Arrival{0, false};
	}
	// Serial number arithmetic on 16 bits: a distance below 2^15 is ahead.
	const uint16_t ahead = static_cast<uint16_t>(pathSeq - expected.next);
	if (ahead >= 0x8000)
		return Arrival{0, true};
	expected.next = static_cast<uint16_t>(pathSeq + 1);
	return Arrival{ahead, false};
}

} // namespace ns3
