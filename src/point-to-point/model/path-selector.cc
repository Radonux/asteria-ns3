#include "path-selector.h"
#include "load-balancing.h"
#include <ns3/assert.h>
#include <utility>

namespace ns3 {

void PathSelector::OnAck(uint16_t, bool, uint64_t){
}

void PathSelector::OnTrim(uint16_t, bool, bool, uint64_t){
}

void PathSelector::OnLoss(uint16_t, uint64_t){
}

void PathSelector::OnTimeout(uint64_t){
}

UniformSpineSelector::UniformSpineSelector(Ptr<UniformRandomVariable> random,
		uint32_t spines)
	: m_random(random), m_spines(spines)
{
}

uint16_t UniformSpineSelector::Choose(uint64_t){
	const uint8_t spine = m_random->GetInteger(0, m_spines - 1);
	return SpineIdentification(spine, spine);
}

ObliviousSelector::ObliviousSelector(Ptr<UniformRandomVariable> random)
	: m_random(random)
{
}

uint16_t ObliviousSelector::Choose(uint64_t){
	return m_random->GetInteger(0, UINT16_MAX);
}

RepsSelector::RepsSelector(Ptr<UniformRandomVariable> random,
		uint32_t bufferSize, uint64_t freezingTimeoutNs, uint32_t explorePackets)
	: m_random(random), m_buffer(bufferSize, Entry{0, false}), m_head(0),
	  m_valid(0), m_written(0), m_frozenNext(0), m_frozen(false),
	  m_freezingTimeoutNs(freezingTimeoutNs), m_exitFreezingNs(0),
	  m_explorePackets(explorePackets), m_exploreCounter(0)
{
}

// Algorithm 2's onSend. The exploration that follows freezing mode draws every
// packet fresh, as the REPS artifact does: the pseudocode assigns one packet in
// eight and leaves the other seven unassigned.
uint16_t RepsSelector::Choose(uint64_t){
	if (m_exploreCounter > 0){
		m_exploreCounter--;
		return Fresh();
	}
	// Before the first acknowledgement, the connection's first window, and
	// whenever every cached value has been used outside freezing mode.
	if (m_written == 0 || (m_valid == 0 && !m_frozen))
		return Fresh();
	return Cached();
}

uint16_t RepsSelector::Fresh(){
	return m_random->GetInteger(0, UINT16_MAX);
}

uint16_t RepsSelector::Cached(){
	if (m_valid > 0){
		const uint32_t size = m_buffer.size();
		Entry &oldest = m_buffer[(m_head + size - m_valid) % size];
		oldest.valid = false;
		m_valid--;
		return oldest.ev;
	}
	// Frozen with nothing valid: the next held value in turn. The pseudocode
	// cycles the write position over the whole buffer; cycling over the slots
	// written so far, as the artifact does, never reads a slot that holds no
	// value.
	const uint16_t ev = m_buffer[m_frozenNext].ev;
	m_frozenNext = (m_frozenNext + 1) % m_written;
	return ev;
}

// Algorithm 1's onAck.
void RepsSelector::OnAck(uint16_t path, bool marked, uint64_t nowNs){
	if (marked)
		return;
	Entry &entry = m_buffer[m_head];
	if (!entry.valid)
		m_valid++;
	entry = Entry{path, true};
	m_head = (m_head + 1) % m_buffer.size();
	if (m_written < m_buffer.size())
		m_written++;
	if (m_frozen && nowNs > m_exitFreezingNs){
		m_frozen = false;
		m_exploreCounter = m_explorePackets;
	}
}

// Algorithm 1's onFailureDetection.
void RepsSelector::OnTimeout(uint64_t nowNs){
	if (m_frozen || m_exploreCounter != 0)
		return;
	m_frozen = true;
	m_exitFreezingNs = nowNs + m_freezingTimeoutNs;
}

EntropyRotation::EntropyRotation(Ptr<UniformRandomVariable> random,
		uint32_t size)
	: m_random(random), m_order(size), m_next(0)
{
	NS_ASSERT_MSG(size > 0 && size <= UINT16_MAX + 1,
		"an entropy value is 16 bits");
	for (uint32_t ev = 0; ev < size; ev++)
		m_order[ev] = ev;
	Shuffle();
}

uint16_t EntropyRotation::Next(){
	if (m_next == m_order.size()){
		Shuffle();
		m_next = 0;
	}
	return m_order[m_next++];
}

uint32_t EntropyRotation::Size() const{
	return m_order.size();
}

// Fisher-Yates over the whole set.
void EntropyRotation::Shuffle(){
	for (uint32_t i = m_order.size() - 1; i > 0; i--)
		std::swap(m_order[i], m_order[m_random->GetInteger(0, i)]);
}

UeObliviousSelector::UeObliviousSelector(Ptr<UniformRandomVariable> random,
		uint32_t size)
	: m_rotation(random, size)
{
}

uint16_t UeObliviousSelector::Choose(uint64_t){
	return m_rotation.Next();
}

UeAwareSelector::UeAwareSelector(Ptr<UniformRandomVariable> random,
		uint32_t size, double saturationFraction)
	: m_rotation(random, size), m_marked(size, false), m_markedCount(0),
	  m_saturationFraction(saturationFraction)
{
}

uint16_t UeAwareSelector::Choose(uint64_t){
	for (;;){
		const uint16_t ev = m_rotation.Next();
		if (!m_marked[ev])
			return ev;
		const bool saturated =
			m_markedCount > m_saturationFraction * m_rotation.Size();
		m_marked[ev] = false;
		m_markedCount--;
		if (saturated)
			return ev;
	}
}

void UeAwareSelector::OnAck(uint16_t path, bool marked, uint64_t){
	if (marked)
		Mark(path);
}

void UeAwareSelector::OnTrim(uint16_t path, bool lastHop, bool marked,
		uint64_t){
	if (!lastHop || marked)
		Mark(path);
}

void UeAwareSelector::Mark(uint16_t ev){
	NS_ASSERT_MSG(ev < m_marked.size(), "every send takes a value of the space");
	if (m_marked[ev])
		return;
	m_marked[ev] = true;
	m_markedCount++;
}

} /* namespace ns3 */
