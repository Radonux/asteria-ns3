#include "path-selector.h"
#include "load-balancing.h"

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

} /* namespace ns3 */
