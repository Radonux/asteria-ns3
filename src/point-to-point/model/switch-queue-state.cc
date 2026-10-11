#include "switch-queue-state.h"
#include "qbb-net-device.h"
#include <ns3/assert.h>
#include <ns3/error-model.h>
#include <ns3/pointer.h>

namespace ns3 {

SwitchQueueState::SwitchQueueState(uint32_t leaves, uint32_t spines)
	: m_spines(spines), m_uplinks(leaves * spines), m_downlinks(leaves * spines)
{
}

void SwitchQueueState::SetUplink(uint32_t leaf, uint32_t spine,
		const Hop &hop){
	m_uplinks.at(leaf * m_spines + spine) = hop;
}

void SwitchQueueState::SetDownlink(uint32_t spine, uint32_t leaf,
		const Hop &hop){
	m_downlinks.at(leaf * m_spines + spine) = hop;
}

void SwitchQueueState::SetLeafOf(uint32_t address, uint32_t leaf){
	m_leafOf[address] = leaf;
}

uint32_t SwitchQueueState::LeafOf(uint32_t address) const{
	const auto found = m_leafOf.find(address);
	return found == m_leafOf.end() ? kNoLeaf : found->second;
}

uint32_t SwitchQueueState::Leaves() const{
	return m_uplinks.size() / m_spines;
}

std::vector<SpineGrader::SpineValues> SwitchQueueState::PathValues(
		uint32_t source, uint32_t destination, bool delay) const{
	std::vector<SpineGrader::SpineValues> values(m_spines);
	for (uint32_t k = 0; k < m_spines; k++){
		const HopValues up = Read(m_uplinks[source * m_spines + k]);
		const HopValues down = Read(m_downlinks[destination * m_spines + k]);
		// The probability that a packet crossing both hops arrives marked, or
		// is lost on one of them.
		values[k].markFraction = 1 - (1 - up.marking) * (1 - down.marking);
		values[k].delayNs = delay ? up.delayNs + down.delayNs : 0;
		values[k].lossFraction = 1 - (1 - up.loss) * (1 - down.loss);
	}
	return values;
}

SwitchQueueState::HopValues SwitchQueueState::Read(const Hop &hop){
	NS_ASSERT_MSG(hop.sender && hop.receiver, "every hop of the fabric is set");
	HopValues values{0, 0, 0};
	const uint32_t bytes = hop.sender->DataQueueBytes(hop.sendingPort);
	if (hop.sender->IsEcnEnabled())
		values.marking = hop.sender->m_mmu->MarkingProbability(hop.sendingPort,
			bytes);
	const Ptr<QbbNetDevice> sending =
		DynamicCast<QbbNetDevice>(hop.sender->GetDevice(hop.sendingPort));
	// Bits over bits per nanosecond.
	values.delayNs = 8.0 * bytes * 1e9 / sending->GetDataRate().GetBitRate();
	if (hop.receiver->IsBlackholed(hop.receivingPort)){
		values.loss = 1;
		return values;
	}
	PointerValue model;
	hop.receiver->GetDevice(hop.receivingPort)->GetAttribute("LinkErrorModel",
		model);
	const Ptr<RateErrorModel> rate = model.Get<RateErrorModel>();
	if (rate && rate->IsEnabled())
		values.loss = rate->GetRate();
	return values;
}

} // namespace ns3
