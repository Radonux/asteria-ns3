#include "switch-queue-state.h"
#include "qbb-net-device.h"
#include <ns3/assert.h>
#include <ns3/error-model.h>
#include <ns3/pointer.h>
#include <algorithm>

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
		const auto [up, down, reverseUp, reverseDown] =
			PathHops(source, destination, k);
		// The probability that a packet crossing both forward hops arrives
		// marked; the ACK echoes that marking.
		values[k].markFraction = 1 - (1 - up.marking) * (1 - down.marking);
		values[k].delayNs = delay ? up.delayNs + down.delayNs : 0;
		// The probability that the packet or its ACK is lost on one of the
		// four hops.
		values[k].lossFraction = 1 - (1 - up.loss) * (1 - down.loss)
			* (1 - reverseUp.loss) * (1 - reverseDown.loss);
	}
	return values;
}

std::vector<bool> SwitchQueueState::Faulted(uint32_t source,
		uint32_t destination) const{
	std::vector<bool> faulted(m_spines);
	std::vector<uint64_t> slowest(m_spines, UINT64_MAX);
	for (uint32_t k = 0; k < m_spines; k++){
		for (const HopValues &hop : PathHops(source, destination, k)){
			faulted[k] = faulted[k] || hop.loss > 0;
			slowest[k] = std::min(slowest[k], hop.rateBps);
		}
	}
	// A rate is a fault only against the other spines': the fabric's own
	// rate is no fault.
	const uint64_t fastest = *std::max_element(slowest.begin(), slowest.end());
	for (uint32_t k = 0; k < m_spines; k++)
		faulted[k] = faulted[k] || slowest[k] < fastest;
	return faulted;
}

std::array<SwitchQueueState::HopValues, 4> SwitchQueueState::PathHops(
		uint32_t source, uint32_t destination, uint32_t k) const{
	// The forward hops, then those the ACK returns over through the same
	// spine.
	return {Read(m_uplinks[source * m_spines + k]),
		Read(m_downlinks[destination * m_spines + k]),
		Read(m_uplinks[destination * m_spines + k]),
		Read(m_downlinks[source * m_spines + k])};
}

SwitchQueueState::HopValues SwitchQueueState::Read(const Hop &hop){
	NS_ASSERT_MSG(hop.sender && hop.receiver, "every hop of the fabric is set");
	HopValues values{0, 0, 0, 0};
	const uint32_t bytes = hop.sender->DataQueueBytes(hop.sendingPort);
	if (hop.sender->IsEcnEnabled())
		values.marking = hop.sender->m_mmu->MarkingProbability(hop.sendingPort,
			bytes);
	const Ptr<QbbNetDevice> sending =
		DynamicCast<QbbNetDevice>(hop.sender->GetDevice(hop.sendingPort));
	values.rateBps = sending->GetDataRate().GetBitRate();
	// Bits over bits per nanosecond.
	values.delayNs = 8.0 * bytes * 1e9 / values.rateBps;
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
