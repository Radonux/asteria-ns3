#ifndef SWITCH_QUEUE_STATE_H
#define SWITCH_QUEUE_STATE_H

#include <ns3/ptr.h>
#include <ns3/simple-ref-count.h>
#include "spine-grader.h"
#include "switch-node.h"
#include <stdint.h>
#include <unordered_map>
#include <vector>

namespace ns3 {

// Where a receiver's per-spine values come from under spray_policy: its own
// estimate from arrivals, or the switches' state.
enum class GradeCostSource : uint32_t {
	Estimate = 0,
	SwitchQueue,
};

// Simulation only: the state of the ports the paths of a leaf-spine fabric
// cross, read from the simulator's switch and device objects at the instant
// it is asked for. No host can read it.
//
// A path from source leaf s to destination leaf d over spine k crosses two
// hops: s's uplink to k and k's downlink to d. Each hop is sent by one port,
// whose data queue occupancy, ECN marking profile and rate it reads, and
// received by another, whose link error rate, or black hole, it reads.
class SwitchQueueState : public SimpleRefCount<SwitchQueueState> {
public:
	static constexpr uint32_t kNoLeaf = UINT32_MAX;
	struct Hop {
		Ptr<SwitchNode> sender;
		uint32_t sendingPort;
		Ptr<SwitchNode> receiver;
		uint32_t receivingPort;
	};
	SwitchQueueState(uint32_t leaves, uint32_t spines);
	void SetUplink(uint32_t leaf, uint32_t spine, const Hop &hop);
	void SetDownlink(uint32_t spine, uint32_t leaf, const Hop &hop);
	void SetLeafOf(uint32_t address, uint32_t leaf);
	// The leaf of the host at address, or kNoLeaf.
	uint32_t LeafOf(uint32_t address) const;
	uint32_t Leaves() const;
	// Per spine, the values of the path from leaf source to leaf destination
	// as the switches stand now: the marked fraction 1 - (1 - p_up)(1 - p_down)
	// from the marking probability of each hop's data queue occupancy, no
	// trimmed fraction, with delay the queueing delay 8 q / r summed over the
	// forward hops, and the loss fraction
	// 1 - (1 - l_up)(1 - l_down)(1 - l_rup)(1 - l_rdown) from each receiving
	// port's link error rate, 1 where the port is black-holed, over the
	// forward hops and the reverse hops, leaf destination's uplink to the
	// spine and the spine's downlink to leaf source. The loss fraction is
	// that of a packet delivered and acknowledged: the ACK echoes the spine
	// that carried the packet and returns through the same spine, so a loss
	// on a reverse hop drops the ACK and the sender's loss rule declares the
	// delivered packet lost. The marked fraction and the delay are the
	// forward hops' only, as the ACK echoes the packet's marking.
	std::vector<SpineGrader::SpineValues> PathValues(uint32_t source,
		uint32_t destination, bool delay) const;

private:
	struct HopValues {
		double marking;
		double delayNs;
		double loss;
	};
	static HopValues Read(const Hop &hop);

	uint32_t m_spines;
	// Indexed leaf * spines + spine.
	std::vector<Hop> m_uplinks;
	std::vector<Hop> m_downlinks;
	std::unordered_map<uint32_t, uint32_t> m_leafOf;
};

} // namespace ns3

#endif /* SWITCH_QUEUE_STATE_H */
