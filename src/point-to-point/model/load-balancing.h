#ifndef LOAD_BALANCING_H
#define LOAD_BALANCING_H

#include <stdint.h>

namespace ns3 {

// How a data packet's uplink is chosen. The mode is global to a run: the
// sender decides what it writes into the IPv4 identification field, and every
// switch decides what it reads from it.
enum class LoadBalancingMode : uint32_t {
	// The four-tuple hash. The identification is a per-QP counter that no
	// switch reads, so every packet of a flow takes one path.
	Ecmp = 0,
	// The four-tuple hash extended by the identification, which the sender
	// fills with a fresh 16-bit entropy value per packet.
	EntropyHash,
	// The identification names a spine per packet, drawn uniformly by the
	// sender, and the source leaf sends the packet up that spine's uplink.
	SprayUniform,
};

// The IPv4 identification of a data packet that names its spine: the high
// byte is the spine the sender requested and the low byte the spine that
// carried the packet. The sender writes the two equal and only a source leaf
// whose requested uplink is down rewrites the low byte, so the bytes differ
// exactly when the packet was moved to another spine.
inline uint16_t SpineIdentification(uint8_t requested, uint8_t carrying){
	return static_cast<uint16_t>(requested << 8 | carrying);
}

inline uint8_t RequestedSpine(uint16_t identification){
	return static_cast<uint8_t>(identification >> 8);
}

inline uint8_t CarryingSpine(uint16_t identification){
	return static_cast<uint8_t>(identification & 0xff);
}

// The part of a data packet's identification that fixes the queues it passes
// through, so that packets with equal values arrive in the order they were
// sent. Under SprayUniform it is the requested spine: a source leaf moves
// every request for a spine whose uplink is down onto the same live spine, so
// packets with one request still share their queues. Under EntropyHash it is
// the whole entropy value, which every switch hashes alike.
inline uint16_t PathOf(LoadBalancingMode mode, uint16_t identification){
	return mode == LoadBalancingMode::SprayUniform
		? RequestedSpine(identification) : identification;
}

} /* namespace ns3 */

#endif /* LOAD_BALANCING_H */
