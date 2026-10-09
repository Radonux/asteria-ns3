#ifndef LOAD_BALANCING_H
#define LOAD_BALANCING_H

#include <stdint.h>

namespace ns3 {

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

} /* namespace ns3 */

#endif /* LOAD_BALANCING_H */
