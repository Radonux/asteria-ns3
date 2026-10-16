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
	// As SprayUniform on the wire and at every switch, but the sender draws
	// each spine from scores built out of the grades every acknowledgement
	// from the receiver carries.
	SprayPolicy,
};

// Whether a data packet's identification names the spine it is sent up.
inline bool NamesSpine(LoadBalancingMode mode){
	return mode == LoadBalancingMode::SprayUniform ||
		mode == LoadBalancingMode::SprayPolicy;
}

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
// sent. Where the identification names a spine it is the requested spine: a
// source leaf moves every request for a spine whose uplink is down onto the
// same live spine, so packets with one request still share their queues. Under
// EntropyHash it is the whole entropy value, which every switch hashes alike.
inline uint16_t PathOf(LoadBalancingMode mode, uint16_t identification){
	return NamesSpine(mode) ? RequestedSpine(identification) : identification;
}

// How the receiver of a data packet picks the spine of each packet it sends
// back to the sender: an acknowledgement, a NACK or a repair request. Under
// both the answer returns the data packet's identification, whose spines the
// sender reads; only the spine the switches route it over differs.
// SprayDuplicateReport, under SprayCoverageFirst: a receiver flags its answer
// to a data packet whose sequence it held already (qbbHeader::FLAG_DUPLICATE),
// and a sender reads the flag on the answer to a timed-out send's resend.
// Defer tells the selector a timeout's loss only at that answer, none where it
// is flagged; Retract tells it at once and takes it back where the answer is
// flagged (RdmaQueuePair::m_retractTimeoutEvidence).
// SprayOverdueRepair, under a loss hold selector: a queue pair's latest send
// on a spine not held, its answer SprayHeldLossRtts base RTTs overdue, is
// repaired. Evidence declares it lost as a timed-out send is; Probe counts no
// loss as it repairs, and under SprayDuplicateReport the repair's answer
// settles the send (RdmaQueuePair::m_overdueProbe).
enum class OverdueRepair : uint32_t {
	Off = 0,
	Evidence,
	Probe,
};

enum class DuplicateReport : uint32_t {
	Off = 0,
	Defer,
	Retract,
};

enum class AckRoute : uint32_t {
	// Over the spine that carried the data packet it answers, which the
	// identification names.
	DataSpine = 0,
	// Over a spine the receiver draws uniformly per answer, named in the
	// answer's route_spine field.
	Spray,
	// Over a member, drawn uniformly per answer, of a set of spines the
	// receiver keeps per connection, probes and renews (AnswerRouteSet),
	// named in route_spine.
	ReverseSet,
	// Over one spine the receiver keeps per connection and draws again at a
	// duplicate data arrival (AnswerRepath), named in route_spine.
	Repath,
};

// The value of an answer's route_spine field that names no spine: the
// answer is routed by the carrying spine its identification names. Spine
// indices stay below 256, the identification naming each in a byte; a field
// of one byte (SpineFieldBytes) writes it as 255.
constexpr uint16_t kNoRouteSpine = 0xffff;

// The value of an acknowledgement's redirect field that names no spine.
constexpr uint16_t kNoRedirect = 0xffff;

// The spine an acknowledgement's redirect field names among spines, or
// kNoRedirect. A receiver writes only a spine's index or kNoRedirect, so any
// other value names no spine.
inline uint16_t DecodeRedirect(uint16_t field, uint32_t spines){
	return field < spines ? field : kNoRedirect;
}

} /* namespace ns3 */

#endif /* LOAD_BALANCING_H */
