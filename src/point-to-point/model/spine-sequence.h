#ifndef SPINE_SEQUENCE_H
#define SPINE_SEQUENCE_H

#include <stdint.h>
#include <unordered_map>
#include <vector>

namespace ns3 {

// The sending side of a queue pair's path sequences: per requested spine, a
// 16-bit counter of the data packets it has sent up that spine, new data or
// repair alike. Each data packet carries the counter's value before its send
// in the data header's path_seq field; the first packet up a spine carries 0.
class SpineSequenceSender {
public:
	// The path sequence of the next data packet requested up spine, after
	// which the counter of spine moves on by one, wrapping at 2^16.
	uint16_t Next(uint8_t spine);
	// The data packets sent up spine so far, modulo 2^16: the path sequence
	// the next of them will carry.
	uint16_t Count(uint8_t spine) const;

private:
	std::vector<uint16_t> m_next;
};

// The receiving side of a queue pair's path sequences: the next path sequence
// expected per path, a path being the pair of the requested spine and the
// carrying spine an arrival's identification names. An arrival whose path
// sequence is ahead of the expected one by g reveals g packets of that path
// that did not arrive (a gap), charged to the carrying spine. An arrival
// behind the expected one, in 16-bit serial number order, is a late packet
// and changes nothing.
//
// A path whose requested and carrying spines are equal starts expecting 0,
// as the sender's counter starts. A packet a source leaf moved from its
// requested spine j onto carrying spine k is tracked on the path (j, k), so
// that it is charged to the spine that carried it without being taken for a
// gap in the sequence of the packets requested up k; that path starts at its
// first arrival, which reveals no gap. Packets requested up j and carried by
// j again after a move are compared with the last of them carried by j, so
// the packets k carried meanwhile count as a gap on j.
class SpineSequenceReceiver {
public:
	struct Arrival {
		// The packets of the path the arrival shows missing, zero for none.
		uint16_t gaps;
		// Whether the arrival is behind the expected sequence.
		bool late;
	};
	// An arrival with path sequence pathSeq, requested up requested and
	// carried by carrying.
	Arrival OnArrival(uint8_t requested, uint8_t carrying, uint16_t pathSeq);
	// A run of path sequences of one requested spine: count of them from
	// first.
	struct Missing {
		uint16_t first;
		uint16_t count;
	};
	// The path sequences missing of the owed packets the sender reports it
	// sent up requested: from the furthest expected sequence among the
	// paths requested up requested, or from the end of the last run taken
	// where that is further, up to owed. A run taken is not taken again,
	// and an owed count not ahead of that start in 16-bit serial order
	// takes nothing.
	Missing TakeMissing(uint8_t requested, uint16_t owed);

private:
	struct Expected {
		uint16_t next;
		bool started;
	};
	// Keyed by the identification that names the path.
	std::unordered_map<uint16_t, Expected> m_expected;
	// Per requested spine, the end of the runs TakeMissing has taken.
	std::unordered_map<uint8_t, uint16_t> m_taken;
};

} // namespace ns3

#endif /* SPINE_SEQUENCE_H */
