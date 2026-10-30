#ifndef SPINE_SEQUENCE_H
#define SPINE_SEQUENCE_H

#include <stdint.h>
#include <set>
#include <unordered_map>
#include <vector>

namespace ns3 {

// How a receiving host reads the path sequences of a packet that arrives
// after the frontend removed its flow's receive queue pair at the flow's
// completion (RdmaHw::DeleteRxQp): a copy, a repaired send or a repair still
// in flight, which makes a receive queue pair of its own. Fresh: that queue
// pair reads them as a new flow's, from nothing, so the tail counts of a
// closing packet among them name as missing every send of the flow it has
// not seen itself. Closed: they are read against the removed queue pair, the
// flow's own: the packet takes its sequence out of that queue pair's tail
// runs still waiting, and is reported late where that queue pair counted its
// sequence missing, and counts no gap, asks no tail repair and counts nothing
// in the receiver's hold.
enum class StrayPathSeq : uint32_t {
	Fresh = 0,
	Closed,
};

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
// carrying spine an arrival's identification names, and the path sequences of
// the path's trimmed headers at or beyond the expected one (accounted).
//
// Data packets are ordered within a path, so only data arrivals count gaps.
// A data arrival whose path sequence s is ahead of the expected one e reveals
// the sequences in [e, s) that are not accounted as missing (a gap), charged
// to the carrying spine, and the path expects s + 1 next; the accounted
// sequences it passes are dropped. A data arrival behind the expected one, in
// 16-bit serial number order, is a late packet and changes nothing.
//
// The sequences TakeTailMissing counts missing are accounted as well, since
// they are counted already. No data packet arrives with a trimmed header's
// sequence, whose payload was dropped, so a data arrival of an accounted
// sequence is one TakeTailMissing counted, arriving after all: it is a late
// packet too and changes nothing.
//
// Trimmed headers travel in the switches' high-priority queue and can overtake
// earlier data packets of their path. A trimmed header therefore neither moves
// the expected sequence on nor counts a gap: it only marks its path sequence
// as accounted, so that the data arrival that passes it does not count it
// missing. A path keeps at most kAccountedWindow accounted sequences, those
// from the expected one up to kAccountedWindow - 1 ahead of it; a trimmed
// header further ahead is not recorded and its sequence counts as missing.
//
// A path whose requested and carrying spines are equal starts expecting 0,
// as the sender's counter starts. A packet a source leaf moved from its
// requested spine j onto carrying spine k is tracked on the path (j, k), so
// that it is charged to the spine that carried it without being taken for a
// gap in the sequence of the packets requested up k; that path starts at its
// first data arrival, which reveals no gap. Packets requested up j and
// carried by j again after a move are compared with the last of them carried
// by j, so the packets k carried meanwhile count as a gap on j.
class SpineSequenceReceiver {
public:
	// The accounted sequences a path keeps, from its expected one on.
	static constexpr uint16_t kAccountedWindow = 4096;
	struct Arrival {
		// The packets of the path the arrival shows missing, zero for none.
		uint16_t gaps;
		// Whether the arrival is behind the expected sequence, or one
		// TakeTailMissing counted.
		bool late;
		// Whether it is one TakeTailMissing counted.
		bool counted;
	};
	// A data arrival with path sequence pathSeq, requested up requested and
	// carried by carrying.
	Arrival OnArrival(uint8_t requested, uint8_t carrying, uint16_t pathSeq);
	// A trimmed header with path sequence pathSeq, requested up requested
	// and carried by carrying: its sequence is accounted where it is at or
	// beyond the path's expected one and within kAccountedWindow of it.
	// Whether TakeTailMissing counted it on the path: a send is trimmed once,
	// so a sequence its path accounted already was counted.
	bool OnTrim(uint8_t requested, uint8_t carrying, uint16_t pathSeq);
	// Whether pathSeq is accounted on any path requested up requested.
	bool Accounted(uint8_t requested, uint16_t pathSeq) const;
	// A run of path sequences of one requested spine: count of them from
	// first.
	struct Missing {
		uint16_t first;
		uint16_t count;
	};
	// The path sequences missing of the owed packets the sender reports it
	// sent up requested: from the furthest expected sequence among the
	// paths requested up requested, or from the end of the last runs taken
	// where that is further, up to owed, less the accounted sequences of
	// those paths, one run per stretch of consecutive sequences. Sequences
	// taken are not taken again, and an owed count not ahead of that start
	// in 16-bit serial order takes nothing.
	std::vector<Missing> TakeMissing(uint8_t requested, uint16_t owed);
	// Count missing the count path sequences from first, requested up
	// requested, that a receiver found missing and that neither arrived nor
	// were trimmed since: those no data arrival requested up requested has
	// passed, as one that did counted them as a gap. Each sequence counted is
	// accounted on every path requested up requested, the path requested up
	// it and carried by it made where there is none yet, so that a data
	// arrival ahead of it counts it no more and its own is late. Returns how
	// many it counted.
	uint16_t TakeTailMissing(uint8_t requested, uint16_t first, uint16_t count);

private:
	struct Path {
		uint16_t next;
		bool started;
		// The accounted path sequences, each at or beyond next, at most
		// kAccountedWindow of them; before the path starts, those recorded.
		std::set<uint16_t> accounted;
	};
	// Drop the accounted sequences of path that are behind its expected one.
	static void Prune(Path &path);
	// Account pathSeq on path where it is at or beyond the expected sequence
	// and within kAccountedWindow of it, or before the path starts while it
	// keeps fewer than kAccountedWindow.
	static void Account(Path &path, uint16_t pathSeq);
	// The furthest expected sequence among the started paths requested up
	// requested, in next; false, with next left as it is, where none is.
	bool Furthest(uint8_t requested, uint16_t &next) const;
	// An identification's hash, its requested spine in the low bits and its
	// carrying spine above. Hashed as the identity, the path of each spine k
	// whose requested and carrying spines are equal, identification 257 k,
	// would take one bucket of the table's 257, which it has over more than
	// 127 paths; this way no two of them share a bucket.
	struct IdentificationHash {
		size_t operator()(uint16_t identification) const{
			return (identification >> 8) |
				static_cast<size_t>(identification & 0xff) << 16;
		}
	};
	// Keyed by the identification that names the path.
	std::unordered_map<uint16_t, Path, IdentificationHash> m_paths;
	// Per requested spine, the end of the runs TakeMissing has taken.
	std::unordered_map<uint8_t, uint16_t> m_taken;
};

// The runs of missing path sequences a receiver under tail recovery waits on
// before it requests them: per run its requested spine, its path sequences
// and when its request is due. A packet of the run that arrives meanwhile is
// taken out of it, so that only what is still missing when the run is due
// is requested.
class PendingTailRuns {
public:
	static constexpr uint64_t kNone = UINT64_MAX;
	struct Request {
		uint8_t spine;
		uint16_t first;
		uint16_t count;
	};
	void Add(uint8_t spine, uint16_t first, uint16_t count, uint64_t dueNs);
	// A data arrival, or a trimmed header, requested up spine with path
	// sequence pathSeq.
	void OnArrival(uint8_t spine, uint16_t pathSeq);
	// Remove the runs due by nowNs and return what is still missing of them,
	// one request per stretch of consecutive path sequences, in the order
	// the runs were added.
	std::vector<Request> TakeDue(uint64_t nowNs);
	// When the earliest run is due, or kNone.
	uint64_t NextDue() const;
	bool Empty() const;

private:
	struct Run {
		uint8_t spine;
		uint16_t first;
		uint64_t dueNs;
		// Per path sequence from first, whether it is still missing.
		std::vector<bool> missing;
	};
	std::vector<Run> m_runs;
};

} // namespace ns3

#endif /* SPINE_SEQUENCE_H */
