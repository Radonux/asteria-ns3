#ifndef PATH_SELECTOR_H
#define PATH_SELECTOR_H

#include <ns3/ptr.h>
#include <ns3/random-variable-stream.h>
#include <stdint.h>
#include <vector>

namespace ns3 {

// Which design chooses a data packet's entropy value under EntropyHash.
enum class PathSelectorKind : uint32_t {
	Ops = 0,
	Reps,
};

// A queue pair's choice of the path each of its data packets takes, and what
// the answers to those packets tell it. A path is what PathOf names, which
// under EntropyHash is the entropy value itself. Times are simulated
// nanoseconds. The feedback a design has no use for is ignored.
class PathSelector {
public:
	virtual ~PathSelector() = default;
	// The identification of the next data packet, new data or repair alike.
	virtual uint16_t Choose(uint64_t nowNs) = 0;
	// The acknowledgement of a send along path; marked when it echoes CE.
	virtual void OnAck(uint16_t path, bool marked, uint64_t nowNs);
	// The repair request for a send along path that a switch trimmed, at the
	// destination's last hop or before it; marked when the trimmed packet
	// carried CE.
	virtual void OnTrim(uint16_t path, bool lastHop, bool marked, uint64_t nowNs);
	// A send along path declared lost.
	virtual void OnLoss(uint16_t path, uint64_t nowNs);
	// The retransmission timeout expired and declared lost the sends it had
	// waited for, which no switch trimmed: a trimmed send's record is released
	// by its repair request.
	virtual void OnTimeout(uint64_t nowNs);
};

// A spine drawn uniformly per packet, named as both the requested and the
// carrying spine (spray_uniform).
class UniformSpineSelector : public PathSelector {
public:
	UniformSpineSelector(Ptr<UniformRandomVariable> random, uint32_t spines);
	uint16_t Choose(uint64_t nowNs) override;

private:
	Ptr<UniformRandomVariable> m_random;
	uint32_t m_spines;
};

// Oblivious packet spraying, OPS: a fresh 16-bit entropy value drawn
// uniformly per packet, as REPS (Bonato et al., arXiv:2407.21625, section 2.2)
// defines it.
class ObliviousSelector : public PathSelector {
public:
	explicit ObliviousSelector(Ptr<UniformRandomVariable> random);
	uint16_t Choose(uint64_t nowNs) override;

private:
	Ptr<UniformRandomVariable> m_random;
};

// REPS (Bonato et al., EuroSys '26, arXiv:2407.21625, section 3, Algorithms 1
// and 2): the entropy values that come back on unmarked acknowledgements are
// cached in a small circular buffer and reused, each once and oldest first; a
// fresh value is drawn when none is cached. A failure puts the sender into
// freezing mode, in which it draws nothing fresh and cycles through the values
// it holds, and leaving freezing mode it explores fresh values for a window.
class RepsSelector : public PathSelector {
public:
	// explorePackets is NUM_PKTS_CWND, the packets of one window.
	RepsSelector(Ptr<UniformRandomVariable> random, uint32_t bufferSize,
		uint64_t freezingTimeoutNs, uint32_t explorePackets);
	uint16_t Choose(uint64_t nowNs) override;
	void OnAck(uint16_t path, bool marked, uint64_t nowNs) override;
	// With trimming, a congested switch trims rather than drops, so a send that
	// times out untrimmed is taken for a failure (REPS appendix A).
	void OnTimeout(uint64_t nowNs) override;

private:
	struct Entry {
		uint16_t ev;
		bool valid;
	};
	uint16_t Fresh();
	// Algorithm 2's getNextEV.
	uint16_t Cached();

	Ptr<UniformRandomVariable> m_random;
	std::vector<Entry> m_buffer;
	uint32_t m_head; // the slot the next acknowledgement writes
	uint32_t m_valid; // numberOfValidEVs
	uint32_t m_written; // slots ever written, at most the buffer's size
	// Where freezing mode's cycle reads next, over the written slots.
	uint32_t m_frozenNext;
	bool m_frozen;
	uint64_t m_freezingTimeoutNs;
	uint64_t m_exitFreezingNs;
	uint32_t m_explorePackets;
	uint32_t m_exploreCounter;
};

} /* namespace ns3 */

#endif /* PATH_SELECTOR_H */
