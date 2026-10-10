#ifndef PATH_SELECTOR_H
#define PATH_SELECTOR_H

#include <ns3/ptr.h>
#include <ns3/random-variable-stream.h>
#include "spine-report.h"
#include <deque>
#include <stdint.h>
#include <utility>
#include <vector>

namespace ns3 {

// Which design chooses a data packet's entropy value under EntropyHash.
enum class PathSelectorKind : uint32_t {
	Ops = 0,
	Reps,
	UeOblivious,
	UeAware,
	Mrc,
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
	// Asked at every data send: whether a probe of some path is due, and which.
	virtual bool TakeProbe(uint64_t nowNs, uint16_t &path);
	// The answer to a probe of path; marked when the probe arrived with CE.
	virtual void OnProbeAnswer(uint16_t path, bool marked, uint64_t nowNs);
	// The receiving host's spine report an acknowledgement or a repair request
	// carried, before the answer itself is told.
	virtual void OnReport(const SpineReport &report, uint64_t nowNs);
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

// A sender's scores of the spines towards one receiving host, which every
// queue pair to that host draws its spines from. Each report from the host
// that is newer than the last one taken decays every score and then adds the
// spine's grade: s <- (1 - gamma) s + x. A spine is drawn with probability
// p = (1 - epsilon) s / sum(s) + epsilon / N, and uniformly while every score
// is zero; drawing candidates more than one at a time sends on the
// highest-scored of them, the candidates drawn from p or uniformly. While a
// report's edge bit is set the scores stay as they are, and without reports
// they decay once per report interval from two intervals after the last.
//
// A score is fixed-point, 1/256 of a grade, and a decay removes the rounded-up
// fraction gamma of it, so that a score left without deposits reaches zero.
// Every score starts where the top grade holds it.
class SpineScores {
public:
	enum class CandidateDraw : uint32_t {
		Proportional = 0,
		Uniform,
	};
	struct Parameters {
		double gamma;
		double epsilon;
		uint32_t candidates;
		CandidateDraw candidateDraw;
		uint64_t reportIntervalNs;
	};
	SpineScores(uint32_t spines, const Parameters &parameters);
	void OnReport(const SpineReport &report, uint64_t nowNs);
	uint8_t Choose(UniformRandomVariable &random, uint64_t nowNs);
	// p for spine, as the scores stand.
	double Share(uint32_t spine) const;
	uint32_t Score(uint32_t spine) const;

private:
	static constexpr uint32_t kGradeScale = 256;
	void Decay();
	// Decay the scores for every report interval missed by now.
	void Age(uint64_t nowNs);
	void Accumulate();
	uint8_t DrawFromShares(UniformRandomVariable &random) const;

	Parameters m_parameters;
	// gamma in 1/65536.
	uint64_t m_decay;
	std::vector<uint32_t> m_scores;
	// The running sum of the scores in spine order; the last is their sum.
	std::vector<uint64_t> m_accumulated;
	bool m_reported;
	uint8_t m_sequence;
	uint64_t m_reportedNs;
	uint64_t m_nextDecayNs;
};

// A spine drawn per packet from the scores towards the queue pair's
// destination, named as both the requested and the carrying spine
// (spray_policy).
class PolicySpineSelector : public PathSelector {
public:
	PolicySpineSelector(Ptr<UniformRandomVariable> random, SpineScores &scores);
	uint16_t Choose(uint64_t nowNs) override;
	void OnReport(const SpineReport &report, uint64_t nowNs) override;

private:
	Ptr<UniformRandomVariable> m_random;
	SpineScores &m_scores;
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

// The entropy values 0 to size - 1, visited in a pseudo-random order that
// uses each once before any repeats and is drawn anew for every pass (UEC
// 1.0.3 section 3.6.16.3).
class EntropyRotation {
public:
	EntropyRotation(Ptr<UniformRandomVariable> random, uint32_t size);
	uint16_t Next();
	uint32_t Size() const;

private:
	void Shuffle();

	Ptr<UniformRandomVariable> m_random;
	std::vector<uint16_t> m_order;
	uint32_t m_next;
};

// UEC 1.0.3 section 3.6.16.3, oblivious multipath spraying: the values of an
// entropy space in a pseudo-random order, whatever comes back.
class UeObliviousSelector : public PathSelector {
public:
	UeObliviousSelector(Ptr<UniformRandomVariable> random, uint32_t size);
	uint16_t Choose(uint64_t nowNs) override;

private:
	EntropyRotation m_rotation;
};

// UEC 1.0.3 section 3.6.16.4, path-aware multipath spraying with the
// section's congestion bitmap: a value reported congested is marked, and the
// rotation skips a marked value once, clearing its mark, unless more than a
// fraction of the space is marked, when the signal no longer tells paths apart
// and nothing is skipped.
class UeAwareSelector : public PathSelector {
public:
	UeAwareSelector(Ptr<UniformRandomVariable> random, uint32_t size,
		double saturationFraction);
	uint16_t Choose(uint64_t nowNs) override;
	void OnAck(uint16_t path, bool marked, uint64_t nowNs) override;
	// A trim before the last hop reports the path congested; a last-hop trim
	// only through its echoed mark, the destination's own link being on every
	// path (UEC 1.0.3 section 3.6.12.3).
	void OnTrim(uint16_t path, bool lastHop, bool marked, uint64_t nowNs) override;

private:
	void Mark(uint16_t ev);

	EntropyRotation m_rotation;
	std::vector<bool> m_marked;
	uint32_t m_markedCount;
	double m_saturationFraction;
};

// MRC (OCP MRC 1.0 section 9.3.1, with the EV choice of the informative
// example in section 11.2.2): each value of the queue pair's set is GOOD, SKIP
// or ASSUMED_BAD. The rotation sends on a GOOD value and passes over the
// others; the first SKIP value one send passes over it resets to GOOD. A
// marked acknowledgement or a trim before the last hop moves a value to SKIP,
// which also lapses to GOOD after a time; a declared loss, by the loss rule or
// by the timeout, moves it to ASSUMED_BAD, out of service until a periodic
// probe's answer moves it to GOOD, or to SKIP if the probe came back marked.
// DENIED is set only by a controller, which the simulator has no counterpart
// of, so no value is ever DENIED.
class MrcSelector : public PathSelector {
public:
	MrcSelector(Ptr<UniformRandomVariable> random, uint32_t size,
		uint64_t skipNs, uint64_t probeIntervalNs);
	uint16_t Choose(uint64_t nowNs) override;
	void OnAck(uint16_t path, bool marked, uint64_t nowNs) override;
	// A trim before the last hop is the TRIMMED NACK of section 9.3.1; a
	// last-hop trim is TRIMMED_LASTHOP, which moves nothing.
	void OnTrim(uint16_t path, bool lastHop, bool marked, uint64_t nowNs) override;
	void OnLoss(uint16_t path, uint64_t nowNs) override;
	bool TakeProbe(uint64_t nowNs, uint16_t &path) override;
	void OnProbeAnswer(uint16_t path, bool marked, uint64_t nowNs) override;

private:
	enum class State : uint8_t { Good, Skip, AssumedBad };
	struct Entropy {
		State state;
		uint64_t skipUntilNs;
		// When an ASSUMED_BAD value's next probe is due.
		uint64_t probeDueNs;
	};
	Entropy &At(uint16_t ev);
	void Skip(uint16_t ev, uint64_t nowNs);

	EntropyRotation m_rotation;
	std::vector<Entropy> m_entropies;
	// Probes as (value, due time), earliest first. An entry whose value has
	// left ASSUMED_BAD since, or was condemned again, is stale and dropped.
	std::deque<std::pair<uint16_t, uint64_t>> m_probes;
	uint64_t m_skipNs;
	uint64_t m_probeIntervalNs;
};

} /* namespace ns3 */

#endif /* PATH_SELECTOR_H */
