#ifndef PATH_SELECTOR_H
#define PATH_SELECTOR_H

#include <ns3/ptr.h>
#include <ns3/random-variable-stream.h>
#include "spine-report.h"
#include <array>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
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
	// Under SprayUniform: the spine the latest acknowledgement redirects to.
	AckRedirect,
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
	// An acknowledgement's redirect field, the spine it asks this queue pair
	// to send on or kNoRedirect, and the carrying spine of the send it
	// answers, before the answer itself is told.
	virtual void OnRedirect(uint8_t spine, uint8_t carrying, uint64_t nowNs);
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

// The sending side of ACK redirection, modelled on MPTR (Wang et al., IEEE
// TII 2026): every packet, new data or repair, goes up the spine the latest
// acknowledgement redirects to; failing a redirect, up the spine that carried
// the send it answers; and before any acknowledgement, up a spine drawn
// uniformly.
class AckRedirectSelector : public PathSelector {
public:
	AckRedirectSelector(Ptr<UniformRandomVariable> random, uint32_t spines);
	uint16_t Choose(uint64_t nowNs) override;
	void OnRedirect(uint8_t spine, uint8_t carrying, uint64_t nowNs) override;

private:
	UniformSpineSelector m_uniform;
	// kNoRedirect before the first acknowledgement.
	uint8_t m_spine;
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
// With edgeFreeze unset, a report with the edge bit set is taken as any other.
//
// A score is fixed-point, 1/256 of a grade, and a decay removes the rounded-up
// fraction gamma of it, so that a score left without deposits reaches zero.
// Every score starts where the top grade holds it.
//
// With loss thresholds set, the scores also count per spine the sends drawn
// from them and the sends declared lost, and a report taken grades each spine
// sent on since the last report taken at the lower of its report grade and
// the grade its lost fraction, lost over sent and at most 1, earns through
// the loss thresholds; a spine not sent on keeps its report grade. The counts
// restart at every report taken. Scores used as a pool count the sends and
// losses of every set of scores pooled with them over fixed loss intervals,
// and at each interval's end grade each spine by the lost fraction of the
// interval that ended, a spine not sent on in it at the top grade; a report a
// pooled set of scores takes is taken by the pool at the lower of the grade
// it took and the pool's loss grade.
//
// A spine is held while the last report taken graded it 0; drawn through a
// pool, while the last report the pool took did. Under the epsilon held
// exploration a held spine keeps its epsilon / N of the draws. Under the
// sentinel and the none held exploration the epsilon term is spread over the
// spines not held, and a held spine is drawn only by its score, which grade 0
// decays to zero; while every spine is held the term stays spread over all.
// Under the sentinel held exploration a held spine is sent one packet per
// sentinel interval in place of a draw, the sentinel, which is a send as any
// other: it is counted, may be declared lost and, arriving, is counted by the
// receiver. Under none a held spine is sent nothing, so only a receiver whose
// hold does not wait for arrivals, the switch_queue cost source's, can
// release it.
//
// Scores used as a pool with a pool decay interval set decay once per fixed
// pool decay interval, at the multiples of it, in place of once per report
// taken: at the first report taken in an interval its base becomes the
// scores decayed once, and each report taken in the interval, unless its
// edge bit holds the scores, adds its grade to the interval's per-spine grade
// sum and count. A spine's score is its base plus the interval's mean grade,
// in 1/256 and rounded; a spine with nothing counted in the interval keeps its
// base. The mean, not the sum, keeps the fixed point at grade / gamma
// whatever the number of pooled sets of scores reporting. A spine is then
// held while the interval's mean grade rounds to 0, under 1/2; one with
// nothing counted in the interval keeps its held flag. Without reports the
// scores still age as above.
class SpineScores {
public:
	enum class CandidateDraw : uint32_t {
		Proportional = 0,
		Uniform,
	};
	enum class HeldExploration : uint32_t {
		Epsilon = 0,
		Sentinel,
		None,
	};
	struct Parameters {
		double gamma;
		double epsilon;
		uint32_t candidates;
		CandidateDraw candidateDraw;
		uint64_t reportIntervalNs;
		bool edgeFreeze = true;
		// The lost fractions from which a spine grades 2, 1 and 0; unset,
		// sends and losses are not counted.
		std::optional<std::array<double, 3>> lossThresholds;
		// The loss interval of scores used as a pool; zero, the report
		// interval.
		uint64_t lossIntervalNs = 0;
		HeldExploration heldExploration = HeldExploration::Epsilon;
		// Under the sentinel held exploration, the interval between two
		// sentinels on one held spine; zero, the report interval.
		uint64_t sentinelIntervalNs = 0;
		// As a pool: the pool decay interval; zero decays once per report
		// taken.
		uint64_t poolDecayIntervalNs = 0;
	};
	SpineScores(uint32_t spines, const Parameters &parameters);
	void OnReport(const SpineReport &report, uint64_t nowNs);
	uint8_t Choose(UniformRandomVariable &random, uint64_t nowNs);
	// Under the sentinel held exploration: whether a held spine's sentinel is
	// due by now, its last sentinel a sentinel interval or more ago, and which
	// spine; the lowest such spine is taken and its sentinel interval restarts
	// now.
	bool TakeSentinel(uint64_t nowNs, uint8_t &spine);
	// Whether spine is held for the draws from these scores.
	bool Held(uint32_t spine) const;
	// A send drawn from these scores on spine, and a send on spine declared
	// lost; each is counted by the pool too.
	void OnSend(uint32_t spine, uint64_t nowNs);
	void OnLoss(uint32_t spine, uint64_t nowNs);
	// Mix these scores with pool's, which outlive them. Every report these
	// scores take, pool takes too, by the same rule; and with weight w above
	// zero, every draw and every p use w * pool's score + (1 - w) * this
	// score of each spine in place of this score. Several scores may share a
	// pool.
	void Pool(SpineScores &pool, double weight);
	// As a pool with loss thresholds set: called as each loss interval is
	// closed, with the time it ended at and, per spine, the sends and losses
	// counted in it and the grade its lost fraction earns, the top grade for
	// a spine not sent on. A later interval that ended before the next send,
	// loss or report counted nothing and is not passed.
	using LossIntervalCallback = std::function<void(uint64_t endNs,
		const std::vector<uint32_t> &sent, const std::vector<uint32_t> &lost,
		const std::vector<uint8_t> &grades)>;
	void SetLossIntervalCallback(LossIntervalCallback callback);
	// p for spine, as the scores stand.
	double Share(uint32_t spine) const;
	uint32_t Score(uint32_t spine) const;

private:
	static constexpr uint32_t kGradeScale = 256;
	// Decay the scores and add report's grades, unless its edge bit holds
	// them; whether report is newer than the last is the caller's test.
	void Take(const SpineReport &report, uint64_t nowNs);
	// As a pool with a pool decay interval: take report into the interval's
	// mean grades, decaying the base once per interval.
	void PoolTake(const SpineReport &report, uint64_t nowNs);
	bool IsPooled() const;
	// The score a pooled draw weighs spine by, and the sum over the spines.
	double Mixed(uint32_t spine) const;
	double MixedSum() const;
	uint8_t DrawFromMixed(UniformRandomVariable &random) const;
	void Decay();
	void Decay(std::vector<uint32_t> &scores) const;
	// Decay the scores for every report interval missed by now.
	void Age(uint64_t nowNs);
	void Accumulate();
	uint8_t DrawFromShares(UniformRandomVariable &random) const;
	// The spines not held, under a held exploration that spreads the
	// epsilon term over them; the epsilon draw u / epsilon in [0, 1) mapped
	// onto the spines it is spread over.
	uint32_t Unheld() const;
	uint8_t DrawEpsilon(double fraction) const;
	// The epsilon term of spine's share.
	double EpsilonShare(uint32_t spine) const;
	// The grade the lost fraction of sent sends earns.
	uint8_t LossGrade(uint32_t sent, uint32_t lost) const;
	// As a pool: close the loss intervals ended by now, grading the spines by
	// the last of them, before a send or loss is counted or a grade read.
	void AdvanceLossInterval(uint64_t nowNs);

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
	// Null, or the scores mixed with these, and the weight of theirs.
	SpineScores *m_pool;
	double m_poolWeight;
	// Per spine, the sends and the losses counted since the counts restarted.
	std::vector<uint32_t> m_sent;
	std::vector<uint32_t> m_lost;
	// As a pool: the loss interval, the end of the one in progress, and the
	// grades the last one that ended earned.
	uint64_t m_lossIntervalNs;
	uint64_t m_lossIntervalEndNs;
	std::vector<uint8_t> m_lossGrades;
	LossIntervalCallback m_onLossInterval;
	// Per spine, whether the last report taken graded it 0, and when its
	// last sentinel was sent.
	std::vector<bool> m_held;
	std::vector<uint64_t> m_sentinelNs;
	uint64_t m_sentinelIntervalNs;
	// As a pool with a pool decay interval: the end of the interval in
	// progress, the scores decayed once at its start, and per spine the sum
	// and count of the grades taken in it.
	uint64_t m_poolDecayEndNs;
	std::vector<uint32_t> m_base;
	std::vector<uint32_t> m_gradeSum;
	std::vector<uint32_t> m_gradeCount;
};

// A spine drawn per packet from the scores towards the queue pair's
// destination, named as both the requested and the carrying spine
// (spray_policy). The scores are shared with every queue pair to the same
// host, or owned by this queue pair alone.
class PolicySpineSelector : public PathSelector {
public:
	PolicySpineSelector(Ptr<UniformRandomVariable> random, SpineScores &scores);
	PolicySpineSelector(Ptr<UniformRandomVariable> random,
		std::unique_ptr<SpineScores> scores);
	// A held spine's sentinel when one is due, else the spine drawn; either
	// is counted as a send in the scores.
	uint16_t Choose(uint64_t nowNs) override;
	// A loss on the requested spine path names is counted in the scores.
	void OnLoss(uint16_t path, uint64_t nowNs) override;
	void OnReport(const SpineReport &report, uint64_t nowNs) override;

private:
	Ptr<UniformRandomVariable> m_random;
	// Declared before m_scores, which refers to it when it is set.
	std::unique_ptr<SpineScores> m_ownScores;
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
