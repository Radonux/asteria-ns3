#ifndef PATH_SELECTOR_H
#define PATH_SELECTOR_H

#include <ns3/ptr.h>
#include <ns3/random-variable-stream.h>
#include "receiver-hold.h"
#include "spine-report.h"
#include <array>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdint.h>
#include <utility>
#include <vector>

namespace ns3 {

// What the host-wide loss hold of RepsHold is kept for: the host's every
// destination, each destination leaf apart, per spine either of the two by
// the destination leaves it loses toward (AutoLossHold), or each destination
// host apart.
enum class HoldScope : uint32_t {
	Host = 0,
	Leaf,
	Auto,
	Pair,
};

// How long a sending host keeps a queue pair's path selector: for its one
// message, or for the connection to its destination, a selector per
// destination and priority group shared by the queue pairs to it and kept
// past them (SharedPathSelector).
enum class PathState : uint32_t {
	Message = 0,
	Connection,
};

// What puts a spine in the loss hold of RepsHold, RepsFlowHold and MrcHold,
// per hold key: Count, a closed loss interval whose lost fraction grades it 0
// through the loss thresholds; Llr, the key's LossLikelihoodRatio reaching its
// threshold, at the send declared lost that takes it there.
enum class HoldRule : uint32_t {
	Count = 0,
	Llr,
};

// Wald's sequential probability ratio test of a hold key's sends, each taken
// for a Bernoulli trial lost with probability p0 while the key's path works
// and p1 while it is faulted. The statistic L starts at 0; each send adds
// ln((1 - p1) / (1 - p0)), which is not positive, and each send declared lost
// adds ln(p1 / p0). With p0 zero a loss adds +infinity, so that the first
// loss reaches the threshold, and a send adds 0. The test is passed once L
// reaches ln((1 - beta) / alpha), alpha being the probability of passing it
// on a working path and beta that of missing a faulted one. L is clamped
// below at 0, so that a clean stretch builds no unbounded credit against the
// losses that follow it, and restarts at 0 at Reset.
class LossLikelihoodRatio {
public:
	struct Parameters {
		double p0 = 0;
		double p1 = 0.001;
		double alpha = 0.001;
		double beta = 0.001;
	};
	// 0 <= p0 < p1 < 1 and alpha and beta in (0, 0.5).
	explicit LossLikelihoodRatio(const Parameters &parameters);
	static bool Valid(const Parameters &parameters);
	void Send();
	// A send declared lost; whether L has reached the threshold.
	bool Loss();
	void Reset();
	double Value() const;
	double SendStep() const;
	double LossStep() const;
	double Threshold() const;
	// The sends and the sends declared lost since L was last clamped at 0 by
	// a send, that send included, or since the start or Reset: the evidence
	// L holds.
	uint32_t WindowSent() const;
	uint32_t WindowLost() const;

private:
	double m_sendStep;
	double m_lossStep;
	double m_threshold;
	double m_value;
	uint32_t m_windowSent;
	uint32_t m_windowLost;
};

// How a spine leaves the loss hold of RepsHold, RepsFlowHold and MrcHold:
// Sentinels, by the clean sends of the sentinels sent one per sentinel
// interval; Trickle, by TrickleRelease, its trickle sends being a fraction of
// each queue pair's sends (TrickleSchedule).
enum class HoldRelease : uint32_t {
	Sentinels = 0,
	Trickle,
};

// The release statistic of a held key under HoldRelease::Trickle. At the
// hold's start p1 is recorded: the lost fraction of the hold's evidence,
// lost over sent and at most 1, at least the p1 of the parameters, the
// evidence being the loss interval that graded the key 0 under
// HoldRule::Count and the LossLikelihoodRatio's window under Llr. R starts
// at 0; each trickle send acknowledged adds ln((1 - p0) / (1 - p1)), -ln(1 -
// p1) with p0 zero, and each trickle send declared lost restarts R at 0. The
// key is released once R reaches ln(1 / alpha).
class TrickleRelease {
public:
	explicit TrickleRelease(const LossLikelihoodRatio::Parameters &parameters);
	// A hold starts on evidence of sent sends of which lost were declared
	// lost, or on an evidence rate.
	void Begin(uint32_t sent, uint32_t lost);
	void Begin(double rate);
	static double Rate(uint32_t sent, uint32_t lost);
	// The answer to a trickle send of the held key, lost or acknowledged;
	// whether R has reached the threshold.
	bool Answer(bool lost);
	double Value() const;
	double P1() const;
	double Step() const;
	double Threshold() const;

private:
	double m_p0;
	double m_floorP1;
	double m_threshold;
	double m_p1;
	double m_step;
	double m_value;
};

// What an exploration send onto a held spine of a loss hold carries, for
// every selector that sends through the hold (LossHoldExploration): Data,
// the exploration send is the data packet itself, in place of the selector's
// choice (a sentinel under HoldRelease::Sentinels, a trickle send under
// Trickle); Duplicate, the data packet goes up the spine the selector
// chooses, as if no exploration were due, and a copy of it with the same
// sequence goes up the held spine in the same send (PathSelector::TakeCopy).
// The copy is counted in the hold as a send on the held spine, its
// acknowledgement and its loss move the hold's statistics as a data
// exploration send's do, and it never asks for a retransmission or a timeout
// (RdmaQueuePair::AddCopy). Where the selector chooses the held spine
// itself, which it does only while every spine is held, the data packet is
// the exploration send and no copy is made.
enum class SentinelMode : uint32_t {
	Data = 0,
	Duplicate,
};

// When a copy under SentinelMode::Duplicate that its own acknowledgement has
// not answered is declared lost for the selector (RdmaQueuePair::ExpireCopies).
// DataAck: once the data send of its sequence is acknowledged, at the first
// acknowledgement after one base RTT since the copy was sent, so that a copy
// queued on its spine for longer than its data's path is lost. Deadline: at
// the first acknowledgement after a deadline since the copy was sent, the
// tail loss guard of SprayTailLossRtts base RTTs, whether its data send is
// acknowledged or not.
enum class CopyExpiry : uint32_t {
	DataAck = 0,
	Deadline,
};

// The copy a selector's latest Choose asked for under SentinelMode::Duplicate.
struct PendingCopy {
	bool pending = false;
	uint8_t spine = 0;
	bool trickle = false;
	// Under Duplicate, with REPS's choice live made and the exploration send
	// due on explored, set the copy unless they coincide; whether the live
	// send is the exploration send itself.
	bool Set(uint8_t live, uint8_t explored, bool trickleSend);
	bool Take(uint8_t &copySpine, bool &copyTrickle);
};

// One queue pair's schedule of trickle sends onto the spines its loss hold
// holds: per spine it counts the queue pair's sends while the spine is held,
// the send in progress included, and restarts at 0 while it is not; a held
// spine whose count reaches every is sent the send in progress and its count
// restarts, the lowest such spine first. Each held spine thus receives one
// in every sends of the queue pair, by a counter and not a draw.
class TrickleSchedule {
public:
	explicit TrickleSchedule(uint32_t spines);
	// With every above zero, whether the send in progress is a trickle send,
	// and onto which spine.
	bool Take(uint32_t every, const std::function<bool(uint32_t)> &held,
		uint8_t &spine);

private:
	std::vector<uint32_t> m_count;
};

// Which design chooses a data packet's entropy value under EntropyHash.
enum class PathSelectorKind : uint32_t {
	Ops = 0,
	Reps,
	UeOblivious,
	UeAware,
	Mrc,
	// Under SprayUniform: the spine the latest acknowledgement redirects to.
	AckRedirect,
	// Under SprayUniform: REPS over spines, leaving out the spines the host's
	// loss hold holds, or the queue pair's own.
	RepsHold,
	RepsFlowHold,
	// Under SprayUniform: REPS over spines, leaving out the spines the
	// switches' state shows a fault on toward the destination's leaf.
	RepsOracle,
	// Under SprayUniform: MRC over spines, leaving out the spines RepsHold's
	// loss hold holds.
	MrcHold,
	// Under SprayUniform: REPS over spines, leaving out none.
	RepsSpine,
	// Under SprayUniform: Themis's spraying by packet sequence number, with
	// its source's per-path avoidance (ThemisSelector).
	Themis,
	// Under SprayUniform: Themis, leaving out the spines RepsHold's loss hold
	// holds (ThemisHoldSelector).
	ThemisHold,
};

// The queue pair's sends per trickle send onto one held spine that receives
// fraction of them: the fraction's inverse, rounded, fraction in (0, 0.5].
uint32_t TrickleEvery(double fraction);

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
	// The loss of a send along path told to OnLoss at countedNs, taken back:
	// the send had arrived. Whether that took back the hold the loss began;
	// a selector without a loss hold that can take a loss back keeps it.
	virtual bool RetractLoss(uint16_t path, uint64_t countedNs, uint64_t nowNs);
	// The retransmission timeout expired and declared lost the sends it had
	// waited for, which no switch trimmed: a trimmed send's record is released
	// by its repair request.
	virtual void OnTimeout(uint64_t nowNs);
	// Asked at every data send: whether a probe of some path is due, and which.
	virtual bool TakeProbe(uint64_t nowNs, uint16_t &path);
	// The answer to a probe of path; marked when the probe arrived with CE.
	virtual void OnProbeAnswer(uint16_t path, bool marked, uint64_t nowNs);
	// Whether the latest Choose was a trickle send of a loss hold
	// (HoldRelease::Trickle), whose answer is told to OnTrickleAnswer.
	virtual bool ChoseTrickle() const;
	// Whether the latest Choose's data packet was an exploration send onto a
	// held spine: a sentinel or a trickle send.
	virtual bool ChoseExploration() const;
	// Whether the latest Choose's data packet goes up a spine a loss hold, or
	// the receivers' hold, holds as it is chosen: an exploration send under
	// SentinelMode::Data, or any send while every spine is held.
	virtual bool ChoseHeld() const;
	// A trickle send along path acknowledged, or declared lost, after
	// OnAck or OnLoss has been told.
	virtual void OnTrickleAnswer(uint16_t path, bool lost, uint64_t nowNs);
	// Under SentinelMode::Duplicate: whether the latest Choose asked for a
	// copy of its data packet onto a held spine, which spine, and whether
	// the copy is a trickle send; the request is taken. The copy's loss is
	// told to OnLoss and, for a trickle copy, its answers to
	// OnTrickleAnswer.
	virtual bool TakeCopy(uint8_t &spine, bool &trickle);
	// The receiving host's spine report an acknowledgement or a repair request
	// carried, before the answer itself is told.
	virtual void OnReport(const SpineReport &report, uint64_t nowNs);
	// An acknowledgement's redirect field, the spine it asks this queue pair
	// to send on or kNoRedirect, and the carrying spine of the send it
	// answers, before the answer itself is told.
	virtual void OnRedirect(uint16_t spine, uint8_t carrying, uint64_t nowNs);
	// An acknowledgement's receiver_hold bitmap from receiver, before the
	// answer itself is told; partial for a partial spine list, which adds
	// the spines it names and releases none.
	virtual void OnReceiverHold(uint32_t receiver, const SpineBitmap &bitmap,
		bool partial, uint64_t nowNs);
	// Under SprayReceiverHold, the marks of the receivers on the destination's
	// leaf, which a selector with a loss hold holds spines by; others ignore
	// them.
	virtual void SetReceiverHold(std::shared_ptr<ReceiverHoldMarks> marks,
		uint32_t leaf);
	// Before Choose for a data packet: its packet sequence number in the
	// connection (the queue pair's base plus its sequence in packets), and
	// whether it is a repair.
	virtual void NextSend(uint64_t psn, bool repair);
	// An acknowledgement flagged as Themis's Path Avoidance Signal for the
	// packet of sequence number psn; returns the spine avoided, or -1 for
	// none.
	virtual int OnPathAvoid(uint64_t psn, uint64_t nowNs);
};

// A queue pair's handle on the path selector of its connection under
// PathState::Connection, which the host's other queue pairs to the same
// destination share and which outlives them all: every call goes to it.
class SharedPathSelector : public PathSelector {
public:
	explicit SharedPathSelector(std::shared_ptr<PathSelector> shared);
	uint16_t Choose(uint64_t nowNs) override;
	void OnAck(uint16_t path, bool marked, uint64_t nowNs) override;
	void OnTrim(uint16_t path, bool lastHop, bool marked, uint64_t nowNs) override;
	void OnLoss(uint16_t path, uint64_t nowNs) override;
	bool RetractLoss(uint16_t path, uint64_t countedNs, uint64_t nowNs) override;
	void OnTimeout(uint64_t nowNs) override;
	bool TakeProbe(uint64_t nowNs, uint16_t &path) override;
	void OnProbeAnswer(uint16_t path, bool marked, uint64_t nowNs) override;
	bool ChoseTrickle() const override;
	bool ChoseExploration() const override;
	bool ChoseHeld() const override;
	void OnTrickleAnswer(uint16_t path, bool lost, uint64_t nowNs) override;
	bool TakeCopy(uint8_t &spine, bool &trickle) override;
	void OnReport(const SpineReport &report, uint64_t nowNs) override;
	void OnRedirect(uint16_t spine, uint8_t carrying, uint64_t nowNs) override;
	void OnReceiverHold(uint32_t receiver, const SpineBitmap &bitmap,
		bool partial, uint64_t nowNs) override;
	void SetReceiverHold(std::shared_ptr<ReceiverHoldMarks> marks,
		uint32_t leaf) override;
	void NextSend(uint64_t psn, bool repair) override;
	int OnPathAvoid(uint64_t psn, uint64_t nowNs) override;

private:
	std::shared_ptr<PathSelector> m_shared;
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

// The source side of Themis (Liu et al., EuroSys 2026), over spines. The
// packet of sequence number psn goes up the spine of path psn mod N, the N
// paths spaced spines / N apart from a base drawn once (equation 1). A Path
// Avoidance Signal for psn sets its path's counter to the avoidance window;
// while the path a packet is assigned has a nonzero counter, the packet goes
// up another path drawn at random among those with none, and the counter
// falls by one (section 4.4). Under reroute a repair, too, goes up another
// path. When every other path is avoided, the packet keeps its own. The
// counters, one per spine, are the selector's own, or under counters given
// shared with other selectors (a host's connections to one leaf). A spine
// Excluded names is passed over as an avoided one is, taking no count.
class ThemisSelector : public PathSelector {
public:
	using Counters = std::shared_ptr<std::vector<uint64_t>>;
	ThemisSelector(Ptr<UniformRandomVariable> random, uint32_t spines,
		uint32_t paths, uint64_t window, bool reroute, Counters counters = nullptr);
	// With the base given, for tests.
	ThemisSelector(Ptr<UniformRandomVariable> random, uint32_t spines,
		uint32_t paths, uint64_t window, bool reroute, uint32_t base,
		Counters counters = nullptr);
	uint16_t Choose(uint64_t nowNs) override;
	void NextSend(uint64_t psn, bool repair) override;
	int OnPathAvoid(uint64_t psn, uint64_t nowNs) override;
	// The spine path psn mod N names.
	uint8_t Assigned(uint64_t psn) const;
	uint64_t Counter(uint8_t spine) const;
	// The packets sent up another path than their own while it was avoided.
	uint64_t Redirects() const;

protected:
	// The spine of the send NextSend named.
	uint8_t ChooseSpine();
	// None by default.
	virtual bool Excluded(uint8_t spine) const;

private:
	uint8_t DrawOther(uint8_t assigned);
	Ptr<UniformRandomVariable> m_random;
	uint32_t m_spines;
	uint32_t m_paths;
	uint32_t m_stride;
	uint32_t m_base;
	uint64_t m_window;
	bool m_reroute;
	Counters m_counter;
	// The packet NextSend named, and whether its Choose has taken a count
	// already (a repair moved off a waiting tail loss chooses again).
	uint64_t m_psn = 0;
	bool m_repair = false;
	bool m_decremented = false;
	uint64_t m_redirects = 0;
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
	void OnRedirect(uint16_t spine, uint8_t carrying, uint64_t nowNs) override;

private:
	UniformSpineSelector m_uniform;
	// kNoRedirect before the first acknowledgement.
	uint16_t m_spine;
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
// With loss hold clean sentinels n above zero, the loss grading also holds:
// per spine, in the counts each report takes and in the pool's loss
// intervals alike, a report period or interval whose lost fraction grades a
// spine 0 puts it in loss hold with its clean count at zero. While in loss
// hold a spine's loss grade is 0: a period with a loss restarts its clean
// count, and a period without one adds its sends to the count until it
// reaches n, when the loss hold ends and that period's loss grade is the top
// grade, the report's grade then applying through the lower of the two as
// before. A period with nothing sent leaves a spine's loss hold as it is: in
// loss hold its loss grade stays 0, and outside it the period grades as
// above, the report's grade standing and the pool's loss grade the top; so
// the pool's loss grade of a spine in loss hold stays 0 across intervals with
// nothing sent. The loss hold outlives the counts, which still restart at
// every report taken. Under the sentinel held
// exploration a held spine's sends are its sentinels, so n clean sentinels in
// a row are the evidence that releases it.
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
//
// Drawn through a pool with pool hold propagation set, a spine is held while
// these scores hold it, the last report they took grading it 0, or the pool
// does, by its held flag or its loss hold. A spine the pool holds weighs
// nothing in the draws and in p, under every held exploration, the others
// sharing 1 - epsilon in proportion to their mixed scores; under the epsilon
// held exploration it keeps its epsilon / N, and under sentinel and none it
// has no share of the epsilon term, as a spine held only by these scores has
// none, while the latter keeps its mixed score's part. While the pool holds
// every spine, nothing is excluded for the pool's hold. Under the sentinel
// held exploration the sentinels of a spine the pool holds follow the pool's
// sentinel schedule, one per sentinel interval for every set of scores
// pooled with it, taken by whichever draws first once one is due; a spine
// held only by these scores follows their own schedule. The pool's hold ends
// by the pool's rule, a report it takes grading the spine above 0 and its
// loss hold's clean sends, and these scores' hold by theirs.
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
		// With loss thresholds set, the clean sends that end a loss hold;
		// zero holds nothing.
		uint32_t lossHoldCleanSentinels = 0;
		HeldExploration heldExploration = HeldExploration::Epsilon;
		// Under the sentinel held exploration, the interval between two
		// sentinels on one held spine; zero, the report interval.
		uint64_t sentinelIntervalNs = 0;
		// Which sends take a sentinel once it is due: one in sentinelSpread
		// on average for the host sentinelSalt names (SentinelSpreadTakes);
		// at 1 the first.
		uint32_t sentinelSpread = 1;
		uint64_t sentinelSalt = 0;
		// As a pool: the pool decay interval; zero decays once per report
		// taken.
		uint64_t poolDecayIntervalNs = 0;
		// Drawn through a pool: whether a spine the pool holds is held for
		// these scores' draws and sentinels too.
		bool poolHoldPropagates = false;
		// Scores used as a loss hold (CountIntervalSend and
		// CountIntervalLoss): what puts a spine in loss hold, and under
		// HoldRule::Llr the test kept per spine.
		HoldRule holdRule = HoldRule::Count;
		LossLikelihoodRatio::Parameters likelihood;
		// How a spine leaves the loss hold, and under HoldRelease::Trickle
		// the fraction of each queue pair's sends a held spine receives;
		// TrickleRelease reads p0, p1 and alpha of likelihood.
		HoldRelease holdRelease = HoldRelease::Sentinels;
		double trickleFraction = 0.01;
		// What the selectors sending through the hold send a held spine.
		SentinelMode sentinelMode = SentinelMode::Data;
		// Scores used as a loss hold by the selectors that send through it
		// (LossHoldExploration): the clean sends that end a loss hold are
		// the sends on the spine acknowledged in the loss intervals
		// (CountIntervalAnswer) instead of the sends counted, so that a send
		// whose answer is still due when its interval closes counts as no
		// clean send.
		bool cleanOnAnswers = false;
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
	// Whether these scores hold spine themselves, by their held flag or their
	// loss hold, and whether they hold every spine.
	bool Holds(uint32_t spine) const;
	bool HoldsEvery() const;
	// With loss thresholds set, scores used as a loss hold without reports,
	// as a pool is: a send on spine and a send on spine declared lost,
	// counted in the loss interval in progress once those ended by now are
	// closed. Under HoldRule::Llr each is also a step of spine's
	// LossLikelihoodRatio while spine is not in loss hold, and the loss that
	// passes its test puts spine in loss hold at once, with its clean count
	// at zero; the loss thresholds then grade the intervals passed to the
	// loss interval callback but hold nothing, a spine in loss hold grading
	// 0. A loss hold's release restarts spine's test.
	void CountIntervalSend(uint32_t spine, uint64_t nowNs);
	void CountIntervalLoss(uint32_t spine, uint64_t nowNs);
	// With loss thresholds set, the acknowledgement of a send on spine,
	// counted in the loss interval in progress once those ended by now are
	// closed; under cleanOnAnswers these are a loss hold's clean sends.
	void CountIntervalAnswer(uint32_t spine, uint64_t nowNs);
	// Under HoldRule::Llr, spine's L.
	double LossRatio(uint32_t spine) const;
	// Under HoldRelease::Trickle, a held spine is sent no sentinel: each
	// queue pair sends it one in TrickleEvery of its sends, and the answer
	// to each such send is told here, after the send and any loss of it
	// have been counted. A spine not in loss hold ignores it. The release
	// TrickleRelease decides ends the loss hold, restarts the spine's test
	// and its counts in the loss interval in progress; the loss intervals'
	// clean sends release nothing.
	uint32_t TrickleEvery() const;
	void TrickleAnswer(uint32_t spine, bool lost, uint64_t nowNs);
	SentinelMode Sentinels() const;
	// Under HoldRelease::Trickle, spine's R and the p1 recorded at its loss
	// hold's start.
	double ReleaseValue(uint32_t spine) const;
	double ReleaseP1(uint32_t spine) const;
	// Scores used as a loss hold: close the loss intervals ended by now and,
	// under the sentinel held exploration, whether a sentinel on a spine
	// these scores hold is due, and which; the lowest such spine is taken
	// and its sentinel interval restarts now.
	bool TakeHoldSentinel(uint64_t nowNs, uint8_t &spine);
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
	// counted in it and the grade its lost fraction earns through the loss
	// hold, the top grade for a spine not sent on and not in loss hold. A
	// later interval that ended before the next send, loss or report counted
	// nothing and is not passed.
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
	// Drawn through a pool with pool hold propagation set.
	bool Propagates() const;
	// Whether the pool holds spine: its held flag or its loss hold.
	bool PoolHolds(uint32_t spine) const;
	// Whether spine's sentinel is due by now, its last one a sentinel
	// interval or more ago; if so its sentinel interval restarts now.
	bool SentinelDue(uint32_t spine, uint64_t nowNs);
	// Under propagation: whether a spine the pool holds is excluded from the
	// mixed draw, which it is unless the pool holds every spine.
	bool ExcludesPoolHeld() const;
	// The mixed score of spine, or 0 when it is excluded; excluding is
	// ExcludesPoolHeld() as computed by the caller.
	double Weight(uint32_t spine, bool excluding) const;
	double WeightSum(bool excluding) const;
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
	// The grade the lost fraction of sent sends earns; losses with no sends
	// are a fraction of 1, and neither the top grade.
	uint8_t LossGrade(uint32_t sent, uint32_t lost) const;
	// The loss grade of a period in which spine had sent sends and lost
	// declared lost, through the loss hold; the top grade for a spine not
	// sent on, not lost on and not in loss hold.
	uint8_t HeldLossGrade(uint32_t spine, uint32_t sent, uint32_t lost,
		uint32_t answered);
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
	// Per spine, the acknowledgements counted since the counts restarted.
	std::vector<uint32_t> m_answered;
	// As a pool: the loss interval, the end of the one in progress, and the
	// grades the last one that ended earned.
	uint64_t m_lossIntervalNs;
	uint64_t m_lossIntervalEndNs;
	std::vector<uint8_t> m_lossGrades;
	LossIntervalCallback m_onLossInterval;
	// Per spine, whether it is in loss hold, and its clean sends since the
	// loss hold started or last saw a loss.
	std::vector<bool> m_lossHeld;
	std::vector<uint32_t> m_lossClean;
	// Per spine, its test under HoldRule::Llr and its release under
	// HoldRelease::Trickle.
	std::vector<LossLikelihoodRatio> m_likelihood;
	std::vector<TrickleRelease> m_trickle;
	// Start spine's loss hold on evidence of sent sends and lost losses.
	void StartLossHold(uint32_t spine, uint32_t sent, uint32_t lost);
	// Per spine, whether the last report taken graded it 0, and when its
	// last sentinel was sent.
	std::vector<bool> m_held;
	std::vector<uint64_t> m_sentinelNs;
	uint64_t m_sentinelIntervalNs;
	uint32_t m_sentinelSpread;
	uint64_t m_sentinelSalt;
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

protected:
	// Over the paths 0 to paths - 1. A path Excluded names is never chosen:
	// a cached one is passed over, used up as if it had been sent on, and a
	// fresh one is drawn again. While the buffer holds only such values,
	// freezing mode draws fresh ones too. Excluded must leave some path in.
	RepsSelector(Ptr<UniformRandomVariable> random, uint32_t bufferSize,
		uint64_t freezingTimeoutNs, uint32_t explorePackets, uint32_t paths);
	virtual bool Excluded(uint16_t path) const;

private:
	struct Entry {
		uint16_t ev;
		bool valid;
	};
	uint16_t Fresh();
	// Algorithm 2's getNextEV.
	uint16_t Cached();

	Ptr<UniformRandomVariable> m_random;
	uint32_t m_paths;
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

// The host-wide loss hold of RepsHold under HoldScope::Auto. Per destination
// leaf and spine it counts the sends and the sends declared lost over each
// loss interval, one interval clock for the host, and per spine it keeps the
// destination leaves with at least one loss on it since that spine was last
// released, or since the start. When a closed interval's lost fraction
// toward leaf d grades spine k 0 through the loss thresholds and k is not
// held for d, k is held for every destination (host scope) if at least
// autoLeaves leaves have a loss on k, else for d alone (leaf scope). A spine
// held in leaf scope widens to host scope once a loss on it brings its leaves
// to autoLeaves. A hold ends by the loss hold's rule at the scope it has: a
// closed interval with a loss on k, toward d in leaf scope or toward any leaf
// in host scope, restarts its clean sends, and otherwise the sends counted on
// k there add to them, until they reach the clean sends. A host-scope release
// clears k's leaves, as does the release of the last leaf-scope hold on k.
// Each held (spine, scope) is sent one sentinel per sentinel interval for the
// host: a host-scope hold by the first queue pair to choose once one is due,
// a leaf-scope hold by the first queue pair to its leaf.
//
// Under HoldRule::Llr, per destination leaf d and spine k a
// LossLikelihoodRatio takes each send and each send declared lost toward d on
// k while k is not held toward d, and the loss that passes its test holds k
// at once, by the same choice of scope: for every destination if at least
// autoLeaves leaves have a loss on k, else for d alone. The loss thresholds
// then grade the intervals passed to the callback but hold nothing. A release
// in leaf scope restarts the test of (d, k), one in host scope that of k
// toward every leaf.
//
// Under HoldRelease::Trickle no sentinel is sent: each queue pair sends a
// spine held toward its leaf, at either scope, one in TrickleEvery of its
// sends, and each held (spine, scope) keeps a TrickleRelease that the
// answers to them move. A host-scope hold records the larger of its own
// evidence's rate and the p1 of the leaf-scope holds it ends. A release ends
// the hold at its scope as the clean sends do, restarting the tests and the
// interval's counts it covers.
class AutoLossHold {
public:
	// Called as each loss interval is closed, once per leaf seen, with the
	// leaf and what SpineScores::LossIntervalCallback passes; a spine's grade
	// is that of the hold at its scope, 0 while held.
	using LeafLossIntervalCallback = std::function<void(uint32_t leaf,
		uint64_t endNs, const std::vector<uint32_t> &sent,
		const std::vector<uint32_t> &lost, const std::vector<uint8_t> &grades)>;
	// The loss thresholds, loss interval, clean sends, sentinel interval,
	// hold rule and likelihood of parameters, as SpineScores reads them;
	// autoLeaves is at least 2.
	AutoLossHold(uint32_t spines, const SpineScores::Parameters &parameters,
		uint32_t autoLeaves);
	// Make leaf's counts, if they are not yet made.
	void AddLeaf(uint32_t leaf);
	// A send toward leaf on spine, and one declared lost, counted in the loss
	// interval in progress once those ended by now are closed.
	void CountSend(uint32_t leaf, uint32_t spine, uint64_t nowNs);
	void CountLoss(uint32_t leaf, uint32_t spine, uint64_t nowNs);
	// The loss toward leaf on spine that CountLoss counted at countedNs,
	// taken back once the loss intervals ended by now are closed. In the
	// interval in progress the count is taken out. In one closed since, the
	// hold that interval's close began on spine toward leaf, at leaf or at
	// host scope, under HoldRule::Count, is graded again without the count
	// and released where the grade is no longer 0; whether it was. A clean
	// count the loss restarted, a scope its leaf widened and the trace of a
	// closed interval stay as they were.
	bool RetractLoss(uint32_t leaf, uint32_t spine, uint64_t countedNs,
		uint64_t nowNs);
	// The acknowledgement of a send toward leaf on spine, counted likewise;
	// under cleanOnAnswers these are a hold's clean sends.
	void CountAnswer(uint32_t leaf, uint32_t spine, uint64_t nowNs);
	// Close the loss intervals ended by now and whether a sentinel toward
	// leaf is due on a spine held for it, and which; the lowest such spine is
	// taken, and the sentinel interval of the hold it has restarts now.
	bool TakeSentinel(uint32_t leaf, uint64_t nowNs, uint8_t &spine);
	// Whether spine is held toward leaf, at either scope, and whether every
	// spine is.
	bool Holds(uint32_t leaf, uint32_t spine) const;
	bool HoldsEvery(uint32_t leaf) const;
	// Whether spine is held in host scope, and the leaves with a loss on it.
	bool HostHolds(uint32_t spine) const;
	uint32_t LossLeaves(uint32_t spine) const;
	// Under HoldRule::Llr, the L of (leaf, spine).
	double LossRatio(uint32_t leaf, uint32_t spine) const;
	// Under HoldRelease::Trickle: the trickle sends' spacing, the answer to
	// a trickle send toward leaf on spine, and the R and recorded p1 of the
	// hold spine has toward leaf, host scope first.
	uint32_t TrickleEvery() const;
	void TrickleAnswer(uint32_t leaf, uint32_t spine, bool lost,
		uint64_t nowNs);
	SentinelMode Sentinels() const;
	double ReleaseValue(uint32_t leaf, uint32_t spine) const;
	double ReleaseP1(uint32_t leaf, uint32_t spine) const;
	void SetLossIntervalCallback(LeafLossIntervalCallback callback);

private:
	struct Leaf {
		Leaf(uint32_t spines, const LossLikelihoodRatio::Parameters &likelihood);
		// The interval's counts; whether a spine is held in leaf scope, its
		// clean sends, when its last leaf-scope sentinel was sent, and its
		// test under HoldRule::Llr.
		std::vector<uint32_t> sent;
		std::vector<uint32_t> lost;
		std::vector<uint32_t> answered;
		std::vector<bool> held;
		std::vector<uint32_t> clean;
		std::vector<uint64_t> sentinelNs;
		std::vector<LossLikelihoodRatio> likelihood;
		std::vector<TrickleRelease> trickle;
		// Of a leaf-scope hold an interval's close began: the end of that
		// interval and its counts toward the leaf; 0 for any other hold.
		std::vector<uint64_t> heldEndNs;
		std::vector<uint32_t> heldSent;
		std::vector<uint32_t> heldLost;
	};
	// The interval close's evidence of the hold it began on spine
	// (Leaf::heldEndNs), of a leaf's counts.
	struct Evidence {
		uint64_t endNs = 0;
		uint32_t leaf = 0;
		uint32_t sent = 0;
		uint32_t lost = 0;
	};
	// Hold spine toward leaf by the auto scope's choice, on evidence of the
	// rate given.
	void Hold(Leaf &counts, uint32_t spine, double rate);
	// Restart the test of spine toward every leaf.
	void ResetLikelihood(uint32_t spine);
	void AdvanceLossInterval(uint64_t nowNs);
	void CloseLossInterval();
	uint8_t LossGrade(uint32_t sent, uint32_t lost) const;
	// Hold spine in host scope, ending its leaf-scope holds, on evidence of
	// the rate given.
	void HoldHost(uint32_t spine, double rate);
	bool Trickle() const;
	bool LeafHolds(uint32_t spine) const;
	bool SentinelDue(uint64_t &lastNs, uint64_t nowNs) const;

	uint32_t m_spines;
	std::array<double, 3> m_lossThresholds;
	uint64_t m_lossIntervalNs;
	uint64_t m_lossIntervalEndNs;
	uint32_t m_cleanSends;
	bool m_cleanOnAnswers;
	uint64_t m_sentinelIntervalNs;
	uint32_t m_sentinelSpread;
	uint64_t m_sentinelSalt;
	uint32_t m_autoLeaves;
	HoldRule m_holdRule;
	LossLikelihoodRatio::Parameters m_likelihood;
	HoldRelease m_holdRelease;
	uint32_t m_trickleEvery;
	SentinelMode m_sentinelMode;
	// Ordered, so that the intervals are passed in leaf order.
	std::map<uint32_t, Leaf> m_leaves;
	std::vector<std::set<uint32_t>> m_lossLeaves;
	std::vector<bool> m_hostHeld;
	std::vector<uint32_t> m_hostClean;
	std::vector<uint64_t> m_hostSentinelNs;
	std::vector<TrickleRelease> m_hostTrickle;
	// Of each host-scope hold an interval's close began, as Leaf::heldEndNs.
	std::vector<Evidence> m_hostEvidence;
	LeafLossIntervalCallback m_onLossInterval;
};

// The exploration of a loss hold's held spines and the counting in it that
// every selector sending through the hold shares: RepsHoldSelector and
// MrcHoldSelector. The hold is SpineScores, the host's, a destination leaf's
// or a queue pair's own, or the host's AutoLossHold toward one destination
// leaf. Per send, the exploration send due, if any, is the hold's sentinel,
// else under SprayReceiverHold the receivers' sentinel onto a spine they
// hold and the hold does not, else under HoldRelease::Trickle the queue
// pair's trickle send (TrickleSchedule). Under SentinelMode::Data it is the
// data packet itself; otherwise the selector's own choice is taken, leaving
// out the spines the hold or the receivers hold unless they hold every
// spine, and under Duplicate the exploration send due is a copy onto its
// spine (TakeCopy). The send, and the copy, are counted in the hold, and so
// is every send declared lost.
class LossHoldExploration {
public:
	LossHoldExploration(uint32_t spines, SpineScores &hold);
	// Adds leaf to hold.
	LossHoldExploration(uint32_t spines, AutoLossHold &hold, uint32_t leaf);
	// The spine of the send in progress; choose is the selector's own choice,
	// called with Excluded set for it.
	uint8_t Choose(uint64_t nowNs, const std::function<uint8_t()> &choose);
	bool Excluded(uint32_t spine) const;
	bool ChoseTrickle() const;
	bool ChoseExploration() const;
	bool ChoseHeld() const;
	bool TakeCopy(uint8_t &spine, bool &trickle);
	// The acknowledgement of a send on spine, data or copy, and a send on
	// spine declared lost.
	void OnAck(uint32_t spine, uint64_t nowNs);
	void OnLoss(uint32_t spine, uint64_t nowNs);
	// The loss told to OnLoss at countedNs taken back, of the host's
	// AutoLossHold alone (AutoLossHold::RetractLoss); a SpineScores hold
	// keeps it.
	bool RetractLoss(uint32_t spine, uint64_t countedNs, uint64_t nowNs);
	void OnTrickleAnswer(uint32_t spine, bool lost, uint64_t nowNs);
	// Under SprayReceiverHold: the spines the receivers hold toward leaf, in
	// marks, are held as the loss hold holds them, and sent marks'
	// sentinels where the loss hold does not hold them.
	void SetReceiverHold(std::shared_ptr<ReceiverHoldMarks> marks,
		uint32_t leaf);
	void OnReceiverHold(uint32_t receiver, const SpineBitmap &bitmap,
		bool partial);

private:
	bool TakeHoldSentinel(uint64_t nowNs, uint8_t &spine);
	bool Holds(uint32_t spine) const;
	bool HoldsEvery() const;
	void CountSend(uint32_t spine, uint64_t nowNs);

	// One of the two is set.
	SpineScores *m_scores;
	AutoLossHold *m_autoHold;
	uint32_t m_leaf;
	// Whether the holds leave a spine out, set before each choice.
	bool m_excluding;
	TrickleSchedule m_trickle;
	bool m_choseTrickle;
	bool m_choseExploration;
	bool m_choseHeld;
	PendingCopy m_copy;
	ReceiverHoldView m_receiverHold;
};

// REPS (RepsSelector) over spines under SprayUniform, with a loss hold
// through LossHoldExploration: the spines the hold holds are left out of
// REPS's choices unless it holds every spine, and each held spine is sent
// the hold's sentinels, one per sentinel interval for every selector that
// shares the hold, or its trickle sends, as the exploration sends. Every
// send and every send declared lost is counted in the hold, which grades its
// spines per loss interval as the host's pooled scores do and holds a spine
// by the same loss hold. The hold is SpineScores, shared by every queue pair
// of a host or of a destination leaf, or owned by this queue pair alone,
// whose sentinels it then sends alone; or the host's AutoLossHold toward the
// destination leaf.
class RepsHoldSelector : public RepsSelector {
public:
	RepsHoldSelector(Ptr<UniformRandomVariable> random, uint32_t bufferSize,
		uint64_t freezingTimeoutNs, uint32_t explorePackets, uint32_t spines,
		SpineScores &hold);
	RepsHoldSelector(Ptr<UniformRandomVariable> random, uint32_t bufferSize,
		uint64_t freezingTimeoutNs, uint32_t explorePackets, uint32_t spines,
		std::unique_ptr<SpineScores> hold);
	RepsHoldSelector(Ptr<UniformRandomVariable> random, uint32_t bufferSize,
		uint64_t freezingTimeoutNs, uint32_t explorePackets, uint32_t spines,
		AutoLossHold &hold, uint32_t leaf);
	uint16_t Choose(uint64_t nowNs) override;
	// REPS's own answer, and the acknowledgement counted in the hold.
	void OnAck(uint16_t path, bool marked, uint64_t nowNs) override;
	void OnLoss(uint16_t path, uint64_t nowNs) override;
	bool RetractLoss(uint16_t path, uint64_t countedNs, uint64_t nowNs) override;
	bool ChoseTrickle() const override;
	bool ChoseExploration() const override;
	bool ChoseHeld() const override;
	void OnTrickleAnswer(uint16_t path, bool lost, uint64_t nowNs) override;
	bool TakeCopy(uint8_t &spine, bool &trickle) override;
	void SetReceiverHold(std::shared_ptr<ReceiverHoldMarks> marks,
		uint32_t leaf) override;
	void OnReceiverHold(uint32_t receiver, const SpineBitmap &bitmap,
		bool partial, uint64_t nowNs) override;

protected:
	bool Excluded(uint16_t path) const override;

private:
	// Declared before m_exploration, which refers to it when it is set.
	std::unique_ptr<SpineScores> m_ownHold;
	LossHoldExploration m_exploration;
};

// Simulation only: the spines toward one destination leaf that a reader of
// the switches' state names faulted, read at most once per read interval. A
// spine once named stays faulted for the run. While every spine is faulted
// none is excluded.
class FaultedSpines {
public:
	using Reader = std::function<std::vector<bool>()>;
	FaultedSpines(uint32_t spines, uint64_t readIntervalNs, Reader reader);
	// Read the switches' state if a read interval has passed since the last
	// read, or none was made.
	void Read(uint64_t nowNs);
	bool Excludes(uint32_t spine) const;

private:
	Reader m_reader;
	uint64_t m_readIntervalNs;
	uint64_t m_nextReadNs;
	std::vector<bool> m_faulted;
	bool m_excluding;
};

// The hops over which PathSelectorKind::RepsOracle reads a spine toward a
// destination leaf as faulted. RoundTrip: the four of
// SwitchQueueState::Faulted, the data's two hops and the two its
// acknowledgement returns over through the same spine. Forward: the data's
// two hops alone (SwitchQueueState::ForwardFaulted), the source leaf's uplink
// to the spine and the spine's downlink to the destination leaf, for answers
// that do not return over the data's spine (AckRoute::Spray).
enum class OracleHops : uint32_t {
	RoundTrip = 0,
	Forward,
};

// REPS (RepsSelector) over spines under SprayUniform: the cached values and
// the fresh draws are spine indices, each sent as SpineIdentification(k, k).
// Nothing is excluded, no sentinel is sent and no loss is counted.
class RepsSpineSelector : public RepsSelector {
public:
	RepsSpineSelector(Ptr<UniformRandomVariable> random, uint32_t bufferSize,
		uint64_t freezingTimeoutNs, uint32_t explorePackets, uint32_t spines);
	uint16_t Choose(uint64_t nowNs) override;
};

// REPS (RepsSelector) over spines under SprayUniform, leaving out the spines
// FaultedSpines excludes toward the queue pair's destination leaf. No
// sentinel is sent and nothing is released.
class RepsOracleSelector : public RepsSelector {
public:
	RepsOracleSelector(Ptr<UniformRandomVariable> random, uint32_t bufferSize,
		uint64_t freezingTimeoutNs, uint32_t explorePackets, uint32_t spines,
		FaultedSpines &faulted);
	uint16_t Choose(uint64_t nowNs) override;

protected:
	bool Excluded(uint16_t path) const override;

private:
	FaultedSpines &m_faulted;
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

protected:
	// A value Excluded names is passed over as if it were not in the set: its
	// state is neither read nor moved. Excluded must leave some value in.
	virtual bool Excluded(uint16_t path) const;

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

// MRC (MrcSelector) over spines under SprayUniform, its set the spine
// indices, with RepsHold's loss hold through LossHoldExploration, as
// RepsHoldSelector has it: the spines the hold, or the receivers under
// SprayReceiverHold, hold toward the queue pair's destination are left out
// of MRC's choices unless they hold every spine, the exploration sends are
// sent as the hold's HoldRelease and SentinelMode have them, and every send
// and every send declared lost is counted in the hold. The hold is
// SpineScores, the host's or the destination leaf's, or the host's
// AutoLossHold toward the destination leaf. A declared loss moves MRC's
// state too, and MRC's probes name spines.
class MrcHoldSelector : public MrcSelector {
public:
	MrcHoldSelector(Ptr<UniformRandomVariable> random, uint32_t spines,
		uint64_t skipNs, uint64_t probeIntervalNs, SpineScores &hold);
	MrcHoldSelector(Ptr<UniformRandomVariable> random, uint32_t spines,
		uint64_t skipNs, uint64_t probeIntervalNs, AutoLossHold &hold,
		uint32_t leaf);
	uint16_t Choose(uint64_t nowNs) override;
	// MRC's own answer, and the acknowledgement counted in the hold.
	void OnAck(uint16_t path, bool marked, uint64_t nowNs) override;
	void OnLoss(uint16_t path, uint64_t nowNs) override;
	// The hold's count taken back; MRC's own state keeps the loss.
	bool RetractLoss(uint16_t path, uint64_t countedNs, uint64_t nowNs) override;
	bool TakeProbe(uint64_t nowNs, uint16_t &path) override;
	bool ChoseTrickle() const override;
	bool ChoseExploration() const override;
	bool ChoseHeld() const override;
	void OnTrickleAnswer(uint16_t path, bool lost, uint64_t nowNs) override;
	bool TakeCopy(uint8_t &spine, bool &trickle) override;
	void SetReceiverHold(std::shared_ptr<ReceiverHoldMarks> marks,
		uint32_t leaf) override;
	void OnReceiverHold(uint32_t receiver, const SpineBitmap &bitmap,
		bool partial, uint64_t nowNs) override;

protected:
	bool Excluded(uint16_t path) const override;

private:
	LossHoldExploration m_exploration;
};

// Themis's source (ThemisSelector) with RepsHold's loss hold through
// LossHoldExploration, as MrcHoldSelector has it: the spines the hold, or the
// receivers under SprayReceiverHold, hold toward the queue pair's destination
// are Excluded unless they hold every spine, the exploration sends are sent as
// the hold's HoldRelease and SentinelMode have them, and every send and every
// send declared lost is counted in the hold. A loss hold and Themis's
// avoidance then share one spraying rule and one redirect.
class ThemisHoldSelector : public ThemisSelector {
public:
	ThemisHoldSelector(Ptr<UniformRandomVariable> random, uint32_t spines,
		uint32_t paths, uint64_t window, bool reroute, Counters counters,
		SpineScores &hold);
	ThemisHoldSelector(Ptr<UniformRandomVariable> random, uint32_t spines,
		uint32_t paths, uint64_t window, bool reroute, Counters counters,
		AutoLossHold &hold, uint32_t leaf);
	// With the base given, for tests.
	ThemisHoldSelector(Ptr<UniformRandomVariable> random, uint32_t spines,
		uint32_t paths, uint64_t window, bool reroute, uint32_t base,
		SpineScores &hold);
	uint16_t Choose(uint64_t nowNs) override;
	void OnAck(uint16_t path, bool marked, uint64_t nowNs) override;
	void OnLoss(uint16_t path, uint64_t nowNs) override;
	bool RetractLoss(uint16_t path, uint64_t countedNs, uint64_t nowNs) override;
	bool ChoseTrickle() const override;
	bool ChoseExploration() const override;
	bool ChoseHeld() const override;
	void OnTrickleAnswer(uint16_t path, bool lost, uint64_t nowNs) override;
	bool TakeCopy(uint8_t &spine, bool &trickle) override;
	void SetReceiverHold(std::shared_ptr<ReceiverHoldMarks> marks,
		uint32_t leaf) override;
	void OnReceiverHold(uint32_t receiver, const SpineBitmap &bitmap,
		bool partial, uint64_t nowNs) override;

protected:
	bool Excluded(uint8_t spine) const override;

private:
	LossHoldExploration m_exploration;
};

} /* namespace ns3 */

#endif /* PATH_SELECTOR_H */
