#ifndef SPINE_REDIRECT_H
#define SPINE_REDIRECT_H

#include "load-balancing.h"
#include "spine-grader.h"
#include <stdint.h>
#include <vector>

namespace ns3 {

// How a receiver that redirects its sender treats a header trimmed before the
// last hop: as an arrival on its carrying spine marked with CE, or not at all.
enum class TrimScoring : uint32_t {
	Mark = 0,
	Ignore,
};

// A receive queue pair's choice of the spine its sender should send on,
// modelled on the ACK redirection of MPTR (Wang et al., IEEE TII 2026): each
// arrival over a spine is scored by its one-way delay and its mark, the best
// spine and its score are kept for a validity cycle, and an arrival scoring
// more than a fraction below the best is answered with a request to move to
// the best.
//
// Within a cycle an arrival on the best spine refreshes its score and one
// scoring above it replaces it; the first arrival at or after the cycle's end
// starts a new cycle with its own spine as the best.
class SpineRedirect {
public:
	static constexpr double kEpsilon = 0.001;
	struct Parameters {
		// The weight of the delay term against the mark term.
		double omega;
		// The fraction of the best score an arrival's may fall short of
		// without a redirect.
		double flag;
		uint64_t validityNs;
		// The fabric's one-way delay without queueing, which the delay term
		// counts delay in.
		uint64_t baseOneWayDelayNs;
	};
	// omega / (delay / base + kEpsilon) + (1 - omega) (1 - marked): the delay
	// term is in (0, omega / kEpsilon] and is omega / (1 + kEpsilon) at the
	// base delay, and the mark term falls from 1 - omega to zero as the
	// marked fraction, 0 or 1 for one packet, rises.
	static double Score(double omega, double delayNs, uint64_t baseOneWayDelayNs,
		double marked);
	// Whether an arrival scoring score is answered with a redirect to a best
	// spine scoring best: when it falls short by more than flag of it.
	static bool Redirects(double best, double score, double flag);
	// An arrival over spine with its one-way delay and mark at nowNs; the
	// spine the answer to it asks the sender to move to, or kNoRedirect.
	uint16_t OnArrival(uint8_t spine, uint64_t delayNs, bool marked,
		uint64_t nowNs, const Parameters &parameters);
	// The best spine, kNoRedirect before the first arrival.
	uint16_t Best() const;
	double BestScore() const;
	uint64_t CycleEndNs() const;

private:
	uint16_t m_best = kNoRedirect;
	double m_bestScore = 0;
	uint64_t m_cycleEndNs = 0;
};

// A receiving host's choice of the spine every connection into it should
// send on, from the per-spine averages of the host's grader rather than from
// single arrivals. At each close of the grader's interval each spine is
// scored by SpineRedirect::Score from its congestion average, the larger of
// its marked and its trimmed fraction's, and, with one-way delay, the base
// delay plus its delay average; a held spine scores 0. The best is the
// highest-scored spine, the current best on a tie, else the lowest index.
// Between closes an arrival on another spine is answered with the best as
// SpineRedirect answers it; before the first close nothing is redirected.
class HostSpineRedirect {
public:
	struct Parameters {
		double omega;
		double flag;
		uint64_t baseOneWayDelayNs;
		bool oneWayDelay;
	};
	void OnClose(const std::vector<SpineGrader::SpineInterval> &spines,
		const Parameters &parameters);
	uint16_t OnArrival(uint8_t spine, const Parameters &parameters) const;
	uint16_t Best() const;
	// Spine's score at the latest close.
	double SpineScore(uint32_t spine) const;

private:
	std::vector<double> m_scores;
	uint16_t m_best = kNoRedirect;
};

} // namespace ns3

#endif /* SPINE_REDIRECT_H */
