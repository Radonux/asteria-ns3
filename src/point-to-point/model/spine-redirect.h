#ifndef SPINE_REDIRECT_H
#define SPINE_REDIRECT_H

#include "load-balancing.h"
#include <stdint.h>

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
	// base delay, and the mark term is 1 - omega or zero.
	static double Score(double omega, uint64_t delayNs,
		uint64_t baseOneWayDelayNs, bool marked);
	// An arrival over spine with its one-way delay and mark at nowNs; the
	// spine the answer to it asks the sender to move to, or kNoRedirect.
	uint8_t OnArrival(uint8_t spine, uint64_t delayNs, bool marked,
		uint64_t nowNs, const Parameters &parameters);
	// The best spine, kNoRedirect before the first arrival.
	uint8_t Best() const;
	double BestScore() const;
	uint64_t CycleEndNs() const;

private:
	uint8_t m_best = kNoRedirect;
	double m_bestScore = 0;
	uint64_t m_cycleEndNs = 0;
};

} // namespace ns3

#endif /* SPINE_REDIRECT_H */
