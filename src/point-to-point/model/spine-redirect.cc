#include "spine-redirect.h"
#include <algorithm>

namespace ns3 {

double SpineRedirect::Score(double omega, double delayNs,
		uint64_t baseOneWayDelayNs, double marked){
	// omega / (L / L0 + eps) taken with a single division.
	const double base = baseOneWayDelayNs;
	return omega * base / (delayNs + kEpsilon * base) +
		(1 - omega) * (1 - marked);
}

bool SpineRedirect::Redirects(double best, double score, double flag){
	return best - score > flag * best;
}

uint8_t SpineRedirect::OnArrival(uint8_t spine, uint64_t delayNs, bool marked,
		uint64_t nowNs, const Parameters &parameters){
	const double score = Score(parameters.omega, delayNs,
		parameters.baseOneWayDelayNs, marked ? 1 : 0);
	if (m_best == kNoRedirect || nowNs >= m_cycleEndNs){
		m_best = spine;
		m_bestScore = score;
		m_cycleEndNs = nowNs + parameters.validityNs;
		return kNoRedirect;
	}
	if (spine == m_best || score > m_bestScore){
		m_best = spine;
		m_bestScore = score;
		return kNoRedirect;
	}
	return Redirects(m_bestScore, score, parameters.flag) ? m_best : kNoRedirect;
}

uint8_t SpineRedirect::Best() const{
	return m_best;
}

double SpineRedirect::BestScore() const{
	return m_bestScore;
}

uint64_t SpineRedirect::CycleEndNs() const{
	return m_cycleEndNs;
}

void HostSpineRedirect::OnClose(
		const std::vector<SpineGrader::SpineInterval> &spines,
		const Parameters &parameters){
	m_scores.resize(spines.size());
	const double base = parameters.baseOneWayDelayNs;
	for (uint32_t k = 0; k < spines.size(); k++){
		const SpineGrader::SpineInterval &spine = spines[k];
		m_scores[k] = spine.held ? 0 : SpineRedirect::Score(parameters.omega,
			parameters.oneWayDelay ? base + spine.delayNs : base,
			parameters.baseOneWayDelayNs,
			std::max(spine.markFraction, spine.trimFraction));
	}
	// Starting from the current best and replacing it only by a strictly
	// higher score keeps it on a tie, and otherwise takes the lowest index of
	// the highest.
	uint8_t best = m_best;
	for (uint32_t k = 0; k < m_scores.size(); k++)
		if (best == kNoRedirect || m_scores[k] > m_scores[best])
			best = k;
	m_best = best;
}

uint8_t HostSpineRedirect::OnArrival(uint8_t spine,
		const Parameters &parameters) const{
	if (m_best == kNoRedirect || spine == m_best)
		return kNoRedirect;
	return SpineRedirect::Redirects(m_scores[m_best], m_scores[spine],
		parameters.flag) ? m_best : kNoRedirect;
}

uint8_t HostSpineRedirect::Best() const{
	return m_best;
}

double HostSpineRedirect::SpineScore(uint32_t spine) const{
	return m_scores[spine];
}

} // namespace ns3
