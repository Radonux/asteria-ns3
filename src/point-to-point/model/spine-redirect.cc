#include "spine-redirect.h"

namespace ns3 {

double SpineRedirect::Score(double omega, uint64_t delayNs,
		uint64_t baseOneWayDelayNs, bool marked){
	// omega / (L / L0 + eps) taken with a single division.
	const double base = baseOneWayDelayNs;
	return omega * base / (delayNs + kEpsilon * base) +
		(marked ? 0 : 1 - omega);
}

uint8_t SpineRedirect::OnArrival(uint8_t spine, uint64_t delayNs, bool marked,
		uint64_t nowNs, const Parameters &parameters){
	const double score =
		Score(parameters.omega, delayNs, parameters.baseOneWayDelayNs, marked);
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
	return m_bestScore - score > parameters.flag * m_bestScore
		? m_best : kNoRedirect;
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

} // namespace ns3
