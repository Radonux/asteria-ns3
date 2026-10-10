#include "spine-grader.h"
#include <ns3/assert.h>
#include <algorithm>

namespace ns3 {

namespace {

// 3 below the first threshold, 2 below the second, 1 below the third, else 0.
uint8_t Quantize(double value, const double (&thresholds)[3]){
	for (uint8_t i = 0; i < 3; i++){
		if (value < thresholds[i])
			return SpineReport::kTopGrade - i;
	}
	return 0;
}

} // namespace

void SupervisedEwma::Add(double sample, const Parameters &parameters){
	m_rise = std::max(0.0, m_rise + sample - m_value - parameters.slack);
	m_fall = std::max(0.0, m_fall + m_value - sample - parameters.slack);
	if (m_rise > parameters.threshold || m_fall > parameters.threshold){
		m_value = sample;
		m_rise = m_fall = 0;
		return;
	}
	m_value += parameters.gain * (sample - m_value);
}

double SupervisedEwma::Value() const{
	return m_value;
}

SpineGrader::SpineGrader(const Parameters &parameters)
	: m_parameters(parameters), m_spines(parameters.spines),
	  m_lastInterval(parameters.spines), m_lastHopTrims(0),
	  m_lastIntervalLastHopTrims(0),
	  m_intervalEndNs(parameters.phaseNs + parameters.intervalNs)
{
	NS_ASSERT_MSG(parameters.spines > 0 &&
			parameters.spines <= SpineReport::kMaxSpines,
		"a report grades at most SpineReport::kMaxSpines spines");
	NS_ASSERT_MSG(parameters.intervalNs > 0, "a report is issued per interval");
}

bool SpineGrader::Advance(uint64_t nowNs){
	if (nowNs < m_intervalEndNs)
		return false;
	Close(nowNs);
	// The intervals that passed with nothing to count issue no report of
	// their own; the next interval is the one now falls in.
	m_intervalEndNs += m_parameters.intervalNs *
		(1 + (nowNs - m_intervalEndNs) / m_parameters.intervalNs);
	return true;
}

void SpineGrader::OnArrival(uint8_t requested, uint8_t carrying, bool marked,
		uint64_t delayNs){
	SpineInterval &counting = m_spines[carrying].counting;
	counting.arrivals++;
	counting.marked += marked;
	if (requested != carrying)
		m_spines[requested].counting.moved++;
	if (m_parameters.oneWayDelay){
		counting.delaySumNs += delayNs;
		m_spines[carrying].leastDelayNs =
			std::min(m_spines[carrying].leastDelayNs, delayNs);
	}
}

void SpineGrader::OnTrim(uint8_t carrying, bool lastHop){
	if (lastHop)
		m_lastHopTrims++;
	else
		m_spines[carrying].counting.trimmed++;
}

const SpineReport &SpineGrader::Report() const{
	return m_report;
}

const std::vector<SpineGrader::SpineInterval> &SpineGrader::LastInterval() const{
	return m_lastInterval;
}

uint32_t SpineGrader::LastIntervalLastHopTrims() const{
	return m_lastIntervalLastHopTrims;
}

void SpineGrader::Close(uint64_t nowNs){
	bool everySpineBelowTop = true;
	for (uint32_t k = 0; k < m_spines.size(); k++){
		Spine &spine = m_spines[k];
		SpineInterval &counted = spine.counting;
		if (counted.arrivals > 0){
			spine.markFraction.Add(
				static_cast<double>(counted.marked) / counted.arrivals,
				m_parameters.marks);
			if (m_parameters.oneWayDelay)
				spine.delayNs.Add(
					static_cast<double>(counted.delaySumNs) / counted.arrivals -
						spine.leastDelayNs,
					m_parameters.delayNs);
		}
		if (counted.trimmed > 0 || counted.moved > 0)
			spine.heldUntilNs = m_intervalEndNs +
				m_parameters.holdDownIntervals * m_parameters.intervalNs;
		const uint8_t earned = Earned(spine);
		everySpineBelowTop &= earned < SpineReport::kTopGrade;
		counted.markFraction = spine.markFraction.Value();
		counted.delayNs = spine.delayNs.Value();
		counted.held = nowNs < spine.heldUntilNs;
		m_report.SetGrade(k, counted.held ? 0 : earned);
		m_lastInterval[k] = counted;
		counted = SpineInterval{};
	}
	// Every spine's packets cross the receiver's downlink, so congestion there
	// shows on all of them at once and on none more than another.
	m_report.edgeCongested = m_lastHopTrims > 0 || everySpineBelowTop;
	m_report.sequence++;
	m_lastIntervalLastHopTrims = m_lastHopTrims;
	m_lastHopTrims = 0;
}

uint8_t SpineGrader::Earned(const Spine &spine) const{
	const uint8_t byMarks =
		Quantize(spine.markFraction.Value(), m_parameters.markThresholds);
	if (byMarks < SpineReport::kTopGrade || !m_parameters.oneWayDelay)
		return byMarks;
	return Quantize(spine.delayNs.Value(), m_parameters.delayThresholdsNs);
}

} // namespace ns3
