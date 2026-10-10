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

void SupervisedEwma::Add(double sample, uint32_t packets,
		const Parameters &parameters){
	if (parameters.intervalSamples > 0 && packets >= parameters.intervalSamples){
		m_value = sample;
		m_rise = m_fall = 0;
		return;
	}
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
	const uint32_t spines = m_spines.size();
	std::array<double, SpineReport::kMaxSpines> marks{}, trims{}, delays{};
	for (uint32_t k = 0; k < spines; k++){
		Spine &spine = m_spines[k];
		SpineInterval &counted = spine.counting;
		spine.windowArrivals += counted.arrivals;
		if (counted.moved > 0 && !spine.held){
			// Counted afresh from the hold, so that the arrivals before it
			// cannot release it.
			spine.held = true;
			spine.windowArrivals = 0;
		}
		if (counted.arrivals > 0){
			spine.markFraction.Add(
				static_cast<double>(counted.marked) / counted.arrivals,
				counted.arrivals, m_parameters.fractions);
			if (m_parameters.oneWayDelay)
				spine.delayNs.Add(
					static_cast<double>(counted.delaySumNs) / counted.arrivals -
						spine.leastDelayNs,
					counted.arrivals, m_parameters.delayNs);
		}
		const uint32_t packets = counted.arrivals + counted.trimmed;
		if (packets > 0)
			spine.trimFraction.Add(static_cast<double>(counted.trimmed) / packets,
				packets, m_parameters.fractions);
		marks[k] = spine.markFraction.Value();
		trims[k] = spine.trimFraction.Value();
		delays[k] = spine.delayNs.Value();
	}
	JudgeWindow();
	// Spines alike all earn the top grade against their median, so the
	// median reference, unlike a rank, has no loser on a healthy fabric.
	const bool median = m_parameters.reference == GradeReference::Median;
	const double markReference = median ? Median(marks) : 0;
	const double trimReference = median ? Median(trims) : 0;
	const double delayReference = median ? Median(delays) : 0;
	bool everySpineBelowTop = true;
	for (uint32_t k = 0; k < spines; k++){
		Spine &spine = m_spines[k];
		SpineInterval &counted = spine.counting;
		// A trimmed packet and a marked one both met a queue past a threshold,
		// so whichever kind stands out more is the spine's congestion; adding
		// them would count one queue twice.
		everySpineBelowTop &= Earned(std::max(marks[k], trims[k]), delays[k]) <
			SpineReport::kTopGrade;
		const uint8_t earned = Earned(
			std::max(marks[k] - markReference, trims[k] - trimReference),
			delays[k] - delayReference);
		counted.markFraction = marks[k];
		counted.trimFraction = trims[k];
		counted.delayNs = delays[k];
		counted.held = spine.held;
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

void SpineGrader::JudgeWindow(){
	const uint32_t spines = m_spines.size();
	std::array<double, SpineReport::kMaxSpines> arrivals{};
	for (uint32_t k = 0; k < spines; k++)
		arrivals[k] = m_spines[k].windowArrivals;
	const double median = Median(arrivals);
	if (median < m_parameters.absenceMinimumMedian)
		return;
	// A held spine is sent only the exploration floor, a small share of what
	// the median spine is sent; that share, and not the median's, is what its
	// arrivals return to while its path works.
	const double release = m_parameters.releaseFractionOfFloor *
		m_parameters.floorShareOfMedian * median;
	for (uint32_t k = 0; k < spines; k++){
		Spine &spine = m_spines[k];
		if (spine.held){
			spine.held = spine.windowArrivals < release;
		}else{
			spine.counting.absent = spine.windowArrivals <
				m_parameters.absenceFractionOfMedian * median;
			spine.held = spine.counting.absent;
		}
		spine.windowArrivals = 0;
	}
}

uint8_t SpineGrader::Earned(double congestionCost, double delayCostNs) const{
	const uint8_t byCongestion =
		Quantize(congestionCost, m_parameters.congestionThresholds);
	if (byCongestion < SpineReport::kTopGrade || !m_parameters.oneWayDelay)
		return byCongestion;
	return Quantize(delayCostNs, m_parameters.delayThresholdsNs);
}

double SpineGrader::Median(
		std::array<double, SpineReport::kMaxSpines> values) const{
	const uint32_t spines = m_spines.size();
	const auto middle = values.begin() + spines / 2;
	std::nth_element(values.begin(), middle, values.begin() + spines);
	if (spines % 2 == 1)
		return *middle;
	return (*middle + *std::max_element(values.begin(), middle)) / 2;
}

} // namespace ns3
