#ifndef SPINE_GRADER_H
#define SPINE_GRADER_H

#include "spine-report.h"
#include <stdint.h>
#include <array>
#include <vector>

namespace ns3 {

// An exponentially weighted moving average of a sample taken once per
// interval, supervised by a two-sided CUSUM (Page, 1954): the sums of the
// samples' excess over the average, and of their shortfall below it, each net
// of a slack per sample, are kept, and when either passes a threshold the
// average restarts at the sample and both sums at zero. A step in the samples
// moves the average at once instead of at the pace of its gain. A sample drawn
// from at least intervalSamples packets replaces the average outright, unless
// intervalSamples is zero. Before the first sample the average is zero.
class SupervisedEwma {
public:
	struct Parameters {
		double gain;
		double slack;
		double threshold;
		uint32_t intervalSamples;
	};
	// sample is the mean of a value over packets packets.
	void Add(double sample, uint32_t packets, const Parameters &parameters);
	double Value() const;

private:
	double m_value = 0;
	double m_rise = 0;
	double m_fall = 0;
};

// A receiving host's grades of the spines its data arrives over, by the
// carrying spine each packet's identification names. Over fixed intervals it
// counts per spine the data packets that arrived, those that arrived with CE,
// the packets trimmed before the last hop and the packets a source leaf moved
// off the spine they requested; under one-way delay it also sums each
// packet's delay. At the first event of a new interval the counts of the one
// that ended update three supervised averages per spine, the fraction of
// arrivals marked, the fraction of its packets trimmed before the last hop
// (of arrivals and such trims together), and the mean one-way delay above the
// least the spine has shown, and a report is issued.
//
// A spine's grade comes from its costs through thresholds, its congestion cost
// taking precedence over its delay cost: a congestion cost at or above the
// first threshold grades the spine by it alone, and otherwise the delay cost,
// where delay is measured, grades it. Under the absolute reference a cost is
// the average itself; under the median reference it is the average's excess
// over the median of the spines' averages. The congestion cost is the larger
// of the marked and the trimmed fraction's costs. A moved packet holds the
// spine at grade 0 for a fixed number of intervals from the end of the
// interval it was counted in, whatever the costs say, and so does an interval
// in which the spine's arrivals fall below a fraction of the median spine's.
// The edge bit is set when the receiver's own downlink trimmed a packet in the
// interval, or when no spine's averages earn it the top grade under the
// absolute reference.
class SpineGrader {
public:
	enum class GradeReference : uint32_t {
		Absolute = 0,
		Median,
	};
	struct Parameters {
		uint32_t spines;
		uint64_t intervalNs;
		// Where this receiver's intervals start within the first one.
		uint64_t phaseNs;
		// The marked and the trimmed fraction's averages.
		SupervisedEwma::Parameters fractions;
		// The congestion costs from which a spine grades 2, 1 and 0.
		double congestionThresholds[3];
		bool oneWayDelay;
		SupervisedEwma::Parameters delayNs;
		// The delays above a spine's least from which it grades 2, 1 and 0.
		double delayThresholdsNs[3];
		GradeReference reference;
		uint32_t holdDownIntervals;
		// A spine is held down for arriving less than this fraction of the
		// median spine's arrivals, once that median is at least the minimum.
		double absenceFractionOfMedian;
		uint32_t absenceMinimumMedian;
	};
	// What one spine showed over the last interval that ended, and what it
	// was graded on that.
	struct SpineInterval {
		uint32_t arrivals = 0;
		uint32_t marked = 0;
		uint32_t trimmed = 0;
		uint32_t moved = 0;
		uint64_t delaySumNs = 0;
		double markFraction = 0;
		double trimFraction = 0;
		double delayNs = 0;
		bool absent = false;
		bool held = false;
	};

	explicit SpineGrader(const Parameters &parameters);
	// Close the interval in progress if it has ended by now and issue the
	// report; true if one was issued. Called before each event is counted.
	bool Advance(uint64_t nowNs);
	// A data packet requested on one spine and carried by another, marked or
	// not, with its one-way delay, which is ignored without one-way delay.
	void OnArrival(uint8_t requested, uint8_t carrying, bool marked,
		uint64_t delayNs);
	// A data packet trimmed on its way here, at the last hop or before it.
	void OnTrim(uint8_t carrying, bool lastHop);
	const SpineReport &Report() const;
	// The interval the latest report closed, per spine, and the last-hop trims
	// counted in it.
	const std::vector<SpineInterval> &LastInterval() const;
	uint32_t LastIntervalLastHopTrims() const;

private:
	struct Spine {
		SpineInterval counting;
		SupervisedEwma markFraction;
		SupervisedEwma trimFraction;
		SupervisedEwma delayNs;
		uint64_t leastDelayNs = UINT64_MAX;
		uint64_t heldUntilNs = 0;
	};
	void Close(uint64_t nowNs);
	// The grade a spine's congestion and delay costs earn it.
	uint8_t Earned(double congestionCost, double delayCostNs) const;
	// The median of the first spines values.
	double Median(std::array<double, SpineReport::kMaxSpines> values) const;

	Parameters m_parameters;
	std::vector<Spine> m_spines;
	std::vector<SpineInterval> m_lastInterval;
	uint32_t m_lastHopTrims;
	uint32_t m_lastIntervalLastHopTrims;
	uint64_t m_intervalEndNs;
	SpineReport m_report;
};

} // namespace ns3

#endif /* SPINE_GRADER_H */
