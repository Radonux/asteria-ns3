#include "path-selector.h"
#include "load-balancing.h"
#include "spine-grader.h"
#include <ns3/assert.h>
#include <algorithm>
#include <cmath>
#include <utility>

namespace ns3 {

void PathSelector::OnAck(uint16_t, bool, uint64_t){
}

void PathSelector::OnTrim(uint16_t, bool, bool, uint64_t){
}

void PathSelector::OnLoss(uint16_t, uint64_t){
}

void PathSelector::OnTimeout(uint64_t){
}

bool PathSelector::TakeProbe(uint64_t, uint16_t &){
	return false;
}

void PathSelector::OnProbeAnswer(uint16_t, bool, uint64_t){
}

void PathSelector::OnReport(const SpineReport &, uint64_t){
}

void PathSelector::OnRedirect(uint8_t, uint8_t, uint64_t){
}

UniformSpineSelector::UniformSpineSelector(Ptr<UniformRandomVariable> random,
		uint32_t spines)
	: m_random(random), m_spines(spines)
{
}

uint16_t UniformSpineSelector::Choose(uint64_t){
	const uint8_t spine = m_random->GetInteger(0, m_spines - 1);
	return SpineIdentification(spine, spine);
}

AckRedirectSelector::AckRedirectSelector(Ptr<UniformRandomVariable> random,
		uint32_t spines)
	: m_uniform(random, spines), m_spine(kNoRedirect)
{
	NS_ASSERT_MSG(spines < kNoRedirect, "a spine's index is below kNoRedirect");
}

uint16_t AckRedirectSelector::Choose(uint64_t nowNs){
	if (m_spine == kNoRedirect)
		return m_uniform.Choose(nowNs);
	return SpineIdentification(m_spine, m_spine);
}

void AckRedirectSelector::OnRedirect(uint8_t spine, uint8_t carrying, uint64_t){
	m_spine = spine != kNoRedirect ? spine : carrying;
}

SpineScores::SpineScores(uint32_t spines, const Parameters &parameters)
	: m_parameters(parameters),
	  m_decay(std::llround(parameters.gamma * 65536)),
	  m_accumulated(spines), m_reported(false), m_sequence(0),
	  m_reportedNs(0), m_nextDecayNs(UINT64_MAX), m_pool(nullptr),
	  m_poolWeight(0), m_sent(spines), m_lost(spines),
	  m_lossIntervalNs(parameters.lossIntervalNs > 0
		? parameters.lossIntervalNs : parameters.reportIntervalNs),
	  m_lossIntervalEndNs(m_lossIntervalNs),
	  m_lossGrades(spines, SpineReport::kTopGrade), m_lossHeld(spines),
	  m_lossClean(spines), m_held(spines),
	  m_sentinelNs(spines),
	  m_sentinelIntervalNs(parameters.sentinelIntervalNs > 0
		? parameters.sentinelIntervalNs : parameters.reportIntervalNs),
	  m_poolDecayEndNs(parameters.poolDecayIntervalNs),
	  m_gradeSum(spines), m_gradeCount(spines)
{
	NS_ASSERT_MSG(spines > 0 && spines <= SpineReport::kMaxSpines,
		"a report grades at most SpineReport::kMaxSpines spines");
	NS_ASSERT_MSG(m_decay > 0 && m_decay <= 65536, "gamma is in (0, 1]");
	NS_ASSERT_MSG(parameters.candidates > 0, "a spine is drawn from candidates");
	NS_ASSERT_MSG(!parameters.lossThresholds ||
			((*parameters.lossThresholds)[0] > 0 &&
				(*parameters.lossThresholds)[0] < (*parameters.lossThresholds)[1] &&
				(*parameters.lossThresholds)[1] < (*parameters.lossThresholds)[2]),
		"the loss thresholds are three increasing positive fractions");
	NS_ASSERT_MSG(!parameters.lossThresholds || m_lossIntervalNs > 0,
		"a pool grades loss per interval");
	NS_ASSERT_MSG(parameters.heldExploration !=
			HeldExploration::Sentinel || m_sentinelIntervalNs > 0,
		"a held spine's sentinels are an interval apart");
	// The largest score a decay takes exactly the top grade from.
	m_scores.assign(spines,
		SpineReport::kTopGrade * kGradeScale * uint64_t{65536} / m_decay);
	Accumulate();
	// The first pool decay interval starts from the scores decayed once, so
	// that the top grade holds them where they start.
	m_base = m_scores;
	Decay(m_base);
}

void SpineScores::OnReport(const SpineReport &report, uint64_t nowNs){
	// Acknowledgements return over different spines and overtake one another,
	// so a report can arrive after a newer one. The sequence has eight bits,
	// which tell newer from older for half their range; after that many report
	// intervals without a report, the one that arrives is the latest.
	const bool newer = static_cast<int8_t>(report.sequence - m_sequence) > 0;
	if (m_reported && !newer &&
			nowNs - m_reportedNs < 128 * m_parameters.reportIntervalNs)
		return;
	m_reported = true;
	m_sequence = report.sequence;
	SpineReport combined = report;
	if (m_parameters.lossThresholds){
		// A spine not sent on and not in loss hold earns the top grade, and
		// keeps the report's.
		for (uint32_t k = 0; k < m_scores.size(); k++)
			combined.SetGrade(k, std::min(report.Grade(k),
				HeldLossGrade(k, m_sent[k], m_lost[k])));
		std::fill(m_sent.begin(), m_sent.end(), 0);
		std::fill(m_lost.begin(), m_lost.end(), 0);
	}
	Take(combined, nowNs);
	if (m_pool == nullptr)
		return;
	SpineReport pooled = combined;
	if (m_pool->m_parameters.lossThresholds){
		m_pool->AdvanceLossInterval(nowNs);
		for (uint32_t k = 0; k < m_scores.size(); k++)
			pooled.SetGrade(k,
				std::min(combined.Grade(k), m_pool->m_lossGrades[k]));
	}
	if (m_pool->m_parameters.poolDecayIntervalNs > 0)
		m_pool->PoolTake(pooled, nowNs);
	else
		m_pool->Take(pooled, nowNs);
}

void SpineScores::OnSend(uint32_t spine, uint64_t nowNs){
	NS_ASSERT_MSG(spine < m_scores.size(), "a send is on a scored spine");
	if (m_parameters.lossThresholds)
		m_sent[spine]++;
	if (m_pool != nullptr)
		m_pool->CountIntervalSend(spine, nowNs);
}

void SpineScores::OnLoss(uint32_t spine, uint64_t nowNs){
	NS_ASSERT_MSG(spine < m_scores.size(), "a loss is on a scored spine");
	if (m_parameters.lossThresholds)
		m_lost[spine]++;
	if (m_pool != nullptr)
		m_pool->CountIntervalLoss(spine, nowNs);
}

void SpineScores::CountIntervalSend(uint32_t spine, uint64_t nowNs){
	if (!m_parameters.lossThresholds)
		return;
	AdvanceLossInterval(nowNs);
	m_sent[spine]++;
}

void SpineScores::CountIntervalLoss(uint32_t spine, uint64_t nowNs){
	if (!m_parameters.lossThresholds)
		return;
	AdvanceLossInterval(nowNs);
	m_lost[spine]++;
}

uint8_t SpineScores::LossGrade(uint32_t sent, uint32_t lost) const{
	if (sent == 0)
		return SpineReport::kTopGrade;
	return QuantizeGrade(std::min(1.0, static_cast<double>(lost) / sent),
		*m_parameters.lossThresholds);
}

uint8_t SpineScores::HeldLossGrade(uint32_t spine, uint32_t sent,
		uint32_t lost){
	const uint8_t grade = LossGrade(sent, lost);
	const uint32_t clean = m_parameters.lossHoldCleanSentinels;
	if (clean == 0)
		return grade;
	if (!m_lossHeld[spine]){
		if (grade == 0){
			m_lossHeld[spine] = true;
			m_lossClean[spine] = 0;
		}
		return grade;
	}
	if (lost > 0){
		m_lossClean[spine] = 0;
		return 0;
	}
	m_lossClean[spine] += sent;
	if (m_lossClean[spine] < clean)
		return 0;
	m_lossHeld[spine] = false;
	m_lossClean[spine] = 0;
	return SpineReport::kTopGrade;
}

void SpineScores::AdvanceLossInterval(uint64_t nowNs){
	if (nowNs < m_lossIntervalEndNs)
		return;
	// The counts are those of the interval that ended at m_lossIntervalEndNs;
	// any later interval that ended by now had nothing counted.
	const bool lastCounted = nowNs - m_lossIntervalEndNs < m_lossIntervalNs;
	std::vector<uint8_t> closed(m_scores.size());
	for (uint32_t k = 0; k < m_scores.size(); k++)
		closed[k] = HeldLossGrade(k, m_sent[k], m_lost[k]);
	if (m_onLossInterval)
		m_onLossInterval(m_lossIntervalEndNs, m_sent, m_lost, closed);
	// The intervals after it counted nothing, which leaves a spine in loss
	// hold at 0.
	if (lastCounted)
		m_lossGrades = closed;
	else
		for (uint32_t k = 0; k < m_scores.size(); k++)
			m_lossGrades[k] = m_lossHeld[k] ? 0 : SpineReport::kTopGrade;
	std::fill(m_sent.begin(), m_sent.end(), 0);
	std::fill(m_lost.begin(), m_lost.end(), 0);
	m_lossIntervalEndNs += m_lossIntervalNs *
		(1 + (nowNs - m_lossIntervalEndNs) / m_lossIntervalNs);
}

void SpineScores::SetLossIntervalCallback(LossIntervalCallback callback){
	m_onLossInterval = std::move(callback);
}

void SpineScores::Take(const SpineReport &report, uint64_t nowNs){
	Age(nowNs);
	m_reportedNs = nowNs;
	// A report is due every interval, and the acknowledgements carrying one
	// arrive with an interval of jitter.
	m_nextDecayNs = nowNs + 2 * m_parameters.reportIntervalNs;
	if (report.edgeCongested && m_parameters.edgeFreeze)
		return;
	Decay();
	for (uint32_t k = 0; k < m_scores.size(); k++){
		m_scores[k] += report.Grade(k) * kGradeScale;
		m_held[k] = report.Grade(k) == 0;
	}
	Accumulate();
}

void SpineScores::PoolTake(const SpineReport &report, uint64_t nowNs){
	Age(nowNs);
	m_reportedNs = nowNs;
	m_nextDecayNs = nowNs + 2 * m_parameters.reportIntervalNs;
	const uint64_t interval = m_parameters.poolDecayIntervalNs;
	if (nowNs >= m_poolDecayEndNs){
		m_base = m_scores;
		Decay(m_base);
		std::fill(m_gradeSum.begin(), m_gradeSum.end(), 0);
		std::fill(m_gradeCount.begin(), m_gradeCount.end(), 0);
		m_poolDecayEndNs += interval *
			(1 + (nowNs - m_poolDecayEndNs) / interval);
	}
	if (!(report.edgeCongested && m_parameters.edgeFreeze)){
		for (uint32_t k = 0; k < m_scores.size(); k++){
			m_gradeSum[k] += report.Grade(k);
			m_gradeCount[k]++;
		}
	}
	for (uint32_t k = 0; k < m_scores.size(); k++){
		const uint64_t count = m_gradeCount[k];
		if (count == 0){
			m_scores[k] = m_base[k];
			continue;
		}
		const uint64_t sum = m_gradeSum[k];
		// The mean grade in 1/256, rounded half up; it rounds to grade 0
		// while 2 * sum < count.
		m_scores[k] = m_base[k] + (kGradeScale * sum + count / 2) / count;
		m_held[k] = 2 * sum < count;
	}
	Accumulate();
}

void SpineScores::Pool(SpineScores &pool, double weight){
	NS_ASSERT_MSG(&pool != this, "scores are not their own pool");
	NS_ASSERT_MSG(pool.m_scores.size() == m_scores.size(),
		"a pool scores the same spines");
	NS_ASSERT_MSG(weight >= 0 && weight <= 1, "a pool's weight is in [0, 1]");
	m_pool = &pool;
	m_poolWeight = weight;
}

bool SpineScores::IsPooled() const{
	return m_pool != nullptr && m_poolWeight > 0;
}

bool SpineScores::Propagates() const{
	return IsPooled() && m_parameters.poolHoldPropagates;
}

bool SpineScores::Holds(uint32_t spine) const{
	return m_held[spine] || m_lossHeld[spine];
}

bool SpineScores::HoldsEvery() const{
	for (uint32_t k = 0; k < m_scores.size(); k++){
		if (!Holds(k))
			return false;
	}
	return true;
}

bool SpineScores::PoolHolds(uint32_t spine) const{
	return m_pool->Holds(spine);
}

bool SpineScores::ExcludesPoolHeld() const{
	return Propagates() && !m_pool->HoldsEvery();
}

bool SpineScores::Held(uint32_t spine) const{
	if (!IsPooled())
		return m_held[spine];
	if (!m_parameters.poolHoldPropagates)
		return m_pool->m_held[spine];
	return m_held[spine] || PoolHolds(spine);
}

bool SpineScores::TakeSentinel(uint64_t nowNs, uint8_t &spine){
	if (m_parameters.heldExploration != HeldExploration::Sentinel)
		return false;
	const bool propagates = Propagates();
	if (propagates && m_pool->m_parameters.lossThresholds)
		m_pool->AdvanceLossInterval(nowNs);
	for (uint32_t k = 0; k < m_scores.size(); k++){
		// A spine the pool holds is sent the pool's sentinels, one per
		// interval for every set of scores pooled with it.
		if (propagates && PoolHolds(k)){
			if (m_pool->SentinelDue(k, nowNs)){
				spine = k;
				return true;
			}
			continue;
		}
		if (Held(k) && SentinelDue(k, nowNs)){
			spine = k;
			return true;
		}
	}
	return false;
}

bool SpineScores::TakeHoldSentinel(uint64_t nowNs, uint8_t &spine){
	if (m_parameters.lossThresholds)
		AdvanceLossInterval(nowNs);
	if (m_parameters.heldExploration != HeldExploration::Sentinel)
		return false;
	for (uint32_t k = 0; k < m_scores.size(); k++){
		if (Holds(k) && SentinelDue(k, nowNs)){
			spine = k;
			return true;
		}
	}
	return false;
}

bool SpineScores::SentinelDue(uint32_t spine, uint64_t nowNs){
	if (nowNs < m_sentinelNs[spine] + m_sentinelIntervalNs)
		return false;
	m_sentinelNs[spine] = nowNs;
	return true;
}

uint32_t SpineScores::Unheld() const{
	if (m_parameters.heldExploration == HeldExploration::Epsilon)
		return m_scores.size();
	uint32_t unheld = 0;
	for (uint32_t k = 0; k < m_scores.size(); k++)
		unheld += !Held(k);
	// While every spine is held the term stays spread over all of them.
	return unheld > 0 ? unheld : m_scores.size();
}

uint8_t SpineScores::DrawEpsilon(double fraction) const{
	const uint32_t spines = m_scores.size();
	const uint32_t unheld = Unheld();
	if (unheld == spines)
		return std::min(static_cast<uint32_t>(fraction * spines), spines - 1);
	uint32_t index = std::min(static_cast<uint32_t>(fraction * unheld),
		unheld - 1);
	for (uint32_t k = 0; k < spines; k++){
		if (!Held(k) && index-- == 0)
			return k;
	}
	NS_ASSERT_MSG(false, "the epsilon draw lands on a spine not held");
	return spines - 1;
}

double SpineScores::EpsilonShare(uint32_t spine) const{
	const uint32_t unheld = Unheld();
	if (unheld == m_scores.size())
		return m_parameters.epsilon / m_scores.size();
	return Held(spine) ? 0 : m_parameters.epsilon / unheld;
}

uint8_t SpineScores::Choose(UniformRandomVariable &random, uint64_t nowNs){
	if (nowNs >= m_nextDecayNs)
		Age(nowNs);
	if (IsPooled()){
		if (nowNs >= m_pool->m_nextDecayNs)
			m_pool->Age(nowNs);
		if (Propagates() && m_pool->m_parameters.lossThresholds)
			m_pool->AdvanceLossInterval(nowNs);
		const bool excluding = ExcludesPoolHeld();
		auto candidate = [&]() -> uint8_t {
			return m_parameters.candidateDraw == CandidateDraw::Proportional
				? DrawFromMixed(random)
				: random.GetInteger(0, m_scores.size() - 1);
		};
		uint8_t chosen = candidate();
		for (uint32_t i = 1; i < m_parameters.candidates; i++){
			const uint8_t next = candidate();
			if (Weight(next, excluding) > Weight(chosen, excluding))
				chosen = next;
		}
		return chosen;
	}
	auto candidate = [&]() -> uint8_t {
		return m_parameters.candidateDraw == CandidateDraw::Proportional
			? DrawFromShares(random)
			: random.GetInteger(0, m_scores.size() - 1);
	};
	uint8_t chosen = candidate();
	for (uint32_t i = 1; i < m_parameters.candidates; i++){
		const uint8_t next = candidate();
		if (m_scores[next] > m_scores[chosen])
			chosen = next;
	}
	return chosen;
}

double SpineScores::Share(uint32_t spine) const{
	const double spines = m_scores.size();
	if (IsPooled()){
		const bool excluding = ExcludesPoolHeld();
		if (excluding){
			if (PoolHolds(spine))
				return EpsilonShare(spine);
			const double sum = WeightSum(true);
			double part;
			if (sum > 0)
				part = Mixed(spine) / sum;
			else {
				// Spread uniformly over the spines the pool does not hold.
				uint32_t unexcluded = 0;
				for (uint32_t k = 0; k < m_scores.size(); k++)
					unexcluded += !PoolHolds(k);
				part = 1.0 / unexcluded;
			}
			return (1 - m_parameters.epsilon) * part + EpsilonShare(spine);
		}
		const double sum = MixedSum();
		if (sum == 0)
			return 1 / spines;
		return (1 - m_parameters.epsilon) * Mixed(spine) / sum +
			EpsilonShare(spine);
	}
	const uint64_t sum = m_accumulated.back();
	if (sum == 0)
		return 1 / spines;
	return (1 - m_parameters.epsilon) * m_scores[spine] / sum +
		EpsilonShare(spine);
}

uint32_t SpineScores::Score(uint32_t spine) const{
	return m_scores[spine];
}

void SpineScores::Decay(){
	Decay(m_scores);
}

void SpineScores::Decay(std::vector<uint32_t> &scores) const{
	for (uint32_t &score : scores)
		score -= (score * m_decay + 65535) >> 16;
}

void SpineScores::Age(uint64_t nowNs){
	while (nowNs >= m_nextDecayNs){
		Decay();
		Accumulate();
		m_nextDecayNs = m_accumulated.back() == 0
			? UINT64_MAX : m_nextDecayNs + m_parameters.reportIntervalNs;
	}
}

void SpineScores::Accumulate(){
	uint64_t sum = 0;
	for (uint32_t k = 0; k < m_scores.size(); k++){
		sum += m_scores[k];
		m_accumulated[k] = sum;
	}
}

uint8_t SpineScores::DrawFromShares(UniformRandomVariable &random) const{
	const uint32_t spines = m_scores.size();
	const double epsilon = m_parameters.epsilon;
	const double u = random.GetValue();
	if (u < epsilon)
		return DrawEpsilon(u / epsilon);
	const double v = (u - epsilon) / (1 - epsilon);
	const uint64_t sum = m_accumulated.back();
	if (sum == 0)
		return std::min(static_cast<uint32_t>(v * spines), spines - 1);
	const uint64_t target = std::min(static_cast<uint64_t>(v * sum), sum - 1);
	return std::upper_bound(m_accumulated.begin(), m_accumulated.end(), target) -
		m_accumulated.begin();
}

double SpineScores::Mixed(uint32_t spine) const{
	return m_poolWeight * m_pool->m_scores[spine] +
		(1 - m_poolWeight) * m_scores[spine];
}

double SpineScores::MixedSum() const{
	double sum = 0;
	for (uint32_t k = 0; k < m_scores.size(); k++)
		sum += Mixed(k);
	return sum;
}

double SpineScores::Weight(uint32_t spine, bool excluding) const{
	return excluding && PoolHolds(spine) ? 0 : Mixed(spine);
}

double SpineScores::WeightSum(bool excluding) const{
	double sum = 0;
	for (uint32_t k = 0; k < m_scores.size(); k++)
		sum += Weight(k, excluding);
	return sum;
}

uint8_t SpineScores::DrawFromMixed(UniformRandomVariable &random) const{
	const uint32_t spines = m_scores.size();
	const double epsilon = m_parameters.epsilon;
	const double u = random.GetValue();
	if (u < epsilon)
		return DrawEpsilon(u / epsilon);
	const double v = (u - epsilon) / (1 - epsilon);
	const bool excluding = ExcludesPoolHeld();
	const double sum = WeightSum(excluding);
	if (sum == 0 && !excluding)
		return std::min(static_cast<uint32_t>(v * spines), spines - 1);
	if (sum == 0){
		// Uniformly over the spines the pool does not hold.
		uint32_t unexcluded = 0;
		for (uint32_t k = 0; k < spines; k++)
			unexcluded += !PoolHolds(k);
		uint32_t index = std::min(static_cast<uint32_t>(v * unexcluded),
			unexcluded - 1);
		for (uint32_t k = 0; k < spines; k++){
			if (!PoolHolds(k) && index-- == 0)
				return k;
		}
		return spines - 1;
	}
	// The first spine whose running sum exceeds the target, as DrawFromShares
	// finds it.
	const double target = v * sum;
	double accumulated = 0;
	if (!excluding){
		for (uint32_t k = 0; k + 1 < spines; k++){
			accumulated += Mixed(k);
			if (accumulated > target)
				return k;
		}
		return spines - 1;
	}
	// Excluding, the last spine with a weight takes what rounding leaves.
	uint32_t last = spines - 1;
	for (uint32_t k = 0; k < spines; k++){
		const double weight = Weight(k, excluding);
		if (weight == 0)
			continue;
		last = k;
		accumulated += weight;
		if (accumulated > target)
			return k;
	}
	return last;
}

PolicySpineSelector::PolicySpineSelector(Ptr<UniformRandomVariable> random,
		SpineScores &scores)
	: m_random(random), m_scores(scores)
{
}

PolicySpineSelector::PolicySpineSelector(Ptr<UniformRandomVariable> random,
		std::unique_ptr<SpineScores> scores)
	: m_random(random), m_ownScores(std::move(scores)), m_scores(*m_ownScores)
{
}

uint16_t PolicySpineSelector::Choose(uint64_t nowNs){
	uint8_t spine;
	if (!m_scores.TakeSentinel(nowNs, spine))
		spine = m_scores.Choose(*m_random, nowNs);
	m_scores.OnSend(spine, nowNs);
	return SpineIdentification(spine, spine);
}

void PolicySpineSelector::OnLoss(uint16_t path, uint64_t nowNs){
	m_scores.OnLoss(path, nowNs);
}

void PolicySpineSelector::OnReport(const SpineReport &report, uint64_t nowNs){
	m_scores.OnReport(report, nowNs);
}

ObliviousSelector::ObliviousSelector(Ptr<UniformRandomVariable> random)
	: m_random(random)
{
}

uint16_t ObliviousSelector::Choose(uint64_t){
	return m_random->GetInteger(0, UINT16_MAX);
}

RepsSelector::RepsSelector(Ptr<UniformRandomVariable> random,
		uint32_t bufferSize, uint64_t freezingTimeoutNs, uint32_t explorePackets)
	: RepsSelector(random, bufferSize, freezingTimeoutNs, explorePackets,
		UINT16_MAX + 1)
{
}

RepsSelector::RepsSelector(Ptr<UniformRandomVariable> random,
		uint32_t bufferSize, uint64_t freezingTimeoutNs, uint32_t explorePackets,
		uint32_t paths)
	: m_random(random), m_paths(paths), m_buffer(bufferSize, Entry{0, false}),
	  m_head(0),
	  m_valid(0), m_written(0), m_frozenNext(0), m_frozen(false),
	  m_freezingTimeoutNs(freezingTimeoutNs), m_exitFreezingNs(0),
	  m_explorePackets(explorePackets), m_exploreCounter(0)
{
}

// Algorithm 2's onSend. The exploration that follows freezing mode draws every
// packet fresh, as the REPS artifact does: the pseudocode assigns one packet in
// eight and leaves the other seven unassigned.
uint16_t RepsSelector::Choose(uint64_t){
	if (m_exploreCounter > 0){
		m_exploreCounter--;
		return Fresh();
	}
	// Before the first acknowledgement, the connection's first window, and
	// whenever every cached value has been used outside freezing mode.
	if (m_written == 0 || (m_valid == 0 && !m_frozen))
		return Fresh();
	return Cached();
}

uint16_t RepsSelector::Fresh(){
	uint16_t path;
	do
		path = m_random->GetInteger(0, m_paths - 1);
	while (Excluded(path));
	return path;
}

bool RepsSelector::Excluded(uint16_t) const{
	return false;
}

uint16_t RepsSelector::Cached(){
	const uint32_t size = m_buffer.size();
	while (m_valid > 0){
		Entry &oldest = m_buffer[(m_head + size - m_valid) % size];
		oldest.valid = false;
		m_valid--;
		if (!Excluded(oldest.ev))
			return oldest.ev;
	}
	if (!m_frozen)
		return Fresh();
	// Frozen with nothing valid: the next held value in turn. The pseudocode
	// cycles the write position over the whole buffer; cycling over the slots
	// written so far, as the artifact does, never reads a slot that holds no
	// value.
	for (uint32_t i = 0; i < m_written; i++){
		const uint16_t ev = m_buffer[m_frozenNext].ev;
		m_frozenNext = (m_frozenNext + 1) % m_written;
		if (!Excluded(ev))
			return ev;
	}
	return Fresh();
}

// Algorithm 1's onAck.
void RepsSelector::OnAck(uint16_t path, bool marked, uint64_t nowNs){
	if (marked)
		return;
	Entry &entry = m_buffer[m_head];
	if (!entry.valid)
		m_valid++;
	entry = Entry{path, true};
	m_head = (m_head + 1) % m_buffer.size();
	if (m_written < m_buffer.size())
		m_written++;
	if (m_frozen && nowNs > m_exitFreezingNs){
		m_frozen = false;
		m_exploreCounter = m_explorePackets;
	}
}

// Algorithm 1's onFailureDetection.
void RepsSelector::OnTimeout(uint64_t nowNs){
	if (m_frozen || m_exploreCounter != 0)
		return;
	m_frozen = true;
	m_exitFreezingNs = nowNs + m_freezingTimeoutNs;
}

RepsHoldSelector::RepsHoldSelector(Ptr<UniformRandomVariable> random,
		uint32_t bufferSize, uint64_t freezingTimeoutNs, uint32_t explorePackets,
		uint32_t spines, SpineScores &hold)
	: RepsSelector(random, bufferSize, freezingTimeoutNs, explorePackets,
		spines),
	  m_hold(hold), m_excluding(false)
{
}

uint16_t RepsHoldSelector::Choose(uint64_t nowNs){
	uint8_t spine;
	if (!m_hold.TakeHoldSentinel(nowNs, spine)){
		m_excluding = !m_hold.HoldsEvery();
		spine = RepsSelector::Choose(nowNs);
	}
	m_hold.CountIntervalSend(spine, nowNs);
	return SpineIdentification(spine, spine);
}

void RepsHoldSelector::OnLoss(uint16_t path, uint64_t nowNs){
	m_hold.CountIntervalLoss(path, nowNs);
}

bool RepsHoldSelector::Excluded(uint16_t path) const{
	return m_excluding && m_hold.Holds(path);
}

EntropyRotation::EntropyRotation(Ptr<UniformRandomVariable> random,
		uint32_t size)
	: m_random(random), m_order(size), m_next(0)
{
	NS_ASSERT_MSG(size > 0 && size <= UINT16_MAX + 1,
		"an entropy value is 16 bits");
	for (uint32_t ev = 0; ev < size; ev++)
		m_order[ev] = ev;
	Shuffle();
}

uint16_t EntropyRotation::Next(){
	if (m_next == m_order.size()){
		Shuffle();
		m_next = 0;
	}
	return m_order[m_next++];
}

uint32_t EntropyRotation::Size() const{
	return m_order.size();
}

// Fisher-Yates over the whole set.
void EntropyRotation::Shuffle(){
	for (uint32_t i = m_order.size() - 1; i > 0; i--)
		std::swap(m_order[i], m_order[m_random->GetInteger(0, i)]);
}

UeObliviousSelector::UeObliviousSelector(Ptr<UniformRandomVariable> random,
		uint32_t size)
	: m_rotation(random, size)
{
}

uint16_t UeObliviousSelector::Choose(uint64_t){
	return m_rotation.Next();
}

UeAwareSelector::UeAwareSelector(Ptr<UniformRandomVariable> random,
		uint32_t size, double saturationFraction)
	: m_rotation(random, size), m_marked(size, false), m_markedCount(0),
	  m_saturationFraction(saturationFraction)
{
}

uint16_t UeAwareSelector::Choose(uint64_t){
	for (;;){
		const uint16_t ev = m_rotation.Next();
		if (!m_marked[ev])
			return ev;
		const bool saturated =
			m_markedCount > m_saturationFraction * m_rotation.Size();
		m_marked[ev] = false;
		m_markedCount--;
		if (saturated)
			return ev;
	}
}

void UeAwareSelector::OnAck(uint16_t path, bool marked, uint64_t){
	if (marked)
		Mark(path);
}

void UeAwareSelector::OnTrim(uint16_t path, bool lastHop, bool marked,
		uint64_t){
	if (!lastHop || marked)
		Mark(path);
}

void UeAwareSelector::Mark(uint16_t ev){
	NS_ASSERT_MSG(ev < m_marked.size(), "every send takes a value of the space");
	if (m_marked[ev])
		return;
	m_marked[ev] = true;
	m_markedCount++;
}

MrcSelector::MrcSelector(Ptr<UniformRandomVariable> random, uint32_t size,
		uint64_t skipNs, uint64_t probeIntervalNs)
	: m_rotation(random, size), m_entropies(size, Entropy{State::Good, 0, 0}),
	  m_skipNs(skipNs), m_probeIntervalNs(probeIntervalNs)
{
}

uint16_t MrcSelector::Choose(uint64_t nowNs){
	bool reset = false;
	bool assumedBadSeen = false;
	uint16_t assumedBad = 0;
	// Two passes reach every value, whatever order the second is shuffled
	// into, so the value this send resets is reached again.
	for (uint32_t visited = 0; visited < 2 * m_rotation.Size(); visited++){
		const uint16_t ev = m_rotation.Next();
		Entropy &entropy = m_entropies[ev];
		if (entropy.state == State::Skip && nowNs >= entropy.skipUntilNs)
			entropy.state = State::Good;
		if (entropy.state == State::Good)
			return ev;
		if (entropy.state == State::Skip && !reset){
			entropy.state = State::Good;
			reset = true;
		}else if (entropy.state == State::AssumedBad && !assumedBadSeen){
			assumedBad = ev;
			assumedBadSeen = true;
		}
	}
	// Every value is assumed bad. Sending on one beats not sending; only a
	// probe's answer brings it back, so its data's answers move nothing.
	return assumedBad;
}

void MrcSelector::OnAck(uint16_t path, bool marked, uint64_t nowNs){
	if (marked)
		Skip(path, nowNs);
}

void MrcSelector::OnTrim(uint16_t path, bool lastHop, bool, uint64_t nowNs){
	if (!lastHop)
		Skip(path, nowNs);
}

void MrcSelector::OnLoss(uint16_t path, uint64_t nowNs){
	Entropy &entropy = At(path);
	if (entropy.state == State::AssumedBad)
		return;
	entropy.state = State::AssumedBad;
	entropy.probeDueNs = nowNs + m_probeIntervalNs;
	m_probes.emplace_back(path, entropy.probeDueNs);
}

bool MrcSelector::TakeProbe(uint64_t nowNs, uint16_t &path){
	while (!m_probes.empty() && m_probes.front().second <= nowNs){
		const auto [ev, due] = m_probes.front();
		m_probes.pop_front();
		Entropy &entropy = m_entropies[ev];
		if (entropy.state != State::AssumedBad || entropy.probeDueNs != due)
			continue;
		entropy.probeDueNs = nowNs + m_probeIntervalNs;
		m_probes.emplace_back(ev, entropy.probeDueNs);
		path = ev;
		return true;
	}
	return false;
}

void MrcSelector::OnProbeAnswer(uint16_t path, bool marked, uint64_t nowNs){
	Entropy &entropy = At(path);
	if (entropy.state != State::AssumedBad)
		return;
	entropy.state = State::Good;
	if (marked)
		Skip(path, nowNs);
}

MrcSelector::Entropy &MrcSelector::At(uint16_t ev){
	NS_ASSERT_MSG(ev < m_entropies.size(), "every send takes a value of the set");
	return m_entropies[ev];
}

// A value assumed bad stays out of service until a probe answers.
void MrcSelector::Skip(uint16_t ev, uint64_t nowNs){
	Entropy &entropy = At(ev);
	if (entropy.state == State::AssumedBad)
		return;
	entropy.state = State::Skip;
	entropy.skipUntilNs = nowNs + m_skipNs;
}

} /* namespace ns3 */
