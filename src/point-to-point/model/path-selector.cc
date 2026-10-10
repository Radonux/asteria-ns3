#include "path-selector.h"
#include "load-balancing.h"
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

UniformSpineSelector::UniformSpineSelector(Ptr<UniformRandomVariable> random,
		uint32_t spines)
	: m_random(random), m_spines(spines)
{
}

uint16_t UniformSpineSelector::Choose(uint64_t){
	const uint8_t spine = m_random->GetInteger(0, m_spines - 1);
	return SpineIdentification(spine, spine);
}

SpineScores::SpineScores(uint32_t spines, const Parameters &parameters)
	: m_parameters(parameters),
	  m_decay(std::llround(parameters.gamma * 65536)),
	  m_accumulated(spines), m_reported(false), m_sequence(0),
	  m_reportedNs(0), m_nextDecayNs(UINT64_MAX)
{
	NS_ASSERT_MSG(spines > 0 && spines <= SpineReport::kMaxSpines,
		"a report grades at most SpineReport::kMaxSpines spines");
	NS_ASSERT_MSG(m_decay > 0 && m_decay <= 65536, "gamma is in (0, 1]");
	NS_ASSERT_MSG(parameters.candidates > 0, "a spine is drawn from candidates");
	// The largest score a decay takes exactly the top grade from.
	m_scores.assign(spines,
		SpineReport::kTopGrade * kGradeScale * uint64_t{65536} / m_decay);
	Accumulate();
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
	Age(nowNs);
	m_reported = true;
	m_sequence = report.sequence;
	m_reportedNs = nowNs;
	// A report is due every interval, and the acknowledgements carrying one
	// arrive with an interval of jitter.
	m_nextDecayNs = nowNs + 2 * m_parameters.reportIntervalNs;
	if (report.edgeCongested)
		return;
	Decay();
	for (uint32_t k = 0; k < m_scores.size(); k++)
		m_scores[k] += report.Grade(k) * kGradeScale;
	Accumulate();
}

uint8_t SpineScores::Choose(UniformRandomVariable &random, uint64_t nowNs){
	if (nowNs >= m_nextDecayNs)
		Age(nowNs);
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
	const uint64_t sum = m_accumulated.back();
	if (sum == 0)
		return 1 / spines;
	return (1 - m_parameters.epsilon) * m_scores[spine] / sum +
		m_parameters.epsilon / spines;
}

uint32_t SpineScores::Score(uint32_t spine) const{
	return m_scores[spine];
}

void SpineScores::Decay(){
	for (uint32_t &score : m_scores)
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
		return std::min(static_cast<uint32_t>(u / epsilon * spines), spines - 1);
	const double v = (u - epsilon) / (1 - epsilon);
	const uint64_t sum = m_accumulated.back();
	if (sum == 0)
		return std::min(static_cast<uint32_t>(v * spines), spines - 1);
	const uint64_t target = std::min(static_cast<uint64_t>(v * sum), sum - 1);
	return std::upper_bound(m_accumulated.begin(), m_accumulated.end(), target) -
		m_accumulated.begin();
}

PolicySpineSelector::PolicySpineSelector(Ptr<UniformRandomVariable> random,
		SpineScores &scores)
	: m_random(random), m_scores(scores)
{
}

uint16_t PolicySpineSelector::Choose(uint64_t nowNs){
	const uint8_t spine = m_scores.Choose(*m_random, nowNs);
	return SpineIdentification(spine, spine);
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
	: m_random(random), m_buffer(bufferSize, Entry{0, false}), m_head(0),
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
	return m_random->GetInteger(0, UINT16_MAX);
}

uint16_t RepsSelector::Cached(){
	if (m_valid > 0){
		const uint32_t size = m_buffer.size();
		Entry &oldest = m_buffer[(m_head + size - m_valid) % size];
		oldest.valid = false;
		m_valid--;
		return oldest.ev;
	}
	// Frozen with nothing valid: the next held value in turn. The pseudocode
	// cycles the write position over the whole buffer; cycling over the slots
	// written so far, as the artifact does, never reads a slot that holds no
	// value.
	const uint16_t ev = m_buffer[m_frozenNext].ev;
	m_frozenNext = (m_frozenNext + 1) % m_written;
	return ev;
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
