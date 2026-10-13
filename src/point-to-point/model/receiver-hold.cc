#include "receiver-hold.h"
#include "ns3/assert.h"
#include <algorithm>

namespace ns3 {

ReceiverHold::Leaf::Leaf(uint32_t spines)
	: arrivals(spines), lost(spines), held(spines), clean(spines)
{
}

ReceiverHold::ReceiverHold(uint32_t spines, uint32_t autoLeaves,
		uint32_t cleanArrivals, uint64_t intervalNs)
	: m_spines(spines), m_autoLeaves(autoLeaves),
	  m_cleanArrivals(cleanArrivals), m_intervalNs(intervalNs),
	  m_intervalEndNs(intervalNs), m_lossLeaves(spines), m_hostHeld(spines),
	  m_hostClean(spines)
{
	NS_ASSERT_MSG(spines > 0 && spines <= kReceiverHoldSpines,
		"a receiver's hold names at most kReceiverHoldSpines spines");
	NS_ASSERT_MSG(autoLeaves >= 2, "host scope takes at least two leaves");
	NS_ASSERT_MSG(cleanArrivals > 0, "a receiver's hold ends by clean arrivals");
}

ReceiverHold::Leaf &ReceiverHold::LeafCounts(uint32_t leaf){
	return m_leaves.try_emplace(leaf, m_spines).first->second;
}

void ReceiverHold::OnArrival(uint32_t leaf, uint32_t spine, uint32_t gaps,
		bool late, uint64_t nowNs){
	NS_ASSERT_MSG(spine < m_spines, "an arrival is on a held spine's range");
	AdvanceInterval(nowNs);
	Leaf &counts = LeafCounts(leaf);
	counts.arrivals[spine]++;
	if (gaps > 0){
		CountMissing(counts, leaf, spine, gaps);
		return;
	}
	if (late)
		return;
	if (m_hostHeld[spine]){
		if (++m_hostClean[spine] >= m_cleanArrivals){
			m_hostHeld[spine] = false;
			m_hostClean[spine] = 0;
			m_lossLeaves[spine].clear();
		}
	} else if (counts.held[spine] && ++counts.clean[spine] >= m_cleanArrivals){
		counts.held[spine] = false;
		counts.clean[spine] = 0;
		m_lossLeaves[spine].erase(leaf);
	}
}

void ReceiverHold::OnTrim(uint32_t leaf, uint32_t spine, uint64_t nowNs){
	NS_ASSERT_MSG(spine < m_spines, "a trim is on a held spine's range");
	AdvanceInterval(nowNs);
	LeafCounts(leaf);
}

void ReceiverHold::CountMissing(Leaf &counts, uint32_t leaf, uint32_t spine,
		uint32_t gaps){
	counts.lost[spine] += gaps;
	m_lossLeaves[spine].insert(leaf);
	if (m_hostHeld[spine])
		m_hostClean[spine] = 0;
	else if (m_lossLeaves[spine].size() >= m_autoLeaves)
		HoldHost(spine);
	else {
		counts.held[spine] = true;
		counts.clean[spine] = 0;
	}
}

void ReceiverHold::HoldHost(uint32_t spine){
	m_hostHeld[spine] = true;
	m_hostClean[spine] = 0;
	for (auto &[leaf, counts] : m_leaves){
		counts.held[spine] = false;
		counts.clean[spine] = 0;
	}
}

bool ReceiverHold::Holds(uint32_t leaf, uint32_t spine) const{
	if (m_hostHeld[spine])
		return true;
	const auto found = m_leaves.find(leaf);
	return found != m_leaves.end() && found->second.held[spine];
}

uint8_t ReceiverHold::Bitmap(uint32_t leaf) const{
	uint8_t bitmap = 0;
	for (uint32_t k = 0; k < m_spines; k++){
		if (Holds(leaf, k))
			bitmap |= static_cast<uint8_t>(1u << k);
	}
	return bitmap;
}

bool ReceiverHold::HostHolds(uint32_t spine) const{
	return m_hostHeld[spine];
}

uint32_t ReceiverHold::LossLeaves(uint32_t spine) const{
	return m_lossLeaves[spine].size();
}

uint32_t ReceiverHold::Clean(uint32_t leaf, uint32_t spine) const{
	if (m_hostHeld[spine])
		return m_hostClean[spine];
	const auto found = m_leaves.find(leaf);
	return found != m_leaves.end() ? found->second.clean[spine] : 0;
}

void ReceiverHold::SetIntervalCallback(IntervalCallback callback){
	m_onInterval = std::move(callback);
}

void ReceiverHold::AdvanceInterval(uint64_t nowNs){
	if (m_intervalNs == 0 || nowNs < m_intervalEndNs)
		return;
	// The counts are those of the interval that ended at m_intervalEndNs;
	// any later interval that ended by now counted nothing.
	for (auto &[leaf, counts] : m_leaves){
		if (m_onInterval){
			std::vector<Scope> holds(m_spines, Scope::None);
			for (uint32_t k = 0; k < m_spines; k++)
				holds[k] = m_hostHeld[k] ? Scope::Host
					: counts.held[k] ? Scope::Leaf : Scope::None;
			m_onInterval(leaf, m_intervalEndNs, counts.arrivals, counts.lost,
				holds);
		}
		std::fill(counts.arrivals.begin(), counts.arrivals.end(), 0);
		std::fill(counts.lost.begin(), counts.lost.end(), 0);
	}
	m_intervalEndNs += m_intervalNs *
		(1 + (nowNs - m_intervalEndNs) / m_intervalNs);
}

ReceiverHoldMarks::Leaf::Leaf(uint32_t spines)
	: receivers(spines), sentinelNs(spines)
{
}

ReceiverHoldMarks::ReceiverHoldMarks(uint32_t spines,
		uint64_t sentinelIntervalNs)
	: m_spines(spines), m_sentinelIntervalNs(sentinelIntervalNs)
{
	NS_ASSERT_MSG(spines > 0 && spines <= kReceiverHoldSpines,
		"a receiver's hold names at most kReceiverHoldSpines spines");
	NS_ASSERT_MSG(sentinelIntervalNs > 0,
		"a held spine's sentinels are an interval apart");
}

void ReceiverHoldMarks::OnBitmap(uint32_t leaf, uint32_t receiver,
		uint8_t bitmap){
	Leaf *marks = nullptr;
	const auto found = m_leaves.find(leaf);
	if (found != m_leaves.end())
		marks = &found->second;
	else if (bitmap != 0)
		marks = &m_leaves.try_emplace(leaf, m_spines).first->second;
	if (marks == nullptr)
		return;
	for (uint32_t k = 0; k < m_spines; k++){
		if ((bitmap >> k) & 1)
			marks->receivers[k].insert(receiver);
		else
			marks->receivers[k].erase(receiver);
	}
}

bool ReceiverHoldMarks::Holds(uint32_t leaf, uint32_t spine) const{
	const auto found = m_leaves.find(leaf);
	return found != m_leaves.end() && !found->second.receivers[spine].empty();
}

bool ReceiverHoldMarks::TakeSentinel(uint32_t leaf, uint64_t nowNs,
		const std::function<bool(uint32_t)> &localHolds, uint8_t &spine){
	const auto found = m_leaves.find(leaf);
	if (found == m_leaves.end())
		return false;
	Leaf &marks = found->second;
	for (uint32_t k = 0; k < m_spines; k++){
		if (marks.receivers[k].empty() || localHolds(k) ||
				nowNs < marks.sentinelNs[k] + m_sentinelIntervalNs)
			continue;
		marks.sentinelNs[k] = nowNs;
		spine = k;
		return true;
	}
	return false;
}

uint32_t ReceiverHoldMarks::Spines() const{
	return m_spines;
}

void ReceiverHoldView::Set(std::shared_ptr<ReceiverHoldMarks> marks,
		uint32_t leaf){
	m_marks = std::move(marks);
	m_leaf = leaf;
}

void ReceiverHoldView::OnBitmap(uint32_t receiver, uint8_t bitmap){
	if (m_marks)
		m_marks->OnBitmap(m_leaf, receiver, bitmap);
}

bool ReceiverHoldView::Holds(uint32_t spine) const{
	return m_marks && m_marks->Holds(m_leaf, spine);
}

bool ReceiverHoldView::Active() const{
	return m_marks != nullptr;
}

bool ReceiverHoldView::HoldsEvery(
		const std::function<bool(uint32_t)> &localHolds) const{
	const uint32_t spines = m_marks ? m_marks->Spines() : 0;
	for (uint32_t k = 0; k < spines; k++){
		if (!localHolds(k) && !Holds(k))
			return false;
	}
	return true;
}

bool ReceiverHoldView::TakeSentinel(uint64_t nowNs,
		const std::function<bool(uint32_t)> &localHolds, uint8_t &spine){
	return m_marks && m_marks->TakeSentinel(m_leaf, nowNs, localHolds, spine);
}

} // namespace ns3
