#include "spine-sequence.h"
#include "load-balancing.h"

namespace ns3 {

uint16_t SpineSequenceSender::Next(uint8_t spine){
	if (spine >= m_next.size())
		m_next.resize(spine + 1, 0);
	return m_next[spine]++;
}

uint16_t SpineSequenceSender::Count(uint8_t spine) const{
	return spine < m_next.size() ? m_next[spine] : 0;
}

namespace {
// Serial number arithmetic on 16 bits: whether a is ahead of b.
bool Ahead(uint16_t a, uint16_t b){
	const uint16_t distance = static_cast<uint16_t>(a - b);
	return distance != 0 && distance < 0x8000;
}
} // namespace

void SpineSequenceReceiver::Prune(Path &path){
	for (auto it = path.accounted.begin(); it != path.accounted.end();){
		if (static_cast<uint16_t>(*it - path.next) >= kAccountedWindow)
			it = path.accounted.erase(it);
		else
			++it;
	}
}

void SpineSequenceReceiver::Account(Path &path, uint16_t pathSeq){
	if (!path.started){
		// The path's start is not known yet; its first data arrival drops
		// those behind it.
		if (path.accounted.size() < kAccountedWindow)
			path.accounted.insert(pathSeq);
		return;
	}
	if (static_cast<uint16_t>(pathSeq - path.next) < kAccountedWindow)
		path.accounted.insert(pathSeq);
}

bool SpineSequenceReceiver::Furthest(uint8_t requested, uint16_t &next) const{
	bool any = false;
	for (const auto &path : m_paths){
		if (RequestedSpine(path.first) != requested || !path.second.started)
			continue;
		if (!any || Ahead(path.second.next, next))
			next = path.second.next;
		any = true;
	}
	return any;
}

std::vector<SpineSequenceReceiver::Missing> SpineSequenceReceiver::TakeMissing(
		uint8_t requested, uint16_t owed){
	// Nothing arrived up requested: the sender's counter starts at 0.
	uint16_t from = 0;
	const bool any = Furthest(requested, from);
	const auto taken = m_taken.find(requested);
	if (taken != m_taken.end() && (!any || Ahead(taken->second, from)))
		from = taken->second;
	std::vector<Missing> runs;
	if (!Ahead(owed, from))
		return runs;
	m_taken[requested] = owed;
	const uint16_t span = static_cast<uint16_t>(owed - from);
	for (uint16_t k = 0; k < span; k++){
		const uint16_t seq = static_cast<uint16_t>(from + k);
		if (Accounted(requested, seq))
			continue;
		if (!runs.empty() && static_cast<uint16_t>(runs.back().first +
				runs.back().count) == seq)
			runs.back().count++;
		else
			runs.push_back(Missing{seq, 1});
	}
	return runs;
}

uint16_t SpineSequenceReceiver::TakeTailMissing(uint8_t requested,
		uint16_t first, uint16_t count){
	// Nothing arrived up requested on its own spine, as on a black-holed
	// one: its path expects 0, as OnArrival would make it.
	m_paths.try_emplace(SpineIdentification(requested, requested),
		Path{0, true, {}});
	uint16_t passed = 0;
	Furthest(requested, passed);
	uint16_t counted = 0;
	for (uint16_t k = 0; k < count; k++){
		const uint16_t seq = static_cast<uint16_t>(first + k);
		if (Ahead(passed, seq))
			continue;
		for (auto &path : m_paths)
			if (RequestedSpine(path.first) == requested)
				Account(path.second, seq);
		counted++;
	}
	return counted;
}

bool SpineSequenceReceiver::Accounted(uint8_t requested, uint16_t pathSeq) const{
	for (const auto &path : m_paths)
		if (RequestedSpine(path.first) == requested &&
				path.second.accounted.count(pathSeq) > 0)
			return true;
	return false;
}

SpineSequenceReceiver::Arrival SpineSequenceReceiver::OnArrival(
		uint8_t requested, uint8_t carrying, uint16_t pathSeq){
	Path &path = m_paths.try_emplace(SpineIdentification(requested, carrying),
		Path{0, requested == carrying, {}}).first->second;
	// Counted missing by TakeTailMissing, the only accounted sequence a data
	// packet carries.
	if (path.accounted.count(pathSeq) > 0)
		return Arrival{0, true, true};
	if (!path.started){
		path.started = true;
		path.next = static_cast<uint16_t>(pathSeq + 1);
		Prune(path);
		return Arrival{0, false, false};
	}
	// Serial number arithmetic on 16 bits: a distance below 2^15 is ahead.
	const uint16_t ahead = static_cast<uint16_t>(pathSeq - path.next);
	if (ahead >= 0x8000)
		return Arrival{0, true, false};
	// The accounted sequences in [next, pathSeq) are not missing.
	uint16_t accounted = 0;
	for (const uint16_t seq : path.accounted)
		if (static_cast<uint16_t>(seq - path.next) < ahead)
			accounted++;
	path.next = static_cast<uint16_t>(pathSeq + 1);
	Prune(path);
	return Arrival{static_cast<uint16_t>(ahead - accounted), false, false};
}

bool SpineSequenceReceiver::OnTrim(uint8_t requested, uint8_t carrying,
		uint16_t pathSeq){
	Path &path = m_paths.try_emplace(SpineIdentification(requested, carrying),
		Path{0, requested == carrying, {}}).first->second;
	const bool counted = path.accounted.count(pathSeq) > 0;
	Account(path, pathSeq);
	return counted;
}

void PendingTailRuns::Add(uint8_t spine, uint16_t first, uint16_t count,
		uint64_t dueNs){
	if (count > 0)
		m_runs.push_back(Run{spine, first, dueNs,
			std::vector<bool>(count, true)});
}

void PendingTailRuns::OnArrival(uint8_t spine, uint16_t pathSeq){
	for (Run &run : m_runs){
		const uint16_t offset = static_cast<uint16_t>(pathSeq - run.first);
		if (run.spine == spine && offset < run.missing.size())
			run.missing[offset] = false;
	}
}

std::vector<PendingTailRuns::Request> PendingTailRuns::TakeDue(uint64_t nowNs){
	std::vector<Request> requests;
	std::vector<Run> kept;
	for (Run &run : m_runs){
		if (run.dueNs > nowNs){
			kept.push_back(std::move(run));
			continue;
		}
		for (uint32_t k = 0; k < run.missing.size(); k++){
			if (!run.missing[k])
				continue;
			if (k > 0 && run.missing[k - 1])
				requests.back().count++;
			else
				requests.push_back(Request{run.spine,
					static_cast<uint16_t>(run.first + k), 1});
		}
	}
	m_runs.swap(kept);
	return requests;
}

uint64_t PendingTailRuns::NextDue() const{
	uint64_t due = kNone;
	for (const Run &run : m_runs)
		if (run.dueNs < due)
			due = run.dueNs;
	return due;
}

bool PendingTailRuns::Empty() const{
	return m_runs.empty();
}

} // namespace ns3
