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

SpineSequenceReceiver::Missing SpineSequenceReceiver::TakeMissing(
		uint8_t requested, uint16_t owed){
	// Nothing arrived up requested: the sender's counter starts at 0.
	uint16_t from = 0;
	bool any = false;
	for (const auto &path : m_expected){
		if (RequestedSpine(path.first) != requested || !path.second.started)
			continue;
		if (!any || Ahead(path.second.next, from))
			from = path.second.next;
		any = true;
	}
	const auto taken = m_taken.find(requested);
	if (taken != m_taken.end() && (!any || Ahead(taken->second, from)))
		from = taken->second;
	if (!Ahead(owed, from))
		return Missing{from, 0};
	m_taken[requested] = owed;
	return Missing{from, static_cast<uint16_t>(owed - from)};
}

SpineSequenceReceiver::Arrival SpineSequenceReceiver::OnArrival(
		uint8_t requested, uint8_t carrying, uint16_t pathSeq){
	Expected &expected = m_expected.try_emplace(
		SpineIdentification(requested, carrying),
		Expected{0, requested == carrying}).first->second;
	if (!expected.started){
		expected.started = true;
		expected.next = static_cast<uint16_t>(pathSeq + 1);
		return Arrival{0, false};
	}
	// Serial number arithmetic on 16 bits: a distance below 2^15 is ahead.
	const uint16_t ahead = static_cast<uint16_t>(pathSeq - expected.next);
	if (ahead >= 0x8000)
		return Arrival{0, true};
	expected.next = static_cast<uint16_t>(pathSeq + 1);
	return Arrival{ahead, false};
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
