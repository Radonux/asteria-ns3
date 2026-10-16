#include "answer-route.h"
#include <algorithm>

namespace ns3 {

namespace {
// A spine of spines drawn uniformly among those not in taken.
uint8_t DrawOutside(const std::vector<uint8_t> &taken, uint32_t spines,
		const AnswerRouteDraw &draw){
	std::vector<uint8_t> free;
	for (uint32_t spine = 0; spine < spines; spine++)
		if (std::find(taken.begin(), taken.end(), spine) == taken.end())
			free.push_back(static_cast<uint8_t>(spine));
	return free[draw(free.size())];
}
} // namespace

void AnswerRouteSet::DrawSet(uint32_t spines, uint32_t size,
		const AnswerRouteDraw &draw){
	const uint32_t members = std::min(size, spines);
	while (m_members.size() < members)
		m_members.push_back(DrawOutside(m_members, spines, draw));
	m_unansweredNs.assign(m_members.size(), kNone);
}

uint8_t AnswerRouteSet::Route(uint32_t spines, uint32_t size,
		const AnswerRouteDraw &draw){
	if (m_members.empty())
		DrawSet(spines, size, draw);
	return m_members[draw(m_members.size())];
}

bool AnswerRouteSet::TakeRound(uint64_t nowNs, uint64_t intervalNs,
		uint64_t timeoutNs, uint32_t spines, uint32_t size,
		const AnswerRouteDraw &draw, std::vector<uint8_t> &probes,
		std::vector<std::pair<uint8_t, uint8_t>> &replaced){
	if (m_rounds && nowNs < m_roundNs + intervalNs)
		return false;
	if (m_members.empty())
		DrawSet(spines, size, draw);
	m_rounds = true;
	m_roundNs = nowNs;
	for (uint32_t k = 0; k < m_members.size(); k++){
		if (m_unansweredNs[k] == kNone || nowNs < m_unansweredNs[k] + timeoutNs)
			continue;
		// Only a spine outside the set: with every spine a member, the
		// member stays.
		if (m_members.size() < spines){
			const uint8_t old = m_members[k];
			m_members[k] = DrawOutside(m_members, spines, draw);
			replaced.emplace_back(old, m_members[k]);
		}
		m_unansweredNs[k] = kNone;
	}
	for (uint32_t k = 0; k < m_members.size(); k++){
		probes.push_back(m_members[k]);
		if (m_unansweredNs[k] == kNone)
			m_unansweredNs[k] = nowNs;
	}
	return true;
}

bool AnswerRouteSet::OnReply(uint8_t spine){
	const auto member = std::find(m_members.begin(), m_members.end(), spine);
	if (member == m_members.end())
		return false;
	m_unansweredNs[member - m_members.begin()] = kNone;
	return true;
}

const std::vector<uint8_t> &AnswerRouteSet::Members() const{
	return m_members;
}

uint8_t AnswerRepath::Route(uint32_t spines, const AnswerRouteDraw &draw){
	if (m_route == kNone)
		m_route = static_cast<uint8_t>(draw(spines));
	return m_route;
}

uint8_t AnswerRepath::Repath(uint32_t spines, const AnswerRouteDraw &draw){
	if (m_route == kNone || spines < 2)
		return Route(spines, draw);
	m_route = DrawOutside({m_route}, spines, draw);
	return m_route;
}

} // namespace ns3
