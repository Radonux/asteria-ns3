#include "answer-path-seq.h"

namespace ns3 {

namespace {
// Serial number arithmetic on 16 bits: whether a is ahead of b.
bool Ahead(uint16_t a, uint16_t b){
	const uint16_t distance = static_cast<uint16_t>(a - b);
	return distance != 0 && distance < 0x8000;
}
} // namespace

AnswerSequencer::Number AnswerSequencer::Next(uint16_t route){
	if (route >= m_sent.size())
		m_sent.resize(route + 1, 0);
	Number number{m_sent[route]++, kNoAnswerRoute, 0};
	const uint32_t routes = m_sent.size();
	for (uint32_t step = 0; step < routes; step++){
		const uint16_t other = static_cast<uint16_t>((m_announceNext + step) % routes);
		if (other == route || m_sent[other] == 0)
			continue;
		number.announceRoute = other;
		number.announceSeq = m_sent[other];
		m_announceNext = static_cast<uint16_t>((other + 1) % routes);
		break;
	}
	return number;
}

AnswerSequenceTracker::Route &AnswerSequenceTracker::At(uint16_t route){
	if (route >= m_routes.size())
		m_routes.resize(route + 1);
	return m_routes[route];
}

void AnswerSequenceTracker::OnAnswer(uint16_t route, uint16_t seq,
		uint16_t announceRoute, uint16_t announceSeq, uint64_t nowNs,
		uint64_t guardNs, std::vector<Outcome> &outcomes){
	At(route);
	if (announceRoute != kNoAnswerRoute)
		At(announceRoute);
	for (uint32_t k = 0; k < m_routes.size(); k++){
		Route &due = m_routes[k];
		if (!due.pending || due.dueNs > nowNs)
			continue;
		due.pending = false;
		if (Ahead(due.announced, due.next)){
			outcomes.push_back(Outcome{static_cast<uint16_t>(k),
				static_cast<uint16_t>(due.announced - due.next), false});
			due.next = due.announced;
		}
	}
	Route &own = m_routes[route];
	if (!Ahead(own.next, seq)){
		outcomes.push_back(Outcome{route,
			static_cast<uint16_t>(seq - own.next), true});
		own.next = static_cast<uint16_t>(seq + 1);
		if (own.pending && !Ahead(own.announced, own.next))
			own.pending = false;
	}
	if (announceRoute == kNoAnswerRoute || announceRoute == route)
		return;
	Route &shown = m_routes[announceRoute];
	if (!Ahead(announceSeq, shown.next))
		return;
	if (!shown.pending){
		shown.pending = true;
		shown.dueNs = nowNs + guardNs;
		shown.announced = announceSeq;
	}else if (Ahead(announceSeq, shown.announced))
		shown.announced = announceSeq;
}

AnswerRouteHold::AnswerRouteHold(uint32_t routes, uint64_t intervalNs,
		double threshold, uint32_t clean)
	: m_intervalNs(intervalNs > 0 ? intervalNs : 1),
	  m_threshold(threshold),
	  m_clean(clean),
	  m_endNs(0),
	  m_received(routes, 0),
	  m_lost(routes, 0),
	  m_held(routes, false),
	  m_cleanCount(routes, 0){
}

void AnswerRouteHold::SetChangeCallback(ChangeCallback callback){
	m_onChange = std::move(callback);
}

void AnswerRouteHold::Close(uint64_t endNs){
	for (uint32_t k = 0; k < m_held.size(); k++){
		const uint32_t lost = m_lost[k];
		if (!m_held[k] && lost > 0 &&
				lost >= m_threshold * (lost + m_received[k])){
			m_held[k] = true;
			m_cleanCount[k] = 0;
			if (m_onChange)
				m_onChange(k, true, endNs);
		}
		m_received[k] = 0;
		m_lost[k] = 0;
	}
}

void AnswerRouteHold::Advance(uint64_t nowNs){
	if (m_endNs == 0){
		// Intervals end at the multiples of the interval.
		m_endNs = (nowNs / m_intervalNs + 1) * m_intervalNs;
		return;
	}
	if (nowNs < m_endNs)
		return;
	Close(m_endNs);
	// The intervals after it, up to now, counted nothing.
	m_endNs = (nowNs / m_intervalNs + 1) * m_intervalNs;
}

void AnswerRouteHold::OnReceived(uint32_t route, uint64_t nowNs){
	Advance(nowNs);
	if (route >= m_held.size())
		return;
	m_received[route]++;
	if (m_held[route] && ++m_cleanCount[route] >= m_clean){
		m_held[route] = false;
		m_cleanCount[route] = 0;
		// The interval's counts restart, so that the losses counted while it
		// was held do not hold it again at the interval's end.
		m_received[route] = 0;
		m_lost[route] = 0;
		if (m_onChange)
			m_onChange(route, false, nowNs);
	}
}

void AnswerRouteHold::OnLost(uint32_t route, uint32_t lost, uint64_t nowNs){
	Advance(nowNs);
	if (route >= m_held.size() || lost == 0)
		return;
	m_lost[route] += lost;
	m_cleanCount[route] = 0;
}

SpineBitmap AnswerRouteHold::Bitmap(uint64_t nowNs){
	Advance(nowNs);
	SpineBitmap bitmap;
	for (uint32_t k = 0; k < m_held.size() && k < kMaxSpineCount; k++)
		if (m_held[k])
			bitmap.Set(k);
	return bitmap;
}

bool AnswerRouteHold::Held(uint32_t route) const{
	return route < m_held.size() && m_held[route];
}

AnswerHoldReports::AnswerHoldReports(uint32_t spines)
	: m_spines(spines){
}

void AnswerHoldReports::SetChangeCallback(ChangeCallback callback){
	m_onChange = std::move(callback);
}

void AnswerHoldReports::Report(uint32_t senderIp, uint32_t leaf,
		const SpineBitmap &bitmap, uint64_t nowNs){
	Leaf &state = m_leaves[leaf];
	if (state.lastProbeNs.empty())
		state.lastProbeNs.assign(m_spines, 0);
	auto sender = state.senders.find(senderIp);
	if (sender == state.senders.end()){
		if (!bitmap.Any())
			return;
		sender = state.senders.emplace(senderIp, bitmap).first;
	}else if (sender->second == bitmap)
		return;
	sender->second = bitmap;
	SpineBitmap held;
	for (const auto &reported : state.senders)
		held |= reported.second;
	const SpineBitmap changed = held ^ state.held;
	state.held = held;
	for (uint32_t k = 0; k < m_spines && k < kMaxSpineCount; k++){
		if (!changed.Test(k))
			continue;
		const bool now = held.Test(k);
		// The first probe of a hold is due an interval after it begins.
		if (now)
			state.lastProbeNs[k] = nowNs;
		if (m_onChange)
			m_onChange(leaf, k, now, nowNs);
	}
}

SpineBitmap AnswerHoldReports::Held(uint32_t leaf) const{
	const auto state = m_leaves.find(leaf);
	return state == m_leaves.end() ? SpineBitmap() : state->second.held;
}

bool AnswerHoldReports::TakeProbe(uint32_t leaf, uint64_t nowNs,
		uint64_t intervalNs, uint8_t &spine){
	const auto state = m_leaves.find(leaf);
	if (state == m_leaves.end() || !state->second.held.Any())
		return false;
	for (uint32_t k = 0; k < m_spines && k < kMaxSpineCount; k++){
		if (!state->second.held.Test(k) ||
				nowNs < state->second.lastProbeNs[k] + intervalNs)
			continue;
		state->second.lastProbeNs[k] = nowNs;
		spine = static_cast<uint8_t>(k);
		return true;
	}
	return false;
}

} // namespace ns3
