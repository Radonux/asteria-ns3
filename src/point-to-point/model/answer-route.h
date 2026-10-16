#ifndef ANSWER_ROUTE_H
#define ANSWER_ROUTE_H

#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

namespace ns3 {

// The route spines of one connection's answers, a sending host and priority
// group's, at its receiving host, under RdmaHw AckRoute ReverseSet or Repath.
// A Draw returns a value uniformly in [0, n) from the receiver's stream.
using AnswerRouteDraw = std::function<uint32_t(uint32_t n)>;

// AckRoute ReverseSet: as MRC keeps a set of entropy values for a queue
// pair's data, the receiver keeps a set of route spines for a connection's
// answers, drawn once, and routes each answer over a member drawn uniformly.
// While the connection's data arrives, a round an interval apart probes every
// member; the sender returns each probe over the spine it probed, and a member
// whose oldest unanswered probe was sent a timeout or more before a round is
// replaced at that round by a spine outside the set.
class AnswerRouteSet {
public:
	// The member an answer takes, the set drawn at the first call: size
	// distinct spines of spines, at most all of them.
	uint8_t Route(uint32_t spines, uint32_t size, const AnswerRouteDraw &draw);
	// At a data arrival at nowNs, a round when none was taken yet or
	// intervalNs or more have passed since the last: each member whose oldest
	// unanswered probe was sent timeoutNs or more ago is replaced by a spine
	// drawn outside the set, appended to replaced as (old, new), and then
	// every member is probed, appended to probes. False when no round is due.
	bool TakeRound(uint64_t nowNs, uint64_t intervalNs, uint64_t timeoutNs,
		uint32_t spines, uint32_t size, const AnswerRouteDraw &draw,
		std::vector<uint8_t> &probes,
		std::vector<std::pair<uint8_t, uint8_t>> &replaced);
	// A probe returned over spine: the member's probes are answered. False
	// when spine is no member, as after its replacement.
	bool OnReply(uint8_t spine);
	const std::vector<uint8_t> &Members() const;

private:
	void DrawSet(uint32_t spines, uint32_t size, const AnswerRouteDraw &draw);

	std::vector<uint8_t> m_members;
	// Per member, when its oldest unanswered probe was sent; kNone for none.
	static constexpr uint64_t kNone = UINT64_MAX;
	std::vector<uint64_t> m_unansweredNs;
	bool m_rounds = false;
	uint64_t m_roundNs = 0;
};

// AckRoute Repath: the connection's answers move once its sender resends what
// the receiver already holds, which shows that the sender did not see them:
// one route spine per connection, drawn once, and drawn again among the other
// spines at each duplicate data arrival.
class AnswerRepath {
public:
	uint8_t Route(uint32_t spines, const AnswerRouteDraw &draw);
	// A duplicate arrived: the new route, another spine than the last.
	uint8_t Repath(uint32_t spines, const AnswerRouteDraw &draw);

private:
	// Wider than a spine's index, which spans a byte.
	static constexpr uint16_t kNone = UINT16_MAX;
	uint16_t m_route = kNone;
};

} // namespace ns3

#endif /* ANSWER_ROUTE_H */
