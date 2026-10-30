#ifndef ANSWER_PATH_SEQ_H
#define ANSWER_PATH_SEQ_H

#include <cstdint>
#include <functional>
#include <map>
#include <unordered_map>
#include <vector>

#include "ns3/spine-bitmap.h"

namespace ns3 {

// Answer path sequences (RdmaHw SprayAnswerPathSeq), under AckRoute Spray.
// A receiver's answers to one queue pair (acknowledgements, NACKs and repair
// requests) take the route spines it draws, and each route is one sequence of
// queues, so the answers over one route arrive in the order they were sent.
// The receiver numbers them per route, and each answer also announces how
// many answers another route has carried, so that a route on which nothing
// arrives still shows its answers missing.

// The announcement route of an answer that announces none; in a field of one
// byte (SpineFieldBytes), 255.
constexpr uint16_t kNoAnswerRoute = 0xffff;

// The receiving side, per receive queue pair: the count of the answers sent
// over each route spine.
class AnswerSequencer {
public:
	struct Number {
		// The answer's number on its route: the answers sent over it before.
		uint16_t seq;
		// Another route the queue pair has answered over and its count of
		// answers so far, round robin over those routes; kNoAnswerRoute for
		// none.
		uint16_t announceRoute;
		uint16_t announceSeq;
	};
	// The number of the next answer routed over route, after which route's
	// count moves on, wrapping at 2^16.
	Number Next(uint16_t route);

private:
	std::vector<uint16_t> m_sent;
	uint16_t m_announceNext = 0;
};

// The sending side, per queue pair: the next number expected on each route
// and the answers another route announced that have not arrived.
class AnswerSequenceTracker {
public:
	struct Outcome {
		uint16_t route;
		// The answers over route found lost.
		uint16_t lost;
		// Whether this outcome is an answer received over route, after its
		// lost ones.
		bool received;
	};
	// The answer numbered seq over route, announcing announceSeq answers over
	// announceRoute, at nowNs. Appends to outcomes, in this order: the
	// answers announced on any route, a guard of guardNs or more ago, that
	// have neither arrived nor been passed since, lost; the gap of route's
	// numbers before seq, lost, and the answer itself, received, unless it
	// is behind the next number expected (an answer already counted lost by
	// an announcement, which changes nothing). An announcement of answers
	// not yet received is held for guardNs from the first that showed them,
	// as answers over another route can still be on their way.
	void OnAnswer(uint16_t route, uint16_t seq, uint16_t announceRoute,
		uint16_t announceSeq, uint64_t nowNs, uint64_t guardNs,
		std::vector<Outcome> &outcomes);

private:
	struct Route {
		uint16_t next = 0;
		bool pending = false;
		uint16_t announced = 0;
		uint64_t dueNs = 0;
	};
	Route &At(uint16_t route);
	std::vector<Route> m_routes;
};

// The sending host's hold of the route spines toward one receiver leaf, from
// the answers of every queue pair to that leaf: per loss interval the answers
// received over each route and those found lost. At the end of an interval a
// route not held whose lost fraction lost / (lost + received) is at least
// threshold, with a loss, is held; a held route is released once clean
// answers in a row arrive over it, the receivers' probe answers, a loss
// restarting the count, and the route's counts in the interval restart at
// its release. This is the count rule and the clean-sentinel release
// of the loss hold, on answers.
class AnswerRouteHold {
public:
	using ChangeCallback =
		std::function<void(uint32_t route, bool held, uint64_t nowNs)>;
	AnswerRouteHold(uint32_t routes, uint64_t intervalNs, double threshold,
		uint32_t clean);
	void SetChangeCallback(ChangeCallback callback);
	void OnReceived(uint32_t route, uint64_t nowNs);
	void OnLost(uint32_t route, uint32_t lost, uint64_t nowNs);
	// The routes held, bit k for route k, once the intervals ended by now
	// are closed.
	SpineBitmap Bitmap(uint64_t nowNs);
	bool Held(uint32_t route) const;

private:
	void Advance(uint64_t nowNs);
	void Close(uint64_t endNs);

	uint64_t m_intervalNs;
	double m_threshold;
	uint32_t m_clean;
	uint64_t m_endNs;
	std::vector<uint32_t> m_received;
	std::vector<uint32_t> m_lost;
	std::vector<bool> m_held;
	std::vector<uint32_t> m_cleanCount;
	ChangeCallback m_onChange;
};

// The receiving host's view of the senders' route holds: per sender leaf, each
// sender's latest bitmap (its data packets carry it), and the spines any of
// them holds, which the host leaves out of its draw of the routes of its
// answers to that leaf. A spine held toward a leaf is still sent a probe
// answer once per probe interval, so that its senders can see it clean
// again; it is let go once no sender of the leaf sets its bit.
class AnswerHoldReports {
public:
	using ChangeCallback = std::function<void(uint32_t leaf, uint32_t spine,
		bool held, uint64_t nowNs)>;
	explicit AnswerHoldReports(uint32_t spines);
	void SetChangeCallback(ChangeCallback callback);
	void Report(uint32_t senderIp, uint32_t leaf, const SpineBitmap &bitmap,
		uint64_t nowNs);
	SpineBitmap Held(uint32_t leaf) const;
	// Whether a probe answer toward leaf is due over a spine held toward it,
	// intervalNs or more since its hold began or its last probe, and which;
	// the lowest such spine is taken and its interval restarts now.
	bool TakeProbe(uint32_t leaf, uint64_t nowNs, uint64_t intervalNs,
		uint8_t &spine);

private:
	struct Leaf {
		std::map<uint32_t, SpineBitmap> senders;
		SpineBitmap held;
		std::vector<uint64_t> lastProbeNs;
	};
	uint32_t m_spines;
	std::unordered_map<uint32_t, Leaf> m_leaves;
	ChangeCallback m_onChange;
};

} // namespace ns3

#endif /* ANSWER_PATH_SEQ_H */
