#ifndef LINK_EVENTS_H
#define LINK_EVENTS_H

#include <ns3/data-rate.h>
#include <ns3/ptr.h>
#include <functional>
#include <map>
#include <stdint.h>
#include <string>
#include <utility>
#include <vector>

namespace ns3 {

class QbbNetDevice;
class RateErrorModel;
class SwitchNode;

// What befalls the link between nodes a and b at a time in the run. Down takes
// the link down. Blackhole leaves it up while b, a switch, drops what arrives
// on its port from a. Loss and Rate set the link's error rate or its speed in
// both directions. Restore returns the link to the state it had before its
// latest failure, which must not be a Down.
enum class LinkEventKind { Down, Blackhole, Loss, Rate, Restore };

struct LinkEvent {
	uint64_t timeNs = 0;
	LinkEventKind kind = LinkEventKind::Down;
	uint32_t a = 0, b = 0;
	// Loss's error rate, in (0, 1].
	double errorRate = 0;
	// Rate's data rate.
	DataRate rate;
};

// Sort events into the order they apply in: by time, a restore before a
// failure at the same time, and otherwise as given.
void OrderLinkEvents(std::vector<LinkEvent> &events);

// For events in the order they apply in, why one of them cannot apply: a
// restore of a link no failure came before, or of a link taken down. Empty
// when every one can.
std::string CheckLinkEvents(const std::vector<LinkEvent> &events);

// A link's devices at a and at b, and b when b is a switch, else null.
struct LinkEnds {
	Ptr<QbbNetDevice> atA, atB;
	Ptr<SwitchNode> switchAtB;
};

// Applies link events and keeps, for each link, the state its latest failure
// replaced, which a restore puts back: both ends' data rates under Rate, both
// ends' error models and rates under Loss (a restore removes a model the
// failure added), and b's port's black hole flag under Blackhole. A link is
// named by its two nodes in either order.
class LinkEventApplier {
public:
	// setErrorRate sets a device's link error rate, adding its error model
	// when it has none; takeDown takes the link between two nodes down.
	using SetErrorRate = std::function<void(Ptr<QbbNetDevice>, double)>;
	using TakeDown = std::function<void(uint32_t, uint32_t)>;
	LinkEventApplier(SetErrorRate setErrorRate, TakeDown takeDown);
	void Apply(const LinkEvent &event, const LinkEnds &ends);

private:
	struct EndState {
		DataRate rate;
		// Null when the device had no error model.
		Ptr<RateErrorModel> errorModel;
		double errorRate = 0;
	};
	struct Saved {
		LinkEventKind kind;
		LinkEnds ends;
		EndState atA, atB;
		bool blackholed = false;
	};
	static std::pair<uint32_t, uint32_t> Key(uint32_t a, uint32_t b);
	static EndState Capture(Ptr<QbbNetDevice> device);
	static void PutBack(Ptr<QbbNetDevice> device, const EndState &state,
		LinkEventKind kind);
	void Restore(const LinkEvent &event);

	SetErrorRate m_setErrorRate;
	TakeDown m_takeDown;
	std::map<std::pair<uint32_t, uint32_t>, Saved> m_saved;
};

} /* namespace ns3 */

#endif /* LINK_EVENTS_H */
