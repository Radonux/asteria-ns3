#ifndef LINK_EVENTS_H
#define LINK_EVENTS_H

#include <ns3/data-rate.h>
#include <ns3/error-model.h>
#include <ns3/ptr.h>
#include <functional>
#include <map>
#include <stdint.h>
#include <string>
#include <utility>
#include <vector>

namespace ns3 {

class QbbNetDevice;
class SwitchNode;

// What befalls the link between nodes a and b at a time in the run. Down takes
// the link down. Blackhole leaves it up while b, a switch, drops what arrives
// on its port from a, so the direction from a to b loses everything and the
// direction from b to a is unchanged. Loss and Rate set the link's error rate
// or its speed in the directions their LinkDirection names. Restore returns the
// link to the state it had before its latest failure, which must not be a Down.
enum class LinkEventKind { Down, Blackhole, Loss, Rate, Restore };

// The directions a Loss or a Rate applies in. Both is the two directions.
// Forward is the direction from a to b alone: under Loss b's device drops what
// it receives from a, as a device's error model is applied on receipt, and
// under Rate a's device serializes at the rate, as a device's data rate is its
// transmitter's. The direction from b to a is unchanged.
enum class LinkDirection { Both, Forward };

struct LinkEvent {
	uint64_t timeNs = 0;
	LinkEventKind kind = LinkEventKind::Down;
	uint32_t a = 0, b = 0;
	// Loss's error rate, in (0, 1], of a packet under ERROR_UNIT_PACKET, the
	// default, or of each of a packet's bytes under ERROR_UNIT_BYTE, so that
	// a packet of s bytes is lost with 1 - (1 - errorRate)^s: a frame
	// corrupted in any byte fails its check and is dropped.
	double errorRate = 0;
	RateErrorModel::ErrorUnit unit = RateErrorModel::ERROR_UNIT_PACKET;
	// Under ERROR_UNIT_BYTE, the bytes of the full data frame errorRate was
	// derived at, which the run's data frame must have.
	uint32_t frameBytes = 0;
	// Rate's data rate.
	DataRate rate;
	// Loss's or Rate's directions; the other kinds ignore it.
	LinkDirection direction = LinkDirection::Both;
};

// Sort events into the order they apply in: by time, a restore before a
// failure at the same time, and otherwise as given.
void OrderLinkEvents(std::vector<LinkEvent> &events);

// For events in the order they apply in, why one of them cannot apply: a
// restore of a link taken down, or of a link with no failure before it that
// no restore has put back. A restore puts back the failure named with the
// same two nodes in the same order, else the one named in the other order.
// Empty when every one can.
std::string CheckLinkEvents(const std::vector<LinkEvent> &events);

// A link's devices at a and at b, and b when b is a switch, else null.
struct LinkEnds {
	Ptr<QbbNetDevice> atA, atB;
	Ptr<SwitchNode> switchAtB;
};

// Applies link events and keeps, for each link, the state its latest failure
// replaced, which a restore puts back: the data rates of the ends it set under
// Rate, the error models, rates and units of the ends it set under Loss (a
// restore removes a model the failure added), and b's port's black hole flag
// under Blackhole. A Loss or Rate in both directions sets both ends; a Forward Loss
// sets b's end alone and a Forward Rate a's end alone. The state is kept per
// failure under its two nodes in the order named, so that failures of the two
// directions of a link, such as a black hole at each end, are put back
// independently; a later failure named in the same order replaces the state
// kept. A restore names a link by its two nodes in either order: it puts back
// the failure named in the same order if one is kept, else the one named in
// the other order, and forgets it.
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
		RateErrorModel::ErrorUnit errorUnit = RateErrorModel::ERROR_UNIT_PACKET;
	};
	// device's error model set to event's rate and unit.
	void SetLoss(Ptr<QbbNetDevice> device, const LinkEvent &event);
	struct Saved {
		LinkEventKind kind;
		LinkEnds ends;
		EndState atA, atB;
		bool blackholed = false;
		// Whether the failure set a's end and b's end.
		bool setA = false, setB = false;
	};
	static EndState Capture(Ptr<QbbNetDevice> device);
	static void PutBack(Ptr<QbbNetDevice> device, const EndState &state,
		LinkEventKind kind);
	void Restore(const LinkEvent &event);

	SetErrorRate m_setErrorRate;
	TakeDown m_takeDown;
	// By the failure's a and b, in that order.
	std::map<std::pair<uint32_t, uint32_t>, Saved> m_saved;
};

} /* namespace ns3 */

#endif /* LINK_EVENTS_H */
