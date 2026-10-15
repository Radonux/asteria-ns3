#include "link-events.h"
#include "qbb-net-device.h"
#include "switch-node.h"
#include <ns3/abort.h>
#include <ns3/error-model.h>
#include <algorithm>
#include <set>
#include <sstream>

namespace ns3 {

void OrderLinkEvents(std::vector<LinkEvent> &events){
	std::stable_sort(events.begin(), events.end(),
		[](const LinkEvent &x, const LinkEvent &y){
			if (x.timeNs != y.timeNs)
				return x.timeNs < y.timeNs;
			return x.kind == LinkEventKind::Restore &&
				y.kind != LinkEventKind::Restore;
		});
}

std::string CheckLinkEvents(const std::vector<LinkEvent> &events){
	// The failures no restore has put back yet, by their nodes in the order
	// named, and the links taken down, in either order.
	std::set<std::pair<uint32_t, uint32_t>> failed, down;
	for (const LinkEvent &event : events){
		const std::pair<uint32_t, uint32_t> named{event.a, event.b};
		const std::pair<uint32_t, uint32_t> reversed{event.b, event.a};
		if (event.kind != LinkEventKind::Restore){
			failed.insert(named);
			if (event.kind == LinkEventKind::Down)
				down.insert(std::minmax(event.a, event.b));
			continue;
		}
		std::ostringstream why;
		if (down.count(std::minmax(event.a, event.b)) > 0)
			why << "LINK_RESTORE at " << event.timeNs << " names " << event.a
				<< " " << event.b << ", a link taken down, which no restore "
				"brings back";
		else if (failed.erase(named) == 0 && failed.erase(reversed) == 0)
			why << "LINK_RESTORE at " << event.timeNs << " names " << event.a
				<< " " << event.b << ", a link with no failure before it that "
				"no restore has put back";
		if (!why.str().empty())
			return why.str();
	}
	return "";
}

LinkEventApplier::LinkEventApplier(SetErrorRate setErrorRate, TakeDown takeDown)
	: m_setErrorRate(std::move(setErrorRate)), m_takeDown(std::move(takeDown))
{
}

LinkEventApplier::EndState LinkEventApplier::Capture(Ptr<QbbNetDevice> device){
	EndState state;
	state.rate = device->GetDataRate();
	state.errorModel = DynamicCast<RateErrorModel>(device->GetLinkErrorModel());
	if (state.errorModel){
		state.errorRate = state.errorModel->GetRate();
		state.errorUnit = state.errorModel->GetUnit();
	}
	return state;
}

void LinkEventApplier::PutBack(Ptr<QbbNetDevice> device, const EndState &state,
		LinkEventKind kind){
	if (kind == LinkEventKind::Rate){
		device->SetDataRate(state.rate);
		return;
	}
	if (state.errorModel){
		state.errorModel->SetRate(state.errorRate);
		state.errorModel->SetUnit(state.errorUnit);
	}
	device->SetLinkErrorModel(state.errorModel);
}

void LinkEventApplier::SetLoss(Ptr<QbbNetDevice> device,
		const LinkEvent &event){
	m_setErrorRate(device, event.errorRate);
	DynamicCast<RateErrorModel>(device->GetLinkErrorModel())->SetUnit(
		event.unit);
}

void LinkEventApplier::Apply(const LinkEvent &event, const LinkEnds &ends){
	if (event.kind == LinkEventKind::Restore){
		Restore(event);
		return;
	}
	Saved saved{event.kind, ends, Capture(ends.atA), Capture(ends.atB), false};
	const bool forward = event.direction == LinkDirection::Forward;
	switch (event.kind){
	case LinkEventKind::Down:
		m_takeDown(event.a, event.b);
		break;
	case LinkEventKind::Blackhole:
		NS_ABORT_MSG_IF(!ends.switchAtB, "a black hole is a switch's port");
		saved.blackholed = ends.switchAtB->IsBlackholed(ends.atB->GetIfIndex());
		ends.switchAtB->SetBlackhole(ends.atB->GetIfIndex());
		break;
	case LinkEventKind::Loss:
		// A device's error model drops what it receives.
		saved.setA = !forward;
		saved.setB = true;
		if (saved.setA)
			SetLoss(ends.atA, event);
		SetLoss(ends.atB, event);
		break;
	case LinkEventKind::Rate:
		// A device's data rate is the rate it transmits at.
		saved.setA = true;
		saved.setB = !forward;
		ends.atA->SetDataRate(event.rate);
		if (saved.setB)
			ends.atB->SetDataRate(event.rate);
		break;
	case LinkEventKind::Restore:
		break;
	}
	m_saved[{event.a, event.b}] = saved;
}

void LinkEventApplier::Restore(const LinkEvent &event){
	// The failure named in the same order, else the one named in the other.
	auto found = m_saved.find({event.a, event.b});
	if (found == m_saved.end())
		found = m_saved.find({event.b, event.a});
	NS_ABORT_MSG_IF(found == m_saved.end(),
		"a restore of " << event.a << " " << event.b << " follows no failure "
		"that no restore has put back");
	const Saved saved = found->second;
	m_saved.erase(found);
	switch (saved.kind){
	case LinkEventKind::Down:
		NS_ABORT_MSG("a link taken down is not restored");
		break;
	case LinkEventKind::Blackhole:
		if (!saved.blackholed)
			saved.ends.switchAtB->ClearBlackhole(saved.ends.atB->GetIfIndex());
		break;
	case LinkEventKind::Loss:
	case LinkEventKind::Rate:
		if (saved.setA)
			PutBack(saved.ends.atA, saved.atA, saved.kind);
		if (saved.setB)
			PutBack(saved.ends.atB, saved.atB, saved.kind);
		break;
	case LinkEventKind::Restore:
		break;
	}
}

} /* namespace ns3 */
