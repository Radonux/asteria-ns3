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
	std::set<std::pair<uint32_t, uint32_t>> failed, down;
	for (const LinkEvent &event : events){
		const auto key = std::minmax(event.a, event.b);
		if (event.kind != LinkEventKind::Restore){
			failed.insert(key);
			if (event.kind == LinkEventKind::Down)
				down.insert(key);
			continue;
		}
		std::ostringstream why;
		if (down.count(key) > 0)
			why << "LINK_RESTORE at " << event.timeNs << " names " << event.a
				<< " " << event.b << ", a link taken down, which no restore "
				"brings back";
		else if (failed.count(key) == 0)
			why << "LINK_RESTORE at " << event.timeNs << " names " << event.a
				<< " " << event.b << ", a link with no failure before it";
		if (!why.str().empty())
			return why.str();
	}
	return "";
}

LinkEventApplier::LinkEventApplier(SetErrorRate setErrorRate, TakeDown takeDown)
	: m_setErrorRate(std::move(setErrorRate)), m_takeDown(std::move(takeDown))
{
}

std::pair<uint32_t, uint32_t> LinkEventApplier::Key(uint32_t a, uint32_t b){
	return std::minmax(a, b);
}

LinkEventApplier::EndState LinkEventApplier::Capture(Ptr<QbbNetDevice> device){
	EndState state;
	state.rate = device->GetDataRate();
	state.errorModel = DynamicCast<RateErrorModel>(device->GetLinkErrorModel());
	if (state.errorModel)
		state.errorRate = state.errorModel->GetRate();
	return state;
}

void LinkEventApplier::PutBack(Ptr<QbbNetDevice> device, const EndState &state,
		LinkEventKind kind){
	if (kind == LinkEventKind::Rate){
		device->SetDataRate(state.rate);
		return;
	}
	if (state.errorModel)
		state.errorModel->SetRate(state.errorRate);
	device->SetLinkErrorModel(state.errorModel);
}

void LinkEventApplier::Apply(const LinkEvent &event, const LinkEnds &ends){
	if (event.kind == LinkEventKind::Restore){
		Restore(event);
		return;
	}
	Saved saved{event.kind, ends, Capture(ends.atA), Capture(ends.atB), false};
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
		m_setErrorRate(ends.atA, event.errorRate);
		m_setErrorRate(ends.atB, event.errorRate);
		break;
	case LinkEventKind::Rate:
		ends.atA->SetDataRate(event.rate);
		ends.atB->SetDataRate(event.rate);
		break;
	case LinkEventKind::Restore:
		break;
	}
	m_saved[Key(event.a, event.b)] = saved;
}

void LinkEventApplier::Restore(const LinkEvent &event){
	const auto found = m_saved.find(Key(event.a, event.b));
	NS_ABORT_MSG_IF(found == m_saved.end(),
		"a restore of " << event.a << " " << event.b << " follows no failure");
	const Saved &saved = found->second;
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
		PutBack(saved.ends.atA, saved.atA, saved.kind);
		PutBack(saved.ends.atB, saved.atB, saved.kind);
		break;
	case LinkEventKind::Restore:
		break;
	}
}

} /* namespace ns3 */
