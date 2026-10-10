#include "nscc-window.h"
#include <ns3/assert.h>
#include <algorithm>

namespace ns3 {

namespace {

// base_BDP of Table 3-82, the bandwidth-delay product of 100 Gb/s over a 12 us
// base RTT, and that RTT, which scaling_b divides target_qdelay by. Every
// increase constant is defined on that network and scaled to this one.
constexpr double kReferenceBdp = 150000;
constexpr double kReferenceRtt = 12000;
// How close to the base RTT a round trip must be for fast_increase to count
// it as clear: the "delay ~= 0" of section 3.6.13.6, which neither the
// specification nor SMaRTT quantifies. One microsecond is the tolerance of the
// Ultra Ethernet Consortium's htsim model of NSCC.
constexpr uint64_t kClearDelay = 1000;

} // namespace

NsccWindow::NsccWindow()
	: m_mtu(0), m_config_base_rtt(0), m_target_qdelay(0), m_max_window(0),
	  m_linkspeed(0), m_gamma(0), m_max_md_jump(0), m_fi(0), m_fi_scale(0),
	  m_eta(0), m_alpha(0), m_qa_gate(0), m_qa_threshold(0), m_adjust_bytes(0),
	  m_adjust_period(0), m_cwnd(0), m_max_wnd(0), m_base_rtt(0),
	  m_achieved_bytes(0), m_received_bytes(0), m_fi_count(0),
	  m_trigger_qa(false), m_qa_endtime(0), m_bytes_to_ignore(0),
	  m_bytes_ignored(0), m_inc_bytes(0), m_last_adjust_time(0),
	  m_increase_mode(false), m_last_dec_time(0), m_avg_delay(0),
	  m_last_delay_time(0)
{
}

void NsccWindow::Start(const Parameters &parameters, uint32_t mtu,
		uint64_t bdp, uint64_t baseRtt, uint64_t now){
	NS_ASSERT_MSG(mtu > 0 && bdp > 0 && baseRtt > 0,
		"the window is sized from the path's packet, BDP and base RTT");
	NS_ASSERT_MSG(parameters.target_qdelay > 0,
		"the proportional increase is relative to the target delay");
	// Section 3.6.13.3, with the queue pair's base RTT as config_base_rtt and
	// its bandwidth-delay product as BDP.
	const double scalingA = bdp / kReferenceBdp;
	m_mtu = mtu;
	m_config_base_rtt = baseRtt;
	m_target_qdelay = static_cast<uint64_t>(parameters.target_qdelay * baseRtt);
	const double scalingB = m_target_qdelay / kReferenceRtt;
	m_max_window = parameters.max_window;
	m_linkspeed = static_cast<double>(bdp) / baseRtt;
	m_gamma = parameters.gamma;
	m_max_md_jump = parameters.max_md_jump;
	m_fi = parameters.fair_increase * mtu * scalingA;
	m_fi_scale = parameters.fast_increase_scale * scalingA;
	m_eta = parameters.eta * mtu * scalingA;
	m_alpha = parameters.alpha * scalingA * scalingB * mtu / m_target_qdelay;
	m_qa_gate = parameters.qa_gate;
	m_qa_threshold = parameters.qa_threshold == 0 ? UINT64_MAX
		: static_cast<uint64_t>(parameters.qa_threshold * m_target_qdelay);
	m_adjust_bytes = static_cast<uint64_t>(parameters.adjust_bytes) * mtu;
	m_adjust_period = static_cast<uint64_t>(parameters.adjust_period * baseRtt);

	m_max_wnd = static_cast<uint64_t>(m_max_window * bdp);
	m_cwnd = m_max_wnd;
	m_base_rtt = baseRtt;
	m_achieved_bytes = m_received_bytes = m_fi_count = 0;
	m_trigger_qa = false;
	m_qa_endtime = 0;
	m_bytes_to_ignore = m_bytes_ignored = 0;
	m_inc_bytes = 0;
	m_last_adjust_time = m_last_dec_time = m_last_delay_time = now;
	m_increase_mode = false;
	m_avg_delay = 0;
}

uint64_t NsccWindow::Cwnd() const{
	return m_cwnd;
}

uint64_t NsccWindow::MaxWnd() const{
	return m_max_wnd;
}

uint64_t NsccWindow::BaseRtt() const{
	return m_base_rtt;
}

void NsccWindow::OnAck(uint32_t bytes, bool marked, uint64_t rtt,
		uint64_t inflight, uint64_t now, uint32_t rcvCwndPend){
	m_bytes_ignored += bytes;
	m_received_bytes += bytes;
	m_achieved_bytes += bytes;
	const bool increase = !ApplyCwndPenalty(rcvCwndPend, bytes, inflight) &&
		!marked;
	if (rtt == kNoRtt)
		return;
	UpdateBaseRtt(rtt);
	const uint64_t delay = rtt - m_base_rtt;
	UpdateDelay(delay, now);
	if (QuickAdapt(false, marked, delay, inflight, now))
		return;
	if (increase && delay >= m_target_qdelay){
		// fair_increase: the same bytes for every competing window, so a small
		// window grows by a larger fraction of itself.
		m_inc_bytes += m_fi * bytes;
	}else if (increase){
		// proportional_increase, which fast_increase overrides on a clear path.
		FastIncrease(bytes, delay);
		if (!m_increase_mode)
			m_inc_bytes += m_alpha * bytes * (m_target_qdelay - delay);
	}else if (marked && delay >= m_target_qdelay){
		MultiplicativeDecrease(now);
	}
	// A mark at a delay below the target leaves the window alone: the queue
	// that marked the packet has not yet delayed it, and ECN-driven load
	// balancing gets to move traffic off that queue before the aggregate
	// window shrinks (section 3.6.13).
	if (now - m_last_adjust_time >= m_adjust_period ||
			m_received_bytes > m_adjust_bytes)
		FulfillAdjustment(now);
}

void NsccWindow::OnTrim(uint32_t bytes, uint64_t rtt, uint64_t inflight,
		uint64_t now){
	if (rtt != kNoRtt)
		UpdateBaseRtt(rtt);
	// A trimmed packet met a full queue; config_base_rtt stands in for the
	// delay it would have seen.
	UpdateDelay(m_config_base_rtt, now);
	m_bytes_ignored += bytes;
	// Every trim cuts the window: the last-hop exception of section 3.6.13.5
	// applies only where receiver credit (RCCC) manages the last hop, and this
	// transport has none.
	m_trigger_qa = true;
	// A trim counts as a mark at zero delay, so that the loss rather than a
	// delay decides the QuickAdapt.
	if (!QuickAdapt(true, true, 0, inflight, now))
		Shrink(bytes);
}

void NsccWindow::OnLoss(uint64_t bytes){
	Shrink(bytes);
	m_bytes_ignored += bytes;
}

bool NsccWindow::ApplyCwndPenalty(uint32_t rcvCwndPend, uint32_t bytes,
		uint64_t inflight){
	if (rcvCwndPend == 0)
		return false;
	m_cwnd = std::min(m_cwnd, inflight);
	Shrink((static_cast<uint64_t>(rcvCwndPend) * bytes) >> 7);
	return true;
}

bool NsccWindow::QuickAdapt(bool loss, bool marked, uint64_t delay,
		uint64_t inflight, uint64_t now){
	bool handled = false;
	if (m_bytes_ignored < m_bytes_to_ignore && marked){
		// The marks of the bytes in flight when QuickAdapt last set the window
		// describe the queue it already sized the window for.
		handled = true;
	}else if (now >= m_qa_endtime){
		if (m_qa_endtime != 0 && (m_trigger_qa || loss || delay > m_qa_threshold) &&
				m_achieved_bytes < (m_max_wnd >> m_qa_gate)){
			m_cwnd = std::max(m_achieved_bytes, static_cast<uint64_t>(m_mtu));
			m_bytes_to_ignore = inflight;
			m_bytes_ignored = 0;
			m_trigger_qa = false;
			handled = true;
		}
		m_achieved_bytes = 0;
		m_qa_endtime = now + m_base_rtt + m_target_qdelay;
	}
	if (handled){
		m_inc_bytes = 0;
		m_received_bytes = 0;
	}
	return handled;
}

void NsccWindow::FastIncrease(uint32_t bytes, uint64_t delay){
	if (delay < kClearDelay){
		m_fi_count += bytes;
		if (m_fi_count > m_cwnd || m_increase_mode){
			m_cwnd = std::min(m_cwnd + static_cast<uint64_t>(bytes * m_fi_scale),
				m_max_wnd);
			m_increase_mode = true;
			return;
		}
	}else{
		m_fi_count = 0;
	}
	m_increase_mode = false;
}

void NsccWindow::MultiplicativeDecrease(uint64_t now){
	m_increase_mode = false;
	m_fi_count = 0;
	if (m_avg_delay > m_target_qdelay && now - m_last_dec_time > m_base_rtt){
		// The cut is proportional to how far the average delay exceeds the
		// target, and at most once per base RTT, because the delay of the
		// next RTT's packets does not yet reflect it.
		const double factor = std::max(
			1 - m_gamma * (m_avg_delay - m_target_qdelay) / m_avg_delay,
			m_max_md_jump);
		m_cwnd = std::max(static_cast<uint64_t>(m_cwnd * factor),
			static_cast<uint64_t>(m_mtu));
		m_last_dec_time = now;
	}
}

void NsccWindow::FulfillAdjustment(uint64_t now){
	m_cwnd += static_cast<uint64_t>(m_inc_bytes / m_cwnd);
	if (now - m_last_adjust_time >= m_adjust_period){
		m_last_adjust_time = now;
		m_cwnd += static_cast<uint64_t>(m_eta);
	}
	m_cwnd = std::min(m_cwnd, m_max_wnd);
	m_inc_bytes = 0;
	m_received_bytes = 0;
}

void NsccWindow::UpdateBaseRtt(uint64_t rtt){
	if (rtt >= m_base_rtt)
		return;
	m_base_rtt = rtt;
	m_max_wnd = static_cast<uint64_t>(m_max_window * m_linkspeed * rtt);
}

void NsccWindow::UpdateDelay(uint64_t delay, uint64_t now){
	// get_avg_delay is to return the average over the last base RTT (section
	// 3.6.13.6), so each sample is weighted by the time since the previous one
	// in base RTTs: the average then forgets at the same pace whether the
	// acknowledgements are dense or sparse.
	const double weight = std::min(1.0,
		static_cast<double>(now - m_last_delay_time) / m_base_rtt);
	m_avg_delay += weight * (delay - m_avg_delay);
	m_last_delay_time = now;
}

void NsccWindow::Shrink(uint64_t bytes){
	m_cwnd = m_cwnd > bytes + m_mtu ? m_cwnd - bytes : m_mtu;
}

} // namespace ns3
