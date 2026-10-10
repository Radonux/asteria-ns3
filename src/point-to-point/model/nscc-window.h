#ifndef NSCC_WINDOW_H
#define NSCC_WINDOW_H

#include <stdint.h>

namespace ns3 {

// The congestion window of UEC 1.0.3 section 3.6.13 (NSCC) for one queue pair:
// one window in payload bytes over every path the queue pair's packets take.
// Each answer to a send moves it: an acknowledgement through the four cases of
// ECN mark and queueing delay, QuickAdapt and fast increase, a trim and a
// declared loss through a cut. The sections cited below are of that
// specification. Times are in nanoseconds.
class NsccWindow {
public:
	// Table 3-82's parameters, each a coefficient of the queue pair's packet
	// size, base RTT, bandwidth-delay product or the scaling factors derived
	// from them; Start resolves them for one queue pair.
	struct Parameters {
		double target_qdelay; // in base RTTs
		double max_window; // in BDPs
		double gamma;
		double max_md_jump;
		double fair_increase; // in MTUs times scaling_a
		double fast_increase_scale; // in scaling_a
		double eta; // in MTUs times scaling_a
		double alpha; // in MTUs times scaling_a times scaling_b per target_qdelay
		uint32_t qa_gate;
		double qa_threshold; // in target_qdelays; zero never triggers
		uint32_t adjust_bytes; // in MTUs
		double adjust_period; // in base RTTs
	};
	// The round trip of an answer that cannot be attributed to one send.
	static constexpr uint64_t kNoRtt = UINT64_MAX;

	NsccWindow();
	// Opens the window at its ceiling, MaxWnd (Table 3-83).
	void Start(const Parameters &parameters, uint32_t mtu, uint64_t bdp,
		uint64_t baseRtt, uint64_t now);
	uint64_t Cwnd() const;
	uint64_t MaxWnd() const;
	uint64_t BaseRtt() const;
	// NSCC.OnACK for an acknowledgement of bytes that left the network, with
	// the answered packet's mark and round trip; inflight is what is still
	// outstanding once those bytes are not. rcvCwndPend is the destination's
	// Rcv_Cwnd_Pend (section 3.6.13.2), 0 to 127, with Restore_Cwnd clear.
	void OnAck(uint32_t bytes, bool marked, uint64_t rtt, uint64_t inflight,
		uint64_t now, uint32_t rcvCwndPend);
	// NSCC.OnNACK for a trim of bytes, inflight as for OnAck.
	void OnTrim(uint32_t bytes, uint64_t rtt, uint64_t inflight, uint64_t now);
	// NSCC.OnInferredLoss for bytes declared lost by a timeout or by an
	// acknowledgement.
	void OnLoss(uint64_t bytes);

private:
	// apply_cwnd_penalty: true while the destination holds the window down.
	bool ApplyCwndPenalty(uint32_t rcvCwndPend, uint32_t bytes,
		uint64_t inflight);
	bool QuickAdapt(bool loss, bool marked, uint64_t delay, uint64_t inflight,
		uint64_t now);
	void FastIncrease(uint32_t bytes, uint64_t delay);
	void MultiplicativeDecrease(uint64_t now);
	void FulfillAdjustment(uint64_t now);
	void UpdateBaseRtt(uint64_t rtt);
	void UpdateDelay(uint64_t delay, uint64_t now);
	void Shrink(uint64_t bytes);

	// The parameters as resolved by Start.
	uint32_t m_mtu;
	uint64_t m_config_base_rtt;
	uint64_t m_target_qdelay;
	double m_max_window; // MaxWnd in BDPs
	double m_linkspeed; // bytes per nanosecond
	double m_gamma;
	double m_max_md_jump;
	double m_fi;
	double m_fi_scale;
	double m_eta;
	double m_alpha;
	uint32_t m_qa_gate;
	uint64_t m_qa_threshold;
	uint64_t m_adjust_bytes;
	uint64_t m_adjust_period;

	// The source state of Table 3-83.
	uint64_t m_cwnd;
	uint64_t m_max_wnd;
	uint64_t m_base_rtt;
	uint64_t m_achieved_bytes;
	uint64_t m_received_bytes;
	uint64_t m_fi_count;
	bool m_trigger_qa;
	uint64_t m_qa_endtime;
	uint64_t m_bytes_to_ignore;
	uint64_t m_bytes_ignored;
	double m_inc_bytes;
	uint64_t m_last_adjust_time;
	bool m_increase_mode;
	uint64_t m_last_dec_time;
	// The averaging state behind update_delay and get_avg_delay.
	double m_avg_delay;
	uint64_t m_last_delay_time;
};

} // namespace ns3

#endif /* NSCC_WINDOW_H */
