#ifndef RDMA_HW_H
#define RDMA_HW_H

#include <ns3/rdma.h>
#include <ns3/rdma-queue-pair.h>
#include <ns3/node.h>
#include <ns3/custom-header.h>
#include <ns3/random-variable-stream.h>
#include <ns3/traced-callback.h>
#include "qbb-net-device.h"
#include "load-balancing.h"
#include "nscc-window.h"
#include "spine-grader.h"
#include "spine-redirect.h"
#include "switch-queue-state.h"
#include <memory>
#include <unordered_map>
#include "pint.h"

namespace ns3 {

enum class RdmaFailureReason : uint32_t {
	TimeoutRetryExhausted = 1,
	TrimRetryExhausted,
	// Cumulative acknowledgement made no progress within the configured
	// deadline even though recovery signals kept arriving. This is the
	// bound the retry budget cannot provide once NACKs and trim
	// notifications are (correctly) exempt from it.
	NoForwardProgress,
};

// The data packets that arrived over one spine, by the carrying spine their
// identification names.
struct SpineArrivals {
	uint64_t packets;
	uint64_t payloadBytes;
	uint64_t folded; // carried on a request for another spine
};

struct RdmaInterfaceMgr{
	Ptr<QbbNetDevice> dev;
	Ptr<RdmaQueuePairGroup> qpGrp;

	RdmaInterfaceMgr() : dev(NULL), qpGrp(NULL) {}
	RdmaInterfaceMgr(Ptr<QbbNetDevice> _dev){
		dev = _dev;
	}
};

class RdmaHw : public Object {
public:

	static TypeId GetTypeId (void);
	RdmaHw();

	Ptr<Node> m_node;
	DataRate m_minRate;		//< Min sending rate
	uint32_t m_mtu;
	uint32_t m_cc_mode;
	double m_nack_interval;
	uint32_t m_chunk;
	uint32_t m_ack_interval;
	bool m_backto0;
	uint64_t m_retransmission_timeout_ns;
	uint32_t m_max_retransmission_retries;
	uint64_t m_no_progress_timeout_ns;
	bool m_selective_retransmission;
	// Recovery-domain bounded loss. The transport never decides what may be
	// forgiven: it asks the verdict callback, which owns eligibility, the
	// step, and the budget. Requires selective retransmission, because a
	// forgiven range is absorbed as an accepted out-of-order range.
	bool m_forgiveness;
	// Congestion-exempt forgiveness. A queue pair the exemption callback
	// answers for takes no rate cut until the receiver refuses to forgive one
	// of its trims. The transport asks once, at birth, and never learns why.
	bool m_congestionExemption;
	// Whether a spent report ends an exemption. False is the reference arm in
	// which the budget alone bounds the loss.
	bool m_reengage;
	// LoadBalancingMode. Outside ECMP a queue pair's PathSelector draws the
	// IPv4 identification of each of its data packets from m_pathRandom;
	// SprayUniform and SprayPolicy draw a spine index below m_spineCount.
	uint32_t m_loadBalancing;
	uint32_t m_spineCount;
	Ptr<UniformRandomVariable> m_pathRandom;
	// PathSelectorKind under EntropyHash, and the parameters of the selectors.
	uint32_t m_pathSelectorKind;
	uint32_t m_repsBufferSize;
	uint64_t m_repsFreezingTimeoutNs;
	uint32_t m_ueEvSetSize;
	double m_ueSaturationFraction;
	uint32_t m_mrcEvSetSize;
	double m_mrcSkipBaseRtts;
	double m_mrcProbeTimeouts;
	// PathSelectorKind::AckRedirect, under SprayUniform: SpineRedirect's omega
	// and flag, its validity cycle in units of m_sprayBaseRttNs, and
	// TrimScoring.
	double m_ackRedirectOmega;
	double m_ackRedirectFlag;
	double m_ackRedirectValidityBaseRtts;
	uint32_t m_ackRedirectTrimScoring;
	// GradeScope of the redirect: PerConnection scores each arrival in its
	// receive queue pair's SpineRedirect; PerReceiver scores the spines from
	// the averages of the host's grader, as m_hostRedirect.
	uint32_t m_ackRedirectScope;
	HostSpineRedirect m_hostRedirect;
	bool IsHostRedirect() const;
	HostSpineRedirect::Parameters HostRedirectParameters() const;
	// The addresses of the hosts on this host's leaf, sorted. Their packets
	// cross no spine, so they are not scored for a redirect.
	std::vector<uint32_t> m_ownLeafHosts;
	bool IsAckRedirect() const;
	// The redirect field of the answer to the data packet or trimmed header of
	// ch arriving at q, which is scored unless its sender is on this host's
	// leaf.
	uint8_t ScoreForRedirect(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch,
		bool marked);
	// SprayPolicy. Times are in units of m_sprayBaseRttNs, the fabric's base
	// RTT, which AckRedirect also counts in; the receiver's parameters are
	// those of SpineGrader and the sender's those of SpineScores.
	uint64_t m_sprayBaseRttNs;
	double m_sprayReportIntervalBaseRtts;
	double m_sprayEstimatorGain;
	uint32_t m_sprayEstimatorIntervalSamples;
	double m_sprayFractionCusumSlack;
	double m_sprayFractionCusumThreshold;
	double m_sprayCongestionThreshold1, m_sprayCongestionThreshold2, m_sprayCongestionThreshold3;
	double m_sprayAbsenceFractionOfMedian;
	uint32_t m_sprayAbsenceMinimumMedian;
	double m_sprayReleaseFractionOfFloor;
	bool m_sprayOneWayDelay;
	uint32_t m_sprayGradeReference; // SpineGrader::GradeReference
	uint32_t m_sprayGradeScope; // GradeScope
	// GradeCostSource. Under SwitchQueue, at each close of the host's
	// grader's interval the spines are graded once per source leaf from
	// m_switchQueues, simulator state no host could read, and an answer to a
	// sender on another leaf carries its leaf's report. m_leaf is this host's
	// leaf.
	uint32_t m_sprayGradeCostSource;
	Ptr<SwitchQueueState> m_switchQueues;
	uint32_t m_leaf;
	std::vector<SpineReport> m_leafReports;
	bool IsSwitchQueueCost() const;
	// The report an answer to destinationIp from q carries.
	const SpineReport &ReportFor(Ptr<RdmaRxQueuePair> q, uint32_t destinationIp);
	// Grade every source leaf's paths from the switches' state, at a close.
	void GradeFromSwitchQueues(uint64_t nowNs, const SpineGrader &grader);
	// The SwitchQueueReport trace source: the time, the host's grader, the
	// source leaf, the values read and the report they earned.
	typedef void (*SwitchQueueReportTracedCallback)(uint64_t nowNs,
		const SpineGrader &grader, uint32_t sourceLeaf,
		const std::vector<SpineGrader::SpineValues> &values,
		const SpineReport &report);
	TracedCallback<uint64_t, const SpineGrader &, uint32_t,
		const std::vector<SpineGrader::SpineValues> &, const SpineReport &>
		m_traceSwitchQueueReport;
	double m_sprayDelayCusumSlackBaseRtts;
	double m_sprayDelayCusumThresholdBaseRtts;
	double m_sprayDelayThreshold1BaseRtts, m_sprayDelayThreshold2BaseRtts,
		m_sprayDelayThreshold3BaseRtts;
	double m_sprayGamma;
	double m_sprayEpsilon;
	uint32_t m_sprayCandidates;
	uint32_t m_sprayCandidateDraw; // SpineScores::CandidateDraw
	// SpineScores::Parameters::edgeFreeze.
	bool m_sprayEdgeFreeze;
	// The Rcv_Cwnd_Pend NSCC is handed for an acknowledgement whose report has
	// the edge bit set.
	uint32_t m_sprayEdgeWindowPenalty;
	// Under GradeScope::PerReceiver, the receiver's grader, made at the first
	// event it counts, and the sender's scores towards each destination host
	// by address, shared by every queue pair to that host. Under
	// PerConnection each receive queue pair keeps its own grader and each
	// queue pair its own scores.
	std::unique_ptr<SpineGrader> m_spineGrader;
	std::unordered_map<uint32_t, SpineScores> m_spineScores;
	// The SpineReport trace source: the time, the grader that has just issued
	// a report, and under PerConnection the receive queue pair that owns it,
	// null under PerReceiver.
	typedef void (*SpineReportTracedCallback)(uint64_t nowNs,
		const SpineGrader &grader, const RdmaRxQueuePair *connection);
	TracedCallback<uint64_t, const SpineGrader &, const RdmaRxQueuePair *>
		m_traceSpineReport;
	bool IsSprayPolicy() const;
	// Whether the host keeps a grader: under SprayPolicy, and for the
	// redirect of PathSelectorKind::AckRedirect at GradeScope::PerReceiver.
	bool IsGrading() const;
	bool IsPerConnectionGrading() const;
	uint64_t SprayReportIntervalNs() const;
	SpineGrader::Parameters GraderParameters() const;
	// The host's grader.
	SpineGrader &Grader();
	// The grader q's arrivals feed and q's answers report: the host's, or
	// under PerConnection q's own, in which case q is not null.
	SpineGrader &GraderOf(Ptr<RdmaRxQueuePair> q);
	// That grader once it has closed the interval that ended by now.
	SpineGrader &GraderNow(Ptr<RdmaRxQueuePair> q);
	// The data packet or trimmed header of ch counted by q's grader.
	void GradeArrival(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch,
		bool marked);
	void GradeTrim(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch, bool lastHop);
	// Indexed by carrying spine; kept where the identification names one.
	std::vector<SpineArrivals> m_spineArrivals;
	bool m_var_win, m_fast_react;
	bool m_rateBound;
	uint32_t m_total_pause_times; 
	uint32_t m_paused_times;
	std::vector<RdmaInterfaceMgr> m_nic; // list of running nic controlled by this RdmaHw
	std::unordered_map<uint64_t, Ptr<RdmaQueuePair> > m_qpMap; // mapping from uint64_t to qp
	std::unordered_map<uint64_t, Ptr<RdmaRxQueuePair> > m_rxQpMap; // mapping from uint64_t to rx qp
	std::unordered_map<uint32_t, std::vector<int> > m_rtTable; // map from ip address (u32) to possible ECMP port (index of dev)

	// qp complete callback
	typedef Callback<void, Ptr<RdmaQueuePair> > QpCompleteCallback;
	QpCompleteCallback m_qpCompleteCallback;
	typedef Callback<void, Ptr<RdmaQueuePair>, uint32_t> QpFailureCallback;
	QpFailureCallback m_qpFailureCallback;
	// Host-transport events no packet trace can observe: a retransmission
	// timeout firing and a DCQCN rate cut being taken. The scratch layer
	// aggregates them into transport_summary.csv beside the wire events.
	typedef Callback<void, const char*, uint64_t> TransportEventCallback;
	TransportEventCallback m_transportEventCallback;
	void ReportTransportEvent(const char* event, uint64_t bytes);

	// (sip, dip, sport, dport, seq, len) -> forgive this trimmed range, or
	// repair it. One bit, because the receiver's report about the budget
	// entry is a property of the entry and not of this range, and it is asked
	// for separately below. Unset means repair, which is the behaviour of a
	// transport with no recovery domain at all. The transport is
	// semantics-blind and never reads a training step, a
	// critical-learning-regime label, or a budget.
	typedef Callback<bool, uint32_t, uint32_t, uint16_t, uint16_t, uint64_t,
		uint32_t> RecoveryVerdictCallback;
	RecoveryVerdictCallback m_recoveryVerdictCallback;
	// (sip, dip, sport, dport, holes) -> is the step's allowance gone. Asked
	// wherever an acknowledgement or a repair request leaves the receiver,
	// with the bytes this flow is missing right now; the frontend keeps the
	// sum over the rank's flows and answers against the step's total. Unset
	// means no, which leaves every sender following its controller only for
	// as long as the grant below says nothing else.
	typedef Callback<bool, uint32_t, uint32_t, uint16_t, uint16_t, uint64_t>
		AllowanceGoneCallback;
	AllowanceGoneCallback m_allowanceGoneCallback;
	// (sip, dip, sport, dport) -> may the experiment layer forgive this flow
	// on this step. Asked by the receiver, once per receive queue pair, and
	// carried on every acknowledgement that queue pair emits: the sender
	// cannot know either eligibility or the step's phase. Unset means no,
	// which is the behaviour of a transport with no exemption at all.
	typedef Callback<bool, uint32_t, uint32_t, uint16_t, uint16_t>
		ForgivenessEligibleCallback;
	ForgivenessEligibleCallback m_forgivenessEligibleCallback;
	// (sip, dip, sport, dport, next_expected, accepted_above) -> the end offset
	// the receiver may absorb without waiting for it, or zero to refuse. A
	// second callback rather than a struct on the trim path: the trim verdict
	// never needs an end, and the remainder verdict never needs bits. The
	// frontend knows the flow's size and the transport knows what arrived, so
	// the transport supplies both edges of the hole and the frontend charges
	// the difference. The transport asks at every accepted arrival and the
	// frontend decides; unset means nothing asks, which is the transport with
	// no step stop at all.
	typedef Callback<uint64_t, uint32_t, uint32_t, uint16_t, uint16_t,
		uint64_t, uint64_t> RemainderVerdictCallback;
	RemainderVerdictCallback m_remainderVerdictCallback;
	// (sip, dip, sport, dport, accepted, late_forgiven) -> the payload bytes
	// this arrival added to what the receiver holds, and the bytes of it that
	// arrived for a range already forgiven. A NIC accounts for a byte when it
	// accepts it, so the receiver-local budget grows here rather than at
	// completion. Only newly accepted bytes are counted as accepted: a
	// duplicate adds nothing, and a forgiven range was absorbed without data,
	// so neither is counted twice; the late bytes are reported beside them
	// because the budget was charged for a range the sender then delivered.
	// Unset means the experiment layer keeps no receiver-side account.
	typedef Callback<void, uint32_t, uint32_t, uint16_t, uint16_t, uint64_t,
		uint64_t> DataAcceptedCallback;
	DataAcceptedCallback m_dataAcceptedCallback;
	// What one arriving data packet would add, measured before the receive
	// state moves. Go-back-N accepts nothing out of order, and a range the
	// receiver already holds adds nothing.
	uint64_t AcceptedPayloadBytes(Ptr<RdmaRxQueuePair> q, uint32_t seq,
		uint32_t size) const;
	void SetNode(Ptr<Node> node);
	void Setup(QpCompleteCallback cb, QpFailureCallback failure_cb); // setup shared data and callbacks with the QbbNetDevice
	static uint64_t GetQpKey(uint32_t dip, uint16_t sport, uint16_t pg); // get the lookup key for m_qpMap
	Ptr<RdmaQueuePair> GetQp(uint32_t dip, uint16_t sport, uint16_t pg); // get the qp
	uint32_t GetNicIdxOfQp(Ptr<RdmaQueuePair> qp); // get the NIC index of the qp
	void AddQueuePair(uint32_t src, uint32_t dest, uint64_t tag, uint64_t size, uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport, uint16_t _dport, uint32_t win, uint64_t baseRtt, Callback<void> notifyAppFinish, Callback<void> notifyAppSent); // add a new qp (new send)
	void DeleteQueuePair(Ptr<RdmaQueuePair> qp);

	Ptr<RdmaRxQueuePair> GetRxQp(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport, uint16_t pg, bool create); // get a rxQp
	uint32_t GetNicIdxOfRxQp(Ptr<RdmaRxQueuePair> q); // get the NIC index of the rxQp
	void DeleteRxQp(uint32_t dip, uint16_t pg, uint16_t dport);

	int ReceiveUdp(Ptr<Packet> p, CustomHeader &ch);
	void CountArrival(Ptr<RdmaRxQueuePair> q, uint16_t identification,
		uint32_t payloadSize);
	int ReceiveCnp(Ptr<Packet> p, CustomHeader &ch);
	int ReceiveAck(Ptr<Packet> p, CustomHeader &ch); // handle both ACK and NACK
	int ReceiveTrim(Ptr<Packet> p, CustomHeader &ch);
	int ReceivePathProbe(Ptr<Packet> p, CustomHeader &ch);
	int Receive(Ptr<Packet> p, CustomHeader &ch); // callback function that the QbbNetDevice should use when receive packets. Only NIC can call this function. And do not call this upon PFC

	void PCIePause(uint32_t nic_idx, uint32_t qIndex);
	void PCIeResume(uint32_t nic_idx, uint32_t qIndex);
	void EnablePause();
	bool enable_pcie_pause; 

	void CheckandSendQCN(Ptr<RdmaRxQueuePair> q);
	int ReceiverCheckSeq(uint32_t seq, Ptr<RdmaRxQueuePair> q, uint32_t size);
	void AddHeader (Ptr<Packet> p, uint16_t protocolNumber);
	static uint16_t EtherToPpp (uint16_t protocol);

	void RecoverQueue(Ptr<RdmaQueuePair> qp);
	void RecoverTrimmedQueue(Ptr<RdmaQueuePair> qp, const CustomHeader &ch);
	// q is the receive queue pair of the trimmed packet's flow, which may be
	// null unless its answers carry state of their own.
	void SendTrimNack(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch,
		uint32_t sourceIp, uint32_t destinationIp, uint16_t sport,
		uint16_t dport, uint16_t pg, uint32_t seq, uint32_t payloadSize,
		bool lastHop, bool spent, bool eligible);
	// packetSeq and identification describe the data packet answered, or with
	// probeAnswer the probe; redirect is the redirect field, kNoRedirect
	// outside AckRedirect.
	void SendAck(Ptr<RdmaRxQueuePair> q, uint32_t sourceIp,
		uint32_t destinationIp, uint16_t sport, uint16_t dport, uint16_t pg,
		const IntHeader &ih, uint32_t packetSeq, uint16_t identification,
		bool nack, bool cnp, bool spent, bool probeAnswer, uint8_t redirect);
	// Read the receiver's two bits off an arriving repair request or
	// acknowledgement and follow them: eligible with room grants the
	// exemption and eligible with none returns the sender to its controller,
	// and a packet from a receiver that marks the flow ineligible says
	// nothing either way.
	void FollowAllowanceReport(Ptr<RdmaQueuePair> qp, const CustomHeader &ch);
	// The receiver's report, recomputed where it is emitted: this queue pair's
	// holes go to the frontend and the step's answer comes back.
	bool AllowanceGone(Ptr<RdmaRxQueuePair> q);
	// Hand one congestion observation to whichever controller is configured.
	// False means the queue pair is exempt and the controller never hears of
	// it, which is the whole of the exemption: no controller state moves,
	// so none of their behaviour is ours to defend.
	bool DeliverCongestionSignal(Ptr<RdmaQueuePair> qp);
	void ReceiveTrimmedData(const CustomHeader &ch, uint32_t payloadSize,
		bool lastHop);
	// The step stop's question, asked at every accepted arrival and answered
	// by the frontend, which holds the budget and the step's plan; the
	// question absorbs whatever the frontend grants.
	void AskRemainderOnArrival(Ptr<RdmaRxQueuePair> q);
	// The same question, asked because the frontend's budget says this sender
	// is done rather than because a packet arrived. A flow waiting on a repair
	// receives nothing, so waiting for its next arrival would leave it stopped
	// only by its timeout. False when this receiver holds no queue pair for
	// the five-tuple, which is a flow that has already finished.
	bool StopFlow(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport);
	void QpComplete(Ptr<RdmaQueuePair> qp);
	void QpFail(Ptr<RdmaQueuePair> qp, uint32_t reason);
	void ArmRetransmissionTimeout(Ptr<RdmaQueuePair> qp);
	void HandleRetransmissionTimeout(Ptr<RdmaQueuePair> qp);
	bool EnforceProgressDeadline(Ptr<RdmaQueuePair> qp);
	void SetLinkDown(Ptr<QbbNetDevice> dev);

	// call this function after the NIC is setup
	void AddTableEntry(Ipv4Address &dstAddr, uint32_t intf_idx);
	void ClearTable();
	void RedistributeQp();

	Ptr<Packet> GetNxtPacket(Ptr<RdmaQueuePair> qp); // get next packet to send, inc snd_nxt
	bool IsPathPerPacket() const;
	void SendPathProbe(Ptr<RdmaQueuePair> qp, uint16_t path);
	// Where every data packet draws its own path: keep the queue pair's send
	// records and give it the selector that draws its paths. bdpBytes and
	// baseRttNs are the queue pair's bandwidth-delay product and base round
	// trip, as AddQueuePair receives them.
	void StartPathSelection(Ptr<RdmaQueuePair> qp, uint64_t bdpBytes,
		uint64_t baseRttNs);
	void PktSent(Ptr<RdmaQueuePair> qp, Ptr<Packet> pkt, Time interframeGap);
	void UpdateNextAvail(Ptr<RdmaQueuePair> qp, Time interframeGap, uint32_t pkt_size);
	void ChangeRate(Ptr<RdmaQueuePair> qp, DataRate new_rate);
	/******************************
	 * Mellanox's version of DCQCN
	 *****************************/
	double m_g; //feedback weight
	double m_rateOnFirstCNP; // the fraction of line rate to set on first CNP
	bool m_EcnClampTgtRate;
	double m_rpgTimeReset;
	double m_rateDecreaseInterval;
	uint32_t m_rpgThreshold;
	double m_alpha_resume_interval;
	DataRate m_rai;		//< Rate of additive increase
	DataRate m_rhai;		//< Rate of hyper-additive increase

	// the Mellanox's version of alpha update:
	// every fixed time slot, update alpha.
	void UpdateAlphaMlx(Ptr<RdmaQueuePair> q);
	void ScheduleUpdateAlphaMlx(Ptr<RdmaQueuePair> q);

	// Mellanox's version of CNP receive
	void cnp_received_mlx(Ptr<RdmaQueuePair> q);

	// Mellanox's version of rate decrease
	// It checks every m_rateDecreaseInterval if CNP arrived (m_decrease_cnp_arrived).
	// If so, decrease rate, and reset all rate increase related things
	void CheckRateDecreaseMlx(Ptr<RdmaQueuePair> q);
	void ScheduleDecreaseRateMlx(Ptr<RdmaQueuePair> q, uint32_t delta);

	// Mellanox's version of rate increase
	void RateIncEventTimerMlx(Ptr<RdmaQueuePair> q);
	void RateIncEventMlx(Ptr<RdmaQueuePair> q);
	void FastRecoveryMlx(Ptr<RdmaQueuePair> q);
	void ActiveIncreaseMlx(Ptr<RdmaQueuePair> q);
	void HyperIncreaseMlx(Ptr<RdmaQueuePair> q);

	/***********************
	 * High Precision CC
	 ***********************/
	double m_targetUtil;
	double m_utilHigh;
	uint32_t m_miThresh;
	bool m_multipleRate;
	bool m_sampleFeedback; // only react to feedback every RTT, or qlen > 0
	void HandleAckHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);
	void UpdateRateHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react);
	void UpdateRateHpTest(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react);
	void FastReactHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);

	/**********************
	 * TIMELY
	 *********************/
	double m_tmly_alpha, m_tmly_beta;
	uint64_t m_tmly_TLow, m_tmly_THigh, m_tmly_minRtt;
	void HandleAckTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);
	void UpdateRateTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool us);
	void FastReactTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);

	/**********************
	 * DCTCP
	 *********************/
	DataRate m_dctcp_rai;
	void HandleAckDctcp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);

	/*********************
	 * HPCC-PINT
	 ********************/
	uint32_t pint_smpl_thresh;
	void SetPintSmplThresh(double p);
	void HandleAckHpPint(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);
	void UpdateRateHpPint(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react);

	/*********************
	 * NSCC
	 ********************/
	double m_nscc_target_qdelay;
	double m_nscc_max_window;
	double m_nscc_gamma;
	double m_nscc_max_md_jump;
	double m_nscc_fair_increase;
	double m_nscc_fast_increase_scale;
	double m_nscc_eta;
	double m_nscc_alpha;
	uint32_t m_nscc_qa_gate;
	double m_nscc_qa_threshold;
	uint32_t m_nscc_adjust_bytes;
	double m_nscc_adjust_period;
	NsccWindow::Parameters NsccParameters() const;
	// The acknowledgement of the send of seq along path, as AcknowledgePacket
	// takes it, with the window following what it says and the receiver's
	// Rcv_Cwnd_Pend.
	void HandleAckNscc(Ptr<RdmaQueuePair> qp, uint64_t seq, uint16_t path,
		bool marked, uint32_t rcvCwndPend);
	// The window the NIC enforces follows the controller's, as the rate does
	// under the rate-based modes.
	void ApplyNsccWindow(Ptr<RdmaQueuePair> qp);
	// The NsccWindow trace source: queue pair, old window, new window.
	typedef void (*NsccWindowTracedCallback)(Ptr<RdmaQueuePair> qp,
		uint64_t oldWindow, uint64_t newWindow);
	TracedCallback<Ptr<RdmaQueuePair>, uint64_t, uint64_t> m_traceNsccWindow;
};

} /* namespace ns3 */

#endif /* RDMA_HW_H */
