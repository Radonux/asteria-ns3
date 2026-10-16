#ifndef RDMA_HW_H
#define RDMA_HW_H

#include <ns3/rdma.h>
#include <ns3/rdma-queue-pair.h>
#include <ns3/node.h>
#include <ns3/custom-header.h>
#include <ns3/random-variable-stream.h>
#include <ns3/seq-ts-header.h>
#include "qbb-header.h"
#include <ns3/traced-callback.h>
#include "qbb-net-device.h"
#include "load-balancing.h"
#include "answer-route.h"
#include "nscc-window.h"
#include "receiver-hold.h"
#include "spine-grader.h"
#include "spine-redirect.h"
#include "switch-queue-state.h"
#include <map>
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
	uint16_t ScoreForRedirect(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch,
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
	// Zero leaves the trimmed fraction on the congestion thresholds.
	double m_sprayTrimThreshold1, m_sprayTrimThreshold2, m_sprayTrimThreshold3;
	// SpineGrader::Parameters::congestionGradeFloor.
	uint8_t m_sprayCongestionGradeFloor;
	// Zero counts no loss at the sender and leaves the loss fraction the
	// grader grades under SwitchQueue on the congestion thresholds.
	double m_sprayLossThreshold1, m_sprayLossThreshold2, m_sprayLossThreshold3;
	// SpineScores::Parameters::lossIntervalNs and lossHoldCleanSentinels.
	uint64_t m_sprayLossIntervalNs;
	uint32_t m_sprayLossHoldCleanSentinels;
	// The loss thresholds, unset while SprayLossThreshold1 is zero.
	std::optional<std::array<double, 3>> SprayLossThresholds() const;
	// Whether the sender's scores count sends and losses: under SprayPolicy
	// with the loss thresholds set, outside SwitchQueue, and under
	// PathSelector RepsHold.
	bool CountsSprayLoss() const;
	// Whether the loss counting has the send records it needs.
	bool ValidSprayLoss() const;
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
	// SpineScores::Parameters::heldExploration and sentinelIntervalNs.
	SpineScores::HeldExploration m_sprayHeldExploration;
	uint64_t m_sprayHeldSentinelIntervalNs;
	// Whether the held exploration suits the cost source: none sends a held
	// spine nothing, and only the switch_queue cost source releases one
	// without its arrivals.
	bool ValidHeldExploration() const;
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
	// With m_spraySenderPoolWeight above zero, the host's pooled scores,
	// which every queue pair's scores are mixed with (SpineScores::Pool),
	// made at the first queue pair; null otherwise.
	double m_spraySenderPoolWeight;
	// The pooled scores' SpineScores::Parameters::poolDecayIntervalNs.
	uint64_t m_sprayPoolDecayIntervalNs;
	// Every queue pair's SpineScores::Parameters::poolHoldPropagates, and
	// whether it is set only with a pool to propagate from.
	bool m_sprayPoolHoldPropagates;
	bool ValidPoolHoldPropagates() const;
	// The host's pooled scores, or under PathSelector RepsHold or MrcHold its
	// loss hold; null until the first queue pair that uses them.
	std::unique_ptr<SpineScores> m_senderSpineScores;
	// m_senderSpineScores, made with parameters if it is null, its loss
	// intervals traced to SpineLossInterval when they are graded.
	SpineScores &HostScores(const SpineScores::Parameters &parameters);
	// PathSelector RepsHold, RepsFlowHold or MrcHold, and whether its loss
	// hold has what it needs:
	// spines named per packet, the loss thresholds, a loss interval, a loss
	// hold and the sentinels that release it.
	bool IsRepsHold() const;
	bool IsMrcHold() const;
	bool ValidRepsHold() const;
	// The parameters of scores used as a loss hold alone.
	SpineScores::Parameters LossHoldParameters() const;
	// HoldRule of PathSelector RepsHold, RepsFlowHold and MrcHold, and under
	// Llr its LossLikelihoodRatio::Parameters; whether Llr is set only under
	// them, with a valid test.
	uint32_t m_sprayHoldRule;
	double m_sprayLossP0, m_sprayLossP1, m_sprayHoldAlpha, m_sprayHoldBeta;
	bool ValidHoldRule() const;
	LossLikelihoodRatio::Parameters Likelihood() const;
	// HoldRelease of PathSelector RepsHold, RepsFlowHold and MrcHold and its
	// trickle fraction; whether Trickle is set only under them, without the
	// keys of the sentinels' release, with a fraction in (0, 0.5] and the p0,
	// p1 and alpha TrickleRelease reads valid.
	uint32_t m_sprayHoldRelease;
	double m_sprayHoldTrickle;
	bool IsTrickleRelease() const;
	bool ValidHoldRelease() const;
	// SentinelMode of PathSelector RepsHold, RepsFlowHold and MrcHold.
	uint32_t m_spraySentinelMode;
	// CopyExpiry of their copies under SentinelMode Duplicate: under Deadline
	// every queue pair's m_copyDeadlineRtts is m_sprayTailLossRtts, set at
	// StartPathSelection; whether Deadline is set only with copies and a
	// guard above zero.
	uint32_t m_sprayCopyExpiry;
	bool ValidCopyExpiry() const;
	// Under PathSelector RepsHold, RepsFlowHold and MrcHold, the base RTTs
	// after which a send on a held spine (PathSelector::ChoseHeld) still
	// outstanding is declared lost and repaired (HeldSendDue); zero waits
	// for the retransmission timeout.
	double m_sprayHeldLossRtts;
	// SprayOverdueRepair, under the same selectors with m_sprayHeldLossRtts
	// above zero: the queue pair's latest send on a spine not held is
	// declared lost and repaired as a held send is once its answer is overdue
	// by m_sprayHeldLossRtts base RTTs, one repair per answer or timeout
	// (RdmaQueuePair::m_overdueTaken), its loss evidence as a timed-out
	// send's; and a receiver flags its answers to duplicates as under
	// SprayDuplicateReport, so that a flagged answer to a repaired sequence
	// counts the repair spurious.
	bool m_sprayOverdueRepair;
	bool ValidOverdueRepair() const;
	// SprayAckLossRtts, under LoadBalancing SprayUniform with any selector:
	// the queue pair's base RTTs after which a send the cumulative
	// acknowledgement covered before its own acknowledgement arrived, and
	// whose own acknowledgement has still not arrived, is declared lost on
	// its path for the selector alone (RdmaQueuePair::m_covered), as a lost
	// acknowledgement: no repair, no change to the retransmission timeout and
	// no congestion signal, the data having been delivered. Zero keeps no
	// covered sends.
	double m_sprayAckLossRtts;
	bool ValidAckLoss() const;
	// AckRoute, under LoadBalancing SprayUniform: the spine the switches route
	// each answer this host sends as a receiver over, an acknowledgement, a
	// NACK or a repair request. Under DataSpine it is the spine that carried
	// the data packet it answers; under Spray each answer to a sender off
	// this host's leaf (m_ownLeafHosts) names in its route_spine field a
	// spine drawn uniformly below m_spineCount from m_ackRouteRandom, a
	// stream of this host's own that nothing else draws from. The answer's
	// identification echoes the data packet's either way, and the sender
	// reads its spines from it alone.
	uint32_t m_ackRoute;
	Ptr<UniformRandomVariable> m_ackRouteRandom;
	// AckRoute ReverseSet and Repath keep each connection's answer routes,
	// per sending host and priority group, in m_answerRoutes: under
	// ReverseSet a set of AckRouteSetSize spines (AnswerRouteSet), probed
	// once per SprayBaseRttNs while the connection's data arrives, a member
	// replaced once a probe of it has been unanswered for
	// kAnswerSetProbeTimeoutRtts base RTTs; under Repath one spine
	// (AnswerRepath), drawn again at each data packet that adds nothing the
	// receiver did not hold and whose send was not copied, which the sender's
	// tail flags say (kTailCopied). Each probe is the host transport event
	// answer_set_probe, each replacement answer_set_replace and each redraw
	// answer_repath.
	static constexpr double kAnswerSetProbeTimeoutRtts = 2;
	uint32_t m_ackRouteSetSize;
	struct AnswerRoutes {
		AnswerRouteSet set;
		AnswerRepath repath;
	};
	std::map<std::pair<uint32_t, uint16_t>, AnswerRoutes> m_answerRoutes;
	AnswerRoutes &AnswerRoutesOf(uint32_t senderIp, uint16_t pg);
	uint32_t DrawAnswerRoute(uint32_t n);
	// Whether answers to senderIp cross a spine: it is off this host's leaf.
	bool AnswersCrossSpine(uint32_t senderIp) const;
	// Under ReverseSet, at a data arrival of q's connection: a round of
	// probes when one is due, after the replacements it makes.
	void ProbeAnswerSet(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch);
	// A probe of q's connection over spine, an acknowledgement flagged
	// FLAG_ROUTE_PROBE and routed over spine, which moves no state at its
	// sender.
	void SendRouteProbe(Ptr<RdmaRxQueuePair> q, uint8_t spine);
	// The sender's return of a route probe, a path probe flagged
	// FLAG_ROUTE_PROBE_REPLY back over the spine probed, whether or not the
	// queue pair it names is still open.
	void ReplyRouteProbe(const CustomHeader &ch);
	// Under Repath, a duplicate data arrival from senderIp: the connection's
	// route drawn again.
	void RepathAnswers(uint32_t senderIp, uint16_t pg);
	bool ValidAnswerRoutes() const;
	// AckRouteOracle, under AckRoute Spray: the draw leaves out the spines
	// over which an answer toward the sender's leaf crosses a fault as the
	// switches' state shows it (SwitchQueueState::ForwardFaulted), per sender
	// leaf in m_answerFaultedSpines, made at the first answer toward that
	// leaf and read once per base RTT, as PathSelector 8 reads its own.
	bool m_ackRouteOracle;
	std::unordered_map<uint32_t, FaultedSpines> m_answerFaultedSpines;
	bool ValidAckRoute() const;
	FaultedSpines &AnswerFaultedSpinesTo(uint32_t leaf);
	// The route_spine field of an answer to the host at senderIp, of
	// priority group pg: kNoRouteSpine under DataSpine and toward a host of
	// this leaf, one draw under Spray, and the connection's under ReverseSet
	// and Repath.
	uint16_t AnswerRouteSpine(uint32_t senderIp, uint16_t pg = 0);
	// SprayCoverageFirst, under LoadBalancing SprayUniform: a send the
	// queue pair's coverage shows delivered is an answer loss, not a loss on
	// its path (RdmaQueuePair::m_coverageFirst), each reported as the
	// transport event answer_loss_covered.
	bool m_sprayCoverageFirst;
	bool ValidCoverageFirst() const;
	void ReportCoveredAnswerLoss();
	// SprayDuplicateReport, under SprayCoverageFirst: a receiver flags its
	// answer to a data packet whose sequence it held already
	// (qbbHeader::FLAG_DUPLICATE), and a sender defers the selector's
	// evidence of the sends a retransmission timeout declares lost until the
	// answer to their resend (RdmaQueuePair::m_deferTimeoutEvidence): each
	// resolved as an answer loss is the host transport event
	// answer_loss_duplicate, each told as a data loss timeout_loss_data.
	// Under Retract (DuplicateReport) the timeout tells each loss at once
	// and the flagged answer takes it back (RdmaQueuePair::
	// m_retractTimeoutEvidence): each taken back is answer_loss_duplicate,
	// by the route spine the answer came back over as
	// answer_loss_duplicate_spine<k>, and timeout_retract_release where it
	// released the hold it began; each left a data loss timeout_loss_data.
	// Retract needs the host's AutoLossHold under HoldRule::Count, which
	// takes a loss back.
	uint32_t m_sprayDuplicateReport;
	bool ValidDuplicateReport() const;
	void ReportTimedOutResolved(bool duplicate);
	void ReportTimeoutLossRetracted(uint16_t route, bool released);
	// SprayAnswerPathSeq, under AckRoute Spray with SprayPathSeq and a loss
	// hold selector (answer-path-seq.h). As a receiver this host numbers
	// each queue pair's answers per route spine (AnswerSequencer), every
	// answer carrying its number and another route's count
	// (CustomHeader::ackAnswerSeqBytes 5), and leaves out of its draw the
	// spines the senders of the answer's sender leaf hold
	// (AnswerHoldReports, from their data packets' answer_hold byte,
	// CustomHeader::udpAnswerHoldBytes 1), but for one probe answer per held
	// spine per m_sprayAnswerProbeIntervalNs. As a sender it reads each
	// queue pair's numbered answers (AnswerSequenceTracker), the announced
	// ones waiting kAnswerGuardRtts of the queue pair's base RTT, and counts
	// the answers lost and received per route spine in the hold of the
	// receiver's leaf (AnswerRouteHold, in m_answerRouteHolds): over
	// m_sprayLossIntervalNs with the top loss threshold, and released after
	// m_sprayAnswerHoldClean clean answers in a row, which its data packets
	// to that leaf carry as their answer_hold bitmap. Each answer found lost
	// is the transport event answer_route_lost, each probe answer sent
	// answer_probe, and each change of either hold is traced (AnswerHold).
	static constexpr double kAnswerGuardRtts = 2;
	bool m_sprayAnswerPathSeq;
	uint64_t m_sprayAnswerProbeIntervalNs;
	uint32_t m_sprayAnswerHoldClean;
	std::unordered_map<uint32_t, AnswerRouteHold> m_answerRouteHolds;
	std::unique_ptr<AnswerHoldReports> m_answerHoldReports;
	bool ValidAnswerPathSeq() const;
	AnswerRouteHold &AnswerRouteHoldTo(uint32_t leaf);
	AnswerHoldReports &AnswerHoldReportsOf();
	// The number of q's next answer over route into header, under
	// SprayAnswerPathSeq for a route spine.
	void NumberAnswer(Ptr<RdmaRxQueuePair> q, qbbHeader &header, uint16_t route);
	// The numbered answer ch to qp read and its outcomes counted.
	void ReadAnswerSeq(Ptr<RdmaQueuePair> qp, const CustomHeader &ch);
	// The AnswerHold trace source: a hold changed, at this host as a sender
	// (receiverSide false) of the route spine toward the answers of leaf,
	// or as a receiver of the spine of its answers to leaf.
	typedef void (*AnswerHoldTracedCallback)(bool receiverSide, uint32_t leaf,
		uint32_t spine, bool held, uint64_t nowNs);
	TracedCallback<bool, uint32_t, uint32_t, bool, uint64_t> m_traceAnswerHold;
	// HoldScope of PathSelector RepsHold and MrcHold. Under Leaf the queue
	// pairs to the hosts of one leaf, which m_switchQueues names, share that
	// leaf's loss hold in m_leafHolds, made at the first of them; under Pair
	// the queue pairs to one destination host share its own there, keyed by
	// its address, so that the hold outlasts each queue pair as one queue
	// pair kept across the host's messages to it would; under Host every
	// queue pair shares m_senderSpineScores. Under Auto every queue pair
	// shares m_autoHold, made at the first of them, toward its destination's
	// leaf, which m_switchQueues names; m_sprayHoldAutoLeaves is its
	// AutoLossHold autoLeaves.
	uint32_t m_sprayHoldScope;
	uint32_t m_sprayHoldAutoLeaves;
	std::unordered_map<uint32_t, SpineScores> m_leafHolds;
	std::unique_ptr<AutoLossHold> m_autoHold;
	bool IsLeafHold() const;
	bool IsPairHold() const;
	bool IsAutoHold() const;
	AutoLossHold &AutoHold();
	// The loss hold of the queue pairs to destination under Host, Leaf and
	// Pair.
	SpineScores &DestinationHold(uint32_t destination);
	// PathSelector RepsOracle, whether it has the switches' state to read,
	// and per destination leaf the spines it excludes, made at the first
	// queue pair to that leaf and read once per base RTT over the hops
	// m_repsOracleHops (OracleHops) names; whether OracleHops Forward is set
	// only under RepsOracle.
	uint32_t m_repsOracleHops;
	bool IsRepsOracle() const;
	bool ValidRepsOracle() const;
	// PathSelector RepsSpine: REPS over spines under LoadBalancing 2.
	bool IsRepsSpine() const;
	std::unordered_map<uint32_t, FaultedSpines> m_faultedSpines;
	FaultedSpines &FaultedSpinesTo(uint32_t leaf);
	// The hold in m_leafHolds of key: a destination leaf, or under Pair a
	// destination host's address.
	SpineScores &LeafHold(uint32_t key);
	// The LeafSpineLossInterval trace source: a destination leaf's loss hold
	// closed a loss interval, or under Auto the host's did toward one
	// destination leaf, as SpineLossInterval passes it; under Pair a
	// destination host's did, passed with the host's address for the leaf.
	typedef void (*LeafSpineLossIntervalTracedCallback)(uint32_t leaf,
		uint64_t endNs, const std::vector<uint32_t> &sent,
		const std::vector<uint32_t> &lost, const std::vector<uint8_t> &grades);
	TracedCallback<uint32_t, uint64_t, const std::vector<uint32_t> &,
		const std::vector<uint32_t> &, const std::vector<uint8_t> &>
		m_traceLeafSpineLossInterval;
	// The RetransmissionTimeout trace source: a queue pair's timeout fired
	// and was taken, with the requested spine of its oldest outstanding send,
	// -1 where none is kept or the mode names no spine, whether that send was
	// an exploration send onto a held spine (OutstandingPackets::Exploration;
	// a copy under SentinelMode::Duplicate has no send record, so no timeout
	// waits for one), and the bytes outstanding.
	typedef void (*RetransmissionTimeoutTracedCallback)(uint64_t nowNs,
		Ptr<RdmaQueuePair> qp, int32_t spine, bool exploration,
		uint64_t bytesOutstanding);
	TracedCallback<uint64_t, Ptr<RdmaQueuePair>, int32_t, bool, uint64_t>
		m_traceRetransmissionTimeout;
	// The SpineLossInterval trace source: the host's pooled scores closed a
	// loss interval (SpineScores::LossIntervalCallback): the time it ended
	// at and, per spine, the sends and losses counted in it and the grade
	// its lost fraction earns.
	typedef void (*SpineLossIntervalTracedCallback)(uint64_t endNs,
		const std::vector<uint32_t> &sent, const std::vector<uint32_t> &lost,
		const std::vector<uint8_t> &grades);
	TracedCallback<uint64_t, const std::vector<uint32_t> &,
		const std::vector<uint32_t> &, const std::vector<uint8_t> &>
		m_traceSpineLossInterval;
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
	// SprayPathSeq, under LoadBalancing SprayUniform, whatever the selector:
	// every data packet carries its queue pair's path sequence on its
	// requested spine, and the receiver counts the packets the gaps in it
	// show missing, per carrying spine, in m_spineGaps
	// (SpineSequenceReceiver).
	bool m_sprayPathSeq;
	std::vector<uint64_t> m_spineGaps;
	bool ValidPathSeq() const;
	// The path sequence of the data packet ch arriving at q, or with trimmed
	// of its trimmed header, counted. Only data arrivals count gaps; a
	// trimmed header, which can overtake earlier data of its path in the
	// switches' high-priority queue, marks its sequence as accounted and
	// neither moves the expected sequence on nor counts a gap, and the
	// receiver's hold takes it for no arrival.
	void CountPathSeq(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch,
		bool trimmed = false);
	// SprayStrayPathSeq (StrayPathSeq), under SprayPathSeq. Under Closed the
	// frontend's removal of a receive queue pair (DeleteRxQp) leaves its key
	// in m_closedRxQps, with the removed queue pair while it has a tail run
	// waiting or has counted a path sequence missing, so that a packet of the
	// flow arriving after it, whose receive queue pair GetRxQp makes stray,
	// is read against it (ReadStrayPathSeq) and not by its own.
	uint32_t m_sprayStrayPathSeq;
	std::unordered_map<uint64_t, Ptr<RdmaRxQueuePair>> m_closedRxQps;
	bool ValidStrayPathSeq() const;
	// The path sequence of the data packet or trimmed header ch arriving at
	// the stray queue pair q, read against its flow's removed queue pair,
	// which is let go once it has neither a run waiting nor a count.
	void ReadStrayPathSeq(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch);
	// SprayTailRecovery, under SprayPathSeq: every data packet a queue pair
	// sends once it has no new data left (the last packet of its data and
	// every repair after it) is marked as closing its requested spine
	// (kTailCloses) and carries the count of the data packets sent so far up
	// each spine, CustomHeader::udpTailCounts of them. The receiver of such
	// a packet compares each count with the furthest path sequence it
	// expects of that requested spine, the arriving packet's own spine with
	// its path sequence, and records the path sequences missing, those its
	// trimmed headers accounted left out (SpineSequenceReceiver::TakeMissing). After TailRecoveryDelayNs() it
	// requests what of them is still missing, packets that arrived meanwhile
	// left out, in repair requests with qbbHeader::FLAG_TAIL_NACK, one per
	// stretch of consecutive path sequences of a spine. The sender declares
	// lost the outstanding sends those path sequences name on that spine,
	// as the per-path rule does, and repairs them; the retransmission
	// timeout is left as it is. A lost tail packet whose spine's count
	// arrives about half a base RTT after the message's end is repaired at
	// the receiver about 1.5 base RTT + the delay after that end.
	bool m_sprayTailRecovery;
	// SprayTailRecoveryDelayNs: how long the receiver waits on a missing run
	// before it requests it; kTailDelayBaseRtt, the default, is
	// m_sprayBaseRttNs, and zero requests it at once.
	static constexpr uint64_t kTailDelayBaseRtt = UINT64_MAX;
	uint64_t m_sprayTailRecoveryDelayNs;
	uint64_t TailRecoveryDelayNs() const;
	bool ValidTailRecovery() const;
	// SprayTailLossRtts, under SprayTailRecovery: the queue pair's base RTTs
	// after a tail repair request declares a send lost that the selector is
	// told of it, unless the send's trim notification or acknowledgement
	// arrives first (m_tailLosses); its repair is sent at once all the same. Zero tells the
	// selector at once.
	double m_sprayTailLossRtts;
	// Refused when set to other than its default without SprayTailRecovery.
	bool ValidTailLoss() const;
	// Under SprayTailLossRtts, the sends a tail repair request declared lost
	// whose loss the selector is not yet told, by the time each is due, in
	// the order they were declared among those due together. A request names
	// what the receiver has not seen, and the trimmed header of a send, or
	// the send itself queued longer than the receiver's delay, can reach the
	// receiver after the request for it: the trim notification of seq along
	// path for the queue pair, or the acknowledgement of seq along path, that
	// arrives before the send is due drops it (CancelTailLoss), as a trim is
	// no loss to the selector and an acknowledged send none at all. They are
	// the host's, not the queue pair's: the late trim notification or
	// acknowledgement can follow the queue pair's completion, which neither
	// tells nor drops them. Only its failure drops them (DropTailLosses).
	struct TailLoss {
		// The queue pair, by its key in m_qpMap (GetQpKey) and itself, since
		// a later queue pair can take the same key.
		uint64_t key;
		Ptr<RdmaQueuePair> qp;
		uint64_t seq;
		uint16_t path;
		bool trickle;
		// The receiver's address, by which a loss told after the queue pair
		// completed reaches the host's hold for it (TellTailLoss).
		uint32_t destination;
		// Whether a repair of seq left along path, so that an
		// acknowledgement of seq along path may be the repair's
		// (NoteTailRepair).
		bool repairedOnPath = false;
	};
	std::multimap<uint64_t, TailLoss> m_tailLosses;
	// One timer, for the earliest due.
	EventId m_tailLossTimer;
	// Sends in m_tailLosses dropped by their trim notifications, each
	// reported as the transport event tail_loss_cancelled.
	uint32_t m_tailLossesCancelled = 0;
	// Sends in m_tailLosses dropped by their acknowledgements, each reported
	// as the transport event tail_loss_delivered; and those of them a repair
	// of the same sequence along the same path had left before, whose
	// acknowledgement may be the repair's, each reported as
	// tail_loss_ambiguous instead.
	uint32_t m_tailLossesDelivered = 0;
	uint32_t m_tailLossesAmbiguous = 0;
	// What names a send in m_tailLosses as not lost.
	enum class TailLossEnd { Trimmed, Acknowledged };
	// The sends qp's tail repair request declared lost, due guardNs from now.
	void DeferTailLosses(Ptr<RdmaQueuePair> qp,
		const std::vector<RdmaQueuePair::DeclaredSend> &sends,
		uint64_t guardNs);
	// Whether a send of seq along path waits in m_tailLosses for qp, the
	// queue pair m_qpMap holds for key, or null when it holds none, or for a
	// completed queue pair of key; if so the earliest such is dropped,
	// counted and reported by end.
	bool CancelTailLoss(uint64_t key, Ptr<RdmaQueuePair> qp, uint64_t seq,
		uint16_t path, TailLossEnd end);
	// A repair of seq left for qp along path: each of qp's sends of seq
	// along path in m_tailLosses is marked repairedOnPath.
	void NoteTailRepair(Ptr<RdmaQueuePair> qp, uint64_t seq, uint16_t path);
	// The identification of a repair of seq for qp, which the selector chose
	// as identification at nowNs: while one of qp's sends of seq waits in
	// m_tailLosses along the path it names, the selector chooses again, at
	// most m_spineCount times, and the first choice along another path is
	// taken, so that the acknowledgement of the declared send cannot be taken
	// for the repair's. Each choice again counts in m_tailRepairRedraws, and
	// a repair so moved counts in m_tailRepairsRerouted and reports the
	// transport event tail_repair_rerouted. When every choice names a waiting
	// send's path, the last one is taken, and NoteTailRepair marks the loss.
	uint16_t RerouteTailRepair(Ptr<RdmaQueuePair> qp, uint64_t seq,
		uint16_t identification, uint64_t nowNs);
	// Whether one of qp's sends of seq along path waits in m_tailLosses.
	bool TailLossWaitsOn(Ptr<RdmaQueuePair> qp, uint64_t seq,
		uint16_t path) const;
	uint32_t m_tailRepairRedraws = 0;
	uint32_t m_tailRepairsRerouted = 0;
	// Drop qp's sends in m_tailLosses.
	void DropTailLosses(Ptr<RdmaQueuePair> qp);
	// The timer set for the earliest send in m_tailLosses; and the sends due
	// by now told, and the timer set for the next.
	void ArmTailLossTimer();
	void TailLossDue();
	// Tell the loss of a send in m_tailLosses at nowNs. A queue pair still
	// open, or under SprayPathState connection any queue pair, its selector
	// outliving it, tells its selector, as RdmaQueuePair::DeclareLost does;
	// otherwise, the queue pair having completed, the loss goes to the host's
	// hold for the receiver as LossHoldExploration::OnLoss routes it, under
	// RepsHold and MrcHold of every scope, and is dropped under every other
	// selector, whose state was the queue pair's alone, RepsFlowHold's hold
	// among them.
	void TellTailLoss(const TailLoss &loss, uint64_t nowNs);
	// How many of qp's sends wait in m_tailLosses.
	uint32_t TailLossesWaiting(Ptr<RdmaQueuePair> qp) const;
	// The runs of q due by now, requested, and q's timer set for the next.
	void RequestTailRuns(Ptr<RdmaRxQueuePair> q);
	// The tail fields of the data packet of qp being sent, closing when the
	// queue pair has no new data left.
	void SetTailFields(Ptr<RdmaQueuePair> qp, SeqTsHeader &seqTs,
		bool closing) const;
	// The tail repair requests the closing data packet or trimmed header ch
	// arriving at q asks for, before ch's own path sequence is counted.
	void AskTailMissing(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch);
	// The repair request of q's flow for count path sequences from first on
	// spine; under SprayReceiverTailMissing the receiver's hold then waits on
	// them (WaitTailMissing).
	void SendTailNack(Ptr<RdmaRxQueuePair> q, uint8_t spine, uint16_t first,
		uint16_t count);
	// The sender's handling of a tail repair request for qp.
	void RecoverTail(Ptr<RdmaQueuePair> qp, const CustomHeader &ch);
	// SprayReceiverHold, under SprayPathSeq: the host's ReceiverHold, made at
	// its first arrival, counts the arrivals and the missing packets per
	// sender leaf, which m_switchQueues names, holds spines by them with
	// SprayHoldAutoLeaves as its autoLeaves and m_sprayReceiverHoldClean as
	// its clean arrivals, and every acknowledgement carries the bitmap of
	// the spines it holds for its destination's leaf. As a sender, every
	// queue pair's selector holds the spines the bitmaps hold toward its
	// destination's leaf, in m_receiverHoldMarks, shared by the host's queue
	// pairs, or under PathSelector 7 in marks of its own.
	bool m_sprayReceiverHold;
	uint32_t m_sprayReceiverHoldClean;
	std::unique_ptr<ReceiverHold> m_receiverHold;
	std::shared_ptr<ReceiverHoldMarks> m_receiverHoldMarks;
	bool IsReceiverHold() const;
	bool ValidReceiverHold() const;
	ReceiverHold &ReceiverHoldOf();
	// The receiver_hold field of an answer to the host at senderIp.
	SpineBitmap ReceiverHoldBitmap(uint32_t senderIp) const;
	// Give qp's selector the marks it holds by.
	void AttachReceiverHold(Ptr<RdmaQueuePair> qp);
	// The ReceiverHoldInterval trace source: the host's ReceiverHold closed an
	// interval of SprayLossIntervalNs, once per sender leaf
	// (ReceiverHold::IntervalCallback), with each spine's ReceiverHold::Scope.
	typedef void (*ReceiverHoldIntervalTracedCallback)(uint32_t leaf,
		uint64_t endNs, const std::vector<uint32_t> &arrivals,
		const std::vector<uint32_t> &lost, const std::vector<uint8_t> &holds);
	TracedCallback<uint32_t, uint64_t, const std::vector<uint32_t> &,
		const std::vector<uint32_t> &, const std::vector<uint8_t> &>
		m_traceReceiverHoldInterval;
	// SprayReceiverTailMissing, under SprayReceiverHold and SprayTailRecovery:
	// the path sequences a tail repair request names that are still missing
	// m_sprayTailLossRtts base RTTs (m_sprayBaseRttNs) after the request are
	// counted missing by the receiver's hold, for the sender's leaf on their
	// requested spine (ReceiverHold::OnTailMissing). The gaps alone miss a
	// black-holed spine, where no data arrives to show one. The guard starts at
	// the request, as the sender's report of the sends the request declares
	// lost waits m_sprayTailLossRtts after the request reaches it: a send that
	// arrives late in that time cancels the report by its acknowledgement and
	// is left out here, so both ends count the same sends. A data packet or a
	// trimmed header of the run that arrives before the guard ends takes it
	// out, a trim being no loss; a sequence a data arrival passed meanwhile
	// was counted as a gap and is not counted again; and a sequence counted
	// and arriving later is a late arrival
	// (SpineSequenceReceiver::TakeTailMissing). Each send counted is reported
	// as the transport event receiver_tail_missing, and each of them whose
	// data packet or trimmed header arrives after all as
	// receiver_tail_missing_late (ReportTailMissing).
	bool m_sprayReceiverTailMissing;
	bool ValidReceiverTailMissing() const;
	// The run q requested of count path sequences from first on spine, to
	// be counted when its guard ends.
	void WaitTailMissing(Ptr<RdmaRxQueuePair> q, uint8_t spine, uint16_t first,
		uint16_t count);
	// The runs of q whose guard ended by now counted, and q's timer set for
	// the next.
	void CountTailMissing(Ptr<RdmaRxQueuePair> q);
	// The transport event event of a send requested up spine, and the same
	// with _spine and the spine appended, the transport summary's breakdown of
	// event by spine.
	void ReportTailMissing(const char *event, uint8_t spine);
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
	// add a new qp (new send); maxRate, where nonzero and below the NIC's
	// line rate, is the rate it is paced at
	void AddQueuePair(uint32_t src, uint32_t dest, uint64_t tag, uint64_t size, uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport, uint16_t _dport, uint32_t win, uint64_t baseRtt, Callback<void> notifyAppFinish, Callback<void> notifyAppSent, DataRate maxRate = DataRate(0));
	void DeleteQueuePair(Ptr<RdmaQueuePair> qp);

	Ptr<RdmaRxQueuePair> GetRxQp(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport, uint16_t pg, bool create); // get a rxQp
	uint32_t GetNicIdxOfRxQp(Ptr<RdmaRxQueuePair> q); // get the NIC index of the rxQp
	void DeleteRxQp(uint32_t dip, uint16_t pg, uint16_t dport);
	// The key of m_rxQpMap: the sender's address, priority group and port.
	static uint64_t GetRxQpKey(uint32_t dip, uint16_t pg, uint16_t dport);

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
	// A repair request carrying header, to destinationIp, with the IPv4
	// identification identification.
	void SendRepairRequest(const qbbHeader &header, uint32_t sourceIp,
		uint32_t destinationIp, uint16_t identification,
		Ptr<RdmaRxQueuePair> q = nullptr);
	// packetSeq and identification describe the data packet answered, or with
	// probeAnswer the probe; redirect is the redirect field, kNoRedirect
	// outside AckRedirect.
	void SendAck(Ptr<RdmaRxQueuePair> q, uint32_t sourceIp,
		uint32_t destinationIp, uint16_t sport, uint16_t dport, uint16_t pg,
		const IntHeader &ih, uint32_t packetSeq, uint16_t identification,
		bool nack, bool cnp, bool spent, bool probeAnswer, uint16_t redirect,
		bool duplicate = false);
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
	// The send of seq along path at sentNs, on a held spine, is due: declared
	// lost if still outstanding, and its repair sent.
	void HeldSendDue(Ptr<RdmaQueuePair> qp, uint64_t seq, uint16_t path,
		uint64_t sentNs);
	// Under SprayOverdueRepair: qp's send of seq along path at sentNs, on a
	// spine not held when held is false, taken as its latest send, unless one
	// was repaired since its last answer or timeout, and qp's timer set for
	// it unless it is set; for a held send, none is taken.
	void TakeOverdueSend(Ptr<RdmaQueuePair> qp, uint64_t seq, uint16_t path,
		uint64_t sentNs, bool held);
	// qp's timer: the send taken, once its answer is overdue, declared lost
	// if still outstanding, and its repair sent; the timer set again for a
	// send taken later.
	void OverdueSendDue(Ptr<RdmaQueuePair> qp);
	// Under SprayAckLossRtts: qp's timer set for its earliest covered send,
	// unless it is set; and the covered sends due by now declared lost on
	// their paths, and the timer set for the next.
	void ArmAckLossTimer(Ptr<RdmaQueuePair> qp);
	void AckLossDue(Ptr<RdmaQueuePair> qp);
	bool EnforceProgressDeadline(Ptr<RdmaQueuePair> qp);
	void SetLinkDown(Ptr<QbbNetDevice> dev);

	// call this function after the NIC is setup
	void AddTableEntry(Ipv4Address &dstAddr, uint32_t intf_idx);
	void ClearTable();
	void RedistributeQp();

	Ptr<Packet> GetNxtPacket(Ptr<RdmaQueuePair> qp); // get next packet to send, inc snd_nxt
	// The bytes of a data packet carrying payload bytes as a device receives
	// it, which a link's error model draws on: the PPP header, the IPv4
	// header, the data headers as CustomHeader sizes them for the run, and
	// the payload.
	static uint32_t DataFrameBytes(uint32_t payload);
	bool IsPathPerPacket() const;
	void SendPathProbe(Ptr<RdmaQueuePair> qp, uint16_t path);
	// Where every data packet draws its own path: keep the queue pair's send
	// records and give it the selector that draws its paths. bdpBytes and
	// baseRttNs are the queue pair's bandwidth-delay product and base round
	// trip, as AddQueuePair receives them. Under SprayPathState connection
	// the selector is its connection's, made by the first queue pair to the
	// destination and priority group in m_connectionSelectors with that queue
	// pair's bdpBytes and baseRttNs. Whether the selector is a new one.
	bool StartPathSelection(Ptr<RdmaQueuePair> qp, uint64_t bdpBytes,
		uint64_t baseRttNs);
	// The selector StartPathSelection gives a queue pair of its own, or makes
	// for a connection.
	std::unique_ptr<PathSelector> MakePathSelector(Ptr<RdmaQueuePair> qp,
		uint64_t bdpBytes, uint64_t baseRttNs);
	// SprayPathState, under LoadBalancing 2: PathState::Message gives every
	// queue pair a selector of its own, which ends with it. Under
	// PathState::Connection the queue pairs of the host to one destination
	// and priority group, each made for one message, share one selector
	// through SharedPathSelector, which m_connectionSelectors keeps past them
	// as a queue pair kept across the messages would keep it: what each kind
	// learns of its paths persists, REPS's cached values and freezing mode,
	// MRC's value states, rotation and probes, the loss hold exploration's
	// trickle schedule and the receivers' marks, RepsFlowHold's own hold, and
	// the redirect of AckRedirect. What belongs to one queue pair's packets
	// stays with it: its send records, copies, covered sends and path
	// sequences. The host's holds are as they are under every scope.
	uint32_t m_sprayPathState;
	std::unordered_map<uint64_t, std::shared_ptr<PathSelector>>
		m_connectionSelectors;
	bool IsConnectionPathState() const;
	bool ValidPathState() const;
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
	// NsccWindowState, under CC mode 11: NsccWindowState::Message opens a
	// window per queue pair, which ends with it. Under
	// NsccWindowState::Connection the queue pairs of the host to one
	// destination and priority group share the window of their connection in
	// m_nsccConnections, opened once at MaxWnd by the first of them with its
	// bandwidth-delay product and base RTT: it bounds the bytes they have in
	// flight together, every answer to any of them moves it, NSCC's state
	// with it, and it is kept past them, across idle periods as it stands,
	// since no rule for an idle window is taken from the specification. Each
	// queue pair keeps what belongs to its own packets: its send records,
	// repairs, timers and rate. The other modes keep their state per queue
	// pair and refuse it.
	uint32_t m_nsccWindowState;
	std::unordered_map<uint64_t, std::shared_ptr<NsccConnection>>
		m_nsccConnections;
	bool ValidNsccWindowState() const;
	// NsccRttSample, under CC mode 11 (NsccRttSample in nscc-window.h): the
	// round trip AnswerRtt gives NSCC for the answer to the outstanding send
	// packet of qp, or NsccWindow::kNoRtt. Any, the default, refuses only a
	// first resend after a declared loss; Karn refuses any sequence sent more
	// than once and floors the sample at qp's base RTT.
	uint32_t m_nsccRttSample;
	bool ValidNsccRttSample() const;
	uint64_t AnswerRtt(Ptr<RdmaQueuePair> qp, uint64_t packet) const;
	// The window of qp, its own or its connection's, opened, and qp's window
	// set from it; bdpBytes and baseRttNs as AddQueuePair receives them.
	void StartNscc(Ptr<RdmaQueuePair> qp, uint64_t bdpBytes,
		uint64_t baseRttNs);
	// qp, completed or failed, no longer in flight on its connection.
	void LeaveNsccConnection(Ptr<RdmaQueuePair> qp);
	// The acknowledgement of the send of seq along path, as AcknowledgePacket
	// takes it, with the window following what it says and the receiver's
	// Rcv_Cwnd_Pend.
	void HandleAckNscc(Ptr<RdmaQueuePair> qp, uint64_t seq, uint16_t path,
		bool marked, uint32_t rcvCwndPend);
	// The window the NIC enforces follows the controller's, as the rate does
	// under the rate-based modes: qp's, or every queue pair's open on qp's
	// connection.
	void ApplyNsccWindow(Ptr<RdmaQueuePair> qp);
	// The NsccWindow trace source: queue pair, old window, new window.
	typedef void (*NsccWindowTracedCallback)(Ptr<RdmaQueuePair> qp,
		uint64_t oldWindow, uint64_t newWindow);
	TracedCallback<Ptr<RdmaQueuePair>, uint64_t, uint64_t> m_traceNsccWindow;
};

} /* namespace ns3 */

#endif /* RDMA_HW_H */
