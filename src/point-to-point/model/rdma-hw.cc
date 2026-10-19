#include <ns3/simulator.h>
#include <ns3/seq-ts-header.h>
#include <ns3/udp-header.h>
#include <ns3/ipv4-header.h>
#include "ns3/ppp-header.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/data-rate.h"
#include "ns3/pointer.h"
#include "ns3/abort.h"
#include "rdma-hw.h"
#include "ppp-header.h"
#include "qbb-header.h"
#include "cn-header.h"
#include "spine-redirect.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace ns3{

namespace {

static_assert(sizeof(std::declval<CustomHeader>().ack.spine_report) ==
		1 + SpineReport::GradeBytes(SpineReport::kMaxSpines),
	"an acknowledgement header holds the largest spine report");

// The spine report an acknowledgement or a repair request carries.
SpineReport ReportOf(const CustomHeader &ch){
	SpineReport report;
	report.sequence = ch.ack.spine_report[0];
	std::copy_n(ch.ack.spine_report + 1, CustomHeader::ackReportBytes - 1,
		report.grades);
	report.edgeCongested = (ch.ack.flags >> qbbHeader::FLAG_EDGE_CONGESTED) & 1;
	return report;
}

// A fraction of the golden ratio per node, which sets the hosts' phases apart
// for any number of hosts.
double PhaseOf(uint32_t node){
	return std::fmod(node * 0.6180339887498949, 1.0);
}

} // namespace

TypeId RdmaHw::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RdmaHw")
		.SetParent<Object> ()
		.AddAttribute("MinRate",
				"Minimum rate of a throttled flow",
				DataRateValue(DataRate("100Mb/s")),
				MakeDataRateAccessor(&RdmaHw::m_minRate),
				MakeDataRateChecker())
		.AddAttribute("Mtu",
				"Mtu.",
				UintegerValue(1000),
				MakeUintegerAccessor(&RdmaHw::m_mtu),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute ("CcMode",
				"which mode of DCQCN is running",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_cc_mode),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("NACK_Generation_Interval",
				"The NACK Generation interval",
				DoubleValue(500.0),
				MakeDoubleAccessor(&RdmaHw::m_nack_interval),
				MakeDoubleChecker<double>())
		.AddAttribute("L2ChunkSize",
				"Layer 2 chunk size. Disable chunk mode if equals to 0.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_chunk),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("L2AckInterval",
				"Layer 2 Ack intervals. Disable ack if equals to 0.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_ack_interval),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("L2BackToZero",
				"Layer 2 go back to zero transmission.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_backto0),
				MakeBooleanChecker())
		.AddAttribute("RetransmissionTimeoutNs",
				"Zero disables timeout recovery; otherwise retry unacknowledged data after this interval.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_retransmission_timeout_ns),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("MaxRetransmissionRetries",
				"Number of bounded recovery attempts allowed before a QP fails explicitly.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_max_retransmission_retries),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("NoProgressTimeoutNs",
				"Zero disables the deadline; otherwise fail a queue pair whose "
				"cumulative acknowledgement has not advanced for this simulated "
				"interval. Recovery signals such as NACKs and trim notifications "
				"prove the path is alive and are exempt from the retry budget, "
				"but they are not evidence the transfer is progressing; this "
				"deadline bounds the transfer itself.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_no_progress_timeout_ns),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("SelectiveRetransmission",
				"Retransmit exactly the reported trimmed or missing byte ranges "
				"and accept out-of-order payload at the receiver, instead of "
				"go-back-N. Timeout recovery stays go-back-N as the fallback.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_selective_retransmission),
				MakeBooleanChecker())
		.AddAttribute("Forgiveness",
				"Ask the recovery-verdict callback what to do with a trimmed "
				"range instead of always pulling it. Off reproduces the "
				"pull-everything transport exactly. Requires "
				"SelectiveRetransmission: a forgiven range is absorbed as an "
				"accepted out-of-order range.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_forgiveness),
				MakeBooleanChecker())
		.AddAttribute("CongestionExemption",
				"Ask the congestion-exemption callback, once per queue pair, "
				"whether that queue pair may ignore every congestion signal "
				"until the receiver refuses to forgive one of its trims. Off "
				"leaves every queue pair obeying congestion control. Requires "
				"Forgiveness: refusal is what ends the exemption, and a "
				"transport that never forgives never refuses.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_congestionExemption),
				MakeBooleanChecker())
		.AddAttribute("Reengage",
				"Whether the receiver's report that a step's budget is spent "
				"ends that queue pair's congestion exemption. False is the "
				"reference arm: the report is still carried and counted, and "
				"the exemption stands, so the budget alone bounds the loss.",
				BooleanValue(true),
				MakeBooleanAccessor(&RdmaHw::m_reengage),
				MakeBooleanChecker())
		.AddAttribute("LoadBalancing",
				"What the IPv4 identification of a data packet carries: 0=a "
				"per-QP counter, 1=a 16-bit entropy value drawn per packet, "
				"2=a spine index drawn uniformly per packet, in both bytes, "
				"3=a spine index drawn per packet from scores built out of the "
				"receiver's grades, in both bytes.",
				UintegerValue(static_cast<uint32_t>(LoadBalancingMode::Ecmp)),
				MakeUintegerAccessor(&RdmaHw::m_loadBalancing),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(LoadBalancingMode::Ecmp),
					static_cast<uint32_t>(LoadBalancingMode::SprayPolicy)))
		.AddAttribute("SpineCount",
				"Number of spines a data packet may name when LoadBalancing is 2 "
				"or 3.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_spineCount),
				MakeUintegerChecker<uint32_t>(0, 256))
		.AddAttribute("PathRandomVariable",
				"The variable the identification is drawn from when LoadBalancing "
				"is not 0. Null by default, so that ECMP allocates no stream.",
				PointerValue(),
				MakePointerAccessor(&RdmaHw::m_pathRandom),
				MakePointerChecker<UniformRandomVariable>())
		.AddAttribute("PathSelector",
				"Under LoadBalancing 1, what chooses each data packet's entropy "
				"value: 0=OPS, a fresh value drawn per packet; 1=REPS; 2=UEC "
				"oblivious spraying; 3=UEC path-aware spraying with the "
				"congestion bitmap; 4=MRC. Under LoadBalancing 2, 0 draws each "
				"spine uniformly, 5 sends on the spine the latest "
				"acknowledgement redirects to, which needs IntHeader::mode TS, "
				"6 is REPS over spines leaving out the spines the host's "
				"loss hold holds, 7 as 6 with a loss hold per queue pair, and 8 "
				"is REPS over spines leaving out for good the spines "
				"SwitchQueueState shows a fault on toward the destination's "
				"leaf over RepsOracleHops' hops, which needs "
				"SprayGradeCostSource 1, and 9 is MRC over "
				"spines leaving out the spines 6's loss hold holds, and 10 is "
				"REPS over spines leaving out none.",
				UintegerValue(static_cast<uint32_t>(PathSelectorKind::Ops)),
				MakeUintegerAccessor(&RdmaHw::m_pathSelectorKind),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(PathSelectorKind::Ops),
					static_cast<uint32_t>(PathSelectorKind::RepsSpine)))
		.AddAttribute("RepsBufferSize",
				"REPS: the entries of the circular buffer of entropy values to "
				"reuse. Default 8, REPS section 3.1.",
				UintegerValue(8),
				MakeUintegerAccessor(&RdmaHw::m_repsBufferSize),
				MakeUintegerChecker<uint32_t>(1, UINT8_MAX))
		.AddAttribute("RepsFreezingTimeoutNs",
				"REPS: FREEZING_TIMEOUT, how long freezing mode lasts at least. "
				"Default 10 ms, the REPS artifact's exit_freeze_after.",
				UintegerValue(10000000),
				MakeUintegerAccessor(&RdmaHw::m_repsFreezingTimeoutNs),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("RepsOracleHops",
				"Under PathSelector 8, the hops over which a spine toward the "
				"destination's leaf is read as faulted: 0=the data's two hops "
				"and the two its acknowledgement returns over through the same "
				"spine, the destination leaf's uplink to it and its downlink to "
				"the source's leaf; 1=the data's two hops alone, the source "
				"leaf's uplink to the spine and its downlink to the "
				"destination's leaf. Default 0.",
				UintegerValue(static_cast<uint32_t>(OracleHops::RoundTrip)),
				MakeUintegerAccessor(&RdmaHw::m_repsOracleHops),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(OracleHops::RoundTrip),
					static_cast<uint32_t>(OracleHops::Forward)))
		.AddAttribute("UeEvSetSize",
				"UEC spraying: the entropy values a queue pair sprays over. "
				"Default 256, UEC 1.0.3 section 3.6.16.3's typical size.",
				UintegerValue(256),
				MakeUintegerAccessor(&RdmaHw::m_ueEvSetSize),
				MakeUintegerChecker<uint32_t>(1, UINT16_MAX + 1))
		.AddAttribute("UeSaturationFraction",
				"UEC path-aware spraying: with more than this fraction of the "
				"values marked congested, none is skipped. Default 0.5, UEC "
				"1.0.3 section 3.6.16.4.",
				DoubleValue(0.5),
				MakeDoubleAccessor(&RdmaHw::m_ueSaturationFraction),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("MrcEvSetSize",
				"MRC: the entropy values a queue pair rotates through. Default "
				"128, within the one to two windows of OCP MRC 1.0 section "
				"11.2.1 and the 128 to 256 per queue pair MRC is deployed with.",
				UintegerValue(128),
				MakeUintegerAccessor(&RdmaHw::m_mrcEvSetSize),
				MakeUintegerChecker<uint32_t>(1, UINT16_MAX + 1))
		.AddAttribute("MrcSkipBaseRtts",
				"MRC: how long a value stays SKIP unless the rotation resets it "
				"first, in base RTTs. OCP MRC 1.0 leaves it to the "
				"implementation; default 1, the round trip UEC 1.0.3 section "
				"3.6.16.4 keeps a marked value out of use for.",
				DoubleValue(1.0),
				MakeDoubleAccessor(&RdmaHw::m_mrcSkipBaseRtts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("MrcProbeTimeouts",
				"MRC: the interval between probes of an ASSUMED_BAD value, in "
				"retransmission timeouts. OCP MRC 1.0 leaves it to the "
				"implementation; default 1, so that a value is tried again as "
				"often as the timeout can condemn it.",
				DoubleValue(1.0),
				MakeDoubleAccessor(&RdmaHw::m_mrcProbeTimeouts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("AckRedirectOmega",
				"Under PathSelector 5, the weight of the one-way delay term of an "
				"arrival's score against its ECN term. Default 0.5.",
				DoubleValue(0.5),
				MakeDoubleAccessor(&RdmaHw::m_ackRedirectOmega),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("AckRedirectFlag",
				"Under PathSelector 5, the fraction of the best spine's score an "
				"arrival's may fall short of before its acknowledgement "
				"redirects the sender to the best spine. Default 0.1.",
				DoubleValue(0.1),
				MakeDoubleAccessor(&RdmaHw::m_ackRedirectFlag),
				MakeDoubleChecker<double>(0))
		.AddAttribute("AckRedirectValidityBaseRtts",
				"Under PathSelector 5, how long a receive queue pair keeps its "
				"best spine before the next arrival starts a new cycle, in base "
				"RTTs. Default 2.",
				DoubleValue(2.0),
				MakeDoubleAccessor(&RdmaHw::m_ackRedirectValidityBaseRtts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("AckRedirectTrimScoring",
				"Under PathSelector 5, how a header trimmed before the last hop "
				"is scored: 0=as an arrival on its carrying spine marked with "
				"CE, 1=not at all. A last-hop trim is never scored. Default 0.",
				UintegerValue(static_cast<uint32_t>(TrimScoring::Mark)),
				MakeUintegerAccessor(&RdmaHw::m_ackRedirectTrimScoring),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(TrimScoring::Mark),
					static_cast<uint32_t>(TrimScoring::Ignore)))
		.AddAttribute("AckRedirectScope",
				"Under PathSelector 5, what an arrival is scored against: 1=its "
				"receive queue pair's own best spine, scored per arrival; 0=the "
				"receiving host's best spine, scored at each close of the host's "
				"spine grader's interval from its per-spine averages, which "
				"takes the grader's Spray attributes and none of "
				"AckRedirectValidityBaseRtts and AckRedirectTrimScoring. "
				"Default 1.",
				UintegerValue(static_cast<uint32_t>(GradeScope::PerConnection)),
				MakeUintegerAccessor(&RdmaHw::m_ackRedirectScope),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(GradeScope::PerReceiver),
					static_cast<uint32_t>(GradeScope::PerConnection)))
		.AddAttribute("SprayBaseRttNs",
				"Under LoadBalancing 3 and PathSelector 5, the base RTT the "
				"attributes count time in, and under SprayTailRecovery the "
				"default of SprayTailRecoveryDelayNs. Zero, the default, is refused at the "
				"first use.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_sprayBaseRttNs),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("SprayReportIntervalBaseRtts",
				"Under LoadBalancing 3, the interval a receiver issues a spine "
				"report per, in base RTTs. Default 2: one report per round trip, "
				"which under load is about twice the base RTT.",
				DoubleValue(2.0),
				MakeDoubleAccessor(&RdmaHw::m_sprayReportIntervalBaseRtts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("SprayEstimatorGain",
				"Under LoadBalancing 3, the gain of a receiver's per-spine "
				"averages, per report interval. Default 1/16, DCTCP's gain on the "
				"fraction of marked packets.",
				DoubleValue(1.0 / 16),
				MakeDoubleAccessor(&RdmaHw::m_sprayEstimatorGain),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayEstimatorIntervalSamples",
				"Under LoadBalancing 3, the packets from which an interval's mark "
				"fraction and mean delay replace a spine's averages instead of "
				"moving them by the gain; zero never replaces them. Default 32.",
				UintegerValue(32),
				MakeUintegerAccessor(&RdmaHw::m_sprayEstimatorIntervalSamples),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("SprayFractionCusumSlack",
				"Under LoadBalancing 3, the CUSUM slack of the marked and the "
				"trimmed fraction, per report interval. Default 0.125, half the "
				"width of a grade.",
				DoubleValue(0.125),
				MakeDoubleAccessor(&RdmaHw::m_sprayFractionCusumSlack),
				MakeDoubleChecker<double>(0))
		.AddAttribute("SprayFractionCusumThreshold",
				"Under LoadBalancing 3, the CUSUM sum of the marked or the trimmed "
				"fraction at which its average restarts at the sample. Default "
				"0.5, the width of two grades.",
				DoubleValue(0.5),
				MakeDoubleAccessor(&RdmaHw::m_sprayFractionCusumThreshold),
				MakeDoubleChecker<double>(0))
		.AddAttribute("SprayCongestionThreshold1",
				"Under LoadBalancing 3, the congestion cost, the larger of the "
				"marked and the trimmed fraction's, from which a spine grades 2. "
				"Default 0.25.",
				DoubleValue(0.25),
				MakeDoubleAccessor(&RdmaHw::m_sprayCongestionThreshold1),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayCongestionThreshold2",
				"Under LoadBalancing 3, the congestion cost from which a spine "
				"grades 1. Default 0.5.",
				DoubleValue(0.5),
				MakeDoubleAccessor(&RdmaHw::m_sprayCongestionThreshold2),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayCongestionThreshold3",
				"Under LoadBalancing 3, the congestion cost from which a spine "
				"grades 0. Default 0.75.",
				DoubleValue(0.75),
				MakeDoubleAccessor(&RdmaHw::m_sprayCongestionThreshold3),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayTrimThreshold1",
				"Under LoadBalancing 3, the trimmed fraction's cost from which a "
				"spine grades 2, in place of SprayCongestionThreshold1 for that "
				"fraction alone. Default 0, which takes the congestion "
				"thresholds for the trimmed fraction; the three are set "
				"together.",
				DoubleValue(0),
				MakeDoubleAccessor(&RdmaHw::m_sprayTrimThreshold1),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayTrimThreshold2",
				"Under LoadBalancing 3, the trimmed fraction's cost from which a "
				"spine grades 1. Default 0, as SprayTrimThreshold1.",
				DoubleValue(0),
				MakeDoubleAccessor(&RdmaHw::m_sprayTrimThreshold2),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayTrimThreshold3",
				"Under LoadBalancing 3, the trimmed fraction's cost from which a "
				"spine grades 0. Default 0, as SprayTrimThreshold1.",
				DoubleValue(0),
				MakeDoubleAccessor(&RdmaHw::m_sprayTrimThreshold3),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayCongestionGradeFloor",
				"Under LoadBalancing 3, the least grade the receiver gives a "
				"spine for its marked and trimmed fractions and its one-way "
				"delay, before the report is sent. Default 0, which leaves them "
				"their grade; above zero only the loss grade and a held spine "
				"grade a spine below it.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_sprayCongestionGradeFloor),
				MakeUintegerChecker<uint8_t>(0, SpineReport::kTopGrade))
		.AddAttribute("SprayLossThreshold1",
				"Under LoadBalancing 3, the fraction of a spine's sends declared "
				"lost from which the sender grades it 2, depositing the lower of "
				"that grade and the report's; under SprayGradeCostSource 1, the "
				"loss fraction's cost from which the grader grades a spine 2. "
				"Default 0, which counts no loss at the sender and grades the "
				"loss fraction through the congestion thresholds; the three are "
				"set together.",
				DoubleValue(0),
				MakeDoubleAccessor(&RdmaHw::m_sprayLossThreshold1),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayLossThreshold2",
				"Under LoadBalancing 3, the lost fraction or cost from which a "
				"spine grades 1. Default 0, as SprayLossThreshold1.",
				DoubleValue(0),
				MakeDoubleAccessor(&RdmaHw::m_sprayLossThreshold2),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayLossThreshold3",
				"Under LoadBalancing 3, the lost fraction or cost from which a "
				"spine grades 0. Default 0, as SprayLossThreshold1.",
				DoubleValue(0),
				MakeDoubleAccessor(&RdmaHw::m_sprayLossThreshold3),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayLossIntervalNs",
				"Under LoadBalancing 3 with SprayLossThreshold1 set and "
				"SpraySenderPoolWeight above zero, the interval over which the "
				"host's pooled scores count the sends and losses of all its "
				"queue pairs and grade the spines by them. Default 0, the report "
				"interval.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_sprayLossIntervalNs),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("SprayLossHoldCleanSentinels",
				"Under LoadBalancing 3 with SprayLossThreshold1 set, the clean "
				"sends a spine whose lost fraction graded it 0 needs, counted "
				"across report periods and loss intervals and restarted by any "
				"loss, before its loss grade leaves 0; per destination and in "
				"the host's pooled scores alike. Default 0: no loss hold.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_sprayLossHoldCleanSentinels),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("SprayAbsenceFractionOfMedian",
				"Under LoadBalancing 3, a spine whose arrivals over a window are "
				"below this fraction of the median spine's is held at grade 0, "
				"as one a packet requested on was moved off is. A window closes "
				"once the median is at least SprayAbsenceMinimumMedian. Zero "
				"never holds one for its arrivals. Default 1/8.",
				DoubleValue(0.125),
				MakeDoubleAccessor(&RdmaHw::m_sprayAbsenceFractionOfMedian),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayAbsenceMinimumMedian",
				"Under LoadBalancing 3, the median arrivals per spine at which a "
				"window of intervals closes and every spine is judged. Default "
				"16.",
				UintegerValue(16),
				MakeUintegerAccessor(&RdmaHw::m_sprayAbsenceMinimumMedian),
				MakeUintegerChecker<uint32_t>(1))
		.AddAttribute("SprayReleaseFractionOfFloor",
				"Under LoadBalancing 3, a held spine whose arrivals over a window "
				"reach this fraction of its exploration floor's, SprayEpsilon of "
				"the median spine's, is released. Default 1/2.",
				DoubleValue(0.5),
				MakeDoubleAccessor(&RdmaHw::m_sprayReleaseFractionOfFloor),
				MakeDoubleChecker<double>(0))
		.AddAttribute("SprayOneWayDelay",
				"Under LoadBalancing 3, grade a spine with no marks by the one-way "
				"delay of its packets, which needs IntHeader::mode TS: eight bytes "
				"more in every data packet and acknowledgement. Default false.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_sprayOneWayDelay),
				MakeBooleanChecker())
		.AddAttribute("SprayDelayCusumSlackBaseRtts",
				"Under SprayOneWayDelay, the CUSUM slack of a spine's delay above "
				"its least, in base RTTs. Default 0.125, half the width of a "
				"grade.",
				DoubleValue(0.125),
				MakeDoubleAccessor(&RdmaHw::m_sprayDelayCusumSlackBaseRtts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("SprayDelayCusumThresholdBaseRtts",
				"Under SprayOneWayDelay, the CUSUM sum of a spine's delay at which "
				"its average restarts at the sample, in base RTTs. Default 0.5, "
				"the width of two grades.",
				DoubleValue(0.5),
				MakeDoubleAccessor(&RdmaHw::m_sprayDelayCusumThresholdBaseRtts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("SprayDelayThreshold1BaseRtts",
				"Under SprayOneWayDelay, the average delay above a spine's least "
				"from which it grades 2, in base RTTs. Default 0.25.",
				DoubleValue(0.25),
				MakeDoubleAccessor(&RdmaHw::m_sprayDelayThreshold1BaseRtts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("SprayDelayThreshold2BaseRtts",
				"Under SprayOneWayDelay, the average delay above a spine's least "
				"from which it grades 1, in base RTTs. Default 0.5.",
				DoubleValue(0.5),
				MakeDoubleAccessor(&RdmaHw::m_sprayDelayThreshold2BaseRtts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("SprayDelayThreshold3BaseRtts",
				"Under SprayOneWayDelay, the average delay above a spine's least "
				"from which it grades 0, in base RTTs. Default 0.75, NSCC's "
				"target_qdelay.",
				DoubleValue(0.75),
				MakeDoubleAccessor(&RdmaHw::m_sprayDelayThreshold3BaseRtts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("SprayGradeReference",
				"Under LoadBalancing 3, what a spine's averages are graded "
				"against: 0=nothing, the averages themselves; 1=the median of "
				"the spines' averages, which the averages' excess is graded by. "
				"Default 1.",
				UintegerValue(static_cast<uint32_t>(
					SpineGrader::GradeReference::Median)),
				MakeUintegerAccessor(&RdmaHw::m_sprayGradeReference),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(SpineGrader::GradeReference::Absolute),
					static_cast<uint32_t>(SpineGrader::GradeReference::Median)))
		.AddAttribute("SprayGradeScope",
				"Under LoadBalancing 3, whose arrivals one grader counts: 0=every "
				"connection into the receiving host, whose acknowledgements all "
				"carry its report and whose senders share one set of scores per "
				"destination host; 1=each receive queue pair alone, reported on "
				"its own acknowledgements and scored by its own sending queue "
				"pair. Default 0.",
				UintegerValue(static_cast<uint32_t>(GradeScope::PerReceiver)),
				MakeUintegerAccessor(&RdmaHw::m_sprayGradeScope),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(GradeScope::PerReceiver),
					static_cast<uint32_t>(GradeScope::PerConnection)))
		.AddAttribute("SprayGradeCostSource",
				"Under LoadBalancing 3 and SprayGradeScope 0, where the values "
				"the spines are graded on come from: 0=the receiver's estimate "
				"from its arrivals; 1=simulation only, the switches' own state "
				"at each close of the grader's interval, read through "
				"SwitchQueueState once per source leaf, with the absence rule's "
				"deficit replaced by the paths' configured loss. Default 0.",
				UintegerValue(static_cast<uint32_t>(GradeCostSource::Estimate)),
				MakeUintegerAccessor(&RdmaHw::m_sprayGradeCostSource),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(GradeCostSource::Estimate),
					static_cast<uint32_t>(GradeCostSource::SwitchQueue)))
		.AddAttribute("SprayGamma",
				"Under LoadBalancing 3, the fraction of every spine score a "
				"report decays. Default 0.25.",
				DoubleValue(0.25),
				MakeDoubleAccessor(&RdmaHw::m_sprayGamma),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayEpsilon",
				"Under LoadBalancing 3, the fraction of draws spread evenly over "
				"the spines whatever their scores. Default 0.02.",
				DoubleValue(0.02),
				MakeDoubleAccessor(&RdmaHw::m_sprayEpsilon),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayHeldExploration",
				"Under LoadBalancing 3, what a spine the last report graded 0 "
				"is sent: epsilon=its SprayEpsilon / N share of the draws; "
				"sentinel=no share of the epsilon term, which the other spines "
				"split, and one packet per SprayHeldSentinelIntervalNs; "
				"none=no share and no packet, which needs SprayGradeCostSource "
				"1, whose hold releases from the switches' state. Default "
				"epsilon.",
				EnumValue(SpineScores::HeldExploration::Epsilon),
				MakeEnumAccessor<SpineScores::HeldExploration>(
					&RdmaHw::m_sprayHeldExploration),
				MakeEnumChecker(SpineScores::HeldExploration::Epsilon, "epsilon",
					SpineScores::HeldExploration::Sentinel, "sentinel",
					SpineScores::HeldExploration::None, "none"))
		.AddAttribute("SprayHeldSentinelIntervalNs",
				"Under SprayHeldExploration sentinel, the interval between two "
				"packets sent on one held spine by one set of scores. Default "
				"0, the report interval.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_sprayHeldSentinelIntervalNs),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("SprayHeldSentinelSpread",
				"Under SprayHeldExploration sentinel, which send takes a "
				"sentinel once it is due, the loss hold's and the receivers' "
				"hold's alike: one send in this many on average, by a hash of "
				"the send's time and the node, so that hosts sending in step "
				"take theirs on different sends. Default 1, the first send.",
				UintegerValue(1),
				MakeUintegerAccessor(&RdmaHw::m_sprayHeldSentinelSpread),
				MakeUintegerChecker<uint32_t>(1))
		.AddAttribute("SprayHoldScope",
				"Under PathSelector 6 and 9, what one loss hold is kept for: 0=every "
				"destination of the host, 1=the destinations on one leaf, each "
				"leaf's hold counting the sends and losses of the queue pairs "
				"to it and holding a spine for them alone, 2=per spine either, "
				"the host's every destination once SprayHoldAutoLeaves "
				"destination leaves have a loss on it, else the destination "
				"leaf it is graded 0 toward (AutoLossHold), 3=one destination "
				"host, its hold counting and holding as a leaf's does under 1 "
				"and outlasting the queue pairs to it. Default 2. Leaf and "
				"auto need the hosts' leaves, from SwitchQueueState.",
				UintegerValue(static_cast<uint32_t>(HoldScope::Auto)),
				MakeUintegerAccessor(&RdmaHw::m_sprayHoldScope),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(HoldScope::Host),
					static_cast<uint32_t>(HoldScope::Pair)))
		.AddAttribute("SprayHoldAutoLeaves",
				"Under PathSelector 6 and 9 with SprayHoldScope 2, the destination "
				"leaves with a loss on a spine from which its hold is kept "
				"for every destination. Default 2.",
				UintegerValue(2),
				MakeUintegerAccessor(&RdmaHw::m_sprayHoldAutoLeaves),
				MakeUintegerChecker<uint32_t>(2))
		.AddAttribute("SprayHoldRule",
				"Under PathSelector 6, 7 and 9, what puts a spine in loss hold, per "
				"hold key: 0=a loss interval whose lost fraction grades it 0 "
				"through SprayLossThreshold1-3; 1=the key's sequential "
				"likelihood-ratio test (LossLikelihoodRatio) of SprayLossP0 "
				"against SprayLossP1, at the declared loss that passes it. "
				"Default 0.",
				UintegerValue(static_cast<uint32_t>(HoldRule::Count)),
				MakeUintegerAccessor(&RdmaHw::m_sprayHoldRule),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(HoldRule::Count),
					static_cast<uint32_t>(HoldRule::Llr)))
		.AddAttribute("SprayLossP0",
				"Under SprayHoldRule 1, the loss probability of a send on a "
				"working path, at least 0 and below SprayLossP1; 0 holds at "
				"the first loss. Default 0.",
				DoubleValue(0),
				MakeDoubleAccessor(&RdmaHw::m_sprayLossP0),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayLossP1",
				"Under SprayHoldRule 1, the loss probability of a send on a "
				"faulted path, below 1. Default 0.001.",
				DoubleValue(0.001),
				MakeDoubleAccessor(&RdmaHw::m_sprayLossP1),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayHoldAlpha",
				"Under SprayHoldRule 1, the probability of holding a working "
				"path, in (0, 0.5). Default 0.001.",
				DoubleValue(0.001),
				MakeDoubleAccessor(&RdmaHw::m_sprayHoldAlpha),
				MakeDoubleChecker<double>(0, 0.5))
		.AddAttribute("SprayHoldBeta",
				"Under SprayHoldRule 1, the probability of missing a faulted "
				"path, in (0, 0.5). Default 0.001.",
				DoubleValue(0.001),
				MakeDoubleAccessor(&RdmaHw::m_sprayHoldBeta),
				MakeDoubleChecker<double>(0, 0.5))
		.AddAttribute("SprayHoldRelease",
				"Under PathSelector 6, 7 and 9, how a spine leaves loss hold: "
				"0=SprayLossHoldCleanSentinels clean sends of its sentinels, "
				"one per SprayHeldSentinelIntervalNs; 1=its trickle sends, "
				"SprayHoldTrickle of each queue pair's sends, whose clean "
				"acknowledgements move the hold key's TrickleRelease to "
				"ln(1 / SprayHoldAlpha). Default 0.",
				UintegerValue(static_cast<uint32_t>(HoldRelease::Sentinels)),
				MakeUintegerAccessor(&RdmaHw::m_sprayHoldRelease),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(HoldRelease::Sentinels),
					static_cast<uint32_t>(HoldRelease::Trickle)))
		.AddAttribute("SprayHoldTrickle",
				"Under SprayHoldRelease 1, the fraction of each queue pair's "
				"sends a held spine receives, one in its inverse rounded, in "
				"(0, 0.5]. Default 0.01.",
				DoubleValue(0.01),
				MakeDoubleAccessor(&RdmaHw::m_sprayHoldTrickle),
				MakeDoubleChecker<double>(0, 0.5))
		.AddAttribute("SpraySentinelMode",
				"Under PathSelector 6, 7 and 9, what an exploration send onto a "
				"held spine, a sentinel or a trickle send, is: 0=the data "
				"packet itself; 1=a copy of a data packet sent on the spine "
				"the selector chooses, in the same send, which no retransmission or "
				"timeout waits for (RdmaQueuePair::AddCopy). Default 0.",
				UintegerValue(static_cast<uint32_t>(SentinelMode::Data)),
				MakeUintegerAccessor(&RdmaHw::m_spraySentinelMode),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(SentinelMode::Data),
					static_cast<uint32_t>(SentinelMode::Duplicate)))
		.AddAttribute("SprayCopyExpiry",
				"Under SpraySentinelMode 1, when a copy its own acknowledgement "
				"has not answered is declared lost for the selector: 0=once its "
				"data send is acknowledged, more than one base RTT after the "
				"copy was sent; 1=more than SprayTailLossRtts base RTTs after it "
				"was sent, whether its data send is acknowledged or not. Each "
				"is checked at the queue pair's acknowledgements. Default 0.",
				UintegerValue(static_cast<uint32_t>(CopyExpiry::DataAck)),
				MakeUintegerAccessor(&RdmaHw::m_sprayCopyExpiry),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(CopyExpiry::DataAck),
					static_cast<uint32_t>(CopyExpiry::Deadline)))
		.AddAttribute("SprayHeldLossRtts",
				"Under PathSelector 6, 7 and 9, the queue pair's base RTTs "
				"after which a data send up a spine the loss hold, or the "
				"receivers' hold, held as it was chosen, an exploration send "
				"under SpraySentinelMode 0 or any send while every spine is "
				"held, is declared lost if it is still unacknowledged, and "
				"repaired, without waiting for the retransmission timeout. "
				"0 waits for the timeout. Under SprayOverdueRepair 3 with "
				"PathSelector 8 or 10 or the uniform draw, the wait of the "
				"overdue repair alone. Default 2.",
				DoubleValue(2),
				MakeDoubleAccessor(&RdmaHw::m_sprayHeldLossRtts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("SprayOverdueRepair",
				"Under PathSelector 6, 7 and 9 with SprayHeldLossRtts above "
				"zero (3 alone also under PathSelector 8 and 10 and the "
				"uniform draw, LoadBalancing 2 with PathSelector 0, whose "
				"sends no hold holds), whether a queue pair's latest send on "
				"a spine not held "
				"as it was chosen is repaired, as a held send is, once its "
				"answer is overdue by SprayHeldLossRtts base RTTs: one repair "
				"until the queue pair's next answer or retransmission timeout. "
				"0=no; 1=evidence: the send is declared lost, its loss told, "
				"deferred or kept as a timed-out send's under "
				"SprayDuplicateReport; 2=probe: the repair counts no loss, and "
				"under SprayDuplicateReport the repair's answer settles the "
				"send, unflagged a data loss on its path, flagged an answer "
				"loss, and none when it is lost or the cumulative "
				"acknowledgement covers the sequence first. A receiver flags "
				"its answers to duplicates as under SprayDuplicateReport, and "
				"the answer showing the send arrived first counts the repair "
				"spurious. 3=reprobe: probe's repair, and a repair without an "
				"answer for SprayHeldLossRtts base RTTs is followed by another "
				"of the same send, up to SprayOverdueRepairLimit repairs until "
				"the next answer or timeout. Default 0.",
				UintegerValue(static_cast<uint32_t>(OverdueRepair::Off)),
				MakeUintegerAccessor(&RdmaHw::m_sprayOverdueRepair),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(OverdueRepair::Off),
					static_cast<uint32_t>(OverdueRepair::Reprobe)))
		.AddAttribute("SprayOverdueRepairLimit",
				"Under SprayOverdueRepair 3, the repairs of one send the "
				"overdue repair sends until the queue pair's next answer or "
				"retransmission timeout. Default 2.",
				UintegerValue(2),
				MakeUintegerAccessor(&RdmaHw::m_sprayOverdueRepairLimit),
				MakeUintegerChecker<uint32_t>(1))
		.AddAttribute("SprayAckLossRtts",
				"Under LoadBalancing 2, the queue pair's base RTTs after "
				"which a send the cumulative acknowledgement covered before "
				"its own acknowledgement arrived, still without it, is "
				"declared lost on its path for the path selector alone: no "
				"repair, no change to the retransmission timeout and no "
				"congestion signal. 0 declares nothing. Default 0.",
				DoubleValue(0),
				MakeDoubleAccessor(&RdmaHw::m_sprayAckLossRtts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("AckRoute",
				"Under LoadBalancing 2, the spine the switches route each "
				"answer a receiver sends over, an acknowledgement, a NACK or a "
				"repair request: 0=the spine that carried the data packet it "
				"answers; 1=a spine the receiver draws uniformly per answer "
				"from AckRouteRandomVariable; 2=a member, drawn uniformly per "
				"answer, of the connection's set of AckRouteSetSize spines, "
				"probed once per AckRouteSetProbeRtts SprayBaseRttNs while its "
				"data arrives and a member replaced whose probe goes "
				"unanswered; 3=the "
				"connection's one spine, drawn again at each duplicate data "
				"arrival. Under 1 to 3 the spine is named in the answer's "
				"route_spine field, which needs CustomHeader::ackRouteSpineBytes "
				"SpineFieldBytes of SpineCount. The answer echoes the data "
				"packet's identification either way. Default 0.",
				UintegerValue(static_cast<uint32_t>(AckRoute::DataSpine)),
				MakeUintegerAccessor(&RdmaHw::m_ackRoute),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(AckRoute::DataSpine),
					static_cast<uint32_t>(AckRoute::Repath)))
		.AddAttribute("AckRouteSetSize",
				"Under AckRoute 2, the spines of each connection's set, at "
				"most SpineCount. Default 2.",
				UintegerValue(2),
				MakeUintegerAccessor(&RdmaHw::m_ackRouteSetSize),
				MakeUintegerChecker<uint32_t>(1))
		.AddAttribute("AckRouteSetProbeRtts",
				"Under AckRoute 2, the SprayBaseRttNs between a connection's "
				"rounds of probes of its set while its data arrives; a member "
				"is replaced at a round once a probe of it has been "
				"unanswered for kAnswerSetProbeTimeoutRtts base RTTs. "
				"Default 1.",
				DoubleValue(1),
				MakeDoubleAccessor(&RdmaHw::m_ackRouteSetProbeRtts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("AckRouteRandomVariable",
				"Under AckRoute 1, the variable each answer's spine is drawn "
				"from, a stream of this host's own. Null by default, so that "
				"AckRoute 0 allocates no stream.",
				PointerValue(),
				MakePointerAccessor(&RdmaHw::m_ackRouteRandom),
				MakePointerChecker<UniformRandomVariable>())
		.AddAttribute("AckRouteOracle",
				"Under AckRoute 1, whether the draw leaves out the spines over "
				"which an answer toward the sender's leaf crosses a fault as "
				"SwitchQueueState shows it, read once per SprayBaseRttNs; a "
				"spine once left out stays out. Default false.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_ackRouteOracle),
				MakeBooleanChecker())
		.AddAttribute("SprayDuplicateReport",
				"Under SprayCoverageFirst, whether a receiver flags its answer "
				"to a data packet whose sequence it held already, and what a "
				"sender makes of the flag on the answer to the resend of a send "
				"a retransmission timeout declared lost: 0=no flag; 1=defer, "
				"the timeout's loss told to the selector only at that answer, "
				"none where it is flagged, an answer loss, and on the first "
				"send's path where it is not or the cumulative acknowledgement "
				"covers the sequence first; 2=retract, each loss told at once "
				"and taken back where the answer is flagged, the hold it began "
				"graded again without it, which needs the host's auto loss "
				"hold under SprayHoldRule 0. Default 0.",
				UintegerValue(static_cast<uint32_t>(DuplicateReport::Off)),
				MakeUintegerAccessor(&RdmaHw::m_sprayDuplicateReport),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(DuplicateReport::Off),
					static_cast<uint32_t>(DuplicateReport::Retract)))
		.AddAttribute("SprayCoverageFirst",
				"Under LoadBalancing 2, whether the per-path rule, the held-loss "
				"rule, a tail repair request, a copy's expiry and "
				"SprayAckLossRtts first apply the queue pair's coverage, the "
				"cumulative acknowledgement, the one of the answer read and for "
				"a copy its data send's answer: a send it covers was delivered "
				"and its missing answer is an answer loss, counted as "
				"answer_loss_covered, neither repaired nor told to the "
				"selector. Default false.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_sprayCoverageFirst),
				MakeBooleanChecker())
		.AddAttribute("SprayAnswerPathSeq",
				"Under AckRoute 1 with SprayPathSeq and PathSelector 6, 7 or 9, "
				"whether answers are numbered per route spine and announce "
				"another route's count, the sender holds the routes of a "
				"receiver leaf its answers are lost on and says so in its data "
				"packets, and the receiver leaves the spines its senders hold "
				"out of its draw but for a probe answer per "
				"SprayAnswerProbeIntervalNs. Needs "
				"CustomHeader::ackAnswerSeqBytes 4 plus SpineFieldBytes and "
				"CustomHeader::udpAnswerHoldBytes SpineBitmapBytes of "
				"SpineCount. Default false.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_sprayAnswerPathSeq),
				MakeBooleanChecker())
		.AddAttribute("SprayAnswerProbeIntervalNs",
				"Under SprayAnswerPathSeq, the interval between two probe "
				"answers of a receiving host over one spine held toward one "
				"sender leaf. Default 1 ms.",
				UintegerValue(1000000),
				MakeUintegerAccessor(&RdmaHw::m_sprayAnswerProbeIntervalNs),
				MakeUintegerChecker<uint64_t>(1))
		.AddAttribute("SprayAnswerHoldClean",
				"Under SprayAnswerPathSeq, the answers in a row received over a "
				"held route without a loss that release it at the sender. "
				"Default 3.",
				UintegerValue(3),
				MakeUintegerAccessor(&RdmaHw::m_sprayAnswerHoldClean),
				MakeUintegerChecker<uint32_t>(1))
		.AddAttribute("SprayPathSeq",
				"Under LoadBalancing 2, whether every data packet carries "
				"its queue pair's count of the data packets sent before it up "
				"its requested spine, from which the receiver counts the "
				"packets missing per carrying spine. Needs "
				"CustomHeader::udpCarriesPathSeq. Default false.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_sprayPathSeq),
				MakeBooleanChecker())
		.AddAttribute("SprayTailRecovery",
				"Under SprayPathSeq, whether every data packet a queue pair "
				"sends once it has no new data left closes its spine and "
				"carries the count of the data packets sent up each spine, "
				"from which the receiver asks at once for the path sequences "
				"it misses and the sender repairs them without waiting for "
				"the retransmission timeout. Needs SelectiveRetransmission, "
				"SpineCount at most 8, CustomHeader::udpTailCounts equal to "
				"SpineCount and CustomHeader::ackTailNackBytes 5. Default "
				"false.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_sprayTailRecovery),
				MakeBooleanChecker())
		.AddAttribute("SprayTailRecoveryDelayNs",
				"Under SprayTailRecovery, how long a receiver waits on the "
				"path sequences a closing packet shows missing before it "
				"requests those still missing; zero requests them at once. "
				"Default RdmaHw::kTailDelayBaseRtt: SprayBaseRttNs, which is "
				"then needed.",
				UintegerValue(RdmaHw::kTailDelayBaseRtt),
				MakeUintegerAccessor(&RdmaHw::m_sprayTailRecoveryDelayNs),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("SprayTailLossRtts",
				"Under SprayTailRecovery, the queue pair's base RTTs after a "
				"tail repair request declares a send lost that the path "
				"selector is told of the loss; a trim notification of the "
				"send arriving before then drops it, a trim being no loss. "
				"The send is repaired at once all the same. 0 tells the "
				"selector at once. Default 1.",
				DoubleValue(1),
				MakeDoubleAccessor(&RdmaHw::m_sprayTailLossRtts),
				MakeDoubleChecker<double>(0))
		.AddAttribute("SprayReceiverHold",
				"Under PathSelector 6, 7 and 9 with SprayPathSeq, whether the "
				"receiver holds spines by the packets missing on them per "
				"sender leaf (ReceiverHold) and reports them in every "
				"acknowledgement, whose senders hold them toward its leaf. "
				"Needs SwitchQueueState for the hosts' leaves and "
				"CustomHeader::ackReceiverHoldBytes SpineBitmapBytes of "
				"SpineCount. Default false.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_sprayReceiverHold),
				MakeBooleanChecker())
		.AddAttribute("SprayReceiverHoldClean",
				"Under SprayReceiverHold, the consecutive arrivals without a "
				"missing packet on a held spine that release it. Default 1024.",
				UintegerValue(1024),
				MakeUintegerAccessor(&RdmaHw::m_sprayReceiverHoldClean),
				MakeUintegerChecker<uint32_t>(1))
		.AddAttribute("SprayReceiverHoldSlots",
				"Under SprayReceiverHold, the spines an acknowledgement's "
				"receiver_hold field names as a spine list, zero for the "
				"bitmap over every spine. Over that many held spines the "
				"receiver names that many, in turn from past the last one it "
				"named toward the sender's leaf, and marks the list partial: "
				"its sender adds the spines it names and releases none. Needs "
				"CustomHeader::ackReceiverHoldSlots of it and "
				"ackReceiverHoldBytes SpineListBytes of it. Default 0.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_sprayReceiverHoldSlots),
				MakeUintegerChecker<uint32_t>(0, kMaxSpineListSlots))
		.AddAttribute("SprayReceiverHoldWiden",
				"Under SprayReceiverHold, with the host's marks shared by its "
				"queue pairs: a spine that receivers on this many destination "
				"leaves hold for the host's leaf is held toward every leaf, "
				"its sentinels still sent toward those leaves alone. 0, the "
				"default, holds it toward those leaves only; otherwise at "
				"least 2.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_sprayReceiverHoldWiden),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("SprayReceiverTailMissing",
				"Under SprayReceiverHold and SprayTailRecovery, whether the "
				"receiver's hold also counts missing, on their requested "
				"spine, the path sequences a tail repair request names that "
				"neither arrived nor were trimmed SprayTailLossRtts base RTTs "
				"(SprayBaseRttNs) after the request and that no gap counted, "
				"so that a spine on which nothing arrives is held. Default "
				"false.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_sprayReceiverTailMissing),
				MakeBooleanChecker())
		.AddAttribute("SprayStrayPathSeq",
				"Under SprayPathSeq, how the path sequences of a packet that "
				"arrives after its flow's receive queue pair was removed at "
				"the flow's completion are read: 0=by the receive queue pair "
				"it makes, as a new flow's; 1=against the removed one: the "
				"packet takes its sequence out of that queue pair's waiting "
				"tail runs and is reported receiver_tail_missing_late where "
				"that queue pair counted it missing, and counts no gap, asks "
				"no tail repair and counts nothing in the receiver's hold. "
				"Default 0.",
				UintegerValue(static_cast<uint32_t>(StrayPathSeq::Fresh)),
				MakeUintegerAccessor(&RdmaHw::m_sprayStrayPathSeq),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(StrayPathSeq::Fresh),
					static_cast<uint32_t>(StrayPathSeq::Closed)))
		.AddAttribute("SprayPathState",
				"Under LoadBalancing 2, how long a host keeps the path "
				"selector of a queue pair: 0=for that queue pair alone, one "
				"message; 1=for its connection, one selector per destination "
				"and priority group, shared by the host's queue pairs to it "
				"and kept past them, so that what it learns of the paths "
				"carries from one message to the next. Default 0.",
				UintegerValue(static_cast<uint32_t>(PathState::Message)),
				MakeUintegerAccessor(&RdmaHw::m_sprayPathState),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(PathState::Message),
					static_cast<uint32_t>(PathState::Connection)))
		.AddAttribute("SprayCandidates",
				"Under LoadBalancing 3, the candidates drawn per packet, of which "
				"the highest-scored is sent on. Default 1.",
				UintegerValue(1),
				MakeUintegerAccessor(&RdmaHw::m_sprayCandidates),
				MakeUintegerChecker<uint32_t>(1))
		.AddAttribute("SprayCandidateDraw",
				"Under LoadBalancing 3, how each candidate is drawn: 0=in "
				"proportion to the spine shares, 1=uniformly. Default 0.",
				UintegerValue(static_cast<uint32_t>(
					SpineScores::CandidateDraw::Proportional)),
				MakeUintegerAccessor(&RdmaHw::m_sprayCandidateDraw),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(SpineScores::CandidateDraw::Proportional),
					static_cast<uint32_t>(SpineScores::CandidateDraw::Uniform)))
		.AddAttribute("SprayEdgeFreeze",
				"Under LoadBalancing 3, whether a report whose edge bit is set "
				"leaves the sender's spine scores as they are; when false it "
				"moves them as any other report. Default true.",
				BooleanValue(true),
				MakeBooleanAccessor(&RdmaHw::m_sprayEdgeFreeze),
				MakeBooleanChecker())
		.AddAttribute("SpraySenderPoolWeight",
				"Under LoadBalancing 3, the weight w of the host's pooled spine "
				"scores, which every report the host's scores take moves too: "
				"a spine is drawn by w times its pooled score plus 1 - w times "
				"its score towards the destination, in [0, 1]. Default 0.",
				DoubleValue(0),
				MakeDoubleAccessor(&RdmaHw::m_spraySenderPoolWeight),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("SprayPoolDecayIntervalNs",
				"Under LoadBalancing 3 with SpraySenderPoolWeight above zero, "
				"the fixed interval once per which the host's pooled scores "
				"decay, each spine's score being the interval's decayed base "
				"plus the mean grade the reports taken in it gave the spine. "
				"Default 0: the pooled scores decay once per report taken.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_sprayPoolDecayIntervalNs),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("SprayPoolHoldPropagates",
				"Under LoadBalancing 3 with SpraySenderPoolWeight above zero, "
				"whether a spine the host's pooled scores hold, by their held "
				"flag or their loss hold, is held in every destination's draw: "
				"it weighs nothing in the mixed draw, keeping epsilon / N only "
				"under SprayHeldExploration epsilon, and under sentinel is sent "
				"the pool's sentinels, one per sentinel interval for the host. "
				"Default false.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_sprayPoolHoldPropagates),
				MakeBooleanChecker())
		.AddAttribute("SprayEdgeWindowPenalty",
				"Under LoadBalancing 3 and CC mode 11, the Rcv_Cwnd_Pend (UEC "
				"1.0.3 section 3.6.13.2) of an acknowledgement whose report says "
				"the receiver's downlink is congested, 0 to 127. Default 64, the "
				"section's example that halves a window over a round trip.",
				UintegerValue(64),
				MakeUintegerAccessor(&RdmaHw::m_sprayEdgeWindowPenalty),
				MakeUintegerChecker<uint32_t>(0, 127))
		.AddTraceSource("SpineReport",
				"Under LoadBalancing 3, a receiver issued a spine report: the "
				"time, the grader, which holds the report and the interval it "
				"closed, and under SprayGradeScope 1 the receive queue pair the "
				"grader belongs to, else null.",
				MakeTraceSourceAccessor(&RdmaHw::m_traceSpineReport),
				"ns3::RdmaHw::SpineReportTracedCallback")
		.AddTraceSource("SwitchQueueReport",
				"Under SprayGradeCostSource 1, a receiver graded one source "
				"leaf's paths: the time, the grader, the source leaf, the "
				"values read from the switches and the report they earned.",
				MakeTraceSourceAccessor(&RdmaHw::m_traceSwitchQueueReport),
				"ns3::RdmaHw::SwitchQueueReportTracedCallback")
		.AddTraceSource("SpineLossInterval",
				"Under LoadBalancing 3 with SprayLossThreshold1 set and "
				"SpraySenderPoolWeight above zero, the host's pooled scores "
				"closed a loss interval, and under PathSelector 6 or 9 with "
				"SprayHoldScope 0 the host's loss hold did: the time it ended "
				"at and, per spine, the sends and losses counted in it and the "
				"grade its lost fraction earns.",
				MakeTraceSourceAccessor(&RdmaHw::m_traceSpineLossInterval),
				"ns3::RdmaHw::SpineLossIntervalTracedCallback")
		.AddTraceSource("RetransmissionTimeout",
				"A queue pair's retransmission timeout fired and was taken: the "
				"time, the queue pair, the requested spine of its oldest "
				"outstanding send or -1 where none is kept or no spine is "
				"named, whether that send was an exploration send onto a held "
				"spine, and the bytes outstanding.",
				MakeTraceSourceAccessor(&RdmaHw::m_traceRetransmissionTimeout),
				"ns3::RdmaHw::RetransmissionTimeoutTracedCallback")
		.AddTraceSource("LeafSpineLossInterval",
				"Under PathSelector 6 or 9 with SprayHoldScope 1, the loss hold of "
				"one destination leaf closed a loss interval, and with "
				"SprayHoldScope 2 the host's loss hold did, once per "
				"destination leaf: the leaf, and what SpineLossInterval "
				"passes; with SprayHoldScope 3 the loss hold of one "
				"destination host did, passed with its address for the "
				"leaf.",
				MakeTraceSourceAccessor(&RdmaHw::m_traceLeafSpineLossInterval),
				"ns3::RdmaHw::LeafSpineLossIntervalTracedCallback")
		.AddTraceSource("ReceiverHoldInterval",
				"Under SprayReceiverHold, the receiving host's hold closed an "
				"interval of SprayLossIntervalNs, once per sender leaf: the "
				"leaf, the time it ended at and, per spine, the arrivals and "
				"the missing packets counted in it and the scope of its hold, "
				"0=none, 1=leaf, 2=host.",
				MakeTraceSourceAccessor(&RdmaHw::m_traceReceiverHoldInterval),
				"ns3::RdmaHw::ReceiverHoldIntervalTracedCallback")
		.AddTraceSource("AnswerHold",
				"Under SprayAnswerPathSeq, a hold changed: of a route spine of "
				"a receiver leaf's answers at this host as a sender, or of a "
				"spine of this host's answers to a sender leaf as a receiver; "
				"the side, the leaf, the spine, whether held and the time.",
				MakeTraceSourceAccessor(&RdmaHw::m_traceAnswerHold),
				"ns3::RdmaHw::AnswerHoldTracedCallback")
		.AddAttribute("EwmaGain",
				"Control gain parameter which determines the level of rate decrease",
				DoubleValue(1.0 / 16),
				MakeDoubleAccessor(&RdmaHw::m_g),
				MakeDoubleChecker<double>())
		.AddAttribute ("RateOnFirstCnp",
				"the fraction of rate on first CNP",
				DoubleValue(1.0),
				MakeDoubleAccessor(&RdmaHw::m_rateOnFirstCNP),
				MakeDoubleChecker<double> ())
		.AddAttribute("ClampTargetRate",
				"Clamp target rate.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_EcnClampTgtRate),
				MakeBooleanChecker())
		.AddAttribute("RPTimer",
				"The rate increase timer at RP in microseconds",
				DoubleValue(1500.0),
				MakeDoubleAccessor(&RdmaHw::m_rpgTimeReset),
				MakeDoubleChecker<double>())
		.AddAttribute("RateDecreaseInterval",
				"The interval of rate decrease check",
				DoubleValue(4.0),
				MakeDoubleAccessor(&RdmaHw::m_rateDecreaseInterval),
				MakeDoubleChecker<double>())
		.AddAttribute("FastRecoveryTimes",
				"The rate increase timer at RP",
				UintegerValue(5),
				MakeUintegerAccessor(&RdmaHw::m_rpgThreshold),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("AlphaResumInterval",
				"The interval of resuming alpha",
				DoubleValue(55.0),
				MakeDoubleAccessor(&RdmaHw::m_alpha_resume_interval),
				MakeDoubleChecker<double>())
		.AddAttribute("RateAI",
				"Rate increment unit in AI period",
				DataRateValue(DataRate("5Mb/s")),
				MakeDataRateAccessor(&RdmaHw::m_rai),
				MakeDataRateChecker())
		.AddAttribute("RateHAI",
				"Rate increment unit in hyperactive AI period",
				DataRateValue(DataRate("50Mb/s")),
				MakeDataRateAccessor(&RdmaHw::m_rhai),
				MakeDataRateChecker())
		.AddAttribute("VarWin",
				"Use variable window size or not",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_var_win),
				MakeBooleanChecker())
		.AddAttribute("FastReact",
				"Fast React to congestion feedback",
				BooleanValue(true),
				MakeBooleanAccessor(&RdmaHw::m_fast_react),
				MakeBooleanChecker())
		.AddAttribute("MiThresh",
				"Threshold of number of consecutive AI before MI",
				UintegerValue(5),
				MakeUintegerAccessor(&RdmaHw::m_miThresh),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("TargetUtil",
				"The Target Utilization of the bottleneck bandwidth, by default 95%",
				DoubleValue(0.95),
				MakeDoubleAccessor(&RdmaHw::m_targetUtil),
				MakeDoubleChecker<double>())
		.AddAttribute("UtilHigh",
				"The upper bound of Target Utilization of the bottleneck bandwidth, by default 98%",
				DoubleValue(0.98),
				MakeDoubleAccessor(&RdmaHw::m_utilHigh),
				MakeDoubleChecker<double>())
		.AddAttribute("RateBound",
				"Bound packet sending by rate, for test only",
				BooleanValue(true),
				MakeBooleanAccessor(&RdmaHw::m_rateBound),
				MakeBooleanChecker())
		.AddAttribute("MultiRate",
				"Maintain multiple rates in HPCC",
				BooleanValue(true),
				MakeBooleanAccessor(&RdmaHw::m_multipleRate),
				MakeBooleanChecker())
		.AddAttribute("SampleFeedback",
				"Whether sample feedback or not",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_sampleFeedback),
				MakeBooleanChecker())
		.AddAttribute("TimelyAlpha",
				"Alpha of TIMELY",
				DoubleValue(0.875),
				MakeDoubleAccessor(&RdmaHw::m_tmly_alpha),
				MakeDoubleChecker<double>())
		.AddAttribute("TimelyBeta",
				"Beta of TIMELY",
				DoubleValue(0.8),
				MakeDoubleAccessor(&RdmaHw::m_tmly_beta),
				MakeDoubleChecker<double>())
		.AddAttribute("TimelyTLow",
				"TLow of TIMELY (ns)",
				UintegerValue(50000),
				MakeUintegerAccessor(&RdmaHw::m_tmly_TLow),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("TimelyTHigh",
				"THigh of TIMELY (ns)",
				UintegerValue(500000),
				MakeUintegerAccessor(&RdmaHw::m_tmly_THigh),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("TimelyMinRtt",
				"MinRtt of TIMELY (ns)",
				UintegerValue(20000),
				MakeUintegerAccessor(&RdmaHw::m_tmly_minRtt),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("DctcpRateAI",
				"DCTCP's Rate increment unit in AI period",
				DataRateValue(DataRate("1000Mb/s")),
				MakeDataRateAccessor(&RdmaHw::m_dctcp_rai),
				MakeDataRateChecker())
		.AddAttribute("NsccTargetQdelay",
				"NSCC target_qdelay, the queueing delay from which an acknowledgement "
				"counts as congested, in base RTTs. Default 0.75, UEC 1.0.3 section "
				"3.6.13.3's value where switches trim.",
				DoubleValue(0.75),
				MakeDoubleAccessor(&RdmaHw::m_nscc_target_qdelay),
				MakeDoubleChecker<double>(0))
		.AddAttribute("NsccMaxWindow",
				"NSCC MaxWnd, the ceiling and the initial value of the window, in "
				"bandwidth-delay products. Default 1.5, UEC 1.0.3 Table 3-82.",
				DoubleValue(1.5),
				MakeDoubleAccessor(&RdmaHw::m_nscc_max_window),
				MakeDoubleChecker<double>(0))
		.AddAttribute("NsccGamma",
				"NSCC gamma, the gain of a multiplicative decrease on the average "
				"queueing delay's excess over the target, relative to that average. "
				"Default 0.8, UEC 1.0.3 Table 3-82.",
				DoubleValue(0.8),
				MakeDoubleAccessor(&RdmaHw::m_nscc_gamma),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("NsccMaxMdJump",
				"NSCC max_md_jump, the smallest factor one multiplicative decrease "
				"scales the window by. Default 0.5, UEC 1.0.3 Table 3-82.",
				DoubleValue(0.5),
				MakeDoubleAccessor(&RdmaHw::m_nscc_max_md_jump),
				MakeDoubleChecker<double>(0, 1))
		.AddAttribute("NsccFairIncrease",
				"NSCC fi, the fair increase constant, in MTUs times scaling_a, the "
				"queue pair's bandwidth-delay product over 150000 bytes. Default 5, "
				"UEC 1.0.3 Table 3-82.",
				DoubleValue(5.0),
				MakeDoubleAccessor(&RdmaHw::m_nscc_fair_increase),
				MakeDoubleChecker<double>(0))
		.AddAttribute("NsccFastIncreaseScale",
				"NSCC fi_scale, the window bytes fast increase adds per byte "
				"acknowledged, in units of scaling_a. Default 0.25, UEC 1.0.3 "
				"Table 3-82.",
				DoubleValue(0.25),
				MakeDoubleAccessor(&RdmaHw::m_nscc_fast_increase_scale),
				MakeDoubleChecker<double>(0))
		.AddAttribute("NsccEta",
				"NSCC eta, the additive increase applied once per adjustment period, "
				"in MTUs times scaling_a. Default 0.15, UEC 1.0.3 Table 3-82.",
				DoubleValue(0.15),
				MakeDoubleAccessor(&RdmaHw::m_nscc_eta),
				MakeDoubleChecker<double>(0))
		.AddAttribute("NsccAlpha",
				"NSCC alpha, the proportional increase gain, in MTUs times scaling_a "
				"times scaling_b per target_qdelay, scaling_b being target_qdelay "
				"over 12 us. Default 4, UEC 1.0.3 section 3.6.13.3.",
				DoubleValue(4.0),
				MakeDoubleAccessor(&RdmaHw::m_nscc_alpha),
				MakeDoubleChecker<double>(0))
		.AddAttribute("NsccQaGate",
				"NSCC qa_gate: QuickAdapt sets the window only if the bytes "
				"acknowledged over its last period are below MaxWnd / 2^qa_gate. "
				"Default 3, UEC 1.0.3 Table 3-82.",
				UintegerValue(3),
				MakeUintegerAccessor(&RdmaHw::m_nscc_qa_gate),
				MakeUintegerChecker<uint32_t>(0, 63))
		.AddAttribute("NsccQaThreshold",
				"NSCC qa_threshold, the queueing delay that triggers QuickAdapt "
				"without a trim, in target_qdelays. Zero disables it, as UEC 1.0.3 "
				"section 3.6.13.3 prescribes where switches trim; its value without "
				"trimming is 4. Default 0.",
				DoubleValue(0),
				MakeDoubleAccessor(&RdmaHw::m_nscc_qa_threshold),
				MakeDoubleChecker<double>(0))
		.AddAttribute("NsccAdjustBytes",
				"NSCC adjust_bytes_threshold, in MTUs: the increases accumulated "
				"since the last adjustment are applied once more bytes than this "
				"are acknowledged. Default 8, UEC 1.0.3 Table 3-82.",
				UintegerValue(8),
				MakeUintegerAccessor(&RdmaHw::m_nscc_adjust_bytes),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("NsccAdjustPeriod",
				"NSCC adjust_period_threshold, in base RTTs: the accumulated "
				"increases are applied, and eta added, at least this often. "
				"Default 1, UEC 1.0.3 Table 3-82.",
				DoubleValue(1.0),
				MakeDoubleAccessor(&RdmaHw::m_nscc_adjust_period),
				MakeDoubleChecker<double>(0))
		.AddAttribute("NsccWindowState",
				"Under CC mode 11, how long a host keeps an NSCC window: 0=per "
				"queue pair, one message; 1=per connection, one window per "
				"destination and priority group, opened once at MaxWnd, "
				"shared by the host's queue pairs to it, bounding the bytes "
				"they have in flight together, and kept past them and across "
				"idle periods as it stands. Default 0.",
				UintegerValue(static_cast<uint32_t>(NsccWindowState::Message)),
				MakeUintegerAccessor(&RdmaHw::m_nsccWindowState),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(NsccWindowState::Message),
					static_cast<uint32_t>(NsccWindowState::Connection)))
		.AddAttribute("NsccRttSample",
				"Under CC mode 11, which answers give NSCC a round trip: 0=any "
				"but the first resend after a declared loss; 1=Karn's rule, "
				"none of a sequence sent more than once while unacknowledged "
				"(declared lost, trimmed and released, or copied), and each "
				"sample at least the queue pair's base RTT. Default 0.",
				UintegerValue(static_cast<uint32_t>(NsccRttSample::Any)),
				MakeUintegerAccessor(&RdmaHw::m_nsccRttSample),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(NsccRttSample::Any),
					static_cast<uint32_t>(NsccRttSample::Karn)))
		.AddTraceSource("NsccWindow",
				"Under CC mode 11, the window a queue pair may keep in flight "
				"changed: the queue pair, then the old and the new window in "
				"bytes. Fired once per change, at the queue pair's start too.",
				MakeTraceSourceAccessor(&RdmaHw::m_traceNsccWindow),
				"ns3::RdmaHw::NsccWindowTracedCallback")
		.AddAttribute("PintSmplThresh",
				"PINT's sampling threshold in rand()%65536",
				UintegerValue(65536),
				MakeUintegerAccessor(&RdmaHw::pint_smpl_thresh),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("TotalPauseTimes",
				"The number of pause times to simulate PFC pause due to PCIe",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_total_pause_times),
				MakeUintegerChecker<uint64_t>());
		;
	return tid;
}

RdmaHw::RdmaHw(){
	m_leaf = SwitchQueueState::kNoLeaf;
	enable_pcie_pause = true;
	m_paused_times = 0;
}

void RdmaHw::SetNode(Ptr<Node> node){
	m_node = node;
}
void RdmaHw::Setup(QpCompleteCallback cb, QpFailureCallback failure_cb){
	NS_ABORT_MSG_IF(m_forgiveness && !m_selective_retransmission,
		"Forgiveness needs SelectiveRetransmission: under go-back-N the "
		"receiver never consults its out-of-order ranges, so a forgiven range "
		"would be pulled forever");
	NS_ABORT_MSG_IF(m_congestionExemption && !m_forgiveness,
		"CongestionExemption needs Forgiveness: the receiver's refusal to "
		"forgive is the only signal that ends an exemption, so without "
		"forgiveness an exempt queue pair would never take a rate cut again");
	NS_ABORT_MSG_IF(!m_remainderVerdictCallback.IsNull() &&
			!m_selective_retransmission,
		"The step stop needs SelectiveRetransmission: a forgiven remainder is "
		"absorbed as an accepted out-of-order range");
	NS_ABORT_MSG_IF(!ValidSprayLoss(),
		"SprayLossThreshold1-3 need SelectiveRetransmission: a send is declared "
		"lost only from the send records it keeps");
	NS_ABORT_MSG_IF(!ValidHeldExploration(),
		"SprayHeldExploration none needs SprayGradeCostSource 1: a held spine "
		"is sent nothing, and the receiver's estimate releases a hold only "
		"for arrivals");
	NS_ABORT_MSG_IF(!ValidRepsHold(),
		"PathSelector 6, 7 and 9 hold spines by their loss and need LoadBalancing 2, "
		"SprayLossThreshold1-3, SprayLossIntervalNs above zero, "
		"SprayLossHoldCleanSentinels above zero unless SprayHoldRelease is 1, "
		"and SprayHeldExploration sentinel, and PathSelector 6 and 9 under "
		"SprayHoldScope 1 or 2 need SwitchQueueState for the hosts' leaves");
	NS_ABORT_MSG_IF(!ValidHoldRule(),
		"SprayHoldRule 1 is read only under PathSelector 6, 7 and 9 and needs "
		"0 <= SprayLossP0 < SprayLossP1 < 1 and SprayHoldAlpha and "
		"SprayHoldBeta in (0, 0.5)");
	NS_ABORT_MSG_IF(!ValidHoldRelease(),
		"SprayHoldRelease 1 is read only under PathSelector 6, 7 and 9, reads "
		"neither SprayLossHoldCleanSentinels nor SprayHeldSentinelIntervalNs, "
		"and needs SprayHoldTrickle in (0, 0.5], 0 <= SprayLossP0 < "
		"SprayLossP1 < 1 and SprayHoldAlpha in (0, 0.5)");
	NS_ABORT_MSG_IF(m_spraySentinelMode !=
				static_cast<uint32_t>(SentinelMode::Data) && !IsRepsHold(),
		"SpraySentinelMode 1 is read only under PathSelector 6, 7 and 9");
	NS_ABORT_MSG_IF(!ValidCopyExpiry(),
		"SprayCopyExpiry 1 is read only under SpraySentinelMode 1, whose copies "
		"it expires, and needs SprayTailLossRtts above zero");
	NS_ABORT_MSG_IF(!ValidPathSeq(),
		"SprayPathSeq is read only under LoadBalancing 2, and data "
		"headers carry the path sequence exactly when it is set "
		"(CustomHeader::udpCarriesPathSeq)");
	NS_ABORT_MSG_IF(!ValidTailRecovery(),
		"SprayTailRecovery needs SprayPathSeq, SelectiveRetransmission and "
		"at most 8 spines, and data headers carry a count per spine and "
		"repair requests the tail fields exactly when it is set "
		"(CustomHeader::udpTailCounts, CustomHeader::ackTailNackBytes)");
	NS_ABORT_MSG_IF(!ValidTailLoss(),
		"SprayTailLossRtts is read only under SprayTailRecovery");
	NS_ABORT_MSG_IF(!ValidReceiverHold(),
		"SprayReceiverHold needs SprayPathSeq, PathSelector 6, 7 or 9, "
		"SwitchQueueState for the hosts' leaves, and acknowledgements carry "
		"the receiver_hold field over the spines, or SprayReceiverHoldSlots "
		"of at most 127 as a spine list, exactly when it is set "
		"(CustomHeader::ackReceiverHoldBytes, ackReceiverHoldSlots)");
	NS_ABORT_MSG_IF(!ValidStrayPathSeq(),
		"SprayStrayPathSeq 1 is read only under SprayPathSeq");
	NS_ABORT_MSG_IF(!ValidNsccRttSample(),
		"NsccRttSample 1 is read only under CC mode 11");
	NS_ABORT_MSG_IF(!ValidDuplicateReport(),
		"SprayDuplicateReport needs SprayCoverageFirst, and 2 the host's auto "
		"loss hold (PathSelector 6 or 9 with SprayHoldScope auto) under "
		"SprayHoldRule 0");
	NS_ABORT_MSG_IF(!ValidOverdueRepair(),
		"SprayOverdueRepair needs SprayHeldLossRtts above zero and PathSelector "
		"6, 7 or 9, or, at 3, PathSelector 8 or 10 or the uniform draw");
	NS_ABORT_MSG_IF(!ValidCoverageFirst(),
		"SprayCoverageFirst is read only under LoadBalancing 2, where every "
		"send has a record");
	NS_ABORT_MSG_IF(!ValidAnswerPathSeq(),
		"SprayAnswerPathSeq needs AckRoute 1, SprayPathSeq, PathSelector 6, 7 "
		"or 9 and SwitchQueueState for the hosts' leaves, and answers and data "
		"headers carry its fields over the spines exactly when it is set "
		"(CustomHeader::ackAnswerSeqBytes, CustomHeader::udpAnswerHoldBytes)");
	NS_ABORT_MSG_IF(!ValidReceiverTailMissing(),
		"SprayReceiverTailMissing needs SprayReceiverHold and "
		"SprayTailRecovery");
	NS_ABORT_MSG_IF(!ValidNsccWindowState(),
		"NsccWindowState 1 is read only under CC mode 11: the other modes keep "
		"their congestion state per queue pair");
	NS_ABORT_MSG_IF(!ValidPathState(),
		"SprayPathState 1 is read only under LoadBalancing 2, where an "
		"identification names a spine whatever the queue pair's ports");
	NS_ABORT_MSG_IF(!ValidRepsOracle(),
		"PathSelector 8 reads the spines' faults from SwitchQueueState and "
		"needs LoadBalancing 2 and SprayGradeCostSource 1; RepsOracleHops 1 is "
		"read only under PathSelector 8");
	NS_ABORT_MSG_IF(!ValidPoolHoldPropagates(),
		"SprayPoolHoldPropagates needs SpraySenderPoolWeight above zero: "
		"without a pool there is no pool's hold to propagate");
	NS_ABORT_MSG_IF(!ValidAckLoss(),
		"SprayAckLossRtts is read only under LoadBalancing 2, where every "
		"send has a record and every acknowledgement names its path");
	NS_ABORT_MSG_IF(!ValidAckRoute(),
		"AckRoute 1 is read only under LoadBalancing 2, draws from "
		"AckRouteRandomVariable a spine index below kNoRouteSpine, and answers "
		"carry the route_spine field exactly when it is set "
		"(CustomHeader::ackRouteSpineBytes); AckRouteOracle needs AckRoute 1 "
		"and SwitchQueueState for the hosts' leaves");
	NS_ABORT_MSG_IF(!ValidAnswerRoutes(),
		"AckRoute 2 draws sets of AckRouteSetSize spines, at most SpineCount, "
		"and AckRoute 3 tells a copied send by its tail flags, which need "
		"SprayTailRecovery");
	NS_ABORT_MSG_IF(m_loadBalancing !=
				static_cast<uint32_t>(LoadBalancingMode::Ecmp) && !m_pathRandom,
		"A LoadBalancing mode other than ECMP draws the identification of every "
		"data packet from PathRandomVariable");
	NS_ABORT_MSG_IF(NamesSpine(static_cast<LoadBalancingMode>(m_loadBalancing)) &&
			m_spineCount == 0,
		"LoadBalancing 2 and 3 name a spine per packet and need SpineCount");
	NS_ABORT_MSG_IF(IsGrading() && m_spineCount > SpineReport::kMaxSpines,
		"a spine report grades at most SpineReport::kMaxSpines spines");
	NS_ABORT_MSG_IF(CustomHeader::ackReportBytes != (IsSprayPolicy()
			? 1 + SpineReport::GradeBytes(m_spineCount) : 0),
		"LoadBalancing 3 and only it carries a report on the spines in every "
		"acknowledgement, its sequence and the grades of SpineCount spines");
	NS_ABORT_MSG_IF(IsSwitchQueueCost() && (IsPerConnectionGrading() ||
			!m_switchQueues),
		"SprayGradeCostSource 1 grades the host's paths from SwitchQueueState "
		"and needs SprayGradeScope 0");
	NS_ABORT_MSG_IF(m_sprayOneWayDelay &&
			(!IsGrading() || IntHeader::mode != IntHeader::TS),
		"SprayOneWayDelay grades the spines of LoadBalancing 3 by the send "
		"timestamp of IntHeader::mode TS");
	NS_ABORT_MSG_IF(m_pathSelectorKind !=
				static_cast<uint32_t>(PathSelectorKind::Ops) &&
			m_pathSelectorKind !=
				static_cast<uint32_t>(PathSelectorKind::AckRedirect) &&
			!IsRepsHold() && !IsRepsOracle() && !IsRepsSpine() &&
			m_loadBalancing !=
				static_cast<uint32_t>(LoadBalancingMode::EntropyHash),
		"PathSelector 1 to 4 choose entropy values and need LoadBalancing 1");
	NS_ABORT_MSG_IF(IsRepsSpine() && m_loadBalancing !=
				static_cast<uint32_t>(LoadBalancingMode::SprayUniform),
		"PathSelector 10 names spines and needs LoadBalancing 2");
	NS_ABORT_MSG_IF(m_pathSelectorKind ==
				static_cast<uint32_t>(PathSelectorKind::AckRedirect) &&
			m_loadBalancing !=
				static_cast<uint32_t>(LoadBalancingMode::SprayUniform),
		"PathSelector 5 names spines and needs LoadBalancing 2");
	NS_ABORT_MSG_IF(IsAckRedirect() && IntHeader::mode != IntHeader::TS,
		"PathSelector 5 scores each arrival by the one-way delay from the send "
		"timestamp of IntHeader::mode TS");
	NS_ABORT_MSG_IF(IsAckRedirect() && m_spineCount >= kNoRedirect,
		"PathSelector 5 names a spine by an index below kNoRedirect");
	NS_ABORT_MSG_IF(CustomHeader::ackRedirectBytes !=
			(IsAckRedirect() ? SpineFieldBytes(m_spineCount) : 0),
		"PathSelector 5 and only it carries the redirect field in every "
		"acknowledgement");
	NS_ABORT_MSG_IF((m_pathSelectorKind ==
				static_cast<uint32_t>(PathSelectorKind::Mrc) || IsMrcHold()) &&
			m_retransmission_timeout_ns == 0,
		"MRC probes at multiples of RetransmissionTimeoutNs");
	NS_ABORT_MSG_IF(IsPathPerPacket() && !CustomHeader::ackCarriesPacketSeq,
		"A LoadBalancing mode other than ECMP finds the send an acknowledgement "
		"answers by the packet sequence the acknowledgement carries");
	NS_ABORT_MSG_IF(m_cc_mode == 11 && !IsPathPerPacket(),
		"CC mode 11 (NSCC) clocks its window on the acknowledgement of each data "
		"packet and times it from the packet's send record, which only a "
		"LoadBalancing mode other than ECMP provides");
	for (uint32_t i = 0; i < m_nic.size(); i++){
		Ptr<QbbNetDevice> dev = m_nic[i].dev;
		if (!dev)
			continue;
		// share data with NIC
		dev->m_rdmaEQ->m_qpGrp = m_nic[i].qpGrp;
		// setup callback
		dev->m_rdmaReceiveCb = MakeCallback(&RdmaHw::Receive, this);
		dev->m_rdmaLinkDownCb = MakeCallback(&RdmaHw::SetLinkDown, this);
		dev->m_rdmaPktSent = MakeCallback(&RdmaHw::PktSent, this);
		// config NIC
		dev->m_rdmaEQ->m_rdmaGetNxtPkt = MakeCallback(&RdmaHw::GetNxtPacket, this);
	}
	// setup qp complete callback
	m_qpCompleteCallback = cb;
	m_qpFailureCallback = failure_cb;
}

uint32_t RdmaHw::GetNicIdxOfQp(Ptr<RdmaQueuePair> qp){
	auto &v = m_rtTable[qp->dip.Get()];
	if (v.size() > 0){
		return v[qp->GetHash() % v.size()];
	}else{
		NS_ASSERT_MSG(false, "We assume at least one NIC is alive");
	}
}
uint64_t RdmaHw::GetQpKey(uint32_t dip, uint16_t sport, uint16_t pg){
	return ((uint64_t)dip << 32) | ((uint64_t)sport << 16) | (uint64_t)pg;
}
Ptr<RdmaQueuePair> RdmaHw::GetQp(uint32_t dip, uint16_t sport, uint16_t pg){
	uint64_t key = GetQpKey(dip, sport, pg);
	auto it = m_qpMap.find(key);
	if (it != m_qpMap.end())
		return it->second;
	return NULL;
}
void RdmaHw::AddQueuePair(uint32_t src, uint32_t dest, uint64_t tag, uint64_t size, uint16_t pg, Ipv4Address sip, Ipv4Address dip, uint16_t sport, uint16_t dport, uint32_t win, uint64_t baseRtt, Callback<void> notifyAppFinish, Callback<void> notifyAppSent, DataRate maxRate){
	// create qp
	Ptr<RdmaQueuePair> qp = CreateObject<RdmaQueuePair>(pg, sip, dip, sport, dport);
	qp->SetSrc(src);
	qp->SetDest(dest);
	qp->SetTag(tag);
	qp->SetSize(size);
	qp->SetInitialSize(size);
	qp->SetWin(win);
	qp->SetBaseRtt(baseRtt);
	qp->SetVarWin(m_var_win);
	qp->SetAppNotifyCallback(notifyAppFinish);
	qp->SetAppSentCallback(notifyAppSent);
	// Congestion response is decided once, at birth. The experiment layer owns
	// eligibility, the step and the budget; the transport supplies a five-tuple
	// and stores the answer.
	// add qp
	uint32_t nic_idx = GetNicIdxOfQp(qp);
	m_nic[nic_idx].qpGrp->AddQp(qp);
	uint64_t key = GetQpKey(dip.Get(), sport, pg);
	m_qpMap[key] = qp;
	// A selector kept from an earlier queue pair to the same destination has
	// its receivers' marks already.
	if (IsPathPerPacket() && StartPathSelection(qp, win, baseRtt) &&
			IsReceiverHold())
		AttachReceiverHold(qp);
	// The liveness invariant starts at birth: an unfinished QP always has a
	// pending timer, even if its first send never gets scheduled.
	ArmRetransmissionTimeout(qp);

	// set init variables
	DataRate m_bps = m_nic[nic_idx].dev->GetDataRate();
	// A cap below the line rate paces the queue pair at it, through the
	// NIC's next-send time (UpdateNextAvail) as any rate does.
	if (maxRate.GetBitRate() > 0 && maxRate < m_bps)
		m_bps = maxRate;
	qp->m_rate = m_bps;
	qp->m_max_rate = m_bps;
	if (m_cc_mode == 1){
		qp->mlx.m_targetRate = m_bps;
	}else if (m_cc_mode == 3){
		qp->hp.m_curRate = m_bps;
		if (m_multipleRate){
			for (uint32_t i = 0; i < IntHeader::maxHop; i++)
				qp->hp.hopState[i].Rc = m_bps;
		}
	}else if (m_cc_mode == 7){
		qp->tmly.m_curRate = m_bps;
	}else if (m_cc_mode == 10){
		qp->hpccPint.m_curRate = m_bps;
	}else if (m_cc_mode == 11){
		// The window the frontend hands over is the path's bandwidth-delay
		// product, which NSCC sizes its own window from. The rate stays at the
		// line rate, so VarWin scales nothing.
		NS_ABORT_MSG_IF(win == 0, "CC mode 11 sizes its window from the queue "
			"pair's bandwidth-delay product");
		StartNscc(qp, win, baseRtt);
	}

	// Notify Nic
	m_nic[nic_idx].dev->NewQp(qp);
}

void RdmaHw::DeleteQueuePair(Ptr<RdmaQueuePair> qp){
	// remove qp from the m_qpMap
	uint64_t key = GetQpKey(qp->dip.Get(), qp->sport, qp->m_pg);
	m_qpMap.erase(key);
}

uint64_t RdmaHw::GetRxQpKey(uint32_t dip, uint16_t pg, uint16_t dport){
	return ((uint64_t)dip << 32) | ((uint64_t)pg << 16) | (uint64_t)dport;
}

Ptr<RdmaRxQueuePair> RdmaHw::GetRxQp(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport, uint16_t pg, bool create){
	uint64_t key = GetRxQpKey(dip, pg, dport);
	auto it = m_rxQpMap.find(key);
	if (it != m_rxQpMap.end())
		return it->second;
	if (create){
		// create new rx qp
		Ptr<RdmaRxQueuePair> q = CreateObject<RdmaRxQueuePair>();
		// init the qp
		q->sip = sip;
		q->dip = dip;
		q->sport = sport;
		q->dport = dport;
		q->m_ecn_source.qIndex = pg;
		// Asked once, in the frontend's own five-tuple order: the sender's
		// address and port first. The answer cannot change within a flow,
		// because eligibility and the step are fixed when it is created.
		q->m_forgiveness_eligible = !m_forgivenessEligibleCallback.IsNull() &&
			m_forgivenessEligibleCallback(dip, sip, dport, sport);
		// Only a packet of the removed queue pair's flow comes this way.
		q->m_stray = m_closedRxQps.count(key) > 0;
		// store in map
		m_rxQpMap[key] = q;
		return q;
	}
	return NULL;
}
uint32_t RdmaHw::GetNicIdxOfRxQp(Ptr<RdmaRxQueuePair> q){
	auto &v = m_rtTable[q->dip];
	if (v.size() > 0){
		return v[q->GetHash() % v.size()];
	}else{
		NS_ASSERT_MSG(false, "We assume at least one NIC is alive");
	}
}
void RdmaHw::DeleteRxQp(uint32_t dip, uint16_t pg, uint16_t dport){
	uint64_t key = GetRxQpKey(dip, pg, dport);
	auto it = m_rxQpMap.find(key);
	if (it == m_rxQpMap.end())
		return;
	if (m_sprayStrayPathSeq == static_cast<uint32_t>(StrayPathSeq::Closed)){
		const Ptr<RdmaRxQueuePair> q = it->second;
		m_closedRxQps[key] = !q->m_tailRuns.Empty() ||
				!q->m_tailMissingRuns.Empty() || q->m_countedTailMissing
			? q : nullptr;
	}
	m_rxQpMap.erase(it);
}

void RdmaHw::PCIeResume(uint32_t nic_idx, uint32_t qIndex){
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
	Ptr<Packet> p = dev->NICSendPfc(qIndex, 1);
	m_nic[nic_idx].dev->RdmaEnqueueHighPrioQ(p);
    m_nic[nic_idx].dev->TriggerTransmit();
    Simulator::Schedule(MicroSeconds(1000), &RdmaHw::EnablePause, this);
}

void RdmaHw::EnablePause(){
	enable_pcie_pause = true;
}

void RdmaHw::PCIePause(uint32_t nic_idx, uint32_t qIndex){
	if (m_paused_times >= m_total_pause_times){
		return ;
	}
	m_paused_times++;
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
	Ptr<Packet> p = dev->NICSendPfc(qIndex, 0);
	m_nic[nic_idx].dev->RdmaEnqueueHighPrioQ(p);
    m_nic[nic_idx].dev->TriggerTransmit();
	std::cout << "NIC pause "<< m_node->GetId() << " at " << Simulator::Now().GetNanoSeconds() << " paused " << m_paused_times << std::endl;
    Simulator::Schedule(MicroSeconds(10), &RdmaHw::PCIeResume, this, nic_idx, qIndex);
}

int RdmaHw::ReceiveUdp(Ptr<Packet> p, CustomHeader &ch){
	uint8_t ecnbits = ch.GetIpv4EcnBits();

	uint32_t payload_size = p->GetSize() - ch.GetSerializedSize();
	// TODO find corresponding rx queue pair
	Ptr<RdmaRxQueuePair> rxQp = GetRxQp(ch.dip, ch.sip, ch.udp.dport, ch.udp.sport, ch.udp.pg, true);
	uint32_t nic_id = GetNicIdxOfRxQp(rxQp);
    if (enable_pcie_pause){ //&& ((m_node->GetId())%16 == 0 || (m_node->GetId())%16 == 8)) { //&& Simulator::Now().GetMicroSeconds() > m_node->GetId(4)
      PCIePause(nic_id, ch.udp.pg);
      enable_pcie_pause = false;
    }
	if (ecnbits != 0){
		rxQp->m_ecn_source.ecnbits |= ecnbits;
		rxQp->m_ecn_source.qfb++;
	}
	rxQp->m_ecn_source.total++;
	CountArrival(rxQp, ch.ipid, payload_size);
	if (m_ackRoute == static_cast<uint32_t>(AckRoute::ReverseSet))
		ProbeAnswerSet(rxQp, ch);
	if (m_sprayAnswerPathSeq)
		AnswerHoldReportsOf().Report(ch.sip, m_switchQueues->LeafOf(ch.sip),
			SpineBitmap::FromBytes(ch.udp.answer_hold,
				CustomHeader::udpAnswerHoldBytes),
			Simulator::Now().GetNanoSeconds());
	if (m_sprayPathSeq)
		CountPathSeq(rxQp, ch);
	if (IsGrading())
		GradeArrival(rxQp, ch, ecnbits != 0);
	rxQp->m_milestone_rx = m_ack_interval;

	// No logging on the non-ACK paths: behind a trim- or drop-induced gap,
	// go-back-N delivers up to a full window of out-of-order packets, so a
	// per-packet line here floods stdout exactly when the fabric is doing
	// what a congestion experiment asks of it.
	// Before the check, because the check is what moves the receive state and
	// the account is of what this packet added to it.
	const uint64_t accepted =
		AcceptedPayloadBytes(rxQp, ch.udp.seq, payload_size);
	const uint64_t range_end = static_cast<uint64_t>(ch.udp.seq) + payload_size;
	// Bytes the receiver gave up on and the sender sent anyway. They are
	// dropped here as any duplicate is; the budget keeps its charge, and the
	// analyzer subtracts these to report what was actually lost.
	const uint64_t late_forgiven = rxQp->ForgivenBytes(ch.udp.seq, range_end);
	// A send the receiver already holds, resent, and not one of a copied
	// send's two packets: its sender did not see the answers.
	if (m_ackRoute == static_cast<uint32_t>(AckRoute::Repath) &&
			accepted == 0 && late_forgiven == 0 && payload_size > 0 &&
			!(ch.udp.tail_flags & kTailCopied))
		RepathAnswers(ch.sip, ch.udp.pg);
	rxQp->NoteSeen(range_end);
	int x = ReceiverCheckSeq(ch.udp.seq, rxQp, payload_size);

	// Scored whether or not it is answered, as the grader counts it.
	const uint16_t redirect = IsAckRedirect()
		? ScoreForRedirect(rxQp, ch, ecnbits != 0) : kNoRedirect;
	if (x == 1 || x == 2){ //generate ACK or NACK
		SendAck(rxQp, ch.dip, ch.sip, ch.udp.dport, ch.udp.sport, ch.udp.pg,
			ch.udp.ih, ch.udp.seq, ch.ipid, x == 2, ecnbits != 0,
			AllowanceGone(rxQp), false, redirect,
			(m_sprayDuplicateReport !=
					static_cast<uint32_t>(DuplicateReport::Off) ||
				IsOverdueRepair()) &&
				accepted == 0 && late_forgiven == 0 && payload_size > 0);
	}
	// After the acknowledgement, so this packet's answer carries the state
	// this packet produced and anything the report below absorbs is
	// acknowledged separately. The report can stop other flows from the same
	// sender, and this one: the frontend's budget decides.
	if ((accepted > 0 || late_forgiven > 0) && !m_dataAcceptedCallback.IsNull())
		m_dataAcceptedCallback(ch.sip, ch.dip, ch.udp.sport, ch.udp.dport,
			accepted, late_forgiven);
	// Asked on every accepted arrival: the frontend answers no until the
	// step's plan says this sender is done, so the transport needs no timer
	// and no threshold of its own. A flow the report above already stopped has
	// no hole left and is refused here.
	if (accepted > 0 && !m_remainderVerdictCallback.IsNull())
		AskRemainderOnArrival(rxQp);
	return 0;
}

void RdmaHw::CountArrival(Ptr<RdmaRxQueuePair> q, uint16_t identification,
		uint32_t payloadSize){
	q->m_data_arrivals++;
	if (!NamesSpine(static_cast<LoadBalancingMode>(m_loadBalancing)))
		return;
	const uint8_t carrying = CarryingSpine(identification);
	if (carrying >= m_spineArrivals.size())
		m_spineArrivals.resize(carrying + 1, SpineArrivals{});
	const bool folded = RequestedSpine(identification) != carrying;
	SpineArrivals &spine = m_spineArrivals[carrying];
	spine.packets++;
	spine.payloadBytes += payloadSize;
	spine.folded += folded;
	q->m_folded_arrivals += folded;
}

bool RdmaHw::IsSprayPolicy() const{
	return m_loadBalancing == static_cast<uint32_t>(LoadBalancingMode::SprayPolicy);
}

bool RdmaHw::IsAckRedirect() const{
	return m_pathSelectorKind ==
		static_cast<uint32_t>(PathSelectorKind::AckRedirect);
}

uint16_t RdmaHw::ScoreForRedirect(Ptr<RdmaRxQueuePair> q,
		const CustomHeader &ch, bool marked){
	if (std::binary_search(m_ownLeafHosts.begin(), m_ownLeafHosts.end(), ch.sip))
		return kNoRedirect;
	// The host's grader has already counted this arrival and closed any
	// interval that ended before it.
	if (IsHostRedirect())
		return m_hostRedirect.OnArrival(CarryingSpine(ch.ipid),
			HostRedirectParameters());
	NS_ABORT_MSG_IF(m_sprayBaseRttNs == 0,
		"PathSelector 5 counts its validity cycle in SprayBaseRttNs");
	const Time now = Simulator::Now();
	// The data packet's send timestamp, and the trimmed header's, which keeps
	// the INT prefix; one clock serves the whole simulation.
	const uint64_t delay = (now - TimeStep(ch.udp.ih.ts)).GetNanoSeconds();
	const SpineRedirect::Parameters parameters{m_ackRedirectOmega,
		m_ackRedirectFlag,
		static_cast<uint64_t>(m_ackRedirectValidityBaseRtts * m_sprayBaseRttNs),
		m_sprayBaseRttNs / 2};
	const uint16_t redirect = q->m_spineRedirect.OnArrival(CarryingSpine(ch.ipid),
		delay, marked, now.GetNanoSeconds(), parameters);
	NS_ASSERT_MSG(redirect == kNoRedirect || redirect < m_spineCount,
		"a redirect names a spine by its index");
	return redirect;
}

bool RdmaHw::IsHostRedirect() const{
	return IsAckRedirect() &&
		m_ackRedirectScope == static_cast<uint32_t>(GradeScope::PerReceiver);
}

bool RdmaHw::IsGrading() const{
	return IsSprayPolicy() || IsHostRedirect();
}

HostSpineRedirect::Parameters RdmaHw::HostRedirectParameters() const{
	return HostSpineRedirect::Parameters{m_ackRedirectOmega, m_ackRedirectFlag,
		m_sprayBaseRttNs / 2, m_sprayOneWayDelay};
}

bool RdmaHw::IsSwitchQueueCost() const{
	return IsSprayPolicy() && m_sprayGradeCostSource ==
		static_cast<uint32_t>(GradeCostSource::SwitchQueue);
}

const SpineReport &RdmaHw::ReportFor(Ptr<RdmaRxQueuePair> q,
		uint32_t destinationIp){
	if (IsSwitchQueueCost()){
		// A sender on this leaf crosses no spine and has no path of its own;
		// before the first close no leaf has been graded.
		const uint32_t leaf = m_switchQueues->LeafOf(destinationIp);
		if (leaf != m_leaf && leaf < m_leafReports.size())
			return m_leafReports[leaf];
	}
	return GraderOf(q).Report();
}

void RdmaHw::GradeFromSwitchQueues(uint64_t nowNs, const SpineGrader &grader){
	const std::vector<SpineGrader::SpineInterval> &closed = grader.LastInterval();
	std::vector<bool> held(closed.size());
	// Under the none held exploration a held spine is sent nothing and so
	// never arrives to release the hold a moved packet set; the spine is held
	// only over an interval in which a packet requested on it was moved.
	const bool none =
		m_sprayHeldExploration == SpineScores::HeldExploration::None;
	for (uint32_t k = 0; k < closed.size(); k++)
		held[k] = none ? closed[k].moved > 0 : closed[k].held;
	// A path losing as large a share of its packets as an absent spine is
	// short of the median's is held as that absence would hold it.
	const double lossHold = 1 - m_sprayAbsenceFractionOfMedian;
	m_leafReports.resize(m_switchQueues->Leaves());
	for (uint32_t leaf = 0; leaf < m_leafReports.size(); leaf++){
		if (leaf == m_leaf)
			continue;
		const std::vector<SpineGrader::SpineValues> values =
			m_switchQueues->PathValues(leaf, m_leaf, m_sprayOneWayDelay);
		SpineReport report = grader.Graded(values, held, lossHold);
		report.edgeCongested |= grader.LastIntervalLastHopTrims() > 0;
		m_leafReports[leaf] = report;
		m_traceSwitchQueueReport(nowNs, grader, leaf, values, report);
	}
}

bool RdmaHw::CountsSprayLoss() const{
	// Under the switches' state the paths' loss is read into the reports, so
	// the sender counts none of its own.
	return m_sprayLossThreshold1 > 0 &&
		((IsSprayPolicy() && !IsSwitchQueueCost()) || IsRepsHold());
}

bool RdmaHw::ValidOverdueRepair() const{
	if (!IsOverdueRepair())
		return true;
	if (m_sprayHeldLossRtts <= 0)
		return false;
	if (IsRepsHold())
		return true;
	// Without a loss hold, the repair that counts nothing alone: REPS, the
	// oracle and the uniform draw.
	const bool uniformDraw = m_loadBalancing ==
			static_cast<uint32_t>(LoadBalancingMode::SprayUniform) &&
		m_pathSelectorKind == static_cast<uint32_t>(PathSelectorKind::Ops);
	return m_sprayOverdueRepair == static_cast<uint32_t>(OverdueRepair::Reprobe) &&
		(IsRepsSpine() || IsRepsOracle() || uniformDraw);
}

bool RdmaHw::IsOverdueRepair() const{
	return m_sprayOverdueRepair != static_cast<uint32_t>(OverdueRepair::Off);
}

bool RdmaHw::IsRepsHold() const{
	return m_pathSelectorKind ==
			static_cast<uint32_t>(PathSelectorKind::RepsHold) ||
		m_pathSelectorKind ==
			static_cast<uint32_t>(PathSelectorKind::RepsFlowHold) ||
		IsMrcHold();
}

bool RdmaHw::IsMrcHold() const{
	return m_pathSelectorKind ==
		static_cast<uint32_t>(PathSelectorKind::MrcHold);
}

bool RdmaHw::ValidRepsHold() const{
	return !IsRepsHold() ||
		(m_loadBalancing ==
				static_cast<uint32_t>(LoadBalancingMode::SprayUniform) &&
			m_sprayLossThreshold1 > 0 && m_sprayLossIntervalNs > 0 &&
			(m_sprayLossHoldCleanSentinels > 0 || IsTrickleRelease()) &&
			m_sprayHeldExploration == SpineScores::HeldExploration::Sentinel &&
			((!IsLeafHold() && !IsAutoHold()) || m_switchQueues));
}

bool RdmaHw::ValidHoldRule() const{
	return m_sprayHoldRule == static_cast<uint32_t>(HoldRule::Count) ||
		(IsRepsHold() && LossLikelihoodRatio::Valid(Likelihood()));
}

bool RdmaHw::IsTrickleRelease() const{
	return m_sprayHoldRelease == static_cast<uint32_t>(HoldRelease::Trickle);
}

bool RdmaHw::ValidHoldRelease() const{
	return !IsTrickleRelease() ||
		(IsRepsHold() && m_sprayLossHoldCleanSentinels == 0 &&
			m_sprayHeldSentinelIntervalNs == 0 && m_sprayHoldTrickle > 0 &&
			m_sprayHoldTrickle <= 0.5 &&
			LossLikelihoodRatio::Valid(Likelihood()));
}

LossLikelihoodRatio::Parameters RdmaHw::Likelihood() const{
	return LossLikelihoodRatio::Parameters{m_sprayLossP0, m_sprayLossP1,
		m_sprayHoldAlpha, m_sprayHoldBeta};
}

bool RdmaHw::IsRepsOracle() const{
	return m_pathSelectorKind ==
		static_cast<uint32_t>(PathSelectorKind::RepsOracle);
}

bool RdmaHw::IsRepsSpine() const{
	return m_pathSelectorKind ==
		static_cast<uint32_t>(PathSelectorKind::RepsSpine);
}

bool RdmaHw::ValidRepsOracle() const{
	if (m_repsOracleHops != static_cast<uint32_t>(OracleHops::RoundTrip) &&
			!IsRepsOracle())
		return false;
	return !IsRepsOracle() ||
		(m_loadBalancing ==
				static_cast<uint32_t>(LoadBalancingMode::SprayUniform) &&
			m_sprayGradeCostSource ==
				static_cast<uint32_t>(GradeCostSource::SwitchQueue) &&
			m_switchQueues);
}

FaultedSpines &RdmaHw::FaultedSpinesTo(uint32_t leaf){
	NS_ABORT_MSG_IF(m_sprayBaseRttNs == 0,
		"PathSelector 8 reads the switches once per SprayBaseRttNs");
	NS_ABORT_MSG_IF(m_leaf == SwitchQueueState::kNoLeaf ||
			leaf == SwitchQueueState::kNoLeaf,
		"PathSelector 8 reads the paths between the source's and the "
		"destination's leaves");
	const bool forward =
		m_repsOracleHops == static_cast<uint32_t>(OracleHops::Forward);
	return m_faultedSpines.try_emplace(leaf, m_spineCount, m_sprayBaseRttNs,
		[this, leaf, forward](){
			return forward ? m_switchQueues->ForwardFaulted(m_leaf, leaf)
				: m_switchQueues->Faulted(m_leaf, leaf);
		})
		.first->second;
}

bool RdmaHw::IsLeafHold() const{
	return (m_pathSelectorKind ==
			static_cast<uint32_t>(PathSelectorKind::RepsHold) || IsMrcHold()) &&
		m_sprayHoldScope == static_cast<uint32_t>(HoldScope::Leaf);
}

bool RdmaHw::IsPairHold() const{
	return (m_pathSelectorKind ==
			static_cast<uint32_t>(PathSelectorKind::RepsHold) || IsMrcHold()) &&
		m_sprayHoldScope == static_cast<uint32_t>(HoldScope::Pair);
}

bool RdmaHw::IsAutoHold() const{
	return (m_pathSelectorKind ==
			static_cast<uint32_t>(PathSelectorKind::RepsHold) || IsMrcHold()) &&
		m_sprayHoldScope == static_cast<uint32_t>(HoldScope::Auto);
}

AutoLossHold &RdmaHw::AutoHold(){
	if (m_autoHold)
		return *m_autoHold;
	m_autoHold = std::make_unique<AutoLossHold>(m_spineCount,
		LossHoldParameters(), m_sprayHoldAutoLeaves);
	m_autoHold->SetLossIntervalCallback(
		[this](uint32_t leaf, uint64_t endNs, const std::vector<uint32_t> &sent,
				const std::vector<uint32_t> &lost,
				const std::vector<uint8_t> &grades){
			m_traceLeafSpineLossInterval(leaf, endNs, sent, lost, grades);
		});
	return *m_autoHold;
}

SpineScores &RdmaHw::LeafHold(uint32_t key){
	const auto [found, made] = m_leafHolds.try_emplace(key, m_spineCount,
		LossHoldParameters());
	SpineScores &hold = found->second;
	if (made)
		hold.SetLossIntervalCallback(
			[this, key](uint64_t endNs, const std::vector<uint32_t> &sent,
					const std::vector<uint32_t> &lost,
					const std::vector<uint8_t> &grades){
				m_traceLeafSpineLossInterval(key, endNs, sent, lost, grades);
			});
	return hold;
}

SpineScores &RdmaHw::DestinationHold(uint32_t destination){
	if (IsPairHold())
		return LeafHold(destination);
	if (IsLeafHold())
		return LeafHold(m_switchQueues->LeafOf(destination));
	return HostScores(LossHoldParameters());
}

uint64_t RdmaHw::SentinelSalt() const{
	return m_node ? m_node->GetId() : 0;
}

SpineScores::Parameters RdmaHw::LossHoldParameters() const{
	// The scores are never drawn from: only their loss counting, loss hold
	// and sentinels are read. A sentinel interval of zero is the loss
	// interval, which stands in for the report interval.
	SpineScores::Parameters parameters{1, 0, 1,
		SpineScores::CandidateDraw::Proportional, m_sprayLossIntervalNs};
	parameters.lossThresholds = SprayLossThresholds();
	parameters.lossIntervalNs = m_sprayLossIntervalNs;
	parameters.lossHoldCleanSentinels = m_sprayLossHoldCleanSentinels;
	parameters.heldExploration = m_sprayHeldExploration;
	parameters.sentinelIntervalNs = m_sprayHeldSentinelIntervalNs;
	parameters.sentinelSpread = m_sprayHeldSentinelSpread;
	parameters.sentinelSalt = SentinelSalt();
	parameters.holdRule = static_cast<HoldRule>(m_sprayHoldRule);
	parameters.holdRelease = static_cast<HoldRelease>(m_sprayHoldRelease);
	parameters.trickleFraction = m_sprayHoldTrickle;
	parameters.sentinelMode = static_cast<SentinelMode>(m_spraySentinelMode);
	parameters.cleanOnAnswers = true;
	if (parameters.holdRule == HoldRule::Llr || IsTrickleRelease())
		parameters.likelihood = Likelihood();
	return parameters;
}

SpineScores &RdmaHw::HostScores(const SpineScores::Parameters &parameters){
	if (m_senderSpineScores)
		return *m_senderSpineScores;
	m_senderSpineScores = std::make_unique<SpineScores>(m_spineCount, parameters);
	if (parameters.lossThresholds)
		m_senderSpineScores->SetLossIntervalCallback(
			[this](uint64_t endNs, const std::vector<uint32_t> &sent,
					const std::vector<uint32_t> &lost,
					const std::vector<uint8_t> &grades){
				m_traceSpineLossInterval(endNs, sent, lost, grades);
			});
	return *m_senderSpineScores;
}

bool RdmaHw::ValidSprayLoss() const{
	return !CountsSprayLoss() || m_selective_retransmission;
}

bool RdmaHw::ValidHeldExploration() const{
	return !IsSprayPolicy() ||
		m_sprayHeldExploration != SpineScores::HeldExploration::None ||
		IsSwitchQueueCost();
}

bool RdmaHw::ValidPoolHoldPropagates() const{
	return !IsSprayPolicy() || !m_sprayPoolHoldPropagates ||
		m_spraySenderPoolWeight > 0;
}

bool RdmaHw::IsConnectionPathState() const{
	return m_sprayPathState == static_cast<uint32_t>(PathState::Connection);
}

bool RdmaHw::ValidPathState() const{
	return !IsConnectionPathState() || m_loadBalancing ==
		static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
}

bool RdmaHw::ValidTailLoss() const{
	return m_sprayTailLossRtts == 1 || m_sprayTailRecovery;
}

bool RdmaHw::ValidAckLoss() const{
	return m_sprayAckLossRtts == 0 || m_loadBalancing ==
		static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
}

bool RdmaHw::ValidCopyExpiry() const{
	return m_sprayCopyExpiry == static_cast<uint32_t>(CopyExpiry::DataAck) ||
		(m_spraySentinelMode == static_cast<uint32_t>(SentinelMode::Duplicate) &&
			m_sprayTailLossRtts > 0);
}

bool RdmaHw::ValidAckRoute() const{
	const bool spray = m_ackRoute == static_cast<uint32_t>(AckRoute::Spray);
	// Every route but the data's names its spine in the answer.
	const bool named = m_ackRoute != static_cast<uint32_t>(AckRoute::DataSpine);
	if (CustomHeader::ackRouteSpineBytes !=
			(named ? SpineFieldBytes(m_spineCount) : 0))
		return false;
	if (m_ackRouteOracle && (!spray || !m_switchQueues ||
			m_leaf == SwitchQueueState::kNoLeaf))
		return false;
	return !named || (m_loadBalancing ==
			static_cast<uint32_t>(LoadBalancingMode::SprayUniform) &&
		m_ackRouteRandom && m_spineCount < kNoRouteSpine);
}

FaultedSpines &RdmaHw::AnswerFaultedSpinesTo(uint32_t leaf){
	NS_ABORT_MSG_IF(m_sprayBaseRttNs == 0,
		"AckRouteOracle reads the switches once per SprayBaseRttNs");
	NS_ABORT_MSG_IF(leaf == SwitchQueueState::kNoLeaf,
		"AckRouteOracle reads the paths toward the sender's leaf");
	return m_answerFaultedSpines.try_emplace(leaf, m_spineCount,
		m_sprayBaseRttNs,
		[this, leaf](){ return m_switchQueues->ForwardFaulted(m_leaf, leaf); })
		.first->second;
}

uint16_t RdmaHw::AnswerRouteSpine(uint32_t senderIp, uint16_t pg){
	if (m_ackRoute == static_cast<uint32_t>(AckRoute::DataSpine) ||
			!AnswersCrossSpine(senderIp))
		return kNoRouteSpine;
	if (m_ackRoute == static_cast<uint32_t>(AckRoute::ReverseSet))
		return AnswerRoutesOf(senderIp, pg).set.Route(m_spineCount,
			m_ackRouteSetSize, [this](uint32_t n){ return DrawAnswerRoute(n); });
	if (m_ackRoute == static_cast<uint32_t>(AckRoute::Repath))
		return AnswerRoutesOf(senderIp, pg).repath.Route(m_spineCount,
			[this](uint32_t n){ return DrawAnswerRoute(n); });
	if (!m_ackRouteOracle && !m_sprayAnswerPathSeq)
		return m_ackRouteRandom->GetInteger(0, m_spineCount - 1);
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	const uint32_t leaf = m_switchQueues->LeafOf(senderIp);
	// The spines this leaf's senders hold, but for a probe due on one.
	SpineBitmap held;
	if (m_sprayAnswerPathSeq){
		AnswerHoldReports &reports = AnswerHoldReportsOf();
		uint8_t probe;
		if (reports.TakeProbe(leaf, now, m_sprayAnswerProbeIntervalNs, probe)){
			ReportTransportEvent("answer_probe", 0);
			return probe;
		}
		held = reports.Held(leaf);
	}
	FaultedSpines *faulted = nullptr;
	if (m_ackRouteOracle){
		faulted = &AnswerFaultedSpinesTo(leaf);
		faulted->Read(now);
	}
	uint8_t allowed[UINT8_MAX + 1];
	uint32_t count = 0;
	for (uint32_t spine = 0; spine < m_spineCount; spine++){
		if ((!faulted || !faulted->Excludes(spine)) &&
				!held.Test(spine))
			allowed[count++] = spine;
	}
	// While every spine left is held, none is left out for being held.
	if (count == 0){
		for (uint32_t spine = 0; spine < m_spineCount; spine++){
			if (!faulted || !faulted->Excludes(spine))
				allowed[count++] = spine;
		}
	}
	return allowed[m_ackRouteRandom->GetInteger(0, count - 1)];
}

std::optional<std::array<double, 3>> RdmaHw::SprayLossThresholds() const{
	if (m_sprayLossThreshold1 == 0)
		return std::nullopt;
	return std::array<double, 3>{m_sprayLossThreshold1, m_sprayLossThreshold2,
		m_sprayLossThreshold3};
}

bool RdmaHw::IsPerConnectionGrading() const{
	return IsSprayPolicy() &&
		m_sprayGradeScope == static_cast<uint32_t>(GradeScope::PerConnection);
}

uint64_t RdmaHw::SprayReportIntervalNs() const{
	NS_ABORT_MSG_IF(m_sprayBaseRttNs == 0,
		"LoadBalancing 3 counts its report interval in SprayBaseRttNs");
	return static_cast<uint64_t>(m_sprayReportIntervalBaseRtts * m_sprayBaseRttNs);
}

// Read at the first event, because the base RTT is set only after every host
// is built.
SpineGrader::Parameters RdmaHw::GraderParameters() const{
	const double rtt = m_sprayBaseRttNs;
	SpineGrader::Parameters parameters;
	parameters.spines = m_spineCount;
	parameters.intervalNs = SprayReportIntervalNs();
	// Receivers whose intervals end together move their senders' shares
	// together.
	parameters.phaseNs =
		static_cast<uint64_t>(PhaseOf(m_node->GetId()) * parameters.intervalNs);
	parameters.fractions = {m_sprayEstimatorGain, m_sprayFractionCusumSlack,
		m_sprayFractionCusumThreshold, m_sprayEstimatorIntervalSamples};
	parameters.congestionThresholds[0] = m_sprayCongestionThreshold1;
	parameters.congestionThresholds[1] = m_sprayCongestionThreshold2;
	parameters.congestionThresholds[2] = m_sprayCongestionThreshold3;
	if (m_sprayTrimThreshold1 > 0)
		parameters.trimThresholds = std::array<double, 3>{m_sprayTrimThreshold1,
			m_sprayTrimThreshold2, m_sprayTrimThreshold3};
	parameters.lossThresholds = SprayLossThresholds();
	parameters.congestionGradeFloor = m_sprayCongestionGradeFloor;
	parameters.oneWayDelay = m_sprayOneWayDelay;
	parameters.delayNs = {m_sprayEstimatorGain,
		m_sprayDelayCusumSlackBaseRtts * rtt,
		m_sprayDelayCusumThresholdBaseRtts * rtt, m_sprayEstimatorIntervalSamples};
	parameters.delayThresholdsNs[0] = m_sprayDelayThreshold1BaseRtts * rtt;
	parameters.delayThresholdsNs[1] = m_sprayDelayThreshold2BaseRtts * rtt;
	parameters.delayThresholdsNs[2] = m_sprayDelayThreshold3BaseRtts * rtt;
	parameters.reference =
		static_cast<SpineGrader::GradeReference>(m_sprayGradeReference);
	// Under the switches' state the paths' loss stands in for the deficit of
	// arrivals, so arrivals hold no spine for their absence.
	parameters.absenceFractionOfMedian =
		IsSwitchQueueCost() ? 0 : m_sprayAbsenceFractionOfMedian;
	parameters.absenceMinimumMedian = m_sprayAbsenceMinimumMedian;
	parameters.releaseFractionOfFloor = m_sprayReleaseFractionOfFloor;
	// A held spine is drawn at epsilon / N and a top-graded one at about 1 / N.
	// Under the sentinel and the none held exploration a held spine has no
	// share of the epsilon term: it is sent the sentinel rate, one packet per
	// sentinel interval per set of scores, or nothing. Sentinels arrive once
	// per interval however many the median spine does, so under sentinel a
	// held spine is released by one arrival over a window whenever the floor
	// share asks more; under none the switch_queue cost source's hold does
	// not wait for arrivals.
	parameters.floorShareOfMedian = m_sprayEpsilon;
	parameters.releaseOnOneArrival =
		m_sprayHeldExploration == SpineScores::HeldExploration::Sentinel;
	return parameters;
}

SpineGrader &RdmaHw::Grader(){
	if (!m_spineGrader)
		m_spineGrader = std::make_unique<SpineGrader>(GraderParameters());
	return *m_spineGrader;
}

// A connection's grader keeps its host's phase, so the scope changes whose
// arrivals are counted and not when the intervals close.
SpineGrader &RdmaHw::GraderOf(Ptr<RdmaRxQueuePair> q){
	if (!IsPerConnectionGrading())
		return Grader();
	NS_ASSERT_MSG(q, "a connection's grader belongs to its receive queue pair");
	if (!q->m_spineGrader)
		q->m_spineGrader = std::make_unique<SpineGrader>(GraderParameters());
	return *q->m_spineGrader;
}

SpineGrader &RdmaHw::GraderNow(Ptr<RdmaRxQueuePair> q){
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	SpineGrader &grader = GraderOf(q);
	if (grader.Advance(now)){
		m_traceSpineReport(now, grader,
			IsPerConnectionGrading() ? PeekPointer(q) : nullptr);
		if (IsHostRedirect())
			m_hostRedirect.OnClose(grader.LastInterval(), HostRedirectParameters());
		if (IsSwitchQueueCost())
			GradeFromSwitchQueues(now, grader);
	}
	return grader;
}

void RdmaHw::GradeArrival(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch,
		bool marked){
	// One clock serves the whole simulation, so the difference is the delay.
	const uint64_t delay = m_sprayOneWayDelay
		? (Simulator::Now() - TimeStep(ch.udp.ih.ts)).GetNanoSeconds() : 0;
	GraderNow(q).OnArrival(RequestedSpine(ch.ipid), CarryingSpine(ch.ipid),
		marked, delay);
}

void RdmaHw::GradeTrim(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch,
		bool lastHop){
	GraderNow(q).OnTrim(CarryingSpine(ch.ipid), lastHop);
}

// The receiver's cumulative acknowledgement, shared by the in-order data path
// and the forgiveness fork. The forgive path passes cnp true: the ACK it
// emits carries FLAG_CNP, so ReceiveAck delivers a congestion signal exactly
// as a repaired trim would and forgiving does not hide congestion. The debt is
// paid by the ACK the forgive emits, not carried on whichever ACK comes next.
// It also carries the allowance report, because the charge that empties a
// budget entry is a forgiveness and no repair request follows it.
void RdmaHw::SendAck(Ptr<RdmaRxQueuePair> q, uint32_t sourceIp,
		uint32_t destinationIp, uint16_t sport, uint16_t dport, uint16_t pg,
		const IntHeader &ih, uint32_t packetSeq, uint16_t identification,
		bool nack, bool cnp, bool spent, bool probeAnswer, uint16_t redirect,
		bool duplicate){
	qbbHeader seqh;
	if (duplicate)
		seqh.SetDuplicate();
	seqh.SetSeq(q->ReceiverNextExpectedSeq);
	seqh.SetPacketSeq(packetSeq);
	seqh.SetPG(pg);
	seqh.SetSport(sport);
	seqh.SetDport(dport);
	seqh.SetIntHeader(ih);
	if (cnp)
		seqh.SetCnp();
	if (probeAnswer)
		seqh.SetProbeAnswer();
	seqh.SetAllowanceExhausted(spent);
	if (IsSprayPolicy())
		seqh.SetSpineReport(ReportFor(q, destinationIp));
	seqh.SetRedirect(redirect);
	bool holdPartial = false;
	seqh.SetReceiverHold(ReceiverHoldReport(destinationIp, holdPartial));
	seqh.SetReceiverHoldPartial(holdPartial);
	seqh.SetRouteSpine(AnswerRouteSpine(destinationIp, pg));
	if (m_sprayAnswerPathSeq)
		NumberAnswer(q, seqh, seqh.GetRouteSpine());
	// The grant is this flag without the report above. A receiver that may not
	// forgive this flow, or a step that is critical, never sets it, so those
	// senders obey their controller throughout.
	seqh.SetForgivenessEligible(q->m_forgiveness_eligible);

	Ptr<Packet> newp = Create<Packet>(std::max(60-14-20-(int)seqh.GetSerializedSize(), 0));
	newp->AddHeader(seqh);

	Ipv4Header head;	// Prepare IPv4 header
	head.SetDestination(Ipv4Address(destinationIp));
	head.SetSource(Ipv4Address(sourceIp));
	head.SetProtocol(nack ? 0xFD : 0xFC); //ack=0xFC nack=0xFD
	head.SetTtl(64);
	// UEC 1.0.3 Table 3-76: PDS ACKs and NACKs use DSCP_CONTROL (TC_high) and
	// MUST NOT be marked as trimmable.
	head.SetDscp(static_cast<Ipv4Header::DscpType>(kUetDscpControl));
	head.SetPayloadSize(newp->GetSize());
	// Where each data packet draws its path, the acknowledgement returns the
	// identification of the packet it answers, which names that path.
	head.SetIdentification(IsPathPerPacket() ? identification : q->m_ipid++);

	newp->AddHeader(head);
	AddHeader(newp, 0x800);	// Attach PPP header
	// send
	uint32_t nic_idx = GetNicIdxOfRxQp(q);
	m_nic[nic_idx].dev->RdmaEnqueueHighPrioQ(newp);
	m_nic[nic_idx].dev->TriggerTransmit();
}

int RdmaHw::ReceiveCnp(Ptr<Packet> p, CustomHeader &ch){
	// QCN on NIC
	// This is a Congestion signal
	// Then, extract data from the congestion packet.
	// We assume, without verify, the packet is destinated to me
	uint32_t qIndex = ch.cnp.qIndex;
	if (qIndex == 1){		//DCTCP
		return 0;
	}
	uint16_t udpport = ch.cnp.fid; // corresponds to the sport
	uint8_t ecnbits = ch.cnp.ecnBits;
	uint16_t qfb = ch.cnp.qfb;
	uint16_t total = ch.cnp.total;

	uint32_t i;
	// get qp
	Ptr<RdmaQueuePair> qp = GetQp(ch.sip, udpport, qIndex);
	if (!qp)
		std::cout << "ERROR: QCN NIC cannot find the flow\n";
	// get nic
	uint32_t nic_idx = GetNicIdxOfQp(qp);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;

	if (qp->m_rate == 0)			//lazy initialization
	{
		qp->m_rate = dev->GetDataRate();
		if (m_cc_mode == 1){
			qp->mlx.m_targetRate = dev->GetDataRate();
		}else if (m_cc_mode == 3){
			qp->hp.m_curRate = dev->GetDataRate();
			if (m_multipleRate){
				for (uint32_t i = 0; i < IntHeader::maxHop; i++)
					qp->hp.hopState[i].Rc = dev->GetDataRate();
			}
		}else if (m_cc_mode == 7){
			qp->tmly.m_curRate = dev->GetDataRate();
		}else if (m_cc_mode == 10){
			qp->hpccPint.m_curRate = dev->GetDataRate();
		}
	}
	return 0;
}

int RdmaHw::ReceiveAck(Ptr<Packet> p, CustomHeader &ch){
	// A receiver's probe of its answers' route acknowledges nothing.
	if (m_ackRoute == static_cast<uint32_t>(AckRoute::ReverseSet) &&
			((ch.ack.flags >> qbbHeader::FLAG_ROUTE_PROBE) & 1)){
		ReplyRouteProbe(ch);
		return 0;
	}
	uint16_t qIndex = ch.ack.pg;
	uint16_t port = ch.ack.dport;
	uint32_t seq = ch.ack.seq;
	uint8_t cnp = (ch.ack.flags >> qbbHeader::FLAG_CNP) & 1;
	int i;
	Ptr<RdmaQueuePair> qp = GetQp(ch.sip, port, qIndex);
	if (!qp){
		// The receiver answers a send that arrives after the queue pair
		// completed, as a duplicate; the answer of a send a tail repair
		// request declared lost can still drop that loss.
		if (!m_tailLosses.empty() &&
				!((ch.ack.flags >> qbbHeader::FLAG_PROBE_ANSWER) & 1))
			CancelTailLoss(GetQpKey(ch.sip, port, qIndex), nullptr,
				ch.ack.packet_seq,
				PathOf(static_cast<LoadBalancingMode>(m_loadBalancing), ch.ipid),
				TailLossEnd::Acknowledged);
		return 0;
	}

	uint32_t nic_idx = GetNicIdxOfQp(qp);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
	if (m_sprayAnswerPathSeq)
		ReadAnswerSeq(qp, ch);
	// Both flags report on the path, not on the bytes this ACK covers, so they
	// are read before the acknowledgement that may complete the queue pair and
	// return. A forgiven trim rides its own ACK, and that ACK is often the one
	// that closes the transfer; reading the report first is what lets the
	// congestion signal riding the same ACK reach the controller.
	FollowAllowanceReport(qp, ch);
	if (cnp && m_cc_mode == 1 && DeliverCongestionSignal(qp)){ // mlx version
		cnp_received_mlx(qp);
	}
	if (m_ack_interval == 0)
		std::cout << "ERROR: shouldn't receive ack\n";
	else {
		// Before the cumulative advance, which would remove the answered send
		// without asking what was sent ahead of it on its path. Under
		// SprayCoverageFirst that advance is coverage the rules apply first.
		if (m_sprayCoverageFirst)
			qp->m_answerCoverage = seq;
		if (qp->m_outstanding.IsKept()){
			const uint16_t path =
				PathOf(static_cast<LoadBalancingMode>(m_loadBalancing), ch.ipid);
			const uint64_t now = Simulator::Now().GetNanoSeconds();
			if (IsSprayPolicy())
				qp->m_pathSelector->OnReport(ReportOf(ch), now);
			if (IsReceiverHold())
				qp->m_pathSelector->OnReceiverHold(ch.sip,
					SpineBitmap::FromBytes(ch.ack.receiver_hold,
						CustomHeader::ackReceiverHoldSlots > 0 ? kSpineBitmapBytes
							: CustomHeader::ackReceiverHoldBytes),
					ch.ack.receiver_hold_partial, now);
			uint64_t copyLost = 0;
			if ((ch.ack.flags >> qbbHeader::FLAG_PROBE_ANSWER) & 1){
				qp->m_pathSelector->OnProbeAnswer(path, cnp, now);
			}else if (qp->AcknowledgeCopy(ch.ack.packet_seq, path, copyLost)){
				// A copy's answer reaches the selector as a data exploration
				// send's does, but not the window: the data send of its
				// sequence has its own record and answer.
				qp->m_pathSelector->OnAck(path, cnp, now);
				if (copyLost > 0 && m_cc_mode == 11 && DeliverCongestionSignal(qp)){
					qp->Nscc().OnLoss(copyLost);
					ApplyNsccWindow(qp);
				}
			}else{
				// A send a tail repair request declared lost that is
				// answered arrived: it had waited in a queue longer than the
				// receiver's delay. Its record is gone, so this looks first.
				if (!m_tailLosses.empty())
					CancelTailLoss(GetQpKey(qp->dip.Get(), qp->sport, qp->m_pg),
						qp, ch.ack.packet_seq, path, TailLossEnd::Acknowledged);
				if (IsAckRedirect())
					qp->m_pathSelector->OnRedirect(
						DecodeRedirect(ch.ack.redirect, m_spineCount),
						CarryingSpine(ch.ipid), now);
				qp->m_pathSelector->OnAck(path, cnp, now);
				// The answer to the resend of a timed-out send, flagged where
				// the first send had arrived.
				if (!qp->m_timedOut.empty() || !qp->m_timeoutLosses.empty())
					qp->ResolveTimedOut(ch.ack.packet_seq,
						(ch.ack.flags >> qbbHeader::FLAG_DUPLICATE) & 1,
						ch.ack.route_spine != kNoRouteSpine ? ch.ack.route_spine
							: CarryingSpine(ch.ipid), now);
				// An answer lets the next overdue send be repaired.
				if (IsOverdueRepair()){
					qp->m_overdueSpent = false;
					qp->m_overdueRepeats = 0;
					if (qp->OverdueRepairSpurious(ch.ack.packet_seq, path,
							(ch.ack.flags >> qbbHeader::FLAG_DUPLICATE) & 1, now))
						ReportTransportEvent("overdue_repair_spurious", 0);
				}
				// The destination's downlink being congested is the receiver's
				// flow control of UEC 1.0.3 section 3.6.13.2.
				const uint32_t rcvCwndPend = IsSprayPolicy() &&
						((ch.ack.flags >> qbbHeader::FLAG_EDGE_CONGESTED) & 1)
					? m_sprayEdgeWindowPenalty : 0;
				if (m_cc_mode == 11 && DeliverCongestionSignal(qp))
					HandleAckNscc(qp, ch.ack.packet_seq, path, cnp, rcvCwndPend);
				else
					qp->AcknowledgePacket(ch.ack.packet_seq, path);
				// The answer to a send the cumulative acknowledgement covered
				// first finds no record; the selector has heard it above.
				if (!qp->m_covered.empty())
					qp->AnswerCovered(ch.ack.packet_seq, path);
			}
		}
		const uint64_t acknowledged_before = qp->snd_una;
		const bool keepCovered = m_sprayAckLossRtts > 0;
		if (!m_backto0){
			qp->Acknowledge(seq, keepCovered);
		}else {
			uint32_t goback_seq = seq / m_chunk * m_chunk;
			qp->Acknowledge(goback_seq, keepCovered);
		}
		qp->m_answerCoverage = 0;
		if (qp->snd_una > acknowledged_before){
			qp->m_recovery_retries = 0;
			qp->m_last_progress_ns = Simulator::Now().GetNanoSeconds();
		}
		if (qp->IsFinished()){
			Simulator::Cancel(qp->m_retransmissionTimer);
			QpComplete(qp);
			return 0;
		}
		if (!qp->m_covered.empty())
			ArmAckLossTimer(qp);
	}
	if (ch.l3Prot == 0xFD){ // NACK
		if (EnforceProgressDeadline(qp)){
			return 0;
		}
		if (m_selective_retransmission){
			// The NACK names the gap head (its cumulative sequence) but not
			// the gap's extent, so repair one packet at the head; successive
			// NACKs walk the gap and trim notifications carry exact ranges.
			uint64_t gap_start = qp->snd_una;
			uint64_t gap_end = gap_start + m_mtu;
			if (gap_end > qp->m_size)
				gap_end = qp->m_size;
			qp->AddRepairRange(gap_start, gap_end);
		}else{
			RecoverQueue(qp);
		}
	}

	// Modes 3, 7, 8 and 10 read congestion off every acknowledgement, so the
	// dispatch itself is the delivery and the exemption is checked once here.
	if (m_cc_mode == 3 || m_cc_mode == 7 || m_cc_mode == 8 ||
			m_cc_mode == 10){
		if (DeliverCongestionSignal(qp)){
			if (m_cc_mode == 3){
				HandleAckHp(qp, p, ch);
			}else if (m_cc_mode == 7){
				HandleAckTimely(qp, p, ch);
			}else if (m_cc_mode == 8){
				HandleAckDctcp(qp, p, ch);
			}else{
				HandleAckHpPint(qp, p, ch);
			}
		}
	}
	// ACK may advance the on-the-fly window, allowing more packets to send
	ArmRetransmissionTimeout(qp);
	dev->TriggerTransmit();
	//std:://cout << "ack triggere transmitted\n";
	return 0;
}

int RdmaHw::Receive(Ptr<Packet> p, CustomHeader &ch){
	// UEC 1.0.3 section 3.5.15.1: "Trimmed packet MUST be recognized based on
	// the ip.dscp field". The IP and UDP lengths of a trimmed packet MUST NOT be
	// verified, and the packet MUST NOT reach normal request processing.
	if (IsUetTrimmedDscp(ch.GetIpv4Dscp())){
		return ReceiveTrim(p, ch);
	}
	if (ch.l3Prot == 0x11){ // UDP
		ReceiveUdp(p, ch);
	}else if (ch.l3Prot == kUecTrimRepairProtocol){
		ReceiveTrim(p, ch);
	}else if (ch.l3Prot == 0xFF){ // CNP
		ReceiveCnp(p, ch);
	}else if (ch.l3Prot == 0xFD){ // NACK
		ReceiveAck(p, ch);
	}else if (ch.l3Prot == 0xFC){ // ACK
		ReceiveAck(p, ch);
	}else if (ch.l3Prot == kPathProbeProtocol){
		ReceivePathProbe(p, ch);
	}
	return 0;
}

// A reliability probe (OCP MRC 1.0 section 7.4.6) is answered with an
// acknowledgement that returns its identification and its mark and moves no
// receive state. A probe for a flow this receiver holds no state for is
// dropped: the flow has finished.
int RdmaHw::ReceivePathProbe(Ptr<Packet> p, CustomHeader &ch){
	(void)p;
	// A route probe of this host's answers, returned by their sender.
	if (m_ackRoute == static_cast<uint32_t>(AckRoute::ReverseSet) &&
			((ch.ack.flags >> qbbHeader::FLAG_ROUTE_PROBE_REPLY) & 1)){
		AnswerRoutesOf(ch.sip, ch.ack.pg).set.OnReply(ch.ack.route_spine);
		return 0;
	}
	Ptr<RdmaRxQueuePair> q = GetRxQp(ch.dip, ch.sip, ch.ack.dport, ch.ack.sport,
		ch.ack.pg, false);
	if (q == nullptr)
		return 0;
	SendAck(q, ch.dip, ch.sip, ch.ack.dport, ch.ack.sport, ch.ack.pg,
		IntHeader(), 0, ch.ipid, false, ch.GetIpv4EcnBits() != 0,
		AllowanceGone(q), true, kNoRedirect);
	return 0;
}

// UEC 1.0.3 Table 3-61: a trimmed RUD/ROD/RUDI Request MUST produce a NACK with
// pds.nack_code UET_TRIMMED or UET_TRIMMED_LASTHOP. The NACK is a control packet
// (DSCP_CONTROL / TC_high) and carries only the identity of the lost packet.
void RdmaHw::SendTrimNack(Ptr<RdmaRxQueuePair> q, const CustomHeader &ch,
		uint32_t sourceIp, uint32_t destinationIp, uint16_t sport,
		uint16_t dport, uint16_t pg, uint32_t seq, uint32_t payloadSize,
		bool lastHop, bool spent, bool eligible){
	qbbHeader repair;
	repair.SetSeq(seq);
	repair.SetPG(pg);
	repair.SetSport(sport);
	repair.SetDport(dport);
	repair.SetTrimPayloadSize(payloadSize);
	repair.SetTrimLastHop(lastHop);
	// UEC 1.0.3 section 3.6.12.4: the NACK reports whether the trimmed packet
	// arrived with CE, which a switch before the trimming one may have set.
	if (ch.GetIpv4EcnBits() != 0)
		repair.SetCnp();
	repair.SetAllowanceExhausted(spent);
	// The report is only a report where the receiver says it may forgive this
	// flow; without this flag a clear bit means nothing, because a receiver
	// with no budget at all also sends one.
	repair.SetForgivenessEligible(eligible);
	if (IsSprayPolicy())
		repair.SetSpineReport(ReportFor(q, destinationIp));
	SendRepairRequest(repair, sourceIp, destinationIp, ch.ipid, q);
}

void RdmaHw::SendRepairRequest(const qbbHeader &header, uint32_t sourceIp,
		uint32_t destinationIp, uint16_t identification,
		Ptr<RdmaRxQueuePair> q){
	qbbHeader routed = header;
	routed.SetRouteSpine(AnswerRouteSpine(destinationIp, header.GetPG()));
	if (m_sprayAnswerPathSeq)
		NumberAnswer(q, routed, routed.GetRouteSpine());
	Ptr<Packet> packet = Create<Packet>(
		std::max(60 - 14 - 20 - static_cast<int>(routed.GetSerializedSize()), 0));
	packet->AddHeader(routed);

	Ipv4Header ipHeader;
	ipHeader.SetSource(Ipv4Address(sourceIp));
	ipHeader.SetDestination(Ipv4Address(destinationIp));
	ipHeader.SetProtocol(kUecTrimRepairProtocol);
	ipHeader.SetDscp(static_cast<Ipv4Header::DscpType>(kUetDscpControl));
	ipHeader.SetPayloadSize(packet->GetSize());
	ipHeader.SetTtl(64);
	ipHeader.SetIdentification(identification);
	packet->AddHeader(ipHeader);
	AddHeader(packet, 0x800);

	auto route = m_rtTable.find(destinationIp);
	if (route == m_rtTable.end() || route->second.empty()) {
		return;
	}
	const uint32_t nicIdx = route->second[0];
	m_nic[nicIdx].dev->RdmaEnqueueHighPrioQ(packet);
	m_nic[nicIdx].dev->TriggerTransmit();
}

// The sender follows the latest report. A receiver that marks this flow
// forgivable and has allowance left grants the exemption; the same receiver
// reporting the allowance gone puts the sender back under its controller, and
// a later report with room lets it withhold signals again. The controller
// raises its own rate on its own timers when it hears nothing, so nothing
// inside it is touched either way. A packet from a receiver that does not
// mark the flow says nothing about forgiveness and is left alone, which is
// every flow in every arm without the exemption.
//
// One round trip is spent obeying at the start of each flow, because the
// first report cannot arrive sooner.
void RdmaHw::FollowAllowanceReport(Ptr<RdmaQueuePair> qp,
		const CustomHeader &ch){
	const bool gone = (ch.ack.flags >> qbbHeader::FLAG_ALLOWANCE_EXHAUSTED) & 1;
	if (gone){
		qp->m_allowance_gone_reports++;
		ReportTransportEvent("allowance_gone_reports", 0);
	}
	if (!m_congestionExemption)
		return;
	if (!((ch.ack.flags >> qbbHeader::FLAG_FORGIVENESS_ELIGIBLE) & 1))
		return;
	if (qp->m_cc_report_seen && gone != qp->m_cc_last_report){
		qp->m_cc_transitions++;
		ReportTransportEvent("cc_transition", 0);
	}
	qp->m_cc_report_seen = true;
	qp->m_cc_last_report = gone;
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	// The reference arm that never re-engages. Every report is still carried
	// and still counted, so the two arms differ in the sender's reaction and
	// in nothing else.
	if (gone && !m_reengage)
		return;
	if (gone == !qp->m_cc_exempt)
		return;
	if (gone){
		qp->m_cc_exempt = false;
		qp->m_cc_obey_since_ns = now;
		return;
	}
	qp->m_cc_exempt = true;
	if (qp->m_cc_exempt_granted_ns == 0){
		qp->m_cc_exempt_granted_ns = now;
		ReportTransportEvent("cc_exempt_granted", 0);
		return;
	}
	// Back under the exemption after a stretch of obeying, which is what the
	// obeying time is the sum of.
	qp->m_cc_obeying_ns += now - qp->m_cc_obey_since_ns;
	qp->m_cc_obey_since_ns = 0;
}

// The receiver's report, asked for wherever an acknowledgement or a repair
// request leaves it. A queue pair the experiment layer does not mark has no
// budget to report on, and its sender obeys its controller throughout.
bool RdmaHw::AllowanceGone(Ptr<RdmaRxQueuePair> q){
	if (m_allowanceGoneCallback.IsNull() || !q->m_forgiveness_eligible)
		return false;
	return m_allowanceGoneCallback(q->dip, q->sip, q->dport, q->sport,
		q->Holes());
}

bool RdmaHw::DeliverCongestionSignal(Ptr<RdmaQueuePair> qp){
	if (!qp->m_cc_exempt)
		return true;
	qp->m_cc_signals_withheld++;
	ReportTransportEvent("cc_signal_withheld", 0);
	return false;
}

void RdmaHw::RecoverTrimmedQueue(Ptr<RdmaQueuePair> qp,
		const CustomHeader &ch){
	// Before the stale check, because a stale report still describes the
	// entry, and before the rate cut below, so this trim's own congestion
	// signal reaches the controller.
	FollowAllowanceReport(qp, ch);
	const bool lastHop = (ch.ack.flags >> qbbHeader::FLAG_TRIM_LASTHOP) & 1;
	const uint16_t path =
		PathOf(static_cast<LoadBalancingMode>(m_loadBalancing), ch.ipid);
	// A stale trim reports on its path all the same.
	if (qp->m_outstanding.IsKept()){
		const uint64_t now = Simulator::Now().GetNanoSeconds();
		if (IsSprayPolicy())
			qp->m_pathSelector->OnReport(ReportOf(ch), now);
		qp->m_pathSelector->OnTrim(path, lastHop,
			(ch.ack.flags >> qbbHeader::FLAG_CNP) & 1, now);
		// A send a tail repair request declared lost was trimmed, not lost:
		// the selector is not told of it. Its repair is already queued.
		CancelTailLoss(GetQpKey(qp->dip.Get(), qp->sport, qp->m_pg), qp,
			ch.ack.seq, path, TailLossEnd::Trimmed);
	}
	const uint64_t trimStart = ch.ack.seq;
	const uint64_t trimEnd = trimStart + ch.ack.trim_payload_size;
	if (ch.ack.trim_payload_size == 0 || trimEnd <= qp->snd_una){
		qp->m_stale_trim_notifications++;
		return;
	}
	if (EnforceProgressDeadline(qp)){
		return;
	}
	// A trim notification never consumes the retry budget. Like a NACK, it
	// is proof the path is alive: the switch chose to report congestion
	// instead of staying silent, which is the entire point of trimming.
	// The budget bounds consecutive *silent* retransmission timeouts, the
	// only signal consistent with a dead path. Counting notifications here
	// would fail a queue pair at the sender's own send rate during any
	// sustained blockade — e.g. while a shared-buffer switch fair-shares
	// its pool against an incast burst — turning engineered congestion
	// into spurious transport failure.
	qp->m_trim_notifications++;
	if (qp->m_first_trim_ns == 0)
		qp->m_first_trim_ns = Simulator::Now().GetNanoSeconds();
	qp->m_trimmed_payload_bytes += ch.ack.trim_payload_size;
	if (lastHop) {
		qp->m_trim_lasthop_notifications++;
	}
	qp->m_trim_recovery_events++;
	// UEC 1.0.3 p. 356 excludes DSCP_TRIMMED_LASTHOP from the congestion signal
	// only where RCCC covers the last hop; without RCCC, dropping the cut is
	// what leaves destination incast entirely uncontrolled, so mode 1 reacts to
	// every trim.
	if (m_cc_mode == 1 && DeliverCongestionSignal(qp)) {
		cnp_received_mlx(qp);
	}
	// Timed before the release below removes the trimmed send's record.
	const uint64_t trimmedRtt = m_cc_mode == 11
		? AnswerRtt(qp, qp->m_outstanding.Find(trimStart, path))
		: NsccWindow::kNoRtt;
	if (m_selective_retransmission){
		// The notification names the exact trimmed byte range, so repair
		// only that range instead of rewinding the whole window. The
		// receiver accepts the out-of-order remainder, so nothing else
		// needs resending. With send records, a trimmed send that is no longer
		// outstanding was already declared lost and is being repaired.
		if (!qp->m_outstanding.IsKept() || qp->ReleasePacket(trimStart, path)){
			qp->m_recovery_events++;
			qp->AddRepairRange(trimStart, trimEnd);
		}
	}else{
		RecoverQueue(qp);
	}
	// After the release, so that the bytes QuickAdapt leaves in flight no
	// longer count the trimmed send. A trim whose send was already declared
	// lost is congestion all the same, and cuts the window again.
	if (m_cc_mode == 11 && DeliverCongestionSignal(qp)){
		qp->Nscc().OnTrim(ch.ack.trim_payload_size, trimmedRtt,
			qp->WindowOnTheFly(), Simulator::Now().GetNanoSeconds());
		ApplyNsccWindow(qp);
	}
	const uint32_t nicIdx = GetNicIdxOfQp(qp);
	m_nic[nicIdx].dev->TriggerTransmit();
	ArmRetransmissionTimeout(qp);
}

// A trimmed data packet at its destination, under the recovery domain. The
// range is settled (received, or already forgiven) and acknowledged as a
// duplicate, or it is asked about. The verdict is recomputed on every
// request rather than cached, because the allowance moves under it: charges
// shrink it and later launches grow it. Recomputing is also what a lost
// first request needs.
//
// The verdict is asked about the unsettled bytes, never the whole trimmed
// range. A repair re-segmenter can trim a range that straddles the cumulative
// sequence or overlaps a range already accepted, and charging its full length
// would spend budget on bytes the receiver already holds.
void RdmaHw::ReceiveTrimmedData(const CustomHeader &ch, uint32_t payloadSize,
		bool lastHop){
	// Never create: the flow that owned this receive queue pair may have
	// completed, and resurrecting it would leave a stale cumulative sequence
	// for whichever later flow reuses the source port. A trim with no receive
	// state gets the plain PULL the non-forgiveness path would have sent.
	Ptr<RdmaRxQueuePair> q = GetRxQp(ch.dip, ch.sip, ch.udp.dport, ch.udp.sport,
		ch.udp.pg, false);
	if (q == nullptr){
		SendTrimNack(q, ch, ch.dip, ch.sip, ch.udp.dport, ch.udp.sport,
			ch.udp.pg, ch.udp.seq, payloadSize, lastHop, false, false);
		return;
	}
	const uint64_t start = ch.udp.seq;
	const uint64_t end = start + payloadSize;
	// A trimmed payload is evidence the range exists just as an arrival is,
	// so it raises the frontier the holes are measured below.
	q->NoteSeen(end);
	const uint64_t unsettled = q->UnsettledBytes(start, end);
	if (unsettled == 0){
		SendAck(q, ch.dip, ch.sip, ch.udp.dport, ch.udp.sport, ch.udp.pg,
			ch.udp.ih, ch.udp.seq, ch.ipid, false, false, AllowanceGone(q), false,
			kNoRedirect);
		return;
	}
	if (unsettled < payloadSize){
		// The clip fired: the range partly overlaps what the receiver holds.
		// Counted so a fixture can prove it exercised the path rather than
		// asserting an identity that holds vacuously.
		ReportTransportEvent("clipped_trim", 0);
	}
	const bool forgive = !m_recoveryVerdictCallback.IsNull() &&
		m_recoveryVerdictCallback(ch.sip, ch.dip, ch.udp.sport, ch.udp.dport,
			start, static_cast<uint32_t>(unsettled));
	if (forgive){
		// Forgiving is absorbing: the range joins the accepted out-of-order
		// set as though it had arrived, so the cumulative sequence can pass it
		// and no repair is ever requested. It absorbs exactly the unsettled
		// bytes, which is what the ledger was charged, and the same bytes are
		// recorded as given up before the absorb hides them.
		q->NoteForgiven(start, end);
		q->AddOutOfOrderRange(start, end);
		// Beside the switch's trim_ftd_* events, so the reader can subtract:
		// W' = (trimmed - forgiven) / offered.
		ReportTransportEvent("trim_forgiven", static_cast<uint32_t>(unsettled));
		const uint64_t expected = static_cast<uint64_t>(q->ReceiverNextExpectedSeq);
		if (start <= expected){
			q->ReceiverNextExpectedSeq =
				static_cast<uint32_t>(q->AbsorbContiguousFrom(expected));
		}
		// UEC 1.0.3 p. 356 keeps a last-hop trim out of the congestion signal
		// only where RCCC covers the last hop; this transport has none, so the
		// debt is owed for every trim the sender would otherwise have seen, and
		// this ACK is what pays it.
		SendAck(q, ch.dip, ch.sip, ch.udp.dport, ch.udp.sport, ch.udp.pg,
			ch.udp.ih, ch.udp.seq, ch.ipid, false, true, AllowanceGone(q), false,
			kNoRedirect);
		return;
	}
	// The refused range stays a hole, so the report is computed after the
	// frontier moved and counts it.
	SendTrimNack(q, ch, ch.dip, ch.sip, ch.udp.dport, ch.udp.sport, ch.udp.pg,
		ch.udp.seq, payloadSize, lastHop, AllowanceGone(q),
		q->m_forgiveness_eligible);
}

// The step stop's question. The receive queue pair knows how far its
// cumulative sequence has reached and not how large the flow is, so it asks
// with the former and the frontend answers with the end offset. Absorbing that
// range is what a forgiven trim already does, and the acknowledgement it emits
// carries the flow size, so the sender completes through the IsFinished it
// already had.
void RdmaHw::AskRemainderOnArrival(Ptr<RdmaRxQueuePair> q){
	const uint64_t expected = q->ReceiverNextExpectedSeq;
	// Under selective repeat the flow stalls on a gap while later packets keep
	// arriving, so the bytes above the cumulative sequence are not all
	// missing. The frontend is told how many of them arrived, because charging
	// them would spend the budget on bytes the receiver already holds.
	const uint64_t end = m_remainderVerdictCallback(q->dip, q->sip, q->dport,
		q->sport, expected, q->AcceptedBytesAbove(expected));
	// A refusal emits nothing. No repair is requested here, so no exemption
	// ends on it; the next trim carries the allowance report if the cap is
	// spent.
	if (end <= expected)
		return;
	// Exactly what the absorb below takes, and exactly what the frontend
	// charged, so the two sinks agree by construction rather than by the two
	// sides computing the hole the same way.
	const uint64_t forgiven = q->UnsettledBytes(expected, end);
	q->NoteForgiven(expected, end);
	q->AddOutOfOrderRange(expected, end);
	q->NoteSeen(end);
	q->ReceiverNextExpectedSeq =
		static_cast<uint32_t>(q->AbsorbContiguousFrom(expected));
	// The stop is this queue pair's last word. Nothing it sends afterwards
	// grants an exemption or moves one, because the flow it spoke for is over.
	q->m_forgiveness_eligible = false;
	// Beside trim_forgiven, and separate from it: these bytes were never put
	// on the wire, so they were never trimmed and W does not count them.
	ReportTransportEvent("remainder_forgiven", forgiven);
	// No congestion notification. A trim owes the sender the rate cut the
	// fabric would have signalled; an unsent byte congested nothing. The ECN
	// account's queue index is the priority group GetRxQp created this queue
	// pair with, and the group is part of the map key, so every packet that
	// finds this queue pair carries that same group.
	SendAck(q, q->sip, q->dip, q->sport, q->dport, q->m_ecn_source.qIndex,
		IntHeader(), 0, 0, false, false, false, false, kNoRedirect);
}

// The frontend names a flow by (sender, receiver, sender's port); the receive
// queue pair map is keyed by the priority group as well, which the frontend
// does not carry, so this scans. A receiver holds tens of open receive queue
// pairs and a stop fires once per sender per step, so the scan is cheaper than
// carrying a group through the frontend to make the key.
bool RdmaHw::StopFlow(uint32_t sip, uint32_t dip, uint16_t sport,
		uint16_t dport){
	if (m_remainderVerdictCallback.IsNull())
		return false;
	for (auto &entry : m_rxQpMap){
		Ptr<RdmaRxQueuePair> q = entry.second;
		if (q->dip != sip || q->sip != dip || q->dport != sport ||
				q->sport != dport)
			continue;
		AskRemainderOnArrival(q);
		return true;
	}
	return false;
}

int RdmaHw::ReceiveTrim(Ptr<Packet> p, CustomHeader &ch){
	(void)p;
	// A trimmed data packet reaching its destination. UEC 1.0.3 section 3.5.8.2
	// step 2 and section 3.5.15.1: send a NACK, do not advance any receive state,
	// do not establish a PDC, and drop the packet.
	if (ch.l3Prot == 0x11){
		const uint32_t udpHeaderBytes = ch.UdpHeaderBytes();
		// The UDP length field survives trimming unmodified, so it still reports
		// the size of the original packet that was dropped.
		const uint32_t originalPayload = ch.udp.payload_size > udpHeaderBytes
			? ch.udp.payload_size - udpHeaderBytes : 0;
		const bool lastHop = ch.GetIpv4Dscp() == kUetDscpTrimmedLastHop;
		// A trim can reach a flow before any of its data does; the receive
		// state it makes is what that data would have made.
		const Ptr<RdmaRxQueuePair> q =
				IsPerConnectionGrading() || IsAckRedirect() || m_sprayPathSeq
			? GetRxQp(ch.dip, ch.sip, ch.udp.dport, ch.udp.sport, ch.udp.pg, true)
			: nullptr;
		// The trimmed header keeps the path sequence: the trim keeps at least
		// the header's bytes of IP payload (CustomHeader::UdpHeaderBytes). It only marks
		// that sequence as accounted, since it can overtake earlier data of
		// its path; it counts no gap.
		if (m_sprayPathSeq)
			CountPathSeq(q, ch, true);
		if (IsGrading())
			GradeTrim(q, ch, lastHop);
		// A last-hop trim says nothing of the spine it crossed, and the repair
		// request carries no redirect in any case. The host's redirect takes
		// trims through its grader.
		if (IsAckRedirect() && !IsHostRedirect() && !lastHop &&
				m_ackRedirectTrimScoring ==
				static_cast<uint32_t>(TrimScoring::Mark))
			ScoreForRedirect(q, ch, true);
		if (m_forgiveness && originalPayload > 0){
			ReceiveTrimmedData(ch, originalPayload, lastHop);
			return 0;
		}
		SendTrimNack(q, ch, ch.dip, ch.sip, ch.udp.dport, ch.udp.sport,
			ch.udp.pg, ch.udp.seq, originalPayload, lastHop, false, false);
		return 0;
	}

	// The repair request the destination returned.
	Ptr<RdmaQueuePair> qp = GetQp(ch.sip, ch.ack.dport, ch.ack.pg);
	if (!qp && !m_tailLosses.empty() &&
			!((ch.ack.flags >> qbbHeader::FLAG_TAIL_NACK) & 1)){
		// The trim notification of a send of a completed queue pair, whose
		// loss a tail repair request declared, can still drop that loss.
		CancelTailLoss(GetQpKey(ch.sip, ch.ack.dport, ch.ack.pg), nullptr,
			ch.ack.seq,
			PathOf(static_cast<LoadBalancingMode>(m_loadBalancing), ch.ipid),
			TailLossEnd::Trimmed);
	}
	if (!qp || qp->IsFailed()) {
		return 0;
	}
	if (m_sprayAnswerPathSeq)
		ReadAnswerSeq(qp, ch);
	if (m_sprayTailRecovery &&
			((ch.ack.flags >> qbbHeader::FLAG_TAIL_NACK) & 1)){
		RecoverTail(qp, ch);
		return 0;
	}
	RecoverTrimmedQueue(qp, ch);
	return 0;
}

uint64_t RdmaHw::AcceptedPayloadBytes(Ptr<RdmaRxQueuePair> q, uint32_t seq,
		uint32_t size) const {
	// A packet below the cumulative sequence is a duplicate the receiver
	// discards, and without selective retransmission one above it is dropped
	// rather than accepted. What is left is the part of the range the receiver
	// does not already hold, which is what AddOutOfOrderRange absorbs.
	if (seq < q->ReceiverNextExpectedSeq)
		return 0;
	if (seq > q->ReceiverNextExpectedSeq && !m_selective_retransmission)
		return 0;
	return q->UnsettledBytes(seq, static_cast<uint64_t>(seq) + size);
}

int RdmaHw::ReceiverCheckSeq(uint32_t seq, Ptr<RdmaRxQueuePair> q, uint32_t size){
	uint32_t expected = q->ReceiverNextExpectedSeq;
	if (seq == expected){
		uint64_t advanced = static_cast<uint64_t>(expected) + size;
		if (m_selective_retransmission){
			// Filling the gap head releases every contiguous out-of-order
			// range behind it, so the cumulative sequence can jump.
			advanced = q->AbsorbContiguousFrom(advanced);
		}
		q->ReceiverNextExpectedSeq = static_cast<uint32_t>(advanced);
		if (q->ReceiverNextExpectedSeq >= static_cast<uint32_t>(q->m_milestone_rx)){
			// Single step, as the original transport did. A lagging milestone
			// only means the next packets also generate cumulative ACKs, which
			// is harmless; catching it up in a loop costs one iteration per
			// ack-interval byte on EVERY in-order packet (payload/interval
			// iterations at L2_ACK_INTERVAL 1), a hot-path tax with no
			// behavioral benefit.
			q->m_milestone_rx += m_ack_interval;
			return 1; //Generate ACK
		}else if (IsPathPerPacket() ||
				q->ReceiverNextExpectedSeq % m_chunk == 0){
			return 1;
		}else {
			return 5;
		}
	} else if (seq > expected) {
		if (m_selective_retransmission){
			// Accept the out-of-order payload; only its gap needs repair.
			q->AddOutOfOrderRange(seq, static_cast<uint64_t>(seq) + size);
		}
		// Packets that draw their paths one by one arrive out of order without
		// any loss, so the gap is acknowledged like any arrival and the sender
		// infers loss per path.
		if (IsPathPerPacket())
			return 1;
		// Generate NACK.
		if (Simulator::Now() >= q->m_nackTimer || q->m_lastNACK != expected){
			q->m_nackTimer = Simulator::Now() + MicroSeconds(m_nack_interval);
			q->m_lastNACK = expected;
			if (m_backto0 && !m_selective_retransmission){
				q->ReceiverNextExpectedSeq = q->ReceiverNextExpectedSeq / m_chunk*m_chunk;
			}
			return 2;
		}
		return 4;
	}
	// A duplicate usually means that the sender did not observe a previous
	// ACK. Repeat the cumulative ACK so timeout recovery can terminate.
	return 1;
}
void RdmaHw::AddHeader (Ptr<Packet> p, uint16_t protocolNumber){
	PppHeader ppp;
	ppp.SetProtocol (EtherToPpp (protocolNumber));
	p->AddHeader (ppp);
}
uint16_t RdmaHw::EtherToPpp (uint16_t proto){
	switch(proto){
		case 0x0800: return 0x0021;   //IPv4
		case 0x86DD: return 0x0057;   //IPv6
		default: NS_ASSERT_MSG (false, "PPP Protocol number not defined!");
	}
	return 0;
}

void RdmaHw::RecoverQueue(Ptr<RdmaQueuePair> qp){
	if (qp->IsFailed())
		return;
	qp->m_recovery_events++;
	qp->snd_nxt = qp->snd_una;
}

// Recovery signals prove the path is alive, so NACKs and trim notifications
// are exempt from the silent-loss retry budget — but signal liveness is not
// transfer liveness. A queue pair can cycle recovery signals forever without
// ever advancing snd_una (the flagship livelock: one QP, an otherwise idle
// fabric, simulated time racing to hours with zero progress). This deadline
// bounds the one thing every healthy recovery mode eventually produces:
// cumulative acknowledgement progress. It fails the queue pair with its
// counters on record so the sustaining condition is attributable, instead of
// letting the run burn wall clock until an external timeout kills it blind.
bool RdmaHw::EnforceProgressDeadline(Ptr<RdmaQueuePair> qp){
	if (m_no_progress_timeout_ns == 0 || qp->IsFinished() || qp->IsFailed()){
		return false;
	}
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	if (now - qp->m_last_progress_ns < m_no_progress_timeout_ns){
		return false;
	}
	std::cerr << "No forward progress for " << (now - qp->m_last_progress_ns)
		<< " ns on QP " << qp->m_src << "->" << qp->m_dest
		<< " sport=" << qp->sport
		<< " snd_una=" << qp->snd_una
		<< " snd_nxt=" << qp->snd_nxt
		<< " size=" << qp->m_size
		<< " recovery_events=" << qp->m_recovery_events
		<< " recovery_retries=" << qp->m_recovery_retries
		<< " trim_notifications=" << qp->m_trim_notifications
		<< " stale_trim_notifications=" << qp->m_stale_trim_notifications
		<< " retransmitted_bytes=" << qp->m_retransmitted_bytes
		<< " rate=" << qp->m_rate.GetBitRate() << "bps\n";
	QpFail(qp, static_cast<uint32_t>(RdmaFailureReason::NoForwardProgress));
	return true;
}

// An unfinished queue pair must ALWAYS carry a live timer. The previous
// version cancelled the timer and re-armed nothing once snd_nxt equaled
// snd_una — go-back-N recovery's exact post-state — so a queue pair whose
// resend never happened (window-bound, rate-limited, device busy) was left
// with no pending event of any kind: no RTO, no deadline check, nothing.
// The CI wedge signature was a lone active QP and events_delta collapsing
// to the qlen monitor alone while simulated time raced for hours. A
// deadline enforced only from the victim's own events cannot fire on a
// victim with no events; the always-armed timer is what carries it.
void RdmaHw::ArmRetransmissionTimeout(Ptr<RdmaQueuePair> qp){
	if (m_retransmission_timeout_ns == 0 || qp->IsFinished() || qp->IsFailed())
		return;
	// Removed rather than cancelled: the timer is re-armed on every send and
	// answer, and a cancelled event stays in the scheduler until its time.
	if (!qp->m_retransmissionTimer.IsExpired())
		Simulator::Remove(qp->m_retransmissionTimer);
	// With send records the timer expires when the oldest outstanding send
	// has waited out the timeout, however recently anything else was sent.
	uint64_t delay = m_retransmission_timeout_ns;
	const uint64_t oldest = qp->m_outstanding.Oldest();
	if (oldest != OutstandingPackets::kNone){
		const uint64_t due = qp->m_outstanding.SentNs(oldest) + delay;
		const uint64_t now = Simulator::Now().GetNanoSeconds();
		delay = due > now ? due - now : 0;
	}
	qp->m_retransmissionTimer = Simulator::Schedule(NanoSeconds(delay),
		&RdmaHw::HandleRetransmissionTimeout, this, qp);
}

void RdmaHw::HandleRetransmissionTimeout(Ptr<RdmaQueuePair> qp){
	if (qp->IsFinished() || qp->IsFailed())
		return;
	const Ptr<RdmaQueuePair> active = GetQp(qp->dip.Get(), qp->sport, qp->m_pg);
	if (active != qp)
		return;
	if (EnforceProgressDeadline(qp)){
		return;
	}
	if (qp->GetOnTheFly() == 0){
		// Nothing outstanding: the QP is waiting to (re)send — rate limiter,
		// window, or a busy device. Not a silent-loss retry, so the budget
		// is untouched; keep the timer alive so the deadline above stays
		// enforceable and nudge the NIC in case its next-send event was
		// never scheduled.
		const uint32_t nic_idx = GetNicIdxOfQp(qp);
		m_nic[nic_idx].dev->TriggerTransmit();
		ArmRetransmissionTimeout(qp);
		return;
	}
	if (qp->m_recovery_retries >= m_max_retransmission_retries){
		QpFail(qp, static_cast<uint32_t>(RdmaFailureReason::TimeoutRetryExhausted));
		return;
	}
	qp->m_recovery_retries++;
	qp->m_timeouts++;
	ReportTransportEvent("rto_fired", 0);
	qp->m_overdueSpent = false;
	qp->m_overdueRepeats = 0;
	{
		// The send the timer waited for is the oldest outstanding one.
		const uint64_t oldest = qp->m_outstanding.IsKept()
			? qp->m_outstanding.Oldest() : OutstandingPackets::kNone;
		const bool named = oldest != OutstandingPackets::kNone &&
			NamesSpine(static_cast<LoadBalancingMode>(m_loadBalancing));
		m_traceRetransmissionTimeout(Simulator::Now().GetNanoSeconds(), qp,
			named ? static_cast<int32_t>(qp->m_outstanding.Path(oldest)) : -1,
			oldest != OutstandingPackets::kNone &&
				qp->m_outstanding.Exploration(oldest),
			qp->GetOnTheFly());
	}
	if (qp->m_outstanding.IsKept()){
		const uint64_t now = Simulator::Now().GetNanoSeconds();
		const uint64_t lost =
			qp->DeclareLostSentBy(now - m_retransmission_timeout_ns);
		NS_ASSERT_MSG(lost > 0, "the timer expires with the oldest send");
		qp->m_pathSelector->OnTimeout(now);
		if (m_cc_mode == 11 && DeliverCongestionSignal(qp)){
			qp->Nscc().OnLoss(lost);
			ApplyNsccWindow(qp);
		}
	}else{
		RecoverQueue(qp);
	}
	const uint32_t nic_idx = GetNicIdxOfQp(qp);
	m_nic[nic_idx].dev->TriggerTransmit();
	ArmRetransmissionTimeout(qp);
}

void RdmaHw::HeldSendDue(Ptr<RdmaQueuePair> qp, uint64_t seq, uint16_t path,
		uint64_t sentNs){
	if (qp->IsFinished() || qp->IsFailed())
		return;
	const uint64_t lost = qp->DeclareHeldSendLost(seq, path, sentNs);
	if (lost == 0)
		return;
	if (m_cc_mode == 11 && DeliverCongestionSignal(qp)){
		qp->Nscc().OnLoss(lost);
		ApplyNsccWindow(qp);
	}
	m_nic[GetNicIdxOfQp(qp)].dev->TriggerTransmit();
	ArmRetransmissionTimeout(qp);
}

void RdmaHw::TakeOverdueSend(Ptr<RdmaQueuePair> qp, uint64_t seq,
		uint16_t path, uint64_t sentNs, bool held, bool repair){
	// Under reprobe the repair of the sequence repaired is taken again.
	const bool again = repair && qp->m_overdueRepeat && seq == qp->m_overdueSeq &&
		qp->m_overdueRepeats < m_sprayOverdueRepairLimit;
	if (qp->m_overdueSpent && !again)
		return;
	// A held send's own timer repairs it.
	qp->m_overdueTaken = !held;
	if (held)
		return;
	qp->m_overdueSeq = seq;
	qp->m_overduePath = path;
	qp->m_overdueSentNs = sentNs;
	if (!qp->m_overdueTimer.IsPending())
		qp->m_overdueTimer = Simulator::Schedule(NanoSeconds(
				static_cast<uint64_t>(m_sprayHeldLossRtts * qp->m_baseRtt)),
			&RdmaHw::OverdueSendDue, this, qp);
}

void RdmaHw::OverdueSendDue(Ptr<RdmaQueuePair> qp){
	if (qp->IsFinished() || qp->IsFailed() || !qp->m_overdueTaken)
		return;
	const uint64_t dueNs = qp->m_overdueSentNs +
		static_cast<uint64_t>(m_sprayHeldLossRtts * qp->m_baseRtt);
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	// A later send was taken since the timer was set.
	if (dueNs > now){
		qp->m_overdueTimer = Simulator::Schedule(NanoSeconds(dueNs - now),
			&RdmaHw::OverdueSendDue, this, qp);
		return;
	}
	qp->m_overdueTaken = false;
	const uint64_t lost = qp->DeclareOverdueSendLost(qp->m_overdueSeq,
		qp->m_overduePath, qp->m_overdueSentNs);
	if (lost == 0)
		return;
	qp->m_overdueSpent = true;
	qp->m_overdueRepeats++;
	ReportTransportEvent("overdue_repair", 0);
	if (m_cc_mode == 11 && DeliverCongestionSignal(qp)){
		qp->Nscc().OnLoss(lost);
		ApplyNsccWindow(qp);
	}
	m_nic[GetNicIdxOfQp(qp)].dev->TriggerTransmit();
	ArmRetransmissionTimeout(qp);
}

void RdmaHw::ArmAckLossTimer(Ptr<RdmaQueuePair> qp){
	if (qp->m_covered.empty() || qp->m_ackLossTimer.IsPending())
		return;
	const uint64_t dueNs = qp->m_covered.front().coveredNs +
		static_cast<uint64_t>(m_sprayAckLossRtts * qp->m_baseRtt);
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	qp->m_ackLossTimer = Simulator::Schedule(
		NanoSeconds(dueNs > now ? dueNs - now : 0), &RdmaHw::AckLossDue, this,
		qp);
}

void RdmaHw::AckLossDue(Ptr<RdmaQueuePair> qp){
	if (qp->IsFinished() || qp->IsFailed())
		return;
	const uint32_t declared = qp->DeclareAckLosses(
		Simulator::Now().GetNanoSeconds(),
		static_cast<uint64_t>(m_sprayAckLossRtts * qp->m_baseRtt));
	for (uint32_t i = 0; i < declared; i++)
		ReportTransportEvent("ack_loss_declared", 0);
	ArmAckLossTimer(qp);
}

void RdmaHw::DeferTailLosses(Ptr<RdmaQueuePair> qp,
		const std::vector<RdmaQueuePair::DeclaredSend> &sends,
		uint64_t guardNs){
	if (sends.empty())
		return;
	const uint64_t key = GetQpKey(qp->dip.Get(), qp->sport, qp->m_pg);
	const uint64_t dueNs = Simulator::Now().GetNanoSeconds() + guardNs;
	for (const RdmaQueuePair::DeclaredSend &send : sends)
		m_tailLosses.emplace(dueNs, TailLoss{key, qp, send.seq, send.path,
			send.trickle, qp->dip.Get()});
	ArmTailLossTimer();
}

bool RdmaHw::CancelTailLoss(uint64_t key, Ptr<RdmaQueuePair> qp, uint64_t seq,
		uint16_t path, TailLossEnd end){
	for (auto entry = m_tailLosses.begin(); entry != m_tailLosses.end();
			++entry){
		const TailLoss &loss = entry->second;
		if (loss.key != key || loss.seq != seq || loss.path != path ||
				(loss.qp != qp && !loss.qp->IsFinished()))
			continue;
		const bool repairedOnPath = loss.repairedOnPath;
		m_tailLosses.erase(entry);
		if (end == TailLossEnd::Trimmed){
			m_tailLossesCancelled++;
			ReportTransportEvent("tail_loss_cancelled", 0);
		}else if (repairedOnPath){
			m_tailLossesAmbiguous++;
			ReportTransportEvent("tail_loss_ambiguous", 0);
		}else{
			m_tailLossesDelivered++;
			ReportTransportEvent("tail_loss_delivered", 0);
		}
		ArmTailLossTimer();
		return true;
	}
	return false;
}

void RdmaHw::NoteTailRepair(Ptr<RdmaQueuePair> qp, uint64_t seq,
		uint16_t path){
	for (auto &entry : m_tailLosses){
		TailLoss &loss = entry.second;
		if (loss.qp == qp && loss.seq == seq && loss.path == path)
			loss.repairedOnPath = true;
	}
}

bool RdmaHw::TailLossWaitsOn(Ptr<RdmaQueuePair> qp, uint64_t seq,
		uint16_t path) const{
	for (const auto &entry : m_tailLosses){
		const TailLoss &loss = entry.second;
		if (loss.qp == qp && loss.seq == seq && loss.path == path)
			return true;
	}
	return false;
}

uint16_t RdmaHw::RerouteTailRepair(Ptr<RdmaQueuePair> qp, uint64_t seq,
		uint16_t identification, uint64_t nowNs){
	const LoadBalancingMode mode = static_cast<LoadBalancingMode>(m_loadBalancing);
	if (!TailLossWaitsOn(qp, seq, PathOf(mode, identification)))
		return identification;
	// Chosen before the path sequence and the outstanding record are taken,
	// so that both follow the path finally used.
	for (uint32_t draw = 0; draw < m_spineCount; ++draw){
		identification = qp->m_pathSelector->Choose(nowNs);
		m_tailRepairRedraws++;
		if (!TailLossWaitsOn(qp, seq, PathOf(mode, identification))){
			m_tailRepairsRerouted++;
			ReportTransportEvent("tail_repair_rerouted", 0);
			break;
		}
	}
	return identification;
}

void RdmaHw::DropTailLosses(Ptr<RdmaQueuePair> qp){
	for (auto entry = m_tailLosses.begin(); entry != m_tailLosses.end();){
		if (entry->second.qp == qp)
			entry = m_tailLosses.erase(entry);
		else
			++entry;
	}
	ArmTailLossTimer();
}

void RdmaHw::ArmTailLossTimer(){
	Simulator::Cancel(m_tailLossTimer);
	if (m_tailLosses.empty())
		return;
	const uint64_t dueNs = m_tailLosses.begin()->first;
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	m_tailLossTimer = Simulator::Schedule(
		NanoSeconds(dueNs > now ? dueNs - now : 0), &RdmaHw::TailLossDue, this);
}

void RdmaHw::TailLossDue(){
	// Each loss is told at the time it is told, which is the time the
	// selector and the loss holds count it in.
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	while (!m_tailLosses.empty() && m_tailLosses.begin()->first <= now){
		const TailLoss loss = m_tailLosses.begin()->second;
		m_tailLosses.erase(m_tailLosses.begin());
		TellTailLoss(loss, now);
	}
	ArmTailLossTimer();
}

void RdmaHw::TellTailLoss(const TailLoss &loss, uint64_t nowNs){
	if (loss.qp->IsFailed())
		return;
	// Under SprayPathState connection the queue pair's selector is its
	// connection's, which outlives it.
	if (!loss.qp->IsFinished() || IsConnectionPathState()){
		loss.qp->m_pathSelector->OnLoss(loss.path, nowNs);
		if (loss.trickle)
			loss.qp->m_pathSelector->OnTrickleAnswer(loss.path, true, nowNs);
		return;
	}
	// The hold of RepsFlowHold was the queue pair's own.
	if (!IsRepsHold() || m_pathSelectorKind ==
			static_cast<uint32_t>(PathSelectorKind::RepsFlowHold))
		return;
	if (IsAutoHold()){
		const uint32_t leaf = m_switchQueues->LeafOf(loss.destination);
		AutoHold().CountLoss(leaf, loss.path, nowNs);
		if (loss.trickle)
			AutoHold().TrickleAnswer(leaf, loss.path, true, nowNs);
		return;
	}
	SpineScores &hold = DestinationHold(loss.destination);
	hold.CountIntervalLoss(loss.path, nowNs);
	if (loss.trickle)
		hold.TrickleAnswer(loss.path, true, nowNs);
}

uint32_t RdmaHw::TailLossesWaiting(Ptr<RdmaQueuePair> qp) const{
	uint32_t waiting = 0;
	for (const auto &entry : m_tailLosses)
		waiting += entry.second.qp == qp;
	return waiting;
}

void RdmaHw::ReportTransportEvent(const char* event, uint64_t bytes){
	if (!m_transportEventCallback.IsNull())
		m_transportEventCallback(event, bytes);
}

void RdmaHw::QpComplete(Ptr<RdmaQueuePair> qp){
	NS_ASSERT(!m_qpCompleteCallback.IsNull());
	Simulator::Cancel(qp->m_retransmissionTimer);
	Simulator::Cancel(qp->m_ackLossTimer);
	Simulator::Cancel(qp->m_overdueTimer);
	qp->m_covered.clear();
	LeaveNsccConnection(qp);
	// The losses of qp still waiting in m_tailLosses stay: the trim
	// notification of a send can still follow, and m_tailLosses tells them
	// when due.
	if (m_cc_mode == 1){
		Simulator::Cancel(qp->mlx.m_eventUpdateAlpha);
		Simulator::Cancel(qp->mlx.m_eventDecreaseRate);
		Simulator::Cancel(qp->mlx.m_rpTimer);
	}

	// This callback will log info
	// It may also delete the rxQp on the receiver
	m_qpCompleteCallback(qp);

	qp->m_notifyAppFinish();

	// delete the qp
	DeleteQueuePair(qp);
}

void RdmaHw::QpFail(Ptr<RdmaQueuePair> qp, uint32_t reason){
	if (qp->IsFinished() || qp->IsFailed())
		return;
	NS_ASSERT(!m_qpFailureCallback.IsNull());
	qp->m_failed = true;
	qp->m_failure_reason = reason;
	Simulator::Cancel(qp->m_retransmissionTimer);
	Simulator::Cancel(qp->m_ackLossTimer);
	Simulator::Cancel(qp->m_overdueTimer);
	qp->m_covered.clear();
	DropTailLosses(qp);
	LeaveNsccConnection(qp);
	if (m_cc_mode == 1){
		Simulator::Cancel(qp->mlx.m_eventUpdateAlpha);
		Simulator::Cancel(qp->mlx.m_eventDecreaseRate);
		Simulator::Cancel(qp->mlx.m_rpTimer);
	}
	m_qpFailureCallback(qp, reason);
	DeleteQueuePair(qp);
}

void RdmaHw::SetLinkDown(Ptr<QbbNetDevice> dev){
	printf("RdmaHw: node:%u a link down\n", m_node->GetId());
}

void RdmaHw::AddTableEntry(Ipv4Address &dstAddr, uint32_t intf_idx){
	uint32_t dip = dstAddr.Get();
	m_rtTable[dip].push_back(intf_idx);
}

void RdmaHw::ClearTable(){
	m_rtTable.clear();
}

void RdmaHw::RedistributeQp(){
	// clear old qpGrp
	for (uint32_t i = 0; i < m_nic.size(); i++){
		if (!m_nic[i].dev)
			continue;
		m_nic[i].qpGrp->Clear();
	}

	// redistribute qp
	for (auto &it : m_qpMap){
		Ptr<RdmaQueuePair> qp = it.second;
		uint32_t nic_idx = GetNicIdxOfQp(qp);
		m_nic[nic_idx].qpGrp->AddQp(qp);
		// Notify Nic
		m_nic[nic_idx].dev->ReassignedQp(qp);
	}
}

Ptr<Packet> RdmaHw::GetNxtPacket(Ptr<RdmaQueuePair> qp){
	if (qp->IsFailed())
		return 0;
	// Selective repairs are served before new data: the receiver is waiting
	// on exactly these bytes to advance its cumulative sequence.
	uint64_t seq = qp->snd_nxt;
	uint32_t payload_size = 0;
	bool is_repair = false;
	if (m_selective_retransmission){
		uint64_t repair_start = 0;
		const uint64_t repair_size = qp->TakeRepairSegment(m_mtu, repair_start);
		if (repair_size > 0){
			seq = repair_start;
			payload_size = static_cast<uint32_t>(repair_size);
			is_repair = true;
		}
	}
	if (!is_repair){
		const uint64_t tail =
			qp->m_size >= qp->snd_nxt ? qp->m_size - qp->snd_nxt : 0;
		payload_size = tail > m_mtu ? m_mtu : static_cast<uint32_t>(tail);
	}
	Ptr<Packet> p = Create<Packet> (payload_size);
	// Drawn before the headers are written, so that the path sequence of
	// the spine it names goes into the SeqTsHeader.
	uint16_t identification = qp->m_ipid;
	if (IsPathPerPacket()){
		const uint64_t now = Simulator::Now().GetNanoSeconds();
		// A probe the selector has due leaves the NIC right after this packet.
		uint16_t probed;
		if (qp->m_pathSelector->TakeProbe(now, probed))
			SendPathProbe(qp, probed);
		identification = qp->m_pathSelector->Choose(now);
		if (is_repair && !m_tailLosses.empty())
			identification = RerouteTailRepair(qp, seq, identification, now);
	}
	uint8_t copySpine = 0;
	bool copyTrickle = false;
	const Ptr<Packet> copy =
		IsPathPerPacket() && qp->m_pathSelector->TakeCopy(copySpine, copyTrickle)
		? p->Copy() : nullptr;
	// add SeqTsHeader
	SeqTsHeader seqTs;
	seqTs.SetSeq (seq);
	seqTs.SetPG (qp->m_pg);
	// A copy is a packet up its own spine, so it carries that spine's path
	// sequence, or the receiver would count it against the data's spine.
	// Both path sequences are taken before either header is written, so that
	// the tail counts the two packets carry include both.
	uint16_t pathSeq = 0;
	uint16_t copyPathSeq = 0;
	if (m_sprayPathSeq){
		pathSeq = qp->m_pathSeqSender.Next(RequestedSpine(identification));
		if (copy)
			copyPathSeq = qp->m_pathSeqSender.Next(copySpine);
		seqTs.SetPathSeq(pathSeq);
	}
	if (m_sprayTailRecovery)
		SetTailFields(qp, seqTs,
			(is_repair ? qp->snd_nxt : seq + payload_size) >= qp->m_size);
	if (copy && m_ackRoute == static_cast<uint32_t>(AckRoute::Repath))
		seqTs.AddTailFlags(kTailCopied);
	if (m_sprayAnswerPathSeq)
		seqTs.SetAnswerHold(AnswerRouteHoldTo(
			m_switchQueues->LeafOf(qp->dip.Get())).Bitmap(
				Simulator::Now().GetNanoSeconds()));
	p->AddHeader (seqTs);
	if (copy){
		if (m_sprayPathSeq)
			seqTs.SetPathSeq(copyPathSeq);
		copy->AddHeader(seqTs);
	}
	// add udp header
	UdpHeader udpHeader;
	udpHeader.SetDestinationPort (qp->dport);
	udpHeader.SetSourcePort (qp->sport);
	// Trimming truncates the payload but leaves the UDP length field unmodified
	// (UEC 1.0.3 section 4.1), which is how the destination learns how many
	// payload bytes were discarded. ns-3 otherwise derives that field from the
	// live buffer extent, which does not survive truncation, so write it
	// explicitly.
	udpHeader.ForcePayloadSize (
		CustomHeader::GetUdpHeaderSize(seqTs.GetTailFlags()) + payload_size);
	p->AddHeader (udpHeader);
	if (copy)
		copy->AddHeader(udpHeader);
	// add ipv4 header
	Ipv4Header ipHeader;
	ipHeader.SetSource (qp->sip);
	ipHeader.SetDestination (qp->dip);
	ipHeader.SetProtocol (0x11);
	ipHeader.SetPayloadSize (p->GetSize());
	ipHeader.SetTtl (64);
	// UEC 1.0.3 section 3.6.4.7.1 Table 3-76: PDS Requests carry DSCP_TRIMMABLE
	// so that switches know the packet may be trimmed. The ECN bits stay clear;
	// a congested switch sets them independently of the codepoint.
	ipHeader.SetTos (0);
	ipHeader.SetDscp (static_cast<Ipv4Header::DscpType>(kUetDscpTrimmable));
	if (IsPathPerPacket()){
		ipHeader.SetIdentification(identification);
		const uint16_t path =
			PathOf(static_cast<LoadBalancingMode>(m_loadBalancing), identification);
		const uint64_t sentNs = Simulator::Now().GetNanoSeconds();
		const bool held = IsRepsHold() && qp->m_pathSelector->ChoseHeld();
		qp->m_outstanding.Add(seq, payload_size, path, sentNs,
			qp->m_pathSelector->ChoseTrickle(),
			qp->m_pathSelector->ChoseExploration(), pathSeq, held);
		const uint64_t heldLossNs =
			static_cast<uint64_t>(m_sprayHeldLossRtts * qp->m_baseRtt);
		if (held && heldLossNs > 0)
			Simulator::Schedule(NanoSeconds(heldLossNs), &RdmaHw::HeldSendDue,
				this, qp, seq, path, sentNs);
		if (IsOverdueRepair()){
			if (is_repair)
				qp->NoteOverdueRepairSent(seq, path);
			TakeOverdueSend(qp, seq, path, sentNs, held, is_repair);
		}
		if (copy){
			// The copy leaves ahead of the data packet, through the NIC's
			// high-priority queue, outside the queue pair's pacing; switches
			// queue it by its priority group as any data packet.
			Ipv4Header copyHeader = ipHeader;
			copyHeader.SetIdentification(SpineIdentification(copySpine, copySpine));
			copy->AddHeader(copyHeader);
			AddHeader(copy, 0x800);
			qp->AddCopy(seq, payload_size, copySpine,
				Simulator::Now().GetNanoSeconds(), copyTrickle);
			m_nic[GetNicIdxOfQp(qp)].dev->RdmaEnqueueHighPrioQ(copy);
		}
		if (is_repair && qp->m_outstanding.RepairsAgain(seq))
			qp->m_duplicate_repairs++;
		// The acknowledgement of this repair names seq along path, as that
		// of a send of seq along path declared lost would.
		if (is_repair && !m_tailLosses.empty())
			NoteTailRepair(qp, seq, path);
	}else{
		ipHeader.SetIdentification(qp->m_ipid);
	}
	p->AddHeader(ipHeader);
	// add ppp header
	PppHeader ppp;
	ppp.SetProtocol (0x0021); // EtherToPpp(0x800), see point-to-point-net-device.cc
	p->AddHeader (ppp);

	// Account payload attempts independently from the original QP size so
	// recovery work is visible at both success and failure terminal states.
	if (is_repair || seq < qp->m_highest_sent){
		qp->m_retransmitted_bytes += payload_size;
		if (qp->m_first_repair_ns == 0)
			qp->m_first_repair_ns = Simulator::Now().GetNanoSeconds();
	}else
		qp->m_highest_sent = seq + payload_size;
	qp->m_data_attempted_bytes += payload_size;

	// update state; a repair never advances the new-data cursor
	if (!is_repair)
		qp->snd_nxt += payload_size;
	qp->m_ipid++;

	// return
	return p;
}

bool RdmaHw::IsPathPerPacket() const{
	return m_loadBalancing != static_cast<uint32_t>(LoadBalancingMode::Ecmp);
}

// A probe of path carries no payload and no sequence. It names the queue
// pair's priority group, which a switch queues it by, so that it waits where
// the queue pair's data waits and comes back with the mark the path gives that
// data, and it is not trimmable: a probe that finds a queue full is a probe
// unanswered.
void RdmaHw::SendPathProbe(Ptr<RdmaQueuePair> qp, uint16_t path){
	qbbHeader probe;
	probe.SetPG(qp->m_pg);
	probe.SetSport(qp->sport);
	probe.SetDport(qp->dport);
	Ptr<Packet> packet = Create<Packet>(
		std::max(60 - 14 - 20 - static_cast<int>(probe.GetSerializedSize()), 0));
	packet->AddHeader(probe);
	Ipv4Header ip;
	ip.SetSource(qp->sip);
	ip.SetDestination(qp->dip);
	ip.SetProtocol(kPathProbeProtocol);
	ip.SetTtl(64);
	ip.SetPayloadSize(packet->GetSize());
	ip.SetIdentification(path);
	packet->AddHeader(ip);
	AddHeader(packet, 0x800);
	m_nic[GetNicIdxOfQp(qp)].dev->RdmaEnqueueHighPrioQ(packet);
	ReportTransportEvent("path_probe", 0);
}

uint32_t RdmaHw::DataFrameBytes(uint32_t payload){
	return PppHeader::GetStaticSize() + Ipv4Header().GetSerializedSize() +
		CustomHeader::GetUdpHeaderSize() + payload;
}

bool RdmaHw::StartPathSelection(Ptr<RdmaQueuePair> qp, uint64_t bdpBytes,
		uint64_t baseRttNs){
	qp->m_outstanding.SetPacketSize(m_mtu);
	if (m_sprayCopyExpiry == static_cast<uint32_t>(CopyExpiry::Deadline))
		qp->m_copyDeadlineRtts = m_sprayTailLossRtts;
	if (m_sprayCoverageFirst){
		qp->m_coverageFirst = true;
		qp->m_coveredAnswerLoss =
			MakeCallback(&RdmaHw::ReportCoveredAnswerLoss, this);
	}
	if (m_sprayDuplicateReport != static_cast<uint32_t>(DuplicateReport::Off)){
		if (m_sprayDuplicateReport ==
				static_cast<uint32_t>(DuplicateReport::Defer))
			qp->m_deferTimeoutEvidence = true;
		else {
			qp->m_retractTimeoutEvidence = true;
			qp->m_timeoutLossRetracted =
				MakeCallback(&RdmaHw::ReportTimeoutLossRetracted, this);
		}
		qp->m_timedOutResolved =
			MakeCallback(&RdmaHw::ReportTimedOutResolved, this);
		if (IsOverdueRepair())
			qp->m_timedOutCovered =
				MakeCallback(&RdmaHw::ReportTimedOutCovered, this);
	}
	qp->m_overdueProbe =
		m_sprayOverdueRepair == static_cast<uint32_t>(OverdueRepair::Probe) ||
		m_sprayOverdueRepair == static_cast<uint32_t>(OverdueRepair::Reprobe);
	qp->m_overdueRepeat =
		m_sprayOverdueRepair == static_cast<uint32_t>(OverdueRepair::Reprobe);
	if (!IsConnectionPathState()){
		qp->m_pathSelector = MakePathSelector(qp, bdpBytes, baseRttNs);
		return true;
	}
	// The connection's priority group stands where GetQpKey takes the port.
	std::shared_ptr<PathSelector> &shared =
		m_connectionSelectors[GetQpKey(qp->dip.Get(), 0, qp->m_pg)];
	const bool made = !shared;
	if (made)
		shared = MakePathSelector(qp, bdpBytes, baseRttNs);
	qp->m_pathSelector = std::make_unique<SharedPathSelector>(shared);
	return made;
}

std::unique_ptr<PathSelector> RdmaHw::MakePathSelector(Ptr<RdmaQueuePair> qp,
		uint64_t bdpBytes, uint64_t baseRttNs){
	if (IsAckRedirect())
		return std::make_unique<AckRedirectSelector>(m_pathRandom, m_spineCount);
	if (m_pathSelectorKind ==
			static_cast<uint32_t>(PathSelectorKind::RepsFlowHold))
		return std::make_unique<RepsHoldSelector>(m_pathRandom,
			m_repsBufferSize, m_repsFreezingTimeoutNs, bdpBytes / m_mtu,
			m_spineCount,
			std::make_unique<SpineScores>(m_spineCount, LossHoldParameters()));
	if (IsRepsSpine())
		return std::make_unique<RepsSpineSelector>(m_pathRandom,
			m_repsBufferSize, m_repsFreezingTimeoutNs, bdpBytes / m_mtu,
			m_spineCount);
	if (IsRepsOracle())
		return std::make_unique<RepsOracleSelector>(m_pathRandom,
			m_repsBufferSize, m_repsFreezingTimeoutNs, bdpBytes / m_mtu,
			m_spineCount,
			FaultedSpinesTo(m_switchQueues->LeafOf(qp->dip.Get())));
	if (IsMrcHold()){
		const uint64_t skipNs =
			static_cast<uint64_t>(m_mrcSkipBaseRtts * baseRttNs);
		const uint64_t probeNs = static_cast<uint64_t>(
			m_mrcProbeTimeouts * m_retransmission_timeout_ns);
		if (IsAutoHold())
			return std::make_unique<MrcHoldSelector>(m_pathRandom,
				m_spineCount, skipNs, probeNs, AutoHold(),
				m_switchQueues->LeafOf(qp->dip.Get()));
		return std::make_unique<MrcHoldSelector>(m_pathRandom,
			m_spineCount, skipNs, probeNs, DestinationHold(qp->dip.Get()));
	}
	if (IsAutoHold())
		return std::make_unique<RepsHoldSelector>(
			m_pathRandom, m_repsBufferSize, m_repsFreezingTimeoutNs,
			bdpBytes / m_mtu, m_spineCount, AutoHold(),
			m_switchQueues->LeafOf(qp->dip.Get()));
	if (IsRepsHold())
		return std::make_unique<RepsHoldSelector>(m_pathRandom,
			m_repsBufferSize, m_repsFreezingTimeoutNs, bdpBytes / m_mtu,
			m_spineCount, DestinationHold(qp->dip.Get()));
	if (m_loadBalancing == static_cast<uint32_t>(LoadBalancingMode::SprayUniform))
		return std::make_unique<UniformSpineSelector>(m_pathRandom, m_spineCount);
	if (IsSprayPolicy()){
		SpineScores::Parameters parameters{m_sprayGamma, m_sprayEpsilon,
			m_sprayCandidates,
			static_cast<SpineScores::CandidateDraw>(m_sprayCandidateDraw),
			SprayReportIntervalNs(), m_sprayEdgeFreeze};
		parameters.heldExploration = m_sprayHeldExploration;
		parameters.sentinelIntervalNs = m_sprayHeldSentinelIntervalNs;
		parameters.sentinelSpread = m_sprayHeldSentinelSpread;
		parameters.sentinelSalt = SentinelSalt();
		parameters.poolHoldPropagates = m_sprayPoolHoldPropagates;
		if (CountsSprayLoss()){
			parameters.lossThresholds = SprayLossThresholds();
			parameters.lossIntervalNs = m_sprayLossIntervalNs;
			parameters.lossHoldCleanSentinels = m_sprayLossHoldCleanSentinels;
		}
		if (m_spraySenderPoolWeight > 0){
			SpineScores::Parameters poolParameters = parameters;
			poolParameters.poolDecayIntervalNs = m_sprayPoolDecayIntervalNs;
			HostScores(poolParameters);
		}
		if (IsPerConnectionGrading()){
			auto scores = std::make_unique<SpineScores>(m_spineCount, parameters);
			if (m_senderSpineScores)
				scores->Pool(*m_senderSpineScores, m_spraySenderPoolWeight);
			return std::make_unique<PolicySpineSelector>(
				m_pathRandom, std::move(scores));
		}
		SpineScores &scores = m_spineScores.try_emplace(qp->dip.Get(),
			m_spineCount, parameters).first->second;
		if (m_senderSpineScores)
			scores.Pool(*m_senderSpineScores, m_spraySenderPoolWeight);
		return std::make_unique<PolicySpineSelector>(m_pathRandom, scores);
	}
	switch (static_cast<PathSelectorKind>(m_pathSelectorKind)){
	case PathSelectorKind::Ops:
		return std::make_unique<ObliviousSelector>(m_pathRandom);
	case PathSelectorKind::Reps:
		// NUM_PKTS_CWND is the window in packets, as the REPS artifact sizes
		// its exploration from the bandwidth-delay product.
		return std::make_unique<RepsSelector>(m_pathRandom,
			m_repsBufferSize, m_repsFreezingTimeoutNs, bdpBytes / m_mtu);
	case PathSelectorKind::UeOblivious:
		return std::make_unique<UeObliviousSelector>(m_pathRandom,
			m_ueEvSetSize);
	case PathSelectorKind::UeAware:
		return std::make_unique<UeAwareSelector>(m_pathRandom,
			m_ueEvSetSize, m_ueSaturationFraction);
	case PathSelectorKind::Mrc:
		return std::make_unique<MrcSelector>(m_pathRandom,
			m_mrcEvSetSize, static_cast<uint64_t>(m_mrcSkipBaseRtts * baseRttNs),
			static_cast<uint64_t>(
				m_mrcProbeTimeouts * m_retransmission_timeout_ns));
	default:
		return nullptr;
	}
}

void RdmaHw::PktSent(Ptr<RdmaQueuePair> qp, Ptr<Packet> pkt, Time interframeGap){
	qp->lastPktSize = pkt->GetSize();
	UpdateNextAvail(qp, interframeGap, pkt->GetSize());
	ArmRetransmissionTimeout(qp);
}

void RdmaHw::UpdateNextAvail(Ptr<RdmaQueuePair> qp, Time interframeGap, uint32_t pkt_size){
	Time sendingTime;
	if (m_rateBound)
		sendingTime = interframeGap + qp->m_rate.CalculateBytesTxTime(pkt_size);
	else
		sendingTime = interframeGap + qp->m_max_rate.CalculateBytesTxTime(pkt_size);
	qp->m_nextAvail = Simulator::Now() + sendingTime;
}

void RdmaHw::ChangeRate(Ptr<RdmaQueuePair> qp, DataRate new_rate){
	#if 1
	Time sendingTime = qp->m_rate.CalculateBytesTxTime(qp->lastPktSize);
	Time new_sendintTime = new_rate.CalculateBytesTxTime(qp->lastPktSize);
	qp->m_nextAvail = qp->m_nextAvail + new_sendintTime - sendingTime;
	// update nic's next avail event
	uint32_t nic_idx = GetNicIdxOfQp(qp);
	m_nic[nic_idx].dev->UpdateNextAvail(qp->m_nextAvail);
	#endif

	// change to new rate
	qp->m_rate = new_rate;
}

#define PRINT_LOG 0
/******************************
 * Mellanox's version of DCQCN
 *****************************/
void RdmaHw::UpdateAlphaMlx(Ptr<RdmaQueuePair> q){
	#if PRINT_LOG
	//printf("%lu alpha update: %08x %08x %u %u %.6lf->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_alpha);
	#endif
	if (q->mlx.m_alpha_cnp_arrived){
		q->mlx.m_alpha = (1 - m_g)*q->mlx.m_alpha + m_g; 	//binary feedback
	}else {
		q->mlx.m_alpha = (1 - m_g)*q->mlx.m_alpha; 	//binary feedback
	}
	#if PRINT_LOG
	//printf("%.6lf\n", q->mlx.m_alpha);
	#endif
	q->mlx.m_alpha_cnp_arrived = false; // clear the CNP_arrived bit
	ScheduleUpdateAlphaMlx(q);
}
void RdmaHw::ScheduleUpdateAlphaMlx(Ptr<RdmaQueuePair> q){
	q->mlx.m_eventUpdateAlpha = Simulator::Schedule(MicroSeconds(m_alpha_resume_interval), &RdmaHw::UpdateAlphaMlx, this, q);
}

void RdmaHw::cnp_received_mlx(Ptr<RdmaQueuePair> q){
	q->m_cnp_received++;
	ReportTransportEvent("cnp_taken", 0);
	q->mlx.m_alpha_cnp_arrived = true; // set CNP_arrived bit for alpha update
	q->mlx.m_decrease_cnp_arrived = true; // set CNP_arrived bit for rate decrease
	if (q->mlx.m_first_cnp){
		// init alpha
		q->mlx.m_alpha = 1;
		q->mlx.m_alpha_cnp_arrived = false;
		// schedule alpha update
		ScheduleUpdateAlphaMlx(q);
		// schedule rate decrease
		ScheduleDecreaseRateMlx(q, 1); // add 1 ns to make sure rate decrease is after alpha update
		// set rate on first CNP
		q->mlx.m_targetRate = q->m_rate = m_rateOnFirstCNP * q->m_rate;
		q->mlx.m_first_cnp = false;
	}
}

void RdmaHw::CheckRateDecreaseMlx(Ptr<RdmaQueuePair> q){
	ScheduleDecreaseRateMlx(q, 0);
	if (q->mlx.m_decrease_cnp_arrived){
		#if PRINT_LOG
		printf("%lu rate dec: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
		#endif
		bool clamp = true;
		if (!m_EcnClampTgtRate){
			if (q->mlx.m_rpTimeStage == 0)
				clamp = false;
		}
		if (clamp)
			q->mlx.m_targetRate = q->m_rate;
		q->m_rate = std::max(m_minRate, q->m_rate * (1 - q->mlx.m_alpha / 2));
		// reset rate increase related things
		q->mlx.m_rpTimeStage = 0;
		q->mlx.m_decrease_cnp_arrived = false;
		Simulator::Cancel(q->mlx.m_rpTimer);
		q->mlx.m_rpTimer = Simulator::Schedule(MicroSeconds(m_rpgTimeReset), &RdmaHw::RateIncEventTimerMlx, this, q);
		#if PRINT_LOG
		printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
		#endif
	}
}
void RdmaHw::ScheduleDecreaseRateMlx(Ptr<RdmaQueuePair> q, uint32_t delta){
	q->mlx.m_eventDecreaseRate = Simulator::Schedule(MicroSeconds(m_rateDecreaseInterval) + NanoSeconds(delta), &RdmaHw::CheckRateDecreaseMlx, this, q);
}

void RdmaHw::RateIncEventTimerMlx(Ptr<RdmaQueuePair> q){
	q->mlx.m_rpTimer = Simulator::Schedule(MicroSeconds(m_rpgTimeReset), &RdmaHw::RateIncEventTimerMlx, this, q);
	RateIncEventMlx(q);
	q->mlx.m_rpTimeStage++;
}
void RdmaHw::RateIncEventMlx(Ptr<RdmaQueuePair> q){
	// check which increase phase: fast recovery, active increase, hyper increase
	if (q->mlx.m_rpTimeStage < m_rpgThreshold){ // fast recovery
		FastRecoveryMlx(q);
	}else if (q->mlx.m_rpTimeStage == m_rpgThreshold){ // active increase
		ActiveIncreaseMlx(q);
	}else { // hyper increase
		HyperIncreaseMlx(q);
	}
}

void RdmaHw::FastRecoveryMlx(Ptr<RdmaQueuePair> q){
	#if PRINT_LOG
	printf("%lu fast recovery: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
	q->m_rate = (q->m_rate / 2) + (q->mlx.m_targetRate / 2);
	#if PRINT_LOG
	printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
}
void RdmaHw::ActiveIncreaseMlx(Ptr<RdmaQueuePair> q){
	#if PRINT_LOG
	printf("%lu active inc: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
	// get NIC
	uint32_t nic_idx = GetNicIdxOfQp(q);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
	// increate rate
	q->mlx.m_targetRate += m_rai;
	if (q->mlx.m_targetRate > dev->GetDataRate())
		q->mlx.m_targetRate = dev->GetDataRate();
	q->m_rate = (q->m_rate / 2) + (q->mlx.m_targetRate / 2);
	#if PRINT_LOG
	printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
}
void RdmaHw::HyperIncreaseMlx(Ptr<RdmaQueuePair> q){
	#if PRINT_LOG
	printf("%lu hyper inc: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
	// get NIC
	uint32_t nic_idx = GetNicIdxOfQp(q);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
	// increate rate
	q->mlx.m_targetRate += m_rhai;
	if (q->mlx.m_targetRate > dev->GetDataRate())
		q->mlx.m_targetRate = dev->GetDataRate();
	q->m_rate = (q->m_rate / 2) + (q->mlx.m_targetRate / 2);
	#if PRINT_LOG
	printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
}

/***********************
 * High Precision CC
 ***********************/
void RdmaHw::HandleAckHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	uint32_t ack_seq = ch.ack.seq;
	// update rate
	if (ack_seq > qp->hp.m_lastUpdateSeq){ // if full RTT feedback is ready, do full update
		UpdateRateHp(qp, p, ch, false);
	}else{ // do fast react
		FastReactHp(qp, p, ch);
	}
}

void RdmaHw::UpdateRateHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react){
	uint32_t next_seq = qp->snd_nxt;
	bool print = !fast_react || true;
	if (qp->hp.m_lastUpdateSeq == 0){ // first RTT
		qp->hp.m_lastUpdateSeq = next_seq;
		// store INT
		IntHeader &ih = ch.ack.ih;
		NS_ASSERT(ih.nhop <= IntHeader::maxHop);
		for (uint32_t i = 0; i < ih.nhop; i++)
			qp->hp.hop[i] = ih.hop[i];
		#if PRINT_LOG
		if (print){
			printf("%lu %s %08x %08x %u %u [%u,%u,%u]", Simulator::Now().GetTimeStep(), fast_react? "fast" : "update", qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->hp.m_lastUpdateSeq, ch.ack.seq, next_seq);
			for (uint32_t i = 0; i < ih.nhop; i++)
				printf(" %u %lu %lu", ih.hop[i].GetQlen(), ih.hop[i].GetBytes(), ih.hop[i].GetTime());
			printf("\n");
		}
		#endif
	}else {
		// check packet INT
		IntHeader &ih = ch.ack.ih;
		if (ih.nhop <= IntHeader::maxHop){
			double max_c = 0;
			bool inStable = false;
			#if PRINT_LOG
			if (print)
				printf("%lu %s %08x %08x %u %u [%u,%u,%u]", Simulator::Now().GetTimeStep(), fast_react? "fast" : "update", qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->hp.m_lastUpdateSeq, ch.ack.seq, next_seq);
			#endif
			// check each hop
			double U = 0;
			uint64_t dt = 0;
			bool updated[IntHeader::maxHop] = {false}, updated_any = false;
			NS_ASSERT(ih.nhop <= IntHeader::maxHop);
			for (uint32_t i = 0; i < ih.nhop; i++){
				if (m_sampleFeedback){
					if (ih.hop[i].GetQlen() == 0 && fast_react)
						continue;
				}
				updated[i] = updated_any = true;
				#if PRINT_LOG
				if (print)
					printf(" %u(%u) %lu(%lu) %lu(%lu)", ih.hop[i].GetQlen(), qp->hp.hop[i].GetQlen(), ih.hop[i].GetBytes(), qp->hp.hop[i].GetBytes(), ih.hop[i].GetTime(), qp->hp.hop[i].GetTime());
				#endif
				uint64_t tau = ih.hop[i].GetTimeDelta(qp->hp.hop[i]);;
				double duration = tau * 1e-9;
				double txRate = (ih.hop[i].GetBytesDelta(qp->hp.hop[i])) * 8 / duration;
				double u = txRate / ih.hop[i].GetLineRate() + (double)std::min(ih.hop[i].GetQlen(), qp->hp.hop[i].GetQlen()) * qp->m_max_rate.GetBitRate() / ih.hop[i].GetLineRate() /qp->m_win;
				#if PRINT_LOG
				if (print)
					printf(" %.3lf %.3lf", txRate, u);
				#endif
				if (!m_multipleRate){
					// for aggregate (single R)
					if (u > U){
						U = u;
						dt = tau;
					}
				}else {
					// for per hop (per hop R)
					if (tau > qp->m_baseRtt)
						tau = qp->m_baseRtt;
					qp->hp.hopState[i].u = (qp->hp.hopState[i].u * (qp->m_baseRtt - tau) + u * tau) / double(qp->m_baseRtt);
				}
				qp->hp.hop[i] = ih.hop[i];
			}

			DataRate new_rate;
			int32_t new_incStage;
			DataRate new_rate_per_hop[IntHeader::maxHop];
			int32_t new_incStage_per_hop[IntHeader::maxHop];
			if (!m_multipleRate){
				// for aggregate (single R)
				if (updated_any){
					if (dt > qp->m_baseRtt)
						dt = qp->m_baseRtt;
					qp->hp.u = (qp->hp.u * (qp->m_baseRtt - dt) + U * dt) / double(qp->m_baseRtt);
					max_c = qp->hp.u / m_targetUtil;

					if (max_c >= 1 || qp->hp.m_incStage >= m_miThresh){
						new_rate = qp->hp.m_curRate / max_c + m_rai;
						new_incStage = 0;
					}else{
						new_rate = qp->hp.m_curRate + m_rai;
						new_incStage = qp->hp.m_incStage+1;
					}
					if (new_rate < m_minRate)
						new_rate = m_minRate;
					if (new_rate > qp->m_max_rate)
						new_rate = qp->m_max_rate;
					#if PRINT_LOG
					if (print)
						printf(" u=%.6lf U=%.3lf dt=%u max_c=%.3lf", qp->hp.u, U, dt, max_c);
					#endif
					#if PRINT_LOG
					if (print)
						printf(" rate:%.3lf->%.3lf\n", qp->hp.m_curRate.GetBitRate()*1e-9, new_rate.GetBitRate()*1e-9);
					#endif
				}
			}else{
				// for per hop (per hop R)
				new_rate = qp->m_max_rate;
				for (uint32_t i = 0; i < ih.nhop; i++){
					if (updated[i]){
						double c = qp->hp.hopState[i].u / m_targetUtil;
						if (c >= 1 || qp->hp.hopState[i].incStage >= m_miThresh){
							new_rate_per_hop[i] = qp->hp.hopState[i].Rc / c + m_rai;
							new_incStage_per_hop[i] = 0;
						}else{
							new_rate_per_hop[i] = qp->hp.hopState[i].Rc + m_rai;
							new_incStage_per_hop[i] = qp->hp.hopState[i].incStage+1;
						}
						// bound rate
						if (new_rate_per_hop[i] < m_minRate)
							new_rate_per_hop[i] = m_minRate;
						if (new_rate_per_hop[i] > qp->m_max_rate)
							new_rate_per_hop[i] = qp->m_max_rate;
						// find min new_rate
						if (new_rate_per_hop[i] < new_rate)
							new_rate = new_rate_per_hop[i];
						#if PRINT_LOG
						if (print)
							printf(" [%u]u=%.6lf c=%.3lf", i, qp->hp.hopState[i].u, c);
						#endif
						#if PRINT_LOG
						if (print)
							printf(" %.3lf->%.3lf", qp->hp.hopState[i].Rc.GetBitRate()*1e-9, new_rate.GetBitRate()*1e-9);
						#endif
					}else{
						if (qp->hp.hopState[i].Rc < new_rate)
							new_rate = qp->hp.hopState[i].Rc;
					}
				}
				#if PRINT_LOG
				printf("\n");
				#endif
			}
			if (updated_any)
				ChangeRate(qp, new_rate);
			if (!fast_react){
				if (updated_any){
					qp->hp.m_curRate = new_rate;
					qp->hp.m_incStage = new_incStage;
				}
				if (m_multipleRate){
					// for per hop (per hop R)
					for (uint32_t i = 0; i < ih.nhop; i++){
						if (updated[i]){
							qp->hp.hopState[i].Rc = new_rate_per_hop[i];
							qp->hp.hopState[i].incStage = new_incStage_per_hop[i];
						}
					}
				}
			}
		}
		if (!fast_react){
			if (next_seq > qp->hp.m_lastUpdateSeq)
				qp->hp.m_lastUpdateSeq = next_seq; //+ rand() % 2 * m_mtu;
		}
	}
}

void RdmaHw::FastReactHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	if (m_fast_react)
		UpdateRateHp(qp, p, ch, true);
}

/**********************
 * TIMELY
 *********************/
void RdmaHw::HandleAckTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	uint32_t ack_seq = ch.ack.seq;
	// update rate
	if (ack_seq > qp->tmly.m_lastUpdateSeq){ // if full RTT feedback is ready, do full update
		UpdateRateTimely(qp, p, ch, false);
	}else{ // do fast react
		FastReactTimely(qp, p, ch);
	}
}
void RdmaHw::UpdateRateTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool us){
	uint32_t next_seq = qp->snd_nxt;
	uint64_t rtt = Simulator::Now().GetTimeStep() - ch.ack.ih.ts;
	bool print = !us;
	if (qp->tmly.m_lastUpdateSeq != 0){ // not first RTT
		int64_t new_rtt_diff = (int64_t)rtt - (int64_t)qp->tmly.lastRtt;
		double rtt_diff = (1 - m_tmly_alpha) * qp->tmly.rttDiff + m_tmly_alpha * new_rtt_diff;
		double gradient = rtt_diff / m_tmly_minRtt;
		bool inc = false;
		double c = 0;
		#if PRINT_LOG
		if (print)
			printf("%lu node:%u rtt:%lu rttDiff:%.0lf gradient:%.3lf rate:%.3lf", Simulator::Now().GetTimeStep(), m_node->GetId(), rtt, rtt_diff, gradient, qp->tmly.m_curRate.GetBitRate() * 1e-9);
		#endif
		if (rtt < m_tmly_TLow){
			inc = true;
		}else if (rtt > m_tmly_THigh){
			c = 1 - m_tmly_beta * (1 - (double)m_tmly_THigh / rtt);
			inc = false;
		}else if (gradient <= 0){
			inc = true;
		}else{
			c = 1 - m_tmly_beta * gradient;
			if (c < 0)
				c = 0;
			inc = false;
		}
		if (inc){
			if (qp->tmly.m_incStage < 5){
				qp->m_rate = qp->tmly.m_curRate + m_rai;
			}else{
				qp->m_rate = qp->tmly.m_curRate + m_rhai;
			}
			if (qp->m_rate > qp->m_max_rate)
				qp->m_rate = qp->m_max_rate;
			if (!us){
				qp->tmly.m_curRate = qp->m_rate;
				qp->tmly.m_incStage++;
				qp->tmly.rttDiff = rtt_diff;
			}
		}else{
			qp->m_rate = std::max(m_minRate, qp->tmly.m_curRate * c);
			if (!us){
				qp->tmly.m_curRate = qp->m_rate;
				qp->tmly.m_incStage = 0;
				qp->tmly.rttDiff = rtt_diff;
			}
		}
		#if PRINT_LOG
		if (print){
			printf(" %c %.3lf\n", inc? '^':'v', qp->m_rate.GetBitRate() * 1e-9);
		}
		#endif
	}
	if (!us && next_seq > qp->tmly.m_lastUpdateSeq){
		qp->tmly.m_lastUpdateSeq = next_seq;
		// update
		qp->tmly.lastRtt = rtt;
	}
}
void RdmaHw::FastReactTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
}

/**********************
 * DCTCP
 *********************/
void RdmaHw::HandleAckDctcp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	uint32_t ack_seq = ch.ack.seq;
	uint8_t cnp = (ch.ack.flags >> qbbHeader::FLAG_CNP) & 1;
	bool new_batch = false;

	// update alpha
	qp->dctcp.m_ecnCnt += (cnp > 0);
	if (ack_seq > qp->dctcp.m_lastUpdateSeq){ // if full RTT feedback is ready, do alpha update
		#if PRINT_LOG
		printf("%lu %s %08x %08x %u %u [%u,%u,%u] %.3lf->", Simulator::Now().GetTimeStep(), "alpha", qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->dctcp.m_lastUpdateSeq, ch.ack.seq, qp->snd_nxt, qp->dctcp.m_alpha);
		#endif
		new_batch = true;
		if (qp->dctcp.m_lastUpdateSeq == 0){ // first RTT
			qp->dctcp.m_lastUpdateSeq = qp->snd_nxt;
			qp->dctcp.m_batchSizeOfAlpha = qp->snd_nxt / m_mtu + 1;
		}else {
			double frac = std::min(1.0, double(qp->dctcp.m_ecnCnt) / qp->dctcp.m_batchSizeOfAlpha);
			qp->dctcp.m_alpha = (1 - m_g) * qp->dctcp.m_alpha + m_g * frac;
			qp->dctcp.m_lastUpdateSeq = qp->snd_nxt;
			qp->dctcp.m_ecnCnt = 0;
			qp->dctcp.m_batchSizeOfAlpha = (qp->snd_nxt - ack_seq) / m_mtu + 1;
			#if PRINT_LOG
			printf("%.3lf F:%.3lf", qp->dctcp.m_alpha, frac);
			#endif
		}
		#if PRINT_LOG
		printf("\n");
		#endif
	}

	// check cwr exit
	if (qp->dctcp.m_caState == 1){
		if (ack_seq > qp->dctcp.m_highSeq)
			qp->dctcp.m_caState = 0;
	}

	// check if need to reduce rate: ECN and not in CWR
	if (cnp && qp->dctcp.m_caState == 0){
		#if PRINT_LOG
		printf("%lu %s %08x %08x %u %u %.3lf->", Simulator::Now().GetTimeStep(), "rate", qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->m_rate.GetBitRate()*1e-9);
		#endif
		qp->m_rate = std::max(m_minRate, qp->m_rate * (1 - qp->dctcp.m_alpha / 2));
		#if PRINT_LOG
		printf("%.3lf\n", qp->m_rate.GetBitRate() * 1e-9);
		#endif
		qp->dctcp.m_caState = 1;
		qp->dctcp.m_highSeq = qp->snd_nxt;
	}

	// additive inc
	if (qp->dctcp.m_caState == 0 && new_batch)
		qp->m_rate = std::min(qp->m_max_rate, qp->m_rate + m_dctcp_rai);
}

/*********************
 * HPCC-PINT
 ********************/
void RdmaHw::SetPintSmplThresh(double p){
       pint_smpl_thresh = (uint32_t)(65536 * p);
}
void RdmaHw::HandleAckHpPint(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
       uint32_t ack_seq = ch.ack.seq;
       if (rand() % 65536 >= pint_smpl_thresh)
               return;
       // update rate
       if (ack_seq > qp->hpccPint.m_lastUpdateSeq){ // if full RTT feedback is ready, do full update
               UpdateRateHpPint(qp, p, ch, false);
       }else{ // do fast react
               UpdateRateHpPint(qp, p, ch, true);
       }
}

void RdmaHw::UpdateRateHpPint(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react){
       uint32_t next_seq = qp->snd_nxt;
       if (qp->hpccPint.m_lastUpdateSeq == 0){ // first RTT
               qp->hpccPint.m_lastUpdateSeq = next_seq;
       }else {
               // check packet INT
               IntHeader &ih = ch.ack.ih;
               double U = Pint::decode_u(ih.GetPower());

               DataRate new_rate;
               int32_t new_incStage;
               double max_c = U / m_targetUtil;

               if (max_c >= 1 || qp->hpccPint.m_incStage >= m_miThresh){
                       new_rate = qp->hpccPint.m_curRate / max_c + m_rai;
                       new_incStage = 0;
               }else{
                       new_rate = qp->hpccPint.m_curRate + m_rai;
                       new_incStage = qp->hpccPint.m_incStage+1;
               }
               if (new_rate < m_minRate)
                       new_rate = m_minRate;
               if (new_rate > qp->m_max_rate)
                       new_rate = qp->m_max_rate;
               ChangeRate(qp, new_rate);
               if (!fast_react){
                       qp->hpccPint.m_curRate = new_rate;
                       qp->hpccPint.m_incStage = new_incStage;
               }
               if (!fast_react){
                       if (next_seq > qp->hpccPint.m_lastUpdateSeq)
                               qp->hpccPint.m_lastUpdateSeq = next_seq; //+ rand() % 2 * m_mtu;
               }
       }
}

/*********************
 * NSCC
 ********************/
NsccWindow::Parameters RdmaHw::NsccParameters() const{
	return NsccWindow::Parameters{m_nscc_target_qdelay, m_nscc_max_window,
		m_nscc_gamma, m_nscc_max_md_jump, m_nscc_fair_increase,
		m_nscc_fast_increase_scale, m_nscc_eta, m_nscc_alpha, m_nscc_qa_gate,
		m_nscc_qa_threshold, m_nscc_adjust_bytes, m_nscc_adjust_period};
}

void RdmaHw::HandleAckNscc(Ptr<RdmaQueuePair> qp, uint64_t seq, uint16_t path,
		bool marked, uint32_t rcvCwndPend){
	const uint64_t packet = qp->m_outstanding.Find(seq, path);
	// A repeated or late answer: its bytes already left the window.
	if (packet == OutstandingPackets::kNone)
		return;
	// Read before the acknowledgement removes the record.
	const uint32_t bytes = qp->m_outstanding.Size(packet);
	const uint64_t rtt = AnswerRtt(qp, packet);
	const uint64_t lost = qp->AcknowledgePacket(seq, path);
	qp->Nscc().OnAck(bytes, marked, rtt, qp->WindowOnTheFly(),
		Simulator::Now().GetNanoSeconds(), rcvCwndPend);
	if (lost > 0)
		qp->Nscc().OnLoss(lost);
	ApplyNsccWindow(qp);
}

void RdmaHw::ApplyNsccWindow(Ptr<RdmaQueuePair> qp){
	const uint32_t window = static_cast<uint32_t>(qp->Nscc().Cwnd());
	const auto apply = [this, window](Ptr<RdmaQueuePair> each){
		const uint32_t previous = each->m_win;
		each->SetWin(window);
		if (window != previous)
			m_traceNsccWindow(each, previous, window);
	};
	if (!qp->m_nsccConnection){
		apply(qp);
		return;
	}
	for (const Ptr<RdmaQueuePair> &each : qp->m_nsccConnection->queuePairs)
		apply(each);
}

bool RdmaHw::ValidNsccRttSample() const{
	return m_nsccRttSample == static_cast<uint32_t>(NsccRttSample::Any) ||
		m_cc_mode == 11;
}

// The round trip an answer to the outstanding send `packet` measures, from the
// send's record. An answer to the resend of a send declared lost may be the
// late answer to that send, so such a resend is not timed (UEC 1.0.3 section
// 3.6.13.1); a trimmed send is answered by its trim, so its resend is.
uint64_t RdmaHw::AnswerRtt(Ptr<RdmaQueuePair> qp, uint64_t packet) const{
	const OutstandingPackets &outstanding = qp->m_outstanding;
	if (packet == OutstandingPackets::kNone || outstanding.ResendsLost(packet))
		return NsccWindow::kNoRtt;
	const uint64_t rtt =
		Simulator::Now().GetNanoSeconds() - outstanding.SentNs(packet);
	if (m_nsccRttSample != static_cast<uint32_t>(NsccRttSample::Karn))
		return rtt;
	// The answer may be any send's of the sequence, so its round trip is
	// unknown.
	if (outstanding.Sends(packet) > 1)
		return NsccWindow::kNoRtt;
	return std::max(rtt, qp->m_baseRtt);
}

bool RdmaHw::ValidNsccWindowState() const{
	return m_nsccWindowState ==
			static_cast<uint32_t>(NsccWindowState::Message) ||
		m_cc_mode == 11;
}

void RdmaHw::StartNscc(Ptr<RdmaQueuePair> qp, uint64_t bdpBytes,
		uint64_t baseRttNs){
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	if (m_nsccWindowState ==
			static_cast<uint32_t>(NsccWindowState::Connection)){
		// The connection's priority group stands where GetQpKey takes the
		// port.
		std::shared_ptr<NsccConnection> &connection =
			m_nsccConnections[GetQpKey(qp->dip.Get(), 0, qp->m_pg)];
		if (!connection){
			connection = std::make_shared<NsccConnection>();
			connection->window.Start(NsccParameters(), m_mtu, bdpBytes,
				baseRttNs, now);
		}
		connection->queuePairs.push_back(qp);
		qp->m_nsccConnection = connection;
	}else
		qp->Nscc().Start(NsccParameters(), m_mtu, bdpBytes, baseRttNs, now);
	NS_ABORT_MSG_IF(qp->Nscc().MaxWnd() > UINT32_MAX,
		"a queue pair's window is 32 bits");
	ApplyNsccWindow(qp);
}

void RdmaHw::LeaveNsccConnection(Ptr<RdmaQueuePair> qp){
	if (!qp->m_nsccConnection)
		return;
	std::vector<Ptr<RdmaQueuePair>> &open = qp->m_nsccConnection->queuePairs;
	open.erase(std::remove(open.begin(), open.end(), qp), open.end());
}

}
