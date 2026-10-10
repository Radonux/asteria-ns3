#include <ns3/simulator.h>
#include <ns3/seq-ts-header.h>
#include <ns3/udp-header.h>
#include <ns3/ipv4-header.h>
#include "ns3/ppp-header.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/data-rate.h"
#include "ns3/pointer.h"
#include "ns3/abort.h"
#include "rdma-hw.h"
#include "ppp-header.h"
#include "qbb-header.h"
#include "cn-header.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace ns3{

namespace {

// The round trip an answer to the outstanding send `packet` measures, from the
// send's record. An answer to the resend of a send declared lost may be the
// late answer to that send, so such a resend is not timed (UEC 1.0.3 section
// 3.6.13.1); a trimmed send is answered by its trim, so its resend is.
uint64_t AnswerRtt(const OutstandingPackets &outstanding, uint64_t packet){
	if (packet == OutstandingPackets::kNone || outstanding.ResendsLost(packet))
		return NsccWindow::kNoRtt;
	return Simulator::Now().GetNanoSeconds() - outstanding.SentNs(packet);
}

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
				"congestion bitmap; 4=MRC.",
				UintegerValue(static_cast<uint32_t>(PathSelectorKind::Ops)),
				MakeUintegerAccessor(&RdmaHw::m_pathSelectorKind),
				MakeUintegerChecker<uint32_t>(
					static_cast<uint32_t>(PathSelectorKind::Ops),
					static_cast<uint32_t>(PathSelectorKind::Mrc)))
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
		.AddAttribute("SprayBaseRttNs",
				"Under LoadBalancing 3, the base RTT the attributes below count "
				"time in. Zero, the default, is refused at the first use.",
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
				"time, then the grader, which holds the report and the interval "
				"it closed.",
				MakeTraceSourceAccessor(&RdmaHw::m_traceSpineReport),
				"ns3::RdmaHw::SpineReportTracedCallback")
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
	NS_ABORT_MSG_IF(m_loadBalancing !=
				static_cast<uint32_t>(LoadBalancingMode::Ecmp) && !m_pathRandom,
		"A LoadBalancing mode other than ECMP draws the identification of every "
		"data packet from PathRandomVariable");
	NS_ABORT_MSG_IF(NamesSpine(static_cast<LoadBalancingMode>(m_loadBalancing)) &&
			m_spineCount == 0,
		"LoadBalancing 2 and 3 name a spine per packet and need SpineCount");
	NS_ABORT_MSG_IF(IsSprayPolicy() && m_spineCount > SpineReport::kMaxSpines,
		"a spine report grades at most SpineReport::kMaxSpines spines");
	NS_ABORT_MSG_IF(CustomHeader::ackReportBytes != (IsSprayPolicy()
			? 1 + SpineReport::GradeBytes(m_spineCount) : 0),
		"LoadBalancing 3 and only it carries a report on the spines in every "
		"acknowledgement, its sequence and the grades of SpineCount spines");
	NS_ABORT_MSG_IF(m_sprayOneWayDelay &&
			(!IsSprayPolicy() || IntHeader::mode != IntHeader::TS),
		"SprayOneWayDelay grades the spines of LoadBalancing 3 by the send "
		"timestamp of IntHeader::mode TS");
	NS_ABORT_MSG_IF(m_pathSelectorKind !=
				static_cast<uint32_t>(PathSelectorKind::Ops) &&
			m_loadBalancing !=
				static_cast<uint32_t>(LoadBalancingMode::EntropyHash),
		"PathSelector chooses entropy values and needs LoadBalancing 1");
	NS_ABORT_MSG_IF(m_pathSelectorKind ==
				static_cast<uint32_t>(PathSelectorKind::Mrc) &&
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
void RdmaHw::AddQueuePair(uint32_t src, uint32_t dest, uint64_t tag, uint64_t size, uint16_t pg, Ipv4Address sip, Ipv4Address dip, uint16_t sport, uint16_t dport, uint32_t win, uint64_t baseRtt, Callback<void> notifyAppFinish, Callback<void> notifyAppSent){
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
	if (IsPathPerPacket())
		StartPathSelection(qp, win, baseRtt);
	// The liveness invariant starts at birth: an unfinished QP always has a
	// pending timer, even if its first send never gets scheduled.
	ArmRetransmissionTimeout(qp);

	// set init variables
	DataRate m_bps = m_nic[nic_idx].dev->GetDataRate();
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
		qp->nscc.Start(NsccParameters(), m_mtu, win, baseRtt,
			Simulator::Now().GetNanoSeconds());
		NS_ABORT_MSG_IF(qp->nscc.MaxWnd() > UINT32_MAX,
			"a queue pair's window is 32 bits");
		ApplyNsccWindow(qp);
	}

	// Notify Nic
	m_nic[nic_idx].dev->NewQp(qp);
}

void RdmaHw::DeleteQueuePair(Ptr<RdmaQueuePair> qp){
	// remove qp from the m_qpMap
	uint64_t key = GetQpKey(qp->dip.Get(), qp->sport, qp->m_pg);
	m_qpMap.erase(key);
}

Ptr<RdmaRxQueuePair> RdmaHw::GetRxQp(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport, uint16_t pg, bool create){
	uint64_t key = ((uint64_t)dip << 32) | ((uint64_t)pg << 16) | (uint64_t)dport;
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
	uint64_t key = ((uint64_t)dip << 32) | ((uint64_t)pg << 16) | (uint64_t)dport;
	auto it = m_rxQpMap.find(key);
	if (it == m_rxQpMap.end())
		return;
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
	if (IsSprayPolicy())
		GradeArrival(ch, ecnbits != 0);
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
	rxQp->NoteSeen(range_end);
	int x = ReceiverCheckSeq(ch.udp.seq, rxQp, payload_size);

	if (x == 1 || x == 2){ //generate ACK or NACK
		SendAck(rxQp, ch.dip, ch.sip, ch.udp.dport, ch.udp.sport, ch.udp.pg,
			ch.udp.ih, ch.udp.seq, ch.ipid, x == 2, ecnbits != 0,
			AllowanceGone(rxQp), false);
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

uint64_t RdmaHw::SprayReportIntervalNs() const{
	NS_ABORT_MSG_IF(m_sprayBaseRttNs == 0,
		"LoadBalancing 3 counts its report interval in SprayBaseRttNs");
	return static_cast<uint64_t>(m_sprayReportIntervalBaseRtts * m_sprayBaseRttNs);
}

// Made at the first event, because the base RTT is set only after every host
// is built.
SpineGrader &RdmaHw::Grader(){
	if (m_spineGrader)
		return *m_spineGrader;
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
	parameters.oneWayDelay = m_sprayOneWayDelay;
	parameters.delayNs = {m_sprayEstimatorGain,
		m_sprayDelayCusumSlackBaseRtts * rtt,
		m_sprayDelayCusumThresholdBaseRtts * rtt, m_sprayEstimatorIntervalSamples};
	parameters.delayThresholdsNs[0] = m_sprayDelayThreshold1BaseRtts * rtt;
	parameters.delayThresholdsNs[1] = m_sprayDelayThreshold2BaseRtts * rtt;
	parameters.delayThresholdsNs[2] = m_sprayDelayThreshold3BaseRtts * rtt;
	parameters.reference =
		static_cast<SpineGrader::GradeReference>(m_sprayGradeReference);
	parameters.absenceFractionOfMedian = m_sprayAbsenceFractionOfMedian;
	parameters.absenceMinimumMedian = m_sprayAbsenceMinimumMedian;
	parameters.releaseFractionOfFloor = m_sprayReleaseFractionOfFloor;
	// A held spine is drawn at epsilon / N and a top-graded one at about 1 / N.
	parameters.floorShareOfMedian = m_sprayEpsilon;
	m_spineGrader = std::make_unique<SpineGrader>(parameters);
	return *m_spineGrader;
}

SpineGrader &RdmaHw::GraderNow(){
	const uint64_t now = Simulator::Now().GetNanoSeconds();
	SpineGrader &grader = Grader();
	if (grader.Advance(now))
		m_traceSpineReport(now, grader);
	return grader;
}

void RdmaHw::GradeArrival(const CustomHeader &ch, bool marked){
	// One clock serves the whole simulation, so the difference is the delay.
	const uint64_t delay = m_sprayOneWayDelay
		? (Simulator::Now() - TimeStep(ch.udp.ih.ts)).GetNanoSeconds() : 0;
	GraderNow().OnArrival(RequestedSpine(ch.ipid), CarryingSpine(ch.ipid), marked,
		delay);
}

void RdmaHw::GradeTrim(const CustomHeader &ch, bool lastHop){
	GraderNow().OnTrim(CarryingSpine(ch.ipid), lastHop);
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
		bool nack, bool cnp, bool spent, bool probeAnswer){
	qbbHeader seqh;
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
		seqh.SetSpineReport(Grader().Report());
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
	uint16_t qIndex = ch.ack.pg;
	uint16_t port = ch.ack.dport;
	uint32_t seq = ch.ack.seq;
	uint8_t cnp = (ch.ack.flags >> qbbHeader::FLAG_CNP) & 1;
	int i;
	Ptr<RdmaQueuePair> qp = GetQp(ch.sip, port, qIndex);
	if (!qp){
		return 0;
	}

	uint32_t nic_idx = GetNicIdxOfQp(qp);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
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
		// without asking what was sent ahead of it on its path.
		if (qp->m_outstanding.IsKept()){
			const uint16_t path =
				PathOf(static_cast<LoadBalancingMode>(m_loadBalancing), ch.ipid);
			const uint64_t now = Simulator::Now().GetNanoSeconds();
			if (IsSprayPolicy())
				qp->m_pathSelector->OnReport(ReportOf(ch), now);
			if ((ch.ack.flags >> qbbHeader::FLAG_PROBE_ANSWER) & 1){
				qp->m_pathSelector->OnProbeAnswer(path, cnp, now);
			}else{
				qp->m_pathSelector->OnAck(path, cnp, now);
				// The destination's downlink being congested is the receiver's
				// flow control of UEC 1.0.3 section 3.6.13.2.
				const uint32_t rcvCwndPend = IsSprayPolicy() &&
						((ch.ack.flags >> qbbHeader::FLAG_EDGE_CONGESTED) & 1)
					? m_sprayEdgeWindowPenalty : 0;
				if (m_cc_mode == 11 && DeliverCongestionSignal(qp))
					HandleAckNscc(qp, ch.ack.packet_seq, path, cnp, rcvCwndPend);
				else
					qp->AcknowledgePacket(ch.ack.packet_seq, path);
			}
		}
		const uint64_t acknowledged_before = qp->snd_una;
		if (!m_backto0){
			qp->Acknowledge(seq);
		}else {
			uint32_t goback_seq = seq / m_chunk * m_chunk;
			qp->Acknowledge(goback_seq);
		}
		if (qp->snd_una > acknowledged_before){
			qp->m_recovery_retries = 0;
			qp->m_last_progress_ns = Simulator::Now().GetNanoSeconds();
		}
		if (qp->IsFinished()){
			Simulator::Cancel(qp->m_retransmissionTimer);
			QpComplete(qp);
			return 0;
		}
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
	Ptr<RdmaRxQueuePair> q = GetRxQp(ch.dip, ch.sip, ch.ack.dport, ch.ack.sport,
		ch.ack.pg, false);
	if (q == nullptr)
		return 0;
	SendAck(q, ch.dip, ch.sip, ch.ack.dport, ch.ack.sport, ch.ack.pg,
		IntHeader(), 0, ch.ipid, false, ch.GetIpv4EcnBits() != 0,
		AllowanceGone(q), true);
	return 0;
}

// UEC 1.0.3 Table 3-61: a trimmed RUD/ROD/RUDI Request MUST produce a NACK with
// pds.nack_code UET_TRIMMED or UET_TRIMMED_LASTHOP. The NACK is a control packet
// (DSCP_CONTROL / TC_high) and carries only the identity of the lost packet.
void RdmaHw::SendTrimNack(const CustomHeader &ch, uint32_t sourceIp,
		uint32_t destinationIp, uint16_t sport, uint16_t dport, uint16_t pg,
		uint32_t seq, uint32_t payloadSize, bool lastHop, bool spent,
		bool eligible){
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
		repair.SetSpineReport(Grader().Report());
	Ptr<Packet> packet = Create<Packet>(
		std::max(60 - 14 - 20 - static_cast<int>(repair.GetSerializedSize()), 0));
	packet->AddHeader(repair);

	Ipv4Header ipHeader;
	ipHeader.SetSource(Ipv4Address(sourceIp));
	ipHeader.SetDestination(Ipv4Address(destinationIp));
	ipHeader.SetProtocol(kUecTrimRepairProtocol);
	ipHeader.SetDscp(static_cast<Ipv4Header::DscpType>(kUetDscpControl));
	ipHeader.SetPayloadSize(packet->GetSize());
	ipHeader.SetTtl(64);
	ipHeader.SetIdentification(ch.ipid);
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
		? AnswerRtt(qp->m_outstanding, qp->m_outstanding.Find(trimStart, path))
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
		qp->nscc.OnTrim(ch.ack.trim_payload_size, trimmedRtt, qp->GetOnTheFly(),
			Simulator::Now().GetNanoSeconds());
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
		SendTrimNack(ch, ch.dip, ch.sip, ch.udp.dport, ch.udp.sport, ch.udp.pg,
			ch.udp.seq, payloadSize, lastHop, false, false);
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
			ch.udp.ih, ch.udp.seq, ch.ipid, false, false, AllowanceGone(q), false);
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
			ch.udp.ih, ch.udp.seq, ch.ipid, false, true, AllowanceGone(q), false);
		return;
	}
	// The refused range stays a hole, so the report is computed after the
	// frontier moved and counts it.
	SendTrimNack(ch, ch.dip, ch.sip, ch.udp.dport, ch.udp.sport, ch.udp.pg,
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
		IntHeader(), 0, 0, false, false, false, false);
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
		const uint32_t udpHeaderBytes = CustomHeader::GetUdpHeaderSize();
		// The UDP length field survives trimming unmodified, so it still reports
		// the size of the original packet that was dropped.
		const uint32_t originalPayload = ch.udp.payload_size > udpHeaderBytes
			? ch.udp.payload_size - udpHeaderBytes : 0;
		const bool lastHop = ch.GetIpv4Dscp() == kUetDscpTrimmedLastHop;
		if (IsSprayPolicy())
			GradeTrim(ch, lastHop);
		if (m_forgiveness && originalPayload > 0){
			ReceiveTrimmedData(ch, originalPayload, lastHop);
			return 0;
		}
		SendTrimNack(ch, ch.dip, ch.sip, ch.udp.dport, ch.udp.sport, ch.udp.pg,
			ch.udp.seq, originalPayload, lastHop, false, false);
		return 0;
	}

	// The repair request the destination returned.
	Ptr<RdmaQueuePair> qp = GetQp(ch.sip, ch.ack.dport, ch.ack.pg);
	if (!qp || qp->IsFailed()) {
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
	if (!qp->m_retransmissionTimer.IsExpired())
		Simulator::Cancel(qp->m_retransmissionTimer);
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
	if (qp->m_outstanding.IsKept()){
		const uint64_t now = Simulator::Now().GetNanoSeconds();
		const uint64_t lost =
			qp->DeclareLostSentBy(now - m_retransmission_timeout_ns);
		NS_ASSERT_MSG(lost > 0, "the timer expires with the oldest send");
		qp->m_pathSelector->OnTimeout(now);
		if (m_cc_mode == 11 && DeliverCongestionSignal(qp)){
			qp->nscc.OnLoss(lost);
			ApplyNsccWindow(qp);
		}
	}else{
		RecoverQueue(qp);
	}
	const uint32_t nic_idx = GetNicIdxOfQp(qp);
	m_nic[nic_idx].dev->TriggerTransmit();
	ArmRetransmissionTimeout(qp);
}

void RdmaHw::ReportTransportEvent(const char* event, uint64_t bytes){
	if (!m_transportEventCallback.IsNull())
		m_transportEventCallback(event, bytes);
}

void RdmaHw::QpComplete(Ptr<RdmaQueuePair> qp){
	NS_ASSERT(!m_qpCompleteCallback.IsNull());
	Simulator::Cancel(qp->m_retransmissionTimer);
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
	// add SeqTsHeader
	SeqTsHeader seqTs;
	seqTs.SetSeq (seq);
	seqTs.SetPG (qp->m_pg);
	p->AddHeader (seqTs);
	// add udp header
	UdpHeader udpHeader;
	udpHeader.SetDestinationPort (qp->dport);
	udpHeader.SetSourcePort (qp->sport);
	// Trimming truncates the payload but leaves the UDP length field unmodified
	// (UEC 1.0.3 section 4.1), which is how the destination learns how many
	// payload bytes were discarded. ns-3 otherwise derives that field from the
	// live buffer extent, which does not survive truncation, so write it
	// explicitly.
	udpHeader.ForcePayloadSize (CustomHeader::GetUdpHeaderSize() + payload_size);
	p->AddHeader (udpHeader);
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
		const uint64_t now = Simulator::Now().GetNanoSeconds();
		// A probe the selector has due leaves the NIC right after this packet.
		uint16_t probed;
		if (qp->m_pathSelector->TakeProbe(now, probed))
			SendPathProbe(qp, probed);
		const uint16_t identification = qp->m_pathSelector->Choose(now);
		ipHeader.SetIdentification(identification);
		qp->m_outstanding.Add(seq, payload_size,
			PathOf(static_cast<LoadBalancingMode>(m_loadBalancing), identification),
			Simulator::Now().GetNanoSeconds());
		if (is_repair && qp->m_outstanding.RepairsAgain(seq))
			qp->m_duplicate_repairs++;
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

void RdmaHw::StartPathSelection(Ptr<RdmaQueuePair> qp, uint64_t bdpBytes,
		uint64_t baseRttNs){
	qp->m_outstanding.SetPacketSize(m_mtu);
	if (m_loadBalancing == static_cast<uint32_t>(LoadBalancingMode::SprayUniform)){
		qp->m_pathSelector =
			std::make_unique<UniformSpineSelector>(m_pathRandom, m_spineCount);
		return;
	}
	if (IsSprayPolicy()){
		const SpineScores::Parameters parameters{m_sprayGamma, m_sprayEpsilon,
			m_sprayCandidates,
			static_cast<SpineScores::CandidateDraw>(m_sprayCandidateDraw),
			SprayReportIntervalNs()};
		SpineScores &scores = m_spineScores.try_emplace(qp->dip.Get(),
			m_spineCount, parameters).first->second;
		qp->m_pathSelector =
			std::make_unique<PolicySpineSelector>(m_pathRandom, scores);
		return;
	}
	switch (static_cast<PathSelectorKind>(m_pathSelectorKind)){
	case PathSelectorKind::Ops:
		qp->m_pathSelector = std::make_unique<ObliviousSelector>(m_pathRandom);
		break;
	case PathSelectorKind::Reps:
		// NUM_PKTS_CWND is the window in packets, as the REPS artifact sizes
		// its exploration from the bandwidth-delay product.
		qp->m_pathSelector = std::make_unique<RepsSelector>(m_pathRandom,
			m_repsBufferSize, m_repsFreezingTimeoutNs, bdpBytes / m_mtu);
		break;
	case PathSelectorKind::UeOblivious:
		qp->m_pathSelector =
			std::make_unique<UeObliviousSelector>(m_pathRandom, m_ueEvSetSize);
		break;
	case PathSelectorKind::UeAware:
		qp->m_pathSelector = std::make_unique<UeAwareSelector>(m_pathRandom,
			m_ueEvSetSize, m_ueSaturationFraction);
		break;
	case PathSelectorKind::Mrc:
		qp->m_pathSelector = std::make_unique<MrcSelector>(m_pathRandom,
			m_mrcEvSetSize, static_cast<uint64_t>(m_mrcSkipBaseRtts * baseRttNs),
			static_cast<uint64_t>(
				m_mrcProbeTimeouts * m_retransmission_timeout_ns));
		break;
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
	const uint64_t rtt = AnswerRtt(qp->m_outstanding, packet);
	const uint64_t lost = qp->AcknowledgePacket(seq, path);
	qp->nscc.OnAck(bytes, marked, rtt, qp->GetOnTheFly(),
		Simulator::Now().GetNanoSeconds(), rcvCwndPend);
	if (lost > 0)
		qp->nscc.OnLoss(lost);
	ApplyNsccWindow(qp);
}

void RdmaHw::ApplyNsccWindow(Ptr<RdmaQueuePair> qp){
	const uint32_t previous = qp->m_win;
	const uint32_t window = static_cast<uint32_t>(qp->nscc.Cwnd());
	qp->SetWin(window);
	if (window != previous)
		m_traceNsccWindow(qp, previous, window);
}

}
