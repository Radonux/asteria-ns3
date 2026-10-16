/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#undef PGO_TRAINING
#define PATH_TO_PGO_CONFIG "path_to_pgo_config"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/global-route-manager.h"
#include "ns3/integer.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/flow-id-tag.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/qbb-helper.h"
#include "ns3/qbb-header.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <cmath>
#include <limits>
#include <map>
#include <zstd.h>
#include <ns3/link-events.h>
#include <ns3/load-balancing.h>
#include <ns3/path-selector.h>
#include <ns3/rdma-client-helper.h>
#include <ns3/rdma-client.h>
#include <ns3/rdma-driver.h>
#include <ns3/rdma.h>
#include <ns3/sim-setting.h>
#include <ns3/spine-grader.h>
#include <ns3/switch-node.h>
#include <time.h>
#include <unordered_map>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("GENERIC_SIMULATION");

uint32_t cc_mode = 1;
// Under CC_MODE 11 (NSCC), NSCC_WINDOW_STATE message or connection: how long a
// host keeps an NSCC window. message, the default, opens one per queue pair,
// that is per message. connection keeps one per destination and priority
// group, opened once at MaxWnd by the first queue pair to it, shared by every
// queue pair to it, at once or later, as the bound of the bytes they have in
// flight together, and kept past them: the next message to the destination
// starts from the window the last one left, and an idle connection keeps its
// window as it stands. Each queue pair keeps its own send records, repairs and
// timers. The other CC modes keep their state per queue pair and refuse it.
std::string nscc_window_state = "message";
bool nscc_window_state_given = false;
// Under CC_MODE 11 (NSCC), NSCC_RTT_SAMPLE any or karn: which answers give
// NSCC a round trip. any, the default, takes every answer but that of a
// sequence's first resend after it was declared lost, matched to the newest
// send of its sequence along its path. karn takes Karn's rule: no answer to a
// sequence sent more than once while unacknowledged, whatever became of the
// earlier send (declared lost, trimmed and released, or copied), and floors
// every sample at the queue pair's base RTT, below which no answer returns.
std::string nscc_rtt_sample = "any";
bool nscc_rtt_sample_given = false;
// PAIR_BANDWIDTH path or spines: the bandwidth of a pair of hosts (pairBw),
// which with its base RTT gives the pair's BDP, the link speed and MaxWnd of
// NSCC's window and REPS's buffer. path, the default, is the bottleneck of the
// best shortest path (CalculateRoute). spines, under a LOAD_BALANCING mode
// that names spines, is that of a pair whose packets are sprayed over every
// spine, for hosts on two leaves: the sum over the spines of the lesser of
// the source leaf's uplink to it and its downlink to the destination leaf, at
// most either host's link (SprayPairBandwidth).
std::string pair_bandwidth = "path";
bool enable_qcn = true, use_dynamic_pfc_threshold = true;
uint32_t packet_payload_size = 1000, l2_chunk_size = 0, l2_ack_interval = 0;
double pause_time = 5, simulator_stop_time = 3.01;
std::string topology_file, flow_file, trace_file, trace_output_file;
std::string fct_output_file = "fct.txt";
std::string pfc_output_file = "pfc.txt";
// The raw per-packet log always exists - keeping the research artifact
// invariant - and keeps its historic ns-3 CSV syntax; only the container is
// zstd-compressed.
std::string transport_event_output_file = "transport_events.csv.zst";
// Aggregated per-(event, plane) totals; what the analyzer consumes.
std::string transport_event_summary_output_file = "transport_summary.csv";
// Each switch port's transmissions and marks, and each host's arrivals by
// carrying spine, written when a link fails and at the end of the run. Empty
// writes neither.
std::string port_counter_output_file, spine_arrival_output_file;
std::string spine_report_output_file;
// Each host's pooled loss intervals under SPRAY_LOSS_THRESHOLDS: one row per
// host, closed interval and spine. Empty writes none.
std::string spine_loss_output_file;
// One row per retransmission timeout fired, under every mode: the time, the
// host's rank and the destination's, the requested spine of the oldest
// outstanding send the timer waited for (empty where no send record is kept
// or the mode names no spine), whether that send was an exploration send
// onto a held spine (a sentinel or a trickle send; a copy under
// SPRAY_SENTINEL_MODE duplicate has no send record and no timeout waits for
// it), and the bytes outstanding. Empty writes none.
std::string rto_event_output_file;
// Under SPRAY_RECEIVER_HOLD on, each receiving host's hold: one row per host,
// closed SPRAY_LOSS_INTERVAL_NS interval, sender leaf and spine, with the
// arrivals and the missing packets counted in it and the scope the spine is
// held at as it closed, none, leaf or host. A file of its own, as the rows of
// SPINE_LOSS_OUTPUT_FILE have other columns under each SPRAY_HOLD_SCOPE.
// Empty writes none.
std::string receiver_hold_output_file;
// Under SPRAY_ANSWER_PATH_SEQ on, ANSWER_HOLD_OUTPUT_FILE: one row per change
// of a hold, time_ns,side,rank,leaf,spine,held: side sender for a route spine
// of the answers of receiver leaf leaf held at sender rank, receiver for a
// spine of rank's answers to sender leaf leaf, held 1 or 0.
std::string answer_hold_output_file;
FILE *port_counter_file = nullptr, *spine_arrival_file = nullptr;
FILE *spine_report_file = nullptr, *spine_loss_file = nullptr;
FILE *rto_event_file = nullptr, *receiver_hold_file = nullptr;
FILE *answer_hold_file = nullptr;

double alpha_resume_interval = 55, rp_timer, ewma_gain = 1 / 16;
double rate_decrease_interval = 4;
uint32_t fast_recovery_times = 5;
std::string rate_ai, rate_hai, min_rate = "100Mb/s";
std::string dctcp_rate_ai = "1000Mb/s";

bool clamp_target_rate = false, l2_back_to_zero = false;
double error_rate_per_link = 0.0;
double data_loss_probability = 0.0;
uint64_t data_loss_start_ns = 0, data_loss_duration_ns = 0;
std::string data_loss_scope = "all";
int32_t data_loss_source_host = -1, data_loss_destination_host = -1;
int32_t data_loss_receiver_node = -1;
uint64_t data_loss_rng_stream = 51;
// Fixed ns-3 stream indices. Data loss takes DATA_LOSS_RNG_STREAM upward, two
// per link; the hosts' per-packet path draw takes stream 0, so a change of
// LOAD_BALANCING moves no other draw of a run with the same seed. Draws made
// per device take one stream each from their own block above every data-loss
// stream, so no two of them share a sequence.
const int64_t path_rng_stream = 0;
const int64_t device_rng_block = int64_t(1) << 40;
enum class DeviceRngBlock : int64_t {
  LinkError = 1,
  EcnMarking = 2,
  PfcIdentification = 3,
  // A host's draw of its answers' spines under ACK_ROUTE spray, on the
  // stream of its first NIC.
  AckRoute = 4,
};

int64_t device_rng_stream(DeviceRngBlock block, Ptr<NetDevice> dev) {
  return static_cast<int64_t>(block) * device_rng_block +
         static_cast<int64_t>(dev->GetNode()->GetId()) * SwitchMmu::pCnt +
         dev->GetIfIndex();
}
uint64_t retransmission_timeout_ns = 0;
uint32_t max_retransmission_retries = 0;
uint64_t no_progress_timeout_ns = 0;
uint32_t selective_retransmission = 0;
std::string packet_trim_mode = "disabled";
std::string load_balancing = "ecmp";
// Under LOAD_BALANCING ev_hash, what chooses each data packet's entropy value,
// and under spray_uniform whether ack_redirect or reps_hold chooses its spine
// instead of a uniform draw; and the parameters of the selectors, each the
// default of its source. reps_hold is REPS over spines, leaving out the
// spines the host's loss hold holds; reps_flow_hold keeps a loss hold per
// queue pair instead. reps_oracle is REPS over spines leaving out for good
// the spines the switches' state shows a fault on toward the destination's
// leaf, read under SPRAY_GRADE_COST_SOURCE switch_queue. mrc_hold is MRC over
// spines, leaving out the spines reps_hold's loss hold holds, at the same
// SPRAY_HOLD_SCOPE; reps_hold() holds for it too. reps_spine is REPS over
// spines leaving out none: no loss hold, no sentinels, and none of the loss
// hold keys; it reads REPS_BUFFER_SIZE and REPS_FREEZING_TIMEOUT_NS as reps
// does, and SPRAY_PATH_SEQ and SPRAY_TAIL_RECOVERY as every spray_uniform
// selector does.
std::string path_selector = "ops";
bool reps_hold() {
  return path_selector == "reps_hold" || path_selector == "reps_flow_hold" ||
         path_selector == "mrc_hold";
}
// Whether SPRAY_HOLD_SCOPE is read: reps_hold and mrc_hold.
bool hold_scope_read() {
  return path_selector == "reps_hold" || path_selector == "mrc_hold";
}
// Under reps_hold and mrc_hold, what one loss hold is kept for: host, every
// destination of the host, leaf, the destinations on one leaf, auto, per spine
// either of the two, or pair, one destination host. Under auto the host counts
// each spine's sends and losses per destination leaf and keeps the destination
// leaves with a loss on it since it was last released. A spine graded 0 toward
// a leaf is held for every destination when at least SPRAY_HOLD_AUTO_LEAVES
// leaves have a loss on it, else for that leaf alone; a leaf-scope hold widens
// to every destination once a loss on the spine brings its leaves to
// SPRAY_HOLD_AUTO_LEAVES. A hold ends by the clean sends at the scope it has,
// counted over every leaf for a host-scope hold, whose release clears the
// spine's leaves. Each held (spine, scope) is sent one sentinel per sentinel
// interval for the host.
// SPINE_LOSS_OUTPUT_FILE writes one row per destination leaf and spine, as
// leaf writes it, with the grade of the scope the spine is held at. Under pair
// the hold is leaf's, kept per destination host instead of per leaf: the
// host's queue pairs to one destination, each made for one message, share it,
// so that it outlasts them as one queue pair kept across the messages would
// keep it. SPINE_LOSS_OUTPUT_FILE writes one row per destination and spine,
// with the destination's node id where leaf writes the leaf.
std::string spray_hold_scope = "auto";
bool spray_hold_scope_given = false;
// Under SPRAY_HOLD_SCOPE auto, the destination leaves with a loss on a spine
// from which its hold is kept for every destination; at least 2.
uint32_t spray_hold_auto_leaves = 2;
bool spray_hold_auto_leaves_given = false;
// Under reps_hold, reps_flow_hold and mrc_hold, what puts a spine in loss hold,
// per hold key: the (destination leaf, spine) under SPRAY_HOLD_SCOPE leaf and
// auto, the (destination, spine) under pair, the spine under host, and the
// queue pair's spine under reps_flow_hold. count: a loss interval whose lost
// fraction grades the spine 0 through SPRAY_LOSS_THRESHOLDS. llr: a sequential
// likelihood-ratio test per key, of the loss probability SPRAY_LOSS_P0 of a
// working path against SPRAY_LOSS_P1 of a faulted one: its statistic L starts
// at 0, each send on the key adds ln((1 - p1) / (1 - p0)) and each send
// declared lost ln(p1 / p0), and the key is held at the loss that brings L to
// ln((1 - beta) / alpha), SPRAY_HOLD_ALPHA being the probability of holding a
// working path and SPRAY_HOLD_BETA that of missing a faulted one. L is clamped
// below at 0, so that a clean stretch builds no unbounded credit against the
// losses that follow it, and restarts at 0 at the key's release. With
// SPRAY_LOSS_P0 0 a loss adds +infinity, so the first loss holds as count holds
// it, and a send adds 0. Under llr SPRAY_LOSS_THRESHOLDS grade the intervals
// SPINE_LOSS_OUTPUT_FILE writes but hold nothing, and the release is unchanged.
// Under auto the leaves with a loss on a spine still choose the scope of the
// hold the test starts. SPRAY_HOLD_BETA is read only under llr, and
// SPRAY_LOSS_P0, SPRAY_LOSS_P1 and SPRAY_HOLD_ALPHA only under llr or
// SPRAY_HOLD_RELEASE trickle; 0 <= SPRAY_LOSS_P0 < SPRAY_LOSS_P1 < 1, and
// SPRAY_HOLD_ALPHA and SPRAY_HOLD_BETA are in (0, 0.5).
std::string spray_hold_rule = "count";
bool spray_hold_rule_given = false;
double spray_loss_p0 = 0;
double spray_loss_p1 = 0.001;
double spray_hold_alpha = 0.001;
double spray_hold_beta = 0.001;
std::set<std::string> hold_test_keys_given;
// Under reps_hold, reps_flow_hold and mrc_hold, how a held key is released.
// sentinels: SPRAY_LOSS_HOLD_CLEAN_SENTINELS clean sends of the sentinels
// sent one per SPRAY_HELD_SENTINEL_INTERVAL_NS, a clean send being a send on
// the held key acknowledged in a loss interval that declared no send on it
// lost; a send still unanswered as its interval closes counts as none.
// trickle: each queue pair sends each spine its hold holds one in
// round(1 / SPRAY_HOLD_TRICKLE) of its sends, by a counter per queue pair
// and not a draw, in place of the sentinels; these trickle sends are what SPRAY_HELD_EXPLORATION sentinel
// names under trickle, and it is still required. At a hold's start its key
// records p1, the lost fraction of the hold's evidence, the loss interval
// that graded it 0 under count and the sends since the test's L was last
// clamped at 0 under llr, at least SPRAY_LOSS_P1. A release statistic R per
// hold key starts at 0, each trickle send acknowledged adds
// ln((1 - p0) / (1 - p1)), -ln(1 - p1) with SPRAY_LOSS_P0 0, a trickle send
// declared lost restarts it at 0, and the key is released once R reaches
// ln(1 / SPRAY_HOLD_ALPHA). Under auto a host-scope hold records the larger
// of its evidence's p1 and that of the leaf-scope holds it ends. Under
// trickle SPRAY_LOSS_HOLD_CLEAN_SENTINELS and SPRAY_HELD_SENTINEL_INTERVAL_NS
// are refused, and SPRAY_HOLD_TRICKLE, in (0, 0.5], is read only under it.
// Under reps_hold, reps_flow_hold and mrc_hold, what an exploration send onto a
// held spine (a sentinel under SPRAY_HOLD_RELEASE sentinels, or of the
// receivers' hold under SPRAY_RECEIVER_HOLD, a trickle send under trickle) is.
// data: the data packet itself, in place of the spine REPS, or MRC under
// mrc_hold, would choose. duplicate: the data packet goes up the spine the
// selector chooses among the spines not held, and a copy of it, the same
// sequence, goes up the held spine in the same send, ahead of it through the
// NIC's high-priority queue. The receiver takes whichever arrives first and
// discards the other as a duplicate, acknowledging each with the spine it came
// by. The copy has a record of its own on the held spine, apart from the data
// send records: its acknowledgement and its loss move the hold's counts, its
// test and its release statistic as a data exploration send's do, and the
// per-path rule treats it as a send on its spine. Its loss queues no repair and
// no retransmission timeout waits for it: once the data send is acknowledged, a
// copy unanswered for more than one base RTT since it was sent is declared lost
// for the hold alone. Where the selector's choice is the held spine itself,
// which it is only while every spine is held, no copy is made. Under
// SPRAY_PATH_SEQ the copy carries the path sequence of the held spine, as a
// packet sent up it.
std::string spray_sentinel_mode = "data";
bool spray_sentinel_mode_given = false;
// Under SPRAY_SENTINEL_MODE duplicate, SPRAY_COPY_EXPIRY: when a copy its own
// acknowledgement has not answered is declared lost for the hold. data_ack,
// the default, as above: once its data send is acknowledged, more than one
// base RTT after the copy was sent. deadline: more than SPRAY_TAIL_LOSS_RTTS
// base RTTs after it was sent, the guard a tail repair request's declared
// sends wait, whether its data send is acknowledged or not, so that a copy
// queued on its spine longer than its data's path takes is not lost when its
// data's acknowledgement comes first. Either is checked at the queue pair's
// acknowledgements; deadline needs SPRAY_TAIL_LOSS_RTTS above 0.
std::string spray_copy_expiry = "data_ack";
bool spray_copy_expiry_given = false;
// Under reps_hold, reps_flow_hold and mrc_hold, the queue pair's base RTTs
// after which a data send up a spine held as it was chosen, by the loss hold
// or under SPRAY_RECEIVER_HOLD by the receivers' (the sentinel or trickle send
// under SPRAY_SENTINEL_MODE data, or any send while every spine is held), is
// declared lost if it is still unacknowledged and repaired, without waiting
// for the retransmission timeout; a later answer to it is taken as a
// resend's. 0 waits for the timeout. At least 0.
double spray_held_loss_rtts = 2;
bool spray_held_loss_rtts_given = false;
// Under LOAD_BALANCING spray_uniform, with any PATH_SELECTOR,
// SPRAY_ACK_LOSS_RTTS <x>: a data send that the cumulative acknowledgement
// covers before its own acknowledgement arrives, and whose own acknowledgement
// has still not arrived x of the queue pair's base RTTs after that, is declared
// lost on its path for the path selector alone, as a lost acknowledgement: the
// loss holds and SPINE_LOSS_OUTPUT_FILE count it as a loss on that spine, but
// nothing is repaired, the retransmission timeout is left as it is and the
// congestion controller is told nothing, the data having arrived. Each is
// counted as the host transport event ack_loss_declared. 0, the default,
// declares nothing. At least 0.
double spray_ack_loss_rtts = 0;
bool spray_ack_loss_rtts_given = false;
// Under LOAD_BALANCING spray_uniform, with any PATH_SELECTOR, ACK_ROUTE: the
// spine the switches route each answer a receiver sends to a sender over, an
// acknowledgement, a NACK or a repair request. data_spine, the default, is the
// spine that carried the data packet it answers, which the answer's echoed
// identification names. spray draws a spine uniformly per answer to a sender
// off the receiver's leaf from a stream of the receiver's own, and names it
// in a route_spine byte every answer then carries; answers to a sender on the
// receiver's leaf cross no spine and draw nothing. reverse_set keeps per
// connection, a sending host and priority group, a set of ACK_ROUTE_SET_SIZE
// spines drawn once from that stream, routes each answer over a member drawn
// uniformly, probes every member once per SPRAY_BASE_RTT_NS while the
// connection's data arrives, with an acknowledgement the sender returns over
// the spine probed, and replaces a member by a spine outside the set once its
// probe has gone unanswered for two base RTTs (host transport events
// answer_set_probe and answer_set_replace). repath keeps per connection one
// spine drawn from that stream, and draws another at each data packet that
// adds nothing the receiver held already, a resend, unless its send was also
// sent as a copy, which its tail flags then say (answer_repath); it needs
// SPRAY_TAIL_RECOVERY on. Under each but data_spine the answer names its spine
// in route_spine. The echoed identification is the same under all, and the
// sender reads its spines from it alone.
std::string ack_route = "data_spine";
bool ack_route_given = false;
uint32_t ack_route_set_size = 2;
bool ack_route_set_size_given = false;
// Under ACK_ROUTE spray, ACK_ROUTE_ORACLE on: the draw leaves out the spines
// over which an answer toward the sender's leaf crosses a fault, a loss or a
// black hole on the receiver's leaf's uplink to the spine or on the spine's
// downlink to the sender's leaf, or a port slower than another spine's, as
// the switches' state shows it from the fault's onset, read once per
// SPRAY_BASE_RTT_NS as reps_oracle reads its own; a spine once left out stays
// out. Off, the default.
std::string ack_route_oracle = "off";
bool ack_route_oracle_given = false;
bool ack_route_spray() { return ack_route == "spray"; }
// Whether answers name their route spine: under every ACK_ROUTE but the data's.
bool ack_route_named() { return ack_route != "data_spine"; }
bool ack_route_oracle_on() { return ack_route_oracle == "on"; }
std::string spray_hold_release = "sentinels";
bool spray_hold_release_given = false;
double spray_hold_trickle = 0.01;
bool spray_hold_trickle_given = false;
// Under LOAD_BALANCING spray_uniform, with any PATH_SELECTOR, SPRAY_PATH_SEQ
// on: every data packet carries, in a 2-byte path_seq field after the INT header, its queue pair's
// count of the data packets sent before it up its requested spine. The
// receiver keeps per queue pair and per path, the requested and the carrying
// spine, the path sequence it expects next; an arrival ahead of it by g shows
// g packets missing on the carrying spine, and an arrival behind it is late
// and counts nothing. Only data arrivals, which are ordered within a path,
// count gaps. A trimmed header, which keeps the path sequence, travels in the
// switches' high-priority queue and can overtake earlier data of its path: it
// marks its path sequence as accounted and neither moves the expected
// sequence on nor counts anything. A data arrival ahead of the expected
// sequence counts as missing only the sequences it passes that are not
// accounted, so a trim is not a missing packet. A path keeps the accounted
// sequences from the expected one up to 4096 ahead of it, and drops each as
// the expected sequence passes it. A packet a source leaf moved to another
// spine is charged to the spine that carried it. Off, the default, leaves the
// data header as it is.
std::string spray_path_seq = "off";
bool spray_path_seq_given = false;
// Under LOAD_BALANCING spray_uniform, with any PATH_SELECTOR or none,
// SPRAY_PATH_STATE message or connection: how long a host keeps the path
// selector of a queue pair, each queue pair carrying one message. message, the
// default, keeps it for that queue pair alone, so every message starts afresh.
// connection keeps one per destination and priority group, made by the first
// queue pair to it, shared by every queue pair to it, at once or later, and
// kept past them, as one queue pair kept across the messages would keep it:
// REPS's cached values and freezing mode, MRC's value states, rotation and
// probes, a loss hold selector's trickle schedule and receivers' marks,
// reps_flow_hold's own hold and ack_redirect's redirect carry from one message
// to the next. A queue pair's send records, copies, covered sends and path
// sequences stay its own, and the host's holds are as SPRAY_HOLD_SCOPE keeps
// them. A tail repair request's loss told after its queue pair completed
// reaches the connection's selector.
std::string spray_path_state = "message";
bool spray_path_state_given = false;
// Under LOAD_BALANCING spray_uniform, with any PATH_SELECTOR,
// SPRAY_TAIL_RECOVERY on, which requires SPRAY_PATH_SEQ on and at most 8
// spines: a queue pair's message is its flow's data, and it ends with the send
// of its last new data packet. That packet and every repair the queue pair
// sends after it is the newest send up its requested spine, is marked in the
// data header as closing that spine, and carries after path_seq the count of
// the data packets the queue pair has sent so far up each spine. The data
// header of every data packet, closing or not, grows by 1 + 2 x the spine count
// bytes: a 1-byte flag field and a 16-bit count per spine. The last packet up
// each other spine left before the message ended, when the selector had not yet
// drawn the spines of the packets after it, so it carries no mark; the counts
// tell the receiver it is owed, lost or not. The receiver of a closing packet,
// or of its trimmed header, compares each spine's count with the path sequence
// it expects next of that spine (the furthest of the paths requested up it) and
// the packet's own spine with its path sequence, and at once asks for each run
// of missing path sequences, the sequences its trimmed headers accounted left
// out, in a repair request carrying the spine, the first
// missing path sequence and their count, 5 bytes after receiver_hold in every
// acknowledgement header; a run is asked once. The sender declares lost the
// outstanding sends those path sequences name on that spine, as the per-path
// rule does, so the loss holds and the loss trace see each, and repairs them at
// once; a request for a send already acknowledged or declared lost is ignored.
// The retransmission timeout is unchanged and still recovers what no closing
// packet that arrives reports, such as the loss of the last of them. Off, the
// default, leaves both headers as they are.
std::string spray_tail_recovery = "off";
bool spray_tail_recovery_given = false;
// Under SPRAY_TAIL_RECOVERY on, SPRAY_TAIL_RECOVERY_DELAY_NS <n>: the receiver
// does not request a missing run at once. It records the run and, n ns
// later, by one timer per queue pair, requests what of it is still missing;
// a packet of the run that arrives meanwhile, late behind the packet that
// showed it missing, is left out. A lost tail packet is then repaired at the
// receiver about 1.5 base RTT + n after the message's end. Unset, n is the
// fabric's largest base RTT between two hosts (maxRtt, as RdmaHw's
// SprayBaseRttNs); 0 requests at once.
uint64_t spray_tail_recovery_delay_ns = 0;
bool spray_tail_recovery_delay_ns_given = false;
// Under SPRAY_TAIL_RECOVERY on, SPRAY_TAIL_LOSS_RTTS <x>: a send a tail repair
// request declares lost is repaired at once, but the path selector is told of
// its loss x of the queue pair's base RTTs later, so that the loss holds and
// SPINE_LOSS_OUTPUT_FILE count it then. A trimmed header can reach the receiver
// after its request for the trimmed packet; the trim notification of that send
// arriving before then drops the loss, a trim being no loss to the selector,
// and repairs nothing again; each loss so dropped is counted as the host
// transport event tail_loss_cancelled. An acknowledgement of the declared send
// along its path, the send having arrived after the request, drops the loss as
// well, counted as tail_loss_delivered, or as tail_loss_ambiguous when a repair
// of the send had left along the same path. The selector chooses again, up to
// once per spine, for a repair it would send along the path of the waiting
// declared send, and a repair so moved to another path is counted as
// tail_repair_rerouted. The losses wait on the host, so a queue pair that
// completes leaves its own waiting, and a trim notification that follows the
// completion still drops one; one told after the completion reaches the host's
// loss hold for the receiver's leaf, and under reps_flow_hold, whose hold was
// the queue pair's, is dropped. 0 tells the selector at once. Default 1. At
// least 0.
double spray_tail_loss_rtts = 1;
bool spray_tail_loss_rtts_given = false;
bool tail_recovery() { return spray_tail_recovery == "on"; }
// Under reps_hold, reps_flow_hold or mrc_hold with SPRAY_PATH_SEQ on,
// SPRAY_RECEIVER_HOLD on: each receiving host counts, per sender leaf and spine, the data packets
// that arrived and the packets the path sequences show missing. A missing
// packet on spine k from sender leaf a holds k for a alone, or for every
// sender leaf once SPRAY_HOLD_AUTO_LEAVES sender leaves have a missing packet
// on it; SPRAY_RECEIVER_HOLD_CLEAN consecutive arrivals on k without a missing
// packet, from a or from any leaf as the hold's scope is, release it, and a
// missing packet restarts the count. Every acknowledgement carries a 1-byte
// receiver_hold bitmap, bit k set when the receiver holds k for the leaf of
// the acknowledgement's destination; at most 8 spines. A sender holds spine k
// toward the receiver's leaf while the latest acknowledgement of any receiver
// on that leaf sets bit k, with the loss hold's exclusion and sentinels
// (SPRAY_HELD_SENTINEL_INTERVAL_NS), shared by the host's queue pairs, or under
// reps_flow_hold the queue pair's own; the sender's clean sends do not release
// it. The hosts' leaves come from the switches' state. Off, the default,
// leaves the acknowledgement header as it is.
std::string spray_receiver_hold = "off";
bool spray_receiver_hold_given = false;
uint32_t spray_receiver_hold_clean = 1024;
bool spray_receiver_hold_clean_given = false;
// With SPRAY_RECEIVER_HOLD and SPRAY_TAIL_RECOVERY on,
// SPRAY_RECEIVER_TAIL_MISSING on: the receiving host's hold also counts as
// missing, for the sender's leaf on their requested spine, the path sequences
// a tail repair request names that are still missing SPRAY_TAIL_LOSS_RTTS of
// the fabric's largest base RTT after the request. The gaps alone never show
// a black-holed spine, on which no data arrives. The wait matches the
// sender's: a send that arrives late in it is the one whose acknowledgement
// cancels the sender's report of it. A data packet or a trimmed header of the
// run arriving before then takes it out, a trim being no loss; a sequence a
// data arrival passed meanwhile was counted as a gap and is not counted
// again; and a sequence counted that arrives later is a late arrival, which
// neither counts toward the release nor restarts it. Each send counted is
// the host transport event receiver_tail_missing, and each of them whose data
// packet or trimmed header arrives after all is receiver_tail_missing_late.
// Each is also written as receiver_tail_missing_spine<k> or
// receiver_tail_missing_late_spine<k> for the requested spine k of its send,
// the breakdown of the two by spine. Off, the default, leaves the hold to the
// gaps.
std::string spray_receiver_tail_missing = "off";
bool spray_receiver_tail_missing_given = false;
// Under SPRAY_PATH_SEQ on, SPRAY_STRAY_PATH_SEQ: how a receiving host reads
// the path sequences of a packet that arrives after the frontend removed its
// flow's receive queue pair at the flow's completion, a copy, a repaired send
// or a repair still in flight, which makes a receive queue pair of its own.
// fresh, the default, reads them there as a new flow's, so that the tail
// counts of a closing packet among them name every send of the flow that
// queue pair has not seen as missing, and SPRAY_RECEIVER_TAIL_MISSING counts
// them. closed reads them against the removed queue pair: the packet takes
// its sequence out of that queue pair's tail runs still waiting, is
// receiver_tail_missing_late where that queue pair counted its sequence
// missing, and counts no gap, asks no tail repair and counts nothing in the
// receiver's hold.
std::string spray_stray_path_seq = "fresh";
bool spray_stray_path_seq_given = false;
// Under LOAD_BALANCING spray_uniform, SPRAY_COVERAGE_FIRST on: before the
// per-path rule, the held-loss rule, a tail repair request or a copy's expiry
// declares a send lost, the queue pair applies its coverage, the cumulative
// acknowledgement (the one of the answer it reads, where it reads one) and for
// a copy its data send's answer. A send so covered was delivered, and its
// missing answer is an answer loss: counted as the host transport event
// answer_loss_covered, neither repaired nor told to the selector, so that it
// counts toward no spine's losses, no hold and no copy's. The covered sends
// SPRAY_ACK_LOSS_RTTS declares are counted so too. Off, the default.
std::string spray_coverage_first = "off";
bool spray_coverage_first_given = false;
// With SPRAY_COVERAGE_FIRST on, SPRAY_DUPLICATE_REPORT on: a receiver flags its
// answer to a data packet whose sequence it held already, and a sender tells
// the selector the loss of a send a retransmission timeout declares only at
// the answer to its resend: none where that answer is flagged, the first send
// having arrived, counted as the host transport event answer_loss_duplicate;
// and on the first send's path where it is not, or the cumulative
// acknowledgement covers the sequence first, counted as timeout_loss_data.
// retract: the receiver flags as under on, and the sender tells each loss a
// retransmission timeout declares at once, as with off, a sequence timed out
// again told once more; the flagged answer to the sequence takes each back
// from the host's auto loss hold, regrading without it the hold it began,
// counted as answer_loss_duplicate, by the route spine the answer came back
// over, and timeout_retract_release where the hold was released; a loss
// left standing is timeout_loss_data. retract needs PATH_SELECTOR reps_hold or
// mrc_hold under SPRAY_HOLD_SCOPE auto and SPRAY_HOLD_RULE count. Off, the
// default.
std::string spray_duplicate_report = "off";
bool spray_duplicate_report_given = false;
// Under ACK_ROUTE spray with SPRAY_PATH_SEQ on and a loss hold PATH_SELECTOR,
// SPRAY_ANSWER_PATH_SEQ on: a receiving host numbers each queue pair's answers
// per route spine, and each answer announces another route's count. The
// sender counts per receiver leaf and route spine the answers it receives and
// those it finds lost, from a gap in a route's numbers or from an
// announcement not met within two base RTTs, and holds a route whose lost
// fraction over a SPRAY_LOSS_INTERVAL_NS interval reaches the top of
// SPRAY_LOSS_THRESHOLDS; SPRAY_ANSWER_HOLD_CLEAN answers in a row received
// over a held route without a loss release it. Its data packets to the leaf
// carry the routes it holds in an answer_hold byte, and a receiving host
// leaves out of its draw of the routes of its answers to a sender leaf the
// spines any sender of that leaf last reported held, but for one probe answer
// per held spine and sender leaf every SPRAY_ANSWER_PROBE_INTERVAL_NS. Each
// answer found lost is the host transport event answer_route_lost, with its
// breakdown by route spine, each probe answer answer_probe, and each change of
// a sender's route hold answer_route_hold or answer_route_release;
// ANSWER_HOLD_OUTPUT_FILE traces the changes of both ends' holds. Off, the
// default.
std::string spray_answer_path_seq = "off";
bool spray_answer_path_seq_given = false;
uint64_t spray_answer_probe_interval_ns = 1000000;
bool spray_answer_probe_interval_ns_given = false;
uint32_t spray_answer_hold_clean = 3;
bool spray_answer_hold_clean_given = false;
bool answer_path_seq() { return spray_answer_path_seq == "on"; }
bool receiver_hold() { return spray_receiver_hold == "on"; }
uint32_t reps_buffer_size = 8;
uint64_t reps_freezing_timeout_ns = 10000000;
uint32_t ue_ev_set_size = 256;
double ue_saturation_fraction = 0.5;
uint32_t mrc_ev_set_size = 128;
double mrc_skip_base_rtts = 1.0;
double mrc_probe_timeouts = 1.0;
double ack_redirect_omega = 0.5;
double ack_redirect_flag = 0.1;
double ack_redirect_validity_base_rtts = 2.0;
std::string ack_redirect_trim_scoring = "mark";
std::string ack_redirect_scope = "per_connection";
// Under reps_oracle, REPS_ORACLE_HOPS: the hops over which a spine toward the
// destination's leaf is read as faulted. round_trip, the default: the data's
// two hops and the two its acknowledgement returns over through the same
// spine, the destination leaf's uplink to the spine and the spine's downlink
// to the source's leaf. forward: the data's two hops alone, the source leaf's
// uplink to the spine and the spine's downlink to the destination's leaf, for
// answers that do not return over the data's spine, as under ACK_ROUTE spray.
std::string reps_oracle_hops = "round_trip";
// The selectors each parameter belongs to. A parameter given under another
// selector would be read by nothing, so it is refused.
const std::map<std::string, std::set<std::string>> path_selector_parameter_owners = {
    {"REPS_BUFFER_SIZE",
     {"reps", "reps_hold", "reps_flow_hold", "reps_oracle", "reps_spine"}},
    {"REPS_FREEZING_TIMEOUT_NS",
     {"reps", "reps_hold", "reps_flow_hold", "reps_oracle", "reps_spine"}},
    {"UE_EV_SET_SIZE", {"ue_oblivious", "ue_aware"}},
    {"UE_SATURATION_FRACTION", {"ue_aware"}},
    {"MRC_EV_SET_SIZE", {"mrc"}},
    {"MRC_SKIP_BASE_RTTS", {"mrc", "mrc_hold"}},
    {"MRC_PROBE_TIMEOUTS", {"mrc", "mrc_hold"}},
    {"ACK_REDIRECT_OMEGA", {"ack_redirect"}},
    {"ACK_REDIRECT_FLAG", {"ack_redirect"}},
    {"ACK_REDIRECT_VALIDITY_BASE_RTTS", {"ack_redirect"}},
    {"ACK_REDIRECT_TRIM_SCORING", {"ack_redirect"}},
    {"ACK_REDIRECT_SCOPE", {"ack_redirect"}},
    {"REPS_ORACLE_HOPS", {"reps_oracle"}},
};
// The keys of ack_redirect that only its per_connection scope reads.
const std::set<std::string> ack_redirect_connection_keys = {
    "ACK_REDIRECT_VALIDITY_BASE_RTTS", "ACK_REDIRECT_TRIM_SCORING"};
std::set<std::string> path_selector_parameters_given;
// Under LOAD_BALANCING spray_policy, the receiver's and the sender's
// parameters, each RdmaHw's default; times in base RTTs.
double spray_report_interval_base_rtts = 2.0;
double spray_estimator_gain = 1.0 / 16;
uint32_t spray_estimator_interval_samples = 32;
double spray_fraction_cusum_slack = 0.125;
double spray_fraction_cusum_threshold = 0.5;
double spray_congestion_thresholds[3] = {0.25, 0.5, 0.75};
// Zero, unless SPRAY_TRIM_THRESHOLDS is given: the trimmed fraction is graded
// through spray_congestion_thresholds.
double spray_trim_thresholds[3] = {0, 0, 0};
// The least grade a receiver gives a spine for its marked and trimmed
// fractions and its one-way delay; zero leaves them their grade. Above zero
// only the loss grade and a held spine grade a spine below it.
uint32_t spray_congestion_grade_floor = 0;
// Zero, unless SPRAY_LOSS_THRESHOLDS is given: the sender counts no loss, and
// the switch_queue grader grades the loss fraction through
// spray_congestion_thresholds. Zero spray_loss_interval_ns is the report
// interval.
double spray_loss_thresholds[3] = {0, 0, 0};
uint64_t spray_loss_interval_ns = 0;
// Zero, unless SPRAY_LOSS_HOLD_CLEAN_SENTINELS is given: a spine whose lost
// fraction grades it 0 is not held at 0 past the period that graded it.
uint32_t spray_loss_hold_clean_sentinels = 0;
double spray_absence_fraction_of_median = 0.125;
uint32_t spray_absence_minimum_median = 16;
double spray_release_fraction_of_floor = 0.5;
uint32_t spray_one_way_delay = 0;
std::string spray_grade_reference = "median";
std::string spray_grade_scope = "per_receiver";
std::string spray_grade_cost_source = "estimate";
double spray_delay_cusum_slack_base_rtts = 0.125;
double spray_delay_cusum_threshold_base_rtts = 0.5;
double spray_delay_thresholds_base_rtts[3] = {0.25, 0.5, 0.75};
double spray_gamma = 0.25;
double spray_epsilon = 0.02;
uint32_t spray_candidates = 1;
std::string spray_candidate_draw = "proportional";
uint32_t spray_edge_window_penalty = 64;
uint32_t spray_edge_freeze = 1;
double spray_sender_pool_weight = 0;
// Zero, unless SPRAY_POOL_DECAY_INTERVAL_NS is given: the host's pooled
// scores decay once per report taken.
uint64_t spray_pool_decay_interval_ns = 0;
// 1 holds a spine the host's pooled scores hold in every destination's draw.
uint32_t spray_pool_hold_propagates = 0;
// What a spine graded 0 is sent: epsilon, sentinel or none. Zero
// spray_held_sentinel_interval_ns is the report interval.
std::string spray_held_exploration = "epsilon";
uint64_t spray_held_sentinel_interval_ns = 0;
// The keys that only spray_policy reads, and among them those that only its
// one-way delay reads; a key nothing reads is refused.
const std::set<std::string> spray_policy_keys = {
    "SPRAY_REPORT_INTERVAL_BASE_RTTS", "SPRAY_ESTIMATOR_GAIN",
    "SPRAY_FRACTION_CUSUM_SLACK", "SPRAY_FRACTION_CUSUM_THRESHOLD",
    "SPRAY_CONGESTION_THRESHOLDS", "SPRAY_TRIM_THRESHOLDS",
    "SPRAY_CONGESTION_GRADE_FLOOR",
    "SPRAY_LOSS_THRESHOLDS", "SPRAY_LOSS_INTERVAL_NS",
    "SPRAY_LOSS_HOLD_CLEAN_SENTINELS",
    "SPRAY_RELEASE_FRACTION_OF_FLOOR",
    "SPRAY_ONE_WAY_DELAY", "SPRAY_DELAY_CUSUM_SLACK_BASE_RTTS",
    "SPRAY_DELAY_CUSUM_THRESHOLD_BASE_RTTS", "SPRAY_DELAY_THRESHOLDS_BASE_RTTS",
    "SPRAY_GAMMA", "SPRAY_EPSILON", "SPRAY_CANDIDATES", "SPRAY_CANDIDATE_DRAW",
    "SPRAY_EDGE_WINDOW_PENALTY", "SPRAY_EDGE_FREEZE",
    "SPRAY_SENDER_POOL_WEIGHT", "SPRAY_POOL_DECAY_INTERVAL_NS",
    "SPRAY_POOL_HOLD_PROPAGATES",
    "SPINE_REPORT_OUTPUT_FILE",
    "SPINE_LOSS_OUTPUT_FILE", "SPRAY_HELD_EXPLORATION",
    "SPRAY_HELD_SENTINEL_INTERVAL_NS",
    "SPRAY_ABSENCE_FRACTION_OF_MEDIAN", "SPRAY_ABSENCE_MINIMUM_MEDIAN",
    "SPRAY_GRADE_REFERENCE", "SPRAY_GRADE_SCOPE", "SPRAY_GRADE_COST_SOURCE",
    "SPRAY_ESTIMATOR_INTERVAL_SAMPLES"};
const std::set<std::string> spray_one_way_delay_keys = {
    "SPRAY_DELAY_CUSUM_SLACK_BASE_RTTS",
    "SPRAY_DELAY_CUSUM_THRESHOLD_BASE_RTTS",
    "SPRAY_DELAY_THRESHOLDS_BASE_RTTS"};
// The spray_policy keys the host's grader reads, which ack_redirect at its
// per_receiver scope reads too.
const std::set<std::string> spine_grader_keys = {
    "SPRAY_REPORT_INTERVAL_BASE_RTTS", "SPRAY_ESTIMATOR_GAIN",
    "SPRAY_ESTIMATOR_INTERVAL_SAMPLES", "SPRAY_FRACTION_CUSUM_SLACK",
    "SPRAY_FRACTION_CUSUM_THRESHOLD", "SPRAY_ABSENCE_FRACTION_OF_MEDIAN",
    "SPRAY_ABSENCE_MINIMUM_MEDIAN", "SPRAY_RELEASE_FRACTION_OF_FLOOR",
    "SPRAY_ONE_WAY_DELAY", "SPRAY_DELAY_CUSUM_SLACK_BASE_RTTS",
    "SPRAY_DELAY_CUSUM_THRESHOLD_BASE_RTTS"};
// The spray_policy keys the host's loss hold reads, which reps_hold reads
// too, and those of them it requires; reps_flow_hold reads them but
// SPINE_LOSS_OUTPUT_FILE, whose rows are the host's.
const std::set<std::string> loss_hold_keys = {
    "SPRAY_LOSS_THRESHOLDS", "SPRAY_LOSS_INTERVAL_NS",
    "SPRAY_LOSS_HOLD_CLEAN_SENTINELS", "SPRAY_HELD_EXPLORATION",
    "SPRAY_HELD_SENTINEL_INTERVAL_NS", "SPINE_LOSS_OUTPUT_FILE"};
const std::vector<std::string> loss_hold_required_keys = {
    "SPRAY_LOSS_THRESHOLDS", "SPRAY_LOSS_INTERVAL_NS",
    "SPRAY_LOSS_HOLD_CLEAN_SENTINELS", "SPRAY_HELD_EXPLORATION"};
std::set<std::string> spray_policy_keys_given;
// UEC 1.0.3 section 4.1.4.1 RECOMMENDS three traffic classes: TC_low for data,
// TC_med for trimmed packets, TC_high for control. Queue 0 is TC_high here, so
// the trimmed queue must be a distinct non-zero index.
uint32_t packet_trim_queue = 2;
// MIN_TRIM_SIZE in IP payload bytes; UEC 1.0.3 Table 4-1 requires 24 B for UET
// over UDP/IP so the UDP and PDS request headers survive trimming.
uint32_t min_trim_size = 24;
uint32_t packet_trim_lasthop = 1;
// Percent of egress bandwidth TC_med may take from TC_low. UEC 1.0.3 section 4.1
// recommends WDRR at 25% and warns that an unrestricted trimmed class can drive
// congestion collapse.
uint32_t packet_trim_queue_weight = 25;
// UEC 1.0.3 section 3.6.4.5: "PFC SHOULD NOT be used anywhere in a best-effort
// network." Trimming exists to replace lossless operation, so a trimming fabric
// disables PFC and zeroes the PFC headroom that would otherwise act as a second
// buffer no pause mechanism ever drains.
uint32_t enable_pfc = 1;
// Per-queue egress drop thresholds in bytes (0 = unbounded). These are the
// queue_trimmable and queue_trimmed drop thresholds of section 4.1.
uint32_t data_queue_bytes = 0;
uint32_t trimmed_queue_bytes = 0;
// DATA_QUEUE_BYTES_MAP and TRIMMED_QUEUE_BYTES_MAP, as KMAX_MAP: the two
// thresholds of a switch port by its link speed, in place of DATA_QUEUE_BYTES
// and TRIMMED_QUEUE_BYTES; each needs the other and a row for every switch
// port's speed.
map<uint64_t, uint32_t> rate2data_queue_bytes, rate2trimmed_queue_bytes;
uint32_t has_win = 1;
uint32_t global_t = 1;
uint32_t mi_thresh = 5;
bool var_win = false, fast_react = true;
bool multi_rate = true;
bool sample_feedback = false;
double pint_log_base = 1.05;
double pint_prob = 1.0;
double u_target = 0.95;
uint32_t int_multi = 1;
bool rate_bound = true;
int nic_total_pause_time =
    0; // slightly less than finish time without inefficiency in us

uint32_t ack_high_prio = 0;
uint64_t link_down_time = 0;
uint32_t link_down_A = 0, link_down_B = 0;

// The LINK_FAILURE and LINK_RESTORE lines, put in the order they apply in
// once the configuration is read. Each is one line:
//   LINK_FAILURE <time_ns> down <a> <b>
//   LINK_FAILURE <time_ns> blackhole <a> <b>
//   LINK_FAILURE <time_ns> loss <a> <b> <error_rate> [forward|both]
//                [byte <frame_bytes>]
//   LINK_FAILURE <time_ns> rate <a> <b> <data_rate> [forward|both]
//   LINK_RESTORE <time_ns> <a> <b>
// down takes the link down for the rest of the run. blackhole needs b to be a
// switch, which then drops whatever arrives on its port from a: the direction
// from a to b loses everything, the direction from b to a is unchanged. loss
// sets the link's error rate, in (0, 1], and rate its data rate. Without a
// direction, or with both, they apply in both directions; with forward, in the
// direction from a to b alone, as blackhole does: under loss b drops that
// fraction of what it receives from a, and under rate a transmits toward b at
// the data rate, while what b sends to a is unchanged. LINK_RESTORE takes no
// direction: it puts back what one failure of the link changed, in that
// failure's directions, and nothing may follow its two nodes. It restores the
// failure named with the same two nodes in the same order if one is not yet
// restored, else the one named in the other order, so the two directions of a
// link fail and restore independently when each failure and its restore name
// the nodes in the same order. A later failure named in the same order
// replaces the one before it, which a restore then no longer puts back; a link
// taken down is not restored. Under loss, byte makes error_rate each byte's:
// a packet of s bytes as the device receives it, data, acknowledgement,
// repair request or trimmed header alike, is dropped with probability
// 1 - (1 - error_rate)^s, as a frame corrupted in any byte fails its check.
// frame_bytes is the full data frame error_rate was derived at, which must be
// the run's: the PPP, IPv4 and data headers and PACKET_PAYLOAD_SIZE
// (RdmaHw::DataFrameBytes). Without byte every packet is dropped with
// error_rate whatever its size.
vector<LinkEvent> link_events;

uint32_t enable_trace = 1;

uint32_t buffer_size = 16;

uint32_t qlen_dump_interval = 1000, qlen_mon_interval = 100;
uint64_t qlen_mon_start = 0, qlen_mon_end = 2100000000;
int headroom_factor = 3;
string qlen_mon_file;

unordered_map<uint64_t, uint32_t> rate2kmax, rate2kmin;
unordered_map<uint64_t, double> rate2pmax;

/************************************************
 * Runtime varibles
 ***********************************************/
std::ifstream topof, flowf, tracef;

NodeContainer n;

uint64_t nic_rate;

uint64_t maxRtt, maxBdp;

std::vector<Ipv4Address> serverAddress;

// maintain port number for each host pair
std::unordered_map<uint32_t, unordered_map<uint32_t, uint16_t>> portNumber;

struct Interface {
  uint32_t idx;
  bool up;
  uint64_t delay;
  uint64_t bw;

  Interface() : idx(0), up(false) {}
};
map<Ptr<Node>, map<Ptr<Node>, Interface>> nbr2if;
// Mapping destination to next hop for each node: <node, <dest, <nexthop0, ...>
// > >
map<Ptr<Node>, map<Ptr<Node>, vector<Ptr<Node>>>> nextHop;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairDelay;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairTxDelay;
map<uint32_t, map<uint32_t, uint64_t>> pairBw;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairBdp;
map<uint32_t, map<uint32_t, uint64_t>> pairRtt;

struct FlowInput {
  uint32_t src, dst, pg, maxPacketCount, port, dport;
  double start_time;
  uint32_t idx;
};

FlowInput flow_input = {0};
uint32_t flow_num;
Ipv4Address node_id_to_ip(uint32_t id) {
  return Ipv4Address(0x0b000001 + ((id / 256) * 0x00010000) +
                     ((id % 256) * 0x00000100));
}

uint32_t ip_to_node_id(Ipv4Address ip) { return (ip.Get() >> 8) & 0xffff; }

void get_pfc(FILE *fout, Ptr<QbbNetDevice> dev, uint32_t type,
             uint32_t queue) {
  fprintf(fout, "%lu %u %u %u %u %u\n", Simulator::Now().GetTimeStep(),
          dev->GetNode()->GetId(), dev->GetNode()->GetNodeType(),
          dev->GetIfIndex(), queue, type);
}

// The analyzer reduces transport telemetry to per-(event, plane) counts and
// byte totals, so the simulator aggregates in memory and periodically
// rewrites a small summary file. The raw per-packet log survives as a
// research artifact in its historic ns-3 CSV syntax, but zstd-compressed on
// a library worker thread (the second CI core): uncompressed and flushed
// per row it grew past 160 GB per arm, and the write load - not the
// simulation - is what exhausted shared storage and killed whole runner
// fleets. If compression ever falls behind, ZSTD_compressStream2 blocks the
// producer rather than dropping rows.
FILE *transport_event_summary_file = nullptr;
map<pair<string, string>, pair<uint64_t, uint64_t>> transport_event_totals;
FILE *transport_event_raw_file = nullptr;
ZSTD_CCtx *transport_event_raw_cctx = nullptr;
string transport_event_raw_pending;
vector<char> transport_event_raw_scratch;
time_t transport_event_last_flush = 0;
// The raw stream is a sequence of independently decompressible zstd frames
// ("segments"), rotated when a segment's compressed size crosses the limit
// below. Segments exist so a multi-gigabyte run can ship its raw log
// incrementally and every shipped piece stays under the 2 GiB
// release-asset cap; `cat <name>.zst.* | zstd -d` reconstructs the exact
// historic single-file CSV (the header lives once, in segment .000, so a
// surviving mid-run segment is still headerless rows in the same syntax).
// Rotation only ever happens at a row boundary: rows enter the pending
// buffer whole, and a frame ends only after that buffer fully drains.
uint32_t transport_event_raw_segment_index = 0;
uint64_t transport_event_raw_segment_bytes = 0;
uint64_t transport_event_segment_byte_limit = 1800000000ull;

std::string transport_event_segment_path(uint32_t index) {
  char suffix[8];
  snprintf(suffix, sizeof(suffix), ".%03u", index);
  return transport_event_output_file + suffix;
}

void flush_transport_event_raw(ZSTD_EndDirective directive) {
  ZSTD_inBuffer input = {transport_event_raw_pending.data(),
                         transport_event_raw_pending.size(), 0};
  bool draining = true;
  while (draining) {
    ZSTD_outBuffer output = {transport_event_raw_scratch.data(),
                             transport_event_raw_scratch.size(), 0};
    const size_t remaining =
        ZSTD_compressStream2(transport_event_raw_cctx, &output, &input,
                             directive);
    if (ZSTD_isError(remaining)) {
      std::cerr << "transport event raw stream: "
                << ZSTD_getErrorName(remaining) << "\n";
      break;
    }
    if (output.pos != 0) {
      fwrite(transport_event_raw_scratch.data(), 1, output.pos,
             transport_event_raw_file);
      transport_event_raw_segment_bytes += output.pos;
    }
    // e_flush and e_end must drain the worker completely; e_continue only
    // needs the pending input accepted.
    draining = directive == ZSTD_e_continue ? input.pos < input.size
                                            : remaining != 0;
  }
  transport_event_raw_pending.clear();
}

void rotate_transport_event_raw_segment() {
  // Seal the current segment as a complete zstd frame and start the next
  // one on the same compression parameters. A sealed segment is immutable
  // on disk from this instant - which is the entire coordination protocol
  // with the mid-run uploader: the existence of segment N+1 is the proof
  // that segment N may be shipped and deleted.
  flush_transport_event_raw(ZSTD_e_end);
  fclose(transport_event_raw_file);
  transport_event_raw_segment_index++;
  transport_event_raw_file = fopen(
      transport_event_segment_path(transport_event_raw_segment_index).c_str(),
      "wb");
  if (transport_event_raw_file == nullptr) {
    // Losing the raw stream must not kill a multi-day simulation: the
    // summary file carries everything the analyzer needs regardless.
    std::cerr << "transport event raw stream: cannot open segment "
              << transport_event_raw_segment_index
              << "; raw logging stops here\n";
    ZSTD_freeCCtx(transport_event_raw_cctx);
    transport_event_raw_cctx = nullptr;
    return;
  }
  ZSTD_CCtx_reset(transport_event_raw_cctx, ZSTD_reset_session_only);
  transport_event_raw_segment_bytes = 0;
}

void append_transport_event_raw(const char *row, int length) {
  if (length <= 0)
    return;
  transport_event_raw_pending.append(row, static_cast<size_t>(length));
  if (transport_event_raw_pending.size() >= (1u << 22))
    flush_transport_event_raw(ZSTD_e_continue);
  // Checked after the write path, so a segment can overshoot the limit by
  // at most one drained burst; the limit leaves ~350 MB of headroom under
  // the 2 GiB cap for exactly that reason.
  if (transport_event_raw_segment_bytes >= transport_event_segment_byte_limit)
    rotate_transport_event_raw_segment();
}

void finalize_transport_event_raw() {
  if (transport_event_raw_cctx == nullptr)
    return;
  flush_transport_event_raw(ZSTD_e_end);
  fclose(transport_event_raw_file);
  ZSTD_freeCCtx(transport_event_raw_cctx);
  transport_event_raw_cctx = nullptr;
  transport_event_raw_file = nullptr;
}

// Rewritten in place on every flush, so the totals are observable while the
// simulation runs and complete at exit.
void write_transport_event_summary() {
  if (transport_event_summary_file == nullptr)
    return;
  rewind(transport_event_summary_file);
  fprintf(transport_event_summary_file, "event,plane,event_count,total_bytes\n");
  for (const auto &entry : transport_event_totals)
    fprintf(transport_event_summary_file, "%s,%s,%lu,%lu\n",
            entry.first.first.c_str(), entry.first.second.c_str(),
            static_cast<unsigned long>(entry.second.first),
            static_cast<unsigned long>(entry.second.second));
  fflush(transport_event_summary_file);
}

// Optimistic flush: with the per-row fflush gone, land both outputs about
// once a minute so a hard kill loses at most the trailing window and the
// artifacts stay inspectable mid-run.
void maybe_flush_transport_event_outputs() {
  const time_t now = time(nullptr);
  if (now - transport_event_last_flush < 60)
    return;
  transport_event_last_flush = now;
  write_transport_event_summary();
  if (transport_event_raw_cctx != nullptr) {
    flush_transport_event_raw(ZSTD_e_flush);
    fflush(transport_event_raw_file);
  }
}

void accumulate_transport_event(const string &event, const char *plane,
                                uint64_t bytes) {
  auto &totals = transport_event_totals[{event, plane}];
  totals.first += 1;
  totals.second += bytes;
  maybe_flush_transport_event_outputs();
}

// A host-transport reaction carries no packet, so it contributes a count and
// no bytes. It rides the control plane because that is what it answers to: a
// retransmission timeout is the absence of an ACK, a rate cut is a CNP.
void record_host_transport_event(const char *event, uint64_t bytes) {
  // The two forgiveness events account for payload bytes no receiver was ever
  // given, so they ride the data plane beside the switch's own trim events.
  // The other reactions carry no packet and answer to the control plane: a
  // retransmission timeout is a missing ACK, a rate cut is a CNP.
  const char *plane = (strcmp(event, "trim_forgiven") == 0 ||
                       strcmp(event, "remainder_forgiven") == 0)
                          ? "data"
                          : "control";
  accumulate_transport_event(event, plane, bytes);
}

void write_transport_event(const char *event, Ptr<QbbNetDevice> dev,
                           Ptr<const Packet> packet, uint32_t protocol,
                           int32_t queue) {
  CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header |
                  CustomHeader::L4_Header);
  ch.getInt = 1;
  packet->PeekHeader(ch);
  // A trimmed packet rides the UDP protocol number but carries no payload, so
  // it is reported on the control plane alongside ACKs and NACKs.
  const bool payload_bearing =
      protocol == 0x11 && !IsUetTrimmedDscp(ch.GetIpv4Dscp());
  accumulate_transport_event(event, payload_bearing ? "data" : "control",
                             packet->GetSize());
  if (transport_event_raw_cctx == nullptr)
    return;
  uint16_t source_port = 0;
  uint32_t sequence = 0;
  if (protocol == 0x11) {
    source_port = ch.udp.sport;
    sequence = ch.udp.seq;
  } else if (CarriesQbbHeader(protocol)) {
    source_port = ch.ack.sport;
    sequence = ch.ack.seq;
  }
  char row[224];
  const int length = snprintf(
      row, sizeof(row), "%lu,%s,%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d\n",
      Simulator::Now().GetNanoSeconds(), event,
      payload_bearing ? "data" : "control", protocol,
      dev->GetNode()->GetId(), dev->GetNode()->GetNodeType(),
      dev->GetIfIndex(), ip_to_node_id(Ipv4Address(ch.sip)),
      ip_to_node_id(Ipv4Address(ch.dip)), source_port, sequence,
      packet->GetSize(), queue);
  append_transport_event_raw(row, length);
}

void get_transport_event(const char *event, Ptr<QbbNetDevice> dev,
                         Ptr<const Packet> packet, uint32_t protocol) {
  write_transport_event(event, dev, packet, protocol, -1);
}

void get_queue_event(const char *event, Ptr<QbbNetDevice> dev,
                     Ptr<const Packet> packet, uint32_t queue) {
  CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header |
                  CustomHeader::L4_Header);
  ch.getInt = 1;
  packet->PeekHeader(ch);
  write_transport_event(event, dev, packet, ch.l3Prot,
                        static_cast<int32_t>(queue));
}

// The receiving end of every leaf-spine link, by node id and device index:
// the spine's index, by the rank of its node id as AssignSpinePorts indexes
// it, and whether the link goes up, from the leaf to the spine.
std::map<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, bool>>
    spine_link_receivers;

// An answer a receiver sends to a sender, an acknowledgement, a NACK or a
// repair request, lost where node's device port receives it: if that device
// ends a leaf-spine link, counted as answer_drop_up or answer_drop_down by
// the link's direction, and again with _spine<k> appended for its spine k,
// the breakdown by spine. The losses are those switch_blackhole_drop and
// link_error_drop count already.
void count_answer_drop(uint32_t node, uint32_t port, uint8_t protocol,
                       uint64_t bytes) {
  if (protocol != 0xFC && protocol != 0xFD &&
      protocol != kUecTrimRepairProtocol)
    return;
  const auto link = spine_link_receivers.find({node, port});
  if (link == spine_link_receivers.end())
    return;
  const string event =
      string("answer_drop_") + (link->second.second ? "up" : "down");
  accumulate_transport_event(event, "control", bytes);
  accumulate_transport_event(event + "_spine" + std::to_string(link->second.first),
                             "control", bytes);
}

void get_answer_link_error_drop(Ptr<QbbNetDevice> dev, Ptr<const Packet> packet,
                                uint32_t protocol) {
  count_answer_drop(dev->GetNode()->GetId(), dev->GetIfIndex(), protocol,
                    packet->GetSize());
}

void get_switch_drop(Ptr<SwitchNode> sw,
                     Ptr<const Packet> packet, uint32_t reason) {
  CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header |
                  CustomHeader::L4_Header);
  ch.getInt = 1;
  packet->PeekHeader(ch);
  const bool trimmed = IsUetTrimmedDscp(ch.GetIpv4Dscp());
  const bool udp = ch.l3Prot == 0x11;
  const char *event = "switch_unknown_drop";
  switch (static_cast<SwitchDropReason>(reason)) {
  case SwitchDropReason::Route:
    event = "switch_route_drop";
    break;
  case SwitchDropReason::Admission:
    event = "switch_admission_drop";
    break;
  case SwitchDropReason::EgressQueue:
    event = "switch_egress_queue_drop";
    break;
  case SwitchDropReason::TrimmedQueue:
    // UEC 1.0.3 section 4.1: trimmed packets remain subject to loss at the
    // trimming switch and at every downstream TC_med queue.
    event = "switch_trimmed_queue_drop";
    break;
  case SwitchDropReason::Blackhole:
    event = "switch_blackhole_drop";
    break;
  }
  accumulate_transport_event(event, (udp || trimmed) ? "data" : "control",
                             packet->GetSize());
  // A black hole drops what arrives on its port, which the device tagged.
  FlowIdTag port;
  if (static_cast<SwitchDropReason>(reason) == SwitchDropReason::Blackhole &&
      packet->PeekPacketTag(port))
    count_answer_drop(sw->GetId(), port.GetFlowId(), ch.l3Prot,
                      packet->GetSize());
  if (transport_event_raw_cctx == nullptr)
    return;
  const uint16_t source_port =
      udp ? ch.udp.sport : (trimmed ? ch.ack.sport : 0);
  const uint32_t sequence = udp ? ch.udp.seq : (trimmed ? ch.ack.seq : 0);
  char row[224];
  const int length = snprintf(
      row, sizeof(row), "%lu,%s,%s,%u,%u,%u,-1,%u,%u,%u,%u,%u,-1\n",
      Simulator::Now().GetNanoSeconds(), event,
      (udp || trimmed) ? "data" : "control", ch.l3Prot, sw->GetId(),
      sw->GetNodeType(), ip_to_node_id(Ipv4Address(ch.sip)),
      ip_to_node_id(Ipv4Address(ch.dip)), source_port, sequence,
      packet->GetSize());
  append_transport_event_raw(row, length);
}

void get_switch_trim(Ptr<SwitchNode> sw,
                     Ptr<const Packet> packet, uint32_t trigger) {
  CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header |
                  CustomHeader::L4_Header);
  ch.getInt = 1;
  packet->PeekHeader(ch);
  // A UEC-conformant trim keeps the original UDP packet, truncated and remarked
  // DSCP_TRIMMED; the non-UET back-to-sender mode emits its own notification.
  const bool forwardToDestination = ch.l3Prot == 0x11;
  // The UDP length field is not modified by trimming, so it still reports the
  // payload that the trim replaced.
  const uint32_t trimmed_payload =
      forwardToDestination
          ? (ch.udp.payload_size > ch.UdpHeaderBytes()
                 ? ch.udp.payload_size - ch.UdpHeaderBytes()
                 : 0)
          : ch.ack.trim_payload_size;
  const char *mode = forwardToDestination ? "ftd" : "bts";
  const char *reason = "unknown";
  switch (static_cast<PacketTrimTrigger>(trigger)) {
  case PacketTrimTrigger::Admission:
    reason = "admission";
    break;
  case PacketTrimTrigger::EgressQueue:
    reason = "egress_queue";
    break;
  case PacketTrimTrigger::AdmissionLastHop:
    reason = "lasthop_admission";
    break;
  case PacketTrimTrigger::EgressQueueLastHop:
    reason = "lasthop_egress_queue";
    break;
  }
  accumulate_transport_event(string("trim_") + mode + "_" + reason, "data",
                             trimmed_payload);
  if (transport_event_raw_cctx == nullptr)
    return;
  char row[224];
  const int length = snprintf(
      row, sizeof(row), "%lu,trim_%s_%s,data,%u,%u,%u,-1,%u,%u,%u,%u,%u,-1\n",
      Simulator::Now().GetNanoSeconds(), mode, reason, ch.l3Prot, sw->GetId(),
      sw->GetNodeType(),
      ip_to_node_id(Ipv4Address(forwardToDestination ? ch.sip : ch.dip)),
      ip_to_node_id(Ipv4Address(forwardToDestination ? ch.dip : ch.sip)),
      forwardToDestination ? ch.udp.sport : ch.ack.dport,
      forwardToDestination ? ch.udp.seq : ch.ack.seq, trimmed_payload);
  append_transport_event_raw(row, length);
}

uint32_t data_loss_scope_value() {
  if (data_loss_scope == "all")
    return static_cast<uint32_t>(DataLossScope::All);
  if (data_loss_scope == "host_to_switch")
    return static_cast<uint32_t>(DataLossScope::HostToSwitch);
  if (data_loss_scope == "switch_to_host")
    return static_cast<uint32_t>(DataLossScope::SwitchToHost);
  if (data_loss_scope == "switch_to_switch")
    return static_cast<uint32_t>(DataLossScope::SwitchToSwitch);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t packet_trim_mode_value() {
  if (packet_trim_mode == "disabled")
    return static_cast<uint32_t>(PacketTrimMode::Disabled);
  if (packet_trim_mode == "ftd")
    return static_cast<uint32_t>(PacketTrimMode::ForwardToDestination);
  if (packet_trim_mode == "bts")
    return static_cast<uint32_t>(PacketTrimMode::BackToSender);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t path_selector_value() {
  if (path_selector == "ops")
    return static_cast<uint32_t>(PathSelectorKind::Ops);
  if (path_selector == "reps")
    return static_cast<uint32_t>(PathSelectorKind::Reps);
  if (path_selector == "ue_oblivious")
    return static_cast<uint32_t>(PathSelectorKind::UeOblivious);
  if (path_selector == "ue_aware")
    return static_cast<uint32_t>(PathSelectorKind::UeAware);
  if (path_selector == "mrc")
    return static_cast<uint32_t>(PathSelectorKind::Mrc);
  if (path_selector == "ack_redirect")
    return static_cast<uint32_t>(PathSelectorKind::AckRedirect);
  if (path_selector == "reps_hold")
    return static_cast<uint32_t>(PathSelectorKind::RepsHold);
  if (path_selector == "reps_flow_hold")
    return static_cast<uint32_t>(PathSelectorKind::RepsFlowHold);
  if (path_selector == "reps_oracle")
    return static_cast<uint32_t>(PathSelectorKind::RepsOracle);
  if (path_selector == "mrc_hold")
    return static_cast<uint32_t>(PathSelectorKind::MrcHold);
  if (path_selector == "reps_spine")
    return static_cast<uint32_t>(PathSelectorKind::RepsSpine);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t ack_redirect_trim_scoring_value() {
  if (ack_redirect_trim_scoring == "mark")
    return static_cast<uint32_t>(TrimScoring::Mark);
  if (ack_redirect_trim_scoring == "ignore")
    return static_cast<uint32_t>(TrimScoring::Ignore);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t load_balancing_value() {
  if (load_balancing == "ecmp")
    return static_cast<uint32_t>(LoadBalancingMode::Ecmp);
  if (load_balancing == "ev_hash")
    return static_cast<uint32_t>(LoadBalancingMode::EntropyHash);
  if (load_balancing == "spray_uniform")
    return static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
  if (load_balancing == "spray_policy")
    return static_cast<uint32_t>(LoadBalancingMode::SprayPolicy);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t spray_hold_rule_value() {
  if (spray_hold_rule == "count")
    return static_cast<uint32_t>(HoldRule::Count);
  if (spray_hold_rule == "llr")
    return static_cast<uint32_t>(HoldRule::Llr);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t spray_sentinel_mode_value() {
  if (spray_sentinel_mode == "data")
    return static_cast<uint32_t>(SentinelMode::Data);
  if (spray_sentinel_mode == "duplicate")
    return static_cast<uint32_t>(SentinelMode::Duplicate);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t spray_hold_release_value() {
  if (spray_hold_release == "sentinels")
    return static_cast<uint32_t>(HoldRelease::Sentinels);
  if (spray_hold_release == "trickle")
    return static_cast<uint32_t>(HoldRelease::Trickle);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t spray_hold_scope_value() {
  if (spray_hold_scope == "host")
    return static_cast<uint32_t>(HoldScope::Host);
  if (spray_hold_scope == "leaf")
    return static_cast<uint32_t>(HoldScope::Leaf);
  if (spray_hold_scope == "auto")
    return static_cast<uint32_t>(HoldScope::Auto);
  if (spray_hold_scope == "pair")
    return static_cast<uint32_t>(HoldScope::Pair);
  return std::numeric_limits<uint32_t>::max();
}

// Under reps_hold and mrc_hold, whether a loss hold is kept per destination
// leaf, under leaf or auto, which needs the hosts' leaves.
bool leaf_hold() {
  return hold_scope_read() &&
         (spray_hold_scope == "leaf" || spray_hold_scope == "auto");
}

// Under reps_hold and mrc_hold, whether a loss hold is kept per destination
// host, under pair.
bool pair_hold() { return hold_scope_read() && spray_hold_scope == "pair"; }

uint32_t spray_candidate_draw_value() {
  if (spray_candidate_draw == "proportional")
    return static_cast<uint32_t>(SpineScores::CandidateDraw::Proportional);
  if (spray_candidate_draw == "uniform")
    return static_cast<uint32_t>(SpineScores::CandidateDraw::Uniform);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t spray_grade_reference_value() {
  if (spray_grade_reference == "absolute")
    return static_cast<uint32_t>(SpineGrader::GradeReference::Absolute);
  if (spray_grade_reference == "median")
    return static_cast<uint32_t>(SpineGrader::GradeReference::Median);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t grade_scope_value(const std::string &scope) {
  if (scope == "per_receiver")
    return static_cast<uint32_t>(GradeScope::PerReceiver);
  if (scope == "per_connection")
    return static_cast<uint32_t>(GradeScope::PerConnection);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t spray_grade_cost_source_value() {
  if (spray_grade_cost_source == "estimate")
    return static_cast<uint32_t>(GradeCostSource::Estimate);
  if (spray_grade_cost_source == "switch_queue")
    return static_cast<uint32_t>(GradeCostSource::SwitchQueue);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t spray_held_exploration_value() {
  if (spray_held_exploration == "epsilon")
    return static_cast<uint32_t>(SpineScores::HeldExploration::Epsilon);
  if (spray_held_exploration == "sentinel")
    return static_cast<uint32_t>(SpineScores::HeldExploration::Sentinel);
  if (spray_held_exploration == "none")
    return static_cast<uint32_t>(SpineScores::HeldExploration::None);
  return std::numeric_limits<uint32_t>::max();
}

bool switch_queue_cost() {
  return load_balancing == "spray_policy" &&
         spray_grade_cost_source == "switch_queue";
}

// Under reps_oracle, whether the switches' state is read for the spines'
// faults.
bool reps_oracle() {
  return path_selector == "reps_oracle" &&
         spray_grade_cost_source == "switch_queue";
}

// Under ack_redirect, whether the redirect is scored from the host's grader.
bool host_redirect() {
  return path_selector == "ack_redirect" && ack_redirect_scope == "per_receiver";
}

bool names_spine() {
  return NamesSpine(static_cast<LoadBalancingMode>(load_balancing_value()));
}

void configure_data_loss(Ptr<QbbNetDevice> dev, uint64_t stream_offset) {
  dev->SetAttribute("DataLossStartNs", UintegerValue(data_loss_start_ns));
  dev->SetAttribute("DataLossDurationNs", UintegerValue(data_loss_duration_ns));
  dev->SetAttribute("DataLossScope", UintegerValue(data_loss_scope_value()));
  dev->SetAttribute("DataLossSourceHost", IntegerValue(data_loss_source_host));
  dev->SetAttribute("DataLossDestinationHost",
                    IntegerValue(data_loss_destination_host));
  dev->SetAttribute("DataLossReceiverNode", IntegerValue(data_loss_receiver_node));
  if (data_loss_duration_ns == 0)
    return;
  Ptr<RateErrorModel> model = CreateObject<RateErrorModel>();
  Ptr<UniformRandomVariable> random = CreateObject<UniformRandomVariable>();
  model->SetRandomVariable(random);
  random->SetStream(static_cast<int64_t>(data_loss_rng_stream + stream_offset));
  model->SetAttribute("ErrorRate", DoubleValue(data_loss_probability));
  model->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
  dev->SetAttribute("DataLossErrorModel", PointerValue(model));
}

// Loss on the link itself, which every packet arriving at dev is exposed to.
// A device keeps one model for the run, so a later change of rate continues
// the same stream of draws.
void set_link_error_rate(Ptr<QbbNetDevice> dev, double error_rate) {
  PointerValue current;
  dev->GetAttribute("LinkErrorModel", current);
  Ptr<RateErrorModel> model = current.Get<RateErrorModel>();
  if (!model) {
    model = CreateObject<RateErrorModel>();
    model->SetRandomVariable(CreateObjectWithAttributes<UniformRandomVariable>(
        "Stream",
        IntegerValue(device_rng_stream(DeviceRngBlock::LinkError, dev))));
    model->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
    dev->SetAttribute("LinkErrorModel", PointerValue(model));
  }
  model->SetRate(error_rate);
}

void connect_transport_traces(Ptr<QbbNetDevice> dev) {
  if (data_loss_duration_ns != 0 ||
      packet_trim_mode_value() !=
          static_cast<uint32_t>(PacketTrimMode::Disabled)) {
    dev->TraceConnectWithoutContext(
        "DataPlaneAttempt", MakeBoundCallback(&get_transport_event,
                                                "data_arrival", dev));
    dev->TraceConnectWithoutContext(
        "DataPlaneDeliver", MakeBoundCallback(&get_transport_event,
                                                "data_deliver", dev));
    dev->TraceConnectWithoutContext(
        "DataPlaneLoss", MakeBoundCallback(&get_transport_event,
                                             "data_injected_drop", dev));
    dev->TraceConnectWithoutContext(
        "ControlPlaneAttempt", MakeBoundCallback(&get_transport_event,
                                                   "control_arrival", dev));
    dev->TraceConnectWithoutContext(
        "ControlPlaneDeliver", MakeBoundCallback(&get_transport_event,
                                                   "control_deliver", dev));
  }
  dev->TraceConnectWithoutContext(
      "QbbDrop", MakeBoundCallback(&get_queue_event, "qbb_drop", dev));
  dev->TraceConnectWithoutContext(
      "LinkErrorDrop", MakeBoundCallback(&get_transport_event,
                                         "link_error_drop", dev));
  dev->TraceConnectWithoutContext(
      "LinkErrorDrop", MakeBoundCallback(&get_answer_link_error_drop, dev));
}

struct QlenDistribution {
  vector<uint32_t>
      cnt; // cnt[i] is the number of times that the queue len is i KB

  void add(uint32_t qlen) {
    uint32_t kb = qlen / 1000;
    if (cnt.size() < kb + 1)
      cnt.resize(kb + 1);
    cnt[kb]++;
  }
};
map<uint32_t, map<uint32_t, uint32_t>> queue_result;
void monitor_buffer(FILE *qlen_output, NodeContainer *n) {
  for (uint32_t i = 0; i < n->GetN(); i++) {
    if (n->Get(i)->GetNodeType() == 1) { // is switch
      Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(n->Get(i));
      if (queue_result.find(i) == queue_result.end())
        queue_result[i];
      // fprintf(qlen_output, "\n");
      // fprintf(qlen_output, "time: %lu\n", Simulator::Now().GetTimeStep());
      int test = 0;
      for (uint32_t j = 1; j < sw->GetNDevices(); j++) {
        uint32_t size = 0;
        for (uint32_t k = 0; k < SwitchMmu::qCnt; k++)
          size += sw->m_mmu->egress_bytes[j][k];
        // if (queue_result[i].find(j) == queue_result[i].end())
        //{
        //	vector<uint32_t> v;
        //	queue_result[i][j] = v;
        // }
        if (size >= 1000) {
          queue_result[i][j] = size; // .push_back(size);
          if (test == 0) {
            test = 1;
            fprintf(qlen_output, "time %lu %u ", Simulator::Now().GetTimeStep(),
                    i);
          }
          // if(j==1){
          // fprintf(qlen_output, "t %lu %u j %u %u ",
          // Simulator::Now().GetTimeStep(), i, j, size);
          if (j < sw->GetNDevices() - 1) {
            test = 2;
            fprintf(qlen_output, "j %u %u ", j, size);
          } else if (j == sw->GetNDevices() - 1) {
            fprintf(qlen_output, "j %u %u\n", j, size);
            test = 3;
          }
        }
        if (j == sw->GetNDevices() - 1 && test == 2) {
          fprintf(qlen_output, "\n");
        }
        // else
        //	queue_result[i][j]+=size;
        // queue_result[i][j].add(size);
      }
      // Flush once after the complete sample. Flushing each switch at a
      // packet-scale cadence can dominate high-rate simulations.
    }
  }
  fflush(qlen_output);
  Simulator::Schedule(NanoSeconds(qlen_mon_interval), &monitor_buffer,
                      qlen_output, n);
}

void CalculateRoute(Ptr<Node> host) {
  // queue for the BFS.
  vector<Ptr<Node>> q;
  // Distance from the host to each node.
  map<Ptr<Node>, int> dis;
  map<Ptr<Node>, uint64_t> delay;
  map<Ptr<Node>, uint64_t> txDelay;
  map<Ptr<Node>, uint64_t> bw;
  // init BFS.
  q.push_back(host);
  dis[host] = 0;
  delay[host] = 0;
  txDelay[host] = 0;
  bw[host] = 0xfffffffffffffffflu;
  // BFS. Among the shortest-hop paths to a node, the delay, serialization and
  // bottleneck bandwidth are each the best any of them offers, so that a slow
  // link the routing may avoid sets no base RTT or BDP; BFS order makes each
  // node's values final before it is expanded.
  for (int i = 0; i < (int)q.size(); i++) {
    Ptr<Node> now = q[i];
    int d = dis[now];
    for (auto it = nbr2if[now].begin(); it != nbr2if[now].end(); it++) {
      // skip down link
      if (!it->second.up)
        continue;
      Ptr<Node> next = it->first;
      const uint64_t pathDelay = delay[now] + it->second.delay;
      const uint64_t pathTxDelay =
          txDelay[now] + packet_payload_size * 1000000000lu * 8 / it->second.bw;
      const uint64_t pathBw = std::min(bw[now], it->second.bw);
      if (dis.find(next) == dis.end()) {
        dis[next] = d + 1;
        delay[next] = pathDelay;
        txDelay[next] = pathTxDelay;
        bw[next] = pathBw;
        if (next->GetNodeType() == 1)
          q.push_back(next);
      } else if (d + 1 == dis[next]) {
        delay[next] = std::min(delay[next], pathDelay);
        txDelay[next] = std::min(txDelay[next], pathTxDelay);
        bw[next] = std::max(bw[next], pathBw);
      }
      if (d + 1 == dis[next]) {
        nextHop[next][host].push_back(now);
      }
    }
  }
  for (auto it : delay) {
    // std::cout << "pairDelay first "<< it.first->GetId() << " host " <<
    // host->GetId() << " delay " << it.second << std::endl;
    pairDelay[it.first][host] = it.second;
  }
  for (auto it : txDelay)
    pairTxDelay[it.first][host] = it.second;
  for (auto it : bw) {
    // std::cout << "pairBw first "<< it.first->GetId() << " host " <<
    // host->GetId() << " bw " << it.second << std::endl;
    pairBw[it.first->GetId()][host->GetId()] = it.second;
  }
}

void CalculateRoutes(NodeContainer &n) {
  for (int i = 0; i < (int)n.GetN(); i++) {
    Ptr<Node> node = n.Get(i);
    if (node->GetNodeType() == 0)
      CalculateRoute(node);
  }
}

void SetRoutingEntries() {
  for (auto i = nextHop.begin(); i != nextHop.end(); i++) {
    Ptr<Node> node = i->first;
    auto &table = i->second;
    for (auto j = table.begin(); j != table.end(); j++) {
      Ptr<Node> dst = j->first;
      Ipv4Address dstAddr = dst->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
      vector<Ptr<Node>> nexts = j->second;
      for (int k = 0; k < (int)nexts.size(); k++) {
        Ptr<Node> next = nexts[k];
        uint32_t interface = nbr2if[node][next].idx;
        if (node->GetNodeType() == 1)
          DynamicCast<SwitchNode>(node)->AddTableEntry(dstAddr, interface);
        else {
          node->GetObject<RdmaDriver>()->m_rdma->AddTableEntry(dstAddr,
                                                               interface);
        }
      }
    }
  }
}

// Spines are the switches with no host attached, indexed by the rank of their
// node id, so an index names the same spine in every run of a topology. Each
// leaf learns its uplink to every spine in that order. Returns the number of
// spines, or zero when there is none or a leaf lacks a link to one of them.
// The switches with a host attached, in node order, and the others, the
// spines, in node-id order.
void LeavesAndSpines(NodeContainer &n, vector<Ptr<Node>> &leaves,
                     vector<Ptr<Node>> &spines) {
  for (uint32_t i = 0; i < n.GetN(); i++) {
    Ptr<Node> node = n.Get(i);
    if (node->GetNodeType() != 1)
      continue;
    const bool hosts_attached =
        std::any_of(nbr2if[node].begin(), nbr2if[node].end(),
                    [](const auto &link) { return link.first->GetNodeType() == 0; });
    (hosts_attached ? leaves : spines).push_back(node);
  }
  std::sort(spines.begin(), spines.end(),
            [](Ptr<Node> a, Ptr<Node> b) { return a->GetId() < b->GetId(); });
}

// PAIR_BANDWIDTH spines: every pair of hosts on two leaves of a leaf-spine
// fabric gets the bandwidth of its packets sprayed over every spine, the sum
// over the spines of the lesser of the source leaf's link up to it and its
// link down to the destination leaf, links taken down left out, at most
// either host's link. A pair on one leaf keeps CalculateRoute's.
void SprayPairBandwidth(NodeContainer &n) {
  vector<Ptr<Node>> leaves, spines;
  LeavesAndSpines(n, leaves, spines);
  // Each host's leaf and its link's bandwidth.
  map<Ptr<Node>, std::pair<Ptr<Node>, uint64_t>> attached;
  for (Ptr<Node> leaf : leaves)
    for (const auto &link : nbr2if[leaf])
      if (link.first->GetNodeType() == 0)
        attached[link.first] = {leaf, link.second.bw};
  for (const auto &source : attached) {
    for (const auto &destination : attached) {
      const Ptr<Node> from = source.second.first;
      const Ptr<Node> to = destination.second.first;
      if (from == to)
        continue;
      uint64_t sprayed = 0;
      for (Ptr<Node> spine : spines) {
        const auto up = nbr2if[from].find(spine);
        const auto down = nbr2if[spine].find(to);
        if (up == nbr2if[from].end() || down == nbr2if[spine].end() ||
            !up->second.up || !down->second.up)
          continue;
        sprayed += std::min(up->second.bw, down->second.bw);
      }
      pairBw[source.first->GetId()][destination.first->GetId()] = std::min(
          {source.second.second, destination.second.second, sprayed});
    }
  }
}

uint32_t AssignSpinePorts(NodeContainer &n) {
  vector<Ptr<Node>> leaves, spines;
  LeavesAndSpines(n, leaves, spines);
  for (Ptr<Node> leaf : leaves) {
    vector<uint32_t> ports;
    for (Ptr<Node> spine : spines) {
      auto link = nbr2if[leaf].find(spine);
      if (link == nbr2if[leaf].end())
        return 0;
      ports.push_back(link->second.idx);
    }
    DynamicCast<SwitchNode>(leaf)->SetSpinePorts(ports);
  }
  return spines.size();
}

// spine_link_receivers, for a fabric of leaves and spines.
void FindSpineLinkReceivers(NodeContainer &n) {
  vector<Ptr<Node>> leaves, spines;
  LeavesAndSpines(n, leaves, spines);
  for (Ptr<Node> leaf : leaves) {
    for (uint32_t k = 0; k < spines.size(); k++) {
      const auto link = nbr2if[leaf].find(spines[k]);
      if (link == nbr2if[leaf].end())
        continue;
      spine_link_receivers[{spines[k]->GetId(),
                            nbr2if[spines[k]][leaf].idx}] = {k, true};
      spine_link_receivers[{leaf->GetId(), link->second.idx}] = {k, false};
    }
  }
}

// Simulation only: every hop of every leaf-spine path, for a receiver that
// grades the spines from the switches' own state, and every host's leaf.
// Leaves are numbered in node order; every leaf links to every spine, as
// AssignSpinePorts has checked.
Ptr<SwitchQueueState> BuildSwitchQueueState(NodeContainer &n) {
  vector<Ptr<Node>> leaves, spines;
  LeavesAndSpines(n, leaves, spines);
  Ptr<SwitchQueueState> state =
      Create<SwitchQueueState>(leaves.size(), spines.size());
  for (uint32_t s = 0; s < leaves.size(); s++) {
    Ptr<SwitchNode> leaf = DynamicCast<SwitchNode>(leaves[s]);
    for (uint32_t k = 0; k < spines.size(); k++) {
      Ptr<SwitchNode> spine = DynamicCast<SwitchNode>(spines[k]);
      const uint32_t up = nbr2if[leaf][spine].idx;
      const uint32_t down = nbr2if[spine][leaf].idx;
      state->SetUplink(s, k, SwitchQueueState::Hop{leaf, up, spine, down});
      state->SetDownlink(k, s, SwitchQueueState::Hop{spine, down, leaf, up});
    }
    for (const auto &link : nbr2if[leaf])
      if (link.first->GetNodeType() == 0)
        state->SetLeafOf(node_id_to_ip(link.first->GetId()).Get(), s);
  }
  return state;
}

// The addresses of the hosts attached to the switch host is attached to,
// sorted, host's own among them.
std::vector<uint32_t> own_leaf_hosts(Ptr<Node> host) {
  std::vector<uint32_t> addresses;
  for (const auto &uplink : nbr2if[host]) {
    for (const auto &link : nbr2if[uplink.first])
      if (link.first->GetNodeType() == 0)
        addresses.push_back(node_id_to_ip(link.first->GetId()).Get());
  }
  std::sort(addresses.begin(), addresses.end());
  return addresses;
}

// take down the link between a and b, and redo the routing
void TakeDownLink(NodeContainer n, Ptr<Node> a, Ptr<Node> b) {
  if (!nbr2if[a][b].up)
    return;
  // take down link between a and b
  nbr2if[a][b].up = nbr2if[b][a].up = false;
  nextHop.clear();
  CalculateRoutes(n);
  if (pair_bandwidth == "spines")
    SprayPairBandwidth(n);
  // clear routing tables
  for (uint32_t i = 0; i < n.GetN(); i++) {
    if (n.Get(i)->GetNodeType() == 1)
      DynamicCast<SwitchNode>(n.Get(i))->ClearTable();
    else
      n.Get(i)->GetObject<RdmaDriver>()->m_rdma->ClearTable();
  }
  DynamicCast<QbbNetDevice>(a->GetDevice(nbr2if[a][b].idx))->TakeDown();
  DynamicCast<QbbNetDevice>(b->GetDevice(nbr2if[b][a].idx))->TakeDown();
  // reset routing table
  SetRoutingEntries();

  // redistribute qp on each host
  for (uint32_t i = 0; i < n.GetN(); i++) {
    if (n.Get(i)->GetNodeType() == 0)
      n.Get(i)->GetObject<RdmaDriver>()->m_rdma->RedistributeQp();
  }
}

// The node at the far end of a device's link.
uint32_t neighbour_of(Ptr<NetDevice> dev) {
  Ptr<Channel> channel = dev->GetChannel();
  Ptr<NetDevice> peer = channel->GetDevice(0) == dev ? channel->GetDevice(1)
                                                     : channel->GetDevice(0);
  return peer->GetNode()->GetId();
}

// The counters as they stand now, one row per switch port and one per host
// and spine, under the current time.
void write_fabric_counters() {
  const unsigned long now = Simulator::Now().GetNanoSeconds();
  for (uint32_t i = 0; i < n.GetN(); i++) {
    Ptr<Node> node = n.Get(i);
    if (node->GetNodeType() == 1 && port_counter_file != nullptr) {
      Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(node);
      for (uint32_t port = 1; port < sw->GetNDevices(); port++) {
        const SwitchPortCounters &sent = sw->GetPortCounters(port);
        fprintf(port_counter_file, "%lu,%u,%u,%u,%lu,%lu,%lu,%lu\n", now, i,
                port, neighbour_of(sw->GetDevice(port)),
                static_cast<unsigned long>(sent.txPackets),
                static_cast<unsigned long>(sent.txBytes),
                static_cast<unsigned long>(sent.dataPackets),
                static_cast<unsigned long>(sent.ecnMarks));
      }
    } else if (node->GetNodeType() == 0 && spine_arrival_file != nullptr) {
      Ptr<RdmaHw> hw = node->GetObject<RdmaDriver>()->m_rdma;
      for (uint32_t spine = 0; spine < hw->m_spineCount; spine++) {
        const SpineArrivals arrived = spine < hw->m_spineArrivals.size()
                                          ? hw->m_spineArrivals[spine]
                                          : SpineArrivals{};
        fprintf(spine_arrival_file, "%lu,%u,%u,%lu,%lu,%lu\n", now, i, spine,
                static_cast<unsigned long>(arrived.packets),
                static_cast<unsigned long>(arrived.payloadBytes),
                static_cast<unsigned long>(arrived.folded));
      }
    }
  }
  for (FILE *file : {port_counter_file, spine_arrival_file, spine_report_file,
                     spine_loss_file, rto_event_file, receiver_hold_file,
                     answer_hold_file})
    if (file != nullptr)
      fflush(file);
}

// A report graded from the switches' state for one source leaf: one row per
// spine, with the counts of the grader's interval, the values read in place
// of its averages, and the source leaf and loss fraction.
void write_switch_queue_report(
    uint32_t rank, uint64_t now_ns, const SpineGrader &grader,
    uint32_t source_leaf, const std::vector<SpineGrader::SpineValues> &values,
    const SpineReport &report) {
  const std::vector<SpineGrader::SpineInterval> &spines = grader.LastInterval();
  for (uint32_t spine = 0; spine < spines.size(); spine++) {
    const SpineGrader::SpineInterval &closed = spines[spine];
    fprintf(spine_report_file,
            "%lu,%u,%u,%u,%u,%u,%u,%u,%.6f,%.6f,%.1f,%d,%u,%u,%d,%u,%.6f\n",
            static_cast<unsigned long>(now_ns), rank, report.sequence, spine,
            closed.arrivals, closed.marked, closed.trimmed, closed.moved,
            values[spine].markFraction, values[spine].trimFraction,
            values[spine].delayNs, closed.held ? 1 : 0, report.Grade(spine),
            grader.LastIntervalLastHopTrims(), report.edgeCongested ? 1 : 0,
            source_leaf, values[spine].lossFraction);
  }
}

// A spine report as a receiver issued it: one row per spine, with the
// interval it closed. A connection's report also names the connection by its
// sender's rank and port.
void write_spine_report(uint32_t rank, uint64_t now_ns,
                        const SpineGrader &grader,
                        const RdmaRxQueuePair *connection) {
  const SpineReport &report = grader.Report();
  const std::vector<SpineGrader::SpineInterval> &spines = grader.LastInterval();
  for (uint32_t spine = 0; spine < spines.size(); spine++) {
    const SpineGrader::SpineInterval &closed = spines[spine];
    fprintf(spine_report_file,
            "%lu,%u,%u,%u,%u,%u,%u,%u,%.6f,%.6f,%.1f,%d,%u,%u,%d",
            static_cast<unsigned long>(now_ns), rank, report.sequence, spine,
            closed.arrivals, closed.marked, closed.trimmed, closed.moved,
            closed.markFraction, closed.trimFraction, closed.delayNs,
            closed.held ? 1 : 0,
            report.Grade(spine), grader.LastIntervalLastHopTrims(),
            report.edgeCongested ? 1 : 0);
    if (connection != nullptr)
      fprintf(spine_report_file, ",%u,%u",
              ip_to_node_id(Ipv4Address(connection->dip)), connection->dport);
    fputc('\n', spine_report_file);
  }
}

// A loss interval the host's pooled scores closed: one row per spine, under
// the time the interval ended at, with the sends and losses counted in it and
// the grade its lost fraction earns.
void write_spine_loss_interval(uint32_t rank, uint64_t end_ns,
                               const std::vector<uint32_t> &sent,
                               const std::vector<uint32_t> &lost,
                               const std::vector<uint8_t> &grades) {
  for (uint32_t spine = 0; spine < sent.size(); spine++)
    fprintf(spine_loss_file, "%lu,%u,%u,%u,%u,%u\n",
            static_cast<unsigned long>(end_ns), rank, spine, sent[spine],
            lost[spine], grades[spine]);
}

// A loss interval a destination leaf's loss hold closed under reps_hold, or
// under auto the host's toward that leaf, as write_spine_loss_interval writes
// it with the leaf.
void write_leaf_spine_loss_interval(uint32_t rank, uint32_t leaf,
                                    uint64_t end_ns,
                                    const std::vector<uint32_t> &sent,
                                    const std::vector<uint32_t> &lost,
                                    const std::vector<uint8_t> &grades) {
  for (uint32_t spine = 0; spine < sent.size(); spine++)
    fprintf(spine_loss_file, "%lu,%u,%u,%u,%u,%u,%u\n",
            static_cast<unsigned long>(end_ns), rank, leaf, spine, sent[spine],
            lost[spine], grades[spine]);
}

// A loss interval a destination host's loss hold closed under pair, as
// write_leaf_spine_loss_interval writes it with the destination's node id
// for the leaf.
void write_pair_spine_loss_interval(uint32_t rank, uint32_t destination,
                                    uint64_t end_ns,
                                    const std::vector<uint32_t> &sent,
                                    const std::vector<uint32_t> &lost,
                                    const std::vector<uint8_t> &grades) {
  write_leaf_spine_loss_interval(rank, ip_to_node_id(Ipv4Address(destination)),
                                 end_ns, sent, lost, grades);
}

// A retransmission timeout a host's queue pair took.
void write_rto_event(uint32_t rank, uint64_t now_ns, Ptr<RdmaQueuePair> qp,
                     int32_t spine, bool exploration,
                     uint64_t bytes_outstanding) {
  fprintf(rto_event_file, "%lu,%u,%u,", static_cast<unsigned long>(now_ns),
          rank, ip_to_node_id(qp->dip));
  if (spine >= 0)
    fprintf(rto_event_file, "%d", spine);
  fprintf(rto_event_file, ",%d,%lu\n", exploration ? 1 : 0,
          static_cast<unsigned long>(bytes_outstanding));
}

// An interval a receiving host's hold closed under SPRAY_RECEIVER_HOLD: one row
// per spine for the sender leaf, with the arrivals and missing packets counted
// in it and the scope of the spine's hold.
void write_receiver_hold_interval(uint32_t rank, uint32_t leaf,
                                  uint64_t end_ns,
                                  const std::vector<uint32_t> &arrivals,
                                  const std::vector<uint32_t> &lost,
                                  const std::vector<uint8_t> &holds) {
  static const char *const scopes[] = {"none", "leaf", "host"};
  for (uint32_t spine = 0; spine < arrivals.size(); spine++)
    fprintf(receiver_hold_file, "%lu,%u,%u,%u,%u,%u,%s\n",
            static_cast<unsigned long>(end_ns), rank, leaf, spine,
            arrivals[spine], lost[spine], scopes[holds[spine]]);
}

void write_answer_hold(uint32_t rank, bool receiverSide, uint32_t leaf,
                       uint32_t spine, bool held, uint64_t now_ns) {
  if (answer_hold_file == nullptr)
    return;
  fprintf(answer_hold_file, "%lu,%s,%u,%u,%u,%d\n",
          static_cast<unsigned long>(now_ns),
          receiverSide ? "receiver" : "sender", rank, leaf, spine, held ? 1 : 0);
}

// The link events' applier, which keeps what a restore puts back.
std::unique_ptr<LinkEventApplier> link_event_applier;

void ApplyLinkEvent(NodeContainer n, LinkEvent event) {
  Ptr<Node> a = n.Get(event.a), b = n.Get(event.b);
  LinkEnds ends;
  ends.atA = DynamicCast<QbbNetDevice>(a->GetDevice(nbr2if[a][b].idx));
  ends.atB = DynamicCast<QbbNetDevice>(b->GetDevice(nbr2if[b][a].idx));
  ends.switchAtB = DynamicCast<SwitchNode>(b);
  if (!link_event_applier)
    link_event_applier = std::make_unique<LinkEventApplier>(
        &set_link_error_rate, [n](uint32_t x, uint32_t y) {
          TakeDownLink(n, n.Get(x), n.Get(y));
        });
  link_event_applier->Apply(event, ends);
}

uint64_t get_nic_rate(NodeContainer &n) {
  for (uint32_t i = 0; i < n.GetN(); i++)
    if (n.Get(i)->GetNodeType() == 0)
      return DynamicCast<QbbNetDevice>(n.Get(i)->GetDevice(1))
          ->GetDataRate()
          .GetBitRate();
  // TODO: Complain if you cannot find the NIC rate.
  return 0;
}

bool increasing(const double (&thresholds)[3]) {
  return thresholds[0] > 0 && thresholds[0] < thresholds[1] &&
         thresholds[1] < thresholds[2];
}

// The spray_policy parameters within the ranges their RdmaHw attributes and
// the grades they set take.
bool valid_spray_policy() {
  if (!(spray_report_interval_base_rtts > 0)) {
    std::cerr << "SPRAY_REPORT_INTERVAL_BASE_RTTS must be positive\n";
    return false;
  }
  if (!(spray_estimator_gain > 0 && spray_estimator_gain <= 1)) {
    std::cerr << "SPRAY_ESTIMATOR_GAIN must be in (0, 1]\n";
    return false;
  }
  if (!(spray_fraction_cusum_slack >= 0 &&
        spray_fraction_cusum_threshold > 0 &&
        spray_delay_cusum_slack_base_rtts >= 0 &&
        spray_delay_cusum_threshold_base_rtts > 0)) {
    std::cerr << "a SPRAY_*_CUSUM_SLACK must be at least zero and a "
                 "SPRAY_*_CUSUM_THRESHOLD positive\n";
    return false;
  }
  if (!increasing(spray_congestion_thresholds) ||
      spray_congestion_thresholds[2] > 1) {
    std::cerr << "SPRAY_CONGESTION_THRESHOLDS must be three increasing "
                 "fractions in (0, 1]\n";
    return false;
  }
  if (spray_policy_keys_given.count("SPRAY_TRIM_THRESHOLDS") > 0 &&
      (!increasing(spray_trim_thresholds) || spray_trim_thresholds[2] > 1)) {
    std::cerr << "SPRAY_TRIM_THRESHOLDS must be three increasing "
                 "fractions in (0, 1]\n";
    return false;
  }
  if (spray_congestion_grade_floor > SpineReport::kTopGrade) {
    std::cerr << "SPRAY_CONGESTION_GRADE_FLOOR must be in [0, 3]\n";
    return false;
  }
  if (spray_policy_keys_given.count("SPRAY_LOSS_THRESHOLDS") > 0 &&
      (!increasing(spray_loss_thresholds) || spray_loss_thresholds[2] > 1)) {
    std::cerr << "SPRAY_LOSS_THRESHOLDS must be three increasing "
                 "fractions in (0, 1]\n";
    return false;
  }
  if (spray_policy_keys_given.count("SPRAY_LOSS_INTERVAL_NS") > 0 &&
      spray_policy_keys_given.count("SPRAY_LOSS_THRESHOLDS") == 0) {
    std::cerr << "SPRAY_LOSS_INTERVAL_NS is read only with "
                 "SPRAY_LOSS_THRESHOLDS\n";
    return false;
  }
  if (spray_policy_keys_given.count("SPRAY_LOSS_HOLD_CLEAN_SENTINELS") > 0 &&
      spray_policy_keys_given.count("SPRAY_LOSS_THRESHOLDS") == 0) {
    std::cerr << "SPRAY_LOSS_HOLD_CLEAN_SENTINELS is read only with "
                 "SPRAY_LOSS_THRESHOLDS\n";
    return false;
  }
  // The sender declares a send lost only from the send records selective
  // retransmission keeps; under switch_queue the thresholds grade the paths'
  // loss read from the switches instead.
  if (spray_policy_keys_given.count("SPRAY_LOSS_THRESHOLDS") > 0 &&
      spray_grade_cost_source != "switch_queue" &&
      selective_retransmission == 0) {
    std::cerr << "SPRAY_LOSS_THRESHOLDS requires SELECTIVE_RETRANSMISSION 1\n";
    return false;
  }
  if (!increasing(spray_delay_thresholds_base_rtts)) {
    std::cerr << "SPRAY_DELAY_THRESHOLDS_BASE_RTTS must be three increasing "
                 "positive numbers\n";
    return false;
  }
  if (spray_one_way_delay > 1) {
    std::cerr << "SPRAY_ONE_WAY_DELAY must be 0 or 1\n";
    return false;
  }
  // The delay is the timestamp IntHeader::mode TS carries, which HPCC's modes
  // would replace with their own.
  if (spray_one_way_delay == 1 && (cc_mode == 3 || cc_mode == 10)) {
    std::cerr << "SPRAY_ONE_WAY_DELAY 1 needs the timestamp CC_MODE "
              << cc_mode << " replaces\n";
    return false;
  }
  if (!(spray_absence_fraction_of_median >= 0 &&
        spray_absence_fraction_of_median < 1) ||
      spray_absence_minimum_median == 0 ||
      !(spray_release_fraction_of_floor >= 0)) {
    std::cerr << "SPRAY_ABSENCE_FRACTION_OF_MEDIAN must be in [0, 1), "
                 "SPRAY_ABSENCE_MINIMUM_MEDIAN positive and "
                 "SPRAY_RELEASE_FRACTION_OF_FLOOR at least zero\n";
    return false;
  }
  if (spray_grade_reference_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "SPRAY_GRADE_REFERENCE must be absolute or median\n";
    return false;
  }
  if (grade_scope_value(spray_grade_scope) == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "SPRAY_GRADE_SCOPE must be per_receiver or per_connection\n";
    return false;
  }
  if (spray_grade_cost_source_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "SPRAY_GRADE_COST_SOURCE must be estimate or switch_queue\n";
    return false;
  }
  // The switches' state is read per path into one receiving host, so it
  // stands in for the host's grader only.
  if (spray_grade_cost_source == "switch_queue" &&
      spray_grade_scope != "per_receiver") {
    std::cerr << "SPRAY_GRADE_COST_SOURCE switch_queue requires "
                 "SPRAY_GRADE_SCOPE per_receiver\n";
    return false;
  }
  if (!(spray_gamma > 0 && spray_gamma <= 1)) {
    std::cerr << "SPRAY_GAMMA must be in (0, 1]\n";
    return false;
  }
  if (!(spray_epsilon >= 0 && spray_epsilon <= 1)) {
    std::cerr << "SPRAY_EPSILON must be in [0, 1]\n";
    return false;
  }
  if (spray_candidates == 0) {
    std::cerr << "SPRAY_CANDIDATES must be positive\n";
    return false;
  }
  if (spray_candidate_draw_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "SPRAY_CANDIDATE_DRAW must be proportional or uniform\n";
    return false;
  }
  if (spray_edge_window_penalty > 127) {
    std::cerr << "SPRAY_EDGE_WINDOW_PENALTY must be in [0, 127]\n";
    return false;
  }
  if (spray_edge_freeze > 1) {
    std::cerr << "SPRAY_EDGE_FREEZE must be 0 or 1\n";
    return false;
  }
  if (!(spray_sender_pool_weight >= 0 && spray_sender_pool_weight <= 1)) {
    std::cerr << "SPRAY_SENDER_POOL_WEIGHT must be in [0, 1]\n";
    return false;
  }
  if (spray_policy_keys_given.count("SPRAY_POOL_DECAY_INTERVAL_NS") > 0 &&
      spray_sender_pool_weight == 0) {
    std::cerr << "SPRAY_POOL_DECAY_INTERVAL_NS is read only with "
                 "SPRAY_SENDER_POOL_WEIGHT above zero\n";
    return false;
  }
  if (spray_pool_hold_propagates > 1) {
    std::cerr << "SPRAY_POOL_HOLD_PROPAGATES must be 0 or 1\n";
    return false;
  }
  if (spray_policy_keys_given.count("SPRAY_POOL_HOLD_PROPAGATES") > 0 &&
      spray_sender_pool_weight == 0) {
    std::cerr << "SPRAY_POOL_HOLD_PROPAGATES is read only with "
                 "SPRAY_SENDER_POOL_WEIGHT above zero\n";
    return false;
  }
  if (spray_held_exploration_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "SPRAY_HELD_EXPLORATION must be epsilon, sentinel or none\n";
    return false;
  }
  // Under none a held spine is sent nothing, and the receiver's estimate
  // releases a hold only for arrivals.
  if (spray_held_exploration == "none" &&
      spray_grade_cost_source != "switch_queue") {
    std::cerr << "SPRAY_HELD_EXPLORATION none requires "
                 "SPRAY_GRADE_COST_SOURCE switch_queue\n";
    return false;
  }
  if (spray_policy_keys_given.count("SPRAY_HELD_SENTINEL_INTERVAL_NS") > 0 &&
      spray_held_exploration != "sentinel") {
    std::cerr << "SPRAY_HELD_SENTINEL_INTERVAL_NS is read only with "
                 "SPRAY_HELD_EXPLORATION sentinel\n";
    return false;
  }
  // Only the host's pool counts over loss intervals, and only the sender's
  // own counting, outside switch_queue, feeds it.
  if (!spine_loss_output_file.empty() &&
      (spray_policy_keys_given.count("SPRAY_LOSS_THRESHOLDS") == 0 ||
       spray_sender_pool_weight == 0 ||
       spray_grade_cost_source == "switch_queue")) {
    std::cerr << "SPINE_LOSS_OUTPUT_FILE requires SPRAY_LOSS_THRESHOLDS and "
                 "SPRAY_SENDER_POOL_WEIGHT above zero, outside "
                 "SPRAY_GRADE_COST_SOURCE switch_queue\n";
    return false;
  }
  return true;
}

// The ack_redirect parameters within the ranges their RdmaHw attributes take.
bool valid_ack_redirect() {
  if (!(ack_redirect_omega >= 0 && ack_redirect_omega <= 1)) {
    std::cerr << "ACK_REDIRECT_OMEGA must be in [0, 1]\n";
    return false;
  }
  if (!(ack_redirect_flag >= 0)) {
    std::cerr << "ACK_REDIRECT_FLAG must be at least zero\n";
    return false;
  }
  if (!(ack_redirect_validity_base_rtts > 0)) {
    std::cerr << "ACK_REDIRECT_VALIDITY_BASE_RTTS must be positive\n";
    return false;
  }
  if (ack_redirect_trim_scoring_value() ==
      std::numeric_limits<uint32_t>::max()) {
    std::cerr << "ACK_REDIRECT_TRIM_SCORING must be mark or ignore\n";
    return false;
  }
  if (grade_scope_value(ack_redirect_scope) ==
      std::numeric_limits<uint32_t>::max()) {
    std::cerr << "ACK_REDIRECT_SCOPE must be per_connection or per_receiver\n";
    return false;
  }
  if (host_redirect() && !valid_spray_policy())
    return false;
  // The score reads the send timestamp IntHeader::mode TS carries, which
  // HPCC's modes would replace with their own.
  if (cc_mode == 3 || cc_mode == 10) {
    std::cerr << "PATH_SELECTOR ack_redirect needs the timestamp CC_MODE "
              << cc_mode << " replaces\n";
    return false;
  }
  return true;
}

// The loss hold of PATH_SELECTOR reps_hold and mrc_hold: every key it
// requires given, within the ranges its RdmaHw attributes take, and the
// sentinels that end a hold sent.
bool valid_loss_hold() {
  // Under trickle the trickle sends' answers release a hold, and the
  // sentinels' keys are read by nothing.
  const bool trickle = spray_hold_release == "trickle";
  if (trickle) {
    for (const char *key : {"SPRAY_LOSS_HOLD_CLEAN_SENTINELS",
                            "SPRAY_HELD_SENTINEL_INTERVAL_NS"}) {
      if (spray_policy_keys_given.count(key) > 0) {
        std::cerr << key << " is read only under SPRAY_HOLD_RELEASE "
                            "sentinels\n";
        return false;
      }
    }
  }
  for (const std::string &key : loss_hold_required_keys) {
    if (trickle && key == "SPRAY_LOSS_HOLD_CLEAN_SENTINELS")
      continue;
    if (spray_policy_keys_given.count(key) == 0) {
      std::cerr << "PATH_SELECTOR " << path_selector
                << (trickle
                        ? " requires SPRAY_LOSS_THRESHOLDS, "
                          "SPRAY_LOSS_INTERVAL_NS and SPRAY_HELD_EXPLORATION "
                          "under SPRAY_HOLD_RELEASE trickle\n"
                        : " requires SPRAY_LOSS_THRESHOLDS, "
                          "SPRAY_LOSS_INTERVAL_NS, "
                          "SPRAY_LOSS_HOLD_CLEAN_SENTINELS and "
                          "SPRAY_HELD_EXPLORATION\n");
      return false;
    }
  }
  if (!increasing(spray_loss_thresholds) || spray_loss_thresholds[2] > 1) {
    std::cerr << "SPRAY_LOSS_THRESHOLDS must be three increasing "
                 "fractions in (0, 1]\n";
    return false;
  }
  if (spray_loss_interval_ns == 0) {
    std::cerr << "SPRAY_LOSS_INTERVAL_NS must be positive under PATH_SELECTOR "
              << path_selector << "\n";
    return false;
  }
  if (!trickle && spray_loss_hold_clean_sentinels == 0) {
    std::cerr << "SPRAY_LOSS_HOLD_CLEAN_SENTINELS must be positive under "
                 "PATH_SELECTOR "
              << path_selector << "\n";
    return false;
  }
  // A held spine is left out of every choice, so only its sentinels' clean
  // sends, or under trickle its trickle sends' answers, can end the hold.
  if (spray_held_exploration != "sentinel") {
    std::cerr << "PATH_SELECTOR " << path_selector
              << " requires SPRAY_HELD_EXPLORATION sentinel\n";
    return false;
  }
  return true;
}

// The hold rule of reps_hold, reps_flow_hold and mrc_hold, and the keys of its test
// read only where it reads them, within the ranges its RdmaHw attributes
// take.
bool valid_hold_rule() {
  if (spray_hold_rule_given && !reps_hold()) {
    std::cerr << "SPRAY_HOLD_RULE is read only under PATH_SELECTOR reps_hold, "
                 "reps_flow_hold or mrc_hold\n";
    return false;
  }
  if (spray_hold_rule_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "SPRAY_HOLD_RULE must be count or llr\n";
    return false;
  }
  if (spray_hold_release_given && !reps_hold()) {
    std::cerr << "SPRAY_HOLD_RELEASE is read only under PATH_SELECTOR "
                 "reps_hold, reps_flow_hold or mrc_hold\n";
    return false;
  }
  if (spray_sentinel_mode_given && !reps_hold()) {
    std::cerr << "SPRAY_SENTINEL_MODE is read only under PATH_SELECTOR "
                 "reps_hold, reps_flow_hold or mrc_hold\n";
    return false;
  }
  if (spray_sentinel_mode_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "SPRAY_SENTINEL_MODE must be data or duplicate\n";
    return false;
  }
  if (spray_copy_expiry != "data_ack" && spray_copy_expiry != "deadline") {
    std::cerr << "SPRAY_COPY_EXPIRY must be data_ack or deadline\n";
    return false;
  }
  if (spray_copy_expiry_given && spray_sentinel_mode != "duplicate") {
    std::cerr << "SPRAY_COPY_EXPIRY is read only under SPRAY_SENTINEL_MODE "
                 "duplicate\n";
    return false;
  }
  if (spray_copy_expiry == "deadline" && !(spray_tail_loss_rtts > 0)) {
    std::cerr << "SPRAY_COPY_EXPIRY deadline needs SPRAY_TAIL_LOSS_RTTS above "
                 "0\n";
    return false;
  }
  if (spray_held_loss_rtts_given && !reps_hold()) {
    std::cerr << "SPRAY_HELD_LOSS_RTTS is read only under PATH_SELECTOR "
                 "reps_hold, reps_flow_hold or mrc_hold\n";
    return false;
  }
  if (!(spray_held_loss_rtts >= 0)) {
    std::cerr << "SPRAY_HELD_LOSS_RTTS must be at least 0\n";
    return false;
  }
  if (spray_hold_release_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "SPRAY_HOLD_RELEASE must be sentinels or trickle\n";
    return false;
  }
  const bool llr = reps_hold() && spray_hold_rule == "llr";
  const bool trickle = reps_hold() && spray_hold_release == "trickle";
  if (spray_hold_trickle_given && !trickle) {
    std::cerr << "SPRAY_HOLD_TRICKLE is read only under SPRAY_HOLD_RELEASE "
                 "trickle\n";
    return false;
  }
  if (trickle && !(spray_hold_trickle > 0 && spray_hold_trickle <= 0.5)) {
    std::cerr << "SPRAY_HOLD_TRICKLE must be in (0, 0.5]\n";
    return false;
  }
  for (const std::string &key : hold_test_keys_given) {
    if (key == "SPRAY_HOLD_BETA" && !llr) {
      std::cerr << key << " is read only under SPRAY_HOLD_RULE llr\n";
      return false;
    }
    if (!llr && !trickle) {
      std::cerr << key << " is read only under SPRAY_HOLD_RULE llr or "
                          "SPRAY_HOLD_RELEASE trickle\n";
      return false;
    }
  }
  if (!llr && !trickle)
    return true;
  if (!(spray_loss_p1 < 1)) {
    std::cerr << "SPRAY_LOSS_P1 must be below 1\n";
    return false;
  }
  if (!(spray_loss_p0 >= 0 && spray_loss_p0 < spray_loss_p1)) {
    std::cerr << "SPRAY_LOSS_P0 must be at least 0 and below SPRAY_LOSS_P1\n";
    return false;
  }
  if (!(spray_hold_alpha > 0 && spray_hold_alpha < 0.5)) {
    std::cerr << "SPRAY_HOLD_ALPHA must be in (0, 0.5)\n";
    return false;
  }
  if (llr && !(spray_hold_beta > 0 && spray_hold_beta < 0.5)) {
    std::cerr << "SPRAY_HOLD_BETA must be in (0, 0.5)\n";
    return false;
  }
  return true;
}

bool ReadConf(string network_configuration) {
  // Read the configuration file
  std::ifstream conf;
  conf.open(network_configuration);
  if (!conf.is_open()) {
    std::cout << "Error: cannot find network config file: " << network_configuration << std::endl;
    fflush(stdout);
    return false;
  }
  
  while (!conf.eof()) {
    std::string key;
    conf >> key;
    if (path_selector_parameter_owners.count(key) > 0)
      path_selector_parameters_given.insert(key);
    if (spray_policy_keys.count(key) > 0)
      spray_policy_keys_given.insert(key);

    if (key.compare("ENABLE_QCN") == 0) {
      uint32_t v;
      conf >> v;
      enable_qcn = v;
    } else if (key.compare("USE_DYNAMIC_PFC_THRESHOLD") == 0) {
      uint32_t v;
      conf >> v;
      use_dynamic_pfc_threshold = v;
    } else if (key.compare("CLAMP_TARGET_RATE") == 0) {
      uint32_t v;
      conf >> v;
      clamp_target_rate = v;
    } else if (key.compare("PAUSE_TIME") == 0) {
      double v;
      conf >> v;
      pause_time = v;
    } else if (key.compare("PACKET_PAYLOAD_SIZE") == 0) {
      uint32_t v;
      conf >> v;
      packet_payload_size = v;
    } else if (key.compare("L2_CHUNK_SIZE") == 0) {
      uint32_t v;
      conf >> v;
      l2_chunk_size = v;
    } else if (key.compare("L2_ACK_INTERVAL") == 0) {
      uint32_t v;
      conf >> v;
      l2_ack_interval = v;
    } else if (key.compare("L2_BACK_TO_ZERO") == 0) {
      uint32_t v;
      conf >> v;
      l2_back_to_zero = v;
    } else if (key.compare("TOPOLOGY_FILE") == 0) {
      std::string v;
      conf >> v;
      topology_file = v;
    } else if (key.compare("FLOW_FILE") == 0) {
      std::string v;
      conf >> v;
      flow_file = v;
    } else if (key.compare("TRACE_FILE") == 0) {
      std::string v;
      conf >> v;
      trace_file = v;
    } else if (key.compare("TRACE_OUTPUT_FILE") == 0) {
      std::string v;
      conf >> v;
      trace_output_file = v;
      // Removed to handle new command line arguments in build.sh.
  //if (argc > 2) {
      //  trace_output_file = trace_output_file + std::string(argv[2]);
      //}
    } else if (key.compare("SIMULATOR_STOP_TIME") == 0) {
      double v;
      conf >> v;
      simulator_stop_time = v;
    } else if (key.compare("ALPHA_RESUME_INTERVAL") == 0) {
      double v;
      conf >> v;
      alpha_resume_interval = v;
    } else if (key.compare("RP_TIMER") == 0) {
      double v;
      conf >> v;
      rp_timer = v;
    } else if (key.compare("EWMA_GAIN") == 0) {
      double v;
      conf >> v;
      ewma_gain = v;
    } else if (key.compare("FAST_RECOVERY_TIMES") == 0) {
      uint32_t v;
      conf >> v;
      fast_recovery_times = v;
    } else if (key.compare("RATE_AI") == 0) {
      std::string v;
      conf >> v;
      rate_ai = v;
    } else if (key.compare("RATE_HAI") == 0) {
      std::string v;
      conf >> v;
      rate_hai = v;
    } else if (key.compare("ERROR_RATE_PER_LINK") == 0) {
      double v;
      conf >> v;
      error_rate_per_link = v;
    } else if (key.compare("DATA_LOSS_PROBABILITY") == 0) {
      conf >> data_loss_probability;
    } else if (key.compare("DATA_LOSS_START_NS") == 0) {
      conf >> data_loss_start_ns;
    } else if (key.compare("DATA_LOSS_DURATION_NS") == 0) {
      conf >> data_loss_duration_ns;
    } else if (key.compare("DATA_LOSS_SCOPE") == 0) {
      conf >> data_loss_scope;
    } else if (key.compare("DATA_LOSS_SOURCE_HOST") == 0) {
      conf >> data_loss_source_host;
    } else if (key.compare("DATA_LOSS_DESTINATION_HOST") == 0) {
      conf >> data_loss_destination_host;
    } else if (key.compare("DATA_LOSS_RECEIVER_NODE") == 0) {
      conf >> data_loss_receiver_node;
    } else if (key.compare("DATA_LOSS_RNG_STREAM") == 0) {
      conf >> data_loss_rng_stream;
    } else if (key.compare("RETRANSMISSION_TIMEOUT_NS") == 0) {
      conf >> retransmission_timeout_ns;
    } else if (key.compare("MAX_RETRANSMISSION_RETRIES") == 0) {
      conf >> max_retransmission_retries;
    } else if (key.compare("NO_PROGRESS_TIMEOUT_NS") == 0) {
      conf >> no_progress_timeout_ns;
    } else if (key.compare("SELECTIVE_RETRANSMISSION") == 0) {
      conf >> selective_retransmission;
    } else if (key.compare("LOAD_BALANCING") == 0) {
      conf >> load_balancing;
    } else if (key.compare("PATH_SELECTOR") == 0) {
      conf >> path_selector;
    } else if (key.compare("REPS_BUFFER_SIZE") == 0) {
      conf >> reps_buffer_size;
    } else if (key.compare("REPS_FREEZING_TIMEOUT_NS") == 0) {
      conf >> reps_freezing_timeout_ns;
    } else if (key.compare("REPS_ORACLE_HOPS") == 0) {
      conf >> reps_oracle_hops;
    } else if (key.compare("UE_EV_SET_SIZE") == 0) {
      conf >> ue_ev_set_size;
    } else if (key.compare("UE_SATURATION_FRACTION") == 0) {
      conf >> ue_saturation_fraction;
    } else if (key.compare("MRC_EV_SET_SIZE") == 0) {
      conf >> mrc_ev_set_size;
    } else if (key.compare("MRC_SKIP_BASE_RTTS") == 0) {
      conf >> mrc_skip_base_rtts;
    } else if (key.compare("MRC_PROBE_TIMEOUTS") == 0) {
      conf >> mrc_probe_timeouts;
    } else if (key.compare("ACK_REDIRECT_OMEGA") == 0) {
      conf >> ack_redirect_omega;
    } else if (key.compare("ACK_REDIRECT_FLAG") == 0) {
      conf >> ack_redirect_flag;
    } else if (key.compare("ACK_REDIRECT_VALIDITY_BASE_RTTS") == 0) {
      conf >> ack_redirect_validity_base_rtts;
    } else if (key.compare("ACK_REDIRECT_TRIM_SCORING") == 0) {
      conf >> ack_redirect_trim_scoring;
    } else if (key.compare("ACK_REDIRECT_SCOPE") == 0) {
      conf >> ack_redirect_scope;
    } else if (key.compare("SPRAY_REPORT_INTERVAL_BASE_RTTS") == 0) {
      conf >> spray_report_interval_base_rtts;
    } else if (key.compare("SPRAY_ESTIMATOR_GAIN") == 0) {
      conf >> spray_estimator_gain;
    } else if (key.compare("SPRAY_FRACTION_CUSUM_SLACK") == 0) {
      conf >> spray_fraction_cusum_slack;
    } else if (key.compare("SPRAY_FRACTION_CUSUM_THRESHOLD") == 0) {
      conf >> spray_fraction_cusum_threshold;
    } else if (key.compare("SPRAY_CONGESTION_THRESHOLDS") == 0) {
      conf >> spray_congestion_thresholds[0] >>
          spray_congestion_thresholds[1] >> spray_congestion_thresholds[2];
    } else if (key.compare("SPRAY_TRIM_THRESHOLDS") == 0) {
      conf >> spray_trim_thresholds[0] >> spray_trim_thresholds[1] >>
          spray_trim_thresholds[2];
    } else if (key.compare("SPRAY_CONGESTION_GRADE_FLOOR") == 0) {
      conf >> spray_congestion_grade_floor;
    } else if (key.compare("SPRAY_LOSS_THRESHOLDS") == 0) {
      conf >> spray_loss_thresholds[0] >> spray_loss_thresholds[1] >>
          spray_loss_thresholds[2];
    } else if (key.compare("SPRAY_LOSS_INTERVAL_NS") == 0) {
      conf >> spray_loss_interval_ns;
    } else if (key.compare("SPRAY_LOSS_HOLD_CLEAN_SENTINELS") == 0) {
      conf >> spray_loss_hold_clean_sentinels;
    } else if (key.compare("SPRAY_RELEASE_FRACTION_OF_FLOOR") == 0) {
      conf >> spray_release_fraction_of_floor;
    } else if (key.compare("SPRAY_ONE_WAY_DELAY") == 0) {
      conf >> spray_one_way_delay;
    } else if (key.compare("SPRAY_DELAY_CUSUM_SLACK_BASE_RTTS") == 0) {
      conf >> spray_delay_cusum_slack_base_rtts;
    } else if (key.compare("SPRAY_DELAY_CUSUM_THRESHOLD_BASE_RTTS") == 0) {
      conf >> spray_delay_cusum_threshold_base_rtts;
    } else if (key.compare("SPRAY_DELAY_THRESHOLDS_BASE_RTTS") == 0) {
      conf >> spray_delay_thresholds_base_rtts[0] >>
          spray_delay_thresholds_base_rtts[1] >>
          spray_delay_thresholds_base_rtts[2];
    } else if (key.compare("SPRAY_ABSENCE_FRACTION_OF_MEDIAN") == 0) {
      conf >> spray_absence_fraction_of_median;
    } else if (key.compare("SPRAY_ABSENCE_MINIMUM_MEDIAN") == 0) {
      conf >> spray_absence_minimum_median;
    } else if (key.compare("SPRAY_GRADE_REFERENCE") == 0) {
      conf >> spray_grade_reference;
    } else if (key.compare("SPRAY_GRADE_SCOPE") == 0) {
      conf >> spray_grade_scope;
    } else if (key.compare("SPRAY_GRADE_COST_SOURCE") == 0) {
      conf >> spray_grade_cost_source;
    } else if (key.compare("SPRAY_ESTIMATOR_INTERVAL_SAMPLES") == 0) {
      conf >> spray_estimator_interval_samples;
    } else if (key.compare("SPRAY_GAMMA") == 0) {
      conf >> spray_gamma;
    } else if (key.compare("SPRAY_EPSILON") == 0) {
      conf >> spray_epsilon;
    } else if (key.compare("SPRAY_CANDIDATES") == 0) {
      conf >> spray_candidates;
    } else if (key.compare("SPRAY_CANDIDATE_DRAW") == 0) {
      conf >> spray_candidate_draw;
    } else if (key.compare("SPRAY_EDGE_WINDOW_PENALTY") == 0) {
      conf >> spray_edge_window_penalty;
    } else if (key.compare("SPRAY_EDGE_FREEZE") == 0) {
      conf >> spray_edge_freeze;
    } else if (key.compare("SPRAY_SENDER_POOL_WEIGHT") == 0) {
      conf >> spray_sender_pool_weight;
    } else if (key.compare("SPRAY_POOL_DECAY_INTERVAL_NS") == 0) {
      conf >> spray_pool_decay_interval_ns;
    } else if (key.compare("SPRAY_POOL_HOLD_PROPAGATES") == 0) {
      conf >> spray_pool_hold_propagates;
    } else if (key.compare("SPRAY_HOLD_SCOPE") == 0) {
      conf >> spray_hold_scope;
      spray_hold_scope_given = true;
    } else if (key.compare("SPRAY_HOLD_AUTO_LEAVES") == 0) {
      conf >> spray_hold_auto_leaves;
      spray_hold_auto_leaves_given = true;
    } else if (key.compare("SPRAY_HOLD_RULE") == 0) {
      conf >> spray_hold_rule;
      spray_hold_rule_given = true;
    } else if (key.compare("SPRAY_COPY_EXPIRY") == 0) {
      conf >> spray_copy_expiry;
      spray_copy_expiry_given = true;
    } else if (key.compare("SPRAY_SENTINEL_MODE") == 0) {
      conf >> spray_sentinel_mode;
      spray_sentinel_mode_given = true;
    } else if (key.compare("SPRAY_HELD_LOSS_RTTS") == 0) {
      conf >> spray_held_loss_rtts;
      spray_held_loss_rtts_given = true;
    } else if (key.compare("SPRAY_ACK_LOSS_RTTS") == 0) {
      conf >> spray_ack_loss_rtts;
      spray_ack_loss_rtts_given = true;
    } else if (key.compare("ACK_ROUTE") == 0) {
      conf >> ack_route;
      ack_route_given = true;
    } else if (key.compare("ACK_ROUTE_SET_SIZE") == 0) {
      conf >> ack_route_set_size;
      ack_route_set_size_given = true;
    } else if (key.compare("ACK_ROUTE_ORACLE") == 0) {
      conf >> ack_route_oracle;
      ack_route_oracle_given = true;
    } else if (key.compare("SPRAY_HOLD_RELEASE") == 0) {
      conf >> spray_hold_release;
      spray_hold_release_given = true;
    } else if (key.compare("SPRAY_HOLD_TRICKLE") == 0) {
      conf >> spray_hold_trickle;
      spray_hold_trickle_given = true;
    } else if (key.compare("SPRAY_LOSS_P0") == 0) {
      conf >> spray_loss_p0;
      hold_test_keys_given.insert(key);
    } else if (key.compare("SPRAY_LOSS_P1") == 0) {
      conf >> spray_loss_p1;
      hold_test_keys_given.insert(key);
    } else if (key.compare("SPRAY_HOLD_ALPHA") == 0) {
      conf >> spray_hold_alpha;
      hold_test_keys_given.insert(key);
    } else if (key.compare("SPRAY_HOLD_BETA") == 0) {
      conf >> spray_hold_beta;
      hold_test_keys_given.insert(key);
    } else if (key.compare("SPRAY_PATH_STATE") == 0) {
      conf >> spray_path_state;
      spray_path_state_given = true;
    } else if (key.compare("SPRAY_PATH_SEQ") == 0) {
      conf >> spray_path_seq;
      spray_path_seq_given = true;
    } else if (key.compare("SPRAY_TAIL_RECOVERY") == 0) {
      conf >> spray_tail_recovery;
      spray_tail_recovery_given = true;
    } else if (key.compare("SPRAY_TAIL_RECOVERY_DELAY_NS") == 0) {
      conf >> spray_tail_recovery_delay_ns;
      spray_tail_recovery_delay_ns_given = true;
    } else if (key.compare("SPRAY_TAIL_LOSS_RTTS") == 0) {
      conf >> spray_tail_loss_rtts;
      spray_tail_loss_rtts_given = true;
    } else if (key.compare("SPRAY_RECEIVER_HOLD") == 0) {
      conf >> spray_receiver_hold;
      spray_receiver_hold_given = true;
    } else if (key.compare("SPRAY_RECEIVER_HOLD_CLEAN") == 0) {
      conf >> spray_receiver_hold_clean;
      spray_receiver_hold_clean_given = true;
    } else if (key.compare("SPRAY_RECEIVER_TAIL_MISSING") == 0) {
      conf >> spray_receiver_tail_missing;
      spray_receiver_tail_missing_given = true;
    } else if (key.compare("SPRAY_STRAY_PATH_SEQ") == 0) {
      conf >> spray_stray_path_seq;
      spray_stray_path_seq_given = true;
    } else if (key.compare("SPRAY_COVERAGE_FIRST") == 0) {
      conf >> spray_coverage_first;
      spray_coverage_first_given = true;
    } else if (key.compare("SPRAY_DUPLICATE_REPORT") == 0) {
      conf >> spray_duplicate_report;
      spray_duplicate_report_given = true;
    } else if (key.compare("SPRAY_ANSWER_PATH_SEQ") == 0) {
      conf >> spray_answer_path_seq;
      spray_answer_path_seq_given = true;
    } else if (key.compare("SPRAY_ANSWER_PROBE_INTERVAL_NS") == 0) {
      conf >> spray_answer_probe_interval_ns;
      spray_answer_probe_interval_ns_given = true;
    } else if (key.compare("SPRAY_ANSWER_HOLD_CLEAN") == 0) {
      conf >> spray_answer_hold_clean;
      spray_answer_hold_clean_given = true;
    } else if (key.compare("ANSWER_HOLD_OUTPUT_FILE") == 0) {
      conf >> answer_hold_output_file;
    } else if (key.compare("SPRAY_HELD_EXPLORATION") == 0) {
      conf >> spray_held_exploration;
    } else if (key.compare("SPRAY_HELD_SENTINEL_INTERVAL_NS") == 0) {
      conf >> spray_held_sentinel_interval_ns;
	} else if (key.compare("PACKET_TRIM_MODE") == 0) {
	  conf >> packet_trim_mode;
	} else if (key.compare("PACKET_TRIM_QUEUE") == 0) {
	  conf >> packet_trim_queue;
	} else if (key.compare("MIN_TRIM_SIZE") == 0) {
	  conf >> min_trim_size;
	} else if (key.compare("PACKET_TRIM_LASTHOP") == 0) {
	  conf >> packet_trim_lasthop;
	} else if (key.compare("PACKET_TRIM_QUEUE_WEIGHT") == 0) {
	  conf >> packet_trim_queue_weight;
	} else if (key.compare("ENABLE_PFC") == 0) {
	  conf >> enable_pfc;
	} else if (key.compare("DATA_QUEUE_BYTES") == 0) {
	  conf >> data_queue_bytes;
	} else if (key.compare("TRIMMED_QUEUE_BYTES") == 0) {
	  conf >> trimmed_queue_bytes;
    } else if (key.compare("CC_MODE") == 0) {
      conf >> cc_mode;
    } else if (key.compare("NSCC_RTT_SAMPLE") == 0) {
      conf >> nscc_rtt_sample;
      nscc_rtt_sample_given = true;
    } else if (key.compare("PAIR_BANDWIDTH") == 0) {
      conf >> pair_bandwidth;
    } else if (key.compare("NSCC_WINDOW_STATE") == 0) {
      conf >> nscc_window_state;
      nscc_window_state_given = true;
    } else if (key.compare("RATE_DECREASE_INTERVAL") == 0) {
      double v;
      conf >> v;
      rate_decrease_interval = v;
    } else if (key.compare("MIN_RATE") == 0) {
      conf >> min_rate;
    } else if (key.compare("FCT_OUTPUT_FILE") == 0) {
      conf >> fct_output_file;
    } else if (key.compare("HAS_WIN") == 0) {
      conf >> has_win;
    } else if (key.compare("GLOBAL_T") == 0) {
      conf >> global_t;
    } else if (key.compare("MI_THRESH") == 0) {
      conf >> mi_thresh;
    } else if (key.compare("VAR_WIN") == 0) {
      uint32_t v;
      conf >> v;
      var_win = v;
    } else if (key.compare("FAST_REACT") == 0) {
      uint32_t v;
      conf >> v;
      fast_react = v;
    } else if (key.compare("U_TARGET") == 0) {
      conf >> u_target;
    } else if (key.compare("INT_MULTI") == 0) {
      conf >> int_multi;
    } else if (key.compare("RATE_BOUND") == 0) {
      uint32_t v;
      conf >> v;
      rate_bound = v;
    } else if (key.compare("ACK_HIGH_PRIO") == 0) {
      conf >> ack_high_prio;
    } else if (key.compare("DCTCP_RATE_AI") == 0) {
      conf >> dctcp_rate_ai;
    } else if (key.compare("NIC_TOTAL_PAUSE_TIME") == 0) {
      conf >> nic_total_pause_time;
    } else if (key.compare("PFC_OUTPUT_FILE") == 0) {
      conf >> pfc_output_file;
    } else if (key.compare("TRANSPORT_EVENT_OUTPUT_FILE") == 0) {
      conf >> transport_event_output_file;
    } else if (key.compare("TRANSPORT_EVENT_SEGMENT_BYTES") == 0) {
      conf >> transport_event_segment_byte_limit;
    } else if (key.compare("TRANSPORT_EVENT_SUMMARY_OUTPUT_FILE") == 0) {
      conf >> transport_event_summary_output_file;
    } else if (key.compare("PORT_COUNTER_OUTPUT_FILE") == 0) {
      conf >> port_counter_output_file;
    } else if (key.compare("SPINE_ARRIVAL_OUTPUT_FILE") == 0) {
      conf >> spine_arrival_output_file;
    } else if (key.compare("SPINE_REPORT_OUTPUT_FILE") == 0) {
      conf >> spine_report_output_file;
    } else if (key.compare("SPINE_LOSS_OUTPUT_FILE") == 0) {
      conf >> spine_loss_output_file;
    } else if (key.compare("RTO_EVENT_OUTPUT_FILE") == 0) {
      conf >> rto_event_output_file;
    } else if (key.compare("RECEIVER_HOLD_OUTPUT_FILE") == 0) {
      conf >> receiver_hold_output_file;
    } else if (key.compare("LINK_DOWN") == 0) {
      conf >> link_down_time >> link_down_A >> link_down_B;
    } else if (key.compare("LINK_FAILURE") == 0) {
      LinkEvent failure;
      std::string kind;
      conf >> failure.timeNs >> kind >> failure.a >> failure.b;
      if (kind == "down") {
        failure.kind = LinkEventKind::Down;
      } else if (kind == "blackhole") {
        failure.kind = LinkEventKind::Blackhole;
      } else if (kind == "loss") {
        failure.kind = LinkEventKind::Loss;
        conf >> failure.errorRate;
        if (!std::isfinite(failure.errorRate) || failure.errorRate <= 0.0 ||
            failure.errorRate > 1.0) {
          std::cerr << "LINK_FAILURE loss needs an error rate in (0, 1]\n";
          return false;
        }
      } else if (kind == "rate") {
        failure.kind = LinkEventKind::Rate;
        std::string rate;
        conf >> rate;
        // Parsed apart from conf, whose failbit would end the key loop.
        std::istringstream parse(rate);
        if (!(parse >> failure.rate) || failure.rate.GetBitRate() == 0) {
          std::cerr << "LINK_FAILURE rate needs a positive data rate\n";
          return false;
        }
      } else {
        std::cerr << "LINK_FAILURE kind must be down, blackhole, loss or rate\n";
        return false;
      }
      if (failure.kind == LinkEventKind::Loss ||
          failure.kind == LinkEventKind::Rate) {
        // Optional last tokens on the line: both, the default, or forward,
        // the direction from the first node named to the second alone; then,
        // under loss, byte and the bytes of the full data frame the rate was
        // derived at, the rate being each byte's.
        std::string rest, word;
        std::getline(conf, rest);
        std::istringstream tokens(rest);
        std::vector<std::string> words;
        while (tokens >> word)
          words.push_back(word);
        size_t next = 0;
        if (next < words.size() &&
            (words[next] == "forward" || words[next] == "both")) {
          if (words[next] == "forward")
            failure.direction = LinkDirection::Forward;
          next++;
        }
        if (failure.kind == LinkEventKind::Loss && next < words.size() &&
            words[next] == "byte") {
          std::istringstream frame(next + 1 < words.size() ? words[next + 1]
                                                           : "");
          if (!(frame >> failure.frameBytes) || failure.frameBytes == 0 ||
              !frame.eof()) {
            std::cerr << "LINK_FAILURE loss byte needs the data frame's "
                         "bytes\n";
            return false;
          }
          failure.unit = RateErrorModel::ERROR_UNIT_BYTE;
          next += 2;
        }
        if (next == 0 && !words.empty()) {
          std::cerr << "LINK_FAILURE " << kind
                    << " direction must be forward or both\n";
          return false;
        }
        if (next < words.size()) {
          std::cerr << "LINK_FAILURE " << kind << " takes nothing after its "
                    << (failure.unit == RateErrorModel::ERROR_UNIT_BYTE
                            ? "unit"
                            : "direction")
                    << "\n";
          return false;
        }
      }
      link_events.push_back(failure);
    } else if (key.compare("LINK_RESTORE") == 0) {
      LinkEvent restore;
      restore.kind = LinkEventKind::Restore;
      conf >> restore.timeNs >> restore.a >> restore.b;
      // A restore puts back what its failure set, in the failure's
      // directions, so it takes no direction.
      std::string rest, extra;
      std::getline(conf, rest);
      std::istringstream tokens(rest);
      if (tokens >> extra) {
        std::cerr << "LINK_RESTORE takes nothing after its two nodes\n";
        return false;
      }
      link_events.push_back(restore);
    } else if (key.compare("ENABLE_TRACE") == 0) {
      conf >> enable_trace;
    } else if (key.compare("DATA_QUEUE_BYTES_MAP") == 0 ||
               key.compare("TRIMMED_QUEUE_BYTES_MAP") == 0) {
      map<uint64_t, uint32_t> &rows = key.compare("DATA_QUEUE_BYTES_MAP") == 0
                                          ? rate2data_queue_bytes
                                          : rate2trimmed_queue_bytes;
      int n_k;
      conf >> n_k;
      for (int i = 0; i < n_k; i++) {
        uint64_t rate;
        uint32_t bytes;
        conf >> rate >> bytes;
        rows[rate] = bytes;
      }
    } else if (key.compare("KMAX_MAP") == 0) {
      int n_k;
      conf >> n_k;
      for (int i = 0; i < n_k; i++) {
        uint64_t rate;
        uint32_t k;
        conf >> rate >> k;
        rate2kmax[rate] = k;
      }
    } else if (key.compare("KMIN_MAP") == 0) {
      int n_k;
      conf >> n_k;
      for (int i = 0; i < n_k; i++) {
        uint64_t rate;
        uint32_t k;
        conf >> rate >> k;
        rate2kmin[rate] = k;
      }
    } else if (key.compare("PMAX_MAP") == 0) {
      int n_k;
      conf >> n_k;
      for (int i = 0; i < n_k; i++) {
        uint64_t rate;
        double p;
        conf >> rate >> p;
        rate2pmax[rate] = p;
      }
    } else if (key.compare("BUFFER_SIZE") == 0) {
      conf >> buffer_size;
    } else if (key.compare("QLEN_MON_FILE") == 0) {
      conf >> qlen_mon_file;
    } else if (key.compare("QLEN_MON_START") == 0) {
      conf >> qlen_mon_start;
    } else if (key.compare("QLEN_MON_INTERVAL") == 0) {
      conf >> qlen_mon_interval;
    } else if (key.compare("QLEN_MON_END") == 0) {
      conf >> qlen_mon_end;
    } else if (key.compare("MULTI_RATE") == 0) {
      int v;
      conf >> v;
      multi_rate = v;
    } else if (key.compare("SAMPLE_FEEDBACK") == 0) {
      int v;
      conf >> v;
      sample_feedback = v;
    } else if (key.compare("PINT_LOG_BASE") == 0) {
      conf >> pint_log_base;
    } else if (key.compare("PINT_PROB") == 0) {
      conf >> pint_prob;
    } else if (key == "HEADROOM_FACTOR") {
      int v;
      conf >> v;
      headroom_factor = v;
      std::cout << "headroom factor set to: " << headroom_factor << std::endl;
    }
    fflush(stdout);
  }
  conf.close();
  if (error_rate_per_link != 0.0) {
    std::cerr << "ERROR_RATE_PER_LINK is unsupported; use DATA_LOSS_* controls\n";
    return false;
  }
  if (!std::isfinite(data_loss_probability) || data_loss_probability < 0.0 ||
      data_loss_probability > 1.0) {
    std::cerr << "DATA_LOSS_PROBABILITY must be in [0, 1]\n";
    return false;
  }
  if (data_loss_scope_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "DATA_LOSS_SCOPE must be all, host_to_switch, switch_to_host, or switch_to_switch\n";
    return false;
  }
  if (data_loss_duration_ns == 0 && data_loss_probability != 0.0) {
    std::cerr << "DATA_LOSS_DURATION_NS must be nonzero when loss probability is nonzero\n";
    return false;
  }
  if (data_loss_duration_ns != 0 &&
      (retransmission_timeout_ns == 0 || max_retransmission_retries == 0)) {
    std::cerr << "loss experiments require retransmission timeout and retry budget\n";
    return false;
  }
  if (packet_trim_mode_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "PACKET_TRIM_MODE must be disabled, ftd, or bts\n";
    return false;
  }
  if (packet_trim_mode_value() !=
          static_cast<uint32_t>(PacketTrimMode::Disabled) &&
      (retransmission_timeout_ns == 0 || max_retransmission_retries == 0)) {
    std::cerr << "packet trimming requires retransmission timeout and retry budget\n";
    return false;
  }
  // Trim notifications and NACKs are exempt from the retry budget, so the
  // budget alone cannot bound a recovery loop that never advances snd_una.
  // The forward-progress deadline is the liveness bound for that loop class
  // and is therefore mandatory whenever trimming can generate such signals.
  if (packet_trim_mode_value() !=
          static_cast<uint32_t>(PacketTrimMode::Disabled) &&
      no_progress_timeout_ns == 0) {
    std::cerr << "packet trimming requires NO_PROGRESS_TIMEOUT_NS as the "
                 "liveness bound for budget-exempt recovery signals\n";
    return false;
  }
  if (selective_retransmission != 0 &&
      (retransmission_timeout_ns == 0 || max_retransmission_retries == 0)) {
    std::cerr << "SELECTIVE_RETRANSMISSION requires retransmission timeout and "
                 "retry budget as its silent-loss fallback\n";
    return false;
  }
  // UEC 1.0.3 section 4.1.4.1: switches MUST place trimmed packets in a traffic
  // class distinct from untrimmed data, and DSCP_TRIMMED MUST differ from
  // DSCP_CONTROL. Queue 0 carries DSCP_CONTROL, so TC_med cannot be queue 0.
  if (packet_trim_queue == 0 || packet_trim_queue > 7) {
    std::cerr << "PACKET_TRIM_QUEUE must name a TC_med queue in [1,7]\n";
    return false;
  }
  // UEC 1.0.3 Table 4-1: UET over UDP/IP needs 24 B so the UDP header and the
  // PDS request header survive trimming. The switch additionally raises the
  // retained prefix to cover any INT header the configured CC algorithm adds.
  if (min_trim_size < 24) {
    std::cerr << "MIN_TRIM_SIZE must be at least 24 bytes (UEC 1.0.3 Table 4-1)\n";
    return false;
  }
  OrderLinkEvents(link_events);
  const std::string unrestorable = CheckLinkEvents(link_events);
  if (!unrestorable.empty()) {
    std::cerr << unrestorable << "\n";
    return false;
  }
  if (packet_trim_queue_weight == 0 || packet_trim_queue_weight > 100) {
    std::cerr << "PACKET_TRIM_QUEUE_WEIGHT must be a percentage in [1,100]\n";
    return false;
  }
  // Headroom only exists to absorb packets already in flight when a PAUSE is
  // sent. Without PFC nothing ever pauses, so a nonzero headroom is dead buffer
  // that must fill before any packet can be trimmed or dropped.
  if (enable_pfc == 0 && headroom_factor != 0) {
    std::cerr << "HEADROOM_FACTOR must be 0 when ENABLE_PFC is 0; PFC headroom "
                 "is unreachable buffer in a best-effort fabric\n";
    return false;
  }
  if (packet_trim_mode_value() !=
          static_cast<uint32_t>(PacketTrimMode::Disabled) &&
      enable_pfc != 0) {
    std::cerr << "packet trimming requires ENABLE_PFC 0; UEC 1.0.3 section "
                 "3.6.4.5 excludes PFC from best-effort networks\n";
    return false;
  }
  if (rate2data_queue_bytes.empty() != rate2trimmed_queue_bytes.empty()) {
    std::cerr << "DATA_QUEUE_BYTES_MAP and TRIMMED_QUEUE_BYTES_MAP are read "
                 "together\n";
    return false;
  }
  if (packet_trim_mode_value() !=
          static_cast<uint32_t>(PacketTrimMode::Disabled) &&
      data_queue_bytes == 0) {
    std::cerr << "packet trimming requires a bounded DATA_QUEUE_BYTES; an "
                 "unbounded egress queue can never reject a packet\n";
    return false;
  }
  if (packet_trim_mode_value() ==
      static_cast<uint32_t>(PacketTrimMode::BackToSender)) {
    std::cerr << "warning: PACKET_TRIM_MODE bts sends trim metadata back to the "
                 "source, which UEC 1.0.3 section 4.1 explicitly excludes; use "
                 "ftd for UET-conformant trimming\n";
  }
  if (data_loss_rng_stream >
      static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
    std::cerr << "DATA_LOSS_RNG_STREAM exceeds ns-3 stream range\n";
    return false;
  }
  if (load_balancing_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "LOAD_BALANCING must be ecmp, ev_hash, spray_uniform or "
                 "spray_policy\n";
    return false;
  }
  for (const std::string &key : spray_policy_keys_given) {
    if (load_balancing != "spray_policy" &&
        !(host_redirect() && spine_grader_keys.count(key) > 0) &&
        !(path_selector == "reps_oracle" &&
          key == "SPRAY_GRADE_COST_SOURCE") &&
        !(reps_hold() && loss_hold_keys.count(key) > 0 &&
          !(path_selector == "reps_flow_hold" &&
            key == "SPINE_LOSS_OUTPUT_FILE"))) {
      std::cerr << key << " is read only under LOAD_BALANCING spray_policy"
                << (spine_grader_keys.count(key) > 0
                        ? " or PATH_SELECTOR ack_redirect with "
                          "ACK_REDIRECT_SCOPE per_receiver\n"
                    : key == "SPRAY_GRADE_COST_SOURCE"
                        ? " or PATH_SELECTOR reps_oracle\n"
                    : key == "SPINE_LOSS_OUTPUT_FILE"
                        ? " or PATH_SELECTOR reps_hold or mrc_hold\n"
                    : loss_hold_keys.count(key) > 0
                        ? " or PATH_SELECTOR reps_hold, reps_flow_hold or "
                          "mrc_hold\n"
                        : "\n");
      return false;
    }
    if (spray_one_way_delay_keys.count(key) > 0 && spray_one_way_delay == 0) {
      std::cerr << key << " is read only under SPRAY_ONE_WAY_DELAY 1\n";
      return false;
    }
  }
  if (load_balancing == "spray_policy" && !valid_spray_policy())
    return false;
  if (path_selector_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "PATH_SELECTOR must be ops, reps, ue_oblivious, ue_aware, mrc, "
                 "ack_redirect, reps_hold, reps_flow_hold, reps_oracle, "
                 "mrc_hold or reps_spine\n";
    return false;
  }
  if (reps_hold()) {
    if (load_balancing != "spray_uniform") {
      std::cerr << "PATH_SELECTOR " << path_selector
                << " names spines and requires LOAD_BALANCING spray_uniform\n";
      return false;
    }
    if (!valid_loss_hold())
      return false;
  }
  if (path_selector == "reps_oracle") {
    if (load_balancing != "spray_uniform") {
      std::cerr << "PATH_SELECTOR " << path_selector
                << " names spines and requires LOAD_BALANCING spray_uniform\n";
      return false;
    }
    if (!reps_oracle()) {
      std::cerr << "PATH_SELECTOR reps_oracle requires "
                   "SPRAY_GRADE_COST_SOURCE switch_queue\n";
      return false;
    }
  }
  if (reps_oracle_hops != "round_trip" && reps_oracle_hops != "forward") {
    std::cerr << "REPS_ORACLE_HOPS must be round_trip or forward\n";
    return false;
  }
  if (path_selector == "reps_spine" && load_balancing != "spray_uniform") {
    std::cerr << "PATH_SELECTOR " << path_selector
              << " names spines and requires LOAD_BALANCING spray_uniform\n";
    return false;
  }
  if (spray_hold_scope_given && !hold_scope_read()) {
    std::cerr << "SPRAY_HOLD_SCOPE is read only under PATH_SELECTOR "
                 "reps_hold or mrc_hold\n";
    return false;
  }
  if (spray_hold_scope_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "SPRAY_HOLD_SCOPE must be host, leaf, auto or pair\n";
    return false;
  }
  if (spray_hold_auto_leaves_given &&
      (!hold_scope_read() || spray_hold_scope != "auto") &&
      !(reps_hold() && receiver_hold())) {
    std::cerr << "SPRAY_HOLD_AUTO_LEAVES is read only under PATH_SELECTOR "
                 "reps_hold or mrc_hold with SPRAY_HOLD_SCOPE auto, or with "
                 "SPRAY_RECEIVER_HOLD on\n";
    return false;
  }
  if (spray_hold_auto_leaves < 2) {
    std::cerr << "SPRAY_HOLD_AUTO_LEAVES must be at least 2\n";
    return false;
  }
  if (!valid_hold_rule())
    return false;
  if (spray_path_seq != "on" && spray_path_seq != "off") {
    std::cerr << "SPRAY_PATH_SEQ must be on or off\n";
    return false;
  }
  if (spray_path_seq_given && load_balancing != "spray_uniform") {
    std::cerr << "SPRAY_PATH_SEQ is read only under LOAD_BALANCING "
                 "spray_uniform\n";
    return false;
  }
  if (spray_path_state != "message" && spray_path_state != "connection") {
    std::cerr << "SPRAY_PATH_STATE must be message or connection\n";
    return false;
  }
  if (spray_path_state_given && load_balancing != "spray_uniform") {
    std::cerr << "SPRAY_PATH_STATE is read only under LOAD_BALANCING "
                 "spray_uniform\n";
    return false;
  }
  if (spray_ack_loss_rtts_given && load_balancing != "spray_uniform") {
    std::cerr << "SPRAY_ACK_LOSS_RTTS is read only under LOAD_BALANCING "
                 "spray_uniform\n";
    return false;
  }
  if (!(spray_ack_loss_rtts >= 0)) {
    std::cerr << "SPRAY_ACK_LOSS_RTTS must be at least 0\n";
    return false;
  }
  if (ack_route != "data_spine" && ack_route != "spray" &&
      ack_route != "reverse_set" && ack_route != "repath") {
    std::cerr << "ACK_ROUTE must be data_spine, spray, reverse_set or repath\n";
    return false;
  }
  if (ack_route_set_size_given && ack_route != "reverse_set") {
    std::cerr << "ACK_ROUTE_SET_SIZE is read only under ACK_ROUTE reverse_set\n";
    return false;
  }
  if (ack_route_set_size == 0) {
    std::cerr << "ACK_ROUTE_SET_SIZE must be above zero\n";
    return false;
  }
  if (ack_route == "repath" && spray_tail_recovery != "on") {
    std::cerr << "ACK_ROUTE repath tells a copied send by its tail flags and "
                 "needs SPRAY_TAIL_RECOVERY on\n";
    return false;
  }
  if (ack_route_given && load_balancing != "spray_uniform") {
    std::cerr << "ACK_ROUTE is read only under LOAD_BALANCING spray_uniform\n";
    return false;
  }
  if (ack_route_oracle != "on" && ack_route_oracle != "off") {
    std::cerr << "ACK_ROUTE_ORACLE must be on or off\n";
    return false;
  }
  if (ack_route_oracle_given && !ack_route_spray()) {
    std::cerr << "ACK_ROUTE_ORACLE is read only under ACK_ROUTE spray\n";
    return false;
  }
  if (spray_tail_recovery != "on" && spray_tail_recovery != "off") {
    std::cerr << "SPRAY_TAIL_RECOVERY must be on or off\n";
    return false;
  }
  if (spray_tail_recovery_given && load_balancing != "spray_uniform") {
    std::cerr << "SPRAY_TAIL_RECOVERY is read only under LOAD_BALANCING "
                 "spray_uniform\n";
    return false;
  }
  if (tail_recovery() && spray_path_seq != "on") {
    std::cerr << "SPRAY_TAIL_RECOVERY requires SPRAY_PATH_SEQ on\n";
    return false;
  }
  if (spray_tail_recovery_delay_ns_given && !tail_recovery()) {
    std::cerr << "SPRAY_TAIL_RECOVERY_DELAY_NS is read only with "
                 "SPRAY_TAIL_RECOVERY on\n";
    return false;
  }
  if (spray_tail_recovery_delay_ns_given &&
      spray_tail_recovery_delay_ns == RdmaHw::kTailDelayBaseRtt) {
    std::cerr << "SPRAY_TAIL_RECOVERY_DELAY_NS is out of range\n";
    return false;
  }
  if (spray_tail_loss_rtts_given && !tail_recovery()) {
    std::cerr << "SPRAY_TAIL_LOSS_RTTS is read only with "
                 "SPRAY_TAIL_RECOVERY on\n";
    return false;
  }
  if (!(spray_tail_loss_rtts >= 0)) {
    std::cerr << "SPRAY_TAIL_LOSS_RTTS must be at least 0\n";
    return false;
  }
  if (spray_receiver_hold != "on" && spray_receiver_hold != "off") {
    std::cerr << "SPRAY_RECEIVER_HOLD must be on or off\n";
    return false;
  }
  if (spray_receiver_hold_given && !reps_hold()) {
    std::cerr << "SPRAY_RECEIVER_HOLD is read only under LOAD_BALANCING "
                 "spray_uniform with PATH_SELECTOR reps_hold, "
                 "reps_flow_hold or mrc_hold\n";
    return false;
  }
  if (receiver_hold() && spray_path_seq != "on") {
    std::cerr << "SPRAY_RECEIVER_HOLD requires SPRAY_PATH_SEQ on\n";
    return false;
  }
  if (spray_receiver_hold_clean_given && !receiver_hold()) {
    std::cerr << "SPRAY_RECEIVER_HOLD_CLEAN is read only with "
                 "SPRAY_RECEIVER_HOLD on\n";
    return false;
  }
  if (spray_receiver_hold_clean == 0) {
    std::cerr << "SPRAY_RECEIVER_HOLD_CLEAN must be above zero\n";
    return false;
  }
  if (spray_receiver_tail_missing != "on" &&
      spray_receiver_tail_missing != "off") {
    std::cerr << "SPRAY_RECEIVER_TAIL_MISSING must be on or off\n";
    return false;
  }
  if (spray_receiver_tail_missing_given &&
      (!receiver_hold() || !tail_recovery())) {
    std::cerr << "SPRAY_RECEIVER_TAIL_MISSING is read only with "
                 "SPRAY_RECEIVER_HOLD and SPRAY_TAIL_RECOVERY on\n";
    return false;
  }
  if (spray_stray_path_seq != "fresh" && spray_stray_path_seq != "closed") {
    std::cerr << "SPRAY_STRAY_PATH_SEQ must be fresh or closed\n";
    return false;
  }
  if (spray_stray_path_seq_given && spray_path_seq != "on") {
    std::cerr << "SPRAY_STRAY_PATH_SEQ is read only with SPRAY_PATH_SEQ on\n";
    return false;
  }
  if (spray_coverage_first != "on" && spray_coverage_first != "off") {
    std::cerr << "SPRAY_COVERAGE_FIRST must be on or off\n";
    return false;
  }
  if (spray_coverage_first_given && load_balancing != "spray_uniform") {
    std::cerr << "SPRAY_COVERAGE_FIRST is read only under LOAD_BALANCING "
                 "spray_uniform\n";
    return false;
  }
  if (spray_duplicate_report != "on" && spray_duplicate_report != "off" &&
      spray_duplicate_report != "retract") {
    std::cerr << "SPRAY_DUPLICATE_REPORT must be on, off or retract\n";
    return false;
  }
  if (spray_duplicate_report == "retract" &&
      (!hold_scope_read() || spray_hold_scope != "auto" ||
       spray_hold_rule != "count")) {
    std::cerr << "SPRAY_DUPLICATE_REPORT retract takes losses back from "
                 "PATH_SELECTOR reps_hold's or mrc_hold's loss hold under "
                 "SPRAY_HOLD_SCOPE auto and SPRAY_HOLD_RULE count\n";
    return false;
  }
  if (spray_duplicate_report_given && spray_coverage_first != "on") {
    std::cerr << "SPRAY_DUPLICATE_REPORT is read only with SPRAY_COVERAGE_FIRST "
                 "on\n";
    return false;
  }
  if (spray_answer_path_seq != "on" && spray_answer_path_seq != "off") {
    std::cerr << "SPRAY_ANSWER_PATH_SEQ must be on or off\n";
    return false;
  }
  if (spray_answer_path_seq_given &&
      (load_balancing != "spray_uniform" || !reps_hold() ||
       spray_path_seq != "on" || ack_route != "spray")) {
    std::cerr << "SPRAY_ANSWER_PATH_SEQ is read only under ACK_ROUTE spray "
                 "with SPRAY_PATH_SEQ on and PATH_SELECTOR reps_hold, "
                 "reps_flow_hold or mrc_hold\n";
    return false;
  }
  if ((spray_answer_probe_interval_ns_given || spray_answer_hold_clean_given ||
       !answer_hold_output_file.empty()) &&
      !answer_path_seq()) {
    std::cerr << "SPRAY_ANSWER_PROBE_INTERVAL_NS, SPRAY_ANSWER_HOLD_CLEAN and "
                 "ANSWER_HOLD_OUTPUT_FILE are read only with "
                 "SPRAY_ANSWER_PATH_SEQ on\n";
    return false;
  }
  if (spray_answer_probe_interval_ns == 0 || spray_answer_hold_clean == 0) {
    std::cerr << "SPRAY_ANSWER_PROBE_INTERVAL_NS and SPRAY_ANSWER_HOLD_CLEAN "
                 "must be above zero\n";
    return false;
  }
  if (!receiver_hold_output_file.empty() && !receiver_hold()) {
    std::cerr << "RECEIVER_HOLD_OUTPUT_FILE is read only with "
                 "SPRAY_RECEIVER_HOLD on\n";
    return false;
  }
  if (path_selector == "ack_redirect") {
    if (load_balancing != "spray_uniform") {
      std::cerr << "PATH_SELECTOR ack_redirect names spines and requires "
                   "LOAD_BALANCING spray_uniform\n";
      return false;
    }
    if (!valid_ack_redirect())
      return false;
  } else if (path_selector != "ops" && !reps_hold() &&
             path_selector != "reps_oracle" && path_selector != "reps_spine" &&
             load_balancing_value() !=
                 static_cast<uint32_t>(LoadBalancingMode::EntropyHash)) {
    std::cerr << "PATH_SELECTOR " << path_selector
              << " chooses entropy values and requires LOAD_BALANCING ev_hash\n";
    return false;
  }
  for (const std::string &key : path_selector_parameters_given) {
    if (path_selector_parameter_owners.at(key).count(path_selector) == 0) {
      std::cerr << key << " is not a parameter of PATH_SELECTOR "
                << path_selector << "\n";
      return false;
    }
    if (host_redirect() && ack_redirect_connection_keys.count(key) > 0) {
      std::cerr << key << " is not read under ACK_REDIRECT_SCOPE per_receiver\n";
      return false;
    }
  }
  // Outside ECMP the packets of one flow take different paths and arrive out
  // of order: the receiver must hold out-of-order data instead of dropping it,
  // and a congested switch must hand the destination the range it cut.
  if (load_balancing_value() != static_cast<uint32_t>(LoadBalancingMode::Ecmp) &&
      (selective_retransmission == 0 ||
       packet_trim_mode_value() !=
           static_cast<uint32_t>(PacketTrimMode::ForwardToDestination))) {
    std::cerr << "LOAD_BALANCING " << load_balancing
              << " requires SELECTIVE_RETRANSMISSION 1 and PACKET_TRIM_MODE ftd\n";
    return false;
  }
  // CC_MODE 11 (NSCC) clocks its window on the acknowledgement of every data
  // packet and times each from the packet's send record; under ECMP the
  // receiver acknowledges cumulatively and the sender keeps no records.
  if (cc_mode == 11 &&
      load_balancing_value() == static_cast<uint32_t>(LoadBalancingMode::Ecmp)) {
    std::cerr << "CC_MODE 11 (NSCC) requires LOAD_BALANCING ev_hash or "
                 "spray_uniform, which acknowledge every data packet\n";
    return false;
  }
  // HAS_WIN 0 hands a queue pair no bandwidth-delay product, and NSCC sizes
  // its window from it.
  if (cc_mode == 11 && has_win == 0) {
    std::cerr << "CC_MODE 11 (NSCC) requires HAS_WIN 1\n";
    return false;
  }
  if (nscc_window_state != "message" && nscc_window_state != "connection") {
    std::cerr << "NSCC_WINDOW_STATE must be message or connection\n";
    return false;
  }
  if (nscc_window_state_given && cc_mode != 11) {
    std::cerr << "NSCC_WINDOW_STATE is read only under CC_MODE 11 (NSCC)\n";
    return false;
  }
  if (nscc_rtt_sample != "any" && nscc_rtt_sample != "karn") {
    std::cerr << "NSCC_RTT_SAMPLE must be any or karn\n";
    return false;
  }
  if (nscc_rtt_sample_given && cc_mode != 11) {
    std::cerr << "NSCC_RTT_SAMPLE is read only under CC_MODE 11 (NSCC)\n";
    return false;
  }
  if (pair_bandwidth != "path" && pair_bandwidth != "spines") {
    std::cerr << "PAIR_BANDWIDTH must be path or spines\n";
    return false;
  }
  if (pair_bandwidth == "spines" && !names_spine()) {
    std::cerr << "PAIR_BANDWIDTH spines needs a LOAD_BALANCING mode that "
                 "names spines\n";
    return false;
  }
  if (load_balancing_value() != static_cast<uint32_t>(LoadBalancingMode::Ecmp) &&
      data_loss_duration_ns != 0 &&
      data_loss_rng_stream == static_cast<uint64_t>(path_rng_stream)) {
    std::cerr << "DATA_LOSS_RNG_STREAM " << path_rng_stream
              << " is the path draw's stream under LOAD_BALANCING "
              << load_balancing << "\n";
    return false;
  }
  if (qlen_mon_interval == 0) {
    std::cerr << "QLEN_MON_INTERVAL must be positive\n";
    return false;
  }
  // Only a requested spine is named in the identification, so only then can a
  // receiver tell which spine carried a packet.
  if (!spine_arrival_output_file.empty() && !names_spine()) {
    std::cerr << "SPINE_ARRIVAL_OUTPUT_FILE requires LOAD_BALANCING "
                 "spray_uniform or spray_policy\n";
    return false;
  }
  return true;
}

void SetConfig() {
  bool dynamicth = use_dynamic_pfc_threshold;

  Config::SetDefault("ns3::QbbNetDevice::PauseTime", UintegerValue(pause_time));
  Config::SetDefault("ns3::QbbNetDevice::QcnEnabled", BooleanValue(enable_qcn));
  Config::SetDefault("ns3::QbbNetDevice::QbbEnabled",
                     BooleanValue(enable_pfc != 0));
  Config::SetDefault("ns3::QbbNetDevice::DynamicThreshold",
                     BooleanValue(dynamicth));

  // Give trimmed packets their own TC_med scheduling tier only when trimming is
  // enabled, so baseline runs keep the original two-tier egress discipline.
  if (packet_trim_mode_value() !=
      static_cast<uint32_t>(PacketTrimMode::Disabled)) {
    Config::SetDefault("ns3::BEgressQueue::MediumPriorityQueue",
                       UintegerValue(packet_trim_queue));
    Config::SetDefault("ns3::BEgressQueue::MediumPriorityWeight",
                       UintegerValue(packet_trim_queue_weight));
  }

  // set int_multi
  IntHop::multi = int_multi;
  // IntHeader::mode
  if (cc_mode == 7) // timely, use ts
    IntHeader::mode = IntHeader::TS;
  else if (cc_mode == 3) // hpcc, use int
    IntHeader::mode = IntHeader::NORMAL;
  else if (cc_mode == 10) // hpcc-pint
    IntHeader::mode = IntHeader::PINT;
  else if (spray_one_way_delay == 1 || path_selector == "ack_redirect")
    IntHeader::mode = IntHeader::TS; // the send time of every data packet
  else // others, no extra header
    IntHeader::mode = IntHeader::NONE;
  // Outside ECMP the sender matches each acknowledgement to the data packet
  // that triggered it; ECMP keeps the header at its old size.
  CustomHeader::ackCarriesPacketSeq =
      load_balancing_value() != static_cast<uint32_t>(LoadBalancingMode::Ecmp);
  CustomHeader::ackRedirectBytes = path_selector == "ack_redirect" ? 1 : 0;
  CustomHeader::udpCarriesPathSeq = spray_path_seq == "on";
  CustomHeader::ackReceiverHoldBytes = receiver_hold() ? 1 : 0;
  CustomHeader::ackTailNackBytes = tail_recovery() ? 5 : 0;
  CustomHeader::ackRouteSpineBytes = ack_route_named() ? 1 : 0;
  CustomHeader::ackAnswerSeqBytes = answer_path_seq() ? 5 : 0;
  CustomHeader::udpAnswerHoldBytes = answer_path_seq() ? 1 : 0;

  // Set Pint
  if (cc_mode == 10) {
    Pint::set_log_base(pint_log_base);
    IntHeader::pint_bytes = Pint::get_n_bytes();
    printf("PINT bits: %d bytes: %d\n", Pint::get_n_bits(),
           Pint::get_n_bytes());
  }
}

// `recovery_verdict` is the experiment layer's answer to "what do I do with
// this trimmed range", `forgiveness_eligible` its answer to "may I forgive
// this flow on this step", which the receiver carries on its
// acknowledgements, and `remainder_verdict` its answer to "may I take this
// flow's remainder as delivered": the transport asks, it never decides. `data_accepted` is the one report in the other direction, the
// payload bytes an arrival added to what the receiver holds, which is what the
// receiver-local budget is measured against. Null callbacks with
// `forgiveness` and `congestion_exempt` false leave the pull-everything,
// congestion-obeying transport untouched, and a null `remainder_verdict` is
// what disables the step stop.
// The one call in the other direction, from the experiment layer into the
// transport: the receiver's budget says a sender is done, so its open queue
// pairs run the remainder path now instead of waiting for packets that may
// never come. A lookup by rank rather than another function pointer, because
// the node container already maps a rank to its transport and the caller knows
// which rank is receiving.
bool StopFlowAtReceiver(uint32_t receiver, uint32_t sip, uint32_t dip,
                        uint16_t sport, uint16_t dport) {
  if (receiver >= n.GetN())
    return false;
  Ptr<RdmaDriver> rdma = n.Get(receiver)->GetObject<RdmaDriver>();
  if (rdma == nullptr || rdma->m_rdma == nullptr)
    return false;
  return rdma->m_rdma->StopFlow(sip, dip, sport, dport);
}

bool SetupNetwork(void (*qp_finish)(FILE *, Ptr<RdmaQueuePair>),
                  void (*qp_fail)(FILE *, Ptr<RdmaQueuePair>, uint32_t),
                  bool (*recovery_verdict)(uint32_t, uint32_t, uint16_t,
                                           uint16_t, uint64_t,
                                           uint32_t) = nullptr,
                  bool forgiveness = false,
                  bool (*forgiveness_eligible)(uint32_t, uint32_t, uint16_t,
                                               uint16_t) = nullptr,
                  bool congestion_exempt = false,
                  uint64_t (*remainder_verdict)(uint32_t, uint32_t, uint16_t,
                                                uint16_t, uint64_t,
                                                uint64_t) = nullptr,
                  void (*data_accepted)(uint32_t, uint32_t, uint16_t, uint16_t,
                                        uint64_t, uint64_t) = nullptr,
                  bool reengage = true,
                  bool (*allowance_gone)(uint32_t, uint32_t, uint16_t,
                                         uint16_t, uint64_t) = nullptr) {

  topof.open(topology_file.c_str());
  if (!topof.is_open()) {
    std::cerr << "Error: cannot open topology file: " << topology_file << std::endl;
    return false;
  }

  flowf.open(flow_file.c_str());
  if (!flowf.is_open()) {
    std::cerr << "Error: cannot open flow file: " << flow_file << std::endl;
    return false;
  }

  tracef.open(trace_file.c_str());
  if (!tracef.is_open()) {
    std::cerr << "Error: cannot open trace file: " << trace_file << std::endl;
    return false;
  }

  uint32_t node_num, switch_num, link_num, trace_num;
  topof >> node_num >> switch_num >> link_num;
  flowf >> flow_num;
  tracef >> trace_num;
  if (data_loss_duration_ns != 0 &&
      data_loss_rng_stream + 2 * static_cast<uint64_t>(link_num) >
          static_cast<uint64_t>(device_rng_block)) {
    std::cerr << "DATA_LOSS_RNG_STREAM " << data_loss_rng_stream
              << " leaves too few streams below the per-device streams for "
              << link_num << " links\n";
    return false;
  }

  std::vector<uint32_t> node_type(node_num, 0);
  for (uint32_t i = 0; i < switch_num; i++) {
    uint32_t sid;
    topof >> sid;
    node_type[sid] = 1;
  }
  for (uint32_t i = 0; i < node_num; i++) {
    if (node_type[i] == 0)
      n.Add(CreateObject<Node>());
    else {
      Ptr<SwitchNode> sw = CreateObject<SwitchNode>();
      n.Add(sw);
      sw->SetAttribute("EcnEnabled", BooleanValue(enable_qcn));
    }
  }

  NS_LOG_INFO("Create nodes.");

  InternetStackHelper internet;
  internet.Install(n);

  //
  // Assign IP to each server
  //
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() == 0) {
      serverAddress.resize(i + 1);
      serverAddress[i] = node_id_to_ip(i);
    }
  }

  NS_LOG_INFO("Create channels.");

  FILE *pfc_file = fopen(pfc_output_file.c_str(), "w");
  transport_event_summary_file =
      fopen(transport_event_summary_output_file.c_str(), "w");
  // Segment .000 always exists - the research invariant now holds at
  // segment granularity, and the configured name is the family's base.
  transport_event_raw_file =
      fopen(transport_event_segment_path(0).c_str(), "wb");
  transport_event_raw_cctx = ZSTD_createCCtx();
  if (pfc_file == nullptr || transport_event_summary_file == nullptr ||
      transport_event_raw_file == nullptr ||
      transport_event_raw_cctx == nullptr) {
    std::cerr << "Error: cannot open PFC or transport event output files\n";
    return false;
  }
  // Written under every mode, zero where SPRAY_ACK_LOSS_RTTS is off.
  transport_event_totals[{"ack_loss_declared", "control"}];
  // Written under every mode, zero where SPRAY_TAIL_LOSS_RTTS defers nothing.
  transport_event_totals[{"tail_loss_cancelled", "control"}];
  transport_event_totals[{"tail_loss_delivered", "control"}];
  transport_event_totals[{"tail_repair_rerouted", "control"}];
  transport_event_totals[{"tail_loss_ambiguous", "control"}];
  // Written under every mode, zero where SPRAY_RECEIVER_TAIL_MISSING is off;
  // their breakdowns by spine only where a spine has one.
  transport_event_totals[{"receiver_tail_missing", "control"}];
  transport_event_totals[{"receiver_tail_missing_late", "control"}];
  // Written under every mode; their breakdowns by spine only where a spine
  // has one.
  transport_event_totals[{"answer_drop_up", "control"}];
  transport_event_totals[{"answer_drop_down", "control"}];
  // Written where their key is on.
  if (spray_coverage_first == "on")
    transport_event_totals[{"answer_loss_covered", "control"}];
  if (spray_duplicate_report != "off")
    for (const char *event : {"answer_loss_duplicate", "timeout_loss_data"})
      transport_event_totals[{event, "control"}];
  if (spray_duplicate_report == "retract")
    transport_event_totals[{"timeout_retract_release", "control"}];
  if (answer_path_seq())
    for (const char *event : {"answer_route_lost", "answer_route_hold",
                              "answer_route_release", "answer_probe"})
      transport_event_totals[{event, "control"}];
  if (ack_route == "reverse_set")
    for (const char *event : {"answer_set_probe", "answer_set_replace"})
      transport_event_totals[{event, "control"}];
  if (ack_route == "repath")
    transport_event_totals[{"answer_repath", "control"}];
  // The totals must land even when a watchdog exits the run mid-simulation.
  std::atexit(write_transport_event_summary);
  ZSTD_CCtx_setParameter(transport_event_raw_cctx, ZSTD_c_compressionLevel, 8);
  // Long-distance matching folds the workload's periodic collective
  // patterns; window 2^27 stays within every decoder's no-flags default.
  ZSTD_CCtx_setParameter(transport_event_raw_cctx,
                         ZSTD_c_enableLongDistanceMatching, 1);
  ZSTD_CCtx_setParameter(transport_event_raw_cctx, ZSTD_c_windowLog, 27);
  // One worker keeps compression off the simulation thread. On a
  // single-threaded libzstd this parameter is refused and the stream
  // simply compresses synchronously - correct either way.
  ZSTD_CCtx_setParameter(transport_event_raw_cctx, ZSTD_c_nbWorkers, 1);
  transport_event_raw_pending.reserve(1u << 22);
  transport_event_raw_scratch.resize(ZSTD_CStreamOutSize());
  transport_event_last_flush = time(nullptr);
  static const char raw_header[] =
      "time_ns,event,plane,protocol,node,node_type,interface,source_host,"
      "destination_host,source_port,sequence,packet_bytes,queue\n";
  append_transport_event_raw(raw_header, sizeof(raw_header) - 1);
  std::atexit(finalize_transport_event_raw);
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() == 1) {
      Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(n.Get(i));
      sw->SetAttribute("AckHighPrio", UintegerValue(ack_high_prio));
        sw->SetAttribute("PacketTrimMode",
                 UintegerValue(packet_trim_mode_value()));
      sw->SetAttribute("TrimmedQueueIndex", UintegerValue(packet_trim_queue));
      sw->SetAttribute("MinTrimSize", UintegerValue(min_trim_size));
      sw->SetAttribute("LastHopTrimCodepoint",
                       BooleanValue(packet_trim_lasthop != 0));
      sw->SetAttribute("PfcEnabled", BooleanValue(enable_pfc != 0));
      sw->SetAttribute("LoadBalancing", UintegerValue(load_balancing_value()));
      sw->TraceConnectWithoutContext(
          "SwitchDrop", MakeBoundCallback(&get_switch_drop, sw));
        sw->TraceConnectWithoutContext(
          "PacketTrim", MakeBoundCallback(&get_switch_trim, sw));
    }
  }

  QbbHelper qbb;
  Ipv4AddressHelper ipv4;
  for (uint32_t i = 0; i < link_num; i++) {
    uint32_t src, dst;
    std::string data_rate, link_delay;
    double error_rate;
    topof >> src >> dst >> data_rate >> link_delay >> error_rate;
    if (!std::isfinite(error_rate) || error_rate < 0.0 || error_rate > 1.0) {
      std::cerr << "Topology link " << src << " " << dst
                << " has an error rate outside [0, 1]\n";
      return false;
    }
    Ptr<Node> snode = n.Get(src), dnode = n.Get(dst);

    qbb.SetDeviceAttribute("DataRate", StringValue(data_rate));
    qbb.SetChannelAttribute("Delay", StringValue(link_delay));

    fflush(stdout);

    // Assigne server IP
    // Note: this should be before the automatic assignment below
    // (ipv4.Assign(d)), because we want our IP to be the primary IP (first in
    // the IP address list), so that the global routing is based on our IP
    NetDeviceContainer d = qbb.Install(snode, dnode);
    Ptr<QbbNetDevice> src_dev = DynamicCast<QbbNetDevice>(d.Get(0));
    Ptr<QbbNetDevice> dst_dev = DynamicCast<QbbNetDevice>(d.Get(1));
    configure_data_loss(src_dev, static_cast<uint64_t>(i) * 2);
    configure_data_loss(dst_dev, static_cast<uint64_t>(i) * 2 + 1);
    if (error_rate != 0.0) {
      set_link_error_rate(src_dev, error_rate);
      set_link_error_rate(dst_dev, error_rate);
    }
    connect_transport_traces(src_dev);
    connect_transport_traces(dst_dev);
    for (Ptr<QbbNetDevice> dev : {src_dev, dst_dev})
      dev->SetAttribute(
          "PfcIdentificationStream",
          IntegerValue(device_rng_stream(DeviceRngBlock::PfcIdentification, dev)));
    if (snode->GetNodeType() == 0) {
      Ptr<Ipv4> ipv4 = snode->GetObject<Ipv4>();
      ipv4->AddInterface(d.Get(0));
      ipv4->AddAddress(
          1, Ipv4InterfaceAddress(serverAddress[src], Ipv4Mask(0xff000000)));

    }
    if (dnode->GetNodeType() == 0) {
      Ptr<Ipv4> ipv4 = dnode->GetObject<Ipv4>();
      ipv4->AddInterface(d.Get(1));
      ipv4->AddAddress(
          1, Ipv4InterfaceAddress(serverAddress[dst], Ipv4Mask(0xff000000)));
    }

    // used to create a graph of the topology
    nbr2if[snode][dnode].idx =
        DynamicCast<QbbNetDevice>(d.Get(0))->GetIfIndex();
    nbr2if[snode][dnode].up = true;
    nbr2if[snode][dnode].delay =
        DynamicCast<QbbChannel>(
            DynamicCast<QbbNetDevice>(d.Get(0))->GetChannel())
            ->GetDelay()
            .GetTimeStep();
    nbr2if[snode][dnode].bw =
        DynamicCast<QbbNetDevice>(d.Get(0))->GetDataRate().GetBitRate();
    nbr2if[dnode][snode].idx =
        DynamicCast<QbbNetDevice>(d.Get(1))->GetIfIndex();
    nbr2if[dnode][snode].up = true;
    nbr2if[dnode][snode].delay =
        DynamicCast<QbbChannel>(
            DynamicCast<QbbNetDevice>(d.Get(1))->GetChannel())
            ->GetDelay()
            .GetTimeStep();
    nbr2if[dnode][snode].bw =
        DynamicCast<QbbNetDevice>(d.Get(1))->GetDataRate().GetBitRate();

    // This is just to set up the connectivity between nodes. The IP addresses
    // are useless
    char ipstring[16];
    sprintf(ipstring, "10.%d.%d.0", i / 254 + 1, i % 254 + 1);
    ipv4.SetBase(ipstring, "255.255.255.0");
    ipv4.Assign(d);

    // setup PFC trace
    src_dev->TraceConnectWithoutContext(
        "QbbPfc", MakeBoundCallback(&get_pfc, pfc_file,
                    src_dev));
    dst_dev->TraceConnectWithoutContext(
        "QbbPfc", MakeBoundCallback(&get_pfc, pfc_file,
                    dst_dev));
  }

  nic_rate = get_nic_rate(n);
  // config switch
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() == 1) { // is switch
      Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(n.Get(i));
      uint32_t shift = 3; // by default 1/8

      for (uint32_t j = 1; j < sw->GetNDevices(); j++) {
        Ptr<QbbNetDevice> dev = DynamicCast<QbbNetDevice>(sw->GetDevice(j));
        // set ecn
        uint64_t rate = dev->GetDataRate().GetBitRate();
        NS_ASSERT_MSG(rate2kmin.find(rate) != rate2kmin.end(),
                      "must set kmin for each link speed");
        NS_ASSERT_MSG(rate2kmax.find(rate) != rate2kmax.end(),
                      "must set kmax for each link speed");
        NS_ASSERT_MSG(rate2pmax.find(rate) != rate2pmax.end(),
                      "must set pmax for each link speed");
        sw->m_mmu->ConfigEcn(j, rate2kmin[rate], rate2kmax[rate],
                             rate2pmax[rate],
                             device_rng_stream(DeviceRngBlock::EcnMarking, dev));
        // set pfc
        uint64_t delay = DynamicCast<QbbChannel>(dev->GetChannel())
                             ->GetDelay()
                             .GetTimeStep();
        // uint32_t headroom = 150000; // rate * delay / 8 / 1000000000 * 3;
        uint32_t headroom = rate * delay / 8 / 1000000000 * headroom_factor;
        sw->m_mmu->ConfigHdrm(j, headroom);

        // set pfc alpha, proportional to link bw
        sw->m_mmu->pfc_a_shift[j] = shift;
        while (rate > nic_rate && sw->m_mmu->pfc_a_shift[j] > 0) {
          sw->m_mmu->pfc_a_shift[j]--;
          rate /= 2;
        }
      }
      sw->m_mmu->ConfigNPort(sw->GetNDevices() - 1);
      sw->m_mmu->ConfigBufferSize(buffer_size * 1024 * 1024);
      // TC_low carries data on every configured priority group; TC_med carries
      // trimmed packets. Queue 0 (TC_high, control) stays unbounded.
      for (uint32_t q = 1; q < 8; q++) {
        sw->m_mmu->ConfigEgressThreshold(q, data_queue_bytes);
      }
      sw->m_mmu->ConfigEgressThreshold(packet_trim_queue, trimmed_queue_bytes);
      // Under the maps each port's thresholds are its link speed's.
      for (uint32_t j = 1; j < sw->GetNDevices() && !rate2data_queue_bytes.empty();
           j++) {
        const uint64_t rate = DynamicCast<QbbNetDevice>(sw->GetDevice(j))
                                  ->GetDataRate()
                                  .GetBitRate();
        if (rate2data_queue_bytes.count(rate) == 0 ||
            rate2trimmed_queue_bytes.count(rate) == 0) {
          std::cerr << "DATA_QUEUE_BYTES_MAP and TRIMMED_QUEUE_BYTES_MAP have "
                       "no row for a switch port of "
                    << rate << " bps\n";
          return false;
        }
        for (uint32_t q = 1; q < 8; q++) {
          sw->m_mmu->ConfigEgressThreshold(j, q, rate2data_queue_bytes[rate]);
        }
        sw->m_mmu->ConfigEgressThreshold(j, packet_trim_queue,
                                         rate2trimmed_queue_bytes[rate]);
      }
      sw->m_mmu->node_id = sw->GetId();
    }
  }

  FindSpineLinkReceivers(n);
  uint32_t spine_count = 0;
  if (names_spine()) {
    spine_count = AssignSpinePorts(n);
    if (spine_count == 0) {
      std::cerr << "LOAD_BALANCING " << load_balancing
                << " needs a leaf-spine fabric in which every leaf links to "
                   "every spine\n";
      return false;
    }
    if (spine_count > kMaxSpineCount) {
      std::cerr << "LOAD_BALANCING " << load_balancing
                << " names a spine in a byte of the identification, at most "
                << kMaxSpineCount << " spines\n";
      return false;
    }
  }
  if (load_balancing == "spray_policy") {
    if (spine_count > SpineReport::kMaxSpines) {
      std::cerr << "LOAD_BALANCING spray_policy grades at most "
                << SpineReport::kMaxSpines << " spines\n";
      return false;
    }
    CustomHeader::ackReportBytes = 1 + SpineReport::GradeBytes(spine_count);
  }
  if (host_redirect() && spine_count > SpineReport::kMaxSpines) {
    std::cerr << "ACK_REDIRECT_SCOPE per_receiver grades at most "
              << SpineReport::kMaxSpines << " spines\n";
    return false;
  }
  if (reps_hold() && spine_count > SpineReport::kMaxSpines) {
    std::cerr << "PATH_SELECTOR " << path_selector << " holds at most "
              << SpineReport::kMaxSpines << " spines\n";
    return false;
  }
  if (tail_recovery() && spine_count > kMaxTailCounts) {
    std::cerr << "SPRAY_TAIL_RECOVERY counts at most " << kMaxTailCounts
              << " spines\n";
    return false;
  }
  CustomHeader::udpTailCounts = tail_recovery() ? spine_count : 0;
  if (ack_route == "reverse_set" && ack_route_set_size > spine_count) {
    std::cerr << "ACK_ROUTE_SET_SIZE is at most the fabric's " << spine_count
              << " spines\n";
    return false;
  }
  if (receiver_hold() && spine_count > kReceiverHoldSpines) {
    std::cerr << "SPRAY_RECEIVER_HOLD holds at most " << kReceiverHoldSpines
              << " spines\n";
    return false;
  }
  // SetConfig sized the spine fields and bitmaps in a byte; they take the
  // bytes the fabric's spines need, one up to 8 spines.
  if (spine_count > 0) {
    if (CustomHeader::ackRedirectBytes > 0)
      CustomHeader::ackRedirectBytes = SpineFieldBytes(spine_count);
    if (CustomHeader::ackReceiverHoldBytes > 0)
      CustomHeader::ackReceiverHoldBytes = SpineBitmapBytes(spine_count);
    if (CustomHeader::ackRouteSpineBytes > 0)
      CustomHeader::ackRouteSpineBytes = SpineFieldBytes(spine_count);
    if (CustomHeader::ackAnswerSeqBytes > 0)
      CustomHeader::ackAnswerSeqBytes = 4 + SpineFieldBytes(spine_count);
    if (CustomHeader::udpAnswerHoldBytes > 0)
      CustomHeader::udpAnswerHoldBytes = SpineBitmapBytes(spine_count);
  }
  // The switches' state, which leaf_hold, receiver_hold and
  // SPRAY_ANSWER_PATH_SEQ read only for the hosts' leaves.
  const Ptr<SwitchQueueState> switch_queues =
      switch_queue_cost() || leaf_hold() || reps_oracle() || receiver_hold() ||
              ack_route_oracle_on() || answer_path_seq()
          ? BuildSwitchQueueState(n)
          : nullptr;
  // One variable for every host, so the draw consumes a single fixed stream.
  Ptr<UniformRandomVariable> path_random;
  if (load_balancing_value() != static_cast<uint32_t>(LoadBalancingMode::Ecmp))
    path_random = CreateObjectWithAttributes<UniformRandomVariable>(
        "Stream", IntegerValue(path_rng_stream));

#if ENABLE_QP
  FILE *fct_output = fopen(fct_output_file.c_str(), "w");
  if (fct_output == nullptr) {
    std::cerr << "Error: cannot open FCT output file\n";
    return false;
  }
  std::cout << "QP is enabled " << std::endl;
  //
  // install RDMA driver
  //
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() == 0) { // is server
      // create RdmaHw
      Ptr<RdmaHw> rdmaHw = CreateObject<RdmaHw>();
      rdmaHw->SetAttribute("ClampTargetRate", BooleanValue(clamp_target_rate));
      rdmaHw->SetAttribute("AlphaResumInterval",
                           DoubleValue(alpha_resume_interval));
      rdmaHw->SetAttribute("RPTimer", DoubleValue(rp_timer));
      rdmaHw->SetAttribute("FastRecoveryTimes",
                           UintegerValue(fast_recovery_times));
      rdmaHw->SetAttribute("EwmaGain", DoubleValue(ewma_gain));
      rdmaHw->SetAttribute("RateAI", DataRateValue(DataRate(rate_ai)));
      rdmaHw->SetAttribute("RateHAI", DataRateValue(DataRate(rate_hai)));
      rdmaHw->SetAttribute("L2BackToZero", BooleanValue(l2_back_to_zero));
      rdmaHw->SetAttribute("L2ChunkSize", UintegerValue(l2_chunk_size));
      rdmaHw->SetAttribute("L2AckInterval", UintegerValue(l2_ack_interval));
      rdmaHw->SetAttribute("RetransmissionTimeoutNs",
               UintegerValue(retransmission_timeout_ns));
      rdmaHw->SetAttribute("MaxRetransmissionRetries",
               UintegerValue(max_retransmission_retries));
      rdmaHw->SetAttribute("NoProgressTimeoutNs",
               UintegerValue(no_progress_timeout_ns));
      rdmaHw->SetAttribute("SelectiveRetransmission",
               BooleanValue(selective_retransmission != 0));
      rdmaHw->SetAttribute("CcMode", UintegerValue(cc_mode));
      rdmaHw->SetAttribute(
          "NsccWindowState",
          UintegerValue(static_cast<uint32_t>(
              nscc_window_state == "connection" ? NsccWindowState::Connection
                                                : NsccWindowState::Message)));
      rdmaHw->SetAttribute(
          "NsccRttSample",
          UintegerValue(static_cast<uint32_t>(nscc_rtt_sample == "karn"
                                                  ? NsccRttSample::Karn
                                                  : NsccRttSample::Any)));
      rdmaHw->SetAttribute("RateDecreaseInterval",
                           DoubleValue(rate_decrease_interval));
      rdmaHw->SetAttribute("MinRate", DataRateValue(DataRate(min_rate)));
      rdmaHw->SetAttribute("Mtu", UintegerValue(packet_payload_size));
      rdmaHw->SetAttribute("MiThresh", UintegerValue(mi_thresh));
      rdmaHw->SetAttribute("VarWin", BooleanValue(var_win));
      rdmaHw->SetAttribute("FastReact", BooleanValue(fast_react));
      rdmaHw->SetAttribute("MultiRate", BooleanValue(multi_rate));
      rdmaHw->SetAttribute("SampleFeedback", BooleanValue(sample_feedback));
      rdmaHw->SetAttribute("TargetUtil", DoubleValue(u_target));
      rdmaHw->SetAttribute("RateBound", BooleanValue(rate_bound));
      rdmaHw->SetAttribute("DctcpRateAI",
                           DataRateValue(DataRate(dctcp_rate_ai)));
      rdmaHw->SetPintSmplThresh(pint_prob);
      rdmaHw->SetAttribute("Forgiveness", BooleanValue(forgiveness));
      rdmaHw->SetAttribute("CongestionExemption",
                           BooleanValue(congestion_exempt));
      rdmaHw->SetAttribute("Reengage", BooleanValue(reengage));
      rdmaHw->SetAttribute("LoadBalancing",
                           UintegerValue(load_balancing_value()));
      rdmaHw->SetAttribute("SpineCount", UintegerValue(spine_count));
      rdmaHw->SetAttribute("PathSelector", UintegerValue(path_selector_value()));
      rdmaHw->SetAttribute("RepsBufferSize", UintegerValue(reps_buffer_size));
      rdmaHw->SetAttribute("RepsFreezingTimeoutNs",
                           UintegerValue(reps_freezing_timeout_ns));
      rdmaHw->SetAttribute(
          "RepsOracleHops",
          UintegerValue(static_cast<uint32_t>(reps_oracle_hops == "forward"
                                                  ? OracleHops::Forward
                                                  : OracleHops::RoundTrip)));
      rdmaHw->SetAttribute("SprayHoldScope",
                           UintegerValue(spray_hold_scope_value()));
      rdmaHw->SetAttribute("SprayHoldAutoLeaves",
                           UintegerValue(spray_hold_auto_leaves));
      rdmaHw->SetAttribute("SprayHoldRule",
                           UintegerValue(spray_hold_rule_value()));
      rdmaHw->SetAttribute("SprayLossP0", DoubleValue(spray_loss_p0));
      rdmaHw->SetAttribute("SprayLossP1", DoubleValue(spray_loss_p1));
      rdmaHw->SetAttribute("SprayHoldAlpha", DoubleValue(spray_hold_alpha));
      rdmaHw->SetAttribute("SprayHoldBeta", DoubleValue(spray_hold_beta));
      rdmaHw->SetAttribute("SprayHoldRelease",
                           UintegerValue(spray_hold_release_value()));
      rdmaHw->SetAttribute("SprayHoldTrickle", DoubleValue(spray_hold_trickle));
      rdmaHw->SetAttribute("SpraySentinelMode",
                           UintegerValue(spray_sentinel_mode_value()));
      rdmaHw->SetAttribute(
          "SprayCopyExpiry",
          UintegerValue(static_cast<uint32_t>(spray_copy_expiry == "deadline"
                                                  ? CopyExpiry::Deadline
                                                  : CopyExpiry::DataAck)));
      rdmaHw->SetAttribute("SprayHeldLossRtts",
                           DoubleValue(spray_held_loss_rtts));
      rdmaHw->SetAttribute("SprayAckLossRtts",
                           DoubleValue(spray_ack_loss_rtts));
      rdmaHw->SetAttribute(
          "AckRoute",
          UintegerValue(static_cast<uint32_t>(
              ack_route_spray()            ? AckRoute::Spray
              : ack_route == "reverse_set" ? AckRoute::ReverseSet
              : ack_route == "repath"      ? AckRoute::Repath
                                           : AckRoute::DataSpine)));
      rdmaHw->SetAttribute("AckRouteSetSize",
                           UintegerValue(ack_route_set_size));
      rdmaHw->SetAttribute("AckRouteOracle",
                           BooleanValue(ack_route_oracle_on()));
      // Made only where answers name their spine, so that data_spine
      // allocates no stream.
      if (ack_route_named())
        rdmaHw->SetAttribute(
            "AckRouteRandomVariable",
            PointerValue(CreateObjectWithAttributes<UniformRandomVariable>(
                "Stream", IntegerValue(device_rng_stream(
                              DeviceRngBlock::AckRoute,
                              n.Get(i)->GetDevice(1))))));
      rdmaHw->SetAttribute(
          "SprayPathState",
          UintegerValue(static_cast<uint32_t>(spray_path_state == "connection"
                                                  ? PathState::Connection
                                                  : PathState::Message)));
      rdmaHw->SetAttribute("SprayPathSeq",
                           BooleanValue(spray_path_seq == "on"));
      rdmaHw->SetAttribute("SprayTailRecovery", BooleanValue(tail_recovery()));
      rdmaHw->SetAttribute("SprayTailRecoveryDelayNs",
                           UintegerValue(spray_tail_recovery_delay_ns_given
                                             ? spray_tail_recovery_delay_ns
                                             : RdmaHw::kTailDelayBaseRtt));
      rdmaHw->SetAttribute("SprayTailLossRtts",
                           DoubleValue(spray_tail_loss_rtts));
      rdmaHw->SetAttribute("SprayReceiverHold", BooleanValue(receiver_hold()));
      rdmaHw->SetAttribute("SprayReceiverHoldClean",
                           UintegerValue(spray_receiver_hold_clean));
      rdmaHw->SetAttribute(
          "SprayReceiverTailMissing",
          BooleanValue(spray_receiver_tail_missing == "on"));
      rdmaHw->SetAttribute("SprayCoverageFirst",
                           BooleanValue(spray_coverage_first == "on"));
      rdmaHw->SetAttribute(
          "SprayDuplicateReport",
          UintegerValue(static_cast<uint32_t>(
              spray_duplicate_report == "retract" ? DuplicateReport::Retract
              : spray_duplicate_report == "on"    ? DuplicateReport::Defer
                                                  : DuplicateReport::Off)));
      rdmaHw->SetAttribute("SprayAnswerPathSeq",
                           BooleanValue(answer_path_seq()));
      rdmaHw->SetAttribute("SprayAnswerProbeIntervalNs",
                           UintegerValue(spray_answer_probe_interval_ns));
      rdmaHw->SetAttribute("SprayAnswerHoldClean",
                           UintegerValue(spray_answer_hold_clean));
      rdmaHw->SetAttribute(
          "SprayStrayPathSeq",
          UintegerValue(static_cast<uint32_t>(spray_stray_path_seq == "closed"
                                                  ? StrayPathSeq::Closed
                                                  : StrayPathSeq::Fresh)));
      rdmaHw->SetAttribute("UeEvSetSize", UintegerValue(ue_ev_set_size));
      rdmaHw->SetAttribute("UeSaturationFraction",
                           DoubleValue(ue_saturation_fraction));
      rdmaHw->SetAttribute("MrcEvSetSize", UintegerValue(mrc_ev_set_size));
      rdmaHw->SetAttribute("MrcSkipBaseRtts", DoubleValue(mrc_skip_base_rtts));
      rdmaHw->SetAttribute("MrcProbeTimeouts", DoubleValue(mrc_probe_timeouts));
      rdmaHw->SetAttribute("AckRedirectOmega", DoubleValue(ack_redirect_omega));
      rdmaHw->SetAttribute("AckRedirectFlag", DoubleValue(ack_redirect_flag));
      rdmaHw->SetAttribute("AckRedirectValidityBaseRtts",
                           DoubleValue(ack_redirect_validity_base_rtts));
      rdmaHw->SetAttribute("AckRedirectTrimScoring",
                           UintegerValue(ack_redirect_trim_scoring_value()));
      rdmaHw->SetAttribute("AckRedirectScope",
                           UintegerValue(grade_scope_value(ack_redirect_scope)));
      if (path_selector == "ack_redirect" || ack_route_named())
        rdmaHw->m_ownLeafHosts = own_leaf_hosts(n.Get(i));
      rdmaHw->SetAttribute("SprayReportIntervalBaseRtts",
                           DoubleValue(spray_report_interval_base_rtts));
      rdmaHw->SetAttribute("SprayEstimatorGain",
                           DoubleValue(spray_estimator_gain));
      rdmaHw->SetAttribute("SprayFractionCusumSlack",
                           DoubleValue(spray_fraction_cusum_slack));
      rdmaHw->SetAttribute("SprayFractionCusumThreshold",
                           DoubleValue(spray_fraction_cusum_threshold));
      rdmaHw->SetAttribute("SprayCongestionThreshold1",
                           DoubleValue(spray_congestion_thresholds[0]));
      rdmaHw->SetAttribute("SprayCongestionThreshold2",
                           DoubleValue(spray_congestion_thresholds[1]));
      rdmaHw->SetAttribute("SprayCongestionThreshold3",
                           DoubleValue(spray_congestion_thresholds[2]));
      rdmaHw->SetAttribute("SprayTrimThreshold1",
                           DoubleValue(spray_trim_thresholds[0]));
      rdmaHw->SetAttribute("SprayTrimThreshold2",
                           DoubleValue(spray_trim_thresholds[1]));
      rdmaHw->SetAttribute("SprayTrimThreshold3",
                           DoubleValue(spray_trim_thresholds[2]));
      rdmaHw->SetAttribute("SprayCongestionGradeFloor",
                           UintegerValue(spray_congestion_grade_floor));
      rdmaHw->SetAttribute("SprayLossThreshold1",
                           DoubleValue(spray_loss_thresholds[0]));
      rdmaHw->SetAttribute("SprayLossThreshold2",
                           DoubleValue(spray_loss_thresholds[1]));
      rdmaHw->SetAttribute("SprayLossThreshold3",
                           DoubleValue(spray_loss_thresholds[2]));
      rdmaHw->SetAttribute("SprayLossIntervalNs",
                           UintegerValue(spray_loss_interval_ns));
      rdmaHw->SetAttribute("SprayLossHoldCleanSentinels",
                           UintegerValue(spray_loss_hold_clean_sentinels));
      rdmaHw->SetAttribute("SprayReleaseFractionOfFloor",
                           DoubleValue(spray_release_fraction_of_floor));
      rdmaHw->SetAttribute("SprayOneWayDelay",
                           BooleanValue(spray_one_way_delay == 1));
      rdmaHw->SetAttribute("SprayDelayCusumSlackBaseRtts",
                           DoubleValue(spray_delay_cusum_slack_base_rtts));
      rdmaHw->SetAttribute("SprayDelayCusumThresholdBaseRtts",
                           DoubleValue(spray_delay_cusum_threshold_base_rtts));
      rdmaHw->SetAttribute("SprayDelayThreshold1BaseRtts",
                           DoubleValue(spray_delay_thresholds_base_rtts[0]));
      rdmaHw->SetAttribute("SprayDelayThreshold2BaseRtts",
                           DoubleValue(spray_delay_thresholds_base_rtts[1]));
      rdmaHw->SetAttribute("SprayDelayThreshold3BaseRtts",
                           DoubleValue(spray_delay_thresholds_base_rtts[2]));
      rdmaHw->SetAttribute("SprayAbsenceFractionOfMedian",
                           DoubleValue(spray_absence_fraction_of_median));
      rdmaHw->SetAttribute("SprayAbsenceMinimumMedian",
                           UintegerValue(spray_absence_minimum_median));
      rdmaHw->SetAttribute("SprayGradeReference",
                           UintegerValue(spray_grade_reference_value()));
      rdmaHw->SetAttribute("SprayGradeScope",
                           UintegerValue(grade_scope_value(spray_grade_scope)));
      rdmaHw->SetAttribute("SprayGradeCostSource",
                           UintegerValue(spray_grade_cost_source_value()));
      if (switch_queues) {
        rdmaHw->m_switchQueues = switch_queues;
        rdmaHw->m_leaf =
            switch_queues->LeafOf(node_id_to_ip(n.Get(i)->GetId()).Get());
      }
      rdmaHw->SetAttribute("SprayEstimatorIntervalSamples",
                           UintegerValue(spray_estimator_interval_samples));
      rdmaHw->SetAttribute("SprayGamma", DoubleValue(spray_gamma));
      rdmaHw->SetAttribute("SprayEpsilon", DoubleValue(spray_epsilon));
      rdmaHw->SetAttribute("SprayCandidates", UintegerValue(spray_candidates));
      rdmaHw->SetAttribute("SprayCandidateDraw",
                           UintegerValue(spray_candidate_draw_value()));
      rdmaHw->SetAttribute("SprayEdgeWindowPenalty",
                           UintegerValue(spray_edge_window_penalty));
      rdmaHw->SetAttribute("SprayEdgeFreeze",
                           BooleanValue(spray_edge_freeze == 1));
      rdmaHw->SetAttribute("SpraySenderPoolWeight",
                           DoubleValue(spray_sender_pool_weight));
      rdmaHw->SetAttribute("SprayPoolDecayIntervalNs",
                           UintegerValue(spray_pool_decay_interval_ns));
      rdmaHw->SetAttribute("SprayPoolHoldPropagates",
                           BooleanValue(spray_pool_hold_propagates == 1));
      rdmaHw->SetAttribute(
          "SprayHeldExploration",
          EnumValue(static_cast<SpineScores::HeldExploration>(
              spray_held_exploration_value())));
      rdmaHw->SetAttribute("SprayHeldSentinelIntervalNs",
                           UintegerValue(spray_held_sentinel_interval_ns));
      // A PointerValue cannot carry null, which is the ECMP default.
      if (path_random)
        rdmaHw->SetAttribute("PathRandomVariable", PointerValue(path_random));
      rdmaHw->m_transportEventCallback =
          MakeCallback(&record_host_transport_event);
      if (recovery_verdict != nullptr)
        rdmaHw->m_recoveryVerdictCallback = MakeCallback(recovery_verdict);
      if (forgiveness_eligible != nullptr)
        rdmaHw->m_forgivenessEligibleCallback =
            MakeCallback(forgiveness_eligible);
      if (remainder_verdict != nullptr)
        rdmaHw->m_remainderVerdictCallback = MakeCallback(remainder_verdict);
      if (data_accepted != nullptr)
        rdmaHw->m_dataAcceptedCallback = MakeCallback(data_accepted);
      if (allowance_gone != nullptr)
        rdmaHw->m_allowanceGoneCallback = MakeCallback(allowance_gone);
      rdmaHw->SetAttribute("TotalPauseTimes",
                           UintegerValue(nic_total_pause_time));
      // create and install RdmaDriver
      Ptr<RdmaDriver> rdma = CreateObject<RdmaDriver>();
      Ptr<Node> node = n.Get(i);
      rdma->SetNode(node);
      rdma->SetRdmaHw(rdmaHw);

      node->AggregateObject(rdma);
      rdma->Init();
      rdma->TraceConnectWithoutContext(
          "QpComplete", MakeBoundCallback(qp_finish, fct_output));
        rdma->TraceConnectWithoutContext(
          "QpFailure", MakeBoundCallback(qp_fail, fct_output));
    }
  }
#endif

  // set ACK priority on hosts
  if (ack_high_prio)
    RdmaEgressQueue::ack_q_idx = 0;
  else
    RdmaEgressQueue::ack_q_idx = 3;

  // setup routing
  CalculateRoutes(n);
  if (pair_bandwidth == "spines")
    SprayPairBandwidth(n);
  SetRoutingEntries();

  //
  // get BDP and delay
  //
  maxRtt = maxBdp = 0;
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() != 0)
      continue;
    for (uint32_t j = 0; j < node_num; j++) {
      if (n.Get(j)->GetNodeType() != 0)
        continue;
      uint64_t delay = pairDelay[n.Get(i)][n.Get(j)];
      uint64_t txDelay = pairTxDelay[n.Get(i)][n.Get(j)];
      uint64_t rtt = delay * 2 + txDelay;
      uint64_t bw = pairBw[i][j];
      uint64_t bdp = rtt * bw / 1000000000 / 8;
      pairBdp[n.Get(i)][n.Get(j)] = bdp;
      pairRtt[i][j] = rtt;
      if (bdp > maxBdp)
        maxBdp = bdp;
      if (rtt > maxRtt)
        maxRtt = rtt;
    }
  }
  printf("maxRtt=%lu maxBdp=%lu\n", maxRtt, maxBdp);
  // Known only now that every route is, and read from the first packet on.
  if (load_balancing == "spray_policy" || path_selector == "ack_redirect" ||
      reps_oracle() || tail_recovery() || ack_route_oracle_on() ||
      ack_route == "reverse_set") {
    for (uint32_t i = 0; i < node_num; i++)
      if (n.Get(i)->GetNodeType() == 0)
        n.Get(i)->GetObject<RdmaDriver>()->m_rdma->SetAttribute(
            "SprayBaseRttNs", UintegerValue(maxRtt));
  }

  //
  // setup switch CC
  //
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() == 1) { // switch
      Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(n.Get(i));
      sw->SetAttribute("CcMode", UintegerValue(cc_mode));
      sw->SetAttribute("MaxRtt", UintegerValue(maxRtt));
    }
  }

  //
  // add trace
  //

  NodeContainer trace_nodes;
  for (uint32_t i = 0; i < trace_num; i++) {
    uint32_t nid;
    tracef >> nid;
    if (nid >= n.GetN()) {
      continue;
    }
    trace_nodes = NodeContainer(trace_nodes, n.Get(nid));
  }

  FILE *trace_output = fopen(trace_output_file.c_str(), "w");
  if (enable_trace)
    qbb.EnableTracing(trace_output, trace_nodes);

  // dump link speed to trace file
  {
    SimSetting sim_setting;
    for (auto i : nbr2if) {
      for (auto j : i.second) {
        uint16_t node = i.first->GetId();
        uint8_t intf = j.second.idx;
        uint64_t bps =
            DynamicCast<QbbNetDevice>(i.first->GetDevice(j.second.idx))
                ->GetDataRate()
                .GetBitRate();
        sim_setting.port_speed[node][intf] = bps;
      }
    }
    sim_setting.win = maxBdp;
    sim_setting.Serialize(trace_output);
  }

  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  NS_LOG_INFO("Create Applications.");

  Time interPacketInterval = Seconds(0.0000005 / 2);
  // maintain port number for each host
  for (uint32_t i = 0; i < node_num; i++) {
    if (n.Get(i)->GetNodeType() == 0)
      for (uint32_t j = 0; j < node_num; j++) {
        if (n.Get(j)->GetNodeType() == 0)
          portNumber[i][j] = 10000; // each host pair use port number from 10000
      }
  }
  flow_input.idx = -1;

  topof.close();
  tracef.close();

  // schedule link down
  if (link_down_time > 0) {
    Simulator::Schedule(Seconds(2) + MicroSeconds(link_down_time),
                        &TakeDownLink, n, n.Get(link_down_A),
                        n.Get(link_down_B));
  }
  if (!port_counter_output_file.empty()) {
    port_counter_file = fopen(port_counter_output_file.c_str(), "w");
    if (port_counter_file == nullptr) {
      std::cerr << "Error: cannot open " << port_counter_output_file << "\n";
      return false;
    }
    fprintf(port_counter_file, "time_ns,switch,port,neighbour,tx_packets,"
                               "tx_bytes,data_packets,ecn_marks\n");
  }
  if (!spine_arrival_output_file.empty()) {
    spine_arrival_file = fopen(spine_arrival_output_file.c_str(), "w");
    if (spine_arrival_file == nullptr) {
      std::cerr << "Error: cannot open " << spine_arrival_output_file << "\n";
      return false;
    }
    fprintf(spine_arrival_file,
            "time_ns,rank,spine,packets,payload_bytes,folded_packets\n");
  }
  if (!spine_report_output_file.empty()) {
    spine_report_file = fopen(spine_report_output_file.c_str(), "w");
    if (spine_report_file == nullptr) {
      std::cerr << "Error: cannot open " << spine_report_output_file << "\n";
      return false;
    }
    fprintf(spine_report_file,
            "time_ns,rank,sequence,spine,arrivals,marked,trimmed,moved,"
            "mark_fraction,trim_fraction,delay_ns,held,grade,last_hop_trims,"
            "edge%s\n",
            spray_grade_scope == "per_connection" ? ",sender,sender_port"
            : switch_queue_cost()                 ? ",source_leaf,loss_fraction"
                                                  : "");
    for (uint32_t i = 0; i < node_num; i++)
      if (n.Get(i)->GetNodeType() == 0) {
        Ptr<RdmaHw> hw = n.Get(i)->GetObject<RdmaDriver>()->m_rdma;
        if (switch_queue_cost())
          hw->TraceConnectWithoutContext(
              "SwitchQueueReport",
              MakeBoundCallback(&write_switch_queue_report, i));
        else
          hw->TraceConnectWithoutContext(
              "SpineReport", MakeBoundCallback(&write_spine_report, i));
      }
  }
  if (!spine_loss_output_file.empty()) {
    spine_loss_file = fopen(spine_loss_output_file.c_str(), "w");
    if (spine_loss_file == nullptr) {
      std::cerr << "Error: cannot open " << spine_loss_output_file << "\n";
      return false;
    }
    fprintf(spine_loss_file, "time_ns,rank,%sspine,sent,lost,grade\n",
            pair_hold()   ? "destination,"
            : leaf_hold() ? "destination_leaf,"
                          : "");
    for (uint32_t i = 0; i < node_num; i++)
      if (n.Get(i)->GetNodeType() == 0) {
        Ptr<RdmaHw> hw = n.Get(i)->GetObject<RdmaDriver>()->m_rdma;
        if (pair_hold())
          hw->TraceConnectWithoutContext(
              "LeafSpineLossInterval",
              MakeBoundCallback(&write_pair_spine_loss_interval, i));
        else if (leaf_hold())
          hw->TraceConnectWithoutContext(
              "LeafSpineLossInterval",
              MakeBoundCallback(&write_leaf_spine_loss_interval, i));
        else
          hw->TraceConnectWithoutContext(
              "SpineLossInterval",
              MakeBoundCallback(&write_spine_loss_interval, i));
      }
  }
  if (!rto_event_output_file.empty()) {
    rto_event_file = fopen(rto_event_output_file.c_str(), "w");
    if (rto_event_file == nullptr) {
      std::cerr << "Error: cannot open " << rto_event_output_file << "\n";
      return false;
    }
    fprintf(rto_event_file,
            "time_ns,rank,dst_rank,spine,sentinel,bytes_outstanding\n");
    for (uint32_t i = 0; i < node_num; i++)
      if (n.Get(i)->GetNodeType() == 0)
        n.Get(i)->GetObject<RdmaDriver>()->m_rdma->TraceConnectWithoutContext(
            "RetransmissionTimeout", MakeBoundCallback(&write_rto_event, i));
  }
  if (!answer_hold_output_file.empty()) {
    answer_hold_file = fopen(answer_hold_output_file.c_str(), "w");
    if (answer_hold_file == nullptr) {
      std::cerr << "Error: cannot open " << answer_hold_output_file << "\n";
      return false;
    }
    fprintf(answer_hold_file, "time_ns,side,rank,leaf,spine,held\n");
    for (uint32_t i = 0; i < node_num; i++)
      if (n.Get(i)->GetNodeType() == 0)
        n.Get(i)->GetObject<RdmaDriver>()->m_rdma->TraceConnectWithoutContext(
            "AnswerHold", MakeBoundCallback(&write_answer_hold, i));
  }
  if (!receiver_hold_output_file.empty()) {
    receiver_hold_file = fopen(receiver_hold_output_file.c_str(), "w");
    if (receiver_hold_file == nullptr) {
      std::cerr << "Error: cannot open " << receiver_hold_output_file << "\n";
      return false;
    }
    fprintf(receiver_hold_file,
            "time_ns,rank,sender_leaf,spine,arrivals,lost,hold\n");
    for (uint32_t i = 0; i < node_num; i++)
      if (n.Get(i)->GetNodeType() == 0)
        n.Get(i)->GetObject<RdmaDriver>()->m_rdma->TraceConnectWithoutContext(
            "ReceiverHoldInterval",
            MakeBoundCallback(&write_receiver_hold_interval, i));
  }
  std::set<uint64_t> event_times;
  for (const LinkEvent &event : link_events) {
    const char *key =
        event.kind == LinkEventKind::Restore ? "LINK_RESTORE" : "LINK_FAILURE";
    if (event.a >= node_num || event.b >= node_num ||
        nbr2if[n.Get(event.a)].count(n.Get(event.b)) == 0) {
      std::cerr << key << " names " << event.a << " " << event.b
                << ", which no link joins\n";
      return false;
    }
    if (event.kind == LinkEventKind::Blackhole &&
        n.Get(event.b)->GetNodeType() != 1) {
      std::cerr << "LINK_FAILURE blackhole needs a switch as its second node\n";
      return false;
    }
    // The headers are sized for the run by now.
    if (event.unit == RateErrorModel::ERROR_UNIT_BYTE &&
        event.frameBytes != RdmaHw::DataFrameBytes(packet_payload_size)) {
      std::cerr << "LINK_FAILURE loss byte derives its rate at a data frame of "
                << event.frameBytes << " bytes, and this run's is "
                << RdmaHw::DataFrameBytes(packet_payload_size) << "\n";
      return false;
    }
    event_times.insert(event.timeNs);
  }
  // Scheduled first, so that each set of counters is what stood before the
  // failures and restores at its time.
  for (uint64_t time_ns : event_times)
    Simulator::Schedule(NanoSeconds(time_ns), &write_fabric_counters);
  // In order, which events at one time keep, a restore before a failure.
  for (const LinkEvent &event : link_events)
    Simulator::Schedule(NanoSeconds(event.timeNs), &ApplyLinkEvent, n, event);

  // schedule buffer monitor
  FILE *qlen_output = fopen(qlen_mon_file.c_str(), "w");
  Simulator::Schedule(NanoSeconds(qlen_mon_start), &monitor_buffer, qlen_output,
                      &n);

  return true;
}
