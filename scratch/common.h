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
#include <ns3/load-balancing.h>
#include <ns3/path-selector.h>
#include <ns3/rdma-client-helper.h>
#include <ns3/rdma-client.h>
#include <ns3/rdma-driver.h>
#include <ns3/rdma.h>
#include <ns3/sim-setting.h>
#include <ns3/switch-node.h>
#include <time.h>
#include <unordered_map>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("GENERIC_SIMULATION");

uint32_t cc_mode = 1;
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
FILE *port_counter_file = nullptr, *spine_arrival_file = nullptr;

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
// and the parameters of the selectors, each the default of its source.
std::string path_selector = "ops";
uint32_t reps_buffer_size = 8;
uint64_t reps_freezing_timeout_ns = 10000000;
// The selectors each parameter belongs to. A parameter given under another
// selector would be read by nothing, so it is refused.
const std::map<std::string, std::set<std::string>> path_selector_parameter_owners = {
    {"REPS_BUFFER_SIZE", {"reps"}},
    {"REPS_FREEZING_TIMEOUT_NS", {"reps"}},
};
std::set<std::string> path_selector_parameters_given;
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

// What befalls the link between two nodes at a time in the run. Down takes the
// link down and reroutes around it. Blackhole leaves it up while the second
// node drops the data that arrives from the first. Loss and Rate change the
// link's error rate or its speed in both directions.
enum class LinkFailureKind { Down, Blackhole, Loss, Rate };
struct LinkFailure {
  uint64_t start_ns;
  LinkFailureKind kind;
  uint32_t a, b;
  double error_rate;
  DataRate rate;
};
vector<LinkFailure> link_failures;

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
  } else if (protocol == 0xFC || protocol == 0xFD ||
             protocol == kUecTrimRepairProtocol ||
             protocol == kUecTrimNotificationProtocol) {
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
          ? (ch.udp.payload_size > CustomHeader::GetUdpHeaderSize()
                 ? ch.udp.payload_size - CustomHeader::GetUdpHeaderSize()
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
  return std::numeric_limits<uint32_t>::max();
}

uint32_t load_balancing_value() {
  if (load_balancing == "ecmp")
    return static_cast<uint32_t>(LoadBalancingMode::Ecmp);
  if (load_balancing == "ev_hash")
    return static_cast<uint32_t>(LoadBalancingMode::EntropyHash);
  if (load_balancing == "spray_uniform")
    return static_cast<uint32_t>(LoadBalancingMode::SprayUniform);
  return std::numeric_limits<uint32_t>::max();
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
uint32_t AssignSpinePorts(NodeContainer &n) {
  vector<Ptr<Node>> leaves, spines;
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

// take down the link between a and b, and redo the routing
void TakeDownLink(NodeContainer n, Ptr<Node> a, Ptr<Node> b) {
  if (!nbr2if[a][b].up)
    return;
  // take down link between a and b
  nbr2if[a][b].up = nbr2if[b][a].up = false;
  nextHop.clear();
  CalculateRoutes(n);
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
  for (FILE *file : {port_counter_file, spine_arrival_file})
    if (file != nullptr)
      fflush(file);
}

void ApplyLinkFailure(NodeContainer n, LinkFailure failure) {
  Ptr<Node> a = n.Get(failure.a), b = n.Get(failure.b);
  Ptr<QbbNetDevice> device_at_a =
      DynamicCast<QbbNetDevice>(a->GetDevice(nbr2if[a][b].idx));
  Ptr<QbbNetDevice> device_at_b =
      DynamicCast<QbbNetDevice>(b->GetDevice(nbr2if[b][a].idx));
  switch (failure.kind) {
  case LinkFailureKind::Down:
    TakeDownLink(n, a, b);
    break;
  case LinkFailureKind::Blackhole:
    DynamicCast<SwitchNode>(b)->SetBlackhole(nbr2if[b][a].idx);
    break;
  case LinkFailureKind::Loss:
    set_link_error_rate(device_at_a, failure.error_rate);
    set_link_error_rate(device_at_b, failure.error_rate);
    break;
  case LinkFailureKind::Rate:
    device_at_a->SetDataRate(failure.rate);
    device_at_b->SetDataRate(failure.rate);
    break;
  }
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
    } else if (key.compare("LINK_DOWN") == 0) {
      conf >> link_down_time >> link_down_A >> link_down_B;
    } else if (key.compare("LINK_FAILURE") == 0) {
      LinkFailure failure = {};
      std::string kind;
      conf >> failure.start_ns >> kind >> failure.a >> failure.b;
      if (kind == "down") {
        failure.kind = LinkFailureKind::Down;
      } else if (kind == "blackhole") {
        failure.kind = LinkFailureKind::Blackhole;
      } else if (kind == "loss") {
        failure.kind = LinkFailureKind::Loss;
        conf >> failure.error_rate;
        if (!std::isfinite(failure.error_rate) || failure.error_rate <= 0.0 ||
            failure.error_rate > 1.0) {
          std::cerr << "LINK_FAILURE loss needs an error rate in (0, 1]\n";
          return false;
        }
      } else if (kind == "rate") {
        failure.kind = LinkFailureKind::Rate;
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
      link_failures.push_back(failure);
    } else if (key.compare("ENABLE_TRACE") == 0) {
      conf >> enable_trace;
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
    std::cerr << "LOAD_BALANCING must be ecmp, ev_hash, or spray_uniform\n";
    return false;
  }
  if (path_selector_value() == std::numeric_limits<uint32_t>::max()) {
    std::cerr << "PATH_SELECTOR must be ops or reps\n";
    return false;
  }
  if (path_selector != "ops" &&
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
  if (!spine_arrival_output_file.empty() &&
      load_balancing_value() !=
          static_cast<uint32_t>(LoadBalancingMode::SprayUniform)) {
    std::cerr << "SPINE_ARRIVAL_OUTPUT_FILE requires LOAD_BALANCING "
                 "spray_uniform\n";
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
  else // others, no extra header
    IntHeader::mode = IntHeader::NONE;
  // Outside ECMP the sender matches each acknowledgement to the data packet
  // that triggered it; ECMP keeps the header at its old size.
  CustomHeader::ackCarriesPacketSeq =
      load_balancing_value() != static_cast<uint32_t>(LoadBalancingMode::Ecmp);

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
      sw->m_mmu->node_id = sw->GetId();
    }
  }

  uint32_t spine_count = 0;
  if (load_balancing_value() ==
      static_cast<uint32_t>(LoadBalancingMode::SprayUniform)) {
    spine_count = AssignSpinePorts(n);
    if (spine_count == 0) {
      std::cerr << "LOAD_BALANCING spray_uniform needs a leaf-spine fabric in "
                   "which every leaf links to every spine\n";
      return false;
    }
  }
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
  std::set<uint64_t> failure_times;
  for (const LinkFailure &failure : link_failures) {
    if (failure.a >= node_num || failure.b >= node_num ||
        nbr2if[n.Get(failure.a)].count(n.Get(failure.b)) == 0) {
      std::cerr << "LINK_FAILURE names " << failure.a << " " << failure.b
                << ", which no link joins\n";
      return false;
    }
    if (failure.kind == LinkFailureKind::Blackhole &&
        n.Get(failure.b)->GetNodeType() != 1) {
      std::cerr << "LINK_FAILURE blackhole needs a switch as its second node\n";
      return false;
    }
    failure_times.insert(failure.start_ns);
  }
  // Scheduled first, so that each set of counters is what stood before the
  // failures at its time.
  for (uint64_t start_ns : failure_times)
    Simulator::Schedule(NanoSeconds(start_ns), &write_fabric_counters);
  for (const LinkFailure &failure : link_failures)
    Simulator::Schedule(NanoSeconds(failure.start_ns), &ApplyLinkFailure, n,
                        failure);

  // schedule buffer monitor
  FILE *qlen_output = fopen(qlen_mon_file.c_str(), "w");
  Simulator::Schedule(NanoSeconds(qlen_mon_start), &monitor_buffer, qlen_output,
                      &n);

  return true;
}
