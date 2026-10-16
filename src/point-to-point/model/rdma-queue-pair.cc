#include <ns3/hash.h>
#include <ns3/uinteger.h>
#include <ns3/seq-ts-header.h>
#include <ns3/udp-header.h>
#include <ns3/ipv4-header.h>
#include <ns3/simulator.h>
#include "ns3/ppp-header.h"
#include "rdma-queue-pair.h"
#include "load-balancing.h"
#include <algorithm>
#include <iterator>

namespace ns3 {

/**************************
 * OutstandingPackets
 *************************/
OutstandingPackets::OutstandingPackets()
	: m_packet_size(0), m_first(0), m_oldest(kNone), m_newest(kNone),
	  m_highest_sent(kNone), m_bytes(0)
{
}

void OutstandingPackets::SetPacketSize(uint32_t size){
	m_packet_size = size;
}

bool OutstandingPackets::IsKept() const{
	return m_packet_size != 0;
}

OutstandingPackets::Record &OutstandingPackets::At(uint64_t packet){
	return m_ring[packet & (m_ring.size() - 1)];
}

const OutstandingPackets::Record &OutstandingPackets::At(uint64_t packet) const{
	return m_ring[packet & (m_ring.size() - 1)];
}

void OutstandingPackets::Reserve(uint64_t packet){
	const uint64_t span = packet - m_first + 1;
	if (span <= m_ring.size())
		return;
	uint64_t size = m_ring.empty() ? 1 : m_ring.size();
	while (size < span)
		size *= 2;
	std::vector<Record> ring(size);
	for (uint64_t p = m_first; p < m_first + m_ring.size(); p++)
		ring[p & (size - 1)] = At(p);
	m_ring.swap(ring);
}

void OutstandingPackets::Add(uint64_t seq, uint32_t size, uint16_t path,
		uint64_t sentNs, bool trickle, bool exploration, uint16_t pathSeq,
		bool held){
	NS_ASSERT_MSG(m_packet_size != 0 && seq % m_packet_size == 0,
		"a send record is indexed by its packet number");
	const uint64_t packet = seq / m_packet_size;
	NS_ASSERT_MSG(packet >= m_first, "a packet below the cumulative "
		"acknowledgement is never sent");
	Reserve(packet);
	Record &record = At(packet);
	NS_ASSERT_MSG(!record.outstanding,
		"a packet is resent only after its previous send is removed");
	auto newestOnPath = m_newest_on_path.find(path);
	const uint64_t olderOnPath = newestOnPath == m_newest_on_path.end()
		? kNone : newestOnPath->second;
	const bool resendsLost = m_lost.erase(packet) > 0;
	const bool sentBefore = m_highest_sent != kNone && packet <= m_highest_sent;
	const uint8_t sends = sentBefore
		? static_cast<uint8_t>(std::min<uint32_t>(record.sends + 1u, UINT8_MAX)) : 1;
	if (!sentBefore)
		m_highest_sent = packet;
	record = {sentNs, size, path, pathSeq, true, resendsLost, trickle,
		trickle || exploration, held, m_newest, kNone, olderOnPath, kNone, sends};
	if (m_newest != kNone)
		At(m_newest).newer = packet;
	else
		m_oldest = packet;
	m_newest = packet;
	if (olderOnPath != kNone){
		At(olderOnPath).newer_on_path = packet;
		newestOnPath->second = packet;
	}else{
		m_newest_on_path.emplace(path, packet);
	}
	m_bytes += size;
}

uint64_t OutstandingPackets::Find(uint64_t seq, uint16_t path) const{
	if (m_ring.empty())
		return kNone;
	const uint64_t packet = seq / m_packet_size;
	if (packet < m_first || packet - m_first >= m_ring.size())
		return kNone;
	const Record &record = At(packet);
	return record.outstanding && record.path == path ? packet : kNone;
}

uint64_t OutstandingPackets::Oldest() const{
	return m_oldest;
}

uint64_t OutstandingPackets::OlderOnPath(uint64_t packet) const{
	return At(packet).older_on_path;
}

uint64_t OutstandingPackets::Seq(uint64_t packet) const{
	return packet * m_packet_size;
}

uint32_t OutstandingPackets::Size(uint64_t packet) const{
	return At(packet).size;
}

uint64_t OutstandingPackets::SentNs(uint64_t packet) const{
	return At(packet).sent_ns;
}

uint16_t OutstandingPackets::Path(uint64_t packet) const{
	return At(packet).path;
}

uint16_t OutstandingPackets::PathSeq(uint64_t packet) const{
	return At(packet).path_seq;
}

bool OutstandingPackets::ResendsLost(uint64_t packet) const{
	return At(packet).resends_lost;
}

uint8_t OutstandingPackets::Sends(uint64_t packet) const{
	return At(packet).sends;
}

void OutstandingPackets::NoteCopy(uint64_t seq){
	const uint64_t packet = seq / m_packet_size;
	if (m_ring.empty() || packet < m_first || packet - m_first >= m_ring.size() ||
			!At(packet).outstanding)
		return;
	Record &record = At(packet);
	record.sends = static_cast<uint8_t>(std::min<uint32_t>(record.sends + 1u, UINT8_MAX));
}

bool OutstandingPackets::Trickle(uint64_t packet) const{
	return At(packet).trickle;
}

bool OutstandingPackets::Exploration(uint64_t packet) const{
	return At(packet).exploration;
}

bool OutstandingPackets::Held(uint64_t packet) const{
	return At(packet).held;
}

uint64_t OutstandingPackets::NewestOnPath(uint16_t path) const{
	const auto newest = m_newest_on_path.find(path);
	return newest == m_newest_on_path.end() ? kNone : newest->second;
}

void OutstandingPackets::Remove(uint64_t packet){
	Record &record = At(packet);
	if (record.older != kNone)
		At(record.older).newer = record.newer;
	else
		m_oldest = record.newer;
	if (record.newer != kNone)
		At(record.newer).older = record.older;
	else
		m_newest = record.older;
	if (record.older_on_path != kNone)
		At(record.older_on_path).newer_on_path = record.newer_on_path;
	if (record.newer_on_path != kNone)
		At(record.newer_on_path).older_on_path = record.older_on_path;
	else if (record.older_on_path != kNone)
		m_newest_on_path[record.path] = record.older_on_path;
	else
		m_newest_on_path.erase(record.path);
	record.outstanding = false;
	m_bytes -= record.size;
}

void OutstandingPackets::RemoveLost(uint64_t packet){
	Remove(packet);
	m_lost.insert(packet);
}

bool OutstandingPackets::RepairsAgain(uint64_t seq){
	return !m_repaired.insert(seq / m_packet_size).second;
}

void OutstandingPackets::RemoveBelow(uint64_t seq,
		std::vector<Unanswered> *unanswered){
	// Rounded up so that the flow's last packet, the only one shorter than the
	// packet size, is removed once the acknowledgement covers it.
	const uint64_t end = (seq + m_packet_size - 1) / m_packet_size;
	m_lost.erase(m_lost.begin(), m_lost.lower_bound(end));
	m_repaired.erase(m_repaired.begin(), m_repaired.lower_bound(end));
	for (; m_first < end; m_first++){
		if (m_ring.empty() || !At(m_first).outstanding)
			continue;
		if (unanswered){
			const Record &record = At(m_first);
			unanswered->push_back(Unanswered{Seq(m_first),
				record.path, record.path_seq, record.trickle,
				record.resends_lost});
		}
		Remove(m_first);
	}
}

uint64_t OutstandingPackets::Bytes() const{
	return m_bytes;
}

/**************************
 * RdmaQueuePair
 *************************/
TypeId RdmaQueuePair::GetTypeId (void)
{
static TypeId tid = TypeId ("ns3::RdmaQueuePair")
		.SetParent<Object> ()
		;
	return tid;
}

RdmaQueuePair::RdmaQueuePair(uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport, uint16_t _dport){
	startTime = Simulator::Now();
	sip = _sip;
	dip = _dip;
	sport = _sport;
	dport = _dport;
	m_size = 0;
	m_init_size = 0;
	m_src = -1;
	m_dest = -1;
	m_tag = -1;
	snd_nxt = snd_una = 0;
	m_highest_sent = 0;
	m_data_attempted_bytes = 0;
	m_retransmitted_bytes = 0;
	m_trimmed_payload_bytes = 0;
	m_recovery_events = 0;
	m_trim_notifications = 0;
	m_trim_lasthop_notifications = 0;
	m_trim_recovery_events = 0;
	m_stale_trim_notifications = 0;
	m_recovery_retries = 0;
	m_duplicate_repairs = 0;
	m_tail_declared_lost = 0;
	m_held_declared_lost = 0;
	m_overdueTaken = false;
	m_overdueSeq = 0;
	m_overduePath = 0;
	m_overdueSentNs = 0;
	m_overdueSpent = false;
	m_overdue_declared_lost = 0;
	m_overdue_spurious = 0;
	m_overdueProbe = false;
	m_timeout_covered_losses = 0;
	m_ack_loss_declared = 0;
	m_tail_nacks_ignored = 0;
	m_copyDeadlineRtts = 0;
	m_coverageFirst = false;
	m_answerCoverage = 0;
	m_covered_answer_losses = 0;
	m_deferTimeoutEvidence = false;
	m_retractTimeoutEvidence = false;
	m_retracted_timeout_losses = 0;
	m_duplicate_answer_losses = 0;
	m_timeout_data_losses = 0;
	m_exploration_copies = 0;
	m_exploration_copies_lost = 0;
	m_timeouts = 0;
	m_cnp_received = 0;
	m_cc_exempt = false;
	m_cc_signals_withheld = 0;
	m_allowance_gone_reports = 0;
	m_cc_exempt_granted_ns = 0;
	m_cc_transitions = 0;
	m_cc_obeying_ns = 0;
	m_cc_obey_since_ns = 0;
	m_cc_report_seen = false;
	m_cc_last_report = false;
	m_first_trim_ns = 0;
	m_first_repair_ns = 0;
	m_last_progress_ns = Simulator::Now().GetNanoSeconds();
	m_failure_reason = 0;
	m_failed = false;
	m_pg = pg;
	m_ipid = 0;
	m_win = 0;
	m_baseRtt = 0;
	m_max_rate = 0;
	m_var_win = false;
	m_rate = 0;
	m_nextAvail = Time(0);
	mlx.m_alpha = 1;
	mlx.m_alpha_cnp_arrived = false;
	mlx.m_first_cnp = true;
	mlx.m_decrease_cnp_arrived = false;
	mlx.m_rpTimeStage = 0;
	hp.m_lastUpdateSeq = 0;
	for (uint32_t i = 0; i < sizeof(hp.keep) / sizeof(hp.keep[0]); i++)
		hp.keep[i] = 0;
	hp.m_incStage = 0;
	hp.m_lastGap = 0;
	hp.u = 1;
	for (uint32_t i = 0; i < IntHeader::maxHop; i++){
		hp.hopState[i].u = 1;
		hp.hopState[i].incStage = 0;
	}

	tmly.m_lastUpdateSeq = 0;
	tmly.m_incStage = 0;
	tmly.lastRtt = 0;
	tmly.rttDiff = 0;

	dctcp.m_lastUpdateSeq = 0;
	dctcp.m_caState = 0;
	dctcp.m_highSeq = 0;
	dctcp.m_alpha = 1;
	dctcp.m_ecnCnt = 0;
	dctcp.m_batchSizeOfAlpha = 0;

	hpccPint.m_lastUpdateSeq = 0;
	hpccPint.m_incStage = 0;
}

void RdmaQueuePair::SetSize(uint64_t size){
	m_size = size;
}

void RdmaQueuePair::SetSrc(uint32_t src){
	m_src = src;
}

void RdmaQueuePair::SetDest(uint32_t dest){
	m_dest = dest;
}

uint32_t RdmaQueuePair::GetSrc(){
	return m_src;
}

uint32_t RdmaQueuePair::GetDest(){
	return m_dest;
}

void RdmaQueuePair::SetTag(uint64_t tag){
	m_tag = tag;
}

uint64_t RdmaQueuePair::GetTag(){
	return m_tag;
}

void RdmaQueuePair::SetInitialSize(uint64_t size){
	m_init_size = size;
}

uint64_t RdmaQueuePair::GetInitialSize(){
	return m_init_size;
}

void RdmaQueuePair::SetWin(uint32_t win){
	m_win = win;
}

void RdmaQueuePair::SetBaseRtt(uint64_t baseRtt){
	m_baseRtt = baseRtt;
}

void RdmaQueuePair::SetVarWin(bool v){
	m_var_win = v;
}

void RdmaQueuePair::SetAppNotifyCallback(Callback<void> notifyAppFinish){
	m_notifyAppFinish = notifyAppFinish;
}

void RdmaQueuePair::SetAppSentCallback(Callback<void> notifyAppSent){
	m_notifyAppSent = notifyAppSent;
}


uint64_t RdmaQueuePair::GetBytesLeft(){
	// Pending selective repairs count as sendable bytes: a queue pair whose
	// tail is fully transmitted must stay schedulable until its repair
	// ranges have been resent. This runs inside the egress queue's per-packet
	// scan over every registered queue pair, so the overwhelmingly common
	// no-repairs case must stay one comparison — RepairBytesLeft() walks and
	// prunes the range map and is only entered when ranges exist, which
	// requires selective retransmission to be enabled and active.
	uint64_t tail = m_size >= snd_nxt ? m_size - snd_nxt : 0;
	if (m_repair_ranges.empty())
		return tail;
	return tail + RepairBytesLeft();
}

void RdmaQueuePair::AddRepairRange(uint64_t start, uint64_t end){
	if (start < snd_una)
		start = snd_una;
	if (end > m_size)
		end = m_size;
	if (start >= end)
		return;
	// Merge with any overlapping or adjacent recorded ranges.
	auto it = m_repair_ranges.lower_bound(start);
	if (it != m_repair_ranges.begin()){
		auto prev = std::prev(it);
		if (prev->second >= start){
			start = prev->first;
			if (prev->second > end)
				end = prev->second;
			m_repair_ranges.erase(prev);
		}
	}
	it = m_repair_ranges.lower_bound(start);
	while (it != m_repair_ranges.end() && it->first <= end){
		if (it->second > end)
			end = it->second;
		it = m_repair_ranges.erase(it);
	}
	m_repair_ranges[start] = end;
}

uint64_t RdmaQueuePair::TakeRepairSegment(uint64_t max_bytes, uint64_t &start){
	DropAcknowledgedRepairs();
	if (m_repair_ranges.empty() || max_bytes == 0)
		return 0;
	auto it = m_repair_ranges.begin();
	start = it->first;
	uint64_t size = it->second - it->first;
	if (size > max_bytes)
		size = max_bytes;
	uint64_t new_start = start + size;
	uint64_t end = it->second;
	m_repair_ranges.erase(it);
	if (new_start < end)
		m_repair_ranges[new_start] = end;
	return size;
}

void RdmaQueuePair::DropAcknowledgedRepairs(){
	while (!m_repair_ranges.empty()){
		auto it = m_repair_ranges.begin();
		if (it->second <= snd_una){
			m_repair_ranges.erase(it);
			continue;
		}
		if (it->first < snd_una){
			uint64_t end = it->second;
			m_repair_ranges.erase(it);
			m_repair_ranges[snd_una] = end;
		}
		break;
	}
}

uint64_t RdmaQueuePair::AcknowledgePacket(uint64_t seq, uint16_t path){
	const uint64_t packet = m_outstanding.Find(seq, path);
	if (packet == OutstandingPackets::kNone)
		return 0;
	// Sends along one path pass through one sequence of queues and arrive in
	// the order they left, so an older send on this path that is still
	// outstanding did not arrive. Under EntropyHash two outstanding sends
	// rarely share a path, so this seldom finds one and loss falls to the
	// timeout; only equal values are known to share queues.
	uint64_t lost = 0;
	for (uint64_t older = m_outstanding.OlderOnPath(packet);
			older != OutstandingPackets::kNone;
			older = m_outstanding.OlderOnPath(packet)){
		// Delivered, as its coverage shows: the answer is what was lost.
		if (CoveredAnswerLoss(m_outstanding.Seq(older), m_outstanding.Size(older)))
			m_outstanding.Remove(older);
		else
			lost += DeclareLost(older);
	}
	const uint64_t nowNs = Simulator::Now().GetNanoSeconds();
	// The answer of a resend may be that of the send declared lost before
	// it, so only a first send's answer is told as a trickle send's.
	if (m_outstanding.Trickle(packet) && !m_outstanding.ResendsLost(packet))
		m_pathSelector->OnTrickleAnswer(m_outstanding.Path(packet), false,
			nowNs);
	if (!m_copies.empty()){
		const auto copy = m_copies.find(seq);
		if (copy != m_copies.end())
			copy->second.dataAcknowledged = true;
		DeclareCopiesLostOnPath(path, m_outstanding.SentNs(packet), nowNs);
	}
	m_outstanding.Remove(packet);
	return lost;
}

void RdmaQueuePair::AddCopy(uint64_t seq, uint32_t size, uint16_t path,
		uint64_t sentNs, bool trickle){
	m_copies[seq] = ExplorationCopy{size, path, sentNs, trickle, false};
	m_exploration_copies++;
	m_outstanding.NoteCopy(seq);
}

bool RdmaQueuePair::AcknowledgeCopy(uint64_t seq, uint16_t path,
		uint64_t &lost){
	lost = 0;
	const auto copy = m_copies.find(seq);
	// A data send of seq along the same path, a repair, takes the answer.
	if (copy == m_copies.end() || copy->second.path != path ||
			m_outstanding.Find(seq, path) != OutstandingPackets::kNone)
		return false;
	const uint64_t sentNs = copy->second.sentNs;
	const bool trickle = copy->second.trickle;
	m_copies.erase(copy);
	const uint64_t nowNs = Simulator::Now().GetNanoSeconds();
	// The older data sends along the copy's path did not arrive.
	for (uint64_t older = m_outstanding.NewestOnPath(path);
			older != OutstandingPackets::kNone;){
		const uint64_t next = m_outstanding.OlderOnPath(older);
		if (m_outstanding.SentNs(older) < sentNs){
			if (CoveredAnswerLoss(m_outstanding.Seq(older),
					m_outstanding.Size(older)))
				m_outstanding.Remove(older);
			else
				lost += DeclareLost(older);
		}
		older = next;
	}
	if (trickle)
		m_pathSelector->OnTrickleAnswer(path, false, nowNs);
	DeclareCopiesLostOnPath(path, sentNs, nowNs);
	return true;
}

void RdmaQueuePair::DeclareCopiesLostOnPath(uint16_t path, uint64_t sentNs,
		uint64_t nowNs){
	for (auto copy = m_copies.begin(); copy != m_copies.end();){
		const auto next = std::next(copy);
		if (copy->second.path == path && copy->second.sentNs < sentNs)
			DeclareCopyLost(copy, nowNs);
		copy = next;
	}
	ExpireCopies(nowNs);
}

void RdmaQueuePair::ExpireCopies(uint64_t nowNs){
	const uint64_t deadlineNs =
		static_cast<uint64_t>(m_copyDeadlineRtts * m_baseRtt);
	for (auto copy = m_copies.begin(); copy != m_copies.end();){
		const auto next = std::next(copy);
		const bool acknowledged = copy->second.dataAcknowledged ||
			copy->first + copy->second.size <= snd_una;
		const bool expired = m_copyDeadlineRtts > 0
			? nowNs > copy->second.sentNs + deadlineNs
			: acknowledged && nowNs > copy->second.sentNs + m_baseRtt;
		if (expired)
			DeclareCopyLost(copy, nowNs);
		copy = next;
	}
}

void RdmaQueuePair::DeclareCopyLost(
		std::map<uint64_t, ExplorationCopy>::iterator copy, uint64_t nowNs){
	const uint16_t path = copy->second.path;
	const bool trickle = copy->second.trickle;
	// Its sequence delivered, by its data send's answer or cumulatively.
	if (m_coverageFirst && (copy->second.dataAcknowledged ||
			copy->first + copy->second.size <=
				std::max<uint64_t>(snd_una, m_answerCoverage))){
		CountCoveredAnswerLoss();
		m_copies.erase(copy);
		return;
	}
	m_copies.erase(copy);
	m_exploration_copies_lost++;
	m_pathSelector->OnLoss(path, nowNs);
	if (trickle)
		m_pathSelector->OnTrickleAnswer(path, true, nowNs);
}

bool RdmaQueuePair::ReleasePacket(uint64_t seq, uint16_t path){
	const uint64_t packet = m_outstanding.Find(seq, path);
	if (packet == OutstandingPackets::kNone)
		return false;
	m_outstanding.Remove(packet);
	return true;
}

uint64_t RdmaQueuePair::DeclareLostSentBy(uint64_t sentNs){
	uint64_t lost = 0;
	for (uint64_t oldest = m_outstanding.Oldest();
			oldest != OutstandingPackets::kNone &&
				m_outstanding.SentNs(oldest) <= sentNs;
			oldest = m_outstanding.Oldest())
		lost += DeclareTimedOutSend(oldest);
	return lost;
}

uint64_t RdmaQueuePair::DeclareTimedOutSend(uint64_t packet){
	if (m_deferTimeoutEvidence){
		// The first send's evidence waits for its resend's answer.
		m_timedOut.emplace(m_outstanding.Seq(packet), TimedOutSend{
			m_outstanding.Path(packet), m_outstanding.Trickle(packet)});
		return DeclareLost(packet, false);
	}
	// Told at once, and kept until its resend's answer shows whether to take
	// it back.
	if (m_retractTimeoutEvidence)
		m_timeoutLosses.emplace(m_outstanding.Seq(packet), TimeoutLoss{
			m_outstanding.Path(packet),
			static_cast<uint64_t>(Simulator::Now().GetNanoSeconds())});
	return DeclareLost(packet);
}

uint64_t RdmaQueuePair::DeclareOverdueSendLost(uint64_t seq, uint16_t path,
		uint64_t sentNs){
	const uint64_t packet = m_outstanding.Find(seq, path);
	if (packet == OutstandingPackets::kNone ||
			m_outstanding.SentNs(packet) != sentNs)
		return 0;
	if (CoveredAnswerLoss(seq, m_outstanding.Size(packet))){
		m_outstanding.Remove(packet);
		return 0;
	}
	m_overdue_declared_lost++;
	m_overdueRepaired[seq] =
		OverdueRepair{path, false, 0, m_outstanding.Trickle(packet)};
	// A probe is recovery alone.
	if (m_overdueProbe)
		return DeclareLost(packet, false);
	return DeclareTimedOutSend(packet);
}

void RdmaQueuePair::CountTimedOutCovered(uint32_t losses){
	m_timeout_covered_losses += losses;
	if (!m_timedOutCovered.IsNull())
		for (uint32_t k = 0; k < losses; k++)
			m_timedOutCovered();
}

void RdmaQueuePair::NoteOverdueRepairSent(uint64_t seq, uint16_t path){
	const auto repair = m_overdueRepaired.find(seq);
	if (repair == m_overdueRepaired.end() || repair->second.repairSent)
		return;
	repair->second.repairSent = true;
	repair->second.repairPath = path;
}

bool RdmaQueuePair::OverdueRepairSpurious(uint64_t seq, uint16_t path,
		bool duplicate, uint64_t nowNs){
	const auto repair = m_overdueRepaired.find(seq);
	if (repair == m_overdueRepaired.end())
		return false;
	const OverdueRepair r = repair->second;
	const bool send = path == r.path && !(r.repairSent && r.repairPath == path);
	const bool resend = r.repairSent && path == r.repairPath && path != r.path;
	if (!duplicate && !send && !resend)
		return false;
	m_overdueRepaired.erase(repair);
	if (m_overdueProbe && (m_deferTimeoutEvidence || m_retractTimeoutEvidence)){
		if (duplicate){
			// Only the answer was lost.
			m_duplicate_answer_losses++;
			if (!m_timedOutResolved.IsNull())
				m_timedOutResolved(true);
		}else if (resend){
			// The repair arrived first: the send was lost.
			m_pathSelector->OnLoss(r.path, nowNs);
			if (r.trickle)
				m_pathSelector->OnTrickleAnswer(r.path, true, nowNs);
			m_timeout_data_losses++;
			if (!m_timedOutResolved.IsNull())
				m_timedOutResolved(false);
		}
	}
	if (resend && !duplicate)
		return false;
	m_overdue_spurious++;
	return true;
}

void RdmaQueuePair::ResolveTimedOut(uint64_t seq, bool duplicate,
		uint16_t route, uint64_t nowNs){
	const auto losses = m_timeoutLosses.equal_range(seq);
	for (auto loss = losses.first; loss != losses.second; ++loss){
		if (!duplicate){
			m_timeout_data_losses++;
			if (!m_timedOutResolved.IsNull())
				m_timedOutResolved(false);
			continue;
		}
		const bool released = m_pathSelector->RetractLoss(loss->second.path,
			loss->second.countedNs, nowNs);
		m_retracted_timeout_losses++;
		m_duplicate_answer_losses++;
		if (!m_timeoutLossRetracted.IsNull())
			m_timeoutLossRetracted(route, released);
	}
	m_timeoutLosses.erase(losses.first, losses.second);
	const auto send = m_timedOut.find(seq);
	if (send == m_timedOut.end())
		return;
	if (duplicate){
		m_duplicate_answer_losses++;
	}else{
		m_pathSelector->OnLoss(send->second.path, nowNs);
		if (send->second.trickle)
			m_pathSelector->OnTrickleAnswer(send->second.path, true, nowNs);
		m_timeout_data_losses++;
	}
	m_timedOut.erase(send);
	if (!m_timedOutResolved.IsNull())
		m_timedOutResolved(duplicate);
}

uint64_t RdmaQueuePair::DeclareLostOnPathSeqs(uint16_t path, uint16_t first,
		uint16_t count, std::vector<DeclaredSend> *deferred){
	std::vector<uint64_t> named;
	for (uint64_t send = m_outstanding.NewestOnPath(path);
			send != OutstandingPackets::kNone;
			send = m_outstanding.OlderOnPath(send)){
		const uint16_t offset =
			static_cast<uint16_t>(m_outstanding.PathSeq(send) - first);
		if (offset < count)
			named.push_back(send);
	}
	uint64_t lost = 0;
	// Oldest first, as the per-path rule declares them.
	for (auto send = named.rbegin(); send != named.rend(); ++send){
		if (CoveredAnswerLoss(m_outstanding.Seq(*send),
				m_outstanding.Size(*send))){
			m_outstanding.Remove(*send);
			continue;
		}
		if (deferred)
			deferred->push_back(DeclaredSend{m_outstanding.Seq(*send),
				m_outstanding.Path(*send), m_outstanding.Trickle(*send)});
		lost += DeclareLost(*send, !deferred);
	}
	m_tail_declared_lost += named.size();
	if (named.empty())
		m_tail_nacks_ignored++;
	return lost;
}

uint64_t RdmaQueuePair::DeclareHeldSendLost(uint64_t seq, uint16_t path,
		uint64_t sentNs){
	const uint64_t packet = m_outstanding.Find(seq, path);
	if (packet == OutstandingPackets::kNone ||
			m_outstanding.SentNs(packet) != sentNs || !m_outstanding.Held(packet))
		return 0;
	if (CoveredAnswerLoss(seq, m_outstanding.Size(packet))){
		m_outstanding.Remove(packet);
		return 0;
	}
	m_held_declared_lost++;
	return DeclareLost(packet);
}

bool RdmaQueuePair::AnswerCovered(uint64_t seq, uint16_t path){
	for (auto covered = m_covered.begin(); covered != m_covered.end();
			++covered){
		if (covered->send.seq != seq || covered->send.path != path)
			continue;
		if (covered->send.trickle && !covered->send.resendsLost)
			m_pathSelector->OnTrickleAnswer(path, false,
				Simulator::Now().GetNanoSeconds());
		m_covered.erase(covered);
		return true;
	}
	return false;
}

uint32_t RdmaQueuePair::DeclareAckLosses(uint64_t nowNs, uint64_t delayNs){
	uint32_t declared = 0;
	// Covered in order, and each due delayNs after it was covered.
	while (!m_covered.empty() &&
			m_covered.front().coveredNs + delayNs <= nowNs){
		const OutstandingPackets::Unanswered send = m_covered.front().send;
		m_covered.pop_front();
		// Covered, so delivered: an answer loss and no loss on its path.
		if (m_coverageFirst){
			CountCoveredAnswerLoss();
			continue;
		}
		m_pathSelector->OnLoss(send.path, nowNs);
		if (send.trickle)
			m_pathSelector->OnTrickleAnswer(send.path, true, nowNs);
		declared++;
	}
	m_ack_loss_declared += declared;
	return declared;
}

void RdmaQueuePair::CountCoveredAnswerLoss(){
	m_covered_answer_losses++;
	if (!m_coveredAnswerLoss.IsNull())
		m_coveredAnswerLoss();
}

bool RdmaQueuePair::CoveredAnswerLoss(uint64_t seq, uint64_t size){
	if (!m_coverageFirst ||
			seq + size > std::max<uint64_t>(snd_una, m_answerCoverage))
		return false;
	CountCoveredAnswerLoss();
	return true;
}

uint32_t RdmaQueuePair::DeclareLost(uint64_t packet, bool report){
	const uint64_t seq = m_outstanding.Seq(packet);
	const uint32_t size = m_outstanding.Size(packet);
	if (report){
		m_pathSelector->OnLoss(m_outstanding.Path(packet),
			Simulator::Now().GetNanoSeconds());
		if (m_outstanding.Trickle(packet))
			m_pathSelector->OnTrickleAnswer(m_outstanding.Path(packet), true,
				Simulator::Now().GetNanoSeconds());
	}
	AddRepairRange(seq, seq + size);
	m_outstanding.RemoveLost(packet);
	m_recovery_events++;
	return size;
}

uint64_t RdmaQueuePair::RepairBytesLeft(){
	DropAcknowledgedRepairs();
	uint64_t total = 0;
	for (auto const &range : m_repair_ranges)
		total += range.second - range.first;
	return total;
}

uint32_t RdmaQueuePair::GetHash(void){
	union{
		struct {
			uint32_t sip, dip;
			uint16_t sport, dport;
		};
		char c[12];
	} buf;
	buf.sip = sip.Get();
	buf.dip = dip.Get();
	buf.sport = sport;
	buf.dport = dport;
	return Hash32(buf.c, 12);
}

void RdmaQueuePair::Acknowledge(uint64_t ack, bool keepCovered){
	if (ack > snd_una){
		snd_una = ack;
		// A timed-out sequence acknowledged with no answer to its resend.
		while (!m_timedOut.empty() && m_timedOut.begin()->first < snd_una){
			CountTimedOutCovered(1);
			ResolveTimedOut(m_timedOut.begin()->first, false, kNoRouteSpine,
				Simulator::Now().GetNanoSeconds());
		}
		while (!m_timeoutLosses.empty() &&
				m_timeoutLosses.begin()->first < snd_una){
			CountTimedOutCovered(
				m_timeoutLosses.count(m_timeoutLosses.begin()->first));
			ResolveTimedOut(m_timeoutLosses.begin()->first, false,
				kNoRouteSpine, Simulator::Now().GetNanoSeconds());
		}
		// A probed sequence covered before its repair's answer counts nothing.
		if (m_overdueProbe)
			m_overdueRepaired.erase(m_overdueRepaired.begin(),
				m_overdueRepaired.lower_bound(snd_una));
		if (m_outstanding.IsKept() && keepCovered){
			std::vector<OutstandingPackets::Unanswered> unanswered;
			m_outstanding.RemoveBelow(snd_una, &unanswered);
			const uint64_t nowNs = Simulator::Now().GetNanoSeconds();
			for (const OutstandingPackets::Unanswered &send : unanswered)
				m_covered.push_back(CoveredSend{send, nowNs});
		}else if (m_outstanding.IsKept()){
			m_outstanding.RemoveBelow(snd_una);
		}
		// A cumulative ACK can outrun a go-back-N rewind: resent duplicates
		// make the receiver repeat its frontier ACK, which lands above the
		// rewound snd_nxt. Unclamped, GetOnTheFly() underflows and the window
		// check blocks the queue pair from ever sending again.
		if (snd_nxt < snd_una){
			snd_nxt = snd_una;
		}
	}
}

uint64_t RdmaQueuePair::GetOnTheFly(){
	// With send records the window counts the sends still outstanding, so a
	// hole at the cumulative acknowledgement does not hold back new data while
	// the sends above it are acknowledged.
	if (m_outstanding.IsKept())
		return m_outstanding.Bytes();
	return snd_nxt - snd_una;
}

bool RdmaQueuePair::IsWinBound(){
	uint64_t w = GetWin();
	return w != 0 && WindowOnTheFly() >= w;
}

NsccWindow &RdmaQueuePair::Nscc(){
	return m_nsccConnection ? m_nsccConnection->window : nscc;
}

uint64_t RdmaQueuePair::WindowOnTheFly(){
	if (!m_nsccConnection)
		return GetOnTheFly();
	uint64_t bytes = 0;
	for (const Ptr<RdmaQueuePair> &qp : m_nsccConnection->queuePairs)
		bytes += qp->GetOnTheFly();
	return bytes;
}

uint64_t RdmaQueuePair::GetWin(){
	if (m_win == 0)
		return 0;
	uint64_t w;
	if (m_var_win){
		w = m_win * m_rate.GetBitRate() / m_max_rate.GetBitRate();
		if (w == 0)
			w = 1; // must > 0
	}else{
		w = m_win;
	}
	return w;
}

uint64_t RdmaQueuePair::HpGetCurWin(){
	if (m_win == 0)
		return 0;
	uint64_t w;
	if (m_var_win){
		w = m_win * hp.m_curRate.GetBitRate() / m_max_rate.GetBitRate();
		if (w == 0)
			w = 1; // must > 0
	}else{
		w = m_win;
	}
	return w;
}

bool RdmaQueuePair::IsFinished(){
	return !m_failed && snd_una >= m_size;
}

bool RdmaQueuePair::IsFailed(){
	return m_failed;
}

/*********************
 * RdmaRxQueuePair
 ********************/
TypeId RdmaRxQueuePair::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RdmaRxQueuePair")
		.SetParent<Object> ()
		;
	return tid;
}

RdmaRxQueuePair::RdmaRxQueuePair(){
	sip = dip = sport = dport = 0;
	m_ipid = 0;
	ReceiverNextExpectedSeq = 0;
	m_nackTimer = Time(0);
	m_milestone_rx = 0;
	m_lastNACK = 0;
	m_forgiveness_eligible = false;
	m_highest_seen_end = 0;
	m_data_arrivals = 0;
	m_folded_arrivals = 0;
	m_countedTailMissing = false;
	m_stray = false;
}

uint32_t RdmaRxQueuePair::GetHash(void){
	union{
		struct {
			uint32_t sip, dip;
			uint16_t sport, dport;
		};
		char c[12];
	} buf;
	buf.sip = sip;
	buf.dip = dip;
	buf.sport = sport;
	buf.dport = dport;
	return Hash32(buf.c, 12);
}

namespace {

// Merge [start, end) into a disjoint range set. Two sets are kept on a receive
// queue pair, the accepted out-of-order bytes and the forgiven ones, and they
// are merged and measured by the same algebra, so they share one
// implementation rather than two that could drift.
void AddRange(std::map<uint64_t, uint64_t> &ranges, uint64_t start,
		uint64_t end){
	if (start >= end)
		return;
	auto it = ranges.lower_bound(start);
	if (it != ranges.begin()){
		auto prev = std::prev(it);
		if (prev->second >= start){
			start = prev->first;
			if (prev->second > end)
				end = prev->second;
			ranges.erase(prev);
		}
	}
	it = ranges.lower_bound(start);
	while (it != ranges.end() && it->first <= end){
		if (it->second > end)
			end = it->second;
		it = ranges.erase(it);
	}
	ranges[start] = end;
}

// Bytes of [start, end) the set covers. AddRange merges every overlapping and
// touching range, so the entries are disjoint and no byte is counted twice.
// The scan starts at the last entry beginning at or below start, the only one
// that can reach into the range from the left.
uint64_t CoveredBytes(const std::map<uint64_t, uint64_t> &ranges,
		uint64_t start, uint64_t end){
	if (start >= end)
		return 0;
	uint64_t covered = 0;
	auto it = ranges.upper_bound(start);
	if (it != ranges.begin())
		--it;
	for (; it != ranges.end() && it->first < end; ++it){
		const uint64_t overlap_start = std::max(it->first, start);
		const uint64_t overlap_end = std::min(it->second, end);
		if (overlap_end > overlap_start)
			covered += overlap_end - overlap_start;
	}
	return covered;
}

}  // namespace

void RdmaRxQueuePair::AddOutOfOrderRange(uint64_t start, uint64_t end){
	AddRange(m_ooo_ranges, start, end);
}

void RdmaRxQueuePair::NoteForgiven(uint64_t start, uint64_t end){
	const uint64_t expected = static_cast<uint64_t>(ReceiverNextExpectedSeq);
	if (start < expected)
		start = expected;
	uint64_t cursor = start;
	auto it = m_ooo_ranges.upper_bound(start);
	if (it != m_ooo_ranges.begin())
		--it;
	for (; it != m_ooo_ranges.end() && it->first < end && cursor < end; ++it){
		if (it->second <= cursor)
			continue;
		if (it->first > cursor)
			AddRange(m_forgiven_ranges, cursor, std::min(it->first, end));
		cursor = std::max(cursor, it->second);
	}
	if (cursor < end)
		AddRange(m_forgiven_ranges, cursor, end);
}

uint64_t RdmaRxQueuePair::ForgivenBytes(uint64_t start, uint64_t end) const{
	return CoveredBytes(m_forgiven_ranges, start, end);
}

void RdmaRxQueuePair::NoteSeen(uint64_t end){
	if (end > m_highest_seen_end)
		m_highest_seen_end = end;
}

uint64_t RdmaRxQueuePair::Holes() const{
	return UnsettledBytes(static_cast<uint64_t>(ReceiverNextExpectedSeq),
		m_highest_seen_end);
}

uint64_t RdmaRxQueuePair::AbsorbContiguousFrom(uint64_t expected){
	auto it = m_ooo_ranges.begin();
	while (it != m_ooo_ranges.end() && it->first <= expected){
		if (it->second > expected)
			expected = it->second;
		it = m_ooo_ranges.erase(it);
	}
	return expected;
}

uint64_t RdmaRxQueuePair::UnsettledBytes(uint64_t start, uint64_t end) const{
	// Clip below: everything under the cumulative sequence is delivered, so a
	// trim straddling it names fewer new bytes than its length.
	const uint64_t expected = static_cast<uint64_t>(ReceiverNextExpectedSeq);
	if (start < expected)
		start = expected;
	if (start >= end)
		return 0;
	return (end - start) - CoveredBytes(m_ooo_ranges, start, end);
}

uint64_t RdmaRxQueuePair::AcceptedBytesAbove(uint64_t expected) const{
	uint64_t accepted = 0;
	// The same scan UnsettledBytes runs, open-ended above: the entries are
	// disjoint, and the one entry beginning at or below `expected` is the only
	// one that can reach into the range from the left.
	auto it = m_ooo_ranges.upper_bound(expected);
	if (it != m_ooo_ranges.begin())
		--it;
	for (; it != m_ooo_ranges.end(); ++it){
		const uint64_t start = std::max(it->first, expected);
		if (it->second > start)
			accepted += it->second - start;
	}
	return accepted;
}

/*********************
 * RdmaQueuePairGroup
 ********************/
TypeId RdmaQueuePairGroup::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RdmaQueuePairGroup")
		.SetParent<Object> ()
		;
	return tid;
}

RdmaQueuePairGroup::RdmaQueuePairGroup(void){
}

uint32_t RdmaQueuePairGroup::GetN(void){
	return m_qps.size();
}

Ptr<RdmaQueuePair> RdmaQueuePairGroup::Get(uint32_t idx){
	return m_qps[idx];
}

Ptr<RdmaQueuePair> RdmaQueuePairGroup::operator[](uint32_t idx){
	return m_qps[idx];
}

void RdmaQueuePairGroup::AddQp(Ptr<RdmaQueuePair> qp){
	m_qps.push_back(qp);
}

#if 0
void RdmaQueuePairGroup::AddRxQp(Ptr<RdmaRxQueuePair> rxQp){
	m_rxQps.push_back(rxQp);
}
#endif

void RdmaQueuePairGroup::Clear(void){
	m_qps.clear();
}

}
