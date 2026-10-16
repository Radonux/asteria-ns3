#ifndef RDMA_QUEUE_PAIR_H
#define RDMA_QUEUE_PAIR_H

#include <ns3/object.h>
#include <ns3/packet.h>
#include <ns3/ipv4-address.h>
#include <ns3/data-rate.h>
#include <ns3/event-id.h>
#include <ns3/custom-header.h>
#include <ns3/int-header.h>
#include "nscc-window.h"
#include "path-selector.h"
#include "spine-grader.h"
#include "spine-redirect.h"
#include "spine-sequence.h"
#include "answer-path-seq.h"
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

namespace ns3 {

// The data packets a queue pair has sent and has neither seen acknowledged nor
// declared lost, one record per send. A record is found by its packet number,
// the sequence divided by the packet size, in a ring that grows to the span
// from the cumulative acknowledgement to the newest send and is never
// reallocated once it covers that span. Two doubly linked lists thread the
// records in send order: one across every path, headed by the oldest send,
// and one per path, so that a walk from a send towards older sends on its path
// visits nothing else. Adding, finding and removing a record is O(1), the
// newest send per path being kept in a hash map; empty until SetPacketSize is
// called.
class OutstandingPackets {
public:
	static constexpr uint64_t kNone = UINT64_MAX;
	OutstandingPackets();
	void SetPacketSize(uint32_t size);
	bool IsKept() const;
	// Every sequence is a multiple of the packet size, and a packet is resent
	// only after its previous send has been removed. trickle marks a trickle
	// send of a loss hold (PathSelector::ChoseTrickle), whose answer is told
	// to PathSelector::OnTrickleAnswer; exploration marks any exploration
	// send onto a held spine (PathSelector::ChoseExploration).
	// pathSeq is the send's path sequence under SprayPathSeq. held marks a
	// send up a spine a loss hold held as it was chosen
	// (PathSelector::ChoseHeld).
	void Add(uint64_t seq, uint32_t size, uint16_t path, uint64_t sentNs,
		bool trickle = false, bool exploration = false, uint16_t pathSeq = 0,
		bool held = false);
	// The packet number of the outstanding send of seq along path, or kNone.
	uint64_t Find(uint64_t seq, uint16_t path) const;
	uint64_t Oldest() const;
	uint64_t OlderOnPath(uint64_t packet) const;
	uint64_t Seq(uint64_t packet) const;
	uint32_t Size(uint64_t packet) const;
	uint64_t SentNs(uint64_t packet) const;
	uint16_t Path(uint64_t packet) const;
	uint16_t PathSeq(uint64_t packet) const;
	// Whether the send resends a packet whose previous send was declared
	// lost. A send declared lost may still arrive, and the answer to it would
	// name the resend's record.
	bool ResendsLost(uint64_t packet) const;
	// The packet's sends since its first, this one and its copies included
	// (NoteCopy), at most 255: above 1, an answer to it may be any of them's.
	uint8_t Sends(uint64_t packet) const;
	// A copy of the outstanding send of seq went out too.
	void NoteCopy(uint64_t seq);
	bool Trickle(uint64_t packet) const;
	bool Exploration(uint64_t packet) const;
	bool Held(uint64_t packet) const;
	// The newest outstanding send along path, or kNone.
	uint64_t NewestOnPath(uint16_t path) const;
	void Remove(uint64_t packet);
	// Remove the record of a send declared lost.
	void RemoveLost(uint64_t packet);
	// Note a repair send of seq. True when seq's packet was repaired before,
	// so that this send repeats a repair.
	bool RepairsAgain(uint64_t seq);
	// A send whose record RemoveBelow removed: neither acknowledged by its
	// own acknowledgement nor declared lost.
	struct Unanswered {
		uint64_t seq;
		uint16_t path;
		uint16_t pathSeq;
		bool trickle;
		bool resendsLost;
	};
	// Remove the record of every packet that starts below seq, and append to
	// unanswered, unless it is null, the sends so removed in send order.
	void RemoveBelow(uint64_t seq, std::vector<Unanswered> *unanswered = nullptr);
	uint64_t Bytes() const;

private:
	struct Record {
		uint64_t sent_ns;
		uint32_t size;
		uint16_t path;
		uint16_t path_seq;
		bool outstanding;
		bool resends_lost;
		bool trickle;
		bool exploration;
		bool held;
		// Neighbouring outstanding sends as packet numbers, kNone at either
		// end: across every path, then along this record's path.
		uint64_t older, newer;
		uint64_t older_on_path, newer_on_path;
		uint8_t sends;
	};
	Record &At(uint64_t packet);
	const Record &At(uint64_t packet) const;
	void Reserve(uint64_t packet);

	uint32_t m_packet_size;
	std::vector<Record> m_ring; // a power of two in size
	uint64_t m_first; // every packet below it has been removed
	uint64_t m_oldest, m_newest;
	// The highest packet number ever sent, kNone before the first send: a
	// slot from m_first up to it keeps its packet's record once removed, so
	// its count of sends carries over to the packet's next send.
	uint64_t m_highest_sent;
	std::unordered_map<uint16_t, uint64_t> m_newest_on_path;
	uint64_t m_bytes;
	// The packets whose last send was declared lost and that have not been
	// sent since.
	std::set<uint64_t> m_lost;
	// The packets repaired at least once.
	std::set<uint64_t> m_repaired;
};

struct NsccConnection;

class RdmaQueuePair : public Object {
public:
	Time startTime;
	Ipv4Address sip, dip;
	uint16_t sport, dport;
	uint64_t m_size, m_init_size, m_tag;
	uint32_t m_src, m_dest;
	uint64_t snd_nxt, snd_una; // next seq to send, the highest unacked seq
	uint64_t m_highest_sent;
	uint64_t m_data_attempted_bytes;
	uint64_t m_retransmitted_bytes;
	uint64_t m_trimmed_payload_bytes;
	uint32_t m_recovery_events;
	uint32_t m_trim_notifications;
	uint32_t m_trim_lasthop_notifications;
	uint32_t m_trim_recovery_events;
	uint32_t m_stale_trim_notifications;
	uint32_t m_recovery_retries;
	// Repair sends of a packet already repaired once, kept where every data
	// packet has a send record.
	uint32_t m_duplicate_repairs;
	// Cumulative retransmission-timeout firings. m_recovery_retries resets on
	// every acknowledgement advance, so it cannot answer how often the sender
	// waited out a timeout over the life of the transfer.
	uint32_t m_timeouts;
	// Rate cuts taken. Only CC mode 1 (DCQCN) reacts, so this is zero in every
	// other mode and separates a CC-driven tail from a repair-driven one.
	uint32_t m_cnp_received;
	// Congestion response as a two-variant sum {obey, exempt}. A bool carries
	// it because the value is read on the congestion path: it follows the
	// latest report from the receiver, set while the budget has room and
	// cleared while it has none. An exempt queue pair pays for congestion in
	// bounded loss instead of rate.
	bool m_cc_exempt;
	// Congestion signals withheld from the controller while exempt, and
	// reports that the entry had no allowance left.
	uint32_t m_cc_signals_withheld;
	uint32_t m_allowance_gone_reports;
	// When the receiver granted this queue pair its exemption. Zero means
	// never: no report can arrive before the first send.
	uint64_t m_cc_exempt_granted_ns;
	// What the exemption cost over the transfer's life, now that it follows
	// the report both ways: how many reports changed the bit, and how much
	// simulated time the sender spent delivering signals to its controller
	// after it had been granted. A flow that never saw a set bit spends none.
	uint32_t m_cc_transitions;
	uint64_t m_cc_obeying_ns;
	// When the current stretch of obeying began, and the last report's bit
	// with whether one has been seen at all. Zero means the queue pair is not
	// obeying: no report can arrive at time zero.
	uint64_t m_cc_obey_since_ns;
	bool m_cc_report_seen;
	bool m_cc_last_report;
	// Simulated times of the first trim notification received and the first
	// repair packet sent. Zero means never: no packet can be trimmed or
	// repaired before the transfer's first send.
	uint64_t m_first_trim_ns;
	uint64_t m_first_repair_ns;
	// Simulated time of the last cumulative-acknowledgement advance (or of
	// queue-pair creation). The forward-progress deadline measures from here.
	uint64_t m_last_progress_ns;
	uint32_t m_failure_reason;
	bool m_failed;
	// Selective repair: merged byte ranges awaiting retransmission, always
	// clamped above snd_una. GetNxtPacket serves these before new data.
	std::map<uint64_t, uint64_t> m_repair_ranges;
	// Kept only where every data packet draws its own path, because there a
	// hole below the cumulative acknowledgement says nothing about the sends
	// above it.
	OutstandingPackets m_outstanding;
	// What draws those paths, set together with the send records.
	std::unique_ptr<PathSelector> m_pathSelector;
	// Under SentinelMode::Duplicate, the copies of data packets sent onto a
	// held spine (PathSelector::TakeCopy) not yet answered, by sequence,
	// apart from the send records: a copy is in no window, no retransmission
	// timeout waits for it and its loss queues no repair. A copy ends in one
	// of three ways. Its own acknowledgement, found by its sequence and
	// path, answers it clean; the selector hears of it as of any answer,
	// the window not at all. The per-path rule declares it lost as a send along its
	// path: the acknowledgement of a newer send along that path, data or
	// copy, finds it outstanding; and its own acknowledgement declares lost
	// the older data sends and copies along its path as a data
	// acknowledgement does. With m_copyDeadlineRtts zero (CopyExpiry::DataAck),
	// once the data send of its sequence is acknowledged, by its own answer or
	// cumulatively, a copy outstanding for more than one base RTT since it was
	// sent is declared lost, and one younger waits for its own answer. With
	// it above zero (CopyExpiry::Deadline), a copy outstanding for more than
	// that many base RTTs since it was sent is declared lost, whether its data
	// send is acknowledged or not, and one younger waits for its own answer
	// whatever its data's acknowledgement says. Either is checked at the
	// queue pair's acknowledgements. A declared loss is told to the
	// selector's OnLoss, and of a trickle copy to OnTrickleAnswer, and
	// nothing else.
	struct ExplorationCopy {
		uint32_t size;
		uint16_t path;
		uint64_t sentNs;
		bool trickle;
		bool dataAcknowledged;
	};
	std::map<uint64_t, ExplorationCopy> m_copies;
	double m_copyDeadlineRtts;
	uint32_t m_exploration_copies;
	uint32_t m_exploration_copies_lost;
	// A copy of the send of seq along path, at sentNs. A copy of seq still
	// outstanding is released without a loss.
	void AddCopy(uint64_t seq, uint32_t size, uint16_t path, uint64_t sentNs,
		bool trickle);
	// Whether the acknowledgement of seq along path answers a copy rather
	// than a data send; if so the answer is taken, and lost is set to the
	// data bytes the per-path rule declared lost.
	bool AcknowledgeCopy(uint64_t seq, uint16_t path, uint64_t &lost);
	// Under SprayPathSeq, the path sequence each data packet carries.
	SpineSequenceSender m_pathSeqSender;
	EventId m_retransmissionTimer;
	uint16_t m_pg;
	uint16_t m_ipid;
	uint32_t m_win; // bound of on-the-fly packets
	uint64_t m_baseRtt; // base RTT of this qp
	DataRate m_max_rate; // max rate
	bool m_var_win; // variable window size
	Time m_nextAvail;	//< Soonest time of next send
	uint32_t wp; // current window of packets
	uint32_t lastPktSize;
	Callback<void> m_notifyAppFinish;
	Callback<void> m_notifyAppSent;
	/******************************
	 * runtime states
	 *****************************/
	DataRate m_rate;	//< Current rate
	struct {
		DataRate m_targetRate;	//< Target rate
		EventId m_eventUpdateAlpha;
		double m_alpha;
		bool m_alpha_cnp_arrived; // indicate if CNP arrived in the last slot
		bool m_first_cnp; // indicate if the current CNP is the first CNP
		EventId m_eventDecreaseRate;
		bool m_decrease_cnp_arrived; // indicate if CNP arrived in the last slot
		uint32_t m_rpTimeStage;
		EventId m_rpTimer;
	} mlx;
	struct {
		uint32_t m_lastUpdateSeq;
		DataRate m_curRate;
		IntHop hop[IntHeader::maxHop];
		uint32_t keep[IntHeader::maxHop];
		uint32_t m_incStage;
		double m_lastGap;
		double u;
		struct {
			double u;
			DataRate Rc;
			uint32_t incStage;
		}hopState[IntHeader::maxHop];
	} hp;
	struct{
		uint32_t m_lastUpdateSeq;
		DataRate m_curRate;
		uint32_t m_incStage;
		uint64_t lastRtt;
		double rttDiff;
	} tmly;
	struct{
		uint32_t m_lastUpdateSeq;
		uint32_t m_caState;
		uint32_t m_highSeq; // when to exit cwr
		double m_alpha;
		uint32_t m_ecnCnt;
		uint32_t m_batchSizeOfAlpha;
	} dctcp;
	struct{
		uint32_t m_lastUpdateSeq;
		DataRate m_curRate;
		uint32_t m_incStage;
	}hpccPint;
	NsccWindow nscc;
	// Under NsccWindowState connection, the window of the queue pair's
	// connection, which NSCC moves in place of nscc.
	std::shared_ptr<NsccConnection> m_nsccConnection;
	// The window NSCC moves for the queue pair: nscc, or its connection's.
	NsccWindow &Nscc();
	// The bytes in flight its window bounds: its own, or under a connection's
	// window those of every queue pair open on it.
	uint64_t WindowOnTheFly();

	/***********
	 * methods
	 **********/
	static TypeId GetTypeId (void);
	RdmaQueuePair(uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport, uint16_t _dport);
	void SetSize(uint64_t size);
	void SetWin(uint32_t win);
	void SetBaseRtt(uint64_t baseRtt);
	void SetVarWin(bool v);
	void SetAppNotifyCallback(Callback<void> notifyAppFinish);
	void SetAppSentCallback(Callback<void> notifyAppSent);
	void AddRepairRange(uint64_t start, uint64_t end);
	uint64_t TakeRepairSegment(uint64_t max_bytes, uint64_t &start);
	void DropAcknowledgedRepairs();
	uint64_t RepairBytesLeft();
	// The acknowledgement of the send of seq along path: its record is
	// removed and every older send still outstanding along the same path is
	// declared lost. Nothing happens when no outstanding send matches, which
	// is a duplicate or the answer to an earlier send of a packet resent since.
	// Returns the bytes declared lost.
	uint64_t AcknowledgePacket(uint64_t seq, uint16_t path);
	// A trimmed send of seq along path. False when no outstanding send matches:
	// that send was already declared lost and its repair is under way.
	bool ReleasePacket(uint64_t seq, uint16_t path);
	// Declare lost every outstanding send made at or before sentNs and queue
	// its range for repair. Returns the bytes declared lost.
	uint64_t DeclareLostSentBy(uint64_t sentNs);
	// One outstanding send (m_outstanding's packet) of DeclareLostSentBy:
	// declared lost and its range queued for repair, its loss told, deferred
	// or kept as SprayDuplicateReport says. Returns the bytes declared lost.
	uint64_t DeclareTimedOutSend(uint64_t packet);
	// Under SprayTailRecovery, a tail repair request: declare lost every
	// outstanding send along path whose path sequence is one of the count
	// from first, and queue its range for repair. A send already
	// acknowledged or declared lost is no longer outstanding and is left
	// alone, as is a later send of the same packet, which carries a path
	// sequence of its own. With deferred the selector is not told: each
	// send declared lost is appended to it instead, for RdmaHw to tell the
	// selector later (RdmaHw::m_tailLosses). Returns the bytes declared lost.
	struct DeclaredSend {
		uint64_t seq;
		uint16_t path;
		bool trickle;
	};
	uint64_t DeclareLostOnPathSeqs(uint16_t path, uint16_t first,
		uint16_t count, std::vector<DeclaredSend> *deferred = nullptr);
	// Sends declared lost by tail repair requests, and the requests that
	// found no outstanding send.
	uint32_t m_tail_declared_lost;
	uint32_t m_tail_nacks_ignored;
	// Declare lost the send of seq along path made at sentNs, a send on a
	// held spine (OutstandingPackets::Held), if it is still outstanding, and
	// queue its range for repair. Returns the bytes declared lost: zero when
	// it was acknowledged or declared lost since, or seq was sent again.
	uint64_t DeclareHeldSendLost(uint64_t seq, uint16_t path, uint64_t sentNs);
	// Sends on a held spine declared lost by DeclareHeldSendLost.
	uint32_t m_held_declared_lost;
	// SprayOverdueRepair (RdmaHw): the queue pair's latest send on a spine
	// not held as it was chosen, seq along path at sentNs, while
	// m_overdueTaken (not when the latest send was held, or once it is
	// repaired), whose answer, overdue by SprayHeldLossRtts base RTTs, has it
	// declared lost and repaired as a held send is (RdmaHw::OverdueSendDue);
	// m_overdueTimer is set for it. One repair is sent per answer or
	// timeout: m_overdueSpent, set by the repair, keeps any send from being
	// taken until the queue pair's next answer or retransmission timeout
	// clears it. Each sequence repaired waits in m_overdueRepaired, with the
	// paths of its send and of its repair, for the answer that shows which
	// arrived first (OverdueRepairSpurious).
	bool m_overdueTaken;
	uint64_t m_overdueSeq;
	uint16_t m_overduePath;
	uint64_t m_overdueSentNs;
	bool m_overdueSpent;
	EventId m_overdueTimer;
	struct OverdueRepair {
		uint16_t path;
		bool repairSent;
		uint16_t repairPath;
	};
	std::map<uint64_t, OverdueRepair> m_overdueRepaired;
	uint32_t m_overdue_declared_lost;
	uint32_t m_overdue_spurious;
	// Declare lost the send of seq along path made at sentNs, if it is still
	// outstanding, and queue its range for repair; its loss is evidence as a
	// retransmission timeout's is (DeclareLostSentBy). Returns the bytes
	// declared lost: zero when it was acknowledged or declared lost since, or
	// seq was sent again, or under SprayCoverageFirst it is covered.
	uint64_t DeclareOverdueSendLost(uint64_t seq, uint16_t path,
		uint64_t sentNs);
	// A repair of seq left along path: the first, of a sequence
	// DeclareOverdueSendLost repaired, is the repair's path.
	void NoteOverdueRepairSent(uint64_t seq, uint16_t path);
	// Whether the answer to seq along path, flagged as a duplicate or not,
	// shows a repair of DeclareOverdueSendLost spurious, the send it repaired
	// having arrived first: flagged, or unflagged along the send's path where
	// the repair took another, the send's own answer (m_overdue_spurious).
	// Unflagged along the repair's path, where the send took another, it is
	// the repair's, which arrived first; either answer ends the wait, and
	// along one path for both an unflagged answer leaves it to a flagged one.
	bool OverdueRepairSpurious(uint64_t seq, uint16_t path, bool duplicate);
	// Under SprayAckLossRtts, the sends the cumulative acknowledgement
	// covered before their own acknowledgement arrived, in the order it
	// covered them, each with the time it was covered. Their data was
	// delivered, so only the acknowledgement may have been lost on its path.
	// The own acknowledgement of one of them, found by its sequence and path,
	// drops it; one still here when it is due (AckLossDue) is declared lost
	// on its path for the selector alone. Empty unless a cumulative advance
	// removes unanswered sends.
	struct CoveredSend {
		OutstandingPackets::Unanswered send;
		uint64_t coveredNs;
	};
	std::deque<CoveredSend> m_covered;
	EventId m_ackLossTimer;
	// Covered sends declared lost on their paths.
	uint32_t m_ack_loss_declared;
	// Whether the acknowledgement of seq along path answers a covered send;
	// if so it is dropped, and the answer to a trickle send that resends
	// nothing declared lost is told to OnTrickleAnswer.
	bool AnswerCovered(uint64_t seq, uint16_t path);
	// Declare lost, on its path and for the selector alone, every covered
	// send covered delayNs or more before nowNs, as DeclareLost tells the
	// selector of a send, without a repair. Returns how many.
	uint32_t DeclareAckLosses(uint64_t nowNs, uint64_t delayNs);
	// SprayCoverageFirst (RdmaHw): before the per-path rule, the held-loss
	// rule, a tail repair request or a copy's expiry declares a send lost, the
	// coverage the queue pair has is applied: the cumulative acknowledgement,
	// snd_una or, while an answer is processed, the one it carries
	// (m_answerCoverage), and for a copy the answer of its data send too
	// (ExplorationCopy::dataAcknowledged). A send it covers was delivered, so
	// only its own answer is missing: it is an answer loss, counted in
	// m_covered_answer_losses and told to m_coveredAnswerLoss, and is neither
	// repaired nor told to the selector, so that it enters no spine's loss
	// counts, no hold and no copy's. Under SprayAckLossRtts the covered sends
	// due are counted so too, rather than declared on their paths.
	bool m_coverageFirst;
	uint64_t m_answerCoverage;
	uint32_t m_covered_answer_losses;
	Callback<void> m_coveredAnswerLoss;
	// Under SprayCoverageFirst, whether the range from seq of size bytes is
	// covered; if so it is counted as an answer loss.
	bool CoveredAnswerLoss(uint64_t seq, uint64_t size);
	void CountCoveredAnswerLoss();
	// SprayDuplicateReport (RdmaHw), under SprayCoverageFirst: the sends a
	// retransmission timeout declares lost are repaired at once, but their
	// loss is told to the selector only once the answer to the sequence's
	// resend shows what became of the first send (ResolveTimedOut). An answer
	// the receiver flags as a duplicate, of a sequence it held already, shows
	// the first send delivered: an answer loss, counted in
	// m_duplicate_answer_losses and told to the selector nowhere. An
	// unflagged one, or the cumulative acknowledgement covering the sequence
	// with no answer to its resend, tells the loss on the first send's path
	// as the timeout would have, counted in m_timeout_data_losses. Each is
	// told to m_timedOutResolved, with whether it was a duplicate.
	struct TimedOutSend {
		uint16_t path;
		bool trickle;
	};
	bool m_deferTimeoutEvidence;
	std::map<uint64_t, TimedOutSend> m_timedOut;
	uint32_t m_duplicate_answer_losses;
	uint32_t m_timeout_data_losses;
	Callback<void, bool> m_timedOutResolved;
	// Under SprayDuplicateReport's retract instead, a retransmission timeout
	// tells the loss of each send it declares at once, as without the key,
	// and keeps each in m_timeoutLosses, a sequence timed out again kept
	// once more. An answer to the sequence flagged as a duplicate takes each
	// back (PathSelector::RetractLoss), the first send having arrived:
	// counted in m_duplicate_answer_losses and m_retracted_timeout_losses,
	// and told to m_timeoutLossRetracted with the route spine the answer came
	// back over and whether the hold the loss began was released. An
	// unflagged answer, or the cumulative acknowledgement covering the
	// sequence first, leaves each a data loss, counted in
	// m_timeout_data_losses and told to m_timedOutResolved.
	struct TimeoutLoss {
		uint16_t path;
		uint64_t countedNs;
	};
	bool m_retractTimeoutEvidence;
	std::multimap<uint64_t, TimeoutLoss> m_timeoutLosses;
	uint32_t m_retracted_timeout_losses;
	Callback<void, uint16_t, bool> m_timeoutLossRetracted;
	// The answer to a send of seq over route spine route, kNoRouteSpine for
	// none, flagged as a duplicate or not.
	void ResolveTimedOut(uint64_t seq, bool duplicate, uint16_t route,
		uint64_t nowNs);
	// Under SprayAnswerPathSeq, the numbered answers read so far.
	AnswerSequenceTracker m_answerTracker;

	uint64_t GetBytesLeft();
	uint64_t GetInitialSize();
	uint32_t GetSrc();
	uint32_t GetDest();
	uint64_t GetTag();
	void SetTag(uint64_t tag);void SetSrc(uint32_t src);void SetDest(uint32_t dest);void SetInitialSize(uint64_t size);
	uint32_t GetHash(void);
	// The cumulative acknowledgement of every byte below ack; with
	// keepCovered the sends it removes unanswered are kept in m_covered.
	void Acknowledge(uint64_t ack, bool keepCovered = false);
	uint64_t GetOnTheFly();
	bool IsWinBound();
	uint64_t GetWin(); // window size calculated from m_rate
	bool IsFinished();
	bool IsFailed();
	uint64_t HpGetCurWin(); // window size calculated from hp.m_curRate, used by HPCC

private:
	// Returns the bytes declared lost. Without report the selector is not
	// told.
	uint32_t DeclareLost(uint64_t packet, bool report = true);
	// Declare lost, for the selector alone, the copies along path sent
	// before sentNs, and those ExpireCopies declares by nowNs.
	void DeclareCopiesLostOnPath(uint16_t path, uint64_t sentNs, uint64_t nowNs);
	void ExpireCopies(uint64_t nowNs);
	void DeclareCopyLost(std::map<uint64_t, ExplorationCopy>::iterator copy,
		uint64_t nowNs);
};

// Under NsccWindowState connection, the NSCC window of a sending host's queue
// pairs to one destination and priority group, each carrying one message. It is
// opened once, by the first of them, bounds the bytes all of them have in
// flight together, and outlives them, so that the next message to the
// destination starts from it; it persists across idle periods as it is.
struct NsccConnection {
	NsccWindow window;
	// The queue pairs open on it, each until it completes or fails.
	std::vector<Ptr<RdmaQueuePair>> queuePairs;
};

class RdmaRxQueuePair : public Object { // Rx side queue pair
public:
	struct ECNAccount{
		uint16_t qIndex;
		uint8_t ecnbits;
		uint16_t qfb;
		uint16_t total;

		ECNAccount() { memset(this, 0, sizeof(ECNAccount));}
	};
	ECNAccount m_ecn_source;
	uint32_t sip, dip;
	uint16_t sport, dport;
	uint16_t m_ipid;
	uint32_t ReceiverNextExpectedSeq;
	Time m_nackTimer;
	int32_t m_milestone_rx;
	uint32_t m_lastNACK;
	EventId QcnTimerEvent; // if destroy this rxQp, remember to cancel this timer
	// Out-of-order payload ranges accepted under selective retransmission.
	std::map<uint64_t, uint64_t> m_ooo_ranges;
	// Ranges the experiment layer forgave on this flow. They are absorbed
	// into m_ooo_ranges as though they had arrived, and AbsorbContiguousFrom
	// erases them as the cumulative sequence passes, so the record of what was
	// forgiven has to be kept apart from it. Its one reader counts the bytes
	// that arrive late for a range already given up.
	std::map<uint64_t, uint64_t> m_forgiven_ranges;
	// The end of the highest byte range this receiver has seen any evidence
	// of: a data packet that arrived and a trim header whose payload did not.
	// Everything below it and not settled is a hole, and the budget report is
	// measured against the sum of those.
	uint64_t m_highest_seen_end;
	// Data packets that arrived for this flow, and those among them a source
	// leaf carried on a spine other than the one requested.
	uint64_t m_data_arrivals;
	uint64_t m_folded_arrivals;
	// Under SprayPathSeq, the path sequence expected next per path.
	SpineSequenceReceiver m_pathSeqReceiver;
	// Under SprayTailRecovery with a delay, the missing runs not yet
	// requested, and the one timer that requests those due.
	PendingTailRuns m_tailRuns;
	EventId m_tailTimer;
	// Under SprayAnswerPathSeq, the numbers of this queue pair's answers per
	// route spine.
	AnswerSequencer m_answerSequencer;
	// Under SprayReceiverTailMissing, the runs requested whose guard has
	// not yet ended, and the one timer that counts those due.
	PendingTailRuns m_tailMissingRuns;
	EventId m_tailMissingTimer;
	// Whether the runs counted any path sequence missing.
	bool m_countedTailMissing;
	// Under SprayStrayPathSeq Closed, whether a packet of a flow whose receive
	// queue pair the frontend removed made this one (StrayPathSeq).
	bool m_stray;
	// Whether the experiment layer may forgive this flow on this step, asked
	// once when the queue pair is created. Every acknowledgement this queue
	// pair emits carries it, and an acknowledgement carrying it without the
	// allowance report is what grants the sender its exemption.
	bool m_forgiveness_eligible;
	// Under spray_policy with GradeScope::PerConnection, the grader this
	// connection's arrivals feed and its answers report, made at its first
	// event.
	std::unique_ptr<SpineGrader> m_spineGrader;
	// Under PathSelectorKind::AckRedirect, the spine this connection's answers
	// ask its sender to move to.
	SpineRedirect m_spineRedirect;
	static TypeId GetTypeId (void);
	RdmaRxQueuePair();
	uint32_t GetHash(void);
	void AddOutOfOrderRange(uint64_t start, uint64_t end);
	uint64_t AbsorbContiguousFrom(uint64_t expected);
	// Bytes of [start, end) the receiver has not accepted: neither below the
	// cumulative sequence nor inside an accepted out-of-order range,
	// forgiveness included. Exactly the count AddOutOfOrderRange would
	// absorb, so a ledger charged this figure charges what it takes. Zero
	// means the range is settled and the trim is a duplicate.
	uint64_t UnsettledBytes(uint64_t start, uint64_t end) const;
	// Bytes at or above `expected` the receiver has already accepted out of
	// order, which under selective repeat is everything that arrived past the
	// gap the flow is stalled on. The step stop subtracts it, because a byte
	// that arrived is not a byte to forgive.
	uint64_t AcceptedBytesAbove(uint64_t expected) const;
	// Raise the highest sequence seen. Called for every accepted data packet
	// and every trim header, because a trimmed payload is evidence the range
	// exists just as an arrival is.
	void NoteSeen(uint64_t end);
	// Bytes below the highest sequence seen that are neither received nor
	// forgiven. A range trimmed three times is one hole, and a hole vanishes
	// when its repair lands, so the count falls on its own.
	uint64_t Holes() const;
	// Record what a forgiveness of [start, end) actually gave up: the part of
	// it the receiver did not already hold. Exactly the bytes the ledger was
	// charged, so a later arrival inside it is a byte the budget paid for and
	// the sender delivered anyway. Call before absorbing the range.
	void NoteForgiven(uint64_t start, uint64_t end);
	// Bytes of [start, end) that lie inside a range already forgiven, which is
	// what a late arrival for a given-up range costs.
	uint64_t ForgivenBytes(uint64_t start, uint64_t end) const;
};

class RdmaQueuePairGroup : public Object {
public:
	std::vector<Ptr<RdmaQueuePair> > m_qps;
	//std::vector<Ptr<RdmaRxQueuePair> > m_rxQps;

	static TypeId GetTypeId (void);
	RdmaQueuePairGroup(void);
	uint32_t GetN(void);
	Ptr<RdmaQueuePair> Get(uint32_t idx);
	Ptr<RdmaQueuePair> operator[](uint32_t idx);
	void AddQp(Ptr<RdmaQueuePair> qp);
	//void AddRxQp(Ptr<RdmaRxQueuePair> rxQp);
	void Clear(void);
};

}

#endif /* RDMA_QUEUE_PAIR_H */
