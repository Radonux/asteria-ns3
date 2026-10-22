//yibo

#ifndef QBB_HEADER_H
#define QBB_HEADER_H

#include <stdint.h>
#include "ns3/header.h"
#include "ns3/buffer.h"
#include "ns3/int-header.h"
#include "spine-report.h"
#include "ns3/spine-bitmap.h"

namespace ns3 {

/**
 * \ingroup Pause
 * \brief Header for the Congestion Notification Message
 *
 * This class has two fields: The five-tuple flow id and the quantized
 * congestion level. This can be serialized to or deserialzed from a byte
 * buffer.
 */
 
class qbbHeader : public Header
{
public:
 
  enum {
    FLAG_CNP = 0,
    FLAG_TRIM_LASTHOP = 1,
    // The receiver reports that the budget entry this range belongs to has no
    // allowance left. It is the only thing a sender cannot work out for
    // itself, and the only thing that ends its congestion exemption.
    FLAG_ALLOWANCE_EXHAUSTED = 2,
    // The receiver reports that this flow is one it may forgive on this step.
    // An acknowledgement carrying it with FLAG_ALLOWANCE_EXHAUSTED clear is
    // the grant: from it the sender withholds congestion signals. The sender
    // cannot know either fact, because eligibility and the step's phase are
    // the receiver's, so the grant is the receiver's to give.
    FLAG_FORGIVENESS_ELIGIBLE = 3,
    // The acknowledgement answers a path probe, not a data packet: it returns
    // the probe's identification and mark and acknowledges no send.
    FLAG_PROBE_ANSWER = 4,
    // The spine report this header carries says the receiver's own downlink
    // is congested.
    FLAG_EDGE_CONGESTED = 5,
    // A repair request names missing path sequences on a spine (SetTailNack)
    // rather than a trimmed byte range.
    FLAG_TAIL_NACK = 6,
    // The answer carries its number on its route spine (SetAnswerSeq).
    FLAG_ANSWER_NUMBERED = 7,
    // Under AckRoute ReverseSet: an acknowledgement that probes the route
    // spine it is routed over and acknowledges nothing (SetRouteProbe), and
    // the path probe that returns it over the same spine
    // (SetRouteProbeReply).
    FLAG_ROUTE_PROBE = 8,
    FLAG_ROUTE_PROBE_REPLY = 9,
    // Under SprayDuplicateReport: the acknowledgement answers a data packet
    // whose sequence the receiver held already (SetDuplicate).
    FLAG_DUPLICATE = 10,
    // Under PathSelector Themis: the acknowledgement is Themis's Path
    // Avoidance Signal for the packet its cumulative sequence names
    // (SetPathAvoid).
    FLAG_PATH_AVOID = 11
  };
  qbbHeader (uint16_t pg);
  qbbHeader ();
  virtual ~qbbHeader ();

//Setters
  /**
   * \param pg The PG
   */
  void SetPG (uint16_t pg);
  void SetSeq(uint32_t seq);
  void SetSport(uint32_t _sport);
  void SetDport(uint32_t _dport);
  void SetTs(uint64_t ts);
  void SetCnp();
  void SetTrimPayloadSize(uint32_t payloadSize);
  void SetTrimLastHop(bool lastHop);
  void SetAllowanceExhausted(bool spent);
  void SetForgivenessEligible(bool eligible);
  void SetProbeAnswer();
  // FLAG_ROUTE_PROBE and FLAG_ROUTE_PROBE_REPLY.
  void SetRouteProbe();
  void SetRouteProbeReply();
  // FLAG_DUPLICATE.
  void SetDuplicate();
  // FLAG_PATH_AVOID.
  void SetPathAvoid();
  // The sequence of the data packet this acknowledgement answers, serialized
  // only while CustomHeader::ackCarriesPacketSeq is set.
  void SetPacketSeq(uint32_t seq);
  // The receiving host's latest spine report, serialized only while
  // CustomHeader::ackReportBytes is not zero; its edge bit is a flag.
  void SetSpineReport(const SpineReport &report);
  // The spine the receiver asks the sender to move to, kNoRedirect for none,
  // the default; serialized only while CustomHeader::ackRedirectBytes is not
  // zero.
  void SetRedirect(uint16_t spine);
  // The spines the receiving host holds for the sender's leaf, bit k for
  // spine k, none by default; serialized only while
  // CustomHeader::ackReceiverHoldBytes is not zero, in that many bytes.
  void SetReceiverHold(const SpineBitmap &bitmap);
  // Under CustomHeader::ackReceiverHoldSlots, whether that spine list is
  // partial; false by default.
  void SetReceiverHoldPartial(bool partial);
  // The count path sequences from seq on spine that the receiver misses,
  // and FLAG_TAIL_NACK; serialized only while CustomHeader::ackTailNackBytes
  // is not zero.
  void SetTailNack(uint8_t spine, uint16_t seq, uint16_t count);
  // The spine the switches route this answer over, kNoRouteSpine for the
  // one its identification names as carrying the data, the default;
  // serialized only while CustomHeader::ackRouteSpineBytes is not zero.
  void SetRouteSpine(uint16_t spine);
  // The answer's number on its route spine and the count of answers another
  // route carried (kNoAnswerRoute for none), and FLAG_ANSWER_NUMBERED;
  // serialized only while CustomHeader::ackAnswerSeqBytes is not zero.
  void SetAnswerSeq(uint16_t seq, uint16_t announceRoute, uint16_t announceSeq);
  void SetIntHeader(const IntHeader &_ih);

//Getters
  /**
   * \return The pg
   */
  uint16_t GetPG () const;
  uint32_t GetSeq() const;
  uint16_t GetPort() const;
  uint16_t GetSport() const;
  uint16_t GetDport() const;
  uint64_t GetTs() const;
  uint8_t GetCnp() const;
  uint32_t GetTrimPayloadSize() const;
  bool IsTrimLastHop() const;
  bool IsAllowanceExhausted() const;
  uint32_t GetPacketSeq() const;
  const SpineReport &GetSpineReport() const;
  uint16_t GetRedirect() const;
  SpineBitmap GetReceiverHold() const;
  bool GetReceiverHoldPartial() const;
  bool IsTailNack() const;
  uint8_t GetTailSpine() const;
  uint16_t GetTailSeq() const;
  uint16_t GetTailCount() const;
  uint16_t GetRouteSpine() const;
  bool IsAnswerNumbered() const;
  uint16_t GetAnswerSeq() const;
  uint16_t GetAnnounceRoute() const;
  uint16_t GetAnnounceSeq() const;

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  static uint32_t GetBaseSize(); // size without INT

private:
  uint16_t sport, dport;
  uint16_t flags;
  uint16_t m_pg;
  uint32_t m_seq; // the qbb sequence number.
  uint32_t m_trimPayloadSize;
  uint32_t m_packetSeq;
  SpineReport m_spineReport;
  uint16_t m_redirect;
  SpineBitmap m_receiverHold;
  bool m_receiverHoldPartial;
  uint8_t m_tailSpine;
  uint16_t m_tailSeq;
  uint16_t m_tailCount;
  uint16_t m_routeSpine;
  uint16_t m_answerSeq;
  uint16_t m_announceRoute;
  uint16_t m_announceSeq;
  IntHeader ih;
  
};

}; // namespace ns3

#endif /* QBB_HEADER */
