/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005 INRIA
 *
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
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#ifndef CUSTOM_HEADER_H
#define CUSTOM_HEADER_H

#include "ns3/header.h"
#include "ns3/int-header.h"
#include "ns3/spine-bitmap.h"

namespace ns3 {

constexpr uint8_t kUecTrimRepairProtocol = 0xFA;
constexpr uint8_t kUecTrimNotificationProtocol = 0xFB;
// A sender's probe of the path its identification names, which the receiver
// answers with an acknowledgement.
constexpr uint8_t kPathProbeProtocol = 0xF9;

// Whether a packet of this IPv4 protocol carries a qbbHeader after its IP
// header: an ACK, a NACK, a trim's repair request or notification, or a path
// probe.
constexpr bool CarriesQbbHeader(uint8_t protocol){
	return protocol == 0xFC || protocol == 0xFD ||
		protocol == kUecTrimRepairProtocol ||
		protocol == kUecTrimNotificationProtocol ||
		protocol == kPathProbeProtocol;
}

// The most spines a data header's tail counts name (udpTailCounts), the bit
// of udp.tail_flags that marks a packet closing its requested spine, and,
// under AckRoute Repath only, the bit that marks a send also sent as a copy,
// both of whose packets carry it. Over at most kFixedTailCounts spines every
// data header carries the counts; over more, only a header that closes its
// spine does, and every other one the flags byte alone.
constexpr uint32_t kMaxTailCounts = kMaxSpineCount;
constexpr uint32_t kFixedTailCounts = 8;
constexpr uint8_t kTailCloses = 1;
constexpr uint8_t kTailCopied = 2;

// UET diffserv codepoints. UEC 1.0.3 section 3.6.4.7.1 names the codepoints
// abstractly and section 4.1 states that "this spec does not place any
// constraints on the values of the DSCPs used"; only distinctness is
// normative (section 4.1.4.1: DSCP_TRIMMED MUST differ from both
// DSCP_TRIMMABLE and DSCP_CONTROL). These values realize the RECOMMENDED
// four-codepoint / three-traffic-class mapping of section 4.1.4.1.
constexpr uint8_t kUetDscpControl = 46;        // DSCP_CONTROL         -> TC_high
constexpr uint8_t kUetDscpTrimmable = 8;       // DSCP_TRIMMABLE       -> TC_low
constexpr uint8_t kUetDscpTrimmed = 16;        // DSCP_TRIMMED         -> TC_med
constexpr uint8_t kUetDscpTrimmedLastHop = 18; // DSCP_TRIMMED_LASTHOP -> TC_med

// A switch maps each DSCP_TRIMMABLE codepoint onto a DSCP_TRIMMED codepoint
// (UEC 1.0.3 section 4.1). Only one trimmable codepoint is modeled here, so
// the mapping is a single pair plus the optional last-hop variant.
inline bool IsUetTrimmableDscp(uint8_t dscp){
	return dscp == kUetDscpTrimmable;
}
inline bool IsUetTrimmedDscp(uint8_t dscp){
	return dscp == kUetDscpTrimmed || dscp == kUetDscpTrimmedLastHop;
}
/**
 * \ingroup ipv4
 *
 * \brief Custom packet header
 */
class CustomHeader : public Header 
{
public:
  /**
   * \brief Construct a null custom header
   */
  CustomHeader ();
  CustomHeader (uint32_t _headerType);
  /**
   * \enum EcnType
   * \brief ECN Type defined in \RFC{3168}
   */
  enum EcnType
    {
      // Prefixed with "ECN" to avoid name clash (bug 1723)
      ECN_NotECT = 0x00,
      ECN_ECT1 = 0x01,
      ECN_ECT0 = 0x02,
      ECN_CE = 0x03
    }; 
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);

  uint32_t brief, headerType, getInt;
  enum HeaderType{
	L2_Header = 1,
	L3_Header = 2,
	L4_Header = 4
  };

  // ppp header
  uint16_t pppProto;

  // IPv4 header
  enum FlagsE {
    DONT_FRAGMENT = (1<<0),
    MORE_FRAGMENTS = (1<<1)
  };
  uint16_t m_payloadSize; //!< payload size
  uint16_t ipid; //!< identification
  uint32_t m_tos : 8; //!< TOS, also used as DSCP + ECN value
  uint32_t m_ttl : 8; //!< TTL
  uint32_t l3Prot: 8;  //!< Protocol
  uint32_t ipv4Flags : 3; //!< flags
  uint16_t m_fragmentOffset;  //!< Fragment offset
  uint32_t sip; //!< source address
  uint32_t dip; //!< destination address
  uint16_t m_checksum; //!< checksum
  uint16_t m_headerSize; //!< IP header size

  union {
	  struct {
		  uint16_t sport;        //!< Source port
		  uint16_t dport;   //!< Destination port
		  uint32_t seq;  //!< Sequence number
		  uint32_t ack;       //!< ACK number
		  uint8_t length;             //!< Length (really a uint4_t) in words.
		  uint8_t tcpFlags;              //!< Flags (really a uint6_t)
		  uint16_t windowSize;        //!< Window size
		  uint16_t urgentPointer;     //!< Urgent pointer
		  uint8_t optionBuf[32]; // buffer for storing raw options
	  } tcp;
	  struct {
		  uint16_t sport;        //!< Source port
		  uint16_t dport;   //!< Destination port
		  uint16_t payload_size;
		  // SeqTsHeader
		  uint16_t pg;
		  uint32_t seq;
		  IntHeader ih;
		  // The sender's count of the data packets of this queue pair sent
		  // before this one up the spine it requested. On the wire after the
		  // INT header, only while udpCarriesPathSeq is set.
		  uint16_t path_seq;
		  // Under tail recovery, after path_seq, only while udpTailCounts is
		  // not zero: kTailCloses set on a packet sent once its queue pair
		  // has no new data left, which closes its requested spine, and then
		  // per spine k below udpTailCounts the count of the data packets
		  // the queue pair has sent up k, this one included.
		  uint8_t tail_flags;
		  uint16_t tail_counts[kMaxTailCounts];
		  // Under answer path sequences, after the tail fields, only while
		  // udpAnswerHoldBytes is not zero: the route spines the sender holds
		  // for the answers of the destination's leaf, bit k of byte k / 8
		  // for spine k, udpAnswerHoldBytes of it on the wire.
		  uint8_t answer_hold[kSpineBitmapBytes];
	  } udp;
	  // CnHeader
	  struct {
		  uint16_t fid;
		  uint8_t qIndex;
		  uint16_t qfb;
		  uint8_t ecnBits;
		  uint16_t total;
	  } cnp;
	  // qbbHeader
	  struct {
		  uint16_t sport, dport;
		  uint16_t flags;
		  uint16_t pg;
		  uint32_t seq; // the qbb sequence number.
      uint32_t trim_payload_size;
		  // The sequence of the data packet an acknowledgement answers. On the
		  // wire only while ackCarriesPacketSeq is set.
		  uint32_t packet_seq;
		  // The receiving host's latest report on the spines: its sequence,
		  // then two bits per spine. On the wire, ackReportBytes of it, only
		  // while that is not zero.
		  uint8_t spine_report[1 + kMaxSpineCount / 4];
		  // The spine the receiver asks the sender to move to, or 0xffff for
		  // none. On the wire, after the report, only while ackRedirectBytes
		  // is not zero, SpineFieldBytes of the spines: in one byte, 255 for
		  // none.
		  uint16_t redirect;
		  // The spines the receiving host holds for the sender's leaf, bit k
		  // of byte k / 8 for spine k. On the wire, after the redirect field,
		  // ackReceiverHoldBytes of it, only while that is not zero: the
		  // bitmap, or under ackReceiverHoldSlots the spine list naming them.
		  uint8_t receiver_hold[kSpineBitmapBytes];
		  // Whether that spine list is partial, naming some of the held
		  // spines only; false for the bitmap.
		  bool receiver_hold_partial;
		  // A tail repair request (qbbHeader::FLAG_TAIL_NACK): the spine,
		  // the first path sequence missing on it and how many from there.
		  // On the wire, after receiver_hold, only while ackTailNackBytes is
		  // not zero.
		  uint8_t tail_spine;
		  uint16_t tail_seq;
		  uint16_t tail_count;
		  // The spine the switches route the answer over, or 0xffff for the
		  // carrying spine its identification names. On the wire, after the
		  // tail fields, only while ackRouteSpineBytes is not zero,
		  // SpineFieldBytes of the spines: in one byte, 255 for none.
		  uint16_t route_spine;
		  // Under answer path sequences, after route_spine, only while
		  // ackAnswerSeqBytes is not zero: the answer's number on its route
		  // spine, and another route spine and its count of the queue pair's
		  // answers (0xffff for none; in one byte, 255, while
		  // ackAnswerSeqBytes is five).
		  uint16_t answer_seq;
		  uint16_t announce_route;
		  uint16_t announce_seq;
		  IntHeader ih;
	  } ack;
	  // PauseHeader
	  struct {
		  uint32_t time;
		  uint32_t qlen;
		  uint8_t qIndex;
	  } pfc;
  };

  // Whether acknowledgement headers carry ack.packet_seq. Process-wide, as
  // IntHeader::mode is, because every writer of the header and this parser
  // must agree on its size.
  static bool ackCarriesPacketSeq;
  // How many bytes of ack.spine_report acknowledgement headers carry, zero
  // for none. Process-wide for the same reason.
  static uint32_t ackReportBytes;
  // How many bytes the redirect field takes, zero or SpineFieldBytes of the
  // spines: one up to 255 spines, two at 256. Process-wide for the same
  // reason.
  static uint32_t ackRedirectBytes;
  // Whether data headers carry udp.path_seq. Process-wide for the same
  // reason.
  static bool udpCarriesPathSeq;
  // How many bytes the receiver_hold field takes, zero, SpineBitmapBytes of
  // the spines or SpineListBytes of ackReceiverHoldSlots. Process-wide for
  // the same reason.
  static uint32_t ackReceiverHoldBytes;
  // The slots of the spine list the receiver_hold field takes, zero for the
  // bitmap. Process-wide for the same reason.
  static uint32_t ackReceiverHoldSlots;
  // How many per-spine counts the tail fields of data headers carry, zero
  // for none; the fields take 1 + 2 * udpTailCounts bytes, or over more
  // than kFixedTailCounts spines the flags byte alone where they do not
  // close the spine. Process-wide for the same reason.
  static uint32_t udpTailCounts;
  // How many bytes the tail repair request fields take, zero or five.
  // Process-wide for the same reason.
  static uint32_t ackTailNackBytes;
  // How many bytes the route_spine field takes, zero or SpineFieldBytes of
  // the spines. Process-wide for the same reason.
  static uint32_t ackRouteSpineBytes;
  // How many bytes the answer path sequence fields take, zero or four and
  // announce_route's SpineFieldBytes of the spines. Process-wide for the
  // same reason.
  static uint32_t ackAnswerSeqBytes;
  // How many bytes the answer_hold field of data headers takes, zero or
  // SpineBitmapBytes of the spines; it follows the path sequence, which it
  // needs. Process-wide for the same reason.
  static uint32_t udpAnswerHoldBytes;
  // The tail fields' bytes of a data header whose tail flags are flags, and
  // without an argument of one that does not close its spine.
  static uint32_t GetUdpTailBytes(void);
  static uint32_t GetUdpTailBytes(uint8_t flags);
  // Whether a data header with tail flags flags carries the counts.
  static bool CarriesTailCounts(uint8_t flags);
  // The data header's bytes, of this parsed header: with the counts where
  // its tail flags close its spine.
  uint32_t UdpHeaderBytes(void) const;

  uint8_t GetIpv4EcnBits (void) const;
  uint8_t GetIpv4Dscp (void) const;
  static uint32_t GetAckSerializedSize(void);
  static uint32_t GetUdpHeaderSize(void); // include udp, seqTs, INT
  // The same of a data header with tail flags flags.
  static uint32_t GetUdpHeaderSize(uint8_t flags);
  static uint32_t GetStaticWholeHeaderSize(void); // ppp + ip + udp + int
};

} // namespace ns3


#endif /* CUSTOM_HEADER_H */

