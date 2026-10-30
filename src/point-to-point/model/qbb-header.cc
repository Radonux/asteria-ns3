#include <stdint.h>
#include <iostream>
#include "qbb-header.h"
#include "ns3/buffer.h"
#include "ns3/address-utils.h"
#include "ns3/log.h"
#include "ns3/custom-header.h"

NS_LOG_COMPONENT_DEFINE("qbbHeader");

namespace ns3 {

	namespace {
	// A spine field of bytes bytes, zero, one or two (SpineFieldBytes): in
	// one byte 255 stands for none, 0xffff, which no spine index takes over
	// at most 255 spines.
	void WriteSpineField(Buffer::Iterator &i, uint16_t value, uint32_t bytes){
		if (bytes == 1)
			i.WriteU8(value == 0xffff ? 0xff : static_cast<uint8_t>(value));
		else if (bytes == 2)
			i.WriteU16(value);
	}
	uint16_t ReadSpineField(Buffer::Iterator &i, uint32_t bytes){
		if (bytes == 2)
			return i.ReadU16();
		const uint8_t value = i.ReadU8();
		return value == 0xff ? 0xffff : value;
	}
	} // namespace

	NS_OBJECT_ENSURE_REGISTERED(qbbHeader);

	qbbHeader::qbbHeader(uint16_t pg)
		: sport(0), dport(0), flags(0), m_pg(pg), m_seq(0),
		  m_trimPayloadSize(0), m_packetSeq(0), m_redirect(0xffff),
		  m_receiverHold(), m_receiverHoldPartial(false), m_tailSpine(0),
		  m_tailSeq(0), m_tailCount(0),
		  m_routeSpine(0xffff), m_answerSeq(0), m_announceRoute(0xffff),
		  m_announceSeq(0)
	{
	}

	qbbHeader::qbbHeader()
		: sport(0), dport(0), flags(0), m_pg(0), m_seq(0),
		  m_trimPayloadSize(0), m_packetSeq(0), m_redirect(0xffff),
		  m_receiverHold(), m_receiverHoldPartial(false), m_tailSpine(0),
		  m_tailSeq(0), m_tailCount(0),
		  m_routeSpine(0xffff), m_answerSeq(0), m_announceRoute(0xffff),
		  m_announceSeq(0)
	{}

	qbbHeader::~qbbHeader()
	{}

	void qbbHeader::SetPG(uint16_t pg)
	{
		m_pg = pg;
	}

	void qbbHeader::SetSeq(uint32_t seq)
	{
		m_seq = seq;
	}

	void qbbHeader::SetSport(uint32_t _sport){
		sport = _sport;
	}
	void qbbHeader::SetDport(uint32_t _dport){
		dport = _dport;
	}

	void qbbHeader::SetTs(uint64_t ts){
		NS_ASSERT_MSG(IntHeader::mode == 1, "qbbHeader cannot SetTs when IntHeader::mode != 1");
		ih.ts = ts;
	}
	void qbbHeader::SetCnp(){
		flags |= 1 << FLAG_CNP;
	}
	void qbbHeader::SetTrimPayloadSize(uint32_t payloadSize){
		m_trimPayloadSize = payloadSize;
	}
	void qbbHeader::SetTrimLastHop(bool lastHop){
		if (lastHop)
			flags |= 1 << FLAG_TRIM_LASTHOP;
		else
			flags &= ~(1 << FLAG_TRIM_LASTHOP);
	}
	void qbbHeader::SetAllowanceExhausted(bool spent){
		if (spent)
			flags |= 1 << FLAG_ALLOWANCE_EXHAUSTED;
		else
			flags &= ~(1 << FLAG_ALLOWANCE_EXHAUSTED);
	}
	void qbbHeader::SetForgivenessEligible(bool eligible){
		if (eligible)
			flags |= 1 << FLAG_FORGIVENESS_ELIGIBLE;
		else
			flags &= ~(1 << FLAG_FORGIVENESS_ELIGIBLE);
	}
	void qbbHeader::SetProbeAnswer(){
		flags |= 1 << FLAG_PROBE_ANSWER;
	}
	void qbbHeader::SetRouteProbe(){
		flags |= 1 << FLAG_ROUTE_PROBE;
	}
	void qbbHeader::SetRouteProbeReply(){
		flags |= 1 << FLAG_ROUTE_PROBE_REPLY;
	}
	void qbbHeader::SetDuplicate(){
		flags |= 1 << FLAG_DUPLICATE;
	}
	void qbbHeader::SetPathAvoid(){
		flags |= 1 << FLAG_PATH_AVOID;
	}
	void qbbHeader::SetPacketSeq(uint32_t seq){
		m_packetSeq = seq;
	}
	void qbbHeader::SetSpineReport(const SpineReport &report){
		m_spineReport = report;
		if (report.edgeCongested)
			flags |= 1 << FLAG_EDGE_CONGESTED;
		else
			flags &= ~(1 << FLAG_EDGE_CONGESTED);
	}
	void qbbHeader::SetRedirect(uint16_t spine){
		m_redirect = spine;
	}
	void qbbHeader::SetReceiverHold(const SpineBitmap &bitmap){
		m_receiverHold = bitmap;
	}
	void qbbHeader::SetReceiverHoldPartial(bool partial){
		m_receiverHoldPartial = partial;
	}
	void qbbHeader::SetTailNack(uint8_t spine, uint16_t seq, uint16_t count){
		flags |= 1 << FLAG_TAIL_NACK;
		m_tailSpine = spine;
		m_tailSeq = seq;
		m_tailCount = count;
	}
	void qbbHeader::SetRouteSpine(uint16_t spine){
		m_routeSpine = spine;
	}
	void qbbHeader::SetAnswerSeq(uint16_t seq, uint16_t announceRoute,
			uint16_t announceSeq){
		flags |= 1 << FLAG_ANSWER_NUMBERED;
		m_answerSeq = seq;
		m_announceRoute = announceRoute;
		m_announceSeq = announceSeq;
	}
	void qbbHeader::SetIntHeader(const IntHeader &_ih){
		ih = _ih;
	}

	uint16_t qbbHeader::GetPG() const
	{
		return m_pg;
	}

	uint32_t qbbHeader::GetSeq() const
	{
		return m_seq;
	}

	uint16_t qbbHeader::GetSport() const{
		return sport;
	}
	uint16_t qbbHeader::GetDport() const{
		return dport;
	}

	uint64_t qbbHeader::GetTs() const {
		NS_ASSERT_MSG(IntHeader::mode == 1, "qbbHeader cannot GetTs when IntHeader::mode != 1");
		return ih.ts;
	}
	uint8_t qbbHeader::GetCnp() const{
		return (flags >> FLAG_CNP) & 1;
	}
	uint32_t qbbHeader::GetTrimPayloadSize() const{
		return m_trimPayloadSize;
	}
	bool qbbHeader::IsTrimLastHop() const{
		return (flags >> FLAG_TRIM_LASTHOP) & 1;
	}
	bool qbbHeader::IsAllowanceExhausted() const{
		return (flags >> FLAG_ALLOWANCE_EXHAUSTED) & 1;
	}
	uint32_t qbbHeader::GetPacketSeq() const{
		return m_packetSeq;
	}
	const SpineReport &qbbHeader::GetSpineReport() const{
		return m_spineReport;
	}
	uint16_t qbbHeader::GetRedirect() const{
		return m_redirect;
	}
	SpineBitmap qbbHeader::GetReceiverHold() const{
		return m_receiverHold;
	}
	bool qbbHeader::GetReceiverHoldPartial() const{
		return m_receiverHoldPartial;
	}
	bool qbbHeader::IsTailNack() const{
		return (flags >> FLAG_TAIL_NACK) & 1;
	}
	uint8_t qbbHeader::GetTailSpine() const{
		return m_tailSpine;
	}
	uint16_t qbbHeader::GetTailSeq() const{
		return m_tailSeq;
	}
	uint16_t qbbHeader::GetTailCount() const{
		return m_tailCount;
	}
	uint16_t qbbHeader::GetRouteSpine() const{
		return m_routeSpine;
	}
	bool qbbHeader::IsAnswerNumbered() const{
		return (flags >> FLAG_ANSWER_NUMBERED) & 1;
	}
	uint16_t qbbHeader::GetAnswerSeq() const{
		return m_answerSeq;
	}
	uint16_t qbbHeader::GetAnnounceRoute() const{
		return m_announceRoute;
	}
	uint16_t qbbHeader::GetAnnounceSeq() const{
		return m_announceSeq;
	}

	TypeId
		qbbHeader::GetTypeId(void)
	{
		static TypeId tid = TypeId("ns3::qbbHeader")
			.SetParent<Header>()
			.AddConstructor<qbbHeader>()
			;
		return tid;
	}
	TypeId
		qbbHeader::GetInstanceTypeId(void) const
	{
		return GetTypeId();
	}
	void qbbHeader::Print(std::ostream &os) const
	{
		os << "qbb:" << "pg=" << m_pg << ",seq=" << m_seq;
	}
	uint32_t qbbHeader::GetSerializedSize(void)  const
	{
		return GetBaseSize() + IntHeader::GetStaticSize();
	}
	uint32_t qbbHeader::GetBaseSize() {
		qbbHeader tmp;
		return sizeof(tmp.sport) + sizeof(tmp.dport) + sizeof(tmp.flags) + sizeof(tmp.m_pg) + sizeof(tmp.m_seq) + sizeof(tmp.m_trimPayloadSize) +
			(CustomHeader::ackCarriesPacketSeq ? sizeof(tmp.m_packetSeq) : 0) +
			CustomHeader::ackReportBytes + CustomHeader::ackRedirectBytes +
			CustomHeader::ackReceiverHoldBytes + CustomHeader::ackTailNackBytes +
			CustomHeader::ackRouteSpineBytes + CustomHeader::ackAnswerSeqBytes;
	}
	void qbbHeader::Serialize(Buffer::Iterator start)  const
	{
		Buffer::Iterator i = start;
		i.WriteU16(sport);
		i.WriteU16(dport);
		i.WriteU16(flags);
		i.WriteU16(m_pg);
		i.WriteU32(m_seq);
		i.WriteU32(m_trimPayloadSize);
		if (CustomHeader::ackCarriesPacketSeq)
			i.WriteU32(m_packetSeq);
		if (CustomHeader::ackReportBytes > 0){
			i.WriteU8(m_spineReport.sequence);
			i.Write(m_spineReport.grades, CustomHeader::ackReportBytes - 1);
		}
		WriteSpineField(i, m_redirect, CustomHeader::ackRedirectBytes);
		if (CustomHeader::ackReceiverHoldSlots > 0){
			uint8_t list[SpineListBytes(kMaxSpineListSlots)];
			m_receiverHold.ToList(list, CustomHeader::ackReceiverHoldSlots,
				m_receiverHoldPartial);
			i.Write(list, CustomHeader::ackReceiverHoldBytes);
		}else
			for (uint32_t b = 0; b < CustomHeader::ackReceiverHoldBytes; b++)
				i.WriteU8(m_receiverHold.Byte(b));
		if (CustomHeader::ackTailNackBytes > 0){
			i.WriteU8(m_tailSpine);
			i.WriteU16(m_tailSeq);
			i.WriteU16(m_tailCount);
		}
		WriteSpineField(i, m_routeSpine, CustomHeader::ackRouteSpineBytes);
		if (CustomHeader::ackAnswerSeqBytes > 0){
			i.WriteU16(m_answerSeq);
			WriteSpineField(i, m_announceRoute, CustomHeader::ackAnswerSeqBytes - 4);
			i.WriteU16(m_announceSeq);
		}

		// write IntHeader
		ih.Serialize(i);
	}

	uint32_t qbbHeader::Deserialize(Buffer::Iterator start)
	{
		Buffer::Iterator i = start;
		sport = i.ReadU16();
		dport = i.ReadU16();
		flags = i.ReadU16();
		m_pg = i.ReadU16();
		m_seq = i.ReadU32();
		m_trimPayloadSize = i.ReadU32();
		if (CustomHeader::ackCarriesPacketSeq)
			m_packetSeq = i.ReadU32();
		if (CustomHeader::ackReportBytes > 0){
			m_spineReport.sequence = i.ReadU8();
			i.Read(m_spineReport.grades, CustomHeader::ackReportBytes - 1);
			m_spineReport.edgeCongested = (flags >> FLAG_EDGE_CONGESTED) & 1;
		}
		if (CustomHeader::ackRedirectBytes > 0)
			m_redirect = ReadSpineField(i, CustomHeader::ackRedirectBytes);
		m_receiverHoldPartial = false;
		if (CustomHeader::ackReceiverHoldSlots > 0){
			uint8_t list[SpineListBytes(kMaxSpineListSlots)];
			i.Read(list, CustomHeader::ackReceiverHoldBytes);
			m_receiverHold = SpineBitmap::FromList(list,
				CustomHeader::ackReceiverHoldSlots, m_receiverHoldPartial);
		}else if (CustomHeader::ackReceiverHoldBytes > 0){
			uint8_t bytes[kSpineBitmapBytes] = {};
			for (uint32_t b = 0; b < CustomHeader::ackReceiverHoldBytes; b++)
				bytes[b] = i.ReadU8();
			m_receiverHold = SpineBitmap::FromBytes(bytes, CustomHeader::ackReceiverHoldBytes);
		}
		if (CustomHeader::ackTailNackBytes > 0){
			m_tailSpine = i.ReadU8();
			m_tailSeq = i.ReadU16();
			m_tailCount = i.ReadU16();
		}
		if (CustomHeader::ackRouteSpineBytes > 0)
			m_routeSpine = ReadSpineField(i, CustomHeader::ackRouteSpineBytes);
		if (CustomHeader::ackAnswerSeqBytes > 0){
			m_answerSeq = i.ReadU16();
			m_announceRoute = ReadSpineField(i, CustomHeader::ackAnswerSeqBytes - 4);
			m_announceSeq = i.ReadU16();
		}

		// read IntHeader
		ih.Deserialize(i);
		return GetSerializedSize();
	}
}; // namespace ns3
