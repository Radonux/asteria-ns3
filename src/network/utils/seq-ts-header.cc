/*
 * Copyright (c) 2009 INRIA
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

#include "seq-ts-header.h"
#include "custom-header.h"

#include "ns3/assert.h"
#include "ns3/header.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SeqTsHeader");

NS_OBJECT_ENSURE_REGISTERED(SeqTsHeader);

SeqTsHeader::SeqTsHeader()
    : m_seq(0),
      m_ts(Simulator::Now().GetTimeStep()),
      m_pathSeq(0),
      m_tailFlags(0),
      m_tailCounts{}
{
    NS_LOG_FUNCTION(this);
    if (IntHeader::mode == 1)
        ih.ts = Simulator::Now().GetTimeStep();
}

void
SeqTsHeader::SetSeq(uint32_t seq)
{
    NS_LOG_FUNCTION(this << seq);
    m_seq = seq;
}

uint32_t
SeqTsHeader::GetSeq() const
{
    NS_LOG_FUNCTION(this);
    return m_seq;
}

Time
SeqTsHeader::GetTs() const
{
    NS_LOG_FUNCTION(this);
    return TimeStep(m_ts);
}

void
SeqTsHeader::SetPG (uint16_t pg)
{
	m_pg = pg;
}

uint16_t
SeqTsHeader::GetPG (void) const
{
	return m_pg;
}

void
SeqTsHeader::SetPathSeq (uint16_t pathSeq)
{
	m_pathSeq = pathSeq;
}

uint16_t
SeqTsHeader::GetPathSeq (void) const
{
	return m_pathSeq;
}

void
SeqTsHeader::SetTail (uint8_t flags, const uint16_t* counts, uint32_t n)
{
	m_tailFlags = flags;
	const uint32_t written = std::min(kMaxTailCounts,
		std::max(n, CustomHeader::udpTailCounts));
	for (uint32_t k = 0; k < written; k++)
		m_tailCounts[k] = k < n ? counts[k] : 0;
}

void
SeqTsHeader::AddTailFlags (uint8_t flags)
{
	m_tailFlags |= flags;
}

void
SeqTsHeader::SetAnswerHold (const SpineBitmap &bitmap)
{
	m_answerHold = bitmap;
}

SpineBitmap
SeqTsHeader::GetAnswerHold (void) const
{
	return m_answerHold;
}

uint8_t
SeqTsHeader::GetTailFlags (void) const
{
	return m_tailFlags;
}

uint16_t
SeqTsHeader::GetTailCount (uint32_t spine) const
{
	return spine < std::min(kMaxTailCounts, CustomHeader::udpTailCounts)
		? m_tailCounts[spine] : 0;
}

TypeId
SeqTsHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SeqTsHeader")
                            .SetParent<Header>()
                            .SetGroupName("Applications")
                            .AddConstructor<SeqTsHeader>();
    return tid;
}

TypeId
SeqTsHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

void
SeqTsHeader::Print(std::ostream& os) const
{
    NS_LOG_FUNCTION(this << &os);
    os << "(seq=" << m_seq << " time=" << TimeStep(m_ts).As(Time::S) << ")";
}

uint32_t
SeqTsHeader::GetSerializedSize() const
{
    NS_LOG_FUNCTION(this);
	return 6 + IntHeader::GetStaticSize() +
		(CustomHeader::udpCarriesPathSeq
			? sizeof(uint16_t) + CustomHeader::udpAnswerHoldBytes : 0) +
		CustomHeader::GetUdpTailBytes(m_tailFlags);
}

uint32_t SeqTsHeader::GetHeaderSize(void){
	return 6 + IntHeader::GetStaticSize() +
		(CustomHeader::udpCarriesPathSeq
			? sizeof(uint16_t) + CustomHeader::udpAnswerHoldBytes : 0) +
		CustomHeader::GetUdpTailBytes();
}

void
SeqTsHeader::Serialize(Buffer::Iterator start) const
{
    NS_LOG_FUNCTION(this << &start);
    Buffer::Iterator i = start;
    i.WriteHtonU32(m_seq);
    i.WriteHtonU16(m_pg);

    // write IntHeader
    ih.Serialize(i);
    if (CustomHeader::udpCarriesPathSeq)
    {
        i.Next(IntHeader::GetStaticSize());
        i.WriteHtonU16(m_pathSeq);
        if (CustomHeader::udpTailCounts > 0)
        {
            i.WriteU8(m_tailFlags);
            if (CustomHeader::CarriesTailCounts(m_tailFlags))
                for (uint32_t k = 0; k < CustomHeader::udpTailCounts; k++)
                    i.WriteHtonU16(m_tailCounts[k]);
        }
        if (CustomHeader::udpAnswerHoldBytes > 0)
            for (uint32_t b = 0; b < CustomHeader::udpAnswerHoldBytes; b++)
                i.WriteU8(m_answerHold.Byte(b));
    }
}

uint32_t
SeqTsHeader::Deserialize(Buffer::Iterator start)
{
    NS_LOG_FUNCTION(this << &start);
    Buffer::Iterator i = start;
    m_seq = i.ReadNtohU32();
    m_pg =  i.ReadNtohU16 ();

    // read IntHeader
    ih.Deserialize(i);
    if (CustomHeader::udpCarriesPathSeq)
    {
        i.Next(IntHeader::GetStaticSize());
        m_pathSeq = i.ReadNtohU16();
        if (CustomHeader::udpTailCounts > 0)
        {
            m_tailFlags = i.ReadU8();
            if (CustomHeader::CarriesTailCounts(m_tailFlags))
                for (uint32_t k = 0; k < CustomHeader::udpTailCounts; k++)
                    m_tailCounts[k] = i.ReadNtohU16();
        }
        if (CustomHeader::udpAnswerHoldBytes > 0)
        {
            uint8_t bytes[kSpineBitmapBytes] = {};
            for (uint32_t b = 0; b < CustomHeader::udpAnswerHoldBytes; b++)
                bytes[b] = i.ReadU8();
            m_answerHold = SpineBitmap::FromBytes(bytes, CustomHeader::udpAnswerHoldBytes);
        }
    }
    return GetSerializedSize ();
}

} // namespace ns3
