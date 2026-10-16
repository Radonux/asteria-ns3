#ifndef SPINE_BITMAP_H
#define SPINE_BITMAP_H

#include <array>
#include <cstdint>

namespace ns3 {

// The most spines a leaf-spine fabric names: a data packet's identification
// names its requested and carrying spines in a byte each.
constexpr uint32_t kMaxSpineCount = 256;
// The bytes of a bitmap over kMaxSpineCount spines, bit k of byte k / 8 for
// spine k mod 8.
constexpr uint32_t kSpineBitmapBytes = kMaxSpineCount / 8;

// The bytes a bitmap over spines spines takes on the wire.
constexpr uint32_t SpineBitmapBytes(uint32_t spines){
	return (spines + 7) / 8;
}

// The bytes a field naming one of spines spines or none takes on the wire:
// one, with 255 for none, below 256 spines, else two, with 0xffff.
constexpr uint32_t SpineFieldBytes(uint32_t spines){
	return spines <= 255 ? 1 : 2;
}

// A set of spines, bit k for spine k, as a header's bitmap fields carry it:
// over at most 8 spines the one byte those fields carried before more
// spines were named.
class SpineBitmap {
public:
	SpineBitmap() : m_bytes{} {}
	// The first byte alone, spines 0 to 7.
	static SpineBitmap FromByte(uint8_t byte){
		SpineBitmap bitmap;
		bitmap.m_bytes[0] = byte;
		return bitmap;
	}
	static SpineBitmap FromBytes(const uint8_t *bytes, uint32_t count){
		SpineBitmap bitmap;
		for (uint32_t i = 0; i < count && i < kSpineBitmapBytes; i++)
			bitmap.m_bytes[i] = bytes[i];
		return bitmap;
	}
	void ToBytes(uint8_t *bytes, uint32_t count) const{
		for (uint32_t i = 0; i < count && i < kSpineBitmapBytes; i++)
			bytes[i] = m_bytes[i];
	}
	bool Test(uint32_t spine) const{
		return spine < kMaxSpineCount && ((m_bytes[spine / 8] >> (spine % 8)) & 1);
	}
	void Set(uint32_t spine, bool on = true){
		if (spine >= kMaxSpineCount)
			return;
		const uint8_t bit = static_cast<uint8_t>(1u << (spine % 8));
		m_bytes[spine / 8] = on ? (m_bytes[spine / 8] | bit)
			: static_cast<uint8_t>(m_bytes[spine / 8] & ~bit);
	}
	bool Any() const{
		for (const uint8_t byte : m_bytes)
			if (byte != 0)
				return true;
		return false;
	}
	uint8_t Byte(uint32_t index) const{
		return index < kSpineBitmapBytes ? m_bytes[index] : 0;
	}
	SpineBitmap &operator|=(const SpineBitmap &other){
		for (uint32_t i = 0; i < kSpineBitmapBytes; i++)
			m_bytes[i] |= other.m_bytes[i];
		return *this;
	}
	SpineBitmap operator^(const SpineBitmap &other) const{
		SpineBitmap out;
		for (uint32_t i = 0; i < kSpineBitmapBytes; i++)
			out.m_bytes[i] = static_cast<uint8_t>(m_bytes[i] ^ other.m_bytes[i]);
		return out;
	}
	bool operator==(const SpineBitmap &other) const{
		return m_bytes == other.m_bytes;
	}
	bool operator!=(const SpineBitmap &other) const{
		return !(*this == other);
	}

private:
	std::array<uint8_t, kSpineBitmapBytes> m_bytes;
};

} // namespace ns3

#endif /* SPINE_BITMAP_H */
