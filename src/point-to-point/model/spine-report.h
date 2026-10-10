#ifndef SPINE_REPORT_H
#define SPINE_REPORT_H

#include <stdint.h>

namespace ns3 {

// What a receiving host tells every sender about the spines under
// SprayPolicy: a grade per spine, 3 the best and 0 the worst, and whether its
// own downlink is congested, which every spine's packets cross alike. A
// receiver issues a report per interval, numbered by sequence, and every
// acknowledgement and repair request it sends carries the latest.
struct SpineReport {
	static constexpr uint32_t kMaxSpines = 32;
	static constexpr uint8_t kTopGrade = 3;

	// The bytes the grades of spines take on the wire, four to a byte.
	static constexpr uint32_t GradeBytes(uint32_t spines){
		return (spines + 3) / 4;
	}

	uint8_t Grade(uint32_t spine) const{
		return (grades[spine / 4] >> (2 * (spine % 4))) & kTopGrade;
	}

	void SetGrade(uint32_t spine, uint8_t grade){
		const uint32_t shift = 2 * (spine % 4);
		grades[spine / 4] = static_cast<uint8_t>(
			(grades[spine / 4] & ~(kTopGrade << shift)) | (grade << shift));
	}

	uint8_t sequence = 0;
	bool edgeCongested = false;
	// Spine k in bits 2(k mod 4) and 2(k mod 4) + 1 of byte k / 4; every spine
	// starts at the top grade.
	uint8_t grades[kMaxSpines / 4] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff};
};

} // namespace ns3

#endif /* SPINE_REPORT_H */
