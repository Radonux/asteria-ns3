#ifndef RECEIVER_HOLD_H
#define RECEIVER_HOLD_H

#include <stdint.h>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

namespace ns3 {

// The spines an acknowledgement's receiver_hold bitmap can name: bit k is
// spine k.
constexpr uint32_t kReceiverHoldSpines = 8;

// The receiving host's pooled hold under SprayReceiverHold. Per sender leaf
// and spine it counts the data packets that arrived and the packets the
// path sequences show missing (SpineSequenceReceiver), and per spine it keeps
// the sender leaves with a missing packet on it since that spine was last
// released.
//
// A packet missing on spine k from sender leaf a holds k: for every sender
// leaf (host scope) if at least autoLeaves leaves have a missing packet on
// k, else for a alone (leaf scope); a leaf-scope hold widens to host scope as
// soon as a missing packet brings k's leaves to autoLeaves. A hold ends after
// cleanArrivals consecutive arrivals on k without a missing packet, from a in
// leaf scope or from any leaf in host scope; a missing packet restarts the
// count, and a late arrival neither counts nor restarts it. The release of a
// leaf-scope hold on k takes a out of k's leaves, and a host-scope release
// clears them. This is the rule of AutoLossHold, on the receiver's arrivals
// and immediately at each missing packet instead of per loss interval.
class ReceiverHold {
public:
	enum class Scope : uint8_t {
		None = 0,
		Leaf,
		Host,
	};
	// Called as each interval is closed, once per sender leaf seen, with
	// the leaf, the time the interval ended at and, per spine, the arrivals
	// and the missing packets counted in it and the scope its hold has as the
	// interval closes. A later interval that ended before the next arrival
	// counted nothing and is not passed.
	using IntervalCallback = std::function<void(uint32_t leaf, uint64_t endNs,
		const std::vector<uint32_t> &arrivals, const std::vector<uint32_t> &lost,
		const std::vector<Scope> &holds)>;
	// Over spines, at most kReceiverHoldSpines; autoLeaves is at least 2 and
	// cleanArrivals above zero. intervalNs zero closes no interval.
	ReceiverHold(uint32_t spines, uint32_t autoLeaves, uint32_t cleanArrivals,
		uint64_t intervalNs);
	// A data packet from sender leaf leaf carried by spine at nowNs, which
	// showed gaps packets missing before it, or arrived late.
	void OnArrival(uint32_t leaf, uint32_t spine, uint32_t gaps, bool late,
		uint64_t nowNs);
	// The trimmed header of a data packet from sender leaf leaf carried by
	// spine at nowNs, which showed gaps packets missing before it. A trim is
	// not a missing packet: it is counted neither as an arrival nor as a clean
	// one, and only the packets missing before it count.
	void OnTrim(uint32_t leaf, uint32_t spine, uint32_t gaps, uint64_t nowNs);
	// Whether spine is held for sender leaf leaf, at either scope; and the
	// bitmap of the spines held for it, bit k for spine k.
	bool Holds(uint32_t leaf, uint32_t spine) const;
	uint8_t Bitmap(uint32_t leaf) const;
	// Whether spine is held in host scope, the sender leaves with a missing
	// packet on it, and the clean arrivals the hold that holds it for leaf
	// has counted.
	bool HostHolds(uint32_t spine) const;
	uint32_t LossLeaves(uint32_t spine) const;
	uint32_t Clean(uint32_t leaf, uint32_t spine) const;
	void SetIntervalCallback(IntervalCallback callback);

private:
	struct Leaf {
		explicit Leaf(uint32_t spines);
		std::vector<uint32_t> arrivals;
		std::vector<uint32_t> lost;
		std::vector<bool> held;
		std::vector<uint32_t> clean;
	};
	Leaf &LeafCounts(uint32_t leaf);
	// gaps packets missing on spine from leaf, which counts holds.
	void CountMissing(Leaf &counts, uint32_t leaf, uint32_t spine,
		uint32_t gaps);
	void HoldHost(uint32_t spine);
	void AdvanceInterval(uint64_t nowNs);

	uint32_t m_spines;
	uint32_t m_autoLeaves;
	uint32_t m_cleanArrivals;
	uint64_t m_intervalNs;
	uint64_t m_intervalEndNs;
	// Ordered, so that the intervals are passed in leaf order.
	std::map<uint32_t, Leaf> m_leaves;
	std::vector<std::set<uint32_t>> m_lossLeaves;
	std::vector<bool> m_hostHeld;
	std::vector<uint32_t> m_hostClean;
	IntervalCallback m_onInterval;
};

// The sending host's record of the receivers' holds under SprayReceiverHold:
// per destination leaf and spine, the receivers whose latest acknowledgement
// set the spine's bit. A spine is held toward a leaf while any receiver on it
// holds it; an acknowledgement from a receiver with the bit clear takes that
// receiver out, so the hold ends only by the receivers that set it and never
// by the sender's own clean sends. Each held (leaf, spine) is sent one
// sentinel per sentinel interval for the host, unless the sender's own loss
// hold holds it and sends its own.
class ReceiverHoldMarks {
public:
	ReceiverHoldMarks(uint32_t spines, uint64_t sentinelIntervalNs);
	// The receiver_hold bitmap of an acknowledgement from receiver, a host on
	// leaf.
	void OnBitmap(uint32_t leaf, uint32_t receiver, uint8_t bitmap);
	bool Holds(uint32_t leaf, uint32_t spine) const;
	// Whether a sentinel toward leaf is due on a spine held for it and not
	// held by localHolds, and which; the lowest such spine is taken and its
	// sentinel interval restarts now.
	bool TakeSentinel(uint32_t leaf, uint64_t nowNs,
		const std::function<bool(uint32_t)> &localHolds, uint8_t &spine);
	uint32_t Spines() const;

private:
	struct Leaf {
		explicit Leaf(uint32_t spines);
		std::vector<std::set<uint32_t>> receivers;
		std::vector<uint64_t> sentinelNs;
	};
	uint32_t m_spines;
	uint64_t m_sentinelIntervalNs;
	std::unordered_map<uint32_t, Leaf> m_leaves;
};

// A selector's view of ReceiverHoldMarks toward its destination leaf; it
// holds nothing until Set.
class ReceiverHoldView {
public:
	void Set(std::shared_ptr<ReceiverHoldMarks> marks, uint32_t leaf);
	// Whether it is set.
	bool Active() const;
	void OnBitmap(uint32_t receiver, uint8_t bitmap);
	bool Holds(uint32_t spine) const;
	// Whether every spine is held by the marks or by localHolds.
	bool HoldsEvery(const std::function<bool(uint32_t)> &localHolds) const;
	bool TakeSentinel(uint64_t nowNs,
		const std::function<bool(uint32_t)> &localHolds, uint8_t &spine);

private:
	std::shared_ptr<ReceiverHoldMarks> m_marks;
	uint32_t m_leaf = 0;
};

} // namespace ns3

#endif /* RECEIVER_HOLD_H */
