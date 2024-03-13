#pragma once

#include "OFS_BinarySerialization.h"

#include <cstdint>
#include <limits>

#include "OFS_VectorSet.h"

enum HandleMode : uint8_t {
	None,
	In,
	Out,
	Both
};

struct FunscriptAction
{
public:
	// timestamp as floating point seconds
	// instead of integer milliseconds
	float atS;
	float inTangent;
	float outTangent;
	float inWeight;
	float outWeight;

	int16_t pos;
	HandleMode tangentMode;
	HandleMode weightMode;

	template<typename S>
	void serialize(S& s)
	{
		s.ext(*this, bitsery::ext::Growable{},
			[](S& s, FunscriptAction& o) {
				s.value4b(o.atS);
				s.value2b(o.pos);
				s.value4b(o.inTangent);
				s.value4b(o.inWeight);
				s.value4b(o.outTangent);
				s.value4b(o.outWeight);
				s.value1b(o.tangentMode);
				s.value1b(o.weightMode);
				s.container1b(o.___reserved);
			});
	}

	FunscriptAction() noexcept {
		static_assert(sizeof(FunscriptAction) == 64); 

		this->atS = std::numeric_limits<float>::min();
		this->pos = std::numeric_limits<int16_t>::min();
		this->inTangent = this->outTangent = 0;
		this->inWeight = this->outWeight = 1 / 3.0f;
		this->tangentMode = HandleMode::None;
		this->weightMode = HandleMode::None;
	}

	FunscriptAction(float at, int32_t pos) noexcept
		: FunscriptAction()
	{
		this->atS = at;
		this->pos = pos;
	}

	inline bool operator==(FunscriptAction b) const noexcept {
		return this->atS == b.atS && this->pos == b.pos;
	}

	inline bool operator!=(FunscriptAction b) const noexcept {
		return !(*this == b);
	}

	inline bool operator<(FunscriptAction b) const noexcept {
		return this->atS < b.atS;
	}

	FunscriptAction copy() const noexcept {
		FunscriptAction result(this->atS, this->pos);
		
		result.inTangent = this->inTangent;
		result.outTangent = this->outTangent;
		result.inWeight = this->inWeight;
		result.outWeight = this->outWeight;
		result.tangentMode = this->tangentMode;
		result.weightMode = this->weightMode;

		return result;
	}

private:
	uint8_t ___reserved[38];
};

struct ActionLess
{
	bool operator()(const FunscriptAction& a, const FunscriptAction& b) const noexcept
	{
		return a.atS < b.atS;
	}
};

using FunscriptArray = vector_set<FunscriptAction, ActionLess>;
