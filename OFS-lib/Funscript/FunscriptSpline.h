#pragma once
#include "OFS_Profiling.h"
#include "FunscriptAction.h"
#include <vector>
#include "glm/gtx/spline.hpp"
#include "glm/gtc/epsilon.hpp"
#include "glm/gtc/constants.hpp"

class FunscriptSpline
{
private:
	int32_t cacheIdx = 0;
	static inline float interpolate(const FunscriptArray& actions, int32_t i, float time) noexcept
	{
		OFS_PROFILE(__FUNCTION__);
		int i0 = glm::clamp<int>(i, 0, actions.size() - 1);
		int i1 = glm::clamp<int>(i + 1, 0, actions.size() - 1);

		const FunscriptAction &a0 = actions[i0];
		const FunscriptAction &a1 = actions[i1];
		
		float x0 = a0.atS;
		float y0 = a0.pos / 100.f;

		float x1 = a1.atS;
		float y1 = a1.pos / 100.f;

		if ((a0.tangentMode == HandleMode::None || a0.tangentMode == HandleMode::In) && (a1.tangentMode == HandleMode::None || a1.tangentMode == HandleMode::Out))
			return glm::mix(y0, y1, (time - x0) / (x1 - x0));

		float m0 = a0.tangentMode == HandleMode::Out || a0.tangentMode == HandleMode::Both ? a0.outTangent : 0;
		float m1 = a1.tangentMode == HandleMode::In || a1.tangentMode == HandleMode::Both ? a1.inTangent : 0;

		float w0 = a0.weightMode == HandleMode::Out || a0.weightMode == HandleMode::Both ? a0.outWeight : 1 / 3.f;
		float w1 = a1.weightMode == HandleMode::In || a1.weightMode == HandleMode::Both ? a1.inWeight : 1 / 3.f;

		return interpolate_bezier(time, x0, y0, m0, w0, x1, y1, m1, w1);
	}

	static float interpolate_bezier(float x, float x0, float y0, float m0, float w0, float x1, float y1, float m1, float w1)
	{
		float dx = x1 - x0;
		float dy = y1 - y0;

		m0 = glm::tan(glm::pi<float>() / 2 * glm::clamp(m0, -0.999f, 0.999f));
		m1 = glm::tan(glm::pi<float>() / 2 * glm::clamp(m1, -0.999f, 0.999f));

		w0 = glm::clamp(w0, 0.f, 0.999f);
		w1 = glm::clamp(w1, 0.f, 0.999f);
		float w1s = 1 - w1;

		float ts;
		float t = 0.5f;
		float tx = (x - x0) / dx;

		if (glm::epsilonEqual(w0, 1 / 3.f, FLT_EPSILON * 10) && glm::epsilonEqual(w1, 1 / 3.f, FLT_EPSILON * 10))
		{
			t = tx;
			ts = 1 - t;
		}
		else
		{
			while (true)
			{
				ts = 1 - t;

				float t2 = t * t;
				float ts2 = ts * ts;

				float fg = 3 * ts2 * t * w0 + 3 * ts * t2 * w1s + t2 * t - tx;
				if (glm::abs(fg) < FLT_EPSILON * 10)
					break;

				// third order householder method
				float fpg = 3 * ts2 * w0 + 6 * ts * t * (w1s - w0) + 3 * t2 * (1 - w1s);
				float fppg = 6 * ts * (w1s - 2 * w0) + 6 * t * (1 - 2 * w1s + w0);
				float fpppg = 18 * w0 - 18 * w1s + 6;

				float fg2 = fg * fg;
				float fpg2 = fpg * fpg;
				t -= (6 * fg * fpg2 - 3 * fg2 * fppg) / (6 * fpg2 * fpg - 6 * fg * fpg * fppg + fg2 * fpppg);
			}
		}
		
		float t2 = t * t;
		return y0 + 3 * ts * ts * t * w0 * m0 * dx + 3 * ts * t2 * (dy - w1 * m1 * dx) + t2 * t * dy;
	}

public:
	inline float Sample(const FunscriptArray& actions, float time) noexcept 
	{
		OFS_PROFILE(__FUNCTION__);
		if (actions.size() == 0) { return 0.f; }
		else if (actions.size() == 1) { return actions.front().pos / 100.f; }
		else if (cacheIdx + 1 >= actions.size()) { cacheIdx = 0; }

		if (actions[cacheIdx].atS <= time && actions[cacheIdx + 1].atS >= time) {
			// cache hit!
			return interpolate(actions, cacheIdx, time);
		}
		else if (cacheIdx + 2 < actions.size() && actions[cacheIdx+1].atS <= time && actions[cacheIdx+2].atS >= time) {
			// sort of a cache hit
			cacheIdx += 1;
			return interpolate(actions, cacheIdx, time);
		}
		else {
			// cache miss
			// lookup index
			auto it = actions.upper_bound(FunscriptAction(time, 0));
			if (it == actions.end()) { 
				return actions.back().pos / 100.f; 
			}
			else if (it == actions.begin())			{
				return actions.front().pos / 100.f;
			}

			it--;
			// cache index
			cacheIdx = std::distance(actions.begin(), it);
			return interpolate(actions, cacheIdx, time);
		}

		return 0.f;
	}

	inline static float SampleAtIndex(const FunscriptArray& actions, int32_t index, float time) noexcept
	{
		OFS_PROFILE(__FUNCTION__);
		if (actions.empty()) { return 0.f; }
		if (index + 1 < actions.size())	{
			if (actions[index].atS <= time && actions[index + 1].atS >= time) {
				return interpolate(actions, index, time);
			}
		}

		return actions.back().pos / 100.f;
	}
};