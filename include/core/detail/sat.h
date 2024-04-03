#ifndef DP_SAT_H
#define DP_SAT_H

#include "core/body/rigid_body.h"
#include "config.h"

#include <algorithm>
#include <vector>
#include <cmath>
#include <functional>
#include <float.h>

namespace dp {
	namespace detail {
		// 점들을 한 축에 투영한 결과를 저장
		struct SATProjection {
			float min, max;
		};
		// SAT 알고리즘의 결과값
		struct SATResult {
			Vector2 mtv;
			bool is_intersect;
			Vector2 point;
		};
		// 주어진 두 인덱스가 이루는 벡터의 법선을 반환
		Vector2 perpendicular(const std::vector<Vector2>& points, int i0, int i1);
		// 점들을 투영
		SATProjection project(const std::vector<Vector2>& points, const Vector2& normal);
		// 두 투영 결과가 겹치는 정도를 반환
		float overlap(const SATProjection& p0, const SATProjection& p1);
		// 두 폴리곤의 충돌 정보를 반환
		SATResult SAT(const DPRigidBody& _a, const DPRigidBody& _b);
	}
}



#endif