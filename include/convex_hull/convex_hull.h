#ifndef DP_CONVEX_HULL_H
#define DP_CONVEX_HULL_H

#include "core/data_types.h"

namespace dp {
	namespace convex_hull {
		// 세 점이 이루는 두 선분이 반시계 방향을 향하는지 판단 
		bool is_ccw(const Vector2& a, const Vector2& b, const Vector2& c);
		// Point Cloud를 만들고 그중 외곽선의 점만 추출하여 반환
		std::vector<Vector2> convexHull(std::vector<Vector2> _points);
	}
}

#endif