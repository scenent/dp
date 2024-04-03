#ifndef DP_GJK_EPA_H
#define DP_GJK_EPA_H

#include "core/body/rigid_body.h"
#include "config.h"

#include <algorithm>
#include <vector>
#include <cmath>
#include <functional>
#include <float.h>

namespace dp {
	namespace detail {
		// GJK-EPA 알고리즘의 결과
		struct GJK_EPAResult {
			Vector2 mtv;
		};
		// 벡터 삼중곱 수행
		const Vector2 tripleProduct(const Vector2& a, const Vector2& b, const Vector2& c);
		// 폴리곤의 중심 반환
		const Vector2 getCenter(std::vector<Vector2> vertices);
		// GJK 알고리즘을 위한 서포트 함수
		const size_t support(const std::vector<Vector2>& vertices, const Vector2& d);
		// GJK 알고리즘을 위한 서포트 함수의 래퍼
		const Vector2 support2(const std::vector<Vector2>& vertices1, const std::vector<Vector2>& vertices2, const Vector2& d);
		// 두 폴리곤을 받고 충돌 여부를 반환, 선택적으로 simplex 반환
		bool GJK(const std::vector<Vector2>& vertices1, const std::vector<Vector2>& vertices2, std::vector<Vector2>& __out simplexOut);
		// 두 폴리곤과 simplex를 받고 MTV(Minimum Translation Vector) 반환
		GJK_EPAResult EPA(const std::vector<Vector2>& vertices1, const std::vector<Vector2>& vertices2, std::vector<Vector2> polytope);
	}
}



#endif