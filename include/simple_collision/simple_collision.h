#ifndef DP_SIMPLE_COLLISION_H
#define DP_SIMPLE_COLLISION_H


#include <string>
#include <iostream>

#include "core/data_types.h"


namespace dp {
	namespace simple_collision {
		// 사각형이 점을 포함하는지 반환
		bool intersect(const Vector2& _a, const Rect& _b);
		// 원이 점을 포함하는지 반환
		bool intersect(const Vector2& _a, const Circle& _b);
		// 선이 점을 포함하는지 반환
		bool intersect(const Vector2& _a, const Line& _b);

		// 사각형과 사각형이 교차하는지 반환
		bool intersect(const Rect& _a, const Rect& _b);
		// 사각형과 원이 교차하는지 반환
		bool intersect(const Rect& _a, const Circle& _b);
		// 사각형이 점을 포함하는지 반환
		bool intersect(const Rect& _a, const Vector2& _b);
		// 사각형과 선이 교차하는지 반환
		bool intersect(const Rect& _a, const Line& _b);

		// 원과 원이 교차하는지 반환
		bool intersect(const Circle& _a, const Circle& _b);
		// 원과 사각형이 교차하는지 반환
		bool intersect(const Circle& _a, const Rect& _b);
		// 원이 점을 포함하는지 반환
		bool intersect(const Circle& _a, const Vector2& _b);
		// 원과 선이 교차하는지 반환
		bool intersect(const Circle& _a, const Line& _b);

		// 선과 선이 교차하는지 반환
		bool intersect(const Line& _a, const Line& _b);
		// 선과 사각형이 교차하는지 반환
		bool intersect(const Line& _a, const Rect& _b);
		// 선과 원이 교차하는지 반환
		bool intersect(const Line& _a, const Circle& _b);
		// 선 위에 점이 있는지 반환
		bool intersect(const Line& _a, const Vector2& _b);
	}
}









#endif