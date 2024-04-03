#ifndef DP_CONVEX_HULL_H
#define DP_CONVEX_HULL_H

#include "core/data_types.h"

namespace dp {
	namespace convex_hull {
		// �� ���� �̷�� �� ������ �ݽð� ������ ���ϴ��� �Ǵ� 
		bool is_ccw(const Vector2& a, const Vector2& b, const Vector2& c);
		// Point Cloud�� ����� ���� �ܰ����� ���� �����Ͽ� ��ȯ
		std::vector<Vector2> convexHull(std::vector<Vector2> _points);
	}
}

#endif