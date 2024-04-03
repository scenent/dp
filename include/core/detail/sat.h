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
		// ������ �� �࿡ ������ ����� ����
		struct SATProjection {
			float min, max;
		};
		// SAT �˰����� �����
		struct SATResult {
			Vector2 mtv;
			bool is_intersect;
			Vector2 point;
		};
		// �־��� �� �ε����� �̷�� ������ ������ ��ȯ
		Vector2 perpendicular(const std::vector<Vector2>& points, int i0, int i1);
		// ������ ����
		SATProjection project(const std::vector<Vector2>& points, const Vector2& normal);
		// �� ���� ����� ��ġ�� ������ ��ȯ
		float overlap(const SATProjection& p0, const SATProjection& p1);
		// �� �������� �浹 ������ ��ȯ
		SATResult SAT(const DPRigidBody& _a, const DPRigidBody& _b);
	}
}



#endif