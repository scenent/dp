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
		// GJK-EPA �˰����� ���
		struct GJK_EPAResult {
			Vector2 mtv;
		};
		// ���� ���߰� ����
		const Vector2 tripleProduct(const Vector2& a, const Vector2& b, const Vector2& c);
		// �������� �߽� ��ȯ
		const Vector2 getCenter(std::vector<Vector2> vertices);
		// GJK �˰����� ���� ����Ʈ �Լ�
		const size_t support(const std::vector<Vector2>& vertices, const Vector2& d);
		// GJK �˰����� ���� ����Ʈ �Լ��� ����
		const Vector2 support2(const std::vector<Vector2>& vertices1, const std::vector<Vector2>& vertices2, const Vector2& d);
		// �� �������� �ް� �浹 ���θ� ��ȯ, ���������� simplex ��ȯ
		bool GJK(const std::vector<Vector2>& vertices1, const std::vector<Vector2>& vertices2, std::vector<Vector2>& __out simplexOut);
		// �� ������� simplex�� �ް� MTV(Minimum Translation Vector) ��ȯ
		GJK_EPAResult EPA(const std::vector<Vector2>& vertices1, const std::vector<Vector2>& vertices2, std::vector<Vector2> polytope);
	}
}



#endif