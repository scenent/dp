#ifndef DP_SIMPLE_COLLISION_H
#define DP_SIMPLE_COLLISION_H


#include <string>
#include <iostream>

#include "core/data_types.h"


namespace dp {
	namespace simple_collision {
		// �簢���� ���� �����ϴ��� ��ȯ
		bool intersect(const Vector2& _a, const Rect& _b);
		// ���� ���� �����ϴ��� ��ȯ
		bool intersect(const Vector2& _a, const Circle& _b);
		// ���� ���� �����ϴ��� ��ȯ
		bool intersect(const Vector2& _a, const Line& _b);

		// �簢���� �簢���� �����ϴ��� ��ȯ
		bool intersect(const Rect& _a, const Rect& _b);
		// �簢���� ���� �����ϴ��� ��ȯ
		bool intersect(const Rect& _a, const Circle& _b);
		// �簢���� ���� �����ϴ��� ��ȯ
		bool intersect(const Rect& _a, const Vector2& _b);
		// �簢���� ���� �����ϴ��� ��ȯ
		bool intersect(const Rect& _a, const Line& _b);

		// ���� ���� �����ϴ��� ��ȯ
		bool intersect(const Circle& _a, const Circle& _b);
		// ���� �簢���� �����ϴ��� ��ȯ
		bool intersect(const Circle& _a, const Rect& _b);
		// ���� ���� �����ϴ��� ��ȯ
		bool intersect(const Circle& _a, const Vector2& _b);
		// ���� ���� �����ϴ��� ��ȯ
		bool intersect(const Circle& _a, const Line& _b);

		// ���� ���� �����ϴ��� ��ȯ
		bool intersect(const Line& _a, const Line& _b);
		// ���� �簢���� �����ϴ��� ��ȯ
		bool intersect(const Line& _a, const Rect& _b);
		// ���� ���� �����ϴ��� ��ȯ
		bool intersect(const Line& _a, const Circle& _b);
		// �� ���� ���� �ִ��� ��ȯ
		bool intersect(const Line& _a, const Vector2& _b);
	}
}









#endif