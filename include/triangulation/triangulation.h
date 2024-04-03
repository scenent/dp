#ifndef DP_TRIANGULATION_H
#define DP_TRIANGULATION_H

#include "core/data_types.h"

namespace dp {
	namespace triangulation {
		// 3���� ���͸� ����
		double tripleCross(const Vector2& O, const Vector2& A, const Vector2& B);
		// p�� a, b, c�� �̷�� �ﰢ�� �ȿ� �ִ��� �˻�
		bool isInsideTriangle(const Vector2& p, const Vector2& a, const Vector2& b, const Vector2& c);
		// points�� i��° ���� �͸� �̷���� �˻�
		bool isEar(const std::vector<Vector2>& points, int i);
		// �������� �����߽� ��ȯ
		Vector2 getCentroid(const std::vector<Vector2>& points);
		// �ݽð�������� ������ ������ ��ȯ
		double getCounterClockwiseAngle(const Vector2& p, const Vector2& ref);
		// �� ���� �˻�
		bool comparePoints(const Vector2& p1, const Vector2& p2, const Vector2& ref);
		// �� ���� �������� �ּ����� ���� �ﰢ������ ����
		std::vector<std::vector<Vector2>> triangulate(std::vector<Vector2> points);
	}
}




#endif