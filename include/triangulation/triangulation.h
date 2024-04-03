#ifndef DP_TRIANGULATION_H
#define DP_TRIANGULATION_H

#include "core/data_types.h"

namespace dp {
	namespace triangulation {
		// 3개의 벡터를 외적
		double tripleCross(const Vector2& O, const Vector2& A, const Vector2& B);
		// p가 a, b, c가 이루는 삼각형 안에 있는지 검사
		bool isInsideTriangle(const Vector2& p, const Vector2& a, const Vector2& b, const Vector2& c);
		// points의 i번째 점이 귀를 이루는지 검사
		bool isEar(const std::vector<Vector2>& points, int i);
		// 폴리곤의 무게중심 반환
		Vector2 getCentroid(const std::vector<Vector2>& points);
		// 반시계방향으로 벡터의 각도를 반환
		double getCounterClockwiseAngle(const Vector2& p, const Vector2& ref);
		// 두 점을 검사
		bool comparePoints(const Vector2& p1, const Vector2& p2, const Vector2& ref);
		// 한 개의 폴리곤을 최소한의 여러 삼각형으로 나눔
		std::vector<std::vector<Vector2>> triangulate(std::vector<Vector2> points);
	}
}




#endif