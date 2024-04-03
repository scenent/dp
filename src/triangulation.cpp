#include "triangulation/triangulation.h"

namespace dp {
	namespace triangulation {

		double tripleCross(const Vector2& _a, const Vector2& _b, const Vector2& _c) {
			return (_b.x - _a.x) * (_c.y - _a.y) - (_b.y - _a.y) * (_c.x - _a.x);
		}
		bool isInsideTriangle(const Vector2& p, const Vector2& a, const Vector2& b, const Vector2& c) {
			double area = 0.5 * (-b.y * c.x + a.y * (-b.x + c.x) + a.x * (b.y - c.y) + b.x * c.y);
			double s = 1 / (2 * area) * (a.y * c.x - a.x * c.y + (c.y - a.y) * p.x + (a.x - c.x) * p.y);
			double t = 1 / (2 * area) * (a.x * b.y - a.y * b.x + (a.y - b.y) * p.x + (b.x - a.x) * p.y);

			return s > 0 && t > 0 && (1 - s - t) > 0;
		}
		bool isEar(const std::vector<Vector2>& points, int i) {
			int size = points.size();
			int prev = (i + size - 1) % size;
			int next = (i + 1) % size;
			
			if (tripleCross(points[prev], points[i], points[next]) >= 0)
				return false;
			
			for (int j = 0; j < size; ++j) {
				if (j == prev || j == i || j == next)
					continue;
				if (isInsideTriangle(points[j], points[prev], points[i], points[next]))
					return false;
			}
			return true;
		}

		Vector2 getCentroid(const std::vector<Vector2>& points) {
			Vector2 centroid = { 0, 0 };
			for (const Vector2& p : points) {
				centroid.x += p.x;
				centroid.y += p.y;
			}
			centroid.x /= points.size();
			centroid.y /= points.size();
			return centroid;
		}

		double getCounterClockwiseAngle(const Vector2& p, const Vector2& ref) {
			return atan2(p.y - ref.y, p.x - ref.x);
		}

		bool comparePoints(const Vector2& p1, const Vector2& p2, const Vector2& ref) {
			double angle1 = getCounterClockwiseAngle(p1, ref);
			double angle2 = getCounterClockwiseAngle(p2, ref);
			return angle1 > angle2;
		}
		
		std::vector<std::vector<Vector2>> triangulate(std::vector<Vector2> points) {
			if (points.size() < 3) return {};
			std::vector<std::vector<Vector2>> result;
			unsigned int count = 3;
			while (points.size() > 3) {
				bool earFound = false;
				for (int i = 0; i < points.size(); ++i) {
					if (isEar(points, i)) {
						int prev = (i + points.size() - 1) % points.size();
						int next = (i + 1) % points.size();
						std::vector<Vector2> temp = { points[prev], points[i], points[next] };
						result.push_back(temp);
						count++;
						points.erase(points.begin() + i);
						earFound = true;
						break;
					}
				}
				if (!earFound)
					break;
			}
			std::vector<Vector2> temp = { points.at(0), points.at(1), points.at(2) };
			result.push_back(temp);
			return result;
		}
	}
}