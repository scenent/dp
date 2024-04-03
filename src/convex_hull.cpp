#include "convex_hull/convex_hull.h"

#include <algorithm>

namespace dp {
	namespace convex_hull {
		bool is_ccw(const Vector2& a, const Vector2& b, const Vector2& c) {
			return (b.y - a.y) * (c.x - b.x) > (b.x - a.x) * (c.y - b.y);
		}

		std::vector<Vector2> convexHull(std::vector<Vector2> _points) {
			int n = _points.size(), k = 0;
			if (n <= 3) {
				return _points;
			}
			std::vector<Vector2> H(2 * n);
			std::sort(_points.begin(), _points.end(), [](const Vector2& a, const Vector2& b) {
				return (a.x < b.x) || (a.x == b.x && a.y < b.y);
				});
			for (int i = 0; i < n; ++i) {
				while (k >= 2 && !is_ccw(H[k - 2], H[k - 1], _points[i])) k--;
				H[k++] = _points[i];
			}
			for (int i = n - 1, t = k + 1; i > 0; --i) {
				while (k >= t && !is_ccw(H[k - 2], H[k - 1], _points[i - 1])) k--;
				H[k++] = _points[i - 1];
			}
			H.resize(k - 1);
			return H;
		}
	}
}