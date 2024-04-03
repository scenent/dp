#include "core/detail/gjk_epa.h"

namespace dp{
namespace detail{

const Vector2 tripleProduct(const Vector2& a, const Vector2& b, const Vector2& c) {
	Vector2 r;
	float ac = a.x * c.x + a.y * c.y;
	float bc = b.x * c.x + b.y * c.y;
	r.x = b.x * ac - a.x * bc;
	r.y = b.y * ac - a.y * bc;
	return r;
}

const Vector2 getCenter(std::vector<Vector2> vertices) {
	Vector2 avg = Vector2(0.0f, 0.0f);
	for (size_t i = 0; i < vertices.size(); i++) {
		avg.x += vertices[i].x;
		avg.y += vertices[i].y;
	}
	avg.x /= vertices.size();
	avg.y /= vertices.size();
	return avg;
}

const size_t support(const std::vector<Vector2>& vertices, const Vector2& d) {
	float maxProduct = Vector2::dot(d, vertices[0]);
	size_t index = 0;
	for (size_t i = 1; i < vertices.size(); i++) {
		float product = Vector2::dot(d, vertices[i]);
		if (product > maxProduct) {
			maxProduct = product;
			index = i;
		}
	}
	return index;
}

const Vector2 support2(const std::vector<Vector2>& vertices1, const std::vector<Vector2>& vertices2, const Vector2& d) {
	size_t i = support(vertices1, d);
	size_t j = support(vertices2, -d);
	return vertices1[i] - vertices2[j];
}


bool GJK(const std::vector<Vector2>& vertices1, const std::vector<Vector2>& vertices2, std::vector<Vector2>& __out simplexOut) {
	if (simplexOut.size() != 3) {
		simplexOut = { {}, {}, {} };
	}
	size_t index = 0;
	Vector2 a, b, c, d, ao, ab, ac, abperp, acperp;
	Vector2 position1 = getCenter(vertices1);
	Vector2 position2 = getCenter(vertices2);
	d = position1 - position2;
	if ((d.x == 0) && (d.y == 0))
		d.x = 1.f;
	a = simplexOut[0] = support2(vertices1, vertices2, d);
	if (Vector2::dot(a, d) <= 0)
		return 0;
	d = -a;

	while (1) {
		a = simplexOut[++index] = support2(vertices1, vertices2, d);
		if (Vector2::dot(a, d) <= 0)
			return false;
		ao = -a;
		if (index < 2) {
			b = simplexOut[0];
			ab = b - a;
			d = tripleProduct(ab, ao, ab);
			if (d.length_squared() == 0)
				d = ab.perpendicular();
			continue;
		}

		b = simplexOut[1];
		c = simplexOut[0];
		ab = b - a;
		ac = c - a;
		acperp = tripleProduct(ab, ac, ac);
		if (Vector2::dot(acperp, ao) >= 0) {
			d = acperp;
		}
		else {
			abperp = tripleProduct(ac, ab, ab);
			if (Vector2::dot(abperp, ao) < 0)
				return true;
			simplexOut[0] = simplexOut[1];
			d = abperp;
		}
		simplexOut[1] = simplexOut[2];
		index--;
	}
	return false;
}

GJK_EPAResult EPA(const std::vector<Vector2>& vertices1, const std::vector<Vector2>& vertices2, std::vector<Vector2> polytope) {
	static size_t maxIteration = 100;
	size_t iterationCount = 0;
	while (iterationCount < maxIteration) {
		float smallestDist = std::numeric_limits<float>::infinity();
		Vector2 edgeNormal;
		std::vector<Vector2>::iterator edge;

		for (auto it = polytope.begin(); it != polytope.end(); ++it) {
			Vector2 a = *it;
			Vector2 b = (it + 1 == polytope.end()) ? polytope.front() : *(it + 1);

			Vector2 e = b - a;
			Vector2 oa = a;

			Vector2 n = { -e.y, e.x };
			float l = sqrt(n.x * n.x + n.y * n.y);
			n.x /= l;
			n.y /= l;

			float d = n.x * oa.x + n.y * oa.y;

			if (d < smallestDist) {
				smallestDist = d;
				edgeNormal = n;
				edge = it;
			}
		}

		Vector2 p = support2(vertices1, vertices2, edgeNormal);
		float d = Vector2::dot(p, edgeNormal);

		if (d - smallestDist < 0.00001) {
			return { edgeNormal * smallestDist };
		}
		else {
			polytope.insert(edge + 1, p);
		}
		iterationCount++;
	}
	return { Vector2() };
}

}
}