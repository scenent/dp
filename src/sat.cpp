#include "core/detail/sat.h"

namespace dp {
namespace detail {

Vector2 perpendicular(const std::vector<Vector2>& points, int i0, int i1) {
	if (i1 == points.size()) {
		i1 = 0;
		i0 = points.size() - 1;
	}
	Vector2 temp = (points[i0] - points[i1]).normalized();
	return { -temp.y, temp.x };
}

SATProjection project(const std::vector<Vector2>& points, const Vector2& normal) {
	float min = INFINITY;
	float max = -INFINITY;

	for (int i = 0; i < points.size(); i++) {
		float projection = Vector2::dot(points[i], normal);
		if (projection < min)
			min = projection;
		if (projection > max)
			max = projection;
	}
	return { min, max };
}

float overlap(const SATProjection& p0, const SATProjection& p1) {
	return !(p0.min <= p1.max && p0.max >= p1.min) ? 0.0f : std::min(p0.max, p1.max) - std::max(p0.min, p1.min);
}

SATResult SAT(const DPRigidBody& _a, const DPRigidBody& _b) {
	SATResult result = { {}, true };
	float minOverlap = INFINITY;

	std::vector<Vector2> bodyPoints = _a.toVertices();
	std::vector<Vector2> otherPoints = _b.toVertices();

	std::vector<Vector2> axis;
	axis.assign(bodyPoints.size() + otherPoints.size(), {});
	for (int i = 0; i < bodyPoints.size(); i++)
		axis[i] = perpendicular(bodyPoints, i, i + 1);
	for (int i = 0; i < otherPoints.size(); i++)
		axis[i + bodyPoints.size()] = perpendicular(otherPoints, i, i + 1);


	for (int i = 0; i < bodyPoints.size() + otherPoints.size(); i++) {
		Vector2 a = axis[i];

		SATProjection bodyProjection = project(bodyPoints, a);
		SATProjection otherProjection = project(otherPoints, a);

		float o = overlap(bodyProjection, otherProjection);

		if (!o) {
			return { {}, false };
		}
		else {
			if (o < minOverlap) {
				minOverlap = o;
				result.mtv = a * minOverlap;
			}
		}
	}

	if (Vector2::dot(_a.getCentroid() - _b.getCentroid(), result.mtv) < 0.0f)
		result.mtv *= -1.0f;
	result.is_intersect = true;

	return result;
}


}
}