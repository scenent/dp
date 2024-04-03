#include "core/shape/polygon_shape.h"

namespace dp {
	DPPolygonShape::DPPolygonShape() {
		m_type = DPShapeType::Polygon;
	}
	void DPPolygonShape::setRotation(const dpfloat& _radian) {
		m_rotation.set(_radian);
	}
	void DPPolygonShape::setPoints(const std::vector<Vector2>& _points) {
		m_points = _points;
		m_normals.resize(_points.size());

		for (size_t i1 = 0; i1 < _points.size(); i1++) {
			size_t i2 = i1 + 1 < _points.size() ? i1 + 1 : 0;
			m_normals[i1] = (_points[i2] - _points[i1]).perpendicular();
			m_normals[i1].normalize();
		}
	}
	const std::vector<Vector2>& DPPolygonShape::getPoints() {
		return m_points;
	}
	const std::vector<Vector2>& DPPolygonShape::getNormals() {
		return m_normals;
	}

	const Vector2 DPPolygonShape::getCentroid() {
		Vector2 center;
		float A = 0.0f;
		float area = 0.0f;
		for (int i = 0; i < m_points.size(); i++) {
			Vector2 v0 = m_points[i];
			Vector2 v1 = m_points[(i + 1) != m_points.size() ? (i + 1) : 0];
			float b = Vector2::cross(v0, v1);
			A += b;
			center += (v0 + v1) * b;
		}
		center *= 1.0f / (3.0f * A);
		return center;
	}
}