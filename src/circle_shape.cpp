#include "core/shape/circle_shape.h"

namespace dp {
	DPCircleShape::DPCircleShape() {
		m_type = DPShapeType::Circle;
	}

	void DPCircleShape::setRotation(const dpfloat& _radian) {
		m_rotation.set(_radian);
	}

	const dpfloat& DPCircleShape::getRadius() {
		return m_radius;
	}

	void DPCircleShape::setRadius(const dpfloat& _radius) {
		m_radius = _radius;
	}
}