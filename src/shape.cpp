#include "core/shape/shape.h"

namespace dp {

	DPShape::DPShape()  {

	}

	void DPShape::setOwner(DPRigidBody* _owner) {
		m_owner = _owner;
	}

	const Vector2& DPShape::getPosition() {
		return m_position;
	}

	const Matrix22& DPShape::getRotation() {
		return m_rotation;
	}
	const DPShapeType& DPShape::getType() {
		return m_type;
	}
}