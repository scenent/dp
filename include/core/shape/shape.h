#ifndef DP_SHAPE_H
#define DP_SHAPE_H

#include "core/data_types.h"
#include "config.h"


namespace dp {
	
	class DPRigidBody;

	enum class DPShapeType {
		Polygon, Circle
	};

	class DPShape {
	protected:
		DPShapeType m_type = DPShapeType::Polygon;
	protected:
		DPRigidBody* m_owner = nullptr;
	protected:
		Vector2 m_position{};
		Matrix22 m_rotation{};
	public:
		DPShape();
		void setOwner(DPRigidBody* _owner);
		virtual void setRotation(const dpfloat& _radian) = 0;
		void setPosition(const Vector2& _pos) { m_position = _pos; }
		const Vector2& getPosition();
		const Matrix22& getRotation();
		const DPShapeType& getType();
	};
}


#endif