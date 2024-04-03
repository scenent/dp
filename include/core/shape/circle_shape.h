#ifndef DP_CIRCLE_SHAPE_H
#define DP_CIRCLE_SHAPE_H



#include "core/shape/shape.h"

namespace dp {
	class DPCircleShape : public DPShape {
	private:
		dpfloat m_radius;
	public:
		DPCircleShape();
		void setRotation(const dpfloat& _radian) override;
		const dpfloat& getRadius();
		void setRadius(const dpfloat& _radius);
	};
}






#endif