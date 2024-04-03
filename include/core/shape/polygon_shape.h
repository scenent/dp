#ifndef DP_POLYGON_SHAPE_H
#define DP_POLYGON_SHAPE_H

#include "core/shape/shape.h"

namespace dp {
	class DPPolygonShape : public DPShape {
	private:
		std::vector<Vector2> m_points{};
		std::vector<Vector2> m_normals{};
	public:
		DPPolygonShape();
		void setRotation(const dpfloat& _radian) override;
		void setPoints(const std::vector<Vector2>& _points);
		const std::vector<Vector2>& getPoints();
		const std::vector<Vector2>& getNormals();
		const Vector2 getCentroid();
	};
}







#endif