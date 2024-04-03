#ifndef DP_WORLD_H
#define DP_WORLD_H

#include "core/data_types.h"
#include "core/body/rigid_body.h"

namespace dp {
	bool lineIntersectionEx(const Vector2& a1, const Vector2& a2, const Vector2& b1, const Vector2& b2, Vector2& intersection);
	Vector2 getCollisionPoint(const DPRigidBody& a, const DPRigidBody& b);

	class DPWorld {
	private:
		std::vector<DPRigidBody*> m_bodies{};
		bool m_frictionEnabled = true;
		Vector2 m_gravity = { 0, 9.8f };
		size_t m_stepIteration = 2;
	private:
		void stepFoce(const dpfloat& _dt);
		void stepVelocity(const dpfloat& _dt);
	public:
		DPWorld() = default;
		void addBody(DPRigidBody* _body);
		DPRigidBody* getBody(const std::string& _keyName);
		void step(const dpfloat& _dt, std::vector<Vector2>& __out _colinfo);
	public:
		void setGravity(const Vector2& _gravity);
	};
}







#endif