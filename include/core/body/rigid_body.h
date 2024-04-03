#ifndef DP_RIGID_BODY_H
#define DP_RIGID_BODY_H

#include "core/shape/shape.h"
#include "core/data_types.h"
#include "config.h"

#include <string>


namespace dp {
	class DPWorld;

	enum class DPRigidBodyMode {
		Rigid, Static, Character
	};

	class DPRigidBody {
		friend DPWorld;
	private:
		std::string m_name;
		DPRigidBodyMode m_mode = DPRigidBodyMode::Rigid;
	private:
		DPShape* m_shape = nullptr;
	private:
		AABB m_AABB{};

		Vector2 m_force{};
		Vector2 m_velocity{};
		Vector2 m_position{};

		dpfloat m_torque{};
		dpfloat m_angularVelocity{};
		dpfloat m_rotation{};

		dpfloat m_inertia{};
		dpfloat m_invInertia{};
		dpfloat m_mass{};
		dpfloat m_invMass{};

		dpfloat m_dampingLinear{};
		dpfloat m_dampingAngular{};

		dpfloat m_frictionCoefficient = 1.0f;
	public:
		DPRigidBody();
		void setShape(DPShape* _shape);
		DPShape* getShape() const;
		void setMode(const DPRigidBodyMode& _mode);
		const DPRigidBodyMode& getMode();
		void applyForce(const Vector2& _force);
		void applyImpulse(const Vector2& _impulse, const Vector2& _contact);
		void setPosition(const Vector2& _pos);
		void addPosition(const Vector2& _pos);
		void setRotation(const dpfloat& _rot);
		void addRotation(const dpfloat& _rot);
		void setInertia(const dpfloat& _i);
		void setMass(const dpfloat& _m);
		void setAABB(const struct AABB& _aabb);
		const dpfloat& getRotation()const;
		const dpfloat& getAngularVelocity()const;
		const Vector2& getPosition()const;
		const Vector2& getVelocity()const;
		const Vector2 getCentroid() const;
		const dpfloat& getMass() const;
		const dpfloat& getInvMass() const;
		const dpfloat& getInertia() const;
		const dpfloat& getInvInertia() const;
		const AABB& getAABB() const;
	public:
		const std::vector<Vector2> toVertices()const;
	public:
		const std::string& getName()const;
	};





}

#endif