#include "core/body/rigid_body.h"
#include "core/shape/shape.h"
#include "core/shape/polygon_shape.h"

namespace dp {

	DPRigidBody::DPRigidBody() {
		this->setMass(1.0f);
		this->setInertia(1.0f);
		m_dampingLinear = 0.1f;
		m_dampingAngular = 0.1f;
	}

	void DPRigidBody::setShape(DPShape* _shape) {
		m_shape = _shape;
		m_shape->setOwner(this);
	}
	DPShape* DPRigidBody::getShape() const {
		return m_shape;
	}

	void DPRigidBody::setMode(const DPRigidBodyMode& _mode) {
		m_mode = _mode;
	}

	const DPRigidBodyMode& DPRigidBody::getMode() {
		return m_mode;
	}
	
	void DPRigidBody::applyForce(const Vector2& _force) {
		m_force += _force * m_invMass;
	}

	void DPRigidBody::applyImpulse(const Vector2& _impulse, const Vector2& _contact) {
		m_velocity -= _impulse * m_invMass;
		m_angularVelocity -= m_invInertia * Vector2::cross(_contact, _impulse);
	}

	void DPRigidBody::setPosition(const Vector2& _pos) {
		m_position = _pos;
	}

	void DPRigidBody::addPosition(const Vector2& _pos) {
		m_position += _pos;
	}

	void DPRigidBody::setRotation(const dpfloat& _rot) {
		m_rotation = _rot;
	}
	void DPRigidBody::addRotation(const dpfloat& _rot) {
		m_rotation += _rot;
	}

	void DPRigidBody::setInertia(const dpfloat& _i) {
		m_inertia = _i;
		m_invInertia = 1.0 / _i;
	}

	void DPRigidBody::setMass(const dpfloat& _m) {
		m_mass = _m;
		m_invMass = 1.0 / _m;
	}
	void DPRigidBody::setAABB(const struct AABB& _aabb) {
		m_AABB = _aabb;
	}
	const dpfloat& DPRigidBody::getRotation() const {
		return m_rotation;
	}
	const dpfloat& DPRigidBody::getAngularVelocity()const {
		return m_angularVelocity;
	}
	const Vector2& DPRigidBody::getPosition()const {
		return m_position;
	}
	const Vector2& DPRigidBody::getVelocity()const {
		return m_velocity;
	}
	const std::string& DPRigidBody::getName()const {
		return m_name;
	}

	const Vector2 DPRigidBody::getCentroid() const {
		std::vector<Vector2> globalPoints = this->toVertices();
		Vector2 center;
		float A = 0.0f;
		float area = 0.0f;
		for (int i = 0; i < globalPoints.size(); i++) {
			Vector2 v0 = globalPoints[i];
			Vector2 v1 = globalPoints[(i + 1) != globalPoints.size() ? (i + 1) : 0];
			float b = Vector2::cross(v0, v1);
			A += b;
			center += (v0 + v1) * b;
		}
		center *= 1.0f / (3.0f * A);
		return center;
	}

	const dpfloat& DPRigidBody::getMass() const {
		return m_mass;
	}
	const dpfloat& DPRigidBody::getInvMass() const {
		return m_invMass;
	}
	const dpfloat& DPRigidBody::getInertia() const {
		return m_inertia;
	}
	const dpfloat& DPRigidBody::getInvInertia() const {
		return m_invInertia;
	}
	const AABB& DPRigidBody::getAABB() const {
		return m_AABB;
	}
	const std::vector<Vector2> DPRigidBody::toVertices()const {
		std::vector<Vector2> points;
		switch (m_shape->getType()) {
		case (DPShapeType::Polygon): {
			for (const auto& v : static_cast<DPPolygonShape*>(m_shape)->getPoints()) {
				Vector2 nv = m_position + m_shape->getRotation() * v;
				points.emplace_back(nv);
			}
			break;
		}
		case (DPShapeType::Circle): {
			/*
			static const dpfloat thetaDensity = 3.141592.0 / 36.0;
			dpfloat radius = static_cast<DPCircleShape*>(m_shape)->getRadius();
			dpfloat theta = 0.0;
			while (theta <= 3.141592){
				points.emplace_back(Vector2(std::cos(theta) * radius, std::sin(theta) * radius));
				theta += thetaDensity;
			}
			break;
			*/
		}
		}
		return points;
	}
}