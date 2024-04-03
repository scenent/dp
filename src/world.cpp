#include "core/world/world.h"
#include "core/detail/sat.h"
#include "core/detail/gjk_epa.h"
#include "core/types_to_string.h"
#include "config.h"
#include "core/detail/sap.h"

#include <iostream>

namespace dp {
	bool lineIntersectionEx(const Vector2& a1, const Vector2& a2, const Vector2& b1, const Vector2& b2, Vector2& intersection) {
		Vector2 b = a2 - a1;
		Vector2 d = b2 - b1;
		b *= 1.01;
		d *= 1.01;
		float bDotDPerp = b.x * d.y - b.y * d.x;

		if (bDotDPerp == 0)
			return false;

		Vector2 c = b1 - a1;
		float t = (c.x * d.y - c.y * d.x) / bDotDPerp;
		if (t < 0 || t > 1)
			return false;

		float u = (c.x * b.y - c.y * b.x) / bDotDPerp;
		if (u < 0 || u > 1)
			return false;

		intersection = a1 + b * t;

		return true;
	}
	Vector2 getCollisionPoint(const DPRigidBody& a, const DPRigidBody& b) {
		std::vector<Vector2> aVertices = a.toVertices();
		std::vector<Vector2> bVertices = b.toVertices();

		for (int i = 0; i < aVertices.size(); i++) {
			for (int j = 0; j < bVertices.size(); j++) {
				Vector2 intersection;
				Vector2 a1 = aVertices[i];
				Vector2 a2 = aVertices[(i + 1) % aVertices.size()];
				Vector2 b1 = bVertices[j];
				Vector2 b2 = bVertices[(j + 1) % bVertices.size()];
				if (lineIntersectionEx(a1, a2, b1, b2, intersection)) {
					return intersection;
				}
			}
		}
		return Vector2(0, 0);
	}

	void DPWorld::addBody(DPRigidBody* _body) {
		m_bodies.emplace_back(_body);
	}
	
	void DPWorld::stepFoce(const dpfloat& _dt) {
		for (auto& body : m_bodies) {
			if (body->m_invMass == 0.0f)
				continue;
			if (body->getMode() == DPRigidBodyMode::Static) {
				continue;
			}
			body->m_velocity -= (body->m_force * body->m_invMass + m_gravity) * _dt;
			body->m_angularVelocity -= (body->m_torque * body->m_invInertia) * _dt;
			
			body->m_velocity *= std::exp(-(body->m_dampingLinear) * _dt);
			body->m_angularVelocity *= std::exp(-(body->m_dampingAngular) * _dt);
		}
	}

	void DPWorld::stepVelocity(const dpfloat& _dt) {
		for (auto& body : m_bodies) {
			if (body->m_invMass == 0.0f)
				continue;
			body->m_position -= body->m_velocity * _dt;
			body->m_rotation -= body->m_angularVelocity * _dt;
		}
	}

	DPRigidBody* DPWorld::getBody(const std::string& _keyName) {
		for (const auto& b : m_bodies) {
			if (b->getName() == _keyName) {
				return b;
			}
		}
		return nullptr;
	}

	void DPWorld::step(const dpfloat& _dt, std::vector<Vector2>& __out _colinfo) {
		using namespace detail;
		
		std::vector<std::tuple<DPRigidBody*, DPRigidBody*, Vector2>> collisionInfo;

#ifdef DP_USING_BROAD_PHASE
#if DP_BROAD_PHASE_ALGORITHM==DP_SWEEP_AND_PRUNE
		for (auto& b : m_bodies) {
			setAABB(b);
		}

		std::vector<std::pair<DPRigidBody*, DPRigidBody*>> sapResult = SAP(m_bodies);
		for (auto& p : sapResult) {
			SATResult result = SAT(*p.first, *p.second);
			if (result.is_intersect) {
				collisionInfo.push_back({ p.first, p.second, result.mtv });
				if (p.first->getMode() == DPRigidBodyMode::Rigid) {
					p.first->addPosition(result.mtv);
				}
				if (p.second->getMode() == DPRigidBodyMode::Rigid) {
					p.second->addPosition(-result.mtv);
				}
			}
		}
#elif DP_BROAD_PHASE_ALGORITHM==DP_BOX2D
#endif
#else
		for (size_t i = 0; i < m_bodies.size(); i++) {
			for (size_t j = i + 1; j < m_bodies.size(); j++) {
				SATResult result = SAT(*m_bodies[i], *m_bodies[j]);
				if (result.is_intersect) {
					collisionInfo.push_back({ m_bodies[i], m_bodies[j], result.mtv });
					if (m_bodies[i]->getMode() == DPRigidBodyMode::Rigid) {
						m_bodies[i]->addPosition(result.mtv);
					}
					if (m_bodies[j]->getMode() == DPRigidBodyMode::Rigid) {
						m_bodies[j]->addPosition(-result.mtv);
					}
				}
			}
		}
#endif
		for (int i = 0; i < m_stepIteration; i++) {
			for (auto& info : collisionInfo) {
				DPRigidBody* a = std::get<0>(info);
				DPRigidBody* b = std::get<1>(info);
				const Vector2& mtv = std::get<2>(info);
				Vector2 collisionPoint = getCollisionPoint(*a, *b);
				_colinfo.emplace_back(collisionPoint);
				Vector2 c1 = (collisionPoint - a->getPosition()).normalized();
				Vector2 c2 = (collisionPoint - b->getPosition()).normalized();
				
				Vector2 relVelocity = a->m_velocity - b->m_velocity;
				dpfloat contactVelocity = Vector2::dot(relVelocity, mtv);
				dpfloat frictionForceMagnitude = contactVelocity * a->m_frictionCoefficient * b->m_frictionCoefficient;
				Vector2 frictionForce = (mtv.normalized() * frictionForceMagnitude);

				if (a->getMode() == DPRigidBodyMode::Rigid) {
					a->applyForce(-frictionForce);
					a->applyImpulse(mtv, c1 * a->getInvInertia());
				}
				if (b->getMode() == DPRigidBodyMode::Rigid) {
					b->applyForce(frictionForce);
					b->applyImpulse(-mtv, c2 * b->getInvInertia());
				}
			}
		}
#if DP_INTERGRATION_METHOD==DP_EULER
		stepVelocity(_dt);
		stepFoce(_dt);
#elif DP_INTERGRATION_METHOD==DP_IMPLICIT_EULER
		stepFoce(_dt);
		stepVelocity(_dt);
#endif

		for (auto& body : m_bodies) {
			body->m_force = { 0, 0 };
			body->m_torque = 0;
			body->m_shape->setPosition(body->m_position);
			body->m_shape->setRotation(body->m_rotation);
		}
	}

	void DPWorld::setGravity(const Vector2& _gravity) {
		m_gravity = _gravity;
	}

}