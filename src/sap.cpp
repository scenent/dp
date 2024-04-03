#include "core/detail/sap.h"

namespace dp {
	namespace detail {
		bool compareBody(DPRigidBody* a, DPRigidBody* b) {
			return a->getAABB().getMinX() < b->getAABB().getMinX();
		}

		std::vector<std::pair<DPRigidBody*, DPRigidBody*>> SAP(std::vector<DPRigidBody*> _bodies) {
			std::sort(_bodies.begin(), _bodies.end(), compareBody);
			std::vector<std::pair<DPRigidBody*, DPRigidBody*>> collisions;
			for (size_t i = 0; i < _bodies.size(); ++i) {
				for (size_t j = i + 1; j < _bodies.size(); ++j) {
					if (_bodies[i]->getAABB().getMaxX() < _bodies[j]->getAABB().getMinX()) {
						break;
					}
					collisions.push_back(std::make_pair(_bodies[i], _bodies[j]));
				}
			}
			return collisions;
		}

		void setAABB(DPRigidBody* __in _body) {
			AABB result{ INFINITY, INFINITY, -INFINITY, -INFINITY };
			std::vector<Vector2> _vertices = _body->toVertices();
			for (int i = 0; i < _vertices.size(); i++) {
				Vector2 p = _vertices[i];
				result.x = p.x < result.x ? p.x : result.x;
				result.y = p.y < result.y ? p.y : result.y;
				result.w = p.x > result.w ? p.x : result.w;
				result.h = p.y > result.h ? p.y : result.h;
			}
			result.w -= result.x;
			result.h -= result.y;
			_body->setAABB(result);
		}
	}
}