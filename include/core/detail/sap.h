#ifndef DP_SAP
#define DP_SAP

#include <vector>
#include <utility>
#include <algorithm>

#include "core/data_types.h"
#include "core/body/rigid_body.h"

namespace dp {
	namespace detail {
		// �� AABB�� ��
		bool compareBody(DPRigidBody* a, DPRigidBody* b);
		// Sweep And Prune ���͸� �˰����� ����
		std::vector<std::pair<DPRigidBody*, DPRigidBody*>> SAP(std::vector<DPRigidBody*> _bodies);
		// ��ü�� AABB�� ���
		void setAABB(DPRigidBody* __in _body);
	}
}







#endif