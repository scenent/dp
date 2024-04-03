#ifndef DP_SAP
#define DP_SAP

#include <vector>
#include <utility>
#include <algorithm>

#include "core/data_types.h"
#include "core/body/rigid_body.h"

namespace dp {
	namespace detail {
		// 두 AABB를 비교
		bool compareBody(DPRigidBody* a, DPRigidBody* b);
		// Sweep And Prune 필터링 알고리즘을 수행
		std::vector<std::pair<DPRigidBody*, DPRigidBody*>> SAP(std::vector<DPRigidBody*> _bodies);
		// 강체에 AABB를 등록
		void setAABB(DPRigidBody* __in _body);
	}
}







#endif