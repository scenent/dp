#ifndef DP_TYPES_TO_STRING_H
#define DP_TYPES_TO_STRING_H

#include <string>
#include <iostream>

#include "config.h"
#include "core/data_types.h"

namespace dp {
	static std::ostream & operator<<(std::ostream & _stream, const Vector2 & _other) {
		_stream << "Vector2(" << _other.x << ", " << _other.y << ")";
		return _stream;
	}
	static std::ostream& operator<<(std::ostream& _stream, const Matrix22& _other) {
		_stream << "Matrix22(" << _other.m[0][0] << ", " << _other.m[0][1] << ", " << _other.m[1][0] << ", " << _other.m[1][1] << ")";
		return _stream;
	}
	static std::ostream& operator<<(std::ostream& _stream, const AABB& _other) {
		_stream << "AABB(" << _other.x << ", " << _other.y << ", " << _other.w << ", " << _other.h << ")";
		return _stream;
	}
	namespace simple_collision {
		static std::ostream& operator<<(std::ostream& _stream, const Rect& _other) {
			_stream << "Rect(Position = " << _other.position << ", Size = " << _other.size << ")";
			return _stream;
		}
		static std::ostream& operator<<(std::ostream& _stream, const Circle& _other) {
			_stream << "Circle(Position = " << _other.position << ", Radius = " << _other.radius << ")";
			return _stream;
		}
		static std::ostream& operator<<(std::ostream& _stream, const Line& _other) {
			_stream << "Line(P1 = " << _other.p1 << ", P2 = " << _other.p2 << ")";
			return _stream;
		}
	}
}


#endif