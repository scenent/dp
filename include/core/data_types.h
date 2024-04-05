#ifndef DP_DATA_TYPES_H
#define DP_DATA_TYPES_H

#include <vector>
#include <cmath>

#include "config.h"

namespace dp {
	struct AABB {
		float x, y, w, h;
		float getMinX() const { return x; }
		float getMaxX() const { return x + w; }
	};

	struct Vector2 {
		dpfloat x, y;
		Vector2() : x(0.0f), y(0.0f) {};
		Vector2(const dpfloat& _x, const dpfloat& _y) : x(_x), y(_y) {};
		Vector2& operator=(const Vector2&) = default;
		Vector2& operator=(Vector2&&) = default;
		Vector2(const Vector2&) = default;
		Vector2(Vector2&&) = default;
		bool operator==(const Vector2& _other)const {
			return (this->x == _other.x) && (this->y == _other.y);
		}
		bool operator!=(const Vector2& _other)const {
			return !(*this == _other);
		}
		Vector2 operator+(const Vector2& _other) const {
			return Vector2(this->x + _other.x, this->y + _other.y);
		}
		Vector2& operator+=(const Vector2& _other) {
			this->x += _other.x; this->y += _other.y;
			return *this;
		}
		Vector2 operator-(const Vector2& _other) const {
			return Vector2(this->x - _other.x, this->y - _other.y);
		}
		Vector2 operator-() const  {
			return Vector2(-this->x, -this->y);
		}
		Vector2& operator-=(const Vector2& _other) {
			this->x -= _other.x; this->y -= _other.y;
			return *this;
		}
		Vector2 operator*(const dpfloat& _other) const {
			return Vector2(this->x * _other, this->y * _other);
		}
		Vector2& operator*=(const dpfloat& _other) {
			this->x *= _other; this->y *= _other;
			return *this;
		}
		Vector2 operator/(const dpfloat& _other) const {
			return Vector2(this->x / _other, this->y / _other);
		}
		Vector2& operator/=(const dpfloat& _other) {
			this->x /= _other; this->y /= _other;
			return *this;
		}
		[[nodiscard]]
		dpfloat dot(const Vector2& _other) const {
			return this->x* _other.x + this->y * _other.y;
		}
		[[nodiscard]]
		static dpfloat dot(const Vector2& _a, const Vector2& _b) {
			return _a.x * _b.x + _a.y * _b.y;
		}
		[[nodiscard]]
		dpfloat cross(const Vector2& _other) const {
			return this->x * _other.y - this->y * _other.x;
		}
		[[nodiscard]]
		static dpfloat cross(const Vector2& _a, const Vector2& _b) {
			return _a.x * _b.y - _a.y * _b.x;
		}
		[[nodiscard]]
		Vector2 cross(const dpfloat& _other) const {
			return Vector2(_other * this->y, -_other * this->x);
		}
		[[nodiscard]]
		static Vector2 cross(const Vector2& _a, const dpfloat& _b) {
			return Vector2(_b * _a.y, -_b * _a.x);
		}
		[[nodiscard]]
		dpfloat length() const {
			return std::sqrt(this->x * this->x + this->y * this->y);
		}
		[[nodiscard]]
		dpfloat length_squared() const {
			return (this->x * this->x + this->y * this->y);
		}
		void normalize() {
			dpfloat len = this->length();
			this->x /= len; this->y /= len;
			return;
		}
		[[nodiscard]]
		Vector2 normalized() const {
			dpfloat len = this->length();
			return Vector2(this->x / len, this->y / len);
		}
		[[nodiscard]]
		dpfloat angle() const {
			return std::atan2(this->y, this->x);
		}
		[[nodiscard]]
		dpfloat degree() const {
			return std::atan2(this->y, this->x) * DP_TO_DEGREE;
		}
		[[nodiscard]]
		dpfloat angle_to(const Vector2& _to) const {
			Vector2 v = _to - (*this);
			return v.angle();
		}
		[[nodiscard]]
		void rotate(const dpfloat& _radian) {
			dpfloat s = std::sin(_radian), c = std::cos(_radian);
			dpfloat _x = x * c - y * s;
			dpfloat _y = x * s + y * c;
			this->x = _x;
			this->y = _y;
			return;
		}
		[[nodiscard]]
		Vector2 rotated(const dpfloat& _radian) const {
			dpfloat s = std::sin(_radian), c = std::cos(_radian);
			dpfloat _x = x * c - y * s;
			dpfloat _y = x * s + y * c;
			return { _x, _y };
		}
		[[nodiscard]]
		static dpfloat angle_between(const Vector2& _a, const Vector2& _b) {
			dpfloat d = Vector2::dot(_a, _b);
			dpfloat ll = _a.length() * _b.length();
			dpfloat cs = d / ll;
			return acos(cs);
		}
		[[nodiscard]]
		static Vector2 project(const Vector2& _a, const Vector2& _b) {
			dpfloat dp = Vector2::dot(_a, _b);
			dpfloat mag_squared = _b.x * _b.x + _b.y * _b.y;
			return Vector2((dp / mag_squared) * _b.x, (dp / mag_squared) * _b.y);
		}
		[[nodiscard]]
		static Vector2 reflect(const Vector2& _in, const Vector2& _n) {
			dpfloat dp = Vector2::dot(_in, _n);
			return Vector2(_in.x - 2.0f * dp * _n.x, _in.y - 2.0f * dp * _n.y);
		}
		[[nodiscard]]
		Vector2 perpendicular() const {
			return { -this->y, this->x };
		}
	};

	struct Matrix22 {
		union {
			dpfloat m[2][2];
			dpfloat v[4];
			struct {
				dpfloat   _00, _01;
				dpfloat   _10, _11;
			};
		};
	public:
		Vector2 operator*(const Vector2& v) const {
			Vector2 temp;
			temp.x = this->_00 * v.x + this->_01 * v.y;
			temp.y = this->_10 * v.x + this->_11 * v.y;
			return temp;
		}

		Matrix22 operator*(const dpfloat& _scala)const {
			Matrix22 temp;
			temp._00 = _scala * this->_00;
			temp._01 = _scala * this->_01;
			temp._10 = _scala * this->_10;
			temp._11 = _scala * this->_11;
			return temp;
		}
		Matrix22 operator*(const Matrix22& _other) const {
			Matrix22 temp;
			temp._00 = this->_00 * _other._00 + this->_01 * _other._10;
			temp._01 = this->_00 * _other._01 + this->_01 * _other._11;
			temp._10 = this->_10 * _other._00 + this->_11 * _other._10;
			temp._11 = this->_10 * _other._01 + this->_11 * _other._11;
			return temp;
		}
		void operator*=(const dpfloat& _v) {
			(*this) = (*this) * _v;
		}
		void operator*=(const Matrix22& _m) {
			(*this) = (*this) * _m;
		}
		explicit Matrix22() {
			_00 = 1.0f; _01 = 0.0f; _10 = 0.0f; _11 = 1.0f;
		}
		void set() {
			_00 = 1.0f; _01 = 0.0f;
			_10 = 0.0f; _11 = 1.0f;
		}
		void set(const dpfloat& __00, dpfloat __01, dpfloat __10, dpfloat __11) {
			_00 = __00; _01 = __01; _10 = __10; _11 = __11;
		}
		void set(const Vector2& v1, const Vector2& v2) {
			_00 = v1.x;
			_01 = v2.x;
			_10 = v1.y;
			_11 = v2.y;
		}
		void set(dpfloat _theta) {
			const dpfloat c = std::cos(_theta);
			const dpfloat s = std::sin(_theta);
			_00 = c; _01 = -s;
			_10 = s; _11 = c;
		}
		void set(dpfloat _shearX, dpfloat _shearY) {
			_00 = 1.0f; _01 = _shearY;
			_10 = _shearX; _11 = 1.0f;
		}
		bool getBasis(Vector2* __out _basis, const int& _indexStart) const {
			if (_indexStart == 0) {
				_basis->x = _00;
				_basis->y = _10;
			}
			else if (_indexStart == 1) {
				_basis->x = _01;
				_basis->y = _11;
			}
			else {
				return false;
			}
			return true;
		}
		dpfloat determinant() const {
			return _00 * _11 - _01 * _10;
		}
		Matrix22 inversed() const {
			Matrix22 t;
			t.set(_11, -_01, -_10, _00);
			dpfloat det = determinant();
			t = t * (1.0f / det);
			return t;
		}
		Matrix22 transposed() const {
			Matrix22 m;
			m.set(_00, _10, _01, _11);
			return m;
		}
		Matrix22 abs() const {
			Matrix22 m;
			m.set(std::abs(_00), std::abs(_01), std::abs(_10), std::abs(_11));
			return m;
		}
		Vector2 getXAxis() const {
			return Vector2(_00, _10);
		}
		Vector2 getYAxis() const {
			return Vector2(_01, _11);
		}
	};

	namespace simple_collision {
		struct Rect {
			Vector2 position;
			Vector2 size;
			Rect() : position({}), size({}) {};
			Rect(const Vector2& _pos, const Vector2& _size) : position(_pos), size(_size) {};
		};

		struct Circle {
			Vector2 position;
			dpfloat radius;
			Circle() : position({}), radius(0.0) {};
			Circle(const Vector2& _pos, const dpfloat& _radius) : position(_pos), radius(_radius) {};
		};

		struct Line {
			Vector2 p1, p2;
			Line() : p1({}), p2({}) {};
			Line(const Vector2& _p1, const Vector2& _p2) : p1(_p1), p2(_p2) {};
		};
	}
}






#endif
