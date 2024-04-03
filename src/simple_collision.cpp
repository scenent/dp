#include "simple_collision/simple_collision.h"

namespace dp {
namespace simple_collision {

bool intersect(const Vector2& _a, const Rect& _b) {
    return intersect(_b, _a);
}

bool intersect(const Vector2& _a, const Circle& _b) {
    return intersect(_b, _a);
}

bool intersect(const Vector2& _a, const Line& _b) {
    return intersect(_b, _a);
}

bool intersect(const Rect& _a, const Rect& _b) {
    if (_a.position.x < _b.position.x + _b.size.x &&
        _a.position.x + _a.size.x > _b.position.x &&
        _a.position.y < _b.position.y + _b.size.y &&
        _a.position.y + _a.size.y > _b.position.y) {
        return true;
    }
    return false;
}

bool intersect(const Rect& _a, const Circle& _b) {
    Vector2 circleDistance;
    circleDistance.x = std::abs(_b.position.x - _a.position.x);
    circleDistance.y = std::abs(_b.position.y - _a.position.y);

    if (circleDistance.x > (_a.size.x / 2 + _b.radius)) { return false; }
    if (circleDistance.y > (_a.size.y / 2 + _b.radius)) { return false; }

    if (circleDistance.x <= (_a.size.x / 2)) { return true; }
    if (circleDistance.y <= (_a.size.y / 2)) { return true; }

    float cornerDistance_sq = std::pow((circleDistance.x - _a.size.x / 2), 2) +
        std::pow((circleDistance.y - _a.size.y / 2), 2);

    return (cornerDistance_sq <= std::pow(_b.radius, 2));
}

bool intersect(const Circle& _a, const Circle& _b) {
    return std::sqrt(std::pow(_b.position.x - _a.position.x, 2) + std::pow(_b.position.y - _a.position.y, 2)) <= (_a.radius + _b.radius);
}

bool intersect(const Circle& _a, const Rect& _b) {
	return intersect(_b, _a);
}


bool intersect(const Rect& _a, const Vector2& _b) {
    Vector2 min = _a.position;
    Vector2 max = _a.position + _a.size;
    return (_b.x >= min.x && _b.y >= min.y && _b.x <= max.x && _b.y <= max.y);
}

bool intersect(const Rect& _a, const Line& _b) {
    return intersect(_b, _a);
}

bool intersect(const Circle& _a, const Vector2& _b) {
    Vector2 diff = _a.position - _b;
    return (diff.x * diff.x + diff.y * diff.y <= _a.radius * _a.radius);
}
bool intersect(const Circle& _a, const Line& _b) {
    return intersect(_b, _a);
}

bool intersect(const Line& _a, const Line& _b) {
    Vector2 b = _a.p2 - _a.p1;
    Vector2 d = _b.p2 - _b.p1;
    float bDotDPerp = b.x * d.y - b.y * d.x;

    if (bDotDPerp == 0)
        return false;

    Vector2 c = _b.p1 - _a.p1;
    float t = (c.x * d.y - c.y * d.x) / bDotDPerp;
    if (t < 0.0f || t > 1.0f)
        return false;

    float u = (c.x * b.y - c.y * b.x) / bDotDPerp;
    if (u < 0.0f || u > 1.0f)
        return false;

    return true;
}

bool intersect(const Line& _a, const Rect& _b) {
    return intersect(_a, Line{ _b.position, _b.position + Vector2{_b.size.x, 0} })
        || intersect(_a, Line{ _b.position, _b.position + Vector2{0, _b.size.y} })
        || intersect(_a, Line{ _b.position + _b.size, _b.position + Vector2{_b.size.x, 0} })
        || intersect(_a, Line{ _b.position + _b.size, _b.position + Vector2{0, _b.size.y} });
}

bool intersect(const Line& _a, const Circle& _b) {
    Vector2 ab = _a.p2 - _a.p1;
    Vector2 ac = _b.position - _a.p1;
    float t = ac.dot(ab) / ab.dot(ab);
    if (t < 0.0f) t = 0.0f;
    else if (t > 1.0f) t = 1.0f;

    Vector2 h = (_a.p1 + ab * t) - _b.position;
    return h.dot(h) <= _b.radius * _b.radius;
}

bool intersect(const Line& _a, const Vector2& _b) {
    Vector2 ba = _a.p1 - _a.p2;
    Vector2 bb = _b - _a.p2;
    float cross = ba.x * bb.y - ba.y * bb.x;
    return abs(cross) < FLT_EPSILON;
}


}
}