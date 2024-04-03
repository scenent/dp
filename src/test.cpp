#include "dp.h"
#include <iostream>
using namespace dp;
using namespace dp::simple_collision;

#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

/*
TODO : Implement Restitution
TODO : Implement Perfect Circle Shape
*/

class Example1 : public olc::PixelGameEngine {
private:
	DPWorld* m_world = nullptr;
	std::vector<std::pair<DPRigidBody*, DPPolygonShape*>> bodies{};
	bool running = false;
public:
	Example1() {
		sAppName = "Example1";
	}
public:
	DPRigidBody* addBody(const bool& _static, const Vector2& _pos, const std::vector<Vector2>& _points) {
		DPRigidBody* b1 = new DPRigidBody();
		b1->setMode(DPRigidBodyMode::Rigid);
		b1->setPosition(_pos);
		b1->setInertia(10.0);
		DPPolygonShape* s1 = new DPPolygonShape();
		s1->setPoints(_points);
		if (_static) {
			b1->setMode(DPRigidBodyMode::Static);
		}
		b1->setShape(s1);
		m_world->addBody(b1);
		bodies.push_back({ b1, s1 });
		return b1;
	}

	bool OnUserCreate() override {
		m_world = new DPWorld();

		addBody(false, { 612, 250 }, { {70, 10}, {70, -10}, {-70, -10}, {-70, 10} });// ->setRotation(3.14 / 4);
		addBody(true, { 512, 500 }, { {500, 10}, {500, -10}, {-500, -10}, {-500, 10} });
		addBody(true, { 512, 300 }, { {250, 10}, {250, -10}, {-500, -10}, {-500, 10} });
		addBody(true, { 0, 300 }, { {10, 500}, {10, -500}, {-10, -500}, {-10, 500} });
		addBody(true, { 1024, 300 }, { {10, 500}, {10, -500}, {-10, -500}, {-10, 500} });
		addBody(true, { 900, 250 }, { {100, 10}, {100, -10}, {-100, -10}, {-100, 10} })->setRotation(6.28/3);
		addBody(false, { 312, 90 }, { {70, 10}, {70, -10}, {-70, -10}, {-70, 10} });
		addBody(false, { 612, 140 }, { {50, 50}, {0, 80},  {-50, 50}, {0, -50} });
		addBody(false, { 412, 160 }, { {50, 50}, {-50, 50}, {0, -50}});

		std::vector<Vector2> tempCirclePoints{};
		float theta = 0.0f;
		while (theta <= 3.141592 * 2) {
			tempCirclePoints.push_back({ cos(theta) * 50, sin(theta) * 50 });
			theta += 0.45f;
		}
		addBody(false, { 100, 100 }, tempCirclePoints);

		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override {
		if (GetKey(olc::Key::W).bHeld) {
			bodies[0].first->applyForce({ 0, -100 });
		}
		if (GetKey(olc::Key::A).bHeld) {
			bodies[0].first->applyForce({ -100, 0 });
		}
		if (GetKey(olc::Key::D).bHeld) {
			bodies[0].first->applyForce({ 100, 0 });
		}
		if (GetKey(olc::Key::S).bHeld) {
			bodies[0].first->applyForce({ 0, 100 });
		}
		std::vector<Vector2> cols;

		for (int i = 0; i < 5; i++) {
			m_world->step(fElapsedTime, cols);
		}
		Clear(olc::Pixel(0, 0, 0));

		for (auto& p : bodies) {
			std::vector<Vector2> points = p.first->toVertices();

			for (size_t i1 = 0; i1 < points.size(); i1++) {
				size_t i2 = i1 + 1 < points.size() ? i1 + 1 : 0;
				DrawLine(olc::vi2d(points[i1].x, points[i1].y), olc::vi2d(points[i2].x, points[i2].y));
			}
		}

		for (auto& c : cols) {
			DrawCircle(olc::vi2d(c.x, c.y), 3, olc::RED);
		}

		return true;
	}
};

class Example2 : public olc::PixelGameEngine {
private:
	DPRigidBody* b1 = nullptr;
	DPPolygonShape* s1 = nullptr;
	DPRigidBody* b2 = nullptr;
	DPPolygonShape* s2 = nullptr;
public:
	Example2() {
		sAppName = "Example2";
	}
public:
	bool OnUserCreate() override {
		b1 = new DPRigidBody();
		b1->setPosition({ 0, 0 });
		s1 = new DPPolygonShape();
		s1->setPoints({ {-100, -100}, {100, -100}, {100, 100}, {0, 150},  {-100, 100} });
		b1->setShape(s1);

		b2 = new DPRigidBody();
		b2->setPosition({ 512, 300 });
		s2 = new DPPolygonShape();
		s2->setPoints({ {-100, -10}, {100, -10}, {100, 10}, {-100, 10} });
		b2->setShape(s2);
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override {
		b1->setPosition(Vector2(GetMouseX(), GetMouseY()));
		
		bool isCollide = false;
		std::vector<Vector2> simplex;
		detail::GJK_EPAResult result;
		isCollide = detail::GJK(b1->toVertices(), b2->toVertices(), simplex);
		if (isCollide) {
			result = detail::EPA(b1->toVertices(), b2->toVertices(), simplex);
		}

		Clear(olc::Pixel(0, 0, 0));

		std::vector<Vector2> points = b1->toVertices();
		for (size_t i1 = 0; i1 < points.size(); i1++) {
			size_t i2 = i1 + 1 < points.size() ? i1 + 1 : 0;
			DrawLine(olc::vi2d(points[i1].x, points[i1].y), olc::vi2d(points[i2].x, points[i2].y), (isCollide ? olc::RED : olc::WHITE));
		}

		points = b2->toVertices();
		for (size_t i1 = 0; i1 < points.size(); i1++) {
			size_t i2 = i1 + 1 < points.size() ? i1 + 1 : 0;
			DrawLine(olc::vi2d(points[i1].x, points[i1].y), olc::vi2d(points[i2].x, points[i2].y), (isCollide ? olc::RED : olc::WHITE));
		}

		if (isCollide) {
			DrawLine(olc::vi2d(b2->getPosition().x, b2->getPosition().y), olc::vi2d(b2->getPosition().x, b2->getPosition().y) + olc::vi2d(result.mtv.x, result.mtv.y), olc::MAGENTA);
		}

		return true;
	}
};

int main() {
	Example1 demo;
	if (demo.Construct(1024, 600, 1, 1))
		demo.Start();
	return 0;
}