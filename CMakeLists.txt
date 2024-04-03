cmake_minimum_required(VERSION 3.22)
project(dp)

set(CMAKE_CXX_STANDARD 20)

include_directories(include)

add_executable(${PROJECT_NAME}
	include/dp.h
	include/config.h
	include/olcPixelGameEngine.h
	include/convex_hull/convex_hull.h
	include/core/data_types.h
	include/core/types_to_string.h
	include/core/body/rigid_body.h
	include/core/detail/gjk_epa.h
	include/core/detail/sap.h
	include/core/detail/sat.h
	include/core/shape/circle_shape.h
	include/core/shape/polygon_shape.h
	include/core/shape/shape.h
	include/core/world/world.h
	include/simple_collision/simple_collision.h
	include/triangulation/triangulation.h

	src/circle_shape.cpp
	src/convex_hull.cpp
	src/gjk_epa.cpp
	src/polygon_shape.cpp
	src/rigid_body.cpp
	src/sap.cpp
	src/sat.cpp
	src/shape.cpp
	src/simple_collision.cpp
	src/test.cpp
	src/triangulation.cpp
	src/world.cpp
        )
