px4_add_library(PositionControl
	ControlMath.cpp
	ControlMath.hpp
	PositionControl.cpp
	PositionControl.hpp
)
target_include_directories(PositionControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_unit_gtest(SRC ControlMathTest.cpp LINKLIBS PositionControl)
px4_add_unit_gtest(SRC PositionControlTest.cpp LINKLIBS PositionControl)
