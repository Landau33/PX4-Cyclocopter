px4_add_library(CCPositionControl
	ControlMath.cpp
	ControlMath.hpp
	PositionControl.cpp
	PositionControl.hpp
)
target_include_directories(CCPositionControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_unit_gtest(SRC ControlMathTest.cpp LINKLIBS CCPositionControl)
px4_add_unit_gtest(SRC PositionControlTest.cpp LINKLIBS CCPositionControl)
