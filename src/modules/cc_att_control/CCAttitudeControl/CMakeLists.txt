px4_add_library(CCAttitudeControl
	AttitudeControl.cpp
	AttitudeControl.hpp
	AttitudeControlMath.hpp
)
target_compile_options(CCAttitudeControl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(CCAttitudeControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_unit_gtest(SRC AttitudeControlTest.cpp LINKLIBS CCAttitudeControl)
px4_add_unit_gtest(SRC AttitudeControlMathTest.cpp LINKLIBS CCAttitudeControl)
