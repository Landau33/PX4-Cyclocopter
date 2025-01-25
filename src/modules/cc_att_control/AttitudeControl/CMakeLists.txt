px4_add_library(AttitudeControl
	AttitudeControl.cpp
	AttitudeControl.hpp
	AttitudeControlMath.hpp
)
target_compile_options(AttitudeControl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(AttitudeControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_unit_gtest(SRC AttitudeControlTest.cpp LINKLIBS AttitudeControl)
px4_add_unit_gtest(SRC AttitudeControlMathTest.cpp LINKLIBS AttitudeControl)
