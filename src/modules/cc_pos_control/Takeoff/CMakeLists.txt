px4_add_library(Takeoff
	Takeoff.cpp
	Takeoff.hpp
)
target_include_directories(Takeoff PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Takeoff PUBLIC hysteresis)

px4_add_unit_gtest(SRC TakeoffTest.cpp LINKLIBS Takeoff)
