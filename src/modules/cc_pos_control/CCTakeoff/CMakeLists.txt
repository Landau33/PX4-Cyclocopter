px4_add_library(CCTakeoff
	Takeoff.cpp
	Takeoff.hpp
)
target_include_directories(CCTakeoff PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CCTakeoff PUBLIC hysteresis)

px4_add_unit_gtest(SRC TakeoffTest.cpp LINKLIBS CCTakeoff)
