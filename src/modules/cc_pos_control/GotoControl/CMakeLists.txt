px4_add_library(GotoControl
	GotoControl.cpp
	GotoControl.hpp
)
target_include_directories(GotoControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
