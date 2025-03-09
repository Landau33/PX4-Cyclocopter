px4_add_library(CCGotoControl
	GotoControl.cpp
	GotoControl.hpp
)
target_include_directories(CCGotoControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
