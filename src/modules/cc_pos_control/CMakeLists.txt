add_subdirectory(CCGotoControl)
add_subdirectory(CCPositionControl)
add_subdirectory(CCTakeoff)

px4_add_module(
	MODULE modules__cc_pos_control
	MAIN cc_pos_control
	COMPILE_FLAGS
	SRCS
		CyclocopterPositionControl.cpp
		CyclocopterPositionControl.hpp
	DEPENDS
		CCGotoControl
		CCPositionControl
		CCTakeoff
		controllib
		geo
		SlewRate
	)
