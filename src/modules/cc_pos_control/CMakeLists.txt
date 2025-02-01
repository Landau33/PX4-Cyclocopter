add_subdirectory(GotoControl)
add_subdirectory(PositionControl)
add_subdirectory(Takeoff)

px4_add_module(
	MODULE modules__cc_pos_control
	MAIN cc_pos_control
	COMPILE_FLAGS
	SRCS
		CyclocopterPositionControl.cpp
		CyclocopterPositionControl.hpp
	DEPENDS
		GotoControl
		PositionControl
		Takeoff
		controllib
		geo
		SlewRate
	)
