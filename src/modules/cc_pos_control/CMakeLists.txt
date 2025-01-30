add_subdirectory(GotoControl)
add_subdirectory(PositionControl)
add_subdirectory(Takeoff)

px4_add_module(
	MODULE modules__cc_pos_control
	MAIN mc_pos_control
	COMPILE_FLAGS
	SRCS
		MulticopterPositionControl.cpp
		MulticopterPositionControl.hpp
	DEPENDS
		GotoControl
		PositionControl
		Takeoff
		controllib
		geo
		SlewRate
	)
