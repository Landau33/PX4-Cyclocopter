add_subdirectory(AttitudeControl)

px4_add_module(
	MODULE modules__cc_att_control
	MAIN cc_att_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		cc_att_control_main.cpp
		cc_att_control_main.hpp
	DEPENDS
		AttitudeControl
		mathlib
		px4_work_queue
	)
