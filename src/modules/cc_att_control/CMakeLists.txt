add_subdirectory(CCAttitudeControl)

px4_add_module(
	MODULE modules__cc_att_control
	MAIN cc_att_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		CyclocopterAttitudeControl.cpp
		CyclocopterAttitudeControl.hpp
	DEPENDS
		CCAttitudeControl
		mathlib
		px4_work_queue
	)
