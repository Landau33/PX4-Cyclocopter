px4_add_module(
	MODULE modules__cc_rate_control
	MAIN cc_rate_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		CyclocopterRateControl.cpp
		CyclocopterRateControl.hpp
	DEPENDS
		circuit_breaker
		mathlib
		RateControl
		px4_work_queue
	)
