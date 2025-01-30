px4_add_module(
	MODULE modules__mc_rate_control
	MAIN mc_rate_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		MulticopterRateControl.cpp
		MulticopterRateControl.hpp
	DEPENDS
		circuit_breaker
		mathlib
		RateControl
		px4_work_queue
	)
