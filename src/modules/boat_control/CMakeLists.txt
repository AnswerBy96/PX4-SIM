px4_add_module(
	MODULE modules__boat_control
	MAIN boat_control
	COMPILE_FLAGS
	SRCS
		BoatControl.cpp
		BoatControl.hpp
		pid.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
		#modules__control_allocator
		mathlib
		pid
		geo
		pure_pursuit
)
