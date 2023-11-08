px4_add_module(
	MODULE modules__eboat_mavlink
	MAIN eboat_mavlink
	COMPILE_FLAGS
		#-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
		#-O0             # uncomment when debugging
	SRCS
		eboat_mavlink.cpp
	DEPENDS
		px4_work_queue
	)
