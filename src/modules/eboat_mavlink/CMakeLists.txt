px4_add_module(
	MODULE modules__eboat_mavlink
	MAIN eboat_mavlink
	SRCS
		eboat_mavlink.cpp
		eboat_mavlink.hpp
	DEPENDS
		px4_work_queue
	)
