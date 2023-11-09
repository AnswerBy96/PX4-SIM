px4_add_module(
	MODULE modules__custom_commander
	MAIN custom_commander
	SRCS
		custom_commander.cpp
		custom_commander.hpp
	DEPENDS
		px4_work_queue
	)
