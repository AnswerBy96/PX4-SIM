px4_add_module(
	MODULE modules__plc_data
	MAIN plc_data
	SRCS
		plcdata.cpp
		plcdata.hpp
	DEPENDS
		px4_work_queue
		udp
		can_eth
		uart
	)
