px4_add_module(
	MODULE modules__chassis_data
	MAIN chassis_data
	SRCS
		ChassisData.cpp
		ChassisData.hpp
		decodeData.cpp
	DEPENDS
		px4_work_queue
		udp
		can_eth
		uart
	)
