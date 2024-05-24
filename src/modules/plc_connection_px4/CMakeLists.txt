px4_add_module(
	MODULE modules__plc_connection_px4
	MAIN plc_connection_px4
	SRCS
		PlcConnectionPx4.cpp
		PlcConnectionPx4.hpp
	DEPENDS
		px4_work_queue
		udp
		can_eth
		uart
	)
