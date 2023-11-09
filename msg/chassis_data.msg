uint64 timestamp	# time since system start (microseconds)

bool have_steeringwheel		#是否有方向盘  false -> 没有 , true -> 有
float32 steeringwheel			#方向盘数据	-1.0~1.0
float32 throttle			 	#油门推杆数据	-1.0~1.0
uint8 throttle_left			 	#左手油门推杆数据	0~100
uint8 throttle_right			 	#右手油门推杆数据	0~100
uint8 gear_left				   	 #左手档位,N->0 , D->2 , R->0x04 , Go ahead -> 0x01 , Go back -> 0x08
uint8 gear_right				    #右手档位,N->0 , D->2 , R->0x04 , Go ahead -> 0x01 , Go back -> 0x08
uint8 throttle_button				#油门设备上的按键
