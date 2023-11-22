uint64 timestamp		# time since system start (microseconds)
uint8  current_gear					#vehicle gear
float32 target_throttle					# [-1 , 1]
float32 target_steeringwheel			# [-1 , 1]
uint8 drive_mode				#0 -> manual , 1 -> auto , 2 -> remote
bool  system_start			#turn on or turn off system 	0->turn off 	1->turn on
uint8  system_error		#bit0 -> gear error
