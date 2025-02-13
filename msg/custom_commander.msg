uint64 timestamp		# time since system start (microseconds)
uint8 DRIVE_MODE_IDLE = 0
uint8 DRIVE_MODE_AUTO = 1
uint8 DRIVE_MODE_MANUAL = 2
uint8 DRIVE_MODE_REMOTE = 3
uint8 DRIVE_MODE_ANCHOR = 4
uint8 DRIVE_MODE_CRUISE = 5
uint8  current_gear					#vehicle gear
float32 target_throttle					# [-1 , 1]
float32 target_steeringwheel			# [-1 , 1]
uint8 drive_mode				#0 -> manual , 1 -> auto , 2 -> remote
uint8 sub_mode					#0 -> cruise , 1 -> anchpring , 2 -> port
bool  system_start			#turn on or turn off system 	0->turn off 	1->turn on
uint8  system_error		#bit0 -> gear error
