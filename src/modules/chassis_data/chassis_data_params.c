/**
 * @file chassis_data_params.c
 *
 * Parameters defined by the position control task for ground Boat Defs
 *
 * This is a modification of the fixed wing params and it is designed for ground Boat Defs.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing app are reported in those files.
 *
 * @author DaShen
 */

/*
 * Controller parameters, accessible via MAVLink
 */

/**
 * select the port of uart to receve chassis sensor data
 *
 * @min 0
 * @max 5
 * @value 0 GPS1
 * @value 1 TEL3
 * @value 2 EXT2
 * @value 3 TEL2
 * @value 4 TEL1
 * @value 5 GPS2
 * @group Chassisdata
 */
PARAM_DEFINE_INT32(UART_PORTNAME, 1);
/**
 * set uart baudrate
 *
 * @min 0
 * @max 230400
 * @group Chassisdata
 */
PARAM_DEFINE_INT32(UART_BAUDRATE, 115200);
/**
 * set steering wheel minimal angle
 *
 * @min -180
 * @max -60
 * @group Chassisdata
 */
PARAM_DEFINE_INT32(MIN_ANGLE, -60);
/**
 * set steering wheel maximal angle
 *
 * @min 60
 * @max 180
 * @group Chassisdata
 */
PARAM_DEFINE_INT32(MAX_ANGLE, 60);
/**
 * set maximal throttle
 *
 * @min 0
 * @max 180
 * @group Chassisdata
 */
PARAM_DEFINE_INT32(MAX_THROTTLE, 60);
