#include "ChassisData.hpp"


/*
	CRC16-MODBUS check
	param: 传入需要校验的数据数组
*/
uint16_t ChassisData::crcCheck(unsigned char* pendBuffer)
{
//	bool isequation = false;
	uint16_t crc16 = 0xffff;
	uint16_t poly = 0xa001;
	for(int i = 0;i < 6;i++)
	{
		unsigned char temp = pendBuffer[i];
		crc16 = temp ^ crc16;
		for(int j = 0 ; j < 8 ; j++)
		{
			if((0x1&crc16) == 1)
			{
				crc16 = crc16 >> 1;
				crc16 = poly ^ crc16;
			}
			else
				crc16 = crc16 >> 1;
		}
	}
	return crc16;

}
/*
	解析方向盘角度数据
*/
float ChassisData::decodeSteerWheel(unsigned char *data)
{
	float angle=0.0;
	uint16_t resultCRC = crcCheck(data);
	if((resultCRC & 0xff )==data[6] && (resultCRC>>8 & 0xff)==data[7])
		angle = (float)(data[3]<<24 | data[2]<<16 | data[1]<<8 | data[0])/100.0f/maxAngle;
	return angle;
}

/*
	解析SMC180型号油门传感器数据
*/
void ChassisData::decodeThrottle_SMC180(unsigned char *data)
{
	chassisData.timestamp = hrt_absolute_time();
	switch (data[1])
	{
		case 0x00:	//P挡
			chassisData.gear_right = 0;
			break;
		case 0x08:	//R挡
			chassisData.gear_right = 2;
			break;
		case 0x01:	//D挡
			chassisData.gear_right = 3;
			break;
		default:
			break;
	}

	switch (data[2])
	{
		case 0x00:	//P挡
			chassisData.gear_left = 0;
			break;
		case 0x08:	//R挡
			chassisData.gear_left = 2;
			break;
		case 0x01:	//D挡
			chassisData.gear_left = 3;
			break;
		default:
			break;
	}
	chassisData.throttle_right = data[3];
	chassisData.throttle_left = data[4];
	chassisData.throttle_button = data[5];
	chassisData.throttle = chassisData.throttle_left / 100.0f;
	_orb_chassisData_pub.publish(chassisData);
}
