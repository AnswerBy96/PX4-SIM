/*
 * @Descripttion:CAN Communication
 * @version:
 * @Author: rsj
 * @Date: 2024-05-26 19:44:53
 * @LastEditors: rsj
 * @LastEditTime: 2024-05-29 20:05:45
 */

#define identifier_sendsdo 0x600
#define identifier_recvsdo 0x580
#define canframe_dlc 8

struct can_frame
{
	uint32_t can_id;
	unsigned char data[8];
	int           can_dlc;
};



class can_communication
{
private:
	/* data */
public:
	can_communication(/* args */);
	~can_communication();


	/**
	 * @description:     Sdo报文打包函数，分4字节，2字节，1字节三种情况分成三个重载函数
	 * @param nodeid     canopen设备的节点id
     	 * @param objIndex   canopen设备的对象索引
     	 * @param subIndex   canopen设备的对象子索引
	 * @param value      要写入的值
	 * @return           void
	 */
	can_frame PackageSdo(unsigned char nodeid ,unsigned short objIndex,
                                unsigned char subIndex,int value);
	can_frame PackageSdo(unsigned char nodeid ,unsigned short objIndex,
                                unsigned char subIndex,short value);
	can_frame PackageSdo(unsigned char nodeid ,unsigned short objIndex,
                                unsigned char subIndex,signed char value);


};
