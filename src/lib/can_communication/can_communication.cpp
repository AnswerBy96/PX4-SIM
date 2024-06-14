/*
 * @Descripttion:
 * @version:
 * @Author: rsj
 * @Date: 2024-05-26 19:45:04
 * @LastEditors: rsj
 * @LastEditTime: 2024-05-27 20:34:15
 */
#include "can_communication.h"


can_communication::can_communication(/* args */)
{
}

can_communication::~can_communication()
{
}


can_frame can_communication::PackageSdo(unsigned char nodeid ,unsigned short objIndex,
                                unsigned char subIndex,int value)
{
	can_frame frame;
        frame.data[0] = 0x23;
        frame.data[1] = objIndex & 0xff;
        frame.data[2] = (objIndex >> 8) & 0xff;
        frame.data[3] = subIndex;
        frame.data[4] = value & 0xff;
        frame.data[5] = (value >> 8) & 0xff;
        frame.data[6] = (value >> 16) & 0xff;
        frame.data[7] = (value >> 24) & 0xff;

	frame.can_id = identifier_sendsdo + nodeid;

	frame.can_dlc = canframe_dlc;

	return frame;


};

can_frame can_communication::PackageSdo(unsigned char nodeid ,unsigned short objIndex,
                                unsigned char subIndex,short value)
{
	can_frame frame;
        frame.data[0] = 0x2B;
        frame.data[1] = objIndex & 0xff;
        frame.data[2] = (objIndex >> 8) & 0xff;
        frame.data[3] = subIndex;
        frame.data[4] = value & 0xff;
        frame.data[5] = (value >> 8) & 0xff;
        frame.data[6] = (value >> 16) & 0xff;
        frame.data[7] = (value >> 24) & 0xff;

	frame.can_id = identifier_sendsdo + nodeid;
	frame.can_dlc = canframe_dlc;
	return frame;
};

can_frame can_communication::PackageSdo(unsigned char nodeid ,unsigned short objIndex,
                                unsigned char subIndex,signed char value)
{
	can_frame frame;
        frame.data[0] = 0x2F;
        frame.data[1] = objIndex & 0xff;
        frame.data[2] = (objIndex >> 8) & 0xff;
        frame.data[3] = subIndex;
        frame.data[4] = value & 0xff;
        frame.data[5] = (value >> 8) & 0xff;
        frame.data[6] = (value >> 16) & 0xff;
        frame.data[7] = (value >> 24) & 0xff;

	frame.can_id = identifier_sendsdo + nodeid;
	frame.can_dlc = canframe_dlc;
	return frame;
};



