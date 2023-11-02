/**
 * @file can_eth.h
 *
 *  Converting CAN data to ETH data
 *
 * @author dashen
 */

#ifndef CAN_ETH_H_
#define CAN_ETH_H_

#include <stdio.h>
#include <string.h>

class CanEth{
private:

        uint32_t recvID;
        unsigned char dataLen;
        unsigned char recvData[8]{};
        unsigned char sendData[13]{};

        //true -> stand  ;  false -> extend
        bool frameID{false};
public:
        CanEth(){}

        ~CanEth(){}


        //CAN设备数据转换为ETH数据，message len 13 bytes
        void CanToEth(unsigned char* message);
        //ETH数据转换为CAN数据，message len 8 bytes
        void EthToCan(unsigned char* message , uint32_t sendID , u_char datalen);

        uint32_t getRecvID(){return recvID;}

        u_char* getRecvData(){return recvData;}

        u_char* getSendData(){return sendData;}

        char getDataLen(){return dataLen;}

        bool standOrExtend(){return frameID;}

};

#endif
