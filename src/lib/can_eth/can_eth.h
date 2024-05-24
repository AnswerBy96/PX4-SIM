/*
 * @Descripttion:
 * @version:
 * @Author: chenjw
 * @Date: 2023-11-08 03:28:02
 * @LastEditors: rsj
 * @LastEditTime: 2024-05-09 19:21:26
 */
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


        //将转换器（CAN->Eth）转换后的Eth数据解析为Can数据，message len 13 bytes
        void EthToCan(unsigned char* message);
        //将Can数据转换成Eth数据后通过转换器(Eth->CAN)发出，message len 8 bytes
        void CanToEth(unsigned char* message , uint32_t sendID , u_char datalen);

        uint32_t getRecvID(){return recvID;}

        u_char* getRecvData(){return recvData;}

        u_char* getSendData(){return sendData;}

        char getDataLen(){return dataLen;}

        bool standOrExtend(){return frameID;}

};

#endif
