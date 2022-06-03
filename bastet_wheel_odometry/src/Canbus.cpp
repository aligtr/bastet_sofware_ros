/*
 * Canbus.cpp
 *
 *  Created on: 14 мар. 2022 г.
 *      Author: work
 */

#include "Canbus.h"

void Canbus::sendMsg(char *data,char dataLen,int ID) // id=1
{
    _VCI_CAN_MSG message;
    message.DLC=dataLen;
    message.RTR=0;
    message.ID=ID;
    message.Mode=0;
    memcpy(message.Data,data,dataLen);
    VCI_SendCANMsg(devPort,CAN1,&message);
}

unsigned int Canbus::read(PVCI_CAN_MSG data) // 1010 1020 1030 1040 (if id=3 -> error: {byte 1 - err id, byte 2 - wheel id})
{
    DWORD messageCount=0;
    VCI_Get_RxMsgCnt(devPort,1,&messageCount);
    if(messageCount>0)
    {

        VCI_RecvCANMsg(devPort,1,data);
        return 1;
    }
    else
    {
        return 0;
    }
}