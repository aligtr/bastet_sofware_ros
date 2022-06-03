/*
 * Canbus.h
 *
 *  Created on: 14 мар. 2022 г.
 *      Author: work
 */

#ifndef CANBUS_H_
#define CANBUS_H_

extern "C"
{
    #include "i7565H1H2.h"
}

class Canbus {

private:
	BYTE devPort;
	BYTE devType;
	DWORD CAN1_Baud;
	DWORD CAN2_Baud;
	int errorState;

public:


	Canbus(BYTE port,BYTE dev,DWORD baud1,DWORD baud2) : devPort(port), devType(dev),CAN1_Baud(baud1),CAN2_Baud(baud2)
	{
		errorState=VCI_OpenCAN_NoStruct(devPort,devType,CAN1_Baud,CAN2_Baud);
	}

	int getErrorState(void) {return errorState;}
	void sendMsg(char *data,char dataLen,int ID);
	unsigned int read(PVCI_CAN_MSG data);

	~Canbus()
	{
		_VCI_CAN_PARAM can;
		can.DevPort=devPort;
		VCI_CloseCAN(&can);
	}

	void canFlush(void)
	{
		VCI_Clr_RxMsgBuf(devPort,CAN1);
	}
};

#endif /* CANBUS_H_ */


