/*
 * Bluetooth.cpp
 *
 *  Created on: 2018¦~2¤ë5¤é
 *      Author: Jake
 */

#include "bt.h"



bt::~bt() {
	// TODO Auto-generated destructor stub
}

void bt::setValue(){
	Byte data[4];
	data[3] = buffer[0];
	data[2] = buffer[1];
	data[1] = buffer[2];
	data[0] = buffer[3];
	float value;
	std::memcpy(&value, &data, sizeof(float));
	Byte type = buffer[4];
	switch(type){
	case 0x00:
		servoPID->setkP(value);
		break;
	case 0x01:
		servoPID->setkD(value);
		break;
	case 0x02:
		motorLPID->setDesiredVelocity(value);
		break;
	case 0x03:
		motorLPID->setkP(value);
		break;
	case 0x04:
		motorLPID->setkI(value);
		break;
	case 0x05:
		motorLPID->setkD(value);
		break;
	case 0x06:
		motorRPID->setDesiredVelocity(value);
		break;
	case 0x07:
		motorRPID->setkP(value);
		break;
	case 0x08:
		motorRPID->setkI(value);
		break;
	case 0x09:
		motorRPID->setkD(value);
		break;
	}
	buffer.clear();
}
void bt::sendVelocity(){
	float temp;
	Byte buff[13];
	temp = servoPID->getcurrentVelocity();
	//temp = 0.4;
	buff[0] = ((Byte*)&temp)[0];
	buff[1] = ((Byte*)&temp)[1];
	buff[2] = ((Byte*)&temp)[2];
	buff[3] = ((Byte*)&temp)[3];
	temp = motorLPID->getcurrentVelocity();
	//temp = 0.5;
	buff[4] = ((Byte*)&temp)[0];
	buff[5] = ((Byte*)&temp)[1];
	buff[6] = ((Byte*)&temp)[2];
	buff[7] = ((Byte*)&temp)[3];
	temp = motorRPID->getcurrentVelocity();
	//temp = 0.6;
	buff[8] = ((Byte*)&temp)[0];
	buff[9] = ((Byte*)&temp)[1];
	buff[10] = ((Byte*)&temp)[2];
	buff[11] = ((Byte*)&temp)[3];
	buff[12] = (Byte)'\n';
	m_bt.SendBuffer(buff, 13);
}

