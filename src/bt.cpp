<<<<<<< HEAD
/*
 * Bluetooth.cpp
 *
 *  Created on: 2018年2月5日
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
	Byte buff[8];
	temp = motorLPID->getcurrentVelocity();
	buff[0] = ((Byte*)&temp)[0];
	buff[1] = ((Byte*)&temp)[1];
	buff[2] = ((Byte*)&temp)[2];
	buff[3] = ((Byte*)&temp)[3];
	temp = motorRPID->getcurrentVelocity();
	buff[4] = ((Byte*)&temp)[0];
	buff[5] = ((Byte*)&temp)[1];
	buff[6] = ((Byte*)&temp)[2];
	buff[7] = ((Byte*)&temp)[3];
	m_bt.SendBuffer(buff, 8);
}

=======
/*
 * Bluetooth.cpp
 *
 *  Created on: 2018年2月5日
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
	Byte buff[8];
	temp = motorLPID->getcurrentVelocity();
	buff[0] = ((Byte*)&temp)[0];
	buff[1] = ((Byte*)&temp)[1];
	buff[2] = ((Byte*)&temp)[2];
	buff[3] = ((Byte*)&temp)[3];
	temp = motorRPID->getcurrentVelocity();
	buff[4] = ((Byte*)&temp)[0];
	buff[5] = ((Byte*)&temp)[1];
	buff[6] = ((Byte*)&temp)[2];
	buff[7] = ((Byte*)&temp)[3];
	m_bt.SendBuffer(buff, 8);
}

>>>>>>> morris_branch
