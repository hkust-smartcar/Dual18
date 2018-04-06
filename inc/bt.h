/*
 * Bluetooth.h
 *
 *  Created on: 2018¦~2¤ë5¤é
 *      Author: Jake
 */

#ifndef BT_H_
#define BT_H_
#include <vector>
#include <functional>
#include <cstring>

#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include "libsc/k60/jy_mcu_bt_106.h"
#include <libbase/misc_types.h>
#include "PID.h"
#include <vector>
#include "config.h"

using libsc::k60::JyMcuBt106;
using std::vector;
using std::function;
class bt {
public:
	bt(PID* a, PID* b, PID* c, PID* d, float* e, float *f):m_bt(myConfig::GetBluetoothConfig(std::function<bool(const Byte *data, const size_t size)>([this](const Byte* buff, const size_t size) -> bool{
    	this->buffer.push_back(*buff);
    	if(*buff == '\n' && (int) this->buffer.size() == 6){
    		this->setValue();
    	}
    	return true;
    }))),servoPIDCurve(a),servoPIDStraight(b),motorLPID(c),motorRPID(d),speed(e),xRatio(f){};
	std::vector<Byte> buffer;
	void sendVelocity();
	void setValue();
	virtual ~bt();

private:
    JyMcuBt106 m_bt;
    PID* servoPIDCurve = nullptr;
    PID* servoPIDStraight = nullptr;
    PID* motorLPID = nullptr;
    PID* motorRPID = nullptr;
    float* speed = nullptr;
    float* xRatio = nullptr;

};

#endif /* BLUETOOTH_H_ */
