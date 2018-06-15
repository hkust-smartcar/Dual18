<<<<<<< HEAD
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
	bt(PID* a, PID* b, PID* c):m_bt(myConfig::GetBluetoothConfig(std::function<bool(const Byte *data, const size_t size)>([this](const Byte* buff, const size_t size) -> bool{
    	this->buffer.push_back(*buff);
    	if(*buff == '\n' && (int) this->buffer.size() == 6){
    		this->setValue();
    	}
    	return true;
    }))),servoPID(a),motorLPID(b),motorRPID(c){};
	std::vector<Byte> buffer;
	void sendVelocity();
	void setValue();
	virtual ~bt();

private:
    JyMcuBt106 m_bt;
    PID* servoPID = nullptr;
    PID* motorLPID = nullptr;
    PID* motorRPID = nullptr;

};

#endif /* BLUETOOTH_H_ */
=======
/*
 * Bluetooth.h
 *
 *  Created on: 2018ï¿½~2ï¿½ï¿½5ï¿½ï¿½
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
	bt(PID* a, PID* b, PID* c):m_bt(myConfig::GetBluetoothConfig(std::function<bool(const Byte *data, const size_t size)>([this](const Byte* buff, const size_t size) -> bool{
    	this->buffer.push_back(*buff);
    	if(*buff == '\n' && (int) this->buffer.size() == 6){
    		this->setValue();
    	}
    	return true;
    }))),servoPID(a),motorLPID(b),motorRPID(c){};
	std::vector<Byte> buffer;
	void sendVelocity();
	void setValue();
	virtual ~bt();

private:
    JyMcuBt106 m_bt;
    PID* servoPID = nullptr;
    PID* motorLPID = nullptr;
    PID* motorRPID = nullptr;

};

#endif /* BLUETOOTH_H_ */
>>>>>>> morris_branch
