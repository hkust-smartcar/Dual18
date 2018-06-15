/*
 * morris_pid.h
 *
 *  Created on: Apr 6, 2018
 *      Author: morristseng
 */

#ifndef INC_MORRIS_PID_H_
#define INC_MORRIS_PID_H_


class morris_pid{
public:
	morris_pid(bool is_motor):motor(is_motor){}

	float kp = 0;
	float ki = 0.001;
	float kd = 0;

	float get_kp(){return kp;}
	float get_ki(){return ki;}
	float get_kd(){return kd;}
	float get_difference(){return error;}

	void set_kp(float m_kp){ kp = m_kp;}
	void set_ki(float m_ki){ kp = m_ki;}
	void set_kd(float m_kd){ kp = m_kd;}
	void set_integral(float m_integral){ integral = m_integral;}


	int calculate_pid(int current_value, int desire_value);


private:
	bool motor;

	int servo_dt = 16;
	int servo_bias = 1000;
	int motor_dt = 10;
	float error = 0;
	float error_prior = 0;
	float integral = 0;
};





#endif /* INC_MORRIS_PID_H_ */
