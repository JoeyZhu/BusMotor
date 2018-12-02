/*
 * bus_motor.hpp
 *
 *  Created on: Nov 17, 2018
 *      Author: joey
 */
#include "raw_serial.hpp"
#include <thread>
#include <vector>

#ifndef BUS_MOTOR_HPP_
#define BUS_MOTOR_HPP_



class BusMotor{
	public:
	BusMotor(void);
	~BusMotor(void);
	int init(const char* serial_port);
	int rotate(std::vector<float> &rotate_degree);
	int set_zero(void);
	void quit(void);
	void slow_rotate_thread_func(void);
	int slow_rotate_to(std::vector<float> &rotate_degree);

	private:
	RawSerial rs;
	std::thread slow_rotate_thread;
	int running_;
	int update_period_;
	int rotate_speed_;	// degree / second
	std::vector<float> degree_;
	std::vector<float> degree_pre_;
};


#endif /* BUS_MOTOR_HPP_ */
