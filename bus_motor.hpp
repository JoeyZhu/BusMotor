/*
 * bus_motor.hpp
 *
 *  Created on: Nov 17, 2018
 *      Author: joey
 */
#include "raw_serial.hpp"

#ifndef BUS_MOTOR_HPP_
#define BUS_MOTOR_HPP_



class BusMotor{
	public:
	BusMotor(void);
	~BusMotor(void);
	int init(const char* serial_port);
	int rotate(int motor_id[], float rotate_degree[]);
	void quit(void);
	
	private:
	RawSerial rs;
};


#endif /* BUS_MOTOR_HPP_ */
