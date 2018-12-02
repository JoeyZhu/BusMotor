/*
 * main.cpp
 *
 *  Created on: Nov 17, 2018
 *      Author: joey
 */



#include <stdio.h>
#include "raw_serial.hpp"
#include "bus_motor.hpp"
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv){
/*
	printf("hello\n");
	RawSerial rs;
	rs.init("/dev/ttyUSB1");
	char test_char[10];
	for(int i = 0 ; i < 10; i++){
		test_char[i] = i;
	}
	rs.raw_send(test_char, 10);
	printf("send finish\n");
	unsigned char test_rx_char[10];
	memset(test_rx_char, 0, sizeof(test_rx_char));
	rs.read_respond(test_rx_char, sizeof(test_rx_char), 0);
	printf("rx: %s\n", test_rx_char);
	rs.close_port();

*/

	// test BusMotor

	if(argc != 2){
		printf("usage: ./exe degree\n");
		return -1;
	}
	int degree = atoi(argv[1]);

	BusMotor motor;
	int ret = motor.init("/dev/ttyAMA0");
	if(ret < 0){
		printf("init serial port failed\n");
		return -1;
	}

	// reset zero degree
	if(degree == 361){
		motor.set_zero();
		motor.quit();
		exit(0);
	}

	std::vector<float> rotate_degree;
	rotate_degree.push_back(180.0);
	rotate_degree.push_back(180.0);
	rotate_degree.push_back(0.0);
	rotate_degree.push_back(0.0);
	motor.rotate(rotate_degree);

	rotate_degree.clear();
	rotate_degree.push_back(degree);
	rotate_degree.push_back(degree);
	rotate_degree.push_back(0);
	rotate_degree.push_back(0);
//	motor.rotate(rotate_degree);

	motor.slow_rotate_to(rotate_degree);

	std::this_thread::sleep_for(std::chrono::seconds(10));

	motor.quit();
}

