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
	int ret = motor.init("/dev/ttyS0");
	if(ret < 0){
		printf("init serial port failed\n");
		return -1;
	}
	int motor_id[4];
	float rotate_degree[4];
	motor_id[0] = 1;
	rotate_degree[0] = degree;
	rotate_degree[1] = 0;
	rotate_degree[2] = 0;
	rotate_degree[3] = 0;
	motor.rotate(motor_id, rotate_degree);
	motor.quit();
}

