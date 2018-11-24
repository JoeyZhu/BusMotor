/*
 * bus_motor.cpp
 *
 *  Created on: Nov 17, 2018
 *      Author: joey
 */
#include <stdio.h>
#include "bus_motor.hpp"
#include <string.h>
#include <stdint.h>

const int MOTOR_NUMBER = 4;
const int BUFFER_SIZE = MOTOR_NUMBER * 2 + 2;

BusMotor::BusMotor(void){

}

BusMotor::~BusMotor(void){

}

int BusMotor::init(const char* serial_port){
	return rs.init(serial_port);
}

int BusMotor::rotate(int motor_id[], float rotate_degree[]){
	unsigned char cmd_buffer[BUFFER_SIZE];
	memset(cmd_buffer, 0, sizeof(cmd_buffer));
	cmd_buffer[0] = 0xA1;
	uint16_t rotate_value[MOTOR_NUMBER];
	for(int i = 0; i < MOTOR_NUMBER; i++){
		if(rotate_degree[i] > 360.0){
			printf("WARNING, rotate degree too large: %f\n", rotate_degree[i]);
		}
		rotate_value[i] = (int)(rotate_degree[i] / 360.0 * 16384*2);
		printf("rotate_value %d: %d\n", i, rotate_value[i]);
	}
	cmd_buffer[1] = ((rotate_value[0] & 0xFF00) >> 8);
	cmd_buffer[2] = rotate_value[0] & 0x00FF;

	// check sum
	uint16_t check_sum = 0;
	for(int i = 0; i < BUFFER_SIZE -1; i++){
		check_sum += cmd_buffer[i];
	}
	cmd_buffer[BUFFER_SIZE - 1] = check_sum & 0x00FF;


	for(int i = 0 ; i < BUFFER_SIZE; i++){
		printf("send buffer %d: 0x%02X\n", i, cmd_buffer[i]);
	}

	printf("rotate %d, %f degree\n", motor_id[0], rotate_degree[0]);

	rs.raw_send(cmd_buffer, BUFFER_SIZE);

	// recieve
	unsigned char rx_buffer[BUFFER_SIZE*2];
	memset(rx_buffer, 0, sizeof(rx_buffer));
	rs.read_respond(rx_buffer, sizeof(rx_buffer), 0);

	return 0;
}

void BusMotor::quit(void){
	rs.close_port();
}
