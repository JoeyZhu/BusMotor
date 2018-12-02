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
#include <thread>
#include <chrono>

const int MOTOR_NUMBER = 4;
const int BUFFER_SIZE = MOTOR_NUMBER * 2 + 2;

BusMotor::BusMotor(void):slow_rotate_thread(&BusMotor::slow_rotate_thread_func, this), running_(1),
		update_period_(1), rotate_speed_(1){

}

BusMotor::~BusMotor(void){

}

void BusMotor::slow_rotate_thread_func(void){

	while(running_){
		std::vector<float> next_degree;

//		rotate(next_degree);
		std::this_thread::sleep_for(std::chrono::milliseconds(update_period_));

	}
}
int BusMotor::init(const char* serial_port){

	return rs.init(serial_port);
}

int BusMotor::slow_rotate_to(std::vector<float> &rotate_degree){

	degree_ = rotate_degree;

}


int BusMotor::set_zero(void){
	unsigned char cmd_buffer[BUFFER_SIZE];
	memset(cmd_buffer, 0, sizeof(cmd_buffer));
	cmd_buffer[0] = 0xA2;
	for(int i = 0; i < MOTOR_NUMBER; i++){

		cmd_buffer[i * 2 + 1] = 0xFF;
		cmd_buffer[i * 2 + 2] = 0xFF;
	}


	// check sum
	uint16_t check_sum = 0;
	for(int i = 0; i < BUFFER_SIZE -1; i++){
		check_sum += cmd_buffer[i];
	}
	cmd_buffer[BUFFER_SIZE - 1] = check_sum & 0x00FF;


	for(int i = 0 ; i < BUFFER_SIZE; i++){
		printf("send buffer %d: 0x%02X\n", i, cmd_buffer[i]);
	}

	rs.raw_send(cmd_buffer, BUFFER_SIZE);

	// recieve
	unsigned char rx_buffer[BUFFER_SIZE*2];
	memset(rx_buffer, 0, sizeof(rx_buffer));
	rs.read_respond(rx_buffer, sizeof(rx_buffer), 0);

}

int BusMotor::rotate(std::vector<float> &rotate_degree){
	unsigned char cmd_buffer[BUFFER_SIZE];
	memset(cmd_buffer, 0, sizeof(cmd_buffer));
	cmd_buffer[0] = 0xA1;
	uint16_t rotate_value[MOTOR_NUMBER];
	for(int i = 0; i < rotate_degree.size(); i++){
		if(rotate_degree[i] > 360.0){
			printf("WARNING, rotate degree too large: %f\n", rotate_degree[i]);
		}
		rotate_value[i] = (int)(rotate_degree[i] / 360.0 * 16384*2);

		cmd_buffer[i * 2 + 1] = ((rotate_value[i] & 0xFF00) >> 8);
		cmd_buffer[i * 2 + 2] = rotate_value[i] & 0x00FF;
		printf("rotate_value %d: %d\n", i, rotate_value[i]);
	}

	// check sum
	uint16_t check_sum = 0;
	for(int i = 0; i < BUFFER_SIZE -1; i++){
		check_sum += cmd_buffer[i];
	}
	cmd_buffer[BUFFER_SIZE - 1] = check_sum & 0x00FF;


	for(int i = 0 ; i < BUFFER_SIZE; i++){
		printf("send buffer %d: 0x%02X\n", i, cmd_buffer[i]);
	}

	printf("rotate %f degree\n", rotate_degree[0]);

	rs.raw_send(cmd_buffer, BUFFER_SIZE);

	// recieve
	unsigned char rx_buffer[BUFFER_SIZE*2];
	memset(rx_buffer, 0, sizeof(rx_buffer));
	rs.read_respond(rx_buffer, sizeof(rx_buffer), 0);

	degree_pre_.clear();
	degree_pre_ = rotate_degree;

	return 0;
}

void BusMotor::quit(void){
	rs.close_port();
	running_ = 0;
	slow_rotate_thread.join();
}
