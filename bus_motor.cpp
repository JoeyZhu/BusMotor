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
#include <math.h>

#define DEBUG_BUS_MOTOR 0

const int MOTOR_NUMBER = 4;
const int BUFFER_SIZE = MOTOR_NUMBER * 2 + 2;

int vector_equal(std::vector<float> a, std::vector<float> b){
	if(a.size() != b.size()){
		printf("WARNING: vector_euqal not equal: %ld, %ld\n", a.size(), b.size());
		return 0;
	}

	for(int i = 0; i < a.size(); i++){
		if(fabs(a[i] - b[i]) > 0.05){
			return 0;
		}
	}
	return 1;
}

BusMotor::BusMotor(void):slow_rotate_thread(&BusMotor::slow_rotate_thread_func, this), running_(1),
		update_period_(1), rotate_speed_(0.01), new_degree_(0){

}

BusMotor::~BusMotor(void){

}

void BusMotor::slow_rotate_thread_func(void){

	while(running_){
		std::vector<float> next_degree;
		if(new_degree_){
			if(degree_pre_.size() != set_degree_.size()){
				degree_pre_.clear();
				degree_pre_.resize(set_degree_.size());
			}
			next_degree.resize(set_degree_.size());
			float delta_degree = rotate_speed_ * update_period_;

			for(int i = 0; i < set_degree_.size(); i++){
				if((set_degree_[i] - degree_pre_[i]) > delta_degree){
					next_degree[i] = degree_pre_[i] + delta_degree;
				}else if((set_degree_[i] - degree_pre_[i]) < delta_degree){
					next_degree[i] = degree_pre_[i] - delta_degree;
				}else{
					next_degree[i] = set_degree_[i];
				}

			}
#if DEBUG_BUS_MOTOR
			printf("set degree: %f, next degree: %f\n", set_degree_[0], next_degree[0]);
#endif
			rotate(next_degree);

			if(vector_equal(next_degree, set_degree_)){
				new_degree_ = 0;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(update_period_));

	}
}
int BusMotor::init(const char* serial_port){

	return rs.init(serial_port);
}

int BusMotor::slow_rotate_to(std::vector<float> &rotate_degree){

	set_degree_ = rotate_degree;
	new_degree_ = 1;
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

#if DEBUG_BUS_MOTOR
	for(int i = 0 ; i < BUFFER_SIZE; i++){
		printf("send buffer %d: 0x%02X\n", i, cmd_buffer[i]);
	}
#endif
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
#if DEBUG_BUS_MOTOR
		printf("rotate_value %u: %u\n", i, rotate_value[i]);
#endif
	}

	// check sum
	uint16_t check_sum = 0;
	for(int i = 0; i < BUFFER_SIZE -1; i++){
		check_sum += cmd_buffer[i];
	}
	cmd_buffer[BUFFER_SIZE - 1] = check_sum & 0x00FF;

#if DEBUG_BUS_MOTOR
	for(int i = 0 ; i < BUFFER_SIZE; i++){
		printf("send buffer %d: 0x%02X\n", i, cmd_buffer[i]);
	}
#endif

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
