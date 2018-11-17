/*
 * raw_serial.hpp
 *
 *  Created on: Nov 17, 2018
 *      Author: joey
 */

#ifndef RAW_SERIAL_HPP_
#define RAW_SERIAL_HPP_

#define BAUD_RATE 		(2000000)
#define READ_WAIT_TIME	1		// in 100ms
#define MIN_NUM_RX 		1

#define TRUE 1
#define FALSE 0

class RawSerial{
public:
	RawSerial(void);
	~RawSerial(void);
	int init(const char* serial_port);
	int raw_send(const unsigned char* send_buf, int size);
	int read_respond(unsigned char* responds, unsigned int size, unsigned int write_position);
	int crc_data(unsigned char* crc_buffer, unsigned int size);
	void close_port(void);
private:
	char serial_port_[30];
	int set_Parity(int fd, int databits, int stopbits, int parity);
	int set_speed(int fd, int speed);
	int sp_fd_;
};

#endif /* RAW_SERIAL_HPP_ */
