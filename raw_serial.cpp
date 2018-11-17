/*
 * raw_serial.cpp
 *
 *  Created on: Nov 17, 2018
 *      Author: joey
 */


#include <stdio.h>
#include "raw_serial.hpp"
#include <string.h>

// serial port relevant
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>	/*PPSIX 终端控制定义*/

#define DEBUG_SERIAL_OUTPUT 1	// output raw serial rx output

RawSerial::RawSerial(void){
	memset(serial_port_, 0, sizeof(serial_port_));
	strcpy(serial_port_, "/dev/ttyUSB0");
	sp_fd_ = 0;
}
RawSerial::~RawSerial(void){
	if(sp_fd_){
		close(sp_fd_);
	}
}

int RawSerial::init(const char* serial_port){
	if(serial_port){
		strcpy(serial_port_, serial_port);
	}
	// O_NOCTTY选项防止程序受键盘控制中止操作键等影响
	// O_NDELAY  告诉 UNIX 不必另一端端口是否启用.(检测 DCD信号线状态)
	printf("RawSerial Opening: %s\n", serial_port_);
	sp_fd_ = open(serial_port_, O_RDWR | O_NOCTTY | O_NDELAY );
	if(sp_fd_ == -1){
		//perror("unable to open /dev/ttyxxx\n");
		printf("unable to open %s\n", serial_port_);
		return -1;
	}
	fcntl(sp_fd_, F_SETFL, 0);	//change 0 to FNDELAY to immediately return after read()

	if(set_speed(sp_fd_, BAUD_RATE) != TRUE){
		perror("Unable to set baudrate\n");
		return -1;
	}
	if(set_Parity(sp_fd_, 8, 1, 'N') != TRUE){
		perror("Set Parity Error\n");
		return -1;
	}
	return 0;
}

int RawSerial::raw_send(const unsigned char* send_buf, int size){

#if DEBUG_SERIAL_OUTPUT
	printf("%s send:" , __FUNCTION__);
	for(int i = 0 ; i < size; i++){
		printf("0x%02X ", *(send_buf + i));
	}
	printf("\n");
#endif

	int bytes_written;
	bytes_written = write(sp_fd_, send_buf, size);
	if(bytes_written < 0){
		perror("write failed\n");
		return -1;
	}
	return 0;
}

int RawSerial::read_respond(unsigned char* responds, unsigned int size, unsigned int write_position){

	fd_set rfds;
	struct timeval tv;
	int ret;								//每次读的结果
	int sret;								//select监控结果
	unsigned int readlen = 0;						//实际读到的字节数
	unsigned char * ptr;
	/*传入的timeout是ms级别的单位，这里需要转换为struct timeval 结构的*/
	int timeout = 100;
	tv.tv_sec  = timeout / 1000;
	tv.tv_usec = (timeout%1000)*1000;
	if( write_position >= size){
		printf("write position error\n");
	}
	ptr = responds + write_position;							//读指针，每次移动，因为实际读出的长度和传入参数可能存在差异

	FD_ZERO(&rfds);						//清除文件描述符集合
	FD_SET(sp_fd_,&rfds);					//将fd加入fds文件描述符，以待下面用select方法监听

	/*TODO:防止读数据长度超过缓冲区*/
	//one method
//	int start = get_ticks();
#if DEBUG_SERIAL_OUTPUT
	printf("in read_respond, we got: ");
#endif
	while(readlen < size)
	{
		sret = select(sp_fd_+1,&rfds,NULL,NULL,&tv);		//检测串口是否可读

		if(sret == -1)										//检测失败
		{
			perror("select:");
			break;
		}
		else if(sret > 0)									//检测成功可读
		{
			ret = read(sp_fd_,ptr,1);
			//printf("sec: %d,usec: %d\n",tv.tv_sec,tv.tv_usec);
#if DEBUG_SERIAL_OUTPUT
			printf("0x%02X ", *ptr);
#endif
			if(ret < 0)
			{
				perror("read err:");
				break;
			}
			else if(ret == 0)
				break;

			readlen += ret;									//更新读的长度
			write_position += ret;
			write_position = write_position % size;
			ptr     = responds + write_position;			//更新读的位置
		}
		else													//超时
		{
#if DEBUG_SERIAL_OUTPUT
			printf(" got %d data. Select read timeout!\n", readlen);	//	timeout at least once
#endif
			break;
		}
	}

	//just read
	//readlen = read(sp_fd_, responds,size);
//	int time_used = get_ticks() - start;
//	PRINT_OUT_TIME_COST(time_used);
	//another method
/*	sret = select(sp_fd_+1,&rfds,NULL,NULL,&tv);		//检测串口是否可读
	if(sret){
		ret = read(sp_fd_,ptr,size);
		return ret;
	}else{
		return 0;
	}*/
#if DEBUG_SERIAL_OUTPUT
/*	size_t i;
	for (i = 0; i < readlen; i++)
	{
		printf("\\x%02X",(responds[i]));
	}*/
	printf("\n");
#endif
	return readlen;
}
/**
 *@brief   设置串口波特率
 *@param  fd     类型  int  打开的串口文件句柄
 *@param  speed 类型  int 波特率数值
 *@return 0:ok
 */
static int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,B115200, B921600, B4000000, B2000000, B1000000};
static int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200, 300, 115200, 921600, 4000000, 2000000, 1000000};
int RawSerial::set_speed(int fd, int speed){
       unsigned int   i;
       int   status;
       struct termios   Opt;
       //Get the current options for the port

       tcgetattr(fd, &Opt);

       for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {
              if  (speed == name_arr[i]) {
                     cfsetispeed(&Opt, speed_arr[i]);
                     cfsetospeed(&Opt, speed_arr[i]);
                     status = tcsetattr(fd, TCSANOW, &Opt);
                     if  (status != 0) {
                            perror("tcsetattr fd");
                            return FALSE;
                     }
                     tcflush(fd,TCIOFLUSH);
                     return TRUE;
              }
       }
       return FALSE;	//should return before
}


/**
 *@brief   设置串口数据位，停止位和效验位
 *@param  fd     类型  int  打开的串口文件句柄
 *@param  databits 类型  int 数据位   取值 为 7 或者8
 *@param  stopbits 类型  int 停止位   取值为 1 或者2
 *@param  parity  类型  int  效验类型 取值为N,E,O,,S
 *@return 1:ok
 */
int RawSerial::set_Parity(int fd, int databits, int stopbits, int parity) {
	struct termios options;
	if (tcgetattr(fd, &options) != 0) {
		perror("SetupSerial 1");
		return (FALSE);
	}
	options.c_cflag &= ~CSIZE;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input*/
	options.c_oflag &= ~OPOST; /*Output*/
	//set serialport to binary read and write mode
	options.c_oflag &= ~(ONLCR|OCRNL);
	options.c_iflag &= ~(INLCR|ICRNL);
	options.c_iflag &= ~(ICRNL | IGNCR );
	cfmakeraw(&options);
	//options.c_oflag &= ~(OCRNL | OGNCR );
	switch (databits) /*设置数据位数*/
	{
	case 7:
		options.c_cflag |= CS7;
		break;
	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		fprintf(stderr, "Unsupported data size/n");
		return (0);
	}
	switch (parity) {
	case 'n':
	case 'N':
		options.c_cflag &= ~PARENB; /* Clear parity enable */
		options.c_iflag &= ~INPCK; /* Enable parity checking */
		break;
	case 'o':
	case 'O':
		options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
		options.c_iflag |= INPCK; /* Disnable parity checking */
		break;
	case 'e':
	case 'E':
		options.c_cflag |= PARENB; /* Enable parity */
		options.c_cflag &= ~PARODD; /* 转换为偶效验*/
		options.c_iflag |= INPCK; /* Disnable parity checking */
		break;
	case 'S':
	case 's': /*as no parity*/
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		break;
	default:
		fprintf(stderr, "Unsupported parity/n");
		return (FALSE);
	}
	/* 设置停止位*/
	switch (stopbits) {
	case 1:
		options.c_cflag &= ~CSTOPB;
		break;
	case 2:
		options.c_cflag |= CSTOPB;
		break;
	default:
		fprintf(stderr, "Unsupported stop bits/n");
		return (FALSE);
	}
	/* Set input parity option */
	if (parity != 'n')
		options.c_iflag |= INPCK;
	tcflush(fd, TCIFLUSH);
	options.c_cc[VTIME] = READ_WAIT_TIME; /* waiting time n 100ms, (start from 1st byte if VMIN not zero)*/
	options.c_cc[VMIN] = MIN_NUM_RX; /* define the minimum bytes data to be readed*/
	if (tcsetattr(fd, TCSANOW, &options) != 0) {
		perror("SetupSerial 3");
		return (FALSE);
	}
	return (TRUE);
}

void RawSerial::close_port(void){
	if(sp_fd_){
		tcflush(sp_fd_,TCIOFLUSH);
		close(sp_fd_);
	}else{
		printf("close port failed, no port descripter\n");
	}
}
