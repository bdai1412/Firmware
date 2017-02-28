/*
 * uart.h
 *
 *  Created on: May 24, 2016
 *      Author: bdai
 */
#ifndef UART_H_
#define UART_H_
int uart_init(const char *uart_name);
int set_uart_baudrate(const int fd, unsigned int baud);
#endif /* UART_H_ */
