#ifndef TRANSMIT_H
#define TRANSMIT_H

#include "main.h"
#include "usart.h"
#include "lidar_driver.h"



void send_terminal(const char *data);
void send_terminal_nr(const char *data);

void LPUART1_Transmit(const char *data);//Transmit to terminal
void send_terminal_buffer(char buffer,int size);




#endif
