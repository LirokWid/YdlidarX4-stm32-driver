/*
 * lidar_driver.h
 *
 *  Created on: Oct 5, 2023
 *      Author: Baptiste
 *      //http://stm32f4-discovery.net/2017/07/stm32-tutorial-efficiently-receive-uart-data-using-dma/
 *
 */

#ifndef INC_LIDAR_DRIVER_H_
#define INC_LIDAR_DRIVER_H_
#include "gpio.h"
#include "main.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "transmit.h"

#define SERIAL_DEBUG  // enable printf debug on some functions


#define DEF_PREFIX 					0xA5
#define DEF_COMMAND_START_SCAN  	0x60
#define DEF_COMMAND_STOP_SCAN 	 	0x65
#define DEF_COMMAND_GET_ID 		 	0x90
#define DEF_COMMAND_GET_STATUS 	 	0x91
#define DEF_COMMAND_SOFT_RESTART 	0x80

#define RX_BUFFER_SIZE				100

#define LIDAR_SERIAL_SPEED 128000

typedef enum lidar_systm_cmd_enum
{
	PREFIX 					= 0xA5,
	COMMAND_START_SCAN 		= 0x60,
	COMMAND_STOP_SCAN 		= 0x65,
	COMMAND_GET_ID 			= 0x90,
	COMMAND_GET_STATUS 		= 0x91,
	COMMAND_SOFT_RESTART 	= 0x80
}LidarCommand;

typedef struct lidar_command_struct {
    uint8_t prefix;
    uint8_t command;
} LidarCommandStruct;

typedef int (* lidar_transmit_drv_t)(uint8_t address, uint8_t *p_data, uint16_t size);
typedef int (* lidar_receive_drv_t)(uint8_t address, uint8_t *p_data, uint16_t size);

typedef struct lidar_serial_drv_struct
{
	lidar_transmit_drv_t transmit;
	lidar_receive_drv_t receive;
} lidar_serial_drv_t;

typedef struct h_lidar_struct
{
	UART_HandleTypeDef *lidar_uart;

	uint8_t model;
	uint8_t hardware;
	uint8_t	serial_number[16];
	struct firmware{
		uint8_t major;
		uint8_t minor;
	}firmware;

	struct health{
		uint8_t status;
		uint8_t error_code[2];
	}health;
}h_lidar_t;

typedef struct lidar_frame_response_t{

	uint8_t 	content[RX_BUFFER_SIZE];

	uint8_t		start_sign[2];
	uint8_t 	mode;
	uint8_t 	type_code;
	uint32_t	content_size;

}lidar_frame_response_t;

typedef struct device_health_t {
  uint8_t   status;
  uint16_t  error_code;
} device_health_t;

int lidar_init(h_lidar_t *lidar,UART_HandleTypeDef* lidar_uart1);

void start_motor();

void stop_motor();

void print_response(lidar_frame_response_t *response);

int get_ID(h_lidar_t* lidar);
int get_health(h_lidar_t* lidar);

int lidar_send_command(LidarCommand command);
int lidar_receive_blocking(lidar_frame_response_t *response);


#endif /* INC_LIDAR_DRIVER_H_ */
