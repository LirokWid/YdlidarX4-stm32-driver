/*
 * lidar_driver.c
 *
 *  Created on: Oct 5, 2023
 *      Author: Baptiste
 */
#include "lidar_driver.h"

extern h_lidar_t lidar;



int lidar_init(h_lidar_t *lidar,UART_HandleTypeDef* lidar_uart)
{
	/**
	 * 1.Turn on the motor
	 * 2.Try to connect to the lidar
	 * 3.If connected, get make, ID and status of the lidar
	 * 4.Start scanning in circular DMA mode
	 */

	lidar->lidar_uart = lidar_uart;


	//lidar_send_command(COMMAND_SOFT_RESTART);
	HAL_Delay(200);
	//1.
	start_motor();

	get_ID(lidar);
	HAL_Delay(100);
	get_health(lidar);
}


int get_ID(h_lidar_t* lidar)
{

	lidar_send_command(COMMAND_GET_ID);
	lidar_frame_response_t response;
	lidar_receive_blocking(&response);

	//print_response(&response);
	lidar->model 			= response.content[0];
	lidar->firmware.major	= response.content[1];
	lidar->firmware.minor	= response.content[2];
	lidar->hardware			= response.content[3];
	for(int i=0;i<16;i++)
	{
		lidar->serial_number[i]= response.content[i+4];
	}

	//Print id values on the terminal if debug is defined
#ifdef SERIAL_DEBUG
	printf("***Lidar IDs***\n\r");
	printf("model     | %x\n\r",lidar->model);
	printf("firmware  | %d.%d\n\r",lidar->firmware.major,lidar->firmware.minor);
	printf("hardware  | %x\n\r",lidar->hardware);
	printf("serial nb | ");
	for(int i=0;i<16;i++)
	{
		printf("%x",lidar->serial_number[i]);
	}
	printf("\n\r");
#endif

	return 0;
}

int get_health(h_lidar_t* lidar)
{

	lidar_send_command(COMMAND_GET_STATUS);
	lidar_frame_response_t response;
	lidar_receive_blocking(&response);

	//print_response(&response);

	lidar->health.status	 	= response.content[0];
	lidar->health.error_code[0] = response.content[1];
	lidar->health.error_code[1] = response.content[2];

	//Print id values on the terminal if debug is defined
#ifdef SERIAL_DEBUG
	printf("\r\n***Lidar health***\r\n");
	printf("status    | %d\r\n",lidar->health.status);
	printf("error code| %x%x\r\n",lidar->health.error_code[0],lidar->health.error_code[1]);
	printf("\n\n\n\n");
#endif

	return 0;

}

void print_response(lidar_frame_response_t *response)
{
	printf("\n\n\rlidar response :\n\n\r");
	printf("header code :\t\t%x:%x\r\n",response->start_sign[0],response->start_sign[1]);
	printf("mode :\t\t\t%x\r\n",response->mode);
	printf("typecode :\t\t%x\r\n",response->type_code);

	printf("Response content :\t");
	for(int i=0;i<response->content_size;i++)
	{
		printf("%x",response->content[i]);
	}
	printf("\r\n");


}




void start_motor()
{
	HAL_GPIO_WritePin(LIDAR_MOTOR_ENABLE_GPIO_Port, LIDAR_MOTOR_ENABLE_Pin, 1);
}

void stop_motor()
{
	HAL_GPIO_WritePin(LIDAR_MOTOR_ENABLE_GPIO_Port, LIDAR_MOTOR_ENABLE_Pin, 0);
}

int lidar_send_command(LidarCommand command)
{

    LidarCommandStruct cmd;
    cmd.prefix = PREFIX;
    cmd.command = command;

    int size_cmd_struct = sizeof(LidarCommandStruct);

    // Convert the struct to a byte array
    uint8_t cmdBuffer[sizeof(LidarCommandStruct)];
    memcpy(cmdBuffer, &cmd, sizeof(LidarCommandStruct));


    char charBuffer[30];
    int size = sprintf(charBuffer," \n\rsent data : %x ",cmd.command);
    send_terminal_buffer(charBuffer,size);


    // Send the byte array over UART
    int result = HAL_UART_Transmit(lidar.lidar_uart, cmdBuffer, size_cmd_struct, HAL_MAX_DELAY);

    if (result == HAL_OK)
    {
    	return SUCCESS;
    }else{
    	return ERROR;
    }
}
int lidar_receive_blocking(lidar_frame_response_t *response)
{
	const int header_size = 7;
	uint8_t header_buffer[header_size];

	//Receive header
	if(HAL_UART_Receive(lidar.lidar_uart, header_buffer, header_size, 5000)!= HAL_OK){Error_Handler();}

	//Then parse header response to get content size

	response->start_sign[0] = header_buffer[0];
	response->start_sign[1] = header_buffer[1];

	response->content_size =
			header_buffer[2] |
			header_buffer[3] << 8 |
			header_buffer[4] << 16 |
		    (header_buffer[5] & 0x3F) << 24;		//Select only 6 bits LSB

	response->mode = header_buffer[5]&0xC0;	//Mode is contained in 2 bits MSB of byte 5
	response->type_code = header_buffer[6];


	//Receive content
	if(HAL_UART_Receive(lidar.lidar_uart, response->content, response->content_size, 5000)!= HAL_OK){Error_Handler();}


	return SUCCESS;
}
