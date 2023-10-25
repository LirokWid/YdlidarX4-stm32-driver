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

	lidar->uart = lidar_uart;
	lidar->is_sending = 0;


	lidar_send_command(COMMAND_STOP_SCAN);
	HAL_Delay(200);
	//1.
	start_motor();

	get_ID(lidar);
	get_health(lidar);

	//Start point data transfer to the DMA
	start_scan(lidar);

	return SUCCESS;
}

int start_scan(h_lidar_t* lidar)
{
	HAL_UART_Receive_DMA(lidar->uart, lidar->dma_buffer, RX_BUFFER_SIZE);
	if(lidar_send_command(COMMAND_START_SCAN))
	{
		lidar->is_sending = 1;

		return SUCCESS;
	}else{
		return ERROR;
	}
}

int stop_scan(h_lidar_t* lidar)
{//TODO : this function
	HAL_UART_DMAStop(lidar->uart);

	return 0;
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
	printf("header code |%x:%x\r\n",response->start_sign[0],response->start_sign[1]);
	printf("mode 		|%x\r\n",response->mode);
	printf("typecode 	|%x\r\n",response->type_code);

	printf("Response content -> ");
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
    //TODO : passer en lecture de structure plutot que de passer par le buffer

    // Send the byte array over UART
    int result = HAL_UART_Transmit(lidar.uart, cmdBuffer, size_cmd_struct, HAL_MAX_DELAY);

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
	if(HAL_UART_Receive(lidar.uart, header_buffer, header_size, 5000)!= HAL_OK){Error_Handler();}

	//Then parse header response to get content size
	response->content_size =
			header_buffer[2] |
			header_buffer[3] << 8 |
			header_buffer[4] << 16 |
		    (header_buffer[5] & 0x3F) << 24;		//Select only 6 bits LSB

	//Receive main content from content size
	if(HAL_UART_Receive(lidar.uart, response->content, response->content_size, 5000)!= HAL_OK){Error_Handler();}

	//Finish to compute the header response and exit the function
	response->start_sign[0] = header_buffer[0];
	response->start_sign[1] = header_buffer[1];
	response->mode = header_buffer[5]&0xC0;	//Mode is contained in 2 bits MSB of byte 5
	response->type_code = header_buffer[6];

	return SUCCESS;

}

void parse_dma_buffer()
{
	uint8_t data_to_parse[RX_BUFFER_SIZE];
	for(int i=0;i<RX_BUFFER_SIZE;i++)
	{
		data_to_parse[i] = lidar.dma_buffer[i];
		//printf("%x",data_to_parse[i]);
	}


	//find the first 0xAA55001 data that shows the start of a frame
	uint8_t targetByte1 = 0xaa;  // First byte to search for
	uint8_t targetByte2 = 0x55;  // Second byte to search for

	int arraySize = RX_BUFFER_SIZE;

	const int max_point_nb = 28;
	const int header_size = 9;

	float point_array[max_point_nb];

	// Search for the first byte
	for (int i = 0; i < arraySize - 1; i++)
	{
        if (data_to_parse[i] == targetByte1 && data_to_parse[i + 1] == targetByte2)
        {// We found the start byte, now parsing the frame
        	uint8_t CT 	= data_to_parse[i+2];								//Package type (i.d. type of packet
			int 	LSN = data_to_parse[i+3];								//Number of sampling points
			int 	FSA = data_to_parse[i+4] + (data_to_parse[i+5]<<8); 	//Start angle on 2 bytes LSB first
			int 	LSA = data_to_parse[i+6] + (data_to_parse[i+7]<<8); 	//End angle on 2 bytes LSB first
			int		CS	= data_to_parse[i+8] + (data_to_parse[i+9]<<8); 	//Checksum (XOR)

			for (int j =0;j<LSN*2;j+2)//We know the number of point to sample, loop trough them to get the values
			{
				int dist = data_to_parse[i+j+header_size] +
						(data_to_parse[i+j+header_size+1]<<8);//Distance is on 2 byte /4 to get distance(data sheet)
				point_array[j] = (float) dist/4;
				printf("%f |",point_array[j]);
			}
			printf("\n\r");
//			printf("CT %02X | LSN %02X | FSA %d | LSA %d | CS %d\n\r",CT,LSN,FSA,LSA,CS);
//			i+=9+(LSN*2);//Skip the parsed frame to find the next one
	}
	printf("\n\n\r");

	}
}

