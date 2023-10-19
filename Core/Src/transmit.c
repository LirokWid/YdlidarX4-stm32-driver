#include "transmit.h"





void send_terminal(const char *data)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);
}

void send_terminal_nr(const char *data)
{
    int data_size = strlen(data);

    const char end_line[2] = "\n\r";
    const int end_line_size = 2;

    char new_buffer[data_size + end_line_size + 1];


    if (data_size + end_line_size <= sizeof(new_buffer))
    {
        strcpy(new_buffer, data);
        strcat(new_buffer, end_line);
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)new_buffer, data_size + end_line_size, HAL_MAX_DELAY);
    }
    else
    {
        Error_Handler();
    }
}


void send_terminal_buffer(char buffer,int size)
{
	HAL_UART_Transmit(&hlpuart1,buffer, size, HAL_MAX_DELAY);
}
