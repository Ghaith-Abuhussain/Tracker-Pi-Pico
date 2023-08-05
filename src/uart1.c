#include "uart1.h"

#pragma region UART_REGION

void UART_SendString(char message[], uint32_t len)
{
    for(int i = 0; i < len; i++)
    {
        uart_putc(UART_ID, message[i]);
    }
}

#pragma endregion