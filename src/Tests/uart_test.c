#include "APP_CFG.h"
#if CFG_IS_CURRENT_APP(UART_TEST)

#include "cJson.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_uart.h"
#include "stm32f4xx_nvic.h"
#include "stm32f4xx_adc.h"
#include "lm35.h"
#include "LDR.h"
#include "Relay.h"

UART_Handle_t uart_handle;
    char arr[8] = "Mahmoud";
char e[1] ={'T'};
volatile uint8_t flag = 0;
void callback(void)
{

    UART_ReceiveAsyncZeroCopy(&uart_handle,e,1,callback);
    UART_TransmitAsyncZeroCopy(&uart_handle,e,1,NULL);
}



int main()
{
	cJSON *root = NULL;


    RCC_enuEnablePeripheral(PERIPHERAL_GPIOA);
	RCC_enuEnablePeripheral(PERIPHERAL_USART1);

	NVIC_EnableIRQ(USART1_IRQn);


    gpioPin_t TX;
	gpioPin_t RX;




    TX.GPIO_AT_Type = GPIO_AT_PushPull;
	TX.GPIO_Mode    = GPIO_MODE_AF7;
	TX.GPIO_Pin	    = GPIO_PIN9;
	TX.GPIO_Port	= GPIO_PORTA;
	TX.GPIO_Speed	= GPIO_SPEED_MEDIUM;

	RX.GPIO_AT_Type = GPIO_AT_PullDown;
	RX.GPIO_Mode    = GPIO_MODE_AF7;
	RX.GPIO_Pin	    = GPIO_PIN10;
	RX.GPIO_Port	= GPIO_PORTA;
	RX.GPIO_Speed	= GPIO_SPEED_MEDIUM;

    GPIO_Init(&TX);
	GPIO_Init(&RX);

    uart_handle.pUartInstance 				= USART1;
	uart_handle.UartConfiguration.BaudRate 	= 9600;
	uart_handle.UartConfiguration.Mode 		= UART_MODE_TX_RX;
	uart_handle.UartConfiguration.Parity 		= UART_PARITY_NONE;
	uart_handle.UartConfiguration.StopBits 	= UART_STOP_BITS_ONE;
	uart_handle.UartConfiguration.WordLength 	= UART_WORDLENGTH_8B;
	uart_handle.UartConfiguration.OverSampling = UART_OVERSAMPLING_8; 

	UART_Init(&uart_handle);

	UART_ReceiveAsyncZeroCopy(&uart_handle,e,1,callback);
    while (1)
    {

    }
    
    return 0;
}


#endif