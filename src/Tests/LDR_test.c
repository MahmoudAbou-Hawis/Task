#include "APP_CFG.h"

#if CFG_IS_CURRENT_APP(LDR_TEST)
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




int main()
{
	cJSON *root = NULL;


    RCC_enuEnablePeripheral(PERIPHERAL_GPIOA);
	RCC_enuEnablePeripheral(PERIPHERAL_USART1);
	RCC_enuEnablePeripheral(PERIPHERAL_ADC1);

	NVIC_SetPriority(ADC_IRQn,0,0);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(ADC_IRQn);


    gpioPin_t TX;
	gpioPin_t RX;
	gpioPin_t ch1;
	ADC_Handle_t adcHande;





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

	adcHande.init.ContinuousConvMode  = DISABLE;
	adcHande.init.ScanConvMode        = DISABLE;
	adcHande.init.DataAlignment 	  = ADC_DATAALIGN_RIGHT;
	adcHande.init.NbrOfConversion	  = 1;
	adcHande.init.Resolution		  = ADC_RESOLUTION_12B;
	adcHande.init.EOCSelection        = ADC_EOC_SEQ_CONV; 
	adcHande.Instance				  = ADC1;




	ADC_Init(&adcHande);

	LDR_enuInit();
	uint64_t read = 1;

	
	char *out = NULL;
	uint8_t len = strlen(out);


	while(1)
	{
		LDR_Start(FIRST_LDR);



		LDR_enuGetValue(FIRST_LDR,&read);

		root = cJSON_CreateObject();
		cJSON_AddNumberToObject(root, "ldr",read);
	
		out = NULL;
		out = cJSON_Print(root);
		len = strlen(out);
		while(UART_TransmitAsyncZeroCopy(&uart_handle,out,len,NULL) == UART_ERROR);
		cJSON_Delete(root);

	}






    while (1)
    {

    }
    
    return 0;
}

#endif