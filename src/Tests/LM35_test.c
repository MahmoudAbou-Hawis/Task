#include "APP_CFG.h"

#if CFG_IS_CURRENT_APP(LM35_TEST)
#include "cJson.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_uart.h"
#include "stm32f4xx_nvic.h"
#include "stm32f4xx_adc.h"
#include "lm35.h"


UART_Handle_t uart_handle;
    char arr[8] = "Mahmoud";
char e[1] ={'T'};
volatile uint8_t flag = 0;
void callback(void)
{
	UART_TransmitAsyncZeroCopy(&uart_handle,e,1,NULL);
	UART_ReceiveAsyncZeroCopy(&uart_handle,arr,1,callback);
}



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



	ch1.GPIO_AT_Type = GPIO_AT_None;
	ch1.GPIO_Mode    = GPIO_MODE_AM;
	ch1.GPIO_Pin     = GPIO_PIN1;
	ch1.GPIO_Port    = GPIO_PORTA;
	ch1.GPIO_Speed	= GPIO_SPEED_MEDIUM;

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
	GPIO_Init(&ch1) ;

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
	ADC_Channel_t channelconf;
	channelconf.Channel = ADC_CHANNEL_1;
	channelconf.Rank    = 1;
	channelconf.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	ADC_ConfigChannel(ADC1,&channelconf);
	ADC_StartConversionAsync(ADC1);
	uint32_t read = 1;

	while (read == 1)
	{
		ADC_GetValue(&read);
	}
	
	

	
	root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "value",read);
	char *out = NULL;
	out = cJSON_Print(root);
	uint8_t len = strlen(out);
	UART_TransmitAsyncZeroCopy(&uart_handle,out,len,NULL);


	LM35_enuInit();

	for(int i =0 ; i < 1000000;i++);
	while(1)
	{
		LM35_Start(FIRST_LM35);

		LM35_enuGetValue(FIRST_LM35,&read);
		
		root = cJSON_CreateObject();
		cJSON_AddNumberToObject(root, "value",read);
	
		out = NULL;
		out = cJSON_Print(root);
		len = strlen(out);
		while(UART_TransmitAsyncZeroCopy(&uart_handle,out,len,NULL) == UART_ERROR);
		for(int i = 0 ; i < 10000000;i++);
		cJSON_Delete(root);

	}






    while (1)
    {

    }
    
    return 0;
}


#endif