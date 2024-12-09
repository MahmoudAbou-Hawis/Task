#include "APP/App.h"
#include "cJson.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_uart.h"
#include "stm32f4xx_nvic.h"
#include "stm32f4xx_adc.h"
#include "lm35.h"
#include "LDR.h"
#include "Relay.h"
#include "freeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "string.h"
#include <stdio.h>


UART_Handle_t uart_handle;
ADC_Handle_t adcHande;
TaskHandle_t xParseTaskHandle;
uint32_t LM35Time = 0;
uint32_t LDRTime = 0;

static char rxBuffer[50];
static char tempBuffer[50];

static int idx = 0;
static uint32_t totoalTime = 0;

static void handleCommand(const char *command, int nodeID, const cJSON *data);

void UartCallBack()
{
	if (rxBuffer[idx] == '}')
	{
		rxBuffer[idx + 1] = '\0';
		strcpy(tempBuffer, rxBuffer);
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(xParseTaskHandle, &xHigherPriorityTaskWoken);
		idx = 0;
	}
	else
	{
		idx++;
	}
	UART_ReceiveAsyncZeroCopy(&uart_handle, rxBuffer + idx, 1, UartCallBack);
}

void vParseTask(void *pvParameters)
{
	char parseBuffer[50];

	while (1)
	{

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		strcpy(parseBuffer, tempBuffer);

		cJSON *json = cJSON_Parse(parseBuffer);
		if (json == NULL)
		{

			const char *errorPtr = cJSON_GetErrorPtr();
			if (errorPtr != NULL)
			{
			}
		}
		else
		{

			cJSON *command = cJSON_GetObjectItem(json, "command");
			cJSON *nodeID = cJSON_GetObjectItem(json, "nodeID");
			cJSON *data = cJSON_GetObjectItem(json, "data");

			if (cJSON_IsString(command) && cJSON_IsNumber(nodeID))
			{

				handleCommand(command->valuestring, nodeID->valueint, data);
			}
			else
			{
			}
			cJSON_Delete(json);
		}

		memset(parseBuffer, 0, sizeof(parseBuffer));
	}
}
void handleCommand(const char *command, int nodeID, const cJSON *data)
{
	char jsonString[128]; 
	if (strcmp(command, "ENA") == 0)
	{
		if (nodeID == 0x80)
		{
			LM35_Enable(FIRST_LM35);
			snprintf(jsonString, sizeof(jsonString), "{\"nodeType\":\"NS\", \"nodeID\": 0x80, \"data\":\"DONE\"}");
			UART_TransmitAsyncZeroCopy(&uart_handle, (char *)jsonString, strlen(jsonString), NULL);
		}
		else if (nodeID == 0x81)
		{
			LDR_Enable(FIRST_LDR);
			snprintf(jsonString, sizeof(jsonString), "{\"nodeType\":\"NS\", \"nodeID\": 0x81, \"data\":\"DONE\"}");
			UART_TransmitAsyncZeroCopy(&uart_handle, (char *)jsonString, strlen(jsonString), NULL);
		}
		else if (nodeID == 0x50)
		{
			Relay_Activate(FIRST_RELAY);
			snprintf(jsonString, sizeof(jsonString), "{\"nodeType\":\"NA\", \"nodeID\": 0x50, \"data\":\"DONE\"}");
			UART_TransmitAsyncZeroCopy(&uart_handle, (char *)jsonString, strlen(jsonString), NULL);
		}
	}
	else if (strcmp(command, "DIS") == 0)
	{
		if (nodeID == 0x80)
		{
			LM35_Disable(FIRST_LM35);
		}
		else if (nodeID == 0x81)
		{
			LDR_Disable(FIRST_LDR);
		}
		else 
		{
			Relay_Deactivate(FIRST_RELAY);
		}
	}
	else if (strcmp(command, "DUR") == 0)
	{
		if (data && cJSON_IsNumber(data))
		{
			int duration = cJSON_GetNumberValue(data);
			if (nodeID == 0x80)
			{
				LM35Time = duration;
			}
			else if (nodeID == 0x81)
			{
				LDRTime = duration;
			}
		}
	}
	else if (strcmp(command, "ACT") == 0)
	{
		if (data && cJSON_IsString(data))
		{
			const char *action = cJSON_GetStringValue(data);
			if (strcmp(action, "0") == 0)
			{
				Relay_enuSetStatus(FIRST_RELAY, RELAY_STATE_OFF);
			}
			else if (strcmp(action, "1") == 0)
			{
				Relay_enuSetStatus(FIRST_RELAY, RELAY_STATE_ON);
			}
		}
	}
	else if (strcmp(command, "STA") == 0)
	{
		if (data && cJSON_IsNull(data))
		{
			const char *action = cJSON_GetStringValue(data);
			uint32_t relayStatus = Relay_enuGetStatus(FIRST_RELAY);
			snprintf(jsonString, sizeof(jsonString), "{\"nodeType\":\"NA\", \"nodeID\": 0x%x, \"data\":\"%s\", \"relayStatus\":\"%ld\"}", nodeID, action, relayStatus);
			UART_TransmitAsyncZeroCopy(&uart_handle, (char *)jsonString, strlen(jsonString), NULL);
		}
	}
	else
	{
	}
}

void vPeriodicTaskFunction(void *pvParameters)
{
	for (;;)
	{

		if (LDRTime != 0 && totoalTime % LDRTime == 0)
		{
			uint32_t value;
			LDR_Start(FIRST_LM35);
			vTaskDelay(pdMS_TO_TICKS(100));
			LDR_enuGetValue(FIRST_LM35, &value);
			char jsonString[100];
			snprintf(jsonString, sizeof(jsonString), "{\"nodeType\":\"NS\", \"nodeID\": 0x81, \"data\": %lu}", value);
			UART_TransmitAsyncZeroCopy(&uart_handle, (char *)jsonString, strlen(jsonString), NULL);
		}
		if (LM35Time != 0 && totoalTime % LM35Time)
		{
			uint32_t value;
			LM35_Start(FIRST_LM35);
			vTaskDelay(pdMS_TO_TICKS(100));
			LM35_enuGetValue(FIRST_LM35, &value);
			char jsonString[100];
			snprintf(jsonString, sizeof(jsonString), "{\"nodeType\":\"NS\", \"nodeID\": 0x80, \"data\": %lu}", value);
			UART_TransmitAsyncZeroCopy(&uart_handle, (char *)jsonString, strlen(jsonString), NULL);
		}
		vTaskDelay(pdMS_TO_TICKS(900));
	}
}

static void uart_config()
{
	gpioPin_t TX;
	gpioPin_t RX;

	TX.GPIO_AT_Type = GPIO_AT_PushPull;
	TX.GPIO_Mode = GPIO_MODE_AF7;
	TX.GPIO_Pin = GPIO_PIN9;
	TX.GPIO_Port = GPIO_PORTA;
	TX.GPIO_Speed = GPIO_SPEED_MEDIUM;

	RX.GPIO_AT_Type = GPIO_AT_PullDown;
	RX.GPIO_Mode = GPIO_MODE_AF7;
	RX.GPIO_Pin = GPIO_PIN10;
	RX.GPIO_Port = GPIO_PORTA;
	RX.GPIO_Speed = GPIO_SPEED_MEDIUM;

	GPIO_Init(&TX);
	GPIO_Init(&RX);

	uart_handle.pUartInstance = USART1;
	uart_handle.UartConfiguration.BaudRate = 9600;
	uart_handle.UartConfiguration.Mode = UART_MODE_TX_RX;
	uart_handle.UartConfiguration.Parity = UART_PARITY_NONE;
	uart_handle.UartConfiguration.StopBits = UART_STOP_BITS_ONE;
	uart_handle.UartConfiguration.WordLength = UART_WORDLENGTH_8B;
	uart_handle.UartConfiguration.OverSampling = UART_OVERSAMPLING_8;
	UART_Init(&uart_handle);
}

static void adc_config()
{
	adcHande.init.ContinuousConvMode = DISABLE;
	adcHande.init.ScanConvMode = DISABLE;
	adcHande.init.DataAlignment = ADC_DATAALIGN_RIGHT;
	adcHande.init.NbrOfConversion = 1;
	adcHande.init.Resolution = ADC_RESOLUTION_12B;
	adcHande.init.EOCSelection = ADC_EOC_SEQ_CONV;
	adcHande.Instance = ADC1;
	ADC_Init(&adcHande);
}


void System_Config()
{
	RCC_enuEnablePeripheral(PERIPHERAL_GPIOA);
	RCC_enuEnablePeripheral(PERIPHERAL_USART1);
	RCC_enuEnablePeripheral(PERIPHERAL_ADC1);

	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(ADC_IRQn);

	uart_config();
	adc_config();
	LM35_enuInit();
	LDR_enuInit();
	Relay_enuInit();
    UART_ReceiveAsyncZeroCopy(&uart_handle, rxBuffer, 1, UartCallBack);
    xTaskCreate(vParseTask, "ParseTask", 512, NULL, 1, &xParseTaskHandle);
	xTaskCreate(vPeriodicTaskFunction, "Periodic Function Task", 128, NULL,  2, NULL);
}

void System_Start()
{
	vTaskStartScheduler();
}

