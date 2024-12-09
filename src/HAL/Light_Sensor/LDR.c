/******************************************************************************/
/**
 * @file switch.c
 * @brief Source file for ldr control in the system.
 *
 * @par Project Name
 * Embedded ldr Control System
 *
 * @par Code Language
 * C
 *
 * @par Description
 * This source file contains the implementations of functions for controlling ldr
 * in the embedded system.
 *
 * @par Author
 * Mahmoud Abou-Hawis
 *
 ******************************************************************************/


/******************************************************************************/
/* INCLUDES */
/******************************************************************************/
#include "LDR.h"
#include <stdbool.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
/******************************************************************************/

/******************************************************************************/
/* PRIVATE DEFINES */
/******************************************************************************/


/******************************************************************************/

/******************************************************************************/

/******************************************************************************/
/* PRIVATE MACROS */
/******************************************************************************/

/**
 * @brief Macro to Check if a Value is Not a Valid ldr Identifier
 *
 * @param __LDR__ The value to be checked as a ldr identifier.
 * @return 1 if the value is not a valid switch identifier, 0 otherwise.
 */
#define IS_NOT_LDR(__LDR__)             ((__LDR__ < 0) || (__LDR__ >= _LDR_NUM))



/**
 * @brief Macro to Check if a Pointer is Null
 *
 * This macro checks if the given pointer is null.
 *
 * @param __PTR__ The pointer to be checked.
 * @return 1 if the pointer is null, 0 otherwise.
 */
#define IS_NULL(__PTR__)                  ((__PTR__) == NULL)


#define ADC_MAX 4096         // 12-bit ADC maximum value
#define V_IN 5.0            
#define R_FIXED 10000.0      // Fixed resistor value (10 kΩ)

/******************************************************************************/
/* PRIVATE ENUMS */
/******************************************************************************/

/******************************************************************************/

/******************************************************************************/
/* PRIVATE TYPES */
/******************************************************************************/


/******************************************************************************/

/******************************************************************************/
/* PRIVATE CONSTANT DEFINITIONS */
/******************************************************************************/


/******************************************************************************/

/******************************************************************************/
/* PRIVATE VARIABLE DEFINITIONS */
/******************************************************************************/


/******************************************************************************/

/******************************************************************************/
/* PUBLIC CONSTANT DEFINITIONS */
/******************************************************************************/

/**
 * @brief External Declaration of LDR  Configuration Array
 */
extern  LDR_CFG_t ldr[_LDR_NUM];

/******************************************************************************/

/******************************************************************************/
/* PUBLIC VARIABLE DEFINITIONS */
/******************************************************************************/

/******************************************************************************/

/******************************************************************************/
/* PRIVATE FUNCTION PROTOTYPES */
/******************************************************************************/

/******************************************************************************/

/******************************************************************************/
/* PRIVATE FUNCTION DEFINITIONS */
/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/* PUBLIC FUNCTION DEFINITIONS */
/******************************************************************************/


/******************************************************************************/

LDR_errorStatus LDR_enuInit(void)
{
    ADC_Channel_t channel;
    gpioPin_t pinCfg;
    LDR_errorStatus RET_errorStatus = LDR_SUCCESS;
    for(int i = 0 ; i < _LDR_NUM ; i++)
    {
        channel.Channel         = ldr[i].chl;
        channel.Rank            = ldr[i].Rank;
        channel.SamplingTime    = ldr[i].SamplingTime; 
        pinCfg.GPIO_AT_Type     = GPIO_AT_None;
        pinCfg.GPIO_Pin         = ldr[i].GPIO_Pin;
        pinCfg.GPIO_Port        = ldr[i].GPIO_Port;
	    pinCfg.GPIO_Mode        = GPIO_MODE_AM;
        pinCfg.GPIO_Speed       = GPIO_SPEED_MEDIUM;

        if(GPIO_Init(&pinCfg) == GPIO_SUCCESS)
        {
            if(ADC_ConfigChannel(ADC_INSTANCE,&channel) == ADC_OK)
            {

            }
            else{
                RET_errorStatus = LDR_FAILED;
                break;
            }
        } 
        else
        {
            RET_errorStatus = LDR_FAILED;
            break;
        }
        
    }
    return RET_errorStatus;
}

LDR_errorStatus LDR_Start(LDR_t ldrName)
{
    ADC_Channel_t channel;
    LDR_errorStatus RET_errorStatus = LDR_SUCCESS;
    channel.Channel         = ldr[ldrName].chl;
    channel.Rank            = 1;
    channel.SamplingTime    = ldr[ldrName].SamplingTime;
    if(ADC_ConfigChannel(ADC_INSTANCE,&channel) == ADC_OK && ldr[ldrName].isEnabled == 1)
    {
        if(ADC_StartConversionAsync(ADC_INSTANCE) == ADC_OK)
        {

        }
        else
        {
            RET_errorStatus = LDR_FAILED;
        }
    }
    else{
        RET_errorStatus = LDR_FAILED;
    }
    return RET_errorStatus;
}

LDR_errorStatus LDR_enuGetValue(LDR_t ldrName, uint32_t *Vale)
{
    LDR_errorStatus RET_errorStatus = LDR_SUCCESS;
    if(IS_NULL(Vale) || ldr[ldrName].isEnabled == 0)
    {
        RET_errorStatus = LDR_FAILED;
    }
    else
    {   
        ADC_GetValue(Vale);
       *Vale = (*Vale * 5)/947;
    }
    return RET_errorStatus;
}

void LDR_Enable(LDR_t ldrName)
{
    ldr[ldrName].isEnabled = 1;
}

void LDR_Disable(LDR_t ldrName)
{
    ldr[ldrName].isEnabled = 0;
}

uint32_t isLDREnabled(LDR_t ldrName)
{
    return ldr[ldrName].isEnabled;
}
