/******************************************************************************/
/**
 * @file switch.c
 * @brief Source file for lm35 control in the system.
 *
 * @par Project Name
 * Embedded lm35 Control System
 *
 * @par Code Language
 * C
 *
 * @par Description
 * This source file contains the implementations of functions for controlling lm35
 * in the embedded system.
 *
 * @par Author
 * Mahmoud Abou-Hawis
 *
 ******************************************************************************/


/******************************************************************************/
/* INCLUDES */
/******************************************************************************/
#include "lm35.h"
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
 * @brief Macro to Check if a Value is Not a Valid lm35 Identifier
 *
 * @param __LM35__ The value to be checked as a lm35 identifier.
 * @return 1 if the value is not a valid switch identifier, 0 otherwise.
 */
#define IS_NOT_LM35(__LM35__)             ((__LM35__ < 0) || (__LM35__ >= _LM35_NUM))



/**
 * @brief Macro to Check if a Pointer is Null
 *
 * This macro checks if the given pointer is null.
 *
 * @param __PTR__ The pointer to be checked.
 * @return 1 if the pointer is null, 0 otherwise.
 */
#define IS_NULL(__PTR__)                  ((__PTR__) == NULL)





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
 * @brief External Declaration of LM35  Configuration Array
 */
extern LM35_CFG_t lm35[_LM35_NUM];

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

LM35_errorStatus LM35_enuInit(void)
{
    ADC_Channel_t channel;
    gpioPin_t pinCfg;
    LM35_errorStatus RET_errorStatus = LM35_SUCCESS;
    for(int i = 0 ; i < _LM35_NUM ; i++)
    {
        channel.Channel         = lm35[i].chl;
        channel.Rank            = lm35[i].Rank;
        channel.SamplingTime    = lm35[i].SamplingTime; 
        pinCfg.GPIO_AT_Type     = GPIO_AT_None;
        pinCfg.GPIO_Pin         = lm35[i].GPIO_Pin;
        pinCfg.GPIO_Port        = lm35[i].GPIO_Port;
	    pinCfg.GPIO_Mode        = GPIO_MODE_AM;
        pinCfg.GPIO_Speed       = GPIO_SPEED_MEDIUM;

        if(GPIO_Init(&pinCfg) == GPIO_SUCCESS)
        {
            if(ADC_ConfigChannel(ADC_INSTANCE,&channel) == ADC_OK)
            {

            }
            else{
                RET_errorStatus = LM35_FAILED;
                break;
            }
        } 
        else
        {
            RET_errorStatus = LM35_FAILED;
            break;
        }
        
    }
    return RET_errorStatus;
}

LM35_errorStatus LM35_Start(LM35_t lm35Name)
{
    ADC_Channel_t channel;
    LM35_errorStatus RET_errorStatus = LM35_SUCCESS;
    channel.Channel         = lm35[lm35Name].chl;
    channel.Rank            = 1;
    channel.SamplingTime    = lm35[lm35Name].SamplingTime;
    if(ADC_ConfigChannel(ADC_INSTANCE,&channel) == ADC_OK && lm35[lm35Name].isEnabled == 1)
    {
        if(ADC_StartConversionAsync(ADC_INSTANCE) == ADC_OK)
        {

        }
        else
        {
            RET_errorStatus = LM35_FAILED;
        }
    }
    else{
        RET_errorStatus = LM35_FAILED;
    }
    return RET_errorStatus;
}

LM35_errorStatus LM35_enuGetValue(LM35_t lm35Name, uint32_t *Vale)
{
    LM35_errorStatus RET_errorStatus = LM35_SUCCESS;
    if(IS_NULL(Vale) || IS_NOT_LM35(lm35Name) || lm35[lm35Name].isEnabled == 0)
    {
        RET_errorStatus = LM35_FAILED;
    }
    else
    {   
        ADC_GetValue(Vale);
        *Vale = (*Vale * 150) / 1232;
    }
    return RET_errorStatus;
}

void LM35_Enable(LM35_t lm35Name)
{
    lm35[lm35Name].isEnabled = 1;
}

void LM35_Disable(LM35_t lm35Name)
{
    lm35[lm35Name].isEnabled = 0;
}

uint32_t isEnabled(LM35_t lm35Name)
{
    return lm35[lm35Name].isEnabled;
}
