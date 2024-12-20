/******************************************************************************/
/**
 * @file switch_cfg.c
 * @brief Switch Configuration Implementation
 *
 * @par Project Name
 * Embedded switch Configuration for STM32F401CC
 *
 * @par Code Language
 * C
 *
 * @par Description
 * This implementation file contains the array definition of switch configurations.
 * Each element of the array represents the configuration of a switch, including
 * GPIO port, pin number, and connection type (pull-up or pull-down).
 * 
 * @par Author
 * Mahmoud Abou-Hawis
 *
 ******************************************************************************/

/******************************************************************************/
/* INCLUDES */
/******************************************************************************/
#include "lm35.h"
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

/**
 * @brief Switch Configuration Array
 *
 * This array contains configurations for all switches defined in the system.
 * Each element represents the configuration of a specific switch, including
 * the GPIO port, pin number, and connection type (pull-up or pull-down).
 */
LM35_CFG_t lm35[_LM35_NUM] =
{
    [FIRST_LM35] =
    {
        .GPIO_Pin = GPIO_PIN0,
        .GPIO_Port = GPIO_PORTA,
        .Rank = 1,
        .chl = ADC_CHANNEL_0,
        .SamplingTime = ADC_SAMPLETIME_15CYCLES,
        .isEnabled = 0
    }
};

/******************************************************************************/

/******************************************************************************/
/* PRIVATE VARIABLE DEFINITIONS */
/******************************************************************************/

/******************************************************************************/

/******************************************************************************/
/* PUBLIC CONSTANT DEFINITIONS */
/******************************************************************************/

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