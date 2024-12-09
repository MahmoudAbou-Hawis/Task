/*******************************************************************************/
/**
 * @file Relay.c
 * @brief Implementation file for Relay Module
 *
 * @par Project Name
 * Relay Control System
 *
 * @par Code Language
 * C
 *
 * @par Description
 * This implementation file contains the definitions of functions declared in
 * Relay.h. It provides the actual implementation for controlling relays, such as
 * initialization and setting relay states.
 *
 * @par Author
 * Mahmoud Abou-Hawis
 *
 ******************************************************************************/

/******************************************************************************/
/* INCLUDES */
/******************************************************************************/
#include "Relay.h"
#include <stdbool.h>
#include "stm32f4xx_gpio.h"
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
 * @brief Macro to Check if a Value is Not a Valid Relay Identifier
 *
 * This macro checks if the given value is not a valid relay identifier by verifying
 * if it is less than zero or greater than or equal to the total number of relays
 * defined in the system.
 *
 * @param __RELAY__ The value to be checked as a relay identifier.
 * @return 1 if the value is not a valid relay identifier, 0 otherwise.
 */
#define IS_NOT_RELAY(__RELAY__) ((__RELAY__ < 0) || (__RELAY__ >= _RELAYs_NUM))

/**
 * @brief Macro to Check if a Value is Not a Valid Relay State
 *
 * This macro checks if the given value is not a valid relay state by verifying
 * if it is not equal to RELAY_STATE_ON or RELAY_STATE_OFF.
 *
 * @param __STATE__ The value to be checked as a relay state.
 * @return 1 if the value is not a valid relay state, 0 otherwise.
 */
#define IS_NOT_RELAY_STATE(__STATE__) ((__STATE__ != RELAY_STATE_ON) && \
                                        (__STATE__ != RELAY_STATE_OFF))


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
 * @brief External Declaration of Relay Configuration Array
 *
 * This declaration makes the relay configuration array accessible from other
 * source files. The array contains configurations for all relays defined
 * in the system, including GPIO port, pin number, connection type, and default state.
 */
extern Relay_CFG_t Relays[_RELAYs_NUM];


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

Relay_errorStatus Relay_enuInit(void) {
    /* Temporary GPIO pin configuration structure */
    gpioPin_t pinCfg;

    Relay_errorStatus RET_enuErrorStatus = RELAY_SUCCESS;
    uint8_t Wrong_Configuration = false;

    /* Iterate through each relay in the Relays array */
    for (uint8_t relayIdx = 0; relayIdx < _RELAYs_NUM && !Wrong_Configuration; relayIdx++) {
        /* Retrieve GPIO configuration from the Relays array */
        pinCfg.GPIO_Pin = Relays[relayIdx].GPIO_Pin;
        pinCfg.GPIO_Port = Relays[relayIdx].GPIO_Port;

        /* Set desired GPIO configuration settings */
        pinCfg.GPIO_Speed = GPIO_SPEED_VERY_HIGH;
        pinCfg.GPIO_Mode = GPIO_MODE_OUT_PP;
        pinCfg.GPIO_AT_Type = GPIO_AT_None;

        /* Initialize the GPIO pin */
        if (GPIO_Init(&pinCfg) != GPIO_SUCCESS) {
            RET_enuErrorStatus = RELAY_INITIALIZATION_FAILED;
            Wrong_Configuration = true;
        } else {
            /* No thing */
        }
    }

    return RET_enuErrorStatus;
}


Relay_errorStatus Relay_enuSetStatus(uint32_t Relay_Name, uint32_t Relay_State) {
    Relay_errorStatus RET_enuErrorStatus = RELAY_SUCCESS;

    /* Validate input arguments */
    if (IS_NOT_RELAY(Relay_Name) || IS_NOT_RELAY_STATE(Relay_State) || Relays[Relay_Name].isEnabled == 0) {
        RET_enuErrorStatus = RELAY_FAILED;
    } else {
        Relays[Relay_Name].Relay_State = Relay_State;
        GPIO_SetPinValue(Relays[Relay_Name].GPIO_Port, Relays[Relay_Name].GPIO_Pin,
                                             Relay_State);
                                             
    }

    return RET_enuErrorStatus;
}

uint32_t Relay_enuGetStatus(uint32_t Relay_Name) {
    if (IS_NOT_RELAY(Relay_Name) || Relays[Relay_Name].isEnabled == 0) {
        return RELAY_FAILED;
    } else {
        return Relays[Relay_Name].Relay_State;
    }
}

void Relay_Activate(uint32_t relayName)
{
    Relays[relayName].isEnabled = 1;
}

void Relay_Deactivate(uint32_t relayName)
{
    Relays[relayName].isEnabled = 0;
}

uint32_t isRelayActivated(uint32_t relayName)
{
    return Relays[relayName].isEnabled;
}
