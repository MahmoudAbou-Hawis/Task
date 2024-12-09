/*******************************************************************************/
/**
 * @file Relay.h
 * @brief Header file for Relay Module
 *
 * @par Project Name
 * Embedded RELAY Control for STM32F401CC
 *
 * @par Code Language
 * C
 *
 * @par Description
 * This header file defines the interface for the Relay module. It contains declarations
 * for functions and data types related to controlling relays, such as initialization
 * and setting relay states.
 *
 * @par Configuration
 * Before including this header file in your source code, ensure that you have
 * properly configured all relays in the Relay_cfg.h header file. Each relay
 * should be assigned a unique identifier and its corresponding GPIO pin.
 *
 * @par Usage
 * To use the Relay module in your project:
 * 1. Include the "Relay.h" header file in your source files where relay 
 * functionality is needed.
 * 2. Call Relay_enuInit() to initialize the Relay module before using 
 * any other relay-related functions.
 * 3. Use the Relay_enuSetStatus() function to set the status (ON/OFF) 
 * of individual relays.
 *
 * @par Author
 * Mahmoud Abou-Hawis
 *
 ******************************************************************************/


/******************************************************************************/
/* MULTIPLE INCLUSION GUARD */
/******************************************************************************/
#ifndef RELAY_H_
#define RELAY_H_
/******************************************************************************/

/******************************************************************************/
/* C++ Style GUARD */
/******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
/******************************************************************************/

/******************************************************************************/
/* INCLUDES */
/******************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include "Relay_Conf.h"
/******************************************************************************/

/******************************************************************************/
/* PUBLIC DEFINES */
/******************************************************************************/

#define RELAY_STATE_ON             ((uint32_t)0)
#define RELAY_STATE_OFF            ((uint32_t)1)
/******************************************************************************/

/******************************************************************************/
/* PUBLIC MACROS */
/******************************************************************************/

/******************************************************************************/

/******************************************************************************/
/* PUBLIC ENUMS */
/******************************************************************************/

/**
 * @brief Relay Error Status Enumeration
 *
 * This enumeration defines the possible error status values that can be returned
 * by relay-related functions. It includes the following values:
 * - RELAY_SUCCESS: The operation was successful.
 * - RELAY_INITIALIZATION_FAILED: Relay initialization failed.
 * - RELAY_FAILED: General failure status.
 */
typedef enum {
    RELAY_SUCCESS = 0U,              /**< Operation successful */
    RELAY_INITIALIZATION_FAILED,    /**< Relay initialization failed */
    RELAY_FAILED                     /**< General failure status */
} Relay_errorStatus;

/******************************************************************************/

/******************************************************************************/
/* PUBLIC TYPES */
/******************************************************************************/

/**
 * @brief Relay Configuration Structure
 *
 * This structure defines the configuration parameters for an individual relay.
 * It includes the following members:
 *
 * - **GPIO_Port:** A pointer to the GPIO port to which the relay is connected.
 * - **GPIO_Pin:** The pin number of the GPIO port used for the relay.
 * - **Relay_State:** The initial state of the relay (0 for OFF, 1 for ON).
 */
typedef struct
{
    void * GPIO_Port;    /**< Pointer to the GPIO port used by the relay */
    uint32_t GPIO_Pin;    /**< Pin number of the relay */
    uint32_t Relay_State;    /**< Default state of the relay (ON or OFF) */
    uint32_t isEnabled;      /**is the relay enabled in default state */   
} Relay_CFG_t;

/******************************************************************************/

/******************************************************************************/
/* PUBLIC CONSTANT DECLARATIONS */
/******************************************************************************/

/******************************************************************************/

/******************************************************************************/
/* PUBLIC VARIABLE DECLARATIONS */
/******************************************************************************/


/******************************************************************************/

/******************************************************************************/
/* PUBLIC FUNCTION PROTOTYPES */
/******************************************************************************/

/**
 * @brief Initialize the Relay Module
 *
 * This function initializes the Relay module. It configures the GPIO pins and
 * sets the default state for all relays. If initialization is successful, it
 * returns RELAY_SUCCESS; otherwise, it returns RELAY_INITIALIZATION_FAILED.
 *
 * @return Relay_errorStatus
 * - RELAY_SUCCESS: The Relay module was successfully initialized.
 * - RELAY_INITIALIZATION_FAILED: Relay module initialization failed.
 */
extern Relay_errorStatus Relay_enuInit(void);

/**
 * @brief Set the Status of a Relay
 *
 * This function sets the status of the specified relay to the given state.
 *
 * @param Relay_Name The name or identifier of the relay.
 * @param Relay_State The state to set for the relay (ON or OFF).
 * 
 * @return Relay_errorStatus
 * - RELAY_SUCCESS: The relay status was successfully set.
 * - RELAY_FAILED: Failed to set the relay status.
 * 
 * @note You should call Relay_enuInit() first.
 */
extern Relay_errorStatus Relay_enuSetStatus(uint32_t Relay_Name, uint32_t Relay_State);


/**
 * @brief Reads the current status of a relay.
 *
 * This function reads the current state of the specified relay and returns it.
 *
 * @param Relay_Name The name or identifier of the relay.
 * 
 * @return Relay_State The current state of the relay (0 for OFF, 1 for ON).
 * 
 * @note You should call Relay_enuInit() first.
 */
extern uint32_t Relay_enuGetStatus(uint32_t Relay_Name);



/**
 * Activates a specific relay.
 *
 * This function turns on the specified relay, completing the circuit and allowing
 * current to flow through the relay's contacts. The relay is identified by the
 * `relayName` parameter.
 *
 * @param relayName The name or identifier of the relay to activate.
 */
extern void Relay_Activate(uint32_t relayName);

/**
 * Deactivates a specific relay.
 *
 * This function turns off the specified relay, opening the circuit and preventing
 * current from flowing through the relay's contacts. The relay is identified by the
 * `relayName` parameter.
 *
 * @param relayName The name or identifier of the relay to deactivate.
 */
extern void Relay_Deactivate(uint32_t relayName);

/**
 * Checks if a specific relay is activated.
 *
 * This function returns a boolean value indicating whether the specified relay
 * is currently activated. The relay is identified by the `relayName` parameter.
 *
 * @param relayName The name or identifier of the relay to check.
 * @return 1 if the relay is activated, 0 otherwise.
 */
extern uint32_t isRelayActivated(uint32_t relayName);

/******************************************************************************/

/******************************************************************************/
/* C++ Style GUARD */
/******************************************************************************/
#ifdef __cplusplus
}
#endif /* __cplusplus */
/******************************************************************************/

/******************************************************************************/
/* MULTIPLE INCLUSION GUARD */
/******************************************************************************/
#endif /* FILE_H_ */
/******************************************************************************/