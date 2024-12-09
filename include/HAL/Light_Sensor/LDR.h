/*******************************************************************************/
/**
 * @file switch.h
 * @brief Header file for switch control in the system.
 *
 * @par Project Name
 * Embedded Switch Control System
 *
 * @par Code Language
 * C
 *
 * @par Description
 * This header file provides functions and definitions for controlling switches
 * in the embedded system. It defines an enumeration for different switches,
 * along with functions to initialize and read the state of switches.
 *
 * @par Configuration
 * Before including this header file in your source code, ensure that you have
 * properly configured all switches in the switches_cfg.h header file. Each switch
 * should be assigned a unique identifier and its corresponding GPIO pin.
 *
 * @par How to Use
 * 1. Include this header file in your source code:
 *    #include "switch.h"
 * 
 * 2. Initialize the switch subsystem using `SWITCH_Init()` function.
 * 
 * 3. Use the provided functions to read the state of switches:
 *    - To read the state of a switch, use `SWITCH_enuGetStatus(switch,status)`.
 * 
 * @par Author
 * Mahmoud Abou-Hawis
 *
 ******************************************************************************/ 



/******************************************************************************/
/* MULTIPLE INCLUSION GUARD */
/******************************************************************************/
#ifndef LDR_H_
#define LDR_H_
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
#include "LDR_Conf.h"
/******************************************************************************/

/******************************************************************************/
/* PUBLIC DEFINES */
/******************************************************************************/


/******************************************************************************/

/******************************************************************************/
/* PUBLIC MACROS */
/******************************************************************************/

/******************************************************************************/

/******************************************************************************/
/* PUBLIC ENUMS */
/******************************************************************************/

/**
 * @brief Switch Error Status Enumeration
 *
 * This enumeration defines the possible error status values that can be returned
 * by ldr-related functions. It includes the following values:
 * - LDR_SUCCESS: The operation was successful.
 * - LDR_CONFIGURATION_FAILED: lm35 configuration failed.
 * - LDR_FAILED: General failure status.
 */
typedef enum
{
    LDR_SUCCESS,                 /**< Operation successful */
    LDR_CONFIGURATION_FAILED,    /**< ldr configuration failed */
    LDR_FAILED                   /**< General failure status */
} LDR_errorStatus;


/******************************************************************************/

/******************************************************************************/
/* PUBLIC TYPES */
/******************************************************************************/

/**
 * @brief ldr Configuration Structure
 * 
 * This structure defines the configuration for a lm35, including the GPIO port,
 * pin number, channel , channel rank,and sampling time.
 */
typedef struct
{
    void *  GPIO_Port;     /**< Pointer to the GPIO port used by the ADC */
    uint32_t GPIO_Pin;     /**< Pin number of the switch */
    uint32_t chl;          /**< the channel number */
    uint32_t Rank;         /**< channel rank */
    uint32_t SamplingTime; /**< channel sampling time */  
    uint32_t isEnabled;    /**< default channel status*/
} LDR_CFG_t;
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
 * @brief Initialize the LmM5 Module
 *
 * This function initializes the lm35 module. It configures the GPIO pins and
 * sets up any necessary hardware for lm35 functionality. If initialization
 * is successful, it returns LDR_SUCCESS; otherwise, it returns an error status.
 *
 * @return LDR_errorStatus
 * - LDR_SUCCESS: The switch module was successfully initialized.
 * - LDR_CONFIGURATION_FAILED: Switch configuration failed.
 */
extern LDR_errorStatus LDR_enuInit(void);


/**
 * Initializes the LDR temperature sensor for measurement.
 *
 * This function prepares the specified LDR sensor to start providing temperature readings. 
 * Once initialized, you can use other functions to retrieve the current temperature value.
 * 
 * @param ldrName The name or identifier of the LDR sensor to initialize.
 * 
 * @return LDR_errorStatus An error status indicating the success or failure 
 *          of the initialization process.
 * 
 * **Error Codes:**
 *   - LDR_OK: Initialization successful.
 *   - LDR_ERROR: failed.
 */
extern LDR_errorStatus LDR_Start(LDR_t ldrName);

/**
 * @brief Get the value of a ldr
 *
 * This function retrieves the value of the specified lm35 and stores
 * it in the provided pointer variable. If the lm35 status is successfully retrieved,
 * it returns LDR_SUCCESS; otherwise, it returns an error status.
 *
 * @param ldrName[in] The name or identifier of the lm35.
 * @param Value[in/out] Pointer to a variable where the lm35 value will be stored.
 * @return SWITCH_errorStatus
 * - LDR_SUCCESS: The switch status was successfully retrieved.
 * - LDR_FAILED: Failed to retrieve the switch status.
 */
extern LDR_errorStatus LDR_enuGetValue(LDR_t ldrName, 
                                         uint32_t *Vale);


/**
 * Enables a specific LDR (Light Dependent Resistor) sensor.
 *
 * This function activates the specified LDR sensor, making it ready to measure
 * light intensity. The sensor is identified by the `ldrName` parameter.
 *
 * @param ldrName The name or identifier of the LDR sensor to enable.
 */
extern void LDR_Enable(LDR_t ldrName);

/**
 * Disables a specific LDR (Light Dependent Resistor) sensor.
 *
 * This function deactivates the specified LDR sensor, stopping it from measuring
 * light intensity. The sensor is identified by the `ldrName` parameter.
 *
 * @param ldrName The name or identifier of the LDR sensor to disable.
 */
extern void LDR_Disable(LDR_t ldrName);

/**
 * Checks if a specific LDR (Light Dependent Resistor) sensor is enabled.
 *
 * This function returns a boolean value indicating whether the specified LDR
 * sensor is currently enabled. The sensor is identified by the `ldrName` parameter.
 *
 * @param ldrName The name or identifier of the LDR sensor to check.
 * @return 1 if the sensor is enabled, 0 otherwise.
 */
extern uint32_t isLDREnabled(LDR_t ldrName);

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
#endif /* SWITCH_H_ */
/******************************************************************************/