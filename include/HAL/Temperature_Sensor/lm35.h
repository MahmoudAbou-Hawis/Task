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
#ifndef LM35_H_
#define LM35_H_
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
#include "lm35_conf.h"
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
 * by lm35-related functions. It includes the following values:
 * - LM35_SUCCESS: The operation was successful.
 * - LM35_CONFIGURATION_FAILED: lm35 configuration failed.
 * - LM35_FAILED: General failure status.
 */
typedef enum
{
    LM35_SUCCESS,                 /**< Operation successful */
    LM35_CONFIGURATION_FAILED,    /**< lm35 configuration failed */
    LM35_FAILED                   /**< General failure status */
} LM35_errorStatus;


/******************************************************************************/

/******************************************************************************/
/* PUBLIC TYPES */
/******************************************************************************/

/**
 * @brief lm35 Configuration Structure
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
} LM35_CFG_t;
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
 * is successful, it returns LM35_SUCCESS; otherwise, it returns an error status.
 *
 * @return LM35_errorStatus
 * - LM35_SUCCESS: The switch module was successfully initialized.
 * - LM35_CONFIGURATION_FAILED: Switch configuration failed.
 */
extern LM35_errorStatus LM35_enuInit(void);


/**
 * Initializes the LM35 temperature sensor for measurement.
 *
 * This function prepares the specified LM35 sensor to start providing temperature readings. 
 * Once initialized, you can use other functions to retrieve the current temperature value.
 * 
 * @param lm35Name The name or identifier of the LM35 sensor to initialize.
 * 
 * @return LM35_errorStatus An error status indicating the success or failure 
 *          of the initialization process.
 * 
 * **Error Codes:**
 *   - LM35_OK: Initialization successful.
 *   - LM35_ERROR: failed.
 */
extern LM35_errorStatus LM35_Start(LM35_t lm35Name);

/**
 * @brief Get the value of a lm35
 *
 * This function retrieves the value of the specified lm35 and stores
 * it in the provided pointer variable. If the lm35 status is successfully retrieved,
 * it returns LM35_SUCCESS; otherwise, it returns an error status.
 *
 * @param lm35Name[in] The name or identifier of the lm35.
 * @param Value[in/out] Pointer to a variable where the lm35 value will be stored.
 * @return SWITCH_errorStatus
 * - LM35_SUCCESS: The switch status was successfully retrieved.
 * - LM35_FAILED: Failed to retrieve the switch status.
 */
extern LM35_errorStatus LM35_enuGetValue(LM35_t lm35Name, 
                                         uint32_t *Vale);


/**
 * Enables a specific LM35 temperature sensor.
 *
 * This function activates the specified LM35 sensor, making it ready to provide
 * temperature readings. The sensor is identified by the `lm35Name` parameter.
 *
 * @param lm35Name The name or identifier of the LM35 sensor to enable.
 */
extern void LM35_Enable(LM35_t lm35Name);

/**
 * Disables a specific LM35 temperature sensor.
 *
 * This function deactivates the specified LM35 sensor, stopping it from providing
 * temperature readings. The sensor is identified by the `lm35Name` parameter.
 *
 * @param lm35Name The name or identifier of the LM35 sensor to disable.
 */
extern void LM35_Disable(LM35_t lm35Name);

/**
 * Checks if a specific LM35 temperature sensor is enabled.
 *
 * This function returns a boolean value indicating whether the specified LM35
 * sensor is currently enabled. The sensor is identified by the `lm35Name` parameter.
 *
 * @param lm35Name The name or identifier of the LM35 sensor to check.
 * @return 1 if the sensor is enabled, 0 otherwise.
 */
extern uint32_t isEnabled(LM35_t lm35Name);

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