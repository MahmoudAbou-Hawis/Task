/*******************************************************************************/
/**
 * @file stm32f4xx_adc.h
 * @brief This header file provides a set of functions to initialize and control the 
 * ADC peripheral on STM32F4xx devices.
 *
 * @par Project Name
 * stm32f401xx drivers
 *
 * @par Code Language
 * C
 *
 * @par Description
 * This header file defines the ADC peripheral registers, structures,and function 
 * prototypes. It provides functions for:
 * - Initializing the ADC peripheral
 * - Configuring the ADC channels and conversion parameters
 * - Starting and stopping the ADC conversions
 * - Reading the ADC conversion results
 *
 * @par Author
 * Mahmoud Abou-Hawis
 *
 * @date [5/12/2024]
 *
 *******************************************************************************/

/******************************************************************************/
/* MULTIPLE INCLUSION GUARD */
/******************************************************************************/
#ifndef __STM32F4xx_ADC_H
#define __STM32F4xx_ADC_H
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
#include <stddef.h>
#include <stdint.h>
/******************************************************************************/

/******************************************************************************/
/* PUBLIC DEFINES */
/******************************************************************************/

/** @defgroup Instances  ADC Instances
  * @{
  */
#define  ADC1                                                ((void*)0x40012000)
/**
  * @}
  */

/** @defgroup ADC_Resolution ADC Resolution
  * @{
  */ 

#define ADC_RESOLUTION_12B  0x00000000U
#define ADC_RESOLUTION_10B  ((uint32_t)0x01000000)
#define ADC_RESOLUTION_8B   ((uint32_t)0x02000000)
#define ADC_RESOLUTION_6B   ((uint32_t)0x03000000)
/**
  * @}
  */ 


 /** @defgroup ADC_sampling_times  ADC Sampling Times
  * @{
  */ 
#define ADC_SAMPLETIME_3CYCLES    0x00000000U
#define ADC_SAMPLETIME_15CYCLES   ((uint32_t)0x00000001)
#define ADC_SAMPLETIME_28CYCLES   ((uint32_t)0x00000002)
#define ADC_SAMPLETIME_56CYCLES   ((uint32_t)0x00000003)
#define ADC_SAMPLETIME_84CYCLES   ((uint32_t)0x00000004)
#define ADC_SAMPLETIME_112CYCLES  ((uint32_t)0x00000005)
#define ADC_SAMPLETIME_144CYCLES  ((uint32_t)0x00000006)
#define ADC_SAMPLETIME_480CYCLES  ((uint32_t)0x00000007)
/**
  * @}
  */ 





/** @defgroup ADC_channels  ADC  Channels
  * @{
  */ 
#define ADC_CHANNEL_0           0x00000000U
#define ADC_CHANNEL_1           ((uint32_t)0x00000001)
#define ADC_CHANNEL_2           ((uint32_t)0x00000002)
#define ADC_CHANNEL_3           ((uint32_t)0x00000003)
#define ADC_CHANNEL_4           ((uint32_t)0x00000004)
#define ADC_CHANNEL_5           ((uint32_t)0x00000005)
#define ADC_CHANNEL_6           ((uint32_t)0x00000006)
#define ADC_CHANNEL_7           ((uint32_t)0x00000007)
#define ADC_CHANNEL_8           ((uint32_t)0x00000008)
#define ADC_CHANNEL_9           ((uint32_t)0x00000009)
#define ADC_CHANNEL_10          ((uint32_t)0x0000000A)
#define ADC_CHANNEL_11          ((uint32_t)0x0000000B)
#define ADC_CHANNEL_12          ((uint32_t)0x0000000C)
#define ADC_CHANNEL_13          ((uint32_t)0x0000000D)
#define ADC_CHANNEL_14          ((uint32_t)0x0000000E)
#define ADC_CHANNEL_15          ((uint32_t)0x0000000F)
#define ADC_CHANNEL_16          ((uint32_t)0x00000010)
#define ADC_CHANNEL_17          ((uint32_t)0x00000011)
#define ADC_CHANNEL_18          ((uint32_t)0x00000012)

/**
  * @}
  */ 
  


/** @defgroup ADC_EOCSelection ADC EOC Selection
 * @{
 */ 
#define ADC_EOC_SEQ_CONV              0x00000000U
#define ADC_EOC_SINGLE_CONV           0x00000001U
/**
  * @}
  */ 


/** @defgroup ADC_Data_align ADC Data Align
  * @{
  */ 
#define ADC_DATAALIGN_RIGHT      0x00000000U
#define ADC_DATAALIGN_LEFT       ((uint32_t)0x00000800)
/**
  * @}
  */ 


/** @defgroup Operations
  * @{
  */ 
#define DISABLE                  0x00000000U
#define Enable_SCAN_MODE         ((uint32_t)(0x1 << 8))
#define ENABLE_CONT_MODE         ((uint32_t)(0x1 << 1))
/**
  * @}
  */ 
/******************************************************************************/

/******************************************************************************/
/* PUBLIC MACROS */
/******************************************************************************/

/******************************************************************************/

/******************************************************************************/
/* PUBLIC ENUMS */
/******************************************************************************/


/** @defgroup ADC_Error ADC Errors
  * @{
  */

/**
 * @typedef ADC_ErrorStatus_t
 * @brief Enumerates the different ADC error status codes.
 */
typedef enum
{
  /** @enum{ADC_NULL_PTR_PASSED} - Error code indicating a null pointer 
   *        was passed to a ADC function. */
  ADC_NULL_PTR_PASSED,

  /** @enum{ADC_ERROR} - General error code for any ADC error . */
  ADC_ERROR,

   /** @enum{ADC_PARAM_ERROR} - Error code indicating invalid 
    *       parameters were passed to a ADC function. 
   *                          
   */
  ADC_PARAM_ERROR,

   /** @enum{ADC_OK} - No Error
   *                          
   */
  ADC_OK
} ADC_ErrorStatus_t;

/**
  * @}
  */





/******************************************************************************/

/******************************************************************************/
/* PUBLIC TYPES */
/******************************************************************************/

/**
 * @brief ADC Initialization Structure
 * 
 * This structure defines the parameters for initializing the ADC peripheral.
 * 
 * @details
 * | Member Name          | Description                                                |
 * |----------------------|------------------------------------------------------------|
 * | Resolution           | Resolution of the ADC @ref ADC_Resolution.                 |
 * | DataAlignment        | Data alignment (Left or Right). @ref ADC_Data_align        |
 * | ScanConvMode         | Whether to use scan conversion mode @ref Operations        |
 * | ContinuousConvMode   | Continuous mode or single conversion mode @ref Operations. |
 * | EOCSelection         | End of conversion selection . @ref ADC_EOCSelection        |
 */
typedef struct
{
    uint32_t Resolution;
    uint32_t DataAlignment;
    uint32_t ScanConvMode;
    uint32_t ContinuousConvMode;
    uint32_t EOCSelection;
    uint32_t NbrOfConversion;

} ADC_Init_t;

/**
 * @brief ADC Channel Structure
 * 
 * This structure defines the parameters for configuring an ADC channel.
 * 
 * @details
 * | Member Name          | Description                                            |
 * |----------------------|--------------------------------------------------------|
 * | Channel              | ADC channel number.  @ref ADC_channels                 |
 * | Rank                 | Rank of the channel in scan conversion mode.           |
 * | SamplingTime         | Sampling time for the channel. @ref ADC_sampling_times |
 */
typedef struct
{
    uint32_t Channel;
    uint32_t Rank;
    uint32_t SamplingTime;
} ADC_Channel_t;

/**
 * @brief ADC Handle Structure
 * 
 * This structure represents an ADC handle, containing the instance pointer,
 *  channel configuration, and initialization parameters.
 * 
 * @details
 * | Member Name          | Description                                         |
 * |----------------------|-----------------------------------------------------|
 * | Instance             | Pointer to the ADC peripheral instance.             |
 * | channel              | ADC channel configuration.                          |
 * | init                 | ADC initialization parameters.                      |
 */
typedef struct
{
    void * Instance;
    ADC_Init_t init;
} ADC_Handle_t;



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

/** @defgroup ADC_Functions ADC Functions
  * @{
  */

/**
 * @brief Initializes the ADC peripheral.
 *
 * This function initializes the ADC peripheral with the specified parameters.
 *
 * @param hadc: Pointer to a ADC_Handle_t structure that 
 *              contains the configuration information for the ADC peripheral.
 *
 * @return An ADC_ErrorStatus_t enumeration value 
 *         indicating the status of the initialization process.
 */ 
extern ADC_ErrorStatus_t ADC_Init(ADC_Handle_t *hadc);

/**
 * @brief Starts the ADC conversion in and Enable Interrupts .
 *
 *
 * @param hadc: Pointer to a ADC_Handle_t structure that contains 
 *              the configuration information for the ADC peripheral.
 *
 * @return An ADC_ErrorStatus_t enumeration value 
 *  indicating the status of the start operation.
 */
extern ADC_ErrorStatus_t ADC_StartConversionAsync(void* hadc);

/**
 * @brief Stops the ADC conversion .
 *
 * This function stops the ADC conversion and Disable interrupts.
 *
 * @param hadc: Pointer to a ADC_Handle_t structure that contains 
 *              the configuration information for the ADC peripheral.
 *
 * @return An ADC_ErrorStatus_t enumeration value indicating the 
 *         status of the stop operation.
 */
extern ADC_ErrorStatus_t ADC_StopAsync(void* hadc);

/**
 * @brief Gets the value of the last ADC conversion.
 *
 * This function gets the value of the last ADC conversion.
 *
 * @param[in] hadc: Pointer to a ADC_Handle_t structure that contains
 *                  the configuration information for the ADC peripheral.
 * @param[in out] value: Pointer to a uint32_t variable that will 
 *                       store the value of the last ADC conversion.
 *
 * @return An ADC_ErrorStatus_t enumeration value indicating the 
 *         status of the value retrieval operation.
 */
extern ADC_ErrorStatus_t ADC_GetValue(uint32_t * value);


extern ADC_ErrorStatus_t ADC_ConfigChannel(void *hadc , ADC_Channel_t * channel);

/**
  * @}
  */


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
#endif /* __STM32F4xx_ADC_H */
/******************************************************************************/
