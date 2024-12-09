/******************************************************************************/
/**
 * @file DMA.c
 * @brief **STM32 DMA Peripheral Driver**
 *
 * This file implements a driver for the Direct Memory Access (DMA) controller 
 * on STM32 microcontrollers. It provides functions for initializing DMA streams, 
 * configuring transfers, and starting/stopping DMA operations.
 *
 * @par Project Name
 * **(Replace with your project name)**
 * 
 * @par Code Language
 * C
 *
 * @par Description
 * This driver provides basic functionalities for DMA configuration and transfer 
 * management. It can be adapted to work with different STM32 peripherals that 
 * support DMA transfers.
 *
 * @par Author
 * Mahmoud Abou-Hawis
 *
 */
/******************************************************************************/


/******************************************************************************/
/* INCLUDES */
/******************************************************************************/
#include "stm32f4xx_adc.h"
#include <stddef.h>

/******************************************************************************/

/******************************************************************************/
/* PRIVATE DEFINES */
/******************************************************************************/

/**
 * @brief  Default HSI frequency: 16 MHz
 */
#define SYSTEM_CORE_CLOCK_HSI    16000000U 

/**
 * @brief  Defines 'read / write' structure member permissions.
 */
#define __IOM volatile


#define ADC_CR1_SCAN_Pos          (8U)                                         
#define ADC_CR1_SCAN_Msk          (0x1UL << ADC_CR1_SCAN_Pos)                 
#define ADC_CR1_SCAN              ADC_CR1_SCAN_Msk                             


#define ADC_CR1_RES_Pos           (24U)                                        
#define ADC_CR1_RES_Msk           (0x3UL << ADC_CR1_RES_Pos)                   
#define ADC_CR1_RES               ADC_CR1_RES_Msk    


#define ADC_CR1_DISCEN_Pos        (11U)                                        
#define ADC_CR1_DISCEN_Msk        (0x1UL << ADC_CR1_DISCEN_Pos)                
#define ADC_CR1_DISCEN            ADC_CR1_DISCEN_Msk                


#define ADC_CR2_ALIGN_Pos         (11U)                                        
#define ADC_CR2_ALIGN_Msk         (0x1UL << ADC_CR2_ALIGN_Pos)                  
#define ADC_CR2_ALIGN             ADC_CR2_ALIGN_Msk  


#define ADC_CR2_CONT_Pos          (1U)                                         
#define ADC_CR2_CONT_Msk          (0x1UL << ADC_CR2_CONT_Pos)         
#define ADC_CR2_CONT              ADC_CR2_CONT_Msk                             

#define ADC_CR2_EOCS_Pos          (10U)                                        
#define ADC_CR2_EOCS_Msk          (0x1UL << ADC_CR2_EOCS_Pos)                  
#define ADC_CR2_EOCS              ADC_CR2_EOCS_Msk                        


/******************************************************************************/

/******************************************************************************/

/******************************************************************************/
/* PRIVATE MACROS */
/******************************************************************************/
#define IS_NULL_PARAM(PARAM) ((PARAM) == NULL)

#define IS_VALID_CHANNEL(CHANNEL) \
    (((CHANNEL) == ADC_CHANNEL_0)  || \
     ((CHANNEL) == ADC_CHANNEL_1)  || \
     ((CHANNEL) == ADC_CHANNEL_2)  || \
     ((CHANNEL) == ADC_CHANNEL_3)  || \
     ((CHANNEL) == ADC_CHANNEL_4)  || \
     ((CHANNEL) == ADC_CHANNEL_5)  || \
     ((CHANNEL) == ADC_CHANNEL_6)  || \
     ((CHANNEL) == ADC_CHANNEL_7)  || \
     ((CHANNEL) == ADC_CHANNEL_8)  || \
     ((CHANNEL) == ADC_CHANNEL_9)  || \
     ((CHANNEL) == ADC_CHANNEL_10) || \
     ((CHANNEL) == ADC_CHANNEL_11) || \
     ((CHANNEL) == ADC_CHANNEL_12) || \
     ((CHANNEL) == ADC_CHANNEL_13) || \
     ((CHANNEL) == ADC_CHANNEL_14) || \
     ((CHANNEL) == ADC_CHANNEL_15) || \
     ((CHANNEL) == ADC_CHANNEL_16) || \
     ((CHANNEL) == ADC_CHANNEL_17) || \
     ((CHANNEL) == ADC_CHANNEL_18))


#define IS_VALID_SAMPLING_TIME(SAMPLINGTIME) \
    (((SAMPLINGTIME) == ADC_SAMPLETIME_3CYCLES)   || \
     ((SAMPLINGTIME) == ADC_SAMPLETIME_15CYCLES)  || \
     ((SAMPLINGTIME) == ADC_SAMPLETIME_28CYCLES)  || \
     ((SAMPLINGTIME) == ADC_SAMPLETIME_56CYCLES)  || \
     ((SAMPLINGTIME) == ADC_SAMPLETIME_84CYCLES)  || \
     ((SAMPLINGTIME) == ADC_SAMPLETIME_112CYCLES) || \
     ((SAMPLINGTIME) == ADC_SAMPLETIME_144CYCLES) || \
     ((SAMPLINGTIME) == ADC_SAMPLETIME_480CYCLES))

#define IS_VALID_RANK(RANK) (((RANK) >= 1) && ((RANK) <= 16))




#define IS_VALID_RESOLUTION(RESOLUTION) \
    (((RESOLUTION) == ADC_RESOLUTION_12B) || \
     ((RESOLUTION) == ADC_RESOLUTION_10B) || \
     ((RESOLUTION) == ADC_RESOLUTION_8B)  || \
     ((RESOLUTION) == ADC_RESOLUTION_6B))


#define IS_VALID_EOC_SELECTION(EOC_SELECTION) \
    (((EOC_SELECTION) == ADC_EOC_SEQ_CONV) || \
    ((EOC_SELECTION) == ADC_EOC_SINGLE_CONV))

#define IS_VALID_DATA_ALIGN(DATA_ALIGN) \
    (((DATA_ALIGN) == ADC_DATAALIGN_RIGHT) || \
     ((DATA_ALIGN) == ADC_DATAALIGN_LEFT))

#define IS_VALID_ENABLE(STATE) \
    (((STATE) == DISABLE) || \
     ((STATE) == ENABLE_CONT_MODE) || \
     ((STATE) == Enable_SCAN_MODE))

#define IS_VALID_INSTANCE(INSTANCE) \
    ((INSTANCE) == ADC1)

#define ADC_CR2_ADON_Pos          (0U)                                         
#define ADC_CR2_ADON_Msk          (0x1UL << ADC_CR2_ADON_Pos)                  
#define ADC_CR2_ADON              ADC_CR2_ADON_Msk               


#define ADC_SR_EOC_Pos            (1U)                                         
#define ADC_SR_EOC_Msk            (0x1UL << ADC_SR_EOC_Pos)                     
#define ADC_SR_EOC                ADC_SR_EOC_Msk            
#define ADC_FLAG_EOC              ((uint32_t)ADC_SR_EOC)

#define ADC_SR_OVR_Pos            (5U)                                         
#define ADC_SR_OVR_Msk            (0x1UL << ADC_SR_OVR_Pos)                   
#define ADC_SR_OVR                ADC_SR_OVR_Msk        
#define ADC_FLAG_OVR              ((uint32_t)ADC_SR_OVR)

#define ADC_CR1_EOCIE_Pos         (5U)                                         
#define ADC_CR1_EOCIE_Msk         (0x1UL << ADC_CR1_EOCIE_Pos)                 
#define ADC_CR1_EOCIE             ADC_CR1_EOCIE_Msk             
#define ADC_IT_EOC               ((uint32_t)ADC_CR1_EOCIE)

#define ADC_SQR1_L_Pos            (20U)                                        
#define ADC_SQR1_L_Msk            (0xFUL << ADC_SQR1_L_Pos)                     
#define ADC_SQR1_L                ADC_SQR1_L_Msk       

#define ADC_CR2_SWSTART_Pos       (30U)                                        
#define ADC_CR2_SWSTART_Msk       (0x1UL << ADC_CR2_SWSTART_Pos)               
#define ADC_CR2_SWSTART           ADC_CR2_SWSTART_Msk       
/******************************************************************************/
/* PRIVATE ENUMS */
/******************************************************************************/

/******************************************************************************/

/******************************************************************************/
/* PRIVATE TYPES */
/******************************************************************************/

/**
 * @brief ADC internel registers
 */
typedef struct
{
  __IOM uint32_t SR;     /*!< ADC status register,                        */
  __IOM uint32_t CR1;    /*!< ADC control register 1,                     */
  __IOM uint32_t CR2;    /*!< ADC control register 2,                     */
  __IOM uint32_t SMPR1;  /*!< ADC sample time register 1,                 */
  __IOM uint32_t SMPR2;  /*!< ADC sample time register 2,                 */
  __IOM uint32_t JOFR1;  /*!< ADC injected channel data offset register 1,*/
  __IOM uint32_t JOFR2;  /*!< ADC injected channel data offset register 2,*/
  __IOM uint32_t JOFR3;  /*!< ADC injected channel data offset register 3,*/
  __IOM uint32_t JOFR4;  /*!< ADC injected channel data offset register 4,*/
  __IOM uint32_t HTR;    /*!< ADC watchdog higher threshold register,     */
  __IOM uint32_t LTR;    /*!< ADC watchdog lower threshold register,      */
  __IOM uint32_t SQR1;   /*!< ADC regular sequence register 1,            */
  __IOM uint32_t SQR2;   /*!< ADC regular sequence register 2,            */
  __IOM uint32_t SQR3;   /*!< ADC regular sequence register 3,            */
  __IOM uint32_t JSQR;   /*!< ADC injected sequence register,             */
  __IOM uint32_t JDR1;   /*!< ADC injected data register 1,               */
  __IOM uint32_t JDR2;   /*!< ADC injected data register 2,               */
  __IOM uint32_t JDR3;   /*!< ADC injected data register 3,               */
  __IOM uint32_t JDR4;   /*!< ADC injected data register 4,               */
  __IOM uint32_t DR;     /*!< ADC regular data register,                  */
} ADC_t;



/******************************************************************************/

/******************************************************************************/
/* PRIVATE CONSTANT DEFINITIONS */
/******************************************************************************/


/******************************************************************************/

/******************************************************************************/
/* PRIVATE VARIABLE DEFINITIONS */
/******************************************************************************/
static uint16_t adcValue = 1; 
static volatile uint8_t flag = 0;
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

ADC_ErrorStatus_t ADC_Init(ADC_Handle_t *hadc) {
    if(IS_NULL_PARAM(hadc) ||
       !IS_VALID_RESOLUTION(hadc->init.Resolution) ||
       !IS_VALID_EOC_SELECTION(hadc->init.EOCSelection) ||
       !IS_VALID_DATA_ALIGN(hadc->init.DataAlignment) ||
       !IS_VALID_ENABLE(hadc->init.ContinuousConvMode) ||
       !IS_VALID_ENABLE(hadc->init.ScanConvMode) ||
       !IS_VALID_INSTANCE(hadc->Instance)) 
    {
        return ADC_PARAM_ERROR;
    }

    ADC_t *adc = (ADC_t*)hadc->Instance;
    uint32_t cr1 = 0;
    uint32_t cr2 = 0;

    cr1 |= hadc->init.ScanConvMode;
    cr1 |= hadc->init.Resolution;

    cr2 |= hadc->init.DataAlignment;
    cr2 |= (hadc->init.ContinuousConvMode << ADC_CR2_CONT_Pos);
    cr2 |= (hadc->init.EOCSelection << ADC_CR2_EOCS_Pos);

    adc->CR1 = cr1 | ADC_CR1_EOCIE;
    adc->CR2 = cr2 | ADC_CR2_ADON;   

    adc->SQR1 &= ~ADC_SQR1_L; 
    adc->SQR1 |= ((hadc->init.NbrOfConversion - 1) << ADC_SQR1_L_Pos);

    return ADC_OK;
}


ADC_ErrorStatus_t ADC_StartConversionAsync(void *Instance)
{
    ADC_ErrorStatus_t RET_enuErrorStatus = ADC_OK;
    if(IS_NULL_PARAM(Instance))
    {
        RET_enuErrorStatus = ADC_PARAM_ERROR;
    }
    else
    {
        ((ADC_t*)Instance)->CR2 |= ADC_CR2_SWSTART; 
        flag = 1;
    }
    return RET_enuErrorStatus;
}

ADC_ErrorStatus_t ADC_StopAsync(void *Instance)
{
    ADC_ErrorStatus_t RET_enuErrorStatus = ADC_OK;
    if(IS_NULL_PARAM(Instance))
    {
        RET_enuErrorStatus = ADC_PARAM_ERROR;
    }
    else
    {
        ((ADC_t*)Instance)->CR2 &= ~(ADC_CR2_ADON);
        ((ADC_t*)Instance)->SR = ~(ADC_FLAG_EOC);
        ((ADC_t*)Instance)->CR1 &= ~(ADC_IT_EOC );
    }
    return RET_enuErrorStatus;
}

ADC_ErrorStatus_t ADC_GetValue(uint32_t *value)
{

    ADC_ErrorStatus_t RET_enuErrorStatus = ADC_OK;
    if(IS_NULL_PARAM(value))
    {
        RET_enuErrorStatus = ADC_PARAM_ERROR;
    }
    else
    {
        *value = adcValue;
    }    
    return RET_enuErrorStatus;
}

ADC_ErrorStatus_t ADC_ConfigChannel(void *Instance , ADC_Channel_t * channel)
{
    ADC_ErrorStatus_t RET_enuErrorStatus = ADC_OK;
    
    if (IS_NULL_PARAM(Instance)                        ||
        IS_NULL_PARAM(channel)                         ||
        !IS_VALID_CHANNEL(channel->Channel)            ||
        !IS_VALID_SAMPLING_TIME(channel->SamplingTime) ||
        !IS_VALID_RANK(channel->Rank)
    )
    {
        RET_enuErrorStatus = ADC_PARAM_ERROR;  
    }
    else
    {
        if(channel->Channel > 9) {
            ((ADC_t*)Instance)->SMPR1 &= ~(0x7 << (3 * (channel->Channel - 10))); 
            ((ADC_t*)Instance)->SMPR1 |= (channel->SamplingTime << (3 * (channel->Channel - 10)));
        } else {
            ((ADC_t*)Instance)->SMPR2 &= ~(0x7 << (3 * channel->Channel)); 
            ((ADC_t*)Instance)->SMPR2 |= (channel->SamplingTime << (3 * channel->Channel));
        }

        uint32_t rankIndex = channel->Rank - 1;
        if (rankIndex < 6) {
            ((ADC_t*)Instance)->SQR3 &= ~(0x1F << (5 * rankIndex)); 
            ((ADC_t*)Instance)->SQR3 |= (channel->Channel << (5 * rankIndex));
        } else if (rankIndex < 12) {
            ((ADC_t*)Instance)->SQR2 &= ~(0x1F << (5 * (rankIndex - 6))); 
            ((ADC_t*)Instance)->SQR2 |= (channel->Channel << (5 * (rankIndex - 6)));
        } else {
            ((ADC_t*)Instance)->SQR1 &= ~(0x1F << (5 * (rankIndex - 12))); 
            ((ADC_t*)Instance)->SQR1 |= (channel->Channel << (5 * (rankIndex - 12)));
        }
    }

    return RET_enuErrorStatus;
}

void ADC_IRQHandler(void) {
    flag = 0;
    if (((ADC_t*)ADC1)->SR & ADC_SR_EOC) { 
        adcValue = (uint16_t)((ADC_t*)ADC1)->DR; 
        ((ADC_t*)ADC1)->SR &= ~ADC_SR_EOC;
    }
}
