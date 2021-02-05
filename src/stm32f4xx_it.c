/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f4xx_it.h"
#include <stdbool.h>                   // boolean type


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
 * @brief PIR object detection status: used by application and PIR ISR.
 */
extern bool pir_object_detected;

/*
 * @brief camera DMA - DCMI interface
 */
extern DCMI_HandleTypeDef cam_dcmi_;
extern DMA_HandleTypeDef dma_;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/
void
HardFault_Handler (void)
{
  while (1)
    {
      ; // do nothing
    }
}

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

/*
 * @name   EXTI1_IRQHandler.
 * @brief  handles PIR detection IRQ
 * @param  none.
 * @retval none.
 */
void
EXTI1_IRQHandler (void)
{
  HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_1);
  HAL_NVIC_ClearPendingIRQ (EXTI1_IRQn);

  /* signal object detection */
  pir_object_detected = true;
}

void
DMA2_Stream1_IRQHandler (void)
{
  HAL_DMA_IRQHandler (&dma_);
}

//void
//DCMI_IRQHandler (void)
//{
//  HAL_DCMI_IRQHandler(&cam_dcmi_);
//}
