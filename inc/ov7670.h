/*
 ******************************************************************************
 * @file    ov7670.h
 * @module  app
 * @author  Mohamed Hnezli
 * @version V0.2
 * @date    17-09-2019
 * @brief   defines OV7670 camera driver.
 *
 ******************************************************************************
 * @attention
 *
 * - The present software had been tested only on STM32F407VGT6 and may be non
 * functional on other targets.
 *
 * <h2><center>&copy COPYRIGHT 2019 Mohamed Hnezli </center></h2>
 ******************************************************************************
 */

/* include ------------------------------------------------------------------ */
#include "stm32f4xx.h"
#include <stdbool.h>


/* define ------------------------------------------------------------------- */

/*   Camera wiring:
 *   3V3		-	3V		;		GND		-	GND
 *   SIOC	-	PB8		;		SIOD	-	PB9
 *   VSYNC -	PB7		;		HREF	-	PA4
 *   PCLK	-	PA6		;		XCLK	-	PA8
 *   D7		-	PE6		;		D6		-	PE5
 *   D5		-	PB6		;		D4		-	PE4
 *   D3		-	PC9		;		D2		-	PC8
 *   D1		-	PC7		;		D0		-	PC6
 *   RESET	-	/			;		PWDN	-	/
 *
 *	==========================================================================
 */

/*
 * @brief drivers interfaces.
 */
#define I2C_CAM                        I2C2
#define I2C_CLK_EN_CAM()               __HAL_RCC_I2C2_CLK_ENABLE()

#define DCMI_CAM                       DCMI
#define DCMI_CLK_EN_CAM()              __HAL_RCC_DCMI_CLK_ENABLE()

#define DMA_CAM                        DMA2_Stream1
#define DMA_CLK_EN_CAM()               __HAL_RCC_DMA2_CLK_ENABLE()

/*
 * @brief sensor GPIO ports and pins
 */
#define GPIO_PORT_CAM_I2C              GPIOB
#define GPIO_PIN_I2C_CAM_SCL           GPIO_PIN_10
#define GPIO_PIN_I2C_CAM_SDA           GPIO_PIN_11
#define GPIO_AFUNC_CAM_I2C             GPIO_AF4_I2C2
#define GPIO_CLK_EN_CAM_I2C()          __HAL_RCC_GPIOB_CLK_ENABLE()

#define GPIO_PORT_CAM_DCMI_A           GPIOA
#define GPIO_PORT_CAM_DCMI_B           GPIOB
#define GPIO_PORT_CAM_DCMI_C           GPIOC
#define GPIO_PORT_CAM_DCMI_E           GPIOE
#define GPIO_PIN_DCMI_CAM_D0           GPIO_PIN_6          // PC6
#define GPIO_PIN_DCMI_CAM_D1           GPIO_PIN_10//PA10   GPIO_PIN_7          // PC7
#define GPIO_PIN_DCMI_CAM_D2           GPIO_PIN_0          // PE0
#define GPIO_PIN_DCMI_CAM_D3           GPIO_PIN_1          // PE1
#define GPIO_PIN_DCMI_CAM_D4           GPIO_PIN_4          // PE4
#define GPIO_PIN_DCMI_CAM_D5           GPIO_PIN_6          // PB6
#define GPIO_PIN_DCMI_CAM_D6           GPIO_PIN_5          // PE5
#define GPIO_PIN_DCMI_CAM_D7           GPIO_PIN_6          // PE6
#define GPIO_PIN_DCMI_CAM_HSYNC        GPIO_PIN_4          // PA4
#define GPIO_PIN_DCMI_CAM_VSYNC        GPIO_PIN_7          // PB7
#define GPIO_PIN_DCMI_CAM_PIXCK        GPIO_PIN_6          // PA6
#define GPIO_AFUNC_CAM_DCMI            GPIO_AF13_DCMI
#define GPIO_CLK_EN_CAM_DCMI()         do{\
                                          __HAL_RCC_GPIOA_CLK_ENABLE();\
                                          __HAL_RCC_GPIOB_CLK_ENABLE();\
                                          __HAL_RCC_GPIOC_CLK_ENABLE();\
                                          __HAL_RCC_GPIOE_CLK_ENABLE();\
                                       }while(0)

#define GPIO_PORT_CAM_MCO              GPIOA
#define GPIO_PIN_CAM_MCO               GPIO_PIN_8
#define GPIO_AFUNC_CAM_MCO             GPIO_AF0_MCO
#define GPIO_CLK_EN_CAM_MCO()          __HAL_RCC_GPIOB_CLK_ENABLE()

/*
 * @brief  IRQ parameters
 */
#define IRQ_LINE_DMA_CAM               DMA2_Stream1_IRQn
#define IRQ_PRIO_DMA_CAM               1

// SCCB write address
#define SCCB_REG_ADDR        0x01

// OV7670 camera settings
#define OV7670_REG_NUM       121
#define OV7670_WRITE_ADDR    0x42

// Image settings
#define IMG_ROWS             320//144
#define IMG_COLUMNS          240//174

/* typedef ------------------------------------------------------------------ */
/* variable ----------------------------------------------------------------- */
/*
 * @brief image buffer
 */
extern volatile uint16_t frame_buffer[IMG_ROWS*IMG_COLUMNS];


/* class -------------------------------------------------------------------- */
/* method ------------------------------------------------------------------- */
/* function ----------------------------------------------------------------- */
 void
 ov7670_hwinit (void);

/*
 * @name   error_critical.
 * @brief  halts MCU on hard error occurrence.
 * @param  file: name of file in which error occurred.
 *         line: number of line in which error occurred.
 * @retval none.
 */
bool
OV7670_init (void);
