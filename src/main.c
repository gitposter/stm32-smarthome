/*
 ******************************************************************************
 * @file    main.c
 * @module  app
 * @author  Mohamed Hnezli
 * @version V0.2
 * @date    08-09-2019
 * @brief   implements security system based on proximity+temp+gaz sensor. It
 *          able to print output to tiny 16x2 LCD on take camshots and print
 *          them on bigger LCD.
 ******************************************************************************
 * @attention
 *
 * - The present software had been tested only on STM32F3CCTx and may be non
 * functional on other targets.
 *
 * <h2><center>&copy COPYRIGHT 2019 Mohamed Hnezli </center></h2>
 ******************************************************************************
 */

/* include ------------------------------------------------------------------ */
#include "stm32f4xx.h"                 // MCU HAL
#include "stm32f4_discovery.h"         // BSP
#include <stdbool.h>                   // boolean type
#include <stdio.h>                     // snprintf
#include "STM_MY_LCD16X2.h"            // LCD 16x2 driver
#include "ov7670.h"                    // camera driver
#include "lcd.h"                       // display driver
#include "gui.h"                       // display driver
#include <string.h>                    // strlen


/* define ------------------------------------------------------------------- */
/*
 * @brief sensors interfaces
 */
#define ADC_SENSORS                    ADC1
#define ADC_CLK_EN_SENSORS()           __HAL_RCC_ADC1_CLK_ENABLE()
#define ADC_CH_LM35                    ADC_CHANNEL_1
#define ADC_CH_MQ5                     ADC_CHANNEL_3

/*
 * @brief sensor GPIO ports and pins
 */
#define GPIO_PORT_PIR                  GPIOB
#define GPIO_PIN_PIR                   GPIO_PIN_1
#define GPIO_CLK_EN_PIR()              __HAL_RCC_GPIOB_CLK_ENABLE()

#define GPIO_PORT_LM35                 GPIOA
#define GPIO_PIN_LM35                  GPIO_PIN_1
#define GPIO_CLK_EN_LM35()             __HAL_RCC_GPIOA_CLK_ENABLE()

#define GPIO_PORT_MQ5                  GPIOA
#define GPIO_PIN_MQ5                   GPIO_PIN_3
#define GPIO_CLK_EN_MQ5()              __HAL_RCC_GPIOB_CLK_ENABLE()

#define GPIO_PORT_BUZZER               GPIOC
#define GPIO_PIN_BUZZER                GPIO_PIN_1
#define GPIO_CLK_EN_BUZZER()           __HAL_RCC_GPIOC_CLK_ENABLE()

#define GPIO_PORT_HC06_TX              GPIOD
#define GPIO_PIN_HC06_TX               GPIO_PIN_8
#define GPIO_CLK_EN_HC06_TX()          __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PORT_HC06_RX              GPIOD
#define GPIO_PIN_HC06_RX               GPIO_PIN_9
#define GPIO_CLK_EN_HC06_RX()          __HAL_RCC_GPIOD_CLK_ENABLE()

/*
 * @brief IRQ configuration
 */
#define IRQ_LINE_PIR                   EXTI1_IRQn
#define IRQ_PRIO_PIR                   3

/*
 * @brief SIM900 UART line parameters.
 */
#define UART_BAUDRATE_HC06             9600
#define UART_IFACE_HC06                USART3
#define UART_CLK_EN_HC06()             __HAL_RCC_USART3_CLK_ENABLE()


/* typedef ------------------------------------------------------------------ */
/* variable ----------------------------------------------------------------- */
/*
 * @brief same ADC used for all analog sensors
 */
ADC_HandleTypeDef sensors_adc;

/*
 * @brief PIR object detection status: used by application and PIR ISR.
 */
bool pir_object_detected = false;

/*
 * @brief camera DCMI interface
 */
extern DCMI_HandleTypeDef cam_dcmi_;
static uint8_t frame[IMG_ROWS][IMG_COLUMNS];

/*
 * @brief UART_Hand
 */
UART_HandleTypeDef hc06_;


/* class -------------------------------------------------------------------- */
/* method ------------------------------------------------------------------- */
/* function ----------------------------------------------------------------- */
/*
 * @name   error_critical.
 * @brief  halts MCU on hard error occurrence.
 * @param  file: name of file in which error occurred.
 *         line: number of line in which error occurred.
 * @retval none.
 */
void
error_critical (char* file, int line)
{
  UNUSED (file);
  UNUSED (line);

  while (1)
    {
      ; // halt program
    }

  return;
}

/*
 * @name   sysclk_Config.
 * @brief  system Clock configuration.
 * @param  none.
 * @retval none.
 */
static void
sysclk_Config (void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* initializes the CPU, AHB and APB busses clocks
   * SySclk = (((SRC / M) * N) / P) */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  /* initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  return;
}

/*
 * @name    lcd_init.
 * @brief   configures LCD 16x2 interface.
 * @param   none.
 * @retval  none.
 */
static void
lcd_init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, D0_PIN_Pin | D1_PIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, D3_PIN_Pin | D4_PIN_Pin | D5_PIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, D2_PIN_Pin | D7_PIN_Pin | E_PIN_Pin | RS_PIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, D6_PIN_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = D0_PIN_Pin | D1_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = D3_PIN_Pin | D4_PIN_Pin | D5_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = D2_PIN_Pin | D7_PIN_Pin | E_PIN_Pin | RS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = D6_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  LCD_init ();

  return;
}

/*
 * @name   pir_init.
 * @brief  configures PIR sensor interface and enables detection IRQ.
 * @param  none.
 * @retval none.
 */
static void
pir_init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* enable the GPIO clock */
  GPIO_CLK_EN_PIR();

  /* configure pin as input with external interrupt */
  GPIO_InitStruct.Pin = GPIO_PIN_PIR;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init(GPIO_PORT_PIR, &GPIO_InitStruct);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority (IRQ_LINE_PIR, IRQ_PRIO_PIR, 0);
  HAL_NVIC_EnableIRQ (IRQ_LINE_PIR);

  return;
}

/*
 * @name   buzzer_init.
 * @brief  configures buzzer interface.
 * @param  none.
 * @retval none.
 */
static void
buzzer_init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* enable the GPIO clock */
  GPIO_CLK_EN_BUZZER ();

  /* configure pin as input with external interrupt */
  GPIO_InitStruct.Pin = GPIO_PIN_BUZZER;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIO_PORT_BUZZER, &GPIO_InitStruct);

  return;
}

/*
 * @name   buzzer_set.
 * @brief  changes buzzzer status.
 * @param  status: true: ON, fasle: OFF.
 * @retval none.
 */
static void
buzzer_set (bool status)
{
  int32_t counter = 0;
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//	  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm); //update pwm value
//  HAL_TIM_PWM_Stop (&htim3, TIM_CHANNEL_3);

  if (status == true)
    {
	  /* emulate PWM signal (dc = 0.7) */
	  for (counter = 0; counter < 500; counter++)
        {
          HAL_GPIO_WritePin (GPIO_PORT_BUZZER, GPIO_PIN_BUZZER, GPIO_PIN_SET);
          HAL_Delay (8);
          HAL_GPIO_WritePin (GPIO_PORT_BUZZER, GPIO_PIN_BUZZER, GPIO_PIN_RESET);
          HAL_Delay (2);
        }
    }
  else
    {
      HAL_GPIO_WritePin(GPIO_PORT_BUZZER, GPIO_PIN_BUZZER, GPIO_PIN_RESET);
    }
  return;
}

/*
 * @name   sensors_adc_init.
 * @brief  configure sensors ADC interface.
 * @param  none.
 * @retval none.
 */
static void
sensors_adc_init (void)
{
  ADC_CLK_EN_SENSORS();

  /* configure ADc interface on 10bit accuracy */
  sensors_adc.Instance = ADC_SENSORS;
  sensors_adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  sensors_adc.Init.Resolution = ADC_RESOLUTION_10B;
  sensors_adc.Init.ScanConvMode = DISABLE;
  sensors_adc.Init.ContinuousConvMode = ENABLE;
  sensors_adc.Init.DiscontinuousConvMode = DISABLE;
  sensors_adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  sensors_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  sensors_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  sensors_adc.Init.NbrOfConversion = 1;
  sensors_adc.Init.DMAContinuousRequests = DISABLE;
  sensors_adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&sensors_adc) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  return;
}

/*
 * @name   lm35_init.
 * @brief  configures LM35 GPIO interface.
 * @param  none.
 * @retval none.
 */
static void
lm35_init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_LM35;

  GPIO_CLK_EN_LM35 ();
  HAL_GPIO_Init (GPIO_PORT_LM35, &GPIO_InitStruct);

  return;
}

/*
 * @name   lm35_getval.
 * @brief  reads the temperature in Celcius scale.
 * @param  none.
 * @retval float: positive: temperature in Celcius degrees.
 *                negative: sensor failure.
 */
static float
lm35_getval (void)
{
  float retval = 0.0;
  float voltage;
  ADC_ChannelConfTypeDef sConfig = {0};

  /* configure for the selected ADC regular channel its corresponding rank in the
   * sequencer and its sample time for LM35.
   */
  sConfig.Channel = ADC_CH_LM35;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&sensors_adc, &sConfig) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  if (HAL_ADC_Start (&sensors_adc) != HAL_OK)
    {
      retval = (-1.0);
    }
  else
    {
      HAL_Delay (5);
      if (HAL_ADC_PollForConversion (&sensors_adc, 500) != HAL_OK)
        {
          retval = (-1.0);
        }
      else
        {
          voltage = (((float)HAL_ADC_GetValue (&sensors_adc)) / 1024.0) * 5000.0;
          retval = (voltage / 100) + 20;
        }
    }

  HAL_ADC_Stop (&sensors_adc);

  return retval;
}

/*
 * @name   mq5_init.
 * @brief  configures MQ5 GPIO interface.
 * @param  none.
 * @retval none.
 */
static void
mq5_init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_MQ5;

  GPIO_CLK_EN_MQ5 ();
  HAL_GPIO_Init (GPIO_PORT_MQ5, &GPIO_InitStruct);

  return;
}

/*
 * @name   mq5_getval.
 * @brief  reads the CO2 level.
 * @param  none.
 * @retval float: positive CO2 level on a [0..5000] scale.
 *                negative: sensor failure.
 */
static float
mq5_getval (void)
{
  float retval = 0.0;
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = ADC_CH_MQ5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&sensors_adc, &sConfig) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  if (HAL_ADC_Start (&sensors_adc) != HAL_OK)
    {
      retval = (-1.0);
    }
  else
    {
	  HAL_Delay (5);
	  if (HAL_ADC_PollForConversion (&sensors_adc, 500) != HAL_OK)
        {
          retval = (-1.0);
        }
	  else
	    {
	      retval = ((((float)HAL_ADC_GetValue (&sensors_adc)) / 1024.0) * 5000.0);
	    }
    }

  HAL_ADC_Stop (&sensors_adc);

  return retval;
}

void
cam_init (void)
{
#if 0  // deprecated
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* configure GPIO pins */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AFUNC_CAM_I2C;
  GPIO_InitStruct.Pin = GPIO_PIN_I2C_CAM_SCL | GPIO_PIN_I2C_CAM_SDA;

  GPIO_CLK_EN_CAM_I2C();
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AFUNC_CAM_DCMI;

  GPIO_CLK_EN_CAM_DCMI();
  GPIO_InitStruct.Pin = GPIO_PIN_DCMI_CAM_HSYNC | GPIO_PIN_DCMI_CAM_PIXCK;
  HAL_GPIO_Init(GPIO_PORT_CAM_DCMI_A, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_DCMI_CAM_VSYNC | GPIO_PIN_DCMI_CAM_D5;
  HAL_GPIO_Init(GPIO_PORT_CAM_DCMI_B, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_DCMI_CAM_D0 | GPIO_PIN_DCMI_CAM_D1;
  HAL_GPIO_Init(GPIO_PORT_CAM_DCMI_C, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_DCMI_CAM_D2 | GPIO_PIN_DCMI_CAM_D3
                        | GPIO_PIN_DCMI_CAM_D4 | GPIO_PIN_DCMI_CAM_D6 | GPIO_PIN_DCMI_CAM_D7;
  HAL_GPIO_Init(GPIO_PORT_CAM_DCMI_E, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AFUNC_CAM_MCO;
  GPIO_InitStruct.Pin = GPIO_PIN_CAM_MCO;

  GPIO_CLK_EN_CAM_MCO();
  HAL_GPIO_Init(GPIO_PORT_CAM_CMO, &GPIO_InitStruct);

  /* configure I2C interface */
  cam_i2c_.Instance = I2C_CAM;
  cam_i2c_.Init.ClockSpeed = 100000;
  cam_i2c_.Init.DutyCycle = I2C_DUTYCYCLE_2;
  cam_i2c_.Init.OwnAddress1 = 0;
  cam_i2c_.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  cam_i2c_.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  cam_i2c_.Init.OwnAddress2 = 0;
  cam_i2c_.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  cam_i2c_.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  I2C_CLK_EN_CAM();
  if (HAL_I2C_Init(&cam_i2c_) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  /* configure DCMI interface */
  cam_dcmi_.Instance = DCMI_CAM;
  cam_dcmi_.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  cam_dcmi_.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  cam_dcmi_.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  cam_dcmi_.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  cam_dcmi_.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;//DCMI_PCKPOLARITY_FALLING;
  cam_dcmi_.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  cam_dcmi_.Init.JPEGMode = DCMI_JPEG_DISABLE;

  DCMI_CLK_EN_CAM();
  if (HAL_DCMI_Init(&cam_dcmi_) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  /* configure MCO */
  HAL_RCC_EnableCSS ();
  HAL_RCC_MCOConfig (RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);

  /* wait for camera to set-up */
  HAL_Delay (1000);
#endif // deprecated

  return;
}

static void
dumpFrame (void)
{
  uint8_t *buffer = (uint8_t *)frame;
  unsigned char picH = 0, picL = 0;
  int length = IMG_ROWS * IMG_COLUMNS / 2;
  int i;//,j;

  /* Copy every other byte from the main frame buffer to our temporary buffer (this converts the image to grey scale) */
  LCD_SetWindows(0 , 0, IMG_ROWS, IMG_COLUMNS);
  for(i=0; i<length; i++)
    {
	  picL = *(buffer+(i*2));
      picH = *(buffer+(i*2)+1);
      Lcd_WriteData_16Bit(picH<<8|picL);
    }

//  for (i=0; i<IMG_COLUMNS; i++)
//    {
//      for (j=0; j<IMG_ROWS; j++)
//        {
//          GUI_DrawPoint (i, j, frame[i][j]);
//        }
//    }

//  for (i = 1; i < length; i += 2)
//    {
//      temp_buffer[i / 2] = buffer[i];
//    }

  /* We only send the sync frame if it has been requested */
//	if (send_sync_frame) {
//		for (i = 0x7f; i > 0; i--) {
//			uint8_t val = i;
//			Serial_sendb(&val);
//		}
//		send_sync_frame = false;
//	}

//  for (i = 0; i < length; i++)
//    {
//      if (i > 100)
//        {
//          GUI_DrawPoint (i%IMG_COLUMNS, i/IMG_ROWS, buffer[i]);
//
////          Serial_sendb(&temp_buffer[i]);
//        }
//      else
//        {
////          uint8_t val = 0xff;
////          Serial_sendb(&val); // Change first 100 pixels to white to provide a reference for where the frame starts
//        }
//    }

  /* Enable capture and DMA after we have sent the photo. This is a workaround for the timing issues I've been having where
   * the DMA transfer is not in sync with the frames being sent */
//  DMA_Cmd(DMA2_Stream1, ENABLE);
//  DCMI_Cmd(ENABLE);
//  DCMI_CaptureCmd(ENABLE);

  return;
}

void
cam_show (void)
{
  HAL_StatusTypeDef dcmi_status;

  dcmi_status = HAL_DCMI_Start_DMA (&cam_dcmi_, DCMI_MODE_SNAPSHOT, (uint32_t)frame, (IMG_ROWS * IMG_COLUMNS) / 4);
  while (HAL_DCMI_GetState (&cam_dcmi_) == HAL_DCMI_STATE_BUSY)
    {
      ; // wait until frame is received
    }
  HAL_DCMI_Stop (&cam_dcmi_);

  if (dcmi_status == HAL_OK)
    {
//      LCD_ShowString (2, 2, 16, (uint8_t*)"got a frame!", 0);
      dumpFrame ();
      HAL_Delay (100);
//      Gui_Drawbmp16 (30, 30, frame);
    }
  else
    {
      LCD_ShowString (2, 2, 16, (uint8_t*)"got a nothing!", 0);
      HAL_Delay (1000);
    }

  return;
}

/*
 * @name   hc06_init.
 * @brief  configures GPIO and UART ports of HC06 interface.
 * @param  none.
 * @retval none.
 */
static void
hc06_init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  HAL_StatusTypeDef ret_val = HAL_OK;

  /* setup GPIO pins */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  GPIO_InitStruct.Pin = GPIO_PIN_HC06_TX | GPIO_PIN_HC06_RX;

  GPIO_CLK_EN_HC06_TX ();
  HAL_GPIO_Init (GPIO_PORT_HC06_TX, &GPIO_InitStruct);

  /* set-up hardware */
  hc06_.Instance =  UART_IFACE_HC06;
//  HAL_UART_DeInit (&hc06_);
  hc06_.Init.BaudRate = UART_BAUDRATE_HC06;
  hc06_.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hc06_.Init.Mode = UART_MODE_TX_RX;
  hc06_.Init.Parity = UART_PARITY_NONE;
  hc06_.Init.StopBits = UART_STOPBITS_1;
  hc06_.Init.WordLength = UART_WORDLENGTH_8B;
  hc06_.Init.OverSampling = UART_OVERSAMPLING_16;
//  hc06_.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  hc06_.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  UART_CLK_EN_HC06 ();
  ret_val = HAL_UART_Init (&hc06_);
  if (ret_val != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  return;
}

/*
 * @name   hc06_send.
 * @brief  sends a data via bluetooth module.
 * @param  data: string to send.
 * @retval int32_t: zero on success and negative value on failure.
 */
static int32_t
hc06_send (const char* data)
{
  int32_t retval = 0;
  int32_t rx_status = HAL_OK;

  rx_status = HAL_UART_Transmit (&hc06_, (uint8_t*)data, strlen(data), 0xFFFFF);
  if (rx_status != HAL_OK)
    {
      retval = (-1);
    }
  else
    {
	  rx_status = HAL_UART_Transmit (&hc06_, (uint8_t*)"\r\n", 2, 0xFFFFF);
	  if (rx_status != HAL_OK)
	    {
	      retval = (-1);
	    }
	  else
	    {
	      retval = 0;
	    }
    }

  return retval;
}

/*
 * @name   main.
 * @brief  polls for sensory data and displays camshot on big LCD on object detection.
 * @param  none.
 * @retval none.
 */
int
main (void)
{
  float temp = 0.0, gaz = 0.0;
//  char msg[16];              // LCD 16x2 message
//  bool cam_err = true;
  char hc06_data[32];

  /* configure clocks */
  sysclk_Config ();

  /* configure interfaces */
//  pir_init ();
//  sensors_adc_init ();
//  mq5_init ();
//  lm35_init ();
//  buzzer_init ();
//  lcd_init ();
//  LCD_print ("Hello World!");
//  LCD_Init ();
//  LCD_direction (1);
//  LCD_Clear(WHITE);
  hc06_init ();
//  ov7670_hwinit ();
//  while (cam_err == true)
//    {
//      cam_err = OV7670_init ();
//    }

  HAL_Delay (2000);

  while (1)
    {
      /* check PIR sensor: boolean */
#if 0
      if (pir_object_detected == true)
        {
          /* take camshot and display on big LCD */
          snprintf ((char*)msg, sizeof(msg), "object detected");
          LCD_clear ();
          LCD_setCursor(2, 1);
          LCD_print ((char*)msg);
          buzzer_set (true);
          cam_show ();

          /* prevent IRQ cadence */
          pir_object_detected = false;
          buzzer_set (false);
        }
      else
        {
          snprintf ((char*)msg, sizeof(msg), "No detection!");
          LCD_clear ();
          LCD_setCursor(2, 1);
          LCD_print ((char*)msg);

          HAL_Delay (5000);
        }
#endif // check PIR sensor: boolean

      /* check MQ5 sensor: threshold */
#if 0
      gaz = mq5_getval ();
      snprintf ((char*)msg, sizeof(msg), "gaz: %.2f ppm", gaz);
      LCD_clear ();
      LCD_setCursor(2, 1);
      LCD_print ((char*)msg);
      HAL_Delay (5000);
      // reaction TODO
#endif // check MQ5 sensor: threshold

      /* check LM35 sensor: threshold */
#if 0
      temp = lm35_getval ();
      snprintf ((char*)msg, sizeof(msg), "temp: %.2f C", temp);
      LCD_clear ();
      LCD_setCursor(2, 1);
      LCD_print ((char*)msg);
      // reaction TODO
#endif // check LM35 sensor: threshold

      /* test camera */
#if 0
      cam_show ();
#endif // test camera

      /* send data via bluetooth */
#if 1
      snprintf ((char*)hc06_data, 32, "temp=%.2f gaz=%.2f", temp, gaz);
      hc06_send (hc06_data);
#endif // send data via bluetooth

      HAL_Delay (5 * 1000);
    }

  return (-1);   // should not get here
}
