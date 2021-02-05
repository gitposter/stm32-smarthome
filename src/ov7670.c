/*
 ******************************************************************************
 * @file    ov7670.c
 * @module  app
 * @author  Mohamed Hnezli
 * @version V0.2
 * @date    17-09-2019
 * @brief   implements OV7670 camera driver.
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
#include "ov7670.h"                    // header file


/* define ------------------------------------------------------------------- */
/* typedef ------------------------------------------------------------------ */
/* variable ----------------------------------------------------------------- */
/*
 * @brief camera I2C and DCMI interfaces
 */
I2C_HandleTypeDef cam_i2c_;
DCMI_HandleTypeDef cam_dcmi_;
DMA_HandleTypeDef dma_;

/*
 * @brief image buffer
 */
//volatile uint16_t frame_buffer[IMG_ROWS * IMG_COLUMNS];

/*
 * @brief camera registers and values
 */
#define REG_BATT 0xFF

#if 1
const uint8_t OV7670_reg[][2] = {
  { 0x12, 0x80 },  // reset
  /* Color mode related */
  {0x12, 0x14},   // QVGA, RGB
  {0x8C, 0x00},   // RGB444 Disable
  {0x40, 0x10 + 0xc0},   // RGB565, 00 - FF
  {0x3A, 0x04 + 8},   // UYVY (why?)
  {0x3D, 0x80 + 0x00},   // gamma enable, UV auto adjust, UYVY
  {0xB0, 0x84}, // important

  /* clock related */
  {0x0C, 0x04},  // DCW enable
  {0x3E, 0x19},  // manual scaling, pclk/=2
  {0x70, 0x3A},  // scaling_xsc
  {0x71, 0x35},  // scaling_ysc
  {0x72, 0x11}, // down sample by 2
  {0x73, 0xf1}, // DSP clock /= 2

  /* windowing (empirically decided...) */
  {0x17, 0x16},   // HSTART
  {0x18, 0x04},   // HSTOP
  {0x32, 0x80},   // HREF
  {0x19, 0x03},   // VSTART =  14 ( = 3 * 4 + 2)
  {0x1a, 0x7b},   // VSTOP  = 494 ( = 123 * 4 + 2)
  {0x03, 0x0a},   // VREF (VSTART_LOW = 2, VSTOP_LOW = 2)

  /* color matrix coefficient */
#if 0
  {0x4f, 0xb3},
  {0x50, 0xb3},
  {0x51, 0x00},
  {0x52, 0x3d},
  {0x53, 0xa7},
  {0x54, 0xe4},
  {0x58, 0x9e},
#else
  {0x4f, 0x80},
  {0x50, 0x80},
  {0x51, 0x00},
  {0x52, 0x22},
  {0x53, 0x5e},
  {0x54, 0x80},
  {0x58, 0x9e},
#endif

  /* 3a */
//  {0x13, 0x84},
//  {0x14, 0x0a},   // AGC Ceiling = 2x
//  {0x5F, 0x2f},   // AWB B Gain Range (empirically decided)
//                  // without this bright scene becomes yellow (purple). might be because of color matrix
//  {0x60, 0x98},   // AWB R Gain Range (empirically decided)
//  {0x61, 0x70},   // AWB G Gain Range (empirically decided)
  {0x41, 0x38},   // edge enhancement, de-noise, AWG gain enabled


  /* gamma curve */
#if 1
  {0x7b, 16},
  {0x7c, 30},
  {0x7d, 53},
  {0x7e, 90},
  {0x7f, 105},
  {0x80, 118},
  {0x81, 130},
  {0x82, 140},
  {0x83, 150},
  {0x84, 160},
  {0x85, 180},
  {0x86, 195},
  {0x87, 215},
  {0x88, 230},
  {0x89, 244},
  {0x7a, 16},
#else
  /* gamma = 1 */
  {0x7b, 4},
  {0x7c, 8},
  {0x7d, 16},
  {0x7e, 32},
  {0x7f, 40},
  {0x80, 48},
  {0x81, 56},
  {0x82, 64},
  {0x83, 72},
  {0x84, 80},
  {0x85, 96},
  {0x86, 112},
  {0x87, 144},
  {0x88, 176},
  {0x89, 208},
  {0x7a, 64},
#endif

  /* fps */
//  {0x6B, 0x4a}, //PLL  x4
  {0x11, 0x00}, // pre-scalar = 1/1

  /* others */
  {0x1E, 0x31}, //mirror flip
//  {0x42, 0x08}, // color bar

  {REG_BATT, REG_BATT},
};

#else
const uint8_t OV7670_reg[][2] = {
  { 0x12, 0x80 },  // reset
// Image format
		{ 0x12, 0x8 },		// 0x14 = QVGA size, RGB mode; 0x8 = QCIF, YUV, 0xc = QCIF (RGB)
		{ 0xc, 0x8 }, //
		{ 0x11, 0b1000000 }, //

		{ 0xb0, 0x84 },		//Color mode (Not documented??)

		// Hardware window
		{ 0x11, 0x01 },		//PCLK settings, 15fps
		{ 0x32, 0x80 },		//HREF
		{ 0x17, 0x17 },		//HSTART
		{ 0x18, 0x05 },		//HSTOP
		{ 0x03, 0x0a },		//VREF
		{ 0x19, 0x02 },		//VSTART
		{ 0x1a, 0x7a },		//VSTOP

		// Scalling numbers
		{ 0x70, 0x3a },		//X_SCALING
		{ 0x71, 0x35 },		//Y_SCALING
		{ 0x72, 0x11 },		//DCW_SCALING
		{ 0x73, 0xf0 },		//PCLK_DIV_SCALING
		{ 0xa2, 0x02 },		//PCLK_DELAY_SCALING

		// Matrix coefficients
		{ 0x4f, 0x80 }, //
		{ 0x50, 0x80 }, //
		{ 0x51, 0x00 }, //
		{ 0x52, 0x22 }, //
		{ 0x53, 0x5e }, //
		{ 0x54, 0x80 }, //
		{ 0x58, 0x9e },

		// Gamma curve values
		{ 0x7a, 0x20 }, //
		{ 0x7b, 0x10 }, //
		{ 0x7c, 0x1e }, //
		{ 0x7d, 0x35 }, //
		{ 0x7e, 0x5a }, //
		{ 0x7f, 0x69 }, //
		{ 0x80, 0x76 }, //
		{ 0x81, 0x80 }, //
		{ 0x82, 0x88 }, //
		{ 0x83, 0x8f }, //
		{ 0x84, 0x96 }, //
		{ 0x85, 0xa3 }, //
		{ 0x86, 0xaf }, //
		{ 0x87, 0xc4 }, //
		{ 0x88, 0xd7 }, //
		{ 0x89, 0xe8 },

		// AGC and AEC parameters
		{ 0xa5, 0x05 }, //
		{ 0xab, 0x07 }, //
		{ 0x24, 0x95 }, //
		{ 0x25, 0x33 }, //
		{ 0x26, 0xe3 }, //
		{ 0x9f, 0x78 }, //
		{ 0xa0, 0x68 }, //
		{ 0xa1, 0x03 }, //
		{ 0xa6, 0xd8 }, //
		{ 0xa7, 0xd8 }, //
		{ 0xa8, 0xf0 }, //
		{ 0xa9, 0x90 }, //
		{ 0xaa, 0x94 }, //
		{ 0x10, 0x00 },

		// AWB parameters
		{ 0x43, 0x0a }, //
		{ 0x44, 0xf0 }, //
		{ 0x45, 0x34 }, //
		{ 0x46, 0x58 }, //
		{ 0x47, 0x28 }, //
		{ 0x48, 0x3a }, //
		{ 0x59, 0x88 }, //
		{ 0x5a, 0x88 }, //
		{ 0x5b, 0x44 }, //
		{ 0x5c, 0x67 }, //
		{ 0x5d, 0x49 }, //
		{ 0x5e, 0x0e }, //
		{ 0x6c, 0x0a }, //
		{ 0x6d, 0x55 }, //
		{ 0x6e, 0x11 }, //
		{ 0x6f, 0x9f }, //
		{ 0x6a, 0x40 }, //
		{ 0x01, 0x40 }, //
		{ 0x02, 0x60 }, //
		{ 0x13, 0xe7 },

		// Additional parameters
		{ 0x34, 0x11 }, //
		{ 0x3f, 0x00 }, //
		{ 0x75, 0x05 }, //
		{ 0x76, 0xe1 }, //
		{ 0x4c, 0x00 }, //
		{ 0x77, 0x01 }, //
		{ 0xb8, 0x0a }, //
		{ 0x41, 0x18 }, //
		{ 0x3b, 0x12 }, //
		{ 0xa4, 0x88 }, //
		{ 0x96, 0x00 }, //
		{ 0x97, 0x30 }, //
		{ 0x98, 0x20 }, //
		{ 0x99, 0x30 }, //
		{ 0x9a, 0x84 }, //
		{ 0x9b, 0x29 }, //
		{ 0x9c, 0x03 }, //
		{ 0x9d, 0x4c }, //
		{ 0x9e, 0x3f }, //
		{ 0x78, 0x04 }, //
		{ 0x0e, 0x61 }, //
		{ 0x0f, 0x4b }, //
		{ 0x16, 0x02 }, //
		{ 0x1e, 0x00 }, //
		{ 0x21, 0x02 }, //
		{ 0x22, 0x91 }, //
		{ 0x29, 0x07 }, //
		{ 0x33, 0x0b }, //
		{ 0x35, 0x0b }, //
		{ 0x37, 0x1d }, //
		{ 0x38, 0x71 }, //
		{ 0x39, 0x2a }, //
		{ 0x3c, 0x78 }, //
		{ 0x4d, 0x40 }, //
		{ 0x4e, 0x20 }, //
		{ 0x69, 0x00 }, //
		{ 0x6b, 0x3a }, //
		{ 0x74, 0x10 }, //
		{ 0x8d, 0x4f }, //
		{ 0x8e, 0x00 }, //
		{ 0x8f, 0x00 }, //
		{ 0x90, 0x00 }, //
		{ 0x91, 0x00 }, //
		{ 0x96, 0x00 }, //
		{ 0x9a, 0x00 }, //
		{ 0xb1, 0x0c }, //
		{ 0xb2, 0x0e }, //
		{ 0xb3, 0x82 }, //
		{ 0x4b, 0x01 },
        {REG_BATT, REG_BATT},
};
#endif

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
extern void
error_critical (char* file, int line);

/*
 * @name   error_critical.
 * @brief  halts MCU on hard error occurrence.
 * @param  file: name of file in which error occurred.
 *         line: number of line in which error occurred.
 * @retval none.
 */
void
Delay (volatile long nCount)
{
  while (nCount--)
    {
      ;  // do nothing
    }

  return;
}

static void
MCO1_init (void)
{
#if 0 // SPL
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_ClockSecuritySystemCmd(ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// GPIO config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		//PA8 - XCLK
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// GPIO AF config
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

	// MCO clock source
//	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4); // Using the fast PLL clock results in garbage output, using HSI (at 16Mhz works fine)
	RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1);
#elif 1 // HAL
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* configure GPIO */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AFUNC_CAM_MCO;
  GPIO_InitStruct.Pin = GPIO_PIN_CAM_MCO;

  GPIO_CLK_EN_CAM_MCO();
  HAL_GPIO_Init(GPIO_PORT_CAM_MCO, &GPIO_InitStruct);

  /* configure MCO */
  HAL_RCC_EnableCSS ();
  HAL_RCC_MCOConfig (RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_4);

  /* wait for camera to set-up */
  HAL_Delay (1000);
#endif // lib
}

static void
SCCB_init (void)
{
#if 0 // SPL
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// GPIO config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// GPIO AF config
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_I2C2);

	// I2C config
	I2C_DeInit(I2C2);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);
	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_Cmd(I2C2, ENABLE);

#elif 1 // HAL
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* configure GPIO pins */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AFUNC_CAM_I2C;
  GPIO_InitStruct.Pin = GPIO_PIN_I2C_CAM_SCL | GPIO_PIN_I2C_CAM_SDA;

  GPIO_CLK_EN_CAM_I2C();
  HAL_GPIO_Init(GPIO_PORT_CAM_I2C, &GPIO_InitStruct);

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

#endif // lib

  return;
}

static void
DCMI_DMA_init (void)
{
#if 0 // SPL
	GPIO_InitTypeDef GPIO_InitStructure;
	DCMI_InitTypeDef DCMI_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	// GPIO config

	// PA4 - HREF (HSYNC), PA6 - PCLK (PIXCLK)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;	//PA4 - HREF (HSYNC)
															//PA6 - PCLK (PIXCLK)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PB6 - D5, PB7 - VSYNC, PB8 - D6, PB9 - D7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PC6 - D0, PC7 - D1, PC8 - D2, PC9 - D3, PC11 - D4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// GPIO AF config
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_DCMI);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_DCMI);

	// DCMI config
	DCMI_DeInit();
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot; //DCMI_CaptureMode_SnapShot
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame; //DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_Init(&DCMI_InitStructure);
	DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
	DCMI_ITConfig(DCMI_IT_OVF, ENABLE);
	DCMI_ITConfig(DCMI_IT_ERR, ENABLE);

	// DMA config
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&DCMI->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) frame_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = IMG_ROWS * IMG_COLUMNS / 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TE, ENABLE);

	/* DMA2 IRQ channel Configuration */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);

#elif 1 // HAL
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* configure GPIO pins */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AFUNC_CAM_DCMI;

  GPIO_CLK_EN_CAM_DCMI();
  GPIO_InitStruct.Pin = GPIO_PIN_DCMI_CAM_HSYNC | GPIO_PIN_DCMI_CAM_PIXCK | GPIO_PIN_DCMI_CAM_D1;
  HAL_GPIO_Init(GPIO_PORT_CAM_DCMI_A, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_DCMI_CAM_VSYNC | GPIO_PIN_DCMI_CAM_D5;
  HAL_GPIO_Init(GPIO_PORT_CAM_DCMI_B, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_DCMI_CAM_D0;// | GPIO_PIN_DCMI_CAM_D1;
  HAL_GPIO_Init(GPIO_PORT_CAM_DCMI_C, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_DCMI_CAM_D2 | GPIO_PIN_DCMI_CAM_D3
                        | GPIO_PIN_DCMI_CAM_D4 | GPIO_PIN_DCMI_CAM_D6 | GPIO_PIN_DCMI_CAM_D7;
  HAL_GPIO_Init(GPIO_PORT_CAM_DCMI_E, &GPIO_InitStruct);

  /* configure DCMI interface */
  cam_dcmi_.Instance = DCMI_CAM;
  cam_dcmi_.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  cam_dcmi_.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  cam_dcmi_.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  cam_dcmi_.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  cam_dcmi_.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  cam_dcmi_.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  cam_dcmi_.Init.JPEGMode = DCMI_JPEG_DISABLE;
  cam_dcmi_.DMA_Handle = &dma_;
  DCMI_CLK_EN_CAM();
  if (HAL_DCMI_Init(&cam_dcmi_) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  /* configure DMA */
  dma_.Instance = DMA_CAM;
#if 0
  dma_.Init.Channel = DMA_CHANNEL_1;
  dma_.Init.Direction = DMA_PERIPH_TO_MEMORY;
  dma_.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//  dma_.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  dma_.Init.MemBurst = DMA_MBURST_SINGLE;
  dma_.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  dma_.Init.MemInc = DMA_MINC_DISABLE;
  dma_.Init.Mode = DMA_NORMAL;
//  dma_.Init.PeriphBurst = DMA_PBURST_SINGLE;
  dma_.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dma_.Init.PeriphInc = DMA_PINC_DISABLE;
  dma_.Init.Priority = DMA_PRIORITY_HIGH;
#else
  dma_.Init.Channel = DMA_CHANNEL_1;
  dma_.Init.Direction = DMA_PERIPH_TO_MEMORY;
  dma_.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  dma_.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  dma_.Init.MemBurst = DMA_MBURST_SINGLE;
  dma_.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  dma_.Init.MemInc = DMA_MINC_ENABLE;
  dma_.Init.Mode = DMA_NORMAL;
  dma_.Init.PeriphBurst = DMA_PBURST_SINGLE;
  dma_.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dma_.Init.PeriphInc = DMA_PINC_DISABLE;
  dma_.Init.Priority = DMA_PRIORITY_HIGH;
#endif
  DMA_CLK_EN_CAM();
  HAL_DMA_Init (&dma_);

  __HAL_LINKDMA(&cam_dcmi_, DMA_Handle, dma_);

  HAL_NVIC_EnableIRQ (IRQ_LINE_DMA_CAM);
  HAL_NVIC_SetPriority (IRQ_LINE_DMA_CAM, IRQ_PRIO_DMA_CAM, 0);

#endif // lib

  return;
}

bool SCCB_write_reg(uint8_t reg_addr, uint8_t* data)
{
  uint32_t timeout = 0x7FFFFF;
  uint8_t i2c_frame[2];
  HAL_StatusTypeDef i2c_status = HAL_OK;

  i2c_frame[0] = reg_addr;
  i2c_frame[1] = (*data);

//  while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
  while (HAL_I2C_GetState(&cam_i2c_) != HAL_I2C_STATE_READY)
    {
      if ((timeout--) == 0)
        {
//          Serial_log("Busy Timeout\r\n");
          return true;
        }
    }

  // Send start bit
//	I2C_GenerateSTART(I2C2, ENABLE);
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
//    {
//      if ((timeout--) == 0)
//        {
//          Serial_log("Start bit Timeout\r\n");
//          return true;
//        }
//    }

  // Send slave address (camera write address)
//  I2C_Send7bitAddress(I2C2, OV7670_WRITE_ADDR, I2C_Direction_Transmitter);
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//    {
//      if ((timeout--) == 0) {
//      Serial_log("Slave address timeout\r\n");
//      return true;
//    }
//  }

  // Send register address
//  I2C_SendData(I2C2, reg_addr);
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
//  if ((timeout--) == 0) {
//    Serial_log("Register timeout\r\n");
//    return true;
//    }
//  }

  // Send new register value
//	I2C_SendData(I2C2, *data);
//	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
//		if ((timeout--) == 0) {
//			Serial_log("Value timeout\r\n");
//			return true;
//		}
//	}

  /* send device address and data */
  i2c_status = HAL_I2C_Master_Transmit (&cam_i2c_, OV7670_WRITE_ADDR, (uint8_t*)i2c_frame, 2, 0xFFF);
//  i2c_status = HAL_I2C_Master_Transmit (&cam_i2c_, OV7670_WRITE_ADDR, &reg_addr, 1, 0xFFF);
//  i2c_status |= HAL_I2C_Master_Transmit (&cam_i2c_, OV7670_WRITE_ADDR, data, 1, 0xFFF);
//  i2c_status = HAL_I2C_Mem_Write(&cam_i2c_, OV7670_WRITE_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
  if (i2c_status != HAL_OK)
    {
      return true;
    }

  // Send stop bit
//	I2C_GenerateSTOP(I2C2, ENABLE);

	return false;
}

void
ov7670_hwinit (void)
{
  MCO1_init ();
  SCCB_init ();
  DCMI_DMA_init ();

  return;
}

bool
OV7670_init(void)
{
  uint8_t i = 0;
  uint32_t i2ec_err = 0;
  bool err;

  err = SCCB_write_reg (OV7670_reg[0][0], &(OV7670_reg[0][1]));
  HAL_Delay (50);

  /* configure camera registers */
  for (i = 1; OV7670_reg[i][0] != REG_BATT; i++)
    {
      err = SCCB_write_reg (OV7670_reg[i][0], &(OV7670_reg[i][1]));
//      Serial_log("Writing register: ");
//      Serial_logi(i);
//      Serial_log("\r\n");

      if (err == true)
        {
//          Serial_log("Failed to update register\r\n");
    	  i2ec_err = HAL_I2C_GetError (&cam_i2c_);
          HAL_Delay (0xFF);
          break;
        }

//      HAL_Delay(0xFF);
    }

  return err;
}

