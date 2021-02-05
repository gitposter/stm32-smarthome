/*
		Library: 			Liquid Crystal Display (LCD) 16X2
		Written by:  	Mohamed Yaqoob
		Date:				 	06/04/2016
		Description:	This is a library for the standard 16X2 LCD display, for the STM32F4xx series.
									It perfroms the basic Text printing to your 16X2 LCD and is capable of operating
									in 8 bits and 4 bits modes of operation.
		References**:
									This was written based on the open source Arduino LiquidCrystal library
									and by referring to the DATASHEET of the LCD16X2, also with the help of
									the following YouTube tutorials on LCD 16X2:
									(1): 'RC Tractor Guy' YouTube tutorial on the following link:
									     https://www.youtube.com/watch?v=efi2nlsvbCI
									(2): 'Explore Embedded' YouTube tutorial on the following link:
											 https://www.youtube.com/watch?v=YDJISiPUdA8
*/

//(1): Header files includes
#include "stm32f4xx_hal.h"
#include "STM_MY_LCD16X2.h"
#include  <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
//(2): Variables definition
//---- Private Variables ----//
static bool Mode_8BIT;														// 1: 8 bits, 0: 4bits
static uint8_t DisplayControl = 0x0F;
static uint8_t FunctionSet = 0x38;
//---- Public Variables ----//

//(3): Functions definitions
//---- Private Functions ----//
// Funcion(2): RS and E pins

static void RS_pin(bool state)
{
	if(state)
	{
		HAL_GPIO_WritePin(RS_PIN_GPIO_Port,RS_PIN_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(RS_PIN_GPIO_Port,RS_PIN_Pin, GPIO_PIN_RESET);
	}
}
static void E_pin(bool state)
{
	if(state)
	{
		HAL_GPIO_WritePin(E_PIN_GPIO_Port,E_PIN_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(E_PIN_GPIO_Port,E_PIN_Pin, GPIO_PIN_RESET);
	}
}
// Function(1): Write command
static void writeCommand(uint8_t command)
{
	uint8_t temp;
                temp  = command & 0x01;
                HAL_GPIO_WritePin(D0_PIN_GPIO_Port, D0_PIN_Pin,temp);//D0
                temp  = command & 0x02;
                HAL_GPIO_WritePin(D1_PIN_GPIO_Port, D1_PIN_Pin, temp);//D1
                temp  = command & 0x04;
                HAL_GPIO_WritePin(D2_PIN_GPIO_Port, D2_PIN_Pin, temp);//D2
                temp  = command & 0x08;
                HAL_GPIO_WritePin(D3_PIN_GPIO_Port, D3_PIN_Pin, temp);//D3
                temp  = command & 0x10;
                HAL_GPIO_WritePin(D4_PIN_GPIO_Port, D4_PIN_Pin, temp);//D4
                temp  = command & 0x20;
                HAL_GPIO_WritePin(D5_PIN_GPIO_Port, D5_PIN_Pin, temp);//D5
                temp  = command & 0x40;
                HAL_GPIO_WritePin(D6_PIN_GPIO_Port, D6_PIN_Pin, temp);//D6
                temp  = command & 0x80;
                HAL_GPIO_WritePin(D7_PIN_GPIO_Port, D7_PIN_Pin, temp);//D7
                E_pin(false);
                HAL_Delay(1);
		E_pin(true);
                HAL_Delay(1);
		E_pin(false);
                HAL_Delay(1);
	
	
}

// Funciton(3): Change mode
static void switchMode(bool Mode)
{
	RS_pin(false);
	HAL_Delay(20);
	writeCommand(0x30);
	HAL_Delay(5);
	writeCommand(0x30);
	HAL_Delay(5);
	writeCommand(0x30);
	HAL_Delay(1);
	if(Mode)
	{
		writeCommand(0x30);
	}
	else
	{
		writeCommand(0x20);
	}
	HAL_Delay(1);
}
// Function(4): Write a single Character to the display
static void LCD_writeChar(char text)
{
	RS_pin(true);
	writeCommand(text);
}
static void setDataLength(bool length)
{
	RS_pin(false);
	if(length)
	{
		FunctionSet |= (0x10);
		writeCommand(FunctionSet);
	}
	else
	{
		FunctionSet &= ~(0x10);
		writeCommand(FunctionSet);
	}
}
// Function(5): itoa and ftoa assist functions
static void reverse(char s[])
{
     int i, j;
     char c;

     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
}
static int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverseF(str, i);
    str[i] = '\0';
    return i;
}

static void reverseF(char *str, int len)
{
	  int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
// Function(3): Print to Display
void LCD_print(char string[])
{
	for(uint8_t i=0;  i< 16 && string[i]!=NULL; i++)
	{
		LCD_writeChar(string[i]);
	}
}
// Function(4): Clear display
void LCD_clear(void)
{
	RS_pin(false);
	writeCommand(0x01);
	HAL_Delay(2);
}
// Function (5): Set Cursor position
void LCD_setCursor(uint8_t row, uint8_t col)
{
	uint8_t maskData;
	RS_pin(false);
	maskData = (col-1)&0x0F;
	if(row==1)
	{
		maskData |= (0x80);
		writeCommand(maskData);
	}
	else//if(row==2)
        
	{
		maskData |= (0xc0);
		writeCommand(maskData);
	}
/*
        if(row==3)
        {
          
	
		maskData |= (0x94);
		writeCommand(maskData);
	
        }
if(row==4)
        {
          
	
		maskData |= (0xd4);
		writeCommand(maskData);
	
        }
*/
}
// Function(6): Enable two lines
void LCD_TwoLines(void)
{
	RS_pin(false);
	FunctionSet |= (0x08);
	writeCommand(FunctionSet);
}
void LCD_OneLine(void)
{
	RS_pin(false);
	FunctionSet &= ~(0x08);
	writeCommand(FunctionSet);
}
// Function(7): Blinking cursor
void LCD_noBlink(void)
{
	RS_pin(false);
	DisplayControl &= ~(0x01);
	writeCommand(DisplayControl);
}
void LCD_blink(void)
{
	RS_pin(false);
	DisplayControl |= 0x01;
	writeCommand(DisplayControl);
}
// Function(8): Display ON/OFF
void LCD_noDisplay(void)
{
	RS_pin(false);
	DisplayControl &= ~(0x04);
	writeCommand(DisplayControl);
}
void LCD_display(void)
{
	RS_pin(false);
	DisplayControl |= (0x04);
	writeCommand(DisplayControl);
}
// Function(9): Cursor ON/OFF
void LCD_noCursor(void)
{
	RS_pin(false);
	DisplayControl &= ~(0x03);
	writeCommand(DisplayControl);
}
void LCD_cursor(void)
{
	RS_pin(false);
	DisplayControl |= (0x03);
	writeCommand(DisplayControl);
}
// Function(10): Shift Display or Cursor, right or left
void LCD_shiftToRight(uint8_t num)
{
	RS_pin(false);
	for(uint8_t i=0; i<num;i++)
	{
		writeCommand(0x1c);
	}
}
void LCD_shiftToLeft(uint8_t num)
{
	RS_pin(false);
	for(uint8_t i=0; i<num;i++)
	{
		writeCommand(0x18);
	}
}
void LCD_init()
{
	Mode_8BIT = true; 
        
       // LCD_shiftToRight();
        LCD_TwoLines();// 8 bits mode is enabled
        LCD_noCursor();
	switchMode(true);
	setDataLength(true);
        LCD_noBlink();
	writeCommand(0x0F);
	HAL_Delay(2);
	LCD_clear();
        HAL_Delay(50);
        LCD_display();
}
// Function(11): itoa and ftoa to convert from int and float to string
void LCD_itoa(int n, char s[])
{
     int i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}
void LCD_ftoa(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 1);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
