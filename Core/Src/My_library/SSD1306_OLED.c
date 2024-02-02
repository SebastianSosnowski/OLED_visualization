/*
 * SSD1306_OLED.c
 *
 *  Created on: Feb 2, 2024
 *      Author: akun1
 */

#include "main.h"
#include "My_library/SSD1306_OLED.h"

I2C_HandleTypeDef  *oled_i2c;


/** SSD1306_Command
  * @brief  Issue single command to SSD1306
  * @param  command The command character to send to the display.
  * @retval None
  */
void SSD1306_Command(uint8_t command)
{
	HAL_I2C_Mem_Write(oled_i2c, SSD1303_ADDRESS << 1, 0x00, 1, &command, 1, SSD1306_TIMEOUT);
}

/** SSD1306_Data
  * @brief  Issue single data to SSD1306
  * @param  data The data character to send to the display.
  * @retval None
  */
void SSD1306_Data(uint8_t data)
{
	HAL_I2C_Mem_Write(oled_i2c, SSD1303_ADDRESS << 1, 0x040, 1, &data, 1, SSD1306_TIMEOUT);
}

void SSD1306_Init(I2C_HandleTypeDef *i2c)
{
	oled_i2c = i2c;

	SSD1306_Command(SSD1306_DISPLAYOFF);
	SSD1306_Command(SSD1306_SETDISPLAYCLOCKDIV);
	SSD1306_Command(0x80);
	SSD1306_Command(SSD1306_SETMULTIPLEX);

	SSD1306_Command(SSD1306_LCDHEIGHT - 1);

	SSD1306_Command(SSD1306_SETDISPLAYOFFSET);
	SSD1306_Command(0x00);
	SSD1306_Command(SSD1306_SETSTARTLINE | 0x00);

	SSD1306_Command(SSD1306_CHARGEPUMP);
	SSD1306_Command(0x14); //internal vcc from pump

	SSD1306_Command(SSD1306_MEMORYMODE);
	SSD1306_Command(0x00);
	SSD1306_Command(SSD1306_SEGREMAP | 0x1);
	SSD1306_Command(SSD1306_COMSCANDEC);

	SSD1306_Command(SSD1306_SETCOMPINS);
	SSD1306_Command(0x12);
	SSD1306_Command(SSD1306_SETCONTRAST);
	SSD1306_Command(0xFF);

	SSD1306_Command(SSD1306_SETPRECHARGE);
	SSD1306_Command(0xF1);

	SSD1306_Command(SSD1306_SETVCOMDETECT);
	SSD1306_Command(0x40);
	SSD1306_Command(SSD1306_DISPLAYALLON_RESUME);
	SSD1306_Command(SSD1306_NORMALDISPLAY);
	SSD1306_Command(SSD1306_DEACTIVATE_SCROLL);
	SSD1306_Command(SSD1306_DISPLAYON); // Main screen turn on

}
