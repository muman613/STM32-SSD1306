/*
 * panel.c
 *
 *  Created on: Jul 16, 2019
 *      Author: muman
 */

#include "main.h"
#include "ssd1306.h"
#include "fonts.h"

void drawPanel(const char * header, const char * message)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_GotoXY ((SSD1306_WIDTH - (strlen(header) * 11))/2, 0); // goto 10, 10
	SSD1306_Puts((char *)header, &Font_11x18, SSD1306_COLOR_WHITE);

	SSD1306_GotoXY ((SSD1306_WIDTH - (strlen(message) * 7))/2, (SSD1306_HEIGHT - 18)/2 + 12); // goto 10, 10
	SSD1306_Puts((char *)message, &Font_7x10, SSD1306_COLOR_WHITE);

	SSD1306_DrawRectangle(0, 18, SSD1306_WIDTH, SSD1306_HEIGHT - 18, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
}
