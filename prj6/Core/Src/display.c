#include "display.h"

void display_init(void)
{
  //Init LCD
  HAL_GPIO_WritePin(BLK_PORT, BLK_PIN, GPIO_PIN_SET);
  ST7789_Init();
}

void display_text(const char *buffer)
{
  uint16_t x = 10;
  uint16_t y = 10;

  for (int i = 0; buffer[i] != '\0'; i++)
  {
    if (buffer[i] == '\n' || buffer[i] == '\r')
    {
      y += 10;  // New lines
      x = 10;   // Reset to head of lines
    }
    else
    {
      ST7789_WriteChar(x, y, buffer[i], Font_7x10, BLACK, WHITE);
      x += 7;  // Increase position of clolumn in 1 char
    }
  }
}

void display_text_line(const char *buffer, uint8_t line, uint8_t column)
{
  uint8_t row = line * 10;
  ST7789_WriteString(column, row, buffer, Font_7x10, BLACK, WHITE);
}

void display_clear(void)
{
  ST7789_Fill_Color(WHITE);

}

