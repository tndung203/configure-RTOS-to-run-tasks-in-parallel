#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "st7789.h"
void display_init(void);
void display_text(const char *buffer);
void display_text_line(const char *buffer, uint8_t line, uint8_t column);
void display_clear(void);

#endif
