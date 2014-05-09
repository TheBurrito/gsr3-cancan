#ifndef gsr_lcd_h_
#define gsr_lcd_h_

#include <Wire.h>
#include <LiquidTWI2.h>
#include <Adafruit_RGBLCDShield.h>

#define BLACK 0x0
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define VIOLET 0x5
#define TEAL 0x6
#define WHITE 0x7

extern LiquidTWI2 lcd;

void initLCD();

void setLCD(char *s);

#endif //gsr_lcd_h_
