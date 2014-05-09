#include "gsr_lcd.h"

LiquidTWI2 lcd(0);

void initLCD() {
	lcd.setMCPType(LTI_TYPE_MCP23017);
	// set up the LCD's number of columns and rows: 
	lcd.begin(16, 2);
	lcd.setBacklight(WHITE);
}

void setLCD(char *s) {
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(s);
}
