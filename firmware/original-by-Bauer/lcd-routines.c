// Ansteuerung eines HD44780 kompatiblen LCD im 4-Bit-Interfacemodus
// http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial
//
// Die Pinbelegung ist über defines in lcd-routines.h einstellbar
 
#include <avr/io.h>
#include "lcd-routines.h"
#include <util/delay.h>
#include <avr/eeprom.h>
 
// sendet ein Datenbyte an das LCD
 
void lcd_data(unsigned char temp1)
{
   LCD_PORT |= (1<<LCD_RS);        // RS auf 1 setzen
   lcd_send(temp1);
}
 
// sendet einen Befehl an das LCD
 
void lcd_command(unsigned char temp1)
{
   LCD_PORT &= ~(1<<LCD_RS);        // RS auf 0 setzen
   lcd_send(temp1);
}

//Eigentliche LCD-Zugriffs-Funktion; 4-Bit-Modus
void lcd_send(unsigned char data) {
   // oberes Nibble setzen
  LCD_PORT = (LCD_PORT & 0xF0) | ((data >> 4) & 0x0F);
  _delay_us(5);
  lcd_enable();
   // unteres Nibble setzen
  LCD_PORT = (LCD_PORT & 0xF0) | (data & 0x0F);
  _delay_us(5);
  lcd_enable();
  _delay_us(60);  
  LCD_PORT &= 0xF0;
}

// erzeugt den Enable-Puls
void lcd_enable(void)
{
	LCD_PORT |= (1<<LCD_EN1);
    _delay_us(10);                   // kurze Pause
   // Bei Problemen ggf. Pause gemäß Datenblatt des LCD Controllers verlängern
   // http://www.mikrocontroller.net/topic/80900
   LCD_PORT &= ~(1<<LCD_EN1);
}
 
// Initialisierung: 
// Muss ganz am Anfang des Programms aufgerufen werden.
 
void lcd_init(void)
{
	LCD_DDR = LCD_DDR | 0x0F | (1<<LCD_RS) | (1<<LCD_EN1);   // Port auf Ausgang schalten
	// muss 3mal hintereinander gesendet werden zur Initialisierung
	_delay_ms(30);
	LCD_PORT = (LCD_PORT & 0xF0 & ~(1<<LCD_RS)) | 0x03;
	lcd_enable();

	_delay_ms(5);
	lcd_enable();

	_delay_ms(1);
	lcd_enable();
	_delay_ms(1);
	LCD_PORT = (LCD_PORT & 0xF0 & ~(1<<LCD_RS)) | 0x02;
	_delay_ms(1);
	lcd_enable();
	_delay_ms(1);

	// 4Bit / 2 Zeilen / 5x7
	lcd_command(CMD_SetIFOptions | 0x08);

	// Display ein / Cursor aus / kein Blinken
	lcd_command(CMD_SetDisplayAndCursor | 0x04);

	// inkrement / kein Scrollen    
	lcd_command(CMD_SetEntryMode | 0x02);	
	lcd_clear();
}
 
// Sendet den Befehl zur Löschung des Displays
 
void lcd_clear(void)
{
   lcd_command(CLEAR_DISPLAY);
   _delay_ms(5);
}
 
 
// Schreibt einen String auf das LCD
 
void lcd_string(char *data)
{
    while(*data) {
        lcd_data(*data);
        data++;
    }
}

//String aus EEPROM laden und an LCD senden
void lcd_eep_string(const unsigned char *data)
{	
	unsigned char c;
    while(1) {
		c = eeprom_read_byte(data);
		if(c==0) return;
        lcd_data(c);
        data++;
    }
}
