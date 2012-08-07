// Ansteuerung eines HD44780 kompatiblen LCD im 4-Bit-Interfacemodus
// http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial
//

void lcd_data(unsigned char temp1);
void lcd_command(unsigned char temp1);
void lcd_send(unsigned char data);
void lcd_string(char *data);
void lcd_enable(void);
void lcd_init(void);
void lcd_clear(void);
void lcd_eep_string(const unsigned char *data);

//LCD-Befehle
#define CMD_SetEntryMode         0x04
#define CMD_SetDisplayAndCursor  0x08
#define CMD_SetIFOptions         0x20
#define CMD_SetCGRAMAddress      0x40    // für Custom-Zeichen
#define CMD_SetDDRAMAddress      0x80    // zum Cursor setzen

//Makros für LCD
#define Line1() SetCursor(1,0)	//An den Anfang der 1. Zeile springen
#define Line2() SetCursor(2,0)	//An den Anfang der 2. Zeile springen

#define SetCursor(y, x) lcd_command((uint8_t)(CMD_SetDDRAMAddress + (0x40*(y-1)) + x)) //An eine bestimmte Position springen

#define LCDLoadCustomChar() lcd_command(CMD_SetCGRAMAddress)	//Custom-Zeichen laden

//Eigene Zeichen
#define LCD_CHAR_OMEGA  244	//Omega-Zeichen
#define LCD_CHAR_U  228		//µ-Zeichen
#define LCD_CHAR_DIODE  0	//Dioden-Icon; wird als Custom-Character erstellt
  
// LCD Befehle
 
#define CLEAR_DISPLAY 0x01
 
// Pinbelegung für das LCD, an verwendete Pins anpassen
 
#define LCD_PORT      PORTD
#define LCD_DDR       DDRD
#define LCD_RS        PD4
#define LCD_EN1       PD5
