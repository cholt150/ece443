#define LCDIR 0
#define LCDDR 1
#define busyLCD() readLCD(LCDIR) & 0x80 //ISOLATE BUSY FLAG
#define addrLCD() readLCD(LCDIR) & 0x7F //ISOLATE ADDRESS

void LCD_puts(char *char_string);
void LCD_putc(char char_string);
void PMP_init();
void writeLCD(int addr, char c);
char readLCD(int addr);
void LCD_init();
void hw_delay(unsigned int ms);
