#include <plib.h>
#include "CerebotMX7cK.h"
#include "LCDlib.h"

void PMP_init()
{
    int cfg1 = PMP_ON|PMP_READ_WRITE_EN|PMP_READ_POL_HI|PMP_WRITE_POL_HI; 
    int cfg2 = PMP_DATA_BUS_8 | PMP_MODE_MASTER1 | PMP_WAIT_BEG_4 | PMP_WAIT_MID_15 | PMP_WAIT_END_4; 
    int cfg3 = PMP_PEN_0;        // only PMA0 enabled 
    int cfg4 = PMP_INT_OFF;      // no interrupts used 
    mPMPOpen(cfg1, cfg2, cfg3, cfg4);
}

void writeLCD(int addr, char c)
{
    while(busyLCD());
    PMPSetAddress(addr);
    PMPMasterWrite(c);
}

char readLCD(int addr)
{
    PMPSetAddress(addr);
    mPMPMasterReadByte();
    return mPMPMasterReadByte();
}

void hw_delay(unsigned int ms) {
    unsigned int tWait, tStart;
    tStart = ReadCoreTimer();
    tWait = (CORE_MS_TICK_RATE * ms);
    while ((ReadCoreTimer() - tStart) < tWait);
}

void LCD_init()
{
    hw_delay(50);
    PMPSetAddress(LCDIR);
    PMPMasterWrite(0x38);
    hw_delay(50);
    PMPMasterWrite(0x0F);
    hw_delay(50);
    PMPMasterWrite(0x01);
    hw_delay(5);
    writeLCD(LCDIR,0x01);
    writeLCD(LCDIR,0x02);
}

void LCD_putc(char char_string)
{
    while(busyLCD());
    int address = addrLCD();
    switch (char_string)
    {
        case '\n':
            writeLCD(LCDIR, (0x40 + 0x80));
            break;
        case '\r':
            if(address <= 16)
            {
                writeLCD(LCDIR,(0 + 0x80));
            }
            if((address >= 0x40) && (address <= 0x50))
            {
                writeLCD(LCDIR,(0x40 + 0x80));
            }
            break;
        default:
            if((address > 0x0F) && (address < 0x40))
            {
                writeLCD(LCDIR,(0x40+0x80)); //Set address of cursor to 0x40
            }
            if(address > 0x4F)
            {
                writeLCD(LCDIR,0x80); // set address of cursor to 0x00
            }
            writeLCD(LCDDR,char_string);
    }
}

void LCD_puts(char *char_string)
{
    writeLCD(LCDIR,0x01);
    writeLCD(LCDIR,0x02);
    while(*char_string)
    {
        
        LCD_putc(*char_string);
        *char_string++;
    }
}

