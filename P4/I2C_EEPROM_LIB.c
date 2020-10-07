#include <plib.h>
#include "CerebotMX7cK.h"
#include "I2C_EEPROM_LIB.h"

char BusyI2C2(void) {
    return(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RSEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
}

int init_I2C2(void) {
    OpenI2C2(I2C_EN, BRG_VAL);
    if(BusyI2C2()) return(1);
}

int I2CReadEEPROM(int SlaveAddress, int MemAddress, char *i2cData, int len){
    int write_err = 0, index = 0;
    int MemAddrHigh = MemAddress & 0xFF00;      //CLEAR LOWER BYTE
    MemAddrHigh = (MemAddrHigh >> 4);         //SHIFT TO PROPER POSITION
    int MemAddrLow = MemAddress & 0x00FF;       // CLEAR UPPER BYTE
    
    if((MemAddress < 0) || (MemAddress > 32768)) return(1); // Indicate to Main that there was a problem if mem addr is invalid.
    if(len == 0) return(1);
    if(i2cData == NULL) return(1);
    
    StartI2C2();
    IdleI2C2();
    
    write_err |= MasterWriteI2C2((SlaveAddress << 1) | 0); // CONTROL BYTE WRITE TO ADDR REG
    write_err |= MasterWriteI2C2(MemAddrHigh);             // WRITE MEM ADDR MSB
    write_err |= MasterWriteI2C2(MemAddrLow);              // WRITE MEM ADDR LSB
    
    if(write_err) return(1); //IF ERROR THEN RETURN;
    
    RestartI2C2();           //REVERSE BUS
    IdleI2C2();
    
    MasterWriteI2C2((SlaveAddress << 1) | 1); //SEND CONTROL BYTE READ
    while(len--) { //Loop While there are sill bytes to be read.
        i2cData[index] = MasterReadI2C2();
        if(len > 0) { //If there are still bytes to read, then send ACK.
            AckI2C2();
            IdleI2C2();
        }
        if(len == 0) { //If no more bytes, send NACK.
            NotAckI2C2();
            IdleI2C2();
        }
        index++;
    }
    
    StopI2C2();
    IdleI2C2();
    if(write_err) return(1);
    else return(0);
}

int I2CWriteEEPROM(int SlaveAddress, int MemAddress, char *i2cData, int len){
    int write_err = 0, index = 0, TempAddress;
    int MemAddrHigh = MemAddress & 0xFF00;      //CLEAR LOWER BYTE
    MemAddrHigh = (MemAddrHigh >> 4);         //SHIFT TO PROPER POSITION
    int MemAddrLow = MemAddress & 0x00FF;       // CLEAR UPPER BYTE
    
    if((MemAddress < 0) || (MemAddress > 32768)) return(1); // Indicate to Main that there was a problem if mem addr is invalid.
    if(len == 0) return(1);
    if(i2cData == NULL) return(1);
    
    StartI2C2();
    IdleI2C2();
    
    TempAddress = MemAddress; //Temp address for keeping track of page edges
    write_err |= MasterWriteI2C2((SlaveAddress << 1) | 0); // CONTROL BYTE WRITE TO ADDR REG
    write_err |= MasterWriteI2C2(MemAddrHigh);             // WRITE MEM ADDR MSB
    write_err |= MasterWriteI2C2(MemAddrLow);              // WRITE MEM ADDR LSB
    
    while(len--) { //begin Write loop
        write_err |= MasterWriteI2C2(i2cData[index++]);
        TempAddress++; //Variable for Next memory location        
        
        if((TempAddress % 64) == 0) { //if next mem location is div by 64 commit to memory
            StopI2C2();
            IdleI2C2();
            wait_I2C_xfer(SlaveAddress);
            StartI2C2();
            IdleI2C2();
            //RESEND CONTROL BYTE WITH NEXT MEM ADDR
            write_err |= MasterWriteI2C2((SlaveAddress << 1) | 0); // CONTROL BYTE WRITE TO ADDR REG
            write_err |= MasterWriteI2C2((TempAddress) >> 8);             // WRITE MEM ADDR MSB
            write_err |= MasterWriteI2C2((TempAddress));              // WRITE MEM ADDR LSB
        }
    }
    StopI2C2();
    IdleI2C2();
    wait_I2C_xfer(SlaveAddress); //WAIT UNTIL EEPROM IS DONE WRITING
    if(write_err) return(1); //IF ERROR THEN RETURN;
}

void wait_I2C_xfer(int SlaveAddress){
    StartI2C2();
    IdleI2C2();
    while(MasterWriteI2C2((SlaveAddress << 1) | 0)){ //LOOP UNTIL EEPROM ACK. WRITE CB WITH WRITE
        RestartI2C2();
        IdleI2C2();
    }
    StopI2C2();
    IdleI2C2();
}
