#include <plib.h>
#include "CerebotMX7cK.h"
#include "IR_SMBus.h"

int init_I2C2(void) {
    OpenI2C2(I2C_EN, BRG_VAL);
    IdleI2C2();
}

float readTemp(void) {
    int err = 0;
    int LSB = 0, MSB = 0, PEC = 0;
    StartI2C2();
    IdleI2C2();
    
    err |= MasterWriteI2C2((IR_ADDR << 1) | 0); //Control Byte write
    IdleI2C2();
    
    err |= MasterWriteI2C2(RAM_ADDR);
    IdleI2C2();
    
    RestartI2C2(); //Change bus direction
    IdleI2C2();
    
    err |= MasterWriteI2C2((IR_ADDR << 1) | 1); //Control Byte read
    
    if(err) 
        return -999;
    
    LSB = MasterReadI2C2();
    AckI2C2();
    IdleI2C2();
    
    MSB = MasterReadI2C2();
    AckI2C2();
    IdleI2C2();
    
    PEC = MasterReadI2C2();
    AckI2C2();
    IdleI2C2();
    
    StopI2C2();
    IdleI2C2();
    
    float K = (float)((MSB << 8) | LSB)*0.02;
    float C = K - 273.15;
    
    
    return(C); 
}