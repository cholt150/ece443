#include <plib.h>
#include "CerebotMX7cK.h"
#include "IR_SMBus.h"

int init_I2C1(void) {
    OpenI2C1(I2C_EN, BRG_VAL);
    IdleI2C1();
}

int readTemp(void) {
    int err = 0;
    int LSB = 0, MSB = 0, PEC = 0;
    StartI2C1();
    IdleI2C1();
    
    err |= MasterWriteI2C1((IR_ADDR << 1) | 0); //Control Byte write
    IdleI2C1();
    
    err |= MasterWriteI2C1(RAM_ADDR);
    IdleI2C1();
    
    RestartI2C1(); //Change bus direction
    IdleI2C1();
    
    err |= MasterWriteI2C1((IR_ADDR << 1) | 1); //Control Byte read
    IdleI2C1();
    
//    if(err) 
//        return -999;
    
    LSB = MasterReadI2C1();
    AckI2C1();
    IdleI2C1();
    
    MSB = MasterReadI2C1();
    AckI2C1();
    IdleI2C1();
    
    PEC = MasterReadI2C1();
    AckI2C1();
    IdleI2C1();
    
    StopI2C1();
    IdleI2C1();
    
//    float K = (float)((MSB << 8) | LSB)*0.02;
//    float C = K - 273.15;
//    float F = ((9.0/5.0) * C) + 32;
    
    return((MSB << 8) | LSB);
}