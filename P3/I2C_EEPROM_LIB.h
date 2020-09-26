#define Fsck 400000
#define BRG_VAL ((FPB/2/Fsck)-2)


//FUNCTION PROTOTYPES

int init_I2C2(void);
char BusyI2C2(void);
int I2CReadEEPROM(int SlaveAddress, int MemAddress, char *i2cData, int len);
int I2CWriteEEPROM(int SlaveAddress, int MemAddress, char *i2cData, int len);
void wait_I2C_xfer(int SlaveAddress);