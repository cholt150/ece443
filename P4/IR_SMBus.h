#define IR_ADDR 0x5A //Sensor Slave address
#define RAM_ADDR 0x07 //RAM addr for reading temp

#define Fsck 50000
#define BRG_VAL ((FPB/2/Fsck)-2)

int init_I2C2(void);
float readTemp(void);