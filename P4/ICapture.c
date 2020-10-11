
#include <plib.h>
#include "CerebotMX7cK.h"
#include "ICapture.h"


float motorSpeed = 100.00;

void IC_init() {
    OpenTimer3(T3_ON|T3_PS_1_256| T3_SOURCE_INT, 0xFFFF);
    mT3SetIntPriority(2); // Set Timer 3 interrupt group priority 2
    mT3SetIntSubPriority(2); // Set Timer 3 interrupt subgroup priority 2
    mT3IntEnable(1); // Enable Timer 3 interrupts
    
    PORTSetPinsDigitalIn(IOPORT_D,(MTR_SA|MTR_SB));
    mIC5ClearIntFlag();
    
    OpenCapture5(IC_ON|IC_CAP_16BIT|IC_IDLE_STOP|IC_FEDGE_FALL|IC_TIMER3_SRC|IC_INT_1CAPTURE|IC_EVERY_FALL_EDGE);
    ConfigIntCapture5(IC_INT_ON|IC_INT_PRIOR_3|IC_INT_SUB_PRIOR_0);
}


void __ISR(_TIMER_3_VECTOR, IPL2) Timer3Handler(void){
    LATBINV=LEDC;
    mT3ClearIntFlag();
}

void __ISR(_INPUT_CAPTURE_5_VECTOR, IPL3) Capture5(void){
    static unsigned int con_buf[4]; // Declare an input capture buffer
    // Declare three time capture variables:
    static unsigned short int t_new; // Most recent captured time
    static unsigned short int t_old = 0; // Previous time capture
    static unsigned short int time_diff; // Time between captures
    LATBINV = LEDD; //Toggle LEDD on each input capture interrupt
    ReadCapture5(con_buf); // Read captures into buffer
    t_new = con_buf[0]; // Save time of event
    time_diff = t_new-t_old; // Compute elapsed time in timer ?ticks?
    t_old = t_new; // Replace previous time capture with new
    static int timeValues[16];
    static int index = 0;
    timeValues[index] = time_diff;
    int i = 0;
    float average = 0;
    for(i=0; i<16; i++) {
        average = average + timeValues[i];
    }
    average /= 16.0;
    index = (index+1)%16;
    
    motorSpeed = 1/((average*256.0)/10000000.0);
    mIC5ClearIntFlag(); // Clears interrupt flag
    // INTClearFlag(INT_IC5); // Alternate peripheral library function



}


