#include <plib.h>
#include "CerebotMX7cK.h"
#include "PWM.h"

void OutputCompareInit(int duty){
    mOC3ClearIntFlag();
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, T2_TICK-1);
    mT2SetIntPriority(2);
    mT2SetIntSubPriority(1); 
    
    OpenOC3(OC_ON|OC_TIMER_MODE16|OC_TIMER2_SRC|OC_PWM_FAULT_PIN_DISABLE, (T2_TICK*duty)/100,(T2_TICK*duty)/100 );
    
    mT2IntEnable(1);
}

void ChangeDuty(int duty){
    SetDCOC3PWM((T2_TICK*duty)/100);  
}

void __ISR(_TIMER_2_VECTOR, IPL2) Timer2Handler(void){
    LATBINV = LEDA;
    mT2ClearIntFlag(); // Macro function to clear the interrupt flag
}

