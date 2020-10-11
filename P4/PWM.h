#define T2_PRESCALE 1
#define TOGGLES_PER_SEC 1000
#define T2_TICK (FPB/T2_PRESCALE/TOGGLES_PER_SEC)


void OutputCompareInit(int duty);

void ChangeDuty(int duty);
