/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware specific includes. */
#include <plib.h>
#include<stdio.h>
#include<string.h>
#include "CerebotMX7cK.h"
#include "LCDlib.h"
#include "IR_SMBus.h"
#include "PWM.h"
#include "ICapture.h"
#include "CAN_P4.h"

#define DEBOUNCE_TIME 20/portTICK_RATE_MS
#define ONE_SEC 1000/portTICK_RATE_MS
#define SLAVE_ADDRESS 0x50
/*-----------------------------------------------------------*/

/* Global Variables */
int CONTROL_UNIT_STATE = 0;
extern float motorSpeed;
//Temp global variables to stand in for CAN


/* FreeRTOS elements */
xSemaphoreHandle cn_semaphore;

/* Function Prototypes */
static void prvSetupHardware();

/* Task Prototypes  */
static void control_unit();
static void btn_handler();
static void IO_unit();
void __ISR(_CHANGE_NOTICE_VECTOR, IPL1) vCN_ISR_Wrapper(void);


int main( void )
{   
    prvSetupHardware();		/*  Configure hardware */
    
    /* Create Tasks */
    xTaskCreate(control_unit, "control FSM", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(btn_handler, "btn_handler", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(IO_unit, "IO unit", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    
    /* Create Semaphores */
    cn_semaphore = xSemaphoreCreateBinary();
    
    /*  Finally start the scheduler. */
    vTaskStartScheduler();

    return 0;
}  /* End of main */

static void prvSetupHardware( void )
{
    Cerebot_mx7cK_setup();
    
    PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
    LATBCLR = SM_LEDS;
    
    PORTSetPinsDigitalOut(IOPORT_G, BRD_LEDS);
    LATGCLR = BRD_LEDS;
    
    PORTSetPinsDigitalIn(IOPORT_G, BTN2);
    unsigned int dummy = PORTReadBits(IOPORT_G, BTN2);
    PORTSetPinsDigitalIn(IOPORT_A, BTN3);
    dummy = PORTReadBits(IOPORT_A, BTN3);
    
    
    PMP_init();
    LCD_init();
    LCD_puts("LCD INITIALIZED");
    hw_delay(500);
    init_I2C1();
    LCD_puts("I2C INITIALIZED");
    hw_delay(500);
    
    //Init motor modules
    OutputCompareInit(0);
    IC_init();
    
    CAN1Init();
    CAN2Init();
    
    //BTN1 CN Int init.
	mCNOpen(CN_ON, CN8_ENABLE, 0);
	mCNSetIntPriority(2);
	mCNSetIntSubPriority(0);
    dummy = PORTReadBits(IOPORT_G, BTN1);
	mCNClearIntFlag();
	mCNIntEnable(1);    
    
    INTEnableSystemMultiVectoredInt();
}
/*-----------------------------------------------------------*/

void isr_cn() {
    xSemaphoreGiveFromISR(cn_semaphore,NULL);
    mCNIntEnable(0);
    mCNClearIntFlag();
}

static void btn_handler() {
    while(1) {
        xSemaphoreTake(cn_semaphore, portMAX_DELAY);
        if(CONTROL_UNIT_STATE == 0) {
            CONTROL_UNIT_STATE = 1;
        }
        else {
            CONTROL_UNIT_STATE = 0;
        }
        vTaskDelay(DEBOUNCE_TIME);
        while((PORTG & BTN1) != 0);
        vTaskDelay(DEBOUNCE_TIME);
        mCNClearIntFlag();
        mCNIntEnable(1);
    }
}

static void control_unit() {
    int IR_data = 0;
    float RPS = 0;
    int desired_PWM = 0;
    int current_PWM = 0;
    unsigned int t_start = ReadCoreTimer();; //holds timestamp of next update
    char top_line[16];
    char btm_line[16];
    float temp = 0.0;
    float lo = 0.0, hi = 0.0;
    while(1) {
        //Add CAN request
        if((ReadCoreTimer() - t_start) > (CORE_MS_TICK_RATE * 2000)) {
            CAN1TxSendMsg(1,0);//Send an RTR message
            t_start = ReadCoreTimer();
        }
        //Receive the CAN sensor packet
        // IR_data, RPS, current_PWM
        CAN1RxMsgProcess(&IR_data, &RPS, &current_PWM);
        //Sensor data to temperature
        float K = IR_data * 0.02;
        float C = K - 273.15;
        temp = ((9.0/5.0) * C) + 32;
        
        switch (CONTROL_UNIT_STATE) {
            case 0: //Configuration mode
                LATGCLR = LED1;
                
                if((PORTG & BTN2) != 0) {
                    hi = temp;
                }
                if((PORTA & BTN3) != 0) {
                    lo = temp;
                }
                
                writeLCD(LCDIR,0x01); //clears entire screen
                
                //Line formatting
                sprintf(top_line, "      %2.1f       ", temp);
                sprintf(btm_line, "%3.1f%12.1f", lo, hi);
                
                LCD_cursor_top();
                LCD_puts(top_line);
                LCD_cursor_bottom();
                if(hi>lo) {
                    LCD_puts(btm_line);
                }
                else {
                    hi = lo = 0;
                    LCD_clear_line();
                }
                
                break;
            case 1: //Operational mode
                LATGSET = LED1;
                
                writeLCD(LCDIR,0x01); //clears entire screen
                sprintf(top_line, "%2i%%%13.1f", current_PWM, RPS);
                sprintf(btm_line, "%3.1f%6.1f%6.1f",lo,temp,hi);
                
                LCD_cursor_top();
                LCD_puts(top_line);
                LCD_cursor_bottom();
                LCD_puts(btm_line);
                
//                top_line = {'\0'};
//                btm_line = {'\0'};
                
                if(temp < lo) {
                    desired_PWM = 20;
                }
                else if(temp > hi) {
                    desired_PWM = 95;
                }
                else {
                    float m = (85-30)/(hi-lo);
                    float b = 30-(m*lo);
                    desired_PWM = (m * temp) + b;
                }
                
                CAN1TxSendMsg(0,desired_PWM);
                
                break;
        }
        vTaskDelay(ONE_SEC/2);
    }
}

static void IO_unit() {
    CANTxMessageBuffer * sensor_packet = CANGetTxMessageBuffer(CAN2,CAN_CHANNEL0);
    int RPS;
    int IR_data = 0;
    int current_PWM;
    int desired_PWM;
    while(1) {
        sensor_packet->messageWord[0] = 0;
        sensor_packet->messageWord[1] = 0;
        sensor_packet->messageWord[2] = 0;
        sensor_packet->messageWord[3] = 0;
        IR_data = readTemp();
        
        //Populate sensor packet.
        sensor_packet->data[0] = IR_data & 0x00FF; //LSB
        sensor_packet->data[1] = IR_data >> 8; //MSB
        
        RPS = (int)(motorSpeed * 100); //Upscale to preserve digits
        sensor_packet->data[2] = RPS & 0x0000FF;
        sensor_packet->data[3] = (RPS & 0x00FF00) >> 8;
        sensor_packet->data[4] = (RPS & 0xFF0000) >> 16;
        
        sensor_packet->data[5] = current_PWM & 0x00FF;
        sensor_packet->data[6] = current_PWM >> 8;
        //Add
        desired_PWM = CAN2RxMsgProcess(sensor_packet);
        if(desired_PWM != 0) { //If the message was not an RTR
            ChangeDuty(desired_PWM);
            current_PWM = desired_PWM;
        }
        
        vTaskDelay(ONE_SEC/2);
    }
}

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time task stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook	function is 
	called if a task stack overflow is detected.  Note the system/interrupt
	stack is not checked. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/

void _general_exception_handler( unsigned long ulCause, unsigned long ulStatus )
{
	/* This overrides the definition provided by the kernel.  Other exceptions 
	should be handled here. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
volatile unsigned long ul = 0;

	( void ) pcFile;
	( void ) ulLine;

	__asm volatile( "di" );
	{
		/* Set ul to a non-zero value using the debugger to step out of this
		function. */
		while( ul == 0 )
		{
			portNOP();
		}
	}
	__asm volatile( "ei" );
}