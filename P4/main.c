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

#define DEBOUNCE_TIME 20/portTICK_RATE_MS
#define ONE_SEC 1000/portTICK_RATE_MS
#define SLAVE_ADDRESS 0x50
/*-----------------------------------------------------------*/

/* Global Variables */

/* FreeRTOS elements */

/* Function Prototypes */
static void prettyPrint(char* input);
static void PrintLineOne(char *line);
static void PrintLineTwo(char *line);
static void prvSetupHardware( void );
void isr_uart();

/* Task Prototypes  */
static void heartbeat();
static void mem_write();
static void mem_read();
//void __ISR(_UART1_VECTOR, IPL1) vUART_ISR_Wrapper(void);
//void __ISR(_CHANGE_NOTICE_VECTOR, IPL1) vCN_ISR_Wrapper(void);

    
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString heartbeat_trace;
    traceString uart_isr_trace;
    traceString read_trace;
    traceString write_trace;
#endif

int main( void )
{   
    
    prvSetupHardware();		/*  Configure hardware */
    init_I2C2();
    while(1) {
        char str[16];
        float F = readTemp();
        sprintf(str, "Temp = %f", F);
        LCD_puts(str);
        hw_delay(1000);
    }
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTraceEnable(TRC_START); // Initialize and start recording
        heartbeat_trace = xTraceRegisterString("heartbeat task");
        uart_isr_trace = xTraceRegisterString("UART Interrupt");
        write_trace = xTraceRegisterString("Message Written to EEPROM");
        read_trace = xTraceRegisterString("Message Read from EEPROM");
    #endif


    
    LATBSET = LEDA; //Indicate that you can begin writing a message.

    /*  Finally start the scheduler. */
    vTaskStartScheduler();

/* Will only reach here if there is insufficient heap available to start
 *  the scheduler. */
    return 0;
}  /* End of main */

static void prvSetupHardware( void )
{
    Cerebot_mx7cK_setup();
    
    PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
    LATBCLR = SM_LEDS;
    
    
    
    PMP_init();
    LCD_init();
    LCD_puts("LCD INITIALIZED");
    hw_delay(500);
    init_I2C2();
    LCD_puts("I2C INITIALIZED");
    hw_delay(500);
    initialize_uart1(19200, 1);
    putsU1("UART INITIALIZED");
    
    //UART Interrupt initialization
    mU1RXIntEnable(1);
    mU1SetIntPriority(1);
    mU1SetIntSubPriority(0);
    
    //BTN1 CN Int init.
	mCNOpen(CN_ON, CN8_ENABLE, 0);
	mCNSetIntPriority(2);
	mCNSetIntSubPriority(0);
	unsigned int dummy = PORTReadBits(IOPORT_G, BTN1);
	mCNClearIntFlag();
	mCNIntEnable(1);    
    
    INTEnableSystemMultiVectoredInt();
}
/*-----------------------------------------------------------*/



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