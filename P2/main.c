/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Standard demo includes. */
#include <plib.h>

/* Hardware specific includes. */
#include "CerebotMX7cK.h" 

/* TraceAlyzer config  */
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString trace_handler;
    traceString trace_ms;
	traceString trace_ledc_on;
#endif


/* User Defined Function Prototypes  */
static void prvSetupHardware( void );


/* User Defined Task Prototypes  */
static void msTask();
static void ledHandler();

/* Semaphore handle creation */
xSemaphoreHandle led_semaphore;
 
/* Global Variables */
unsigned int LEDC_COUNT = 0;
void __ISR(_CHANGE_NOTICE_VECTOR, IPL1) ISR_Wrapper(void);

int main( void )
{
    prvSetupHardware();		/*  Configure hardware */
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTraceEnable(TRC_START); // Initialize and start recording
        trace_handler = xTraceRegisterString("handler task");
        trace_ms = xTraceRegisterString("ms task");
		trace_ledc_on = xTraceRegisterString("LEDC on for 2+ ms");
    #endif
    
    BaseType_t xReturned; //Empty task return var.
        
	xReturned = xTaskCreate(msTask, "Task to blink every ms", 
                            configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
	if(xReturned == NULL) return 0;


	xReturned = xTaskCreate(ledHandler, "Toggles LED when unblocked", 
                            configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
	if(xReturned == NULL) return 0;


	/* Create a binary semaphore */
	led_semaphore = xSemaphoreCreateBinary();
	if (led_semaphore == NULL) { return 0; }

	/* Create the tasks then start the scheduler. */

    vTaskStartScheduler();	/*  Finally start the scheduler. */

	// SHould never reach here.
    return 0;
}   /* End of main */

static void prvSetupHardware( void )
{
    Cerebot_mx7cK_setup();
    
    /* Set up IO */
    PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
    LATBSET = SM_LEDS;
    LATBCLR = SM_LEDS;                      /* Clear all SM LED bits */

	PORTSetPinsDigitalIn(IOPORT_G, BTN1);
    
	/* Enable the Interrupt for BTN1 */
	mCNOpen(CN_ON, CN8_ENABLE, 0);
	mCNSetIntPriority(1);
	mCNSetIntSubPriority(0);
	unsigned int dummy = PORTReadBits(IOPORT_G, BTN1);
	mCNClearIntFlag();
	mCNIntEnable(1);

	INTEnableSystemMultiVectoredInt();
}
/*-----------------------------------------------------------*/

void isr_cn() {
	LATBSET = LEDD;
	
	/* Provide the LED handler task with a semaphore */
	/* Argument 2 is optional. change if priority gives problems. See docs. */
	xSemaphoreGiveFromISR(led_semaphore, NULL);
    mCNOpen(CN_ON,  CN_DISABLE_ALL, 0);
	mCNClearIntFlag();
	LATBCLR = LEDD;
}

static void msTask() {
	while(1) {
        #if(configUSE_TRACE_FACILITY)
            vTracePrint(trace_ms, "Toggle LEDB");
        #endif
		LATBINV = LEDB;
		if((PORTB & LEDC) == LEDC) {
			LEDC_COUNT ++;
			if(LEDC_COUNT >= 2) {
				#if(configUSE_TRACE_FACILITY)
            		vTracePrint(trace_ledc_on, "LEDC on twice");
        		#endif
				LEDC_COUNT = 0; //Reset count so that 
			}
		}
		vTaskDelay(1/portTICK_RATE_MS);
	}
}

static void ledHandler() {
	while(1) {
		xSemaphoreTake(led_semaphore, portMAX_DELAY);
		vTaskDelay(20/portTICK_PERIOD_MS); //Stop bouncing pls.
		while((PORTG & BTN1) != 0) //Loop until the button turns off.
        vTaskDelay(20/portTICK_PERIOD_MS);

        //Disable the interrupt. Weird flickering when enabled.
        mCNOpen(CN_ON,  CN8_ENABLE, 0);
        
        //
		#if (configUSE_TRACE_FACILITY)
			vTracePrint(trace_handler, "LED Handler task");
		#endif
		LATBINV = LEDC; //Toggle LEDC
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
	int status = PORTG;
	if((status & BTN1) == BTN1) {
		LATBSET = LEDA;
	}
	else {
		LATBCLR = LEDA;
	}
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
