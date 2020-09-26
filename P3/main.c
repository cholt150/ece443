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
#include "comm.h"
#include "I2C_EEPROM_LIB.h"
#include "LCDlib.h"

/*-----------------------------------------------------------*/

/* Global Variables */
unsigned int INDEX = 0;
//xQueueHandle mem_queue;
static char input_str[80];
int SLAVE_ADDRESS = 0x50;
int MESSAGE_COUNT = 0;
xSemaphoreHandle mem_semaphore;
xSemaphoreHandle cn_semaphore;

/* Function Prototypes */
static void prvSetupHardware( void );
void isr_uart();

/* Task Prototypes  */
static void heartbeat();
static void mem_write();
//static void mem_read();
void __ISR(_UART1_VECTOR, IPL1) vUART_ISR_Wrapper(void);
void __ISR(_CHANGE_NOTICE_VECTOR, IPL1) vCN_ISR_Wrapper(void);

    
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString str;
#endif

/* main Function Description ***************************************
 * SYNTAX:		int main( void );
 * KEYWORDS:		Initialize, create, tasks, scheduler
 * DESCRIPTION:         This is a typical RTOS set up function. Hardware is
 * 			initialized, tasks are created, and the scheduler is
 * 			started.
 * PARAMETERS:		None
 * RETURN VALUE:	Exit code - used for error handling
 * NOTES:		All three buttons are polled using the same code
 *                      for reading the buttons.
 * END DESCRIPTION *****************************************************/
int main( void )
{
    prvSetupHardware();		/*  Configure hardware */
    
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTraceEnable(TRC_START); // Initialize and start recording
        str = xTraceRegisterString("Channel");
    #endif

    //mem_queue = xQueueCreate(5, 80*sizeof(char));

    xTaskCreate(heartbeat, "heartbeat", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(mem_write, "eeprom write", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    LATBSET = LEDA; //Indicate that you can begin writing a message.
    //xTaskCreate(mem_read, "eeprom read", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    
    // Create semaphore here. I trigger an exception if it is created anywhere else...
    mem_semaphore = xSemaphoreCreateBinary();
    cn_semaphore = xSemaphoreCreateBinary();
    
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
	mCNSetIntPriority(1);
	mCNSetIntSubPriority(0);
	unsigned int dummy = PORTReadBits(IOPORT_G, BTN1);
	mCNClearIntFlag();
	mCNIntEnable(1);    
    
    INTEnableSystemMultiVectoredInt();
}
/*-----------------------------------------------------------*/

static void heartbeat() {
    while(1) {
        LATBINV = LEDC;
        vTaskDelay(1000/portTICK_RATE_MS);
    }
}

void isr_uart() {
    //Very neat non-blocking function from comm.c
    if(getstrU1(input_str, sizeof(input_str))) {
        LATBCLR = LEDA; //Clear LEDA to indicate write process.
        putcU1('\n');
        xSemaphoreGiveFromISR(mem_semaphore, NULL);
        //xQueueSendToBack(mem_queue, input_str, 20);
        //LCD_puts(input_str);
    }
    mU1RXClearIntFlag();
}

void isr_cn() {
    
}

static void mem_write() {
    char inbox[80];
    
    int mem_addr = 0x0;
    while(1) {
        xSemaphoreTake(mem_semaphore, portMAX_DELAY);
        //Disable interrupts
        mU1RXIntEnable(0);
        mU1RXIntEnable(0);
        int err = I2CWriteEEPROM(SLAVE_ADDRESS, mem_addr, input_str, 80);
        LATBSET = LEDA; //Reenable LEDA when write is complete.
        MESSAGE_COUNT++;
        if(err != 0) LCD_puts("Error on EEPROM WRITE");
        err = I2CReadEEPROM(SLAVE_ADDRESS, mem_addr, inbox, 80);
        LCD_puts(inbox);
        mU1RXIntEnable(1);
        mU1RXIntEnable(1);
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