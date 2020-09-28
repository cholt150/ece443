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


#define DEBOUNCE_TIME 20/portTICK_RATE_MS
#define SLAVE_ADDRESS 0x50
/*-----------------------------------------------------------*/

/* Global Variables */
static char input_str[80];
int MESSAGE_COUNT = 0;
int MEM_ADDRESS[5] = {0,80,160,240,320};

xSemaphoreHandle mem_semaphore;
xSemaphoreHandle cn_semaphore;
xQueueHandle addr_queue;
xQueueHandle btn_queue;

/* Function Prototypes */
static void prettyPrint(char* input);
static void prvSetupHardware( void );
void isr_uart();

/* Task Prototypes  */
static void heartbeat();
static void mem_write();
static void mem_read();
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

    addr_queue = xQueueCreate(5, sizeof(int));
    btn_queue = xQueueCreate(5, sizeof(int));

    xTaskCreate(heartbeat, "heartbeat", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(mem_write, "eeprom write", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(mem_read, "eeprom read", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    
    LATBSET = LEDA; //Indicate that you can begin writing a message.
    
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
	mCNSetIntPriority(2);
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
    }
    mU1RXClearIntFlag();
}

void isr_cn() {
    int send = 1;
    xSemaphoreGiveFromISR(cn_semaphore, NULL);
//    xQueueSendToBack(btn_queue, &send, 20);
    mCNIntEnable(0);
    mCNClearIntFlag();
}

static void mem_write() {
    char inbox[80];
    while(1) {
        xSemaphoreTake(mem_semaphore, portMAX_DELAY);
        //Disable interrupts
        mU1RXIntEnable(0);
        mCNIntEnable(0);
        
        //Write to the EEPROM
        int err = I2CWriteEEPROM(SLAVE_ADDRESS, MEM_ADDRESS[MESSAGE_COUNT], input_str, 80);
        
        //Enqueue the address
        xQueueSendToBack(addr_queue, &MEM_ADDRESS[MESSAGE_COUNT], 20);
        LATBSET = LEDA; //Reenable LEDA when write is complete.
        MESSAGE_COUNT++;
        if(err != 0) LCD_puts("Error on EEPROM WRITE");
        mU1RXIntEnable(1);
        mCNIntEnable(1);
    }
}

static void mem_read() {
    char inbox[80];
    int read_address;
    int dummy;
    while(1) {
        xSemaphoreTake(cn_semaphore, portMAX_DELAY);
//        xQueueReceive(btn_queue, &dummy, portMAX_DELAY);
        if(MESSAGE_COUNT > 0) {
            vTaskDelay(DEBOUNCE_TIME);
            while((PORTG & BTN1) != 0);
            vTaskDelay(DEBOUNCE_TIME);
            xQueueReceive(addr_queue, &read_address, 20);
            int err = I2CReadEEPROM(SLAVE_ADDRESS, read_address, inbox, 80);
            LCD_puts(inbox);
            MESSAGE_COUNT--;
        }
        mCNClearIntFlag();
        mCNIntEnable(1);
    }
}

static void prettyPrint(char *input) {
    char line0[16];
    char line1[16];
    int start = 0;
    int end = 0;
    
    while(input[end] != ' ') {
        
        end++;
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
    if(MESSAGE_COUNT == 0) LATBSET = LEDB;
    else LATBCLR = LEDB;
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