#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

/* Pinos do HC */
#define TRG_PIO			  PIOA
#define TRG_PIO_ID		  ID_PIOA
#define TRG_PIO_PIN		  6
#define TRG_PIO_PIN_MASK  (1 << TRG_PIO_PIN)

#define ECHO_PIO		  PIOD
#define ECHO_PIO_ID		  ID_PIOD
#define ECHO_PIO_PIN	  30
#define ECHO_PIO_PIN_MASK (1 << ECHO_PIO_PIN)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/* Timer for update reading */
TimerHandle_t xTimer;

/* Queue for distance data */
QueueHandle_t xQueueHC;

/* RTT variables */
#define RTT_TICKS        17000

/* Other variables */
char str[128];

/* prototypes */
void echo_callback(void);
static void HC_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void echo_callback(void) {
	if(pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK)){
		RTT_init(RTT_TICKS,0,0);
		}else{
		int dist = rtt_read_timer_value(RTT);
		BaseType_t xHPTwoken = pdTRUE;
		xQueueSendFromISR(xQueueHC, &dist, &xHPTwoken);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void vTimerCallback(TimerHandle_t xTimer) {
	/* Inits and resets TRIGGER */
	pio_set(TRG_PIO, TRG_PIO_PIN_MASK);
	delay_us(10);
	pio_clear(TRG_PIO, TRG_PIO_PIN_MASK);
}

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	gfx_mono_draw_vertical_line(35,0,32, GFX_PIXEL_SET);
	gfx_mono_draw_horizontal_line(35, 31, 96, GFX_PIXEL_SET);
	int dist_;
	int dists[60] = {0};
	xTimer = xTimerCreate(/* Just a text name, not used by the RTOS
							kernel. */
							"Timer",
							/* The timer period in ticks, must be
							greater than 0. */
							400,
							/* The timers will auto-reload themselves
							when they expire. */
							pdTRUE,
							/* The ID is used to store a count of the
							number of times the timer has expired, which
							is initialised to 0. */
							(void *)0,
							/* Timer callback */
							vTimerCallback);
	xTimerStart(xTimer, 0);
	
	for (;;)  {
		// Handle semaphore and queue
		BaseType_t t = pdTRUE;
		if(xQueueReceive(xQueueHC, &(dist_), &t)){
			sprintf(str, "%d cm\n", dist_);
			printf(str);
			gfx_mono_draw_string(str, 0, 0, &sysfont);
			for(int n = 0; n<59; n++){
				dists[n]=dists[n+1];
			}
			dists[59] = dist_;
			gfx_mono_draw_filled_rect(36,0,128-36,30,GFX_PIXEL_CLR);
			for(int n = 0; n<60; n++){
				int y = 32-dists[n]/2;
				if(y<0){
					y=0;
				}
				gfx_mono_draw_pixel(35+n,y,GFX_PIXEL_SET);
			}
		}
	}
	
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void HC_init(void) {

	/* conf echo como entrada */
	pmc_enable_periph_clk(ECHO_PIO_ID);
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK, PIO_DEFAULT);
	pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_PIO_PIN_MASK, PIO_IT_EDGE , echo_callback);
	
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_PIN_MASK);
	pio_get_interrupt_status(ECHO_PIO);
	
	/* conf Trigger como saída */
	pmc_enable_periph_clk(TRG_PIO_ID);
	pio_configure(TRG_PIO, PIO_OUTPUT_0, TRG_PIO_PIN_MASK, PIO_DEFAULT);
	
	/* configura prioridae do ECHO */
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4);
}

/**
* Configura RTT
*
* arg0 pllPreScale  : Frequência na qual o contador irá incrementar
* arg1 IrqNPulses   : Valor do alarme
* arg2 rttIRQSource : Pode ser uma
*     - 0:
*     - RTT_MR_RTTINCIEN: Interrupção por incremento (pllPreScale)
*     - RTT_MR_ALMIEN : Interrupção por alarme
*/
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}



/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	delay_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	/* Initialize the console uart */
	configure_console();
	
	xQueueHC = xQueueCreate(100, sizeof(int));
	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}

	
	HC_init();
	

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
