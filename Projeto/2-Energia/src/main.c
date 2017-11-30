#include "asf.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/
#define SLEEP_TIME    1
#define ACTIVE_TIME   1



/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO       PIOC
#define LED_PIN		    13
#define LED_PIN_MASK  (1<<LED_PIN)

/**
 * Botão
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

/** 
 *  USART
 */
#define USART_COM     USART1
#define USART_COM_ID  ID_USART1

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
volatile bool g_ledBlinkOn = false;

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

//void BUT_init(void);
void LED_init(int estado);
//void TC1_init(void);
//void RTC_init(void);
//void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/


int count = 0;

void RTT_Handler(void)
{	
	
	rtt_get_status(RTT);
	if (count == 1){
		LED_init(1);
		count = 0;
	}else{
		LED_init(1);
		count = 0;
	}
}


/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/



/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
	
};

    

/**
 * \brief Configure UART console.
 */
static void USART1_init(void){
  
  /* Configura USART1 Pinos */
 sysclk_enable_peripheral_clock(ID_PIOB);
 sysclk_enable_peripheral_clock(ID_PIOA);
 pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4);  // RX
 pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
 MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;
  
  /* Configura opcoes USART */
  const sam_usart_opt_t usart_settings = {
    .baudrate     = 115200,
    .char_length  = US_MR_CHRL_8_BIT,
    .parity_type  = US_MR_PAR_NO,
    .stop_bits    = US_MR_NBSTOP_1_BIT    ,
    .channel_mode = US_MR_CHMODE_NORMAL
  };

  /* Ativa Clock periferico USART0 */
  sysclk_enable_peripheral_clock(USART_COM_ID); 
 	stdio_serial_init(CONF_UART, &usart_settings);

 }

/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){

	
	enum sleepmgr_mode current_sleep_mode = SLEEPMGR_ACTIVE;
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

  /* Configura Leds */
  LED_init(0);
  


 rtt_init(RTT, 32768);
 /* Enable RTT interrupt */
 NVIC_DisableIRQ(RTT_IRQn);
 NVIC_ClearPendingIRQ(RTT_IRQn);
 NVIC_SetPriority(RTT_IRQn, 0);
 NVIC_EnableIRQ(RTT_IRQn);
 rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
  pmc_disable_pllack();
  pmc_set_fast_startup_input(PMC_FSMR_RTTAL);
  #if (!(SAMG51 || SAMG53 || SAMG54))
	supc_set_wakeup_mode(SUPC, SUPC_WUMR_RTTEN_ENABLE);
	supc_backup_sram_off(SUPC);
	
  #endif
  sleepmgr_init();
  sleepmgr_lock_mode(current_sleep_mode);
  
	while (1) {
		
		rtt_write_alarm_time(RTT, rtt_read_timer_value(RTT) + SLEEP_TIME);
		
		sleepmgr_enter_sleep();
		
		sleepmgr_unlock_mode(current_sleep_mode);
		
	}
	
}
