#include "asf.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
#define YEAR        2017
#define MONTH      3
#define DAY         27
#define WEEK        13
#define HOUR        9
#define MINUTE      5
#define SECOND      0

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO       PIOC
#define LED_PIN		    13
#define LED_PIN_MASK  (1<<LED_PIN)

/**
 * Bot�o
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

void BUT_init(void);
void LED_init(int estado);
void TC1_init(void);
void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
 *  Handle Interrupcao botao 1
 */
static void Button1_Handler(uint32_t id, uint32_t mask)
{
	
}

/**
 *  Interrupt handler for TC1 interrupt. 
 */
void TC1_Handler(void){
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
  pin_toggle(LED_PIO, LED_PIN_MASK);
    
 }

/**
 * \brief Interrupt handler for the RTC. Refresh the display.
 */
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* Second increment interrupt */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
	
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);

	} else {
		/* Time or date alarm */
		if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
      
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
		}
	}
}


/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/** 
 *  Toggle pin controlado pelo PIO (out)
 */
void pin_toggle(Pio *pio, uint32_t mask){
   if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
   else
    pio_set(pio,mask);
}

/**
 * @Brief Inicializa o pino do BUT
 */
void BUT_init(void){
    /* config. pino botao em modo de entrada */
    pmc_enable_periph_clk(BUT_PIO_ID);
    pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
    /* config. interrupcao em borda de descida no botao do kit */
    /* indica funcao (but_Handler) a ser chamada quando houver uma interrup��o */
    pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
    pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
    
    /* habilita interrup�c�o do PIO que controla o botao */
    /* e configura sua prioridade                        */
    NVIC_EnableIRQ(BUT_PIO_ID);
    NVIC_SetPriority(BUT_PIO_ID, 1);
};

/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};

/**
 * Configura TimerCounter (TC0) para gerar uma interrupcao no canal 0-(ID_TC1) 
 * a cada 250 ms (4Hz)
 */
void TC1_init(void){   
    uint32_t ul_div;
    uint32_t ul_tcclks;
    uint32_t ul_sysclk = sysclk_get_cpu_hz();
    
    uint32_t channel = 1;
    
    /* Configura o PMC */
    pmc_enable_periph_clk(ID_TC1);    

    /** Configura o TC para operar em  4Mhz e interrup�c�o no RC compare */
    tc_find_mck_divisor(4, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
    tc_init(TC0, channel, ul_tcclks | TC_CMR_CPCTRG);
    tc_write_rc(TC0, channel, (ul_sysclk / ul_div) / 4);

    /* Configura e ativa interrup�c�o no TC canal 0 */
    NVIC_EnableIRQ((IRQn_Type) ID_TC1);
    tc_enable_interrupt(TC0, channel, TC_IER_CPCS);

    /* Inicializa o canal 0 do TC */
    tc_start(TC0, channel);
}

/**
 * Configura o RTC para funcionar com interrupcao de alarme
 */
void RTC_init(){
    /* Configura o PMC */
    pmc_enable_periph_clk(ID_RTC);
        
    /* Default RTC configuration, 24-hour mode */
    rtc_set_hour_mode(RTC, 0);

    /* Configura data e hora manualmente */
    rtc_set_date(RTC, YEAR, MONTH, DAY, WEEK);
    rtc_set_time(RTC, HOUR, MINUTE, SECOND);

    /* Configure RTC interrupts */
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 0);
    NVIC_EnableIRQ(RTC_IRQn);
    
    /* Ativa interrupcao via alarme */
    rtc_enable_interrupt(RTC,  RTC_IER_ALREN); 
    
}

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
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

  /* Configura Leds */
  LED_init(0);
	
	/* Configura os bot�es */
	BUT_init();    
    
  /** Configura timer 0 */
  TC1_init();
    
  /** Configura RTC */
  RTC_init();
  
  /** Inicializa USART como printf */
  USART1_init();
  
	while (1) {
	}

}
