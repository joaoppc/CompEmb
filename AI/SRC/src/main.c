/**
 *    Computacao Embarcada - Computacao - Insper
 *
 *            Avaliacao Intermediaria
 *
 * Faça um firmware que permita a um usuário no computador acessar e configurar algumas
 * informações/ modos  de operação do microcontrolador. Essas funcionalidades devem ser
 * acessadas via comunicação serial (COM). Um menu deve informar ao usuário as possibilidades
 * e os comandos que devem ser digitados para operar o embarcado.
 *
 * Funcionalidades que o firmware deve possuir :
 *
 * 1. Exibir menu
 * 1. O usuário deve ser capaz de ligar/desligar o piscar led (led da placa)
 * 1. O usuário deve ser capaz de aumentar(+2 Hz) e diminuir (-2 Hz) a frequência do led
 *
 * Utilize o programa disponível no repositório (github.com/insper/Computacao-Embarcada/Avaliacoes/A1/)
 * como ponto de parida. O código deve fazer uso de interrupções e periféricos para gerenciar a
 * comunicação com o PC e o LED.
 *
 *  ## Extra (A)
 *
 *  1. O usuário deve ser capaz de ler o relógio do microcontrolador.
 *  1. O usuário deve ser capaz de entrar com um valor de frequência para o led de forma numérica no termina.
 *
 */

/************************************************************************/
/* Includes                                                              */
/************************************************************************/

#include "asf.h"
#include <string.h>

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/
/**
 *  USART
 */
#define USART_COM     USART1
#define USART_COM_ID  ID_USART1

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		    8
#define LED_PIN_MASK    (1<<LED_PIN)

/************************************************************************/
/* Variaveis globais                                                          */
/************************************************************************/
uint8_t buffer[5];
/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/
void led_init(int estado);
uint32_t usart_puts(uint8_t *pstring);
uint32_t usart_gets(uint8_t *pstring);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/
//void but_Handler(){
    /*
     *  limpa interrupcao do PIO
     */
   // uint32_t pioIntStatus;
    //pioIntStatus =  pio_get_interrupt_status(BUT_PIO);
    
   /**
    *  Toggle status led
    */
   //if(pio_get_output_data_status(LED_PIO, LED_PIN_MASK))
   // pio_clear(LED_PIO, LED_PIN_MASK);
   //else
   // pio_set(LED_PIO,LED_PIN_MASK);
    
//}
/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/**
 * \brief Configure USART peripheral
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

  /* Configura USART para operar em modo RS232 */
  usart_init_rs232(USART_COM, &usart_settings, sysclk_get_peripheral_hz());

  /* Enable the receiver and transmitter. */
	usart_enable_tx(USART_COM);
	usart_enable_rx(USART_COM);
}


/**
 *  Envia para o UART uma string
 */
uint32_t usart_puts(uint8_t *pstring){
  uint32_t i = 0 ;

  while(*(pstring + i)){
    usart_serial_putchar(USART_COM, *(pstring+i++));
    while(!uart_is_tx_empty(USART_COM)){};
  }
  return(i);
}

/**
 * Busca do UART uma mensagem enviada pelo computador terminada em \n
 */
uint32_t usart_gets(uint8_t *pstring){
  uint32_t i = 0 ;
  usart_serial_getchar(USART_COM, (pstring+i));
  while(*(pstring+i) != '\n'){
    usart_serial_getchar(USART_COM, (pstring+(++i)));
  }
  *(pstring+i+1)= 0x00;
  return(i);

}

void led_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, 1, 0, 0 );
};

/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

  /** Inicializa USART */
  USART1_init();
  
  //led_init(1);
  PMC->PMC_PCER0 = (1<<LED_PIO_ID);
  PIOC->PIO_OER = (1 << 8);
  PIOC->PIO_PER = (1 << 8);
  PIOC->PIO_CODR =  (1 << 8);

  /** Super loop */
	while (1) {
    usart_puts("Menu .... \n");
	usart_gets(buffer);
	usart_puts(buffer);
	if(buffer[0]=='m'){
		PIOC->PIO_SODR = (1 << 8);
	}
	else if(buffer[0]=='M'){
		PIOC->PIO_CODR = (1<<8);
	}
    delay_s(1);
	}
}
