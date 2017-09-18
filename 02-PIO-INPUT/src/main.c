/**************************************************************************
* Rafael Corsi   - Insper 
* rafael.corsi@insper.edu.br        
*
* Computa��o Embarcada
*
* 08-PIO-ENTRADA
************************************************************************/


#include "asf.h"
#include "conf_clock.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC

#define LED_PIO         PIOC

#define LED_PIN			8

#define LED_PIN_MASK	(1<<LED_PIN) 



/**
 * Bot�o
 */ 

#define BUTTON_PIO_ID		ID_PIOA
	

#define BUTTON_PIO         PIOA
	
#define BUTTON_PIN			64	
#define BUTTON_PIN_MASK	(1<<BUTTON_PIN)

#define BUT_DEBOUNCING_VALUE  79


/************************************************************************/
/* Prototipa��o                                                        */
/************************************************************************/
void ledConfig();
void butConfig();

/************************************************************************/
/* Fun��es	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void ledConfig(){
	//Ativa o clock do PIOA, PIOB, PIOC
	PMC->PMC_PCER0=(LED_PIO_ID);
	
	//define os pinos dos leds como pinos de sa�da
	LED_PIO->PIO_OER=(LED_PIN_MASK);
	
	//define o comando para os PIO's e n�o para outro perif�rico
	LED_PIO->PIO_PER=(LED_PIN_MASK);
	
	//define a saida em n�vel alto
	LED_PIO->PIO_CODR=(LED_PIN_MASK);
};

void butConfig(){

	//ativa o clock do PIOD
	PMC->PMC_PCER0=(1 << BUTTON_PIO_ID);

	
	//define o pino dos bot�es como entrada
	BUTTON_PIO->PIO_ODR=(BUTTON_PIN_MASK);
	
	//define o comando para os PIO's e n�o para outro perif�rico
	BUTTON_PIO->PIO_PER=(BUTTON_PIN_MASK);
	
	//ativa o Pull-up dos bot�es, ou seja conecta ao vcc e a um resistor
	BUTTON_PIO->PIO_PUER=(BUTTON_PIN_MASK);
	
	//ativa o debouncing
	BUTTON_PIO->PIO_IFER=(BUTTON_PIN_MASK);
	
	//ativa o clock perif�rico
	BUTTON_PIO->PIO_IFSCER=(BUTTON_PIN_MASK);
	
	//define a frequ�ncia do debouncing
	BUTTON_PIO->PIO_SCDR=BUT_DEBOUNCING_VALUE;
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{

	/************************************************************************/
	/* Inicializa��o b�sica do uC                                           */
	/************************************************************************/
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	/************************************************************************/
	/* Inicializa perifericos                                               */
	/************************************************************************/
	// Configura LED em modo sa�da
	ledConfig();

	// Configura botao
	butConfig();
	

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
		//checa se o bot�o est� apertado
		if(BUTTON_PIO->PIO_PDSR & (BUTTON_PIN_MASK)){
			LED_PIO->PIO_SODR = LED_PIN_MASK;
		}
		else{
			LED_PIO->PIO_CODR = LED_PIN_MASK;
		}
		
	};
}


