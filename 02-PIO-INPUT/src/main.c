/**************************************************************************
* Rafael Corsi   - Insper 
* rafael.corsi@insper.edu.br        
*
* Computação Embarcada
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
 * Botão
 */ 

#define BUTTON_PIO_ID		ID_PIOA
	

#define BUTTON_PIO         PIOA
	
#define BUTTON_PIN			64	
#define BUTTON_PIN_MASK	(1<<BUTTON_PIN)

#define BUT_DEBOUNCING_VALUE  79


/************************************************************************/
/* Prototipação                                                        */
/************************************************************************/
void ledConfig();
void butConfig();

/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void ledConfig(){
	//Ativa o clock do PIOA, PIOB, PIOC
	PMC->PMC_PCER0=(LED_PIO_ID);
	
	//define os pinos dos leds como pinos de saída
	LED_PIO->PIO_OER=(LED_PIN_MASK);
	
	//define o comando para os PIO's e não para outro periférico
	LED_PIO->PIO_PER=(LED_PIN_MASK);
	
	//define a saida em nível alto
	LED_PIO->PIO_CODR=(LED_PIN_MASK);
};

void butConfig(){

	//ativa o clock do PIOD
	PMC->PMC_PCER0=(1 << BUTTON_PIO_ID);

	
	//define o pino dos botões como entrada
	BUTTON_PIO->PIO_ODR=(BUTTON_PIN_MASK);
	
	//define o comando para os PIO's e não para outro periférico
	BUTTON_PIO->PIO_PER=(BUTTON_PIN_MASK);
	
	//ativa o Pull-up dos botões, ou seja conecta ao vcc e a um resistor
	BUTTON_PIO->PIO_PUER=(BUTTON_PIN_MASK);
	
	//ativa o debouncing
	BUTTON_PIO->PIO_IFER=(BUTTON_PIN_MASK);
	
	//ativa o clock periférico
	BUTTON_PIO->PIO_IFSCER=(BUTTON_PIN_MASK);
	
	//define a frequência do debouncing
	BUTTON_PIO->PIO_SCDR=BUT_DEBOUNCING_VALUE;
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{

	/************************************************************************/
	/* Inicialização básica do uC                                           */
	/************************************************************************/
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	/************************************************************************/
	/* Inicializa perifericos                                               */
	/************************************************************************/
	// Configura LED em modo saída
	ledConfig();

	// Configura botao
	butConfig();
	

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
		//checa se o botão está apertado
		if(BUTTON_PIO->PIO_PDSR & (BUTTON_PIN_MASK)){
			LED_PIO->PIO_SODR = LED_PIN_MASK;
		}
		else{
			LED_PIO->PIO_CODR = LED_PIN_MASK;
		}
		
	};
}


