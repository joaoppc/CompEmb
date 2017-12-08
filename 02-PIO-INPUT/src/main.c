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
#define LED_PIO_ID            ID_PIOA
#define LED_PIO               PIOA
#define LED_PIN              0
#define LED_PIN_MASK          (1<<LED_PIN)




#define LED2_PIO_ID            ID_PIOC
#define LED2_PIO               PIOC
#define LED2_PIN               30
#define LED2_PIN_MASK          (1<<LED2_PIN)





#define LED3_PIO_ID             ID_PIOB
#define LED3_PIO               PIOB
#define LED3_PIN               2
#define LED3_PIN_MASK          (1<<LED3_PIN)
/**
 * Botão
 */
#define BUT_PIO_ID        ID_PIOD
#define BUT_PIO           PIOD
#define BUT_PIN       28
#define BUT_PIN_MASK        (1 << BUT_PIN)


#define BUT2_PIO_ID        ID_PIOC
#define BUT2_PIO           PIOC
#define BUT2_PIN       31
#define BUT2_PIN_MASK        (1 << BUT2_PIN)


#define BUT3_PIO_ID        ID_PIOA
#define BUT3_PIO          PIOA
#define BUT3_PIN          19
#define BUT3_PIN_MASK        (1 << BUT3_PIN)



#define DEBOUNCING_VALUE  79

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
	PMC->PMC_PCER0=(LED2_PIO_ID);
	PMC->PMC_PCER0=(LED3_PIO_ID);
	
	//define os pinos dos leds como pinos de saída
	LED_PIO->PIO_OER=(LED_PIN_MASK);
	LED_PIO->PIO_OER=(LED2_PIN_MASK);
	LED_PIO->PIO_OER=(LED3_PIN_MASK);
	
	//define o comando para os PIO's e não para outro periférico
	LED_PIO->PIO_PER=(LED_PIN_MASK);
	LED_PIO->PIO_PER=(LED2_PIN_MASK);
	LED_PIO->PIO_PER=(LED3_PIN_MASK);
	
	//define a saida em nível alto
	LED_PIO->PIO_CODR=(LED_PIN_MASK);
	LED_PIO->PIO_CODR=(LED2_PIN_MASK);
	LED_PIO->PIO_CODR=(LED3_PIN_MASK);
};

void butConfig(){

	//ativa o clock do PIOD
	PMC->PMC_PCER0=(1 << BUT_PIO_ID);
	PMC->PMC_PCER0=(1 << BUT2_PIO_ID);
	PMC->PMC_PCER0=(1 << BUT3_PIO_ID);

	
	//define o pino dos botões como entrada
	BUT_PIO->PIO_ODR=(BUT_PIN_MASK);
	BUT2_PIO->PIO_ODR=(BUT2_PIN_MASK);
	BUT3_PIO->PIO_ODR=(BUT3_PIN_MASK);
	
	//define o comando para os PIO's e não para outro periférico
	BUT_PIO->PIO_PER=(BUT_PIN_MASK);
	BUT2_PIO->PIO_PER=(BUT2_PIN_MASK);
	BUT3_PIO->PIO_PER=(BUT3_PIN_MASK);
	
	//ativa o Pull-up dos botões, ou seja conecta ao vcc e a um resistor
	BUT_PIO->PIO_PUER=(BUT_PIN_MASK);
	BUT2_PIO->PIO_PUER=(BUT2_PIN_MASK);
	BUT3_PIO->PIO_PUER=(BUT3_PIN_MASK);
	
	//ativa o debouncing
	BUT_PIO->PIO_IFER=(BUT_PIN_MASK);
	BUT2_PIO->PIO_IFER=(BUT2_PIN_MASK);
	BUT3_PIO->PIO_IFER=(BUT3_PIN_MASK);
	
	//ativa o clock periférico
	BUT_PIO->PIO_IFSCER=(BUT_PIN_MASK);
	BUT2_PIO->PIO_IFSCER=(BUT2_PIN_MASK);
	BUT3_PIO->PIO_IFSCER=(BUT3_PIN_MASK);
	
	//define a frequência do debouncing
	BUT_PIO->PIO_SCDR=DEBOUNCING_VALUE;
	BUT2_PIO->PIO_SCDR=DEBOUNCING_VALUE;
	BUT3_PIO->PIO_SCDR=DEBOUNCING_VALUE;
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
		if(BUT_PIO->PIO_PDSR & (BUT_PIN_MASK)){
			LED_PIO->PIO_SODR = LED_PIN_MASK;
		}
		else{
			LED_PIO->PIO_CODR = LED_PIN_MASK;
		}
		if(BUT2_PIO->PIO_PDSR & (BUT2_PIN_MASK)){
			LED2_PIO->PIO_SODR = LED2_PIN_MASK;
		}
		else{
			LED2_PIO->PIO_CODR = LED2_PIN_MASK;
		}
		if(BUT3_PIO->PIO_PDSR & (BUT3_PIN_MASK)){
			LED3_PIO->PIO_SODR = LED3_PIN_MASK;
		}
		else{
			LED3_PIO->PIO_CODR = LED3_PIN_MASK;
		}
		
	};
}


