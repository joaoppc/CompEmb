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
 * Motor
 */
#define IN1_PIO_ID		ID_PIOD
#define IN2_PIO_ID		ID_PIOB
#define IN3_PIO_ID		ID_PIOA
#define IN4_PIO_ID		ID_PIOD

#define IN1_PIO         PIOD
#define IN2_PIO         PIOB
#define IN3_PIO         PIOA
#define IN4_PIO         PIOD

#define IN1_PIN			25
#define IN2_PIN			0
#define IN3_PIN			3
#define IN4_PIN			28

#define IN1_PIN_MASK	(1<<IN1_PIN)
#define IN2_PIN_MASK	(1<<IN2_PIN)
#define IN3_PIN_MASK	(1<<IN3_PIN)
#define IN4_PIN_MASK	(1<<IN4_PIN) 

#define DELAY		    10

/**
 * Sensor de Presença
 */ 

#define SENSOR_PIO_ID		ID_PIOA
	

#define SENSOR_PIO         PIOA
	
#define SENSOR_PIN			0	
#define SENSOR_PIN_MASK	(1<<SENSOR_PIN)

//#define SENS_DEBOUNCING_VALUE  79


/************************************************************************/
/* Prototipação                                                        */
/************************************************************************/
void ledConfig();
void motorConfig();
void sensConfig();


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
}

/**
 * @Brief Inicializa o pino do LED
 */
void motorConfig(){
	//Ativa o clock do PIOA, PIOB, PIOD
	PMC->PMC_PCER0=(IN1_PIO_ID);
	PMC->PMC_PCER0=(IN2_PIO_ID);
	PMC->PMC_PCER0=(IN3_PIO_ID);
	
	//define os pinos dos Indutores como pinos de saída
	IN1_PIO->PIO_OER=(IN1_PIN_MASK);
	IN2_PIO->PIO_OER=(IN2_PIN_MASK);
	IN3_PIO->PIO_OER=(IN3_PIN_MASK);
	IN4_PIO->PIO_OER=(IN4_PIN_MASK);
	
	//define o comando para os PIO's e não para outro periférico
	IN1_PIO->PIO_PER=(IN1_PIN_MASK);
	IN2_PIO->PIO_PER=(IN2_PIN_MASK);
	IN3_PIO->PIO_PER=(IN3_PIN_MASK);
	IN4_PIO->PIO_PER=(IN4_PIN_MASK);
	
	//define a saida em nível baixo
	IN1_PIO->PIO_CODR=(IN1_PIN_MASK);
	IN2_PIO->PIO_CODR=(IN2_PIN_MASK);
	IN3_PIO->PIO_CODR=(IN3_PIN_MASK);
	IN4_PIO->PIO_CODR=(IN4_PIN_MASK);
}

void sensConfig(){

	//ativa o clock do PIOA
	PMC->PMC_PCER0=(1 << SENSOR_PIO_ID);

	
	//define o pino dos botões como entrada
	SENSOR_PIO->PIO_ODR=(SENSOR_PIN_MASK);
	
	//define o comando para os PIO's e não para outro periférico
	SENSOR_PIO->PIO_PER=(SENSOR_PIN_MASK);
	
	//ativa o Pull-up dos sensor, ou seja conecta ao vcc e a um resistor
	SENSOR_PIO->PIO_PUER=(SENSOR_PIN_MASK);
	
	//ativa o debouncing
	//BUTTON_PIO->PIO_IFER=(BUTTON_PIN_MASK);
	
	//ativa o clock periférico
	//BUTTON_PIO->PIO_IFSCER=(BUTTON_PIN_MASK);
	
	//define a frequência do debouncing
	//BUTTON_PIO->PIO_SCDR=BUT_DEBOUNCING_VALUE;
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
	
	//Configura o LED
	ledConfig();
	
	// Configura motor em modo saída
	motorConfig();

	// Configura sensor
	sensConfig();
	

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
		
		//IN4_PIO->PIO_CODR = IN4_PIN_MASK;

		//IN1_PIO->PIO_SODR = IN1_PIN_MASK;
		
		//delay_ms(DELAY);
		
		//IN1_PIO->PIO_CODR = IN1_PIN_MASK;
		
		//IN2_PIO->PIO_SODR = IN2_PIN_MASK;
		
		//delay_ms(DELAY);
		
		//IN2_PIO->PIO_CODR = IN2_PIN_MASK;
		
		//IN3_PIO->PIO_SODR = IN3_PIN_MASK;
		
		//delay_ms(DELAY);
		
		//IN3_PIO->PIO_CODR = IN3_PIN_MASK;
		
		//IN4_PIO->PIO_SODR = IN4_PIN_MASK;
		
		//delay_ms(DELAY);
		
		
	
		//checa se há presença
		if(SENSOR_PIO->PIO_PDSR & (SENSOR_PIN_MASK)){
			LED_PIO->PIO_SODR = LED_PIN_MASK;
			for (int i = 0;i<=64;i++){
				IN4_PIO->PIO_CODR = IN4_PIN_MASK;

				IN1_PIO->PIO_SODR = IN1_PIN_MASK;
				
				delay_ms(DELAY);
				
				IN1_PIO->PIO_CODR = IN1_PIN_MASK;
				
				IN2_PIO->PIO_SODR = IN2_PIN_MASK;
				
				delay_ms(DELAY);
				
				IN2_PIO->PIO_CODR = IN2_PIN_MASK;
				
				IN3_PIO->PIO_SODR = IN3_PIN_MASK;
				
				delay_ms(DELAY);
				
				IN3_PIO->PIO_CODR = IN3_PIN_MASK;
				
				IN4_PIO->PIO_SODR = IN4_PIN_MASK;
				
				delay_ms(DELAY);
			}
		}
		else{
			LED_PIO->PIO_CODR = LED_PIN_MASK;
		}
		
	
		
	}
}

