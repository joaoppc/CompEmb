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
#define LED_M_S_PIO_ID		ID_PIOA //manha sim
#define LED_M_NC_PIO_ID		ID_PIOC //manha nao comeu
#define LED_M_N_PIO_ID		ID_PIOD //manha não
#define LED_T_S_PIO_ID		ID_PIOD //tarde sim
#define LED_T_NC_PIO_ID		ID_PIOA //tarde nao comeu
#define LED_T_N_PIO_ID		ID_PIOA //tarde nao


#define LED_M_S_PIO         PIOA
#define LED_M_NC_PIO        PIOC
#define LED_M_N_PIO         PIOD
#define LED_T_S_PIO         PIOD
#define LED_T_NC_PIO        PIOA
#define LED_T_N_PIO         PIOA

#define LED_M_S_PIN			21
#define LED_M_NC_PIN		13
#define LED_M_N_PIN			11
#define LED_T_S_PIN			26
#define LED_T_NC_PIN		24
#define LED_T_N_PIN			4

#define LED_M_S_PIN_MASK	(1<<LED_M_S_PIN)
#define LED_M_NC_PIN_MASK	(1<<LED_M_NC_PIN)
#define LED_M_N_PIN_MASK	(1<<LED_M_N_PIN)
#define LED_T_S_PIN_MASK	(1<<LED_T_S_PIN)
#define LED_T_NC_PIN_MASK	(1<<LED_T_NC_PIN)
#define LED_T_N_PIN_MASK	(1<<LED_T_N_PIN) 
/**
 * Motor
 */
#define IN1_PIO_ID		ID_PIOD
#define IN2_PIO_ID		ID_PIOA
#define IN3_PIO_ID		ID_PIOC
#define IN4_PIO_ID		ID_PIOA

#define IN1_PIO         PIOD
#define IN2_PIO         PIOA
#define IN3_PIO         PIOC
#define IN4_PIO         PIOA

#define IN1_PIN			30
#define IN2_PIN			6
#define IN3_PIN			19
#define IN4_PIN			2

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
	
#define SENSOR_PIN			3	

#define SENSOR_PIN_MASK	(1<<SENSOR_PIN)


/**
 * Infra-Vermelho
 */ 

#define INFRA_PIO_ID		ID_PIOB
	

#define INFRA_PIO         PIOB
	
#define INFRA_PIN			4	
#define INFRA_PIN_MASK	(1<<INFRA_PIN)

//#define SENS_DEBOUNCING_VALUE  79

#define YEAR        2017
#define MONTH       12
#define DAY         6
#define WEEK        13
#define HOUR        9
#define MINUTE      5
#define SECOND      0

int servido = 0;



/************************************************************************/
/* Prototipação                                                        */
/************************************************************************/
void ledConfig();
void motorConfig();
void sensConfig();
void infraConfig();


/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void ledConfig(){
	//Ativa o clock do PIOA, PIOC, PIOD
	PMC->PMC_PCER0=(LED_M_S_PIO_ID);
	PMC->PMC_PCER0=(LED_M_NC_PIO_ID);
	PMC->PMC_PCER0=(LED_M_N_PIO_ID);
	
	
	
	//define os pinos dos leds como pinos de saída
	LED_M_S_PIO->PIO_OER=(LED_M_S_PIN_MASK);
	LED_M_NC_PIO->PIO_OER=(LED_M_NC_PIN_MASK);
	LED_M_N_PIO->PIO_OER=(LED_M_N_PIN_MASK);
	LED_T_S_PIO->PIO_OER=(LED_T_S_PIN_MASK);
	LED_T_NC_PIO->PIO_OER=(LED_T_NC_PIN_MASK);
	LED_T_N_PIO->PIO_OER=(LED_T_N_PIN_MASK);
	
	//define o comando para os PIO's e não para outro periférico
	LED_M_S_PIO->PIO_PER=(LED_M_S_PIN_MASK);
	LED_M_NC_PIO->PIO_PER=(LED_M_NC_PIN_MASK);
	LED_M_N_PIO->PIO_PER=(LED_M_N_PIN_MASK);
	LED_T_S_PIO->PIO_PER=(LED_T_S_PIN_MASK);
	LED_T_NC_PIO->PIO_PER=(LED_T_NC_PIN_MASK);
	LED_T_N_PIO->PIO_PER=(LED_T_N_PIN_MASK);
	
	//define a saida em nível alto
	LED_M_S_PIO->PIO_CODR=(LED_M_S_PIN_MASK);
	LED_M_NC_PIO->PIO_CODR=(LED_M_NC_PIN_MASK);
	LED_M_N_PIO->PIO_CODR=(LED_M_N_PIN_MASK);
	LED_T_S_PIO->PIO_CODR=(LED_T_S_PIN_MASK);
	LED_T_NC_PIO->PIO_CODR=(LED_T_NC_PIN_MASK);
	LED_T_N_PIO->PIO_CODR=(LED_T_N_PIN_MASK);
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

void infraConfig(){
	PMC->PMC_PCER0 = (1<<INFRA_PIO_ID);
	
	//define o pino dos botões como entrada
	INFRA_PIO->PIO_ODR=(INFRA_PIN_MASK);
	
	//define o comando para os PIO's e não para outro periférico
	INFRA_PIO->PIO_PER=(INFRA_PIN_MASK);
	
	//ativa o Pull-up dos sensor, ou seja conecta ao vcc e a um resistor
	INFRA_PIO->PIO_PUER=(INFRA_PIN_MASK);
	
	
}

/************************************************************************/
/* Call backs / Handler                                                 */
/************************************************************************/

void Motor_Handler(void){
	
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
	
	//Configura o infra-vermelho
	infraConfig();
	

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
		/**/if(SENSOR_PIO->PIO_PDSR & (SENSOR_PIN_MASK)){
			LED_M_S_PIO->PIO_SODR=(LED_M_S_PIN_MASK);
			LED_M_NC_PIO->PIO_SODR=(LED_M_NC_PIN_MASK);
			LED_M_N_PIO->PIO_SODR=(LED_M_N_PIN_MASK);
			LED_T_S_PIO->PIO_SODR=(LED_T_S_PIN_MASK);
			LED_T_NC_PIO->PIO_SODR=(LED_T_NC_PIN_MASK);
			LED_T_N_PIO->PIO_SODR=(LED_T_N_PIN_MASK);
			for (int i = 0;i<=64;i++){
				IN4_PIO->PIO_CODR = IN4_PIN_MASK;

				IN1_PIO->PIO_SODR = IN1_PIN_MASK;
				
				delay_ms(DELAY);
				
				IN1_PIO->PIO_CODR = IN1_PIN_MASK;
				
				delay_ms(DELAY);
				
				IN2_PIO->PIO_SODR = IN2_PIN_MASK;
				
				delay_ms(DELAY);
				
				IN2_PIO->PIO_CODR = IN2_PIN_MASK;
				
				delay_ms(DELAY);
				
				IN3_PIO->PIO_SODR = IN3_PIN_MASK;
				
				delay_ms(DELAY);
				
				IN3_PIO->PIO_CODR = IN3_PIN_MASK;
				
				delay_ms(DELAY);
				
				IN4_PIO->PIO_SODR = IN4_PIN_MASK;
				
				delay_ms(DELAY);
			}
		}
		else{
			LED_M_S_PIO->PIO_CODR=(LED_M_S_PIN_MASK);
			LED_M_NC_PIO->PIO_CODR=(LED_M_NC_PIN_MASK);
			LED_M_N_PIO->PIO_CODR=(LED_M_N_PIN_MASK);
			LED_T_S_PIO->PIO_CODR=(LED_T_S_PIN_MASK);
			LED_T_NC_PIO->PIO_CODR=(LED_T_NC_PIN_MASK);
			LED_T_N_PIO->PIO_CODR=(LED_T_N_PIN_MASK);
		}/**/
		
	
		
	}
}

