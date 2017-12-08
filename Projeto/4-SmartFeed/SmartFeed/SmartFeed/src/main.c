#include "asf.h"
#include "conf_clock.h"
#include "main.h"
#include <string.h>
#include "bsp/include/nm_bsp.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 weather client example --"STRING_EOL	\
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL
	
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
#define HOUR1        6
#define HOUR2        18
#define MINUTE      5
#define SECOND      0

int alimentar_m = 0;
int alimentar_t = 0;
int comer_m = 0;
int comer_t = 0;
int manha = 1;
int tarde = 0;
int motor = 0;

/** IP address of host. */
uint32_t gu32HostIp = 0;

/** TCP client socket handlers. */
static SOCKET tcp_client_socket = -1;

/** Receive buffer definition. */
static uint8_t gau8ReceivedBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};

/** Wi-Fi status variable. */
static bool gbConnectedWifi = false;

/** Get host IP status variable. */
/** Wi-Fi connection state */
static uint8_t wifi_connected;


/** Instance of HTTP client module. */
static bool gbHostIpByName = false;

/** TCP Connection status variable. */
static bool gbTcpConnection = false;

/** Server host name. */
static char server_host_name[] = MAIN_SERVER_NAME;

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
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}


/* 
 * Check whether "cp" is a valid ascii representation
 * of an Internet address and convert to a binary address.
 * Returns 1 if the address is valid, 0 if not.
 * This replaces inet_addr, the return value from which
 * cannot distinguish between failure and a local broadcast address.
 */
 /* http://www.cs.cmu.edu/afs/cs/academic/class/15213-f00/unpv12e/libfree/inet_aton.c */
int inet_aton(const char *cp, in_addr *ap)
{
  int dots = 0;
  register u_long acc = 0, addr = 0;

  do {
	  register char cc = *cp;

	  switch (cc) {
	    case '0':
	    case '1':
	    case '2':
	    case '3':
	    case '4':
	    case '5':
	    case '6':
	    case '7':
	    case '8':
	    case '9':
	        acc = acc * 10 + (cc - '0');
	        break;

	    case '.':
	        if (++dots > 3) {
		    return 0;
	        }
	        /* Fall through */

	    case '\0':
	        if (acc > 255) {
		    return 0;
	        }
	        addr = addr << 8 | acc;
	        acc = 0;
	        break;

	    default:
	        return 0;
    }
  } while (*cp++) ;

  /* Normalize the address */
  if (dots < 3) {
	  addr <<= 8 * (3 - dots) ;
  }

  /* Store it if requested */
  if (ap) {
	  ap->s_addr = _htonl(addr);
  }

  return 1;    
}


/**
 * \brief Callback function of IP address.
 *
 * \param[in] hostName Domain name.
 * \param[in] hostIp Server IP.
 *
 * \return None.
 */
static void resolve_cb(uint8_t *hostName, uint32_t hostIp)
{
	gu32HostIp = hostIp;
	gbHostIpByName = true;
	printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", hostName,
			(int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
			(int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
}

/**
 * \brief Callback function of TCP client socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg Type of Socket notification
 * \param[in] pvMsg A structure contains notification informations.
 *
 * \return None.
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
  
	/* Check for socket event on TCP socket. */
	if (sock == tcp_client_socket) {
    
		switch (u8Msg) {
		case SOCKET_MSG_CONNECT:
		{
      printf("socket_msg_connect\n"); 
			if (gbTcpConnection) {
				memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
				sprintf((char *)gau8ReceivedBuffer, "%s\n", MAIN_PREFIX_BUFFER);

				tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
				if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR) {
          printf("send \n");
					send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);

					memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
					recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
				} else {
					printf("socket_cb: connect error!\r\n");
					gbTcpConnection = false;
					close(tcp_client_socket);
					tcp_client_socket = -1;
				}
			}
		}
		break;
    


		case SOCKET_MSG_RECV:
		{
			char *pcIndxPtr;
			char *pcEndPtr;

			tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
			if (pstrRecv && pstrRecv->s16BufferSize > 0) {
        printf(pstrRecv->pu8Buffer );//retorno do get html
		char barriga[15] = "barriga";
	
		char *ret1;
		
		
		ret1 = strstr(pstrRecv->pu8Buffer, barriga);
		printf("esse rettt %s\n",ret1);
		
		
		if (ret1 !=NULL){
			printf("funcionou mlk");
			motor = 1;
			delay_ms(500);
		}
		printf("\n");
				
				memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
				recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
			} else {
				printf("socket_cb: recv error!\r\n");
				close(tcp_client_socket);
				tcp_client_socket = -1;
			}
		}
		break;

		default:
			break;
		}
	}
}

static void set_dev_name_to_mac(uint8_t *name, uint8_t *mac_addr)
{
	/* Name must be in the format WINC1500_00:00 */
	uint16 len;

	len = m2m_strlen(name);
	if (len >= 5) {
		name[len - 1] = MAIN_HEX2ASCII((mac_addr[5] >> 0) & 0x0f);
		name[len - 2] = MAIN_HEX2ASCII((mac_addr[5] >> 4) & 0x0f);
		name[len - 4] = MAIN_HEX2ASCII((mac_addr[4] >> 0) & 0x0f);
		name[len - 5] = MAIN_HEX2ASCII((mac_addr[4] >> 4) & 0x0f);
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType Type of Wi-Fi notification.
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters.
 *
 * \return None.
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
			gbConnectedWifi = false;
 			wifi_connected = 0;
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		wifi_connected = M2M_WIFI_CONNECTED;
		
    /* Obtain the IP Address by network name */
		//gethostbyname((uint8_t *)server_host_name);
		break;
	}

	default:
	{
		break;
	}
	}
}
void motorTurn(){
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

/**
 * \brief Main application function.
 *
 * Initialize system, UART console, network then start weather client.
 *
 * \return Program return value.
 */


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
	LED_M_S_PIO->PIO_SODR=(LED_M_S_PIN_MASK);
	LED_M_NC_PIO->PIO_SODR=(LED_M_NC_PIN_MASK);
	LED_M_N_PIO->PIO_CODR=(LED_M_N_PIN_MASK);
	LED_T_S_PIO->PIO_SODR=(LED_T_S_PIN_MASK);
	LED_T_NC_PIO->PIO_SODR=(LED_T_NC_PIN_MASK);
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
void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);
	
	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MONTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR1, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);
	
	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);
	
}
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* Second increment interrupt */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);

		} else {
		/* Time or date alarm */
		if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			if (HOUR1 <= 12){
				manha = 1;
				tarde = 0;
				alimentar_m = 0;
			}
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
			rtc_set_time_alarm(RTC, 1, HOUR2, 1, MINUTE, 1, SECOND);
			if (HOUR2 >= 12){
				manha = 0;
				tarde = 1;
				
			}
		}
	}
}
int main(void)
{
	tstrWifiInitParam param;
	int8_t ret;
	uint8_t mac_addr[6];
	uint8_t u8IsMacAddrValid;
	struct sockaddr_in addr_in;
	
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

	/* Initialize the board. */
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	board_init();

	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);

	/* Initialize the BSP. */
	nm_bsp_init();
	
	RTC_init();
  
	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}
  
	/* Initialize socket module. */
	socketInit();
  
	/* Register socket callback function. */
	registerSocketCallback(socket_cb, resolve_cb);

  /* Connect to router. */
	printf("main: connecting to WiFi AP %s...\r\n", (char *)MAIN_WLAN_SSID);
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	addr_in.sin_family = AF_INET;
	addr_in.sin_port = _htons(MAIN_SERVER_PORT);
  inet_aton(MAIN_SERVER_NAME, &addr_in.sin_addr);
  printf("Inet aton : %d", addr_in.sin_addr);
  
  rtc_set_time_alarm(RTC, 1, HOUR1, 1, MINUTE, 1, SECOND);

  while(1){
	  if (motor == 1){
		  motorTurn();
		  motor = 0;
	  }
	  
	  //checa se há presença
	  if(	(SENSOR_PIO->PIO_PDSR & (SENSOR_PIN_MASK))&&
			(!(INFRA_PIO->PIO_PDSR & (INFRA_PIN_MASK)))&&
			(alimentar_m==0) && (manha == 1)){
		  LED_M_N_PIO->PIO_CODR=(LED_M_S_PIN_MASK);
		  LED_M_NC_PIO->PIO_SODR=(LED_M_NC_PIN_MASK);
		  motorTurn();
		  alimentar_m = 1;
	  }
	  if((SENSOR_PIO->PIO_PDSR & (SENSOR_PIN_MASK))&&(!(INFRA_PIO->PIO_PDSR & (INFRA_PIN_MASK)))&&alimentar_t==0&& tarde ==1){
		  LED_T_N_PIO->PIO_SODR=(LED_M_S_PIN_MASK);
		  LED_T_NC_PIO->PIO_CODR=(LED_M_NC_PIN_MASK);
		  motorTurn();
		  alimentar_t = 1;
	  }
	  if (alimentar_m==1&&comer_m==0&&(!(INFRA_PIO->PIO_PDSR & (INFRA_PIN_MASK)))&& manha == 1){
		comer_m=1;
		LED_M_S_PIO->PIO_CODR=(LED_M_S_PIN_MASK);
		LED_M_NC_PIO->PIO_SODR=(LED_M_NC_PIN_MASK);
		
	  }
	  if (alimentar_t==1&&comer_t==0&&(!(INFRA_PIO->PIO_PDSR & (INFRA_PIN_MASK)))&& tarde == 1){
		  comer_t=1;
		  LED_T_S_PIO->PIO_CODR=(LED_M_S_PIN_MASK);
		  LED_T_NC_PIO->PIO_SODR=(LED_M_NC_PIN_MASK);
		  
	  }
	  
 		m2m_wifi_handle_events(NULL);

   	if (wifi_connected == M2M_WIFI_CONNECTED) {  
    	/* Open client socket. */
			if (tcp_client_socket < 0) {
        printf("socket init \n");
				if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
					printf("main: failed to create TCP client socket error!\r\n");
					continue;
				}

				/* Connect server */
        printf("socket connecting\n");
        
				if (connect(tcp_client_socket, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR) {
					close(tcp_client_socket);
					tcp_client_socket = -1;
          printf("error\n");
				}else{
          gbTcpConnection = true;
        }
	  }
    }
  }


	return 0;
}
