 /**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_serial.c
  * @author  Iuro Baptista Pereira Nascimento
  * @version V1.0.0
  * @date    10/11/2014
  * @brief   implementacao do modulo de envio de dados de teste via UART
  ******************************************************************************/

#include "pv_module_serial.h"
#include "c_io_herkulex.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_do
  * \brief Módulo responsavel por transmitir dados.
  *
  * Definição do módulo de transmissão de dados.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	    100//ms
//#define USART_BAUDRATE     460800
#define USART_BAUDRATE     500000



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
USART_TypeDef *USARTx = USART2;
//extern xQueueHandle iEscQueueData;
pv_msg_servo iServoOutput;
pv_msg_servo servo_out_buffer[20];
uint8_t BUFFER[100];
int32_t msg_size;// = sizeof(pv_msg_esc);
uint8_t size;
//pv_msg_input iInputData;
//pv_msg_controlOutput iControlOutputData;
//GPIOPin debugPin;
/* Private function prototypes -----------------------------------------------*/
uint16_t send_queue();
/* Private functions ---------------------------------------------------------*/
void send_data(uint8_t size);
void serialize_servo_msg(pv_msg_servo msg);
uint8_t cksum1(uint8_t buffer[]);
uint8_t cksum2(uint8_t checksum1);
void stub();
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de data out.
  *
  * Instancia as Queues de comunicação inter-thread.
  * @param  None
  * @retval None
  */
void module_serial_init()
{
	/* Inicia a usart2 */
	if (USARTx==USART1) {
		c_common_usart1_init(USART_BAUDRATE);
	} else if (USARTx==USART2) {
		c_common_usart2_init(USART_BAUDRATE);
	} else if (USARTx==USART3) {
		c_common_usart3_init(USART_BAUDRATE);
	} else if (USARTx==USART6) {
		c_common_usart6_init(USART_BAUDRATE);
	}
	//stub();
//#if SERIAL_TEST
//	c_io_herkulex_init(USARTx,USART_BAUDRATE);
//#endif



	/* Reserva o local de memoria compartilhado */
	//iEscQueueData = xQueueCreate(1, sizeof(pv_msg_esc));

	/* Pin for debug */
	//debugPin = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);
	size = 0;
}

void stub() {
	//codigo de teste
	BUFFER[0]=0xFF;
	BUFFER[1]=0xFF;
	BUFFER[2]=3*4+3;//3 float(32bits) + 1 byte de tamanho, +2 de checksum
	float value=  3.14159265;
	float value1=  3.14159265/2.0;
	float value2=  3.14159265/4.0;
	memcpy((BUFFER+3),&value,4);
	memcpy((BUFFER+7),&value1,4);
	memcpy((BUFFER+11),&value2,4);
	BUFFER[15]=cksum1(BUFFER);
	BUFFER[16]=cksum2(BUFFER[15]);
}

/** \brief Função principal do módulo de data out.
  * @param  None
  * @retval None
  *
  */
void module_serial_run()
{
	unsigned int heartBeat=0, xKill=1;
	portBASE_TYPE xStatus;
	while(1)
	{
		lastWakeTime = xTaskGetTickCount();
		heartBeat++;
#if SERIAL_TEST
		stub();
		send_data(BUFFER[2]+2);
		xStatus=0;
		//uint8_t r = receive();
		//xStatus=r;
#else
#if TESTE_FINITO
		uint16_t queue_size = uxQueueMessagesWaiting(pv_interface_serial.iServoOutput);
		if (queue_size>=QUEUE_SIZE)
			xKill = send_queue();
		else
			xStatus = 0; //juntar a parte inferior
#endif

		if (xStatus) {
			send_queue();
		}

#if TESTE_FINITO
		if (xKill)
		{
#endif
#endif
			vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
#if !SERIAL_TEST & TESTE_FINITO
		} else
		{
			break;
		}
#endif
	}
	vTaskDelete(NULL);
}

uint16_t send_queue()
{
	uint16_t xStatus = 0, i, queue_size = 1;
	while (uxQueueMessagesWaiting(pv_interface_serial.iServoOutput)>0)
	{
		xStatus = xQueueReceive(pv_interface_serial.iServoOutput,&iServoOutput,1/portTICK_RATE_MS);
		if (xStatus)
			add_servo_to_buffer(iServoOutput);
	}
	prepare_buffer();
	send_data(BUFFER[2]+2);
	clear_buffer();
	return 0;
}


void add_servo_to_buffer(pv_msg_servo servo_data)
{
	int msg_size = sizeof(pv_msg_servo);
	memcpy((BUFFER + size + 3),&servo_data,msg_size);
	size+=msg_size;
}

void clear_buffer(void)
{
	size = 0;
}

uint8_t cksum1(uint8_t buffer[]) {
  uint8_t i, chksum=0;
  uint8_t n = BUFFER[2];//-2-> tira o checksum  2->por começar em buffer[2]
  /* o buffer é passado inteiro, com o cabeçalho */
  for(i=2;i<n;i++) {
    chksum=chksum^buffer[i];
  }

  return chksum&0xFE;
}

uint8_t cksum2(uint8_t checksum1)
{
	return (~checksum1) & 0xFE;
}

void serialize_servo_msg(pv_msg_servo msg)
{
	int msg_size = sizeof(pv_msg_servo);
	BUFFER[0]=0xFF;
	BUFFER[1]=0xFF;
	BUFFER[2]=msg_size+3;
	memcpy((BUFFER+3),&msg,msg_size);
	BUFFER[msg_size+3]=cksum1(BUFFER);
	BUFFER[msg_size+4]=cksum2(BUFFER[msg_size+3]);
}

void send_data(uint8_t size)
{
	for (int i = 0; i < size ; ++i)
		c_common_usart_putchar(USARTx,BUFFER[i]);
}

void prepare_buffer(void)
{
	BUFFER[0] = 0xFF;
	BUFFER[1] = 0xFF;
	BUFFER[2] = size + 3;
	BUFFER[size + 3] = cksum1(BUFFER);
	BUFFER[size + 4] = cksum2(BUFFER[size + 3]);
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
