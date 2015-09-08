/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_servo.c
  * @author  Iuro Baptista Pereira Nascimento
  * @version V1.0.0
  * @date    06/12/2014
  * @brief   Modulo de testes do servos Herkulex DRS0201
  ******************************************************************************/

#include "pv_module_servo.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

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
#define MODULE_PERIOD	    10//ms
#define USART_BAUDRATE     666666
#define QUEUE_SIZE 500

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
portTickType wakeTime;
static USART_TypeDef *USARTn = USART1;

pv_msg_servo oServoMsg[2];

uint8_t DATA[100];
int32_t size;// = sizeof(pv_msg_esc);
uint8_t servo1_id;
uint8_t servo2_id;

//GPIOPin debugPin;
/* Private function prototypes -----------------------------------------------*/
float position_controller(float r, float y);
float velocity_controller(float r, float y);
float velocity_feedforward(float r);
int16_t saturate(float x, const float max);
int16_t get_stepped_pwm(int heartBeat, int16_t pwm);
int check_outlier(int new,int sec);

/* Private functions ---------------------------------------------------------*/
float position_controller(float r, float y)
{
	static float e_old = 0, u = 0;
	float e = r-y;
	//Tset=200ms PASSOU no teste RP 5/6
	//int K1 = 144.1257, K2 = 92.8029;
	//int K1=619.4, K2=318.4955; //original
	//Passou no teste RP 5x
	//int K1 = 22.0272, K2=20.6867;// slow motherfucker PI
	int K1 = 11.1838, K2 = -10.8214;
	u=u+K1*e-K2*e_old; //intermediario
	e_old=e;
	//saturacao

	//assert(out>=(-1023) && out<=1023);

	return u;
}

float velocity_controller(float r, float y)
{
	static float e_old = 0, u = 0;
	float e = r-y;
	//Tset=200ms PASSOU no teste RP 5/6
	int K1 = 144.1257, K2 = 92.8029;
	//int K1=619.4, K2=318.4955; //original
	//Passou no teste RP 5x20.6867
	//int K1 = 15.966, K2=7.9196;// slow motherfucker PI

	u=u+K1*e-K2*e_old;
	e_old=e;
	//saturacao

	//assert(out>=(-1023) && out<=1023);

	return u;
}

float velocity_feedforward(float r)
{
	static float y = 0;
	//y=0.5142*(y+r); //original
	y=0.6439*y+0.3561*r; //slow

	return y;
}

int16_t saturate(float x, const float max)
{
	if (x>max)
		x=max;
	else if (x<(-max))
		x=-max;
	if (x>0) x+=0.5;
	if (x<0) x-=0.5;

	return (int16_t)x;
}

int16_t get_stepped_pwm(int heartBeat, int16_t pwm)
{
  static int8_t counter = 0, inc=1;

  //parametros da escada
  const int STEP_LENGTH = 50; //tamanho do degraus em ptos de amostragem
  const int STEP_SIZE = 200; //amplitude dos degraus
  const int NUM_STEPS = 5; //numero de degraus

  if ((heartBeat%STEP_LENGTH)==0)
  {
    if (counter == NUM_STEPS) inc=0;
    else if (counter == -NUM_STEPS) inc=1;
    if (inc) counter++;
    else counter--;
    pwm=counter*200;
    //if (pwm) pwm=0;
    //else pwm=1000;
  }

  return pwm;
}
//new value, and secure value
int check_outlier(int new, int sec)
{
	int limite = 0.6;
	return ((sec>=0 && (new>=sec*limite)) ||
			(sec<0 && (new<=sec*limite)));
}



/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de data out.
  *
  * Instancia as Queues de comunicação inter-thread.
  * @param  None
  * @retval None
  */
void module_servo_init()
{
	pv_interface_servo.oServoOutput = xQueueCreate(QUEUE_SIZE,
			sizeof(pv_msg_servo));

	servo1_id=253;
	servo2_id=252;

	c_io_herkulex_init(USARTn,USART_BAUDRATE);
	//c_io_herkulex_setBaudRate(servo1_id,666666);
	//c_io_herkulex_setBaudRate(servo2_id,666666);
	c_io_herkulex_config(servo1_id);
	c_io_herkulex_config(servo2_id);
}

/** \brief Função principal do módulo de data out.
  * @param  None
  * @retval None
  *
  */
void module_servo_run()
{
	uint32_t heartBeat=0;
	int32_t data_counter=0;
	int32_t queue_data_counter = 0;
	int32_t lost_data_counter = 0;
	uint8_t data_received = 0;
	uint8_t status = 0, status_error=0, status_detail=0;
	//int st =0, el=0;
	int16_t pwm = 200;
	float new_vel=0, new_pos = 0;
	//float sec_vel = 0, sec_pos = 0;
	uint8_t l = 0;

	float pos1 = 45*M_PI/180;
	float pos2 = 135*M_PI/180;
	//float pos2 = -pos1;
	//c_io_herkulex_set_goal_position(servo1_id,0);

	//c_io_herkulex_read_data(servo1_id);
	//float initial_position = c_io_herkulex_get_position(servo1_id);

	//float ref_pos = initial_position + 5*PI/180;
	portBASE_TYPE xStatus;
	c_common_utils_delayms(1);

	while(1)
	{
		wakeTime = xTaskGetTickCount();
		heartBeat++;

		/**
		 * Leitura de dados
		 */
#if !SERVO_IN_TEST

		if (c_io_herkulex_readData(servo1_id)) {
			// Aquisição de dados do servo 1
			oServoMsg[0].heartBeat=heartBeat;
			new_vel = c_io_herkulex_getVelocity();
			oServoMsg[0].angularSpeed = new_vel;
			new_pos = c_io_herkulex_getPosition();
			oServoMsg[0].position = new_pos;
			//oServoMsg.status=1;
			oServoMsg[0].servo_id=servo1_id;

			status_error = c_io_herkulex_getStatusError();
			status_detail = c_io_herkulex_getStatusDetail();

			if (status_error) {
				//c_io_herkulex_clear(servo1_id);
			}
		} else {
			data_received = 0;
			// clear servo 1 struct
			oServoMsg[0].angularSpeed=0;
			oServoMsg[0].position=0;
			oServoMsg[0].heartBeat=heartBeat;
			oServoMsg[0].pwm=0;
			//oServoMsg[0].status=0;
			oServoMsg[0].servo_id=servo1_id;
		}

		if (c_io_herkulex_readData(servo2_id)) {
			/* Aquisição de dados do servo 1 */
			oServoMsg[1].heartBeat=heartBeat;
			oServoMsg[1].angularSpeed = c_io_herkulex_getVelocity();
			oServoMsg[1].position = c_io_herkulex_getPosition();
			//oServoMsg.status=1;
			oServoMsg[1].servo_id=servo2_id;
			data_counter++;
			data_received=1;
			status_error = c_io_herkulex_getStatusError();
			status_detail = c_io_herkulex_getStatusDetail();

			if (status_error) {
				//c_io_herkulex_clear(servo2_id);
			}
		} else {
			data_received = 0;
			/* clear servo 1 struct */
			oServoMsg[1].angularSpeed=0;
			oServoMsg[1].position=0;
			oServoMsg[1].heartBeat=heartBeat;
			oServoMsg[1].pwm=0;
			//oServoMsg[0].status=0;
			oServoMsg[1].servo_id=servo2_id;
		}

		/* loop de controle
		//float ref_vel=5;
		if (data_received)
		{	//somente atualiza o pwm se receber dados.
			if (check_outlier(new_vel,sec_vel)) {//checar precedencia
		 	 	//Controle de velocidade - funciona, dependendo do controlador
				//
				//float rv = velocity_feedforward(ref_vel);
				//pwm = saturate(velocity_controller(ref_vel,new_vel),1023);
				sec_vel=new_vel;

				 // Controle de posicao

				//float ref_vel = velocity_feedforward(position_controller(ref_pos,
						//oServoMsg.position));

				if (check_outlier(new_pos,sec_pos))
				{
					sec_pos=new_pos;
					float ref_vel = position_controller(ref_pos,new_pos);
					pwm = saturate(velocity_controller(ref_vel,	new_vel),1023);
				}

			}
		}
		*/
#endif
		/**
		 * Aplicação de entradas nos servos
		 */
		//pwm=200;
		const float delta_deg = deg2rad(90);
		if ((heartBeat%200)==0) {
			float aux;
			aux=pos1;
			if (pos1) {
				pos1 = 0;
				pos2 = 0;
			} else {
				pos1 = -delta_deg;
				pos2 = delta_deg;
			}

			c_io_herkulex_setPosition2Servos(servo1_id, pos1, servo2_id, pos2);
			//c_io_herkulex_setPosition(servo1_id, pos1);
			//c_io_herkulex_setPosition(servo2_id, pos1);
			/*
			if (pwm < 600)
				pwm = 600;
			else
				pwm = 300;
				*/
		}
		//c_io_herkulex_setTorque2Servos(servo1_id, pwm,servo2_id,-pwm);
		//c_io_herkulex_setTorque(servo1_id, pwm);
		//c_io_herkulex_setTorque(servo2_id, pwm);

#if !SERVO_IN_TEST
		/*
		c_io_herkulex_stat(servo2_id);
		status_detail = c_io_herkulex_get_status_detail();
		status_error = c_io_herkulex_get_status_error();
		if (status_error) {
			LOG(status_error, status_detail, l);
			c_io_herkulex_clear(servo2_id);
		}*/

		/**
		 * Envio dos dados para o modulo de comunicação
		 * Verificação da integridade dos pacotes recebidos
		 */
		status = c_io_herkulex_getStatus();//indica erros de comunicação

		if (status) {
			xStatus = xQueueSend(pv_interface_servo.oServoOutput, &oServoMsg[0],
					1/portTICK_RATE_MS);
			if (!xStatus)
				xQueueSend(pv_interface_servo.oServoOutput, &oServoMsg[0],
						1/portTICK_RATE_MS);

			if (xStatus)
				queue_data_counter++;

			xStatus = xQueueSend(pv_interface_servo.oServoOutput, &oServoMsg[1],
							1/portTICK_RATE_MS);
			if (!xStatus)
				xQueueSend(pv_interface_servo.oServoOutput, &oServoMsg[1],
						1/portTICK_RATE_MS);

			if (xStatus)
				queue_data_counter++;
		} else
		{
			c_io_herkulex_readStatus(servo1_id);
			status_error=c_io_herkulex_getStatusError();
			status_detail=c_io_herkulex_getStatusDetail();
			if (status_error!=0 || status_detail!= 0)
			{
				c_io_herkulex_clear(servo1_id);
			}
			lost_data_counter++;

			c_io_herkulex_readStatus(servo2_id);
			status_error=c_io_herkulex_getStatusError();
			status_detail=c_io_herkulex_getStatusDetail();
			if (status_error!=0 || status_detail!= 0)
			{
				c_io_herkulex_clear(servo2_id);
			}
			lost_data_counter++;
		}

#if TESTE_FINITO
		/**
		 * Para utilização em testes finitos dos servos.
		 * O tamanho da fila(xQueue) indica o numero de pontos acumulados
		 */
		//c_common_utils_delayms(1);=1000
		uint16_t queue_size = uxQueueMessagesWaiting(
				pv_interface_servo.oServoOutput);
		if (queue_size<QUEUE_SIZE)
		{
#endif
#endif
			lastWakeTime=wakeTime;
			vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
#if !SERVO_IN_TEST & TESTE_FINITO
		} else
		{
			break;
		}
#endif
	}
	c_io_herkulex_setTorque(servo1_id, 0);
	c_io_herkulex_setTorqueControl(servo1_id,TORQUE_BREAK);//set torque free
	c_io_herkulex_setTorque(servo2_id, 0);
	c_io_herkulex_setTorqueControl(servo2_id,TORQUE_BREAK);//set torque free
	vTaskDelete(NULL);
}




/*
void serialize_esc_msg() {
	memcpy(serial_msg,&iEscMsgData,msg_size);
}*/
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  * @}
  */


