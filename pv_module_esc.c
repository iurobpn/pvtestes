/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_esc.c
  * @author  Iuro Baptista Pereira Nascimento
  * @version V1.0.0
  * @date    08/11/2014
  * @brief   Modulo para teste e modelagem dos motores brushless c/ comunicacao
  * 			com os ESCs
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_esc.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_co
  * \brief Módulo com as principais funcionalidades para calculo de controle e escrita de atuadores.
  *
  * Definição do módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   6//ms
#define ESC_ON           1
#define SERVO_ON         0


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
extern xQueueHandle oEscQueueData;
//pv_msg_esc oEscMsgData;
//pv_msg_input iInputData;
//pv_msg_controlOutput oControlOutputData;

//GPIOPin debugPin;
/* Inboxes buffers */

/* Outboxes buffers*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de controle + output.
  *
  * Instancia as Queues de comunicação inter-thread, inicializa a pinagem necessária para
  * os perifericos e aloca o que for necessário para as equações de controle.
  * @param  None
  * @retval None
  */
void module_esc_init()
{

  /* Inicializar os escs*/
  c_common_i2c_init(I2C3);
  c_io_blctrl_init_i2c(I2C3);

  //reserva memoria para oEscQueueData
  //oEscQueueData = xQueueCreate(1, sizeof(pv_msg_esc));

  /*

  /* Pin for debug */
  //debugPin = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);
//  pv_interface_co.iInputData          = xQueueCreate(1, sizeof(pv_msg_input));
//  pv_interface_co.oControlOutputData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));
}

/** \brief Função principal do módulo de RC.
  * @param  None
  * @retval None
  *
  * Interpreta o recebimento de PPM, calcula sinais de controle e os envia
  * via interface.
  * Devido as diferenças do modelo matematica com a construção mecanica o sinal do angulo do servo direito deve
  * ser adaptado.
  *
  */
void module_esc_run()
{
  unsigned int heartBeat=0;

  while(1)
  {
    /* Variavel para debug */
    heartBeat++;

    /* toggle pin for debug */
    //c_common_gpio_toggle(debugPin);

    /* Passa os valores davariavel compartilha para a variavel iInputData */
//    xQueueReceive(pv_interface_co.iInputData, &iInputData, 0);


    /* Leitura do numero de ciclos atuais */
	lastWakeTime = xTaskGetTickCount();

    /* Escrita dos escs */
    c_io_blctrl_setSpeed(0, 20);
    //c_common_utils_delayus(10);
    //c_io_blctrl_setSpeed(1, 10 );

    //c_io_blctrl_updateBuffer(0);
    /*
    oEscMsgData.ID=0;
    oEscMsgData.angularSpeed = c_io_blctrl_readSpeed(0);
    oEscMsgData.voltage = c_io_blctrl_readVoltage(0);;
    oEscMsgData.current = c_io_blctrl_readCurrent(0);


    oEscMsgData.heartBeat                  = heartBeat;
    unsigned int timeNow=xTaskGetTickCount();
    oEscMsgData.sampleTime                  = timeNow - lastWakeTime;
    */

    /* toggle pin for debug */
    //c_common_gpio_toggle(debugPin);

    //if(oEscQueueData != 0)
      //xQueueOverwrite(oEscQueueData, &oEscMsgData);

    /* A thread dorme ate o tempo final ser atingido */
    vTaskDelayUntil( &lastWakeTime, MODULE_PERIOD / portTICK_RATE_MS);
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */




