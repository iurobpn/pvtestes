/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_servo.h
  * @author  Iuro Baptista Pereira Nascimento
  * @version V1.0.0
  * @date    06/12/2014
  * @brief   implementacao do modulo de testes do servos Herkulex DRS0201
  ******************************************************************************/

#ifndef APP_REMOTE_CONTROLLED_FLIGHT_PV_MODULE_SERVO_H_
#define APP_REMOTE_CONTROLLED_FLIGHT_PV_MODULE_SERVO_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* proVANT includes */
#include "c_common_gpio.h"
#include "c_common_uart.h"
#include "c_common_utils.h"
#include "c_io_herkulex.h"
#include "pv_typedefs.h"




/* Exported types ------------------------------------------------------------*/
 //xQueueHandle iEscOutputData;

struct pv_interface_servo
 {
   xQueueHandle oServoOutput;
 } pv_interface_servo;

/* Exported constants --------------------------------------------------------*/
#define SERVO_IN_TEST 0
#define TESTE_FINITO 0
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void module_servo_init();
void module_servo_run();
//void serialize_esc_msg();

#ifdef __cplusplus
}
#endif
#endif /* APP_REMOTE_CONTROLLED_FLIGHT_PV_MODULE_SERVO_H_ */
