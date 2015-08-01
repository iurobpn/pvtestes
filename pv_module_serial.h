/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_serial.h
  * @author  Iuro Baptista Pereira Nascimento
  * @version V1.0.0
  * @date    10/11/2014
  * @brief   Modulo para envio de dados de teste via UART
  ******************************************************************************/
#ifndef APP_REMOTE_CONTROLLED_FLIGHT_PV_MODULE_SERIAL_H_
#define APP_REMOTE_CONTROLLED_FLIGHT_PV_MODULE_SERIAL_H_

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
#include "c_common_i2c.h"
#include "c_common_uart.h"
#include "c_common_utils.h"
#include "pv_typedefs.h"




/* Exported types ------------------------------------------------------------*/
 //xQueueHandle iEscOutputData;
 struct pv_interface_serial
 {
   xQueueHandle iServoOutput;
 } pv_interface_serial;

/* Exported constants --------------------------------------------------------*/
#define SERIAL_TEST 0
#define QUEUE_SIZE 500
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void module_serial_init();
void module_serial_run();
void serialize_esc_msg();
void prepare_buffer(void);
void clear_buffer(void);
void add_servo_to_buffer(pv_msg_servo servo_data);

#ifdef __cplusplus
}
#endif



#endif /* APP_REMOTE_CONTROLLED_FLIGHT_PV_MODULE_SERIAL_H_ */
