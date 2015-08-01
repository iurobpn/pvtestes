/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_esc.c
  * @author  Iuro Baptista Pereira Nascimento
  * @version V1.0.0
  * @date    08/11/2014
  * @brief   Modulo para teste e modelagem dos motores brushless c/ comunicacao
  * 			com os ESCs
  ******************************************************************************/

#ifndef APP_REMOTE_CONTROLLED_FLIGHT_PV_MODULE_ESC_H_
#define APP_REMOTE_CONTROLLED_FLIGHT_PV_MODULE_ESC_H_

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
#include "c_io_blctrl.h"

#include "pv_typedefs.h"

/* Exported types ------------------------------------------------------------*/
/*struct pv_interface_co
{
  xQueueHandle iInputData;
  xQueueHandle oControlOutputData;
} pv_interface_co;*/
xQueueHandle oEscQueueData;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void module_esc_init();
void module_esc_run();

#ifdef __cplusplus
}
#endif



#endif /* APP_REMOTE_CONTROLLED_FLIGHT_PV_MODULE_ESC_H_ */
