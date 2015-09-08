/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/main.c
  * @author  Martin Vincent Bloedorn & Patrick José Pereira
  * @version V1.0.0
  * @date    27-August-2014
  * @brief   Startup do projeto.
  * @warning Modificar os arquivos 
    provant-software/io-board/stm32f4/common/modules/common/stm32f4xx_conf.h
    provant-software/io-board/stm32f4/lib/cmsis/inc/stm32f4xx.h
    provant-software/io-board/stm32f4/lib/cmsis/inc/stm32f4xx_conf.h
    dependendo da placa que esta trabalhando.
    Recompilando com:
    make allclean
    make
  *	TODO
  *
  *****************************************************************************/
/* Includes ------------------------------------------------------------------*/

/* Std includes */
#include <stdio.h>
#include <math.h>
#include "inttypes.h"

/* Hardware includes. */
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"
#define ARM_MATH_CM4
#include "arm_math.h"

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "task.h"

/* FreeRTOS+Trace Includes */
#include "trcUser.h"

/* ProVANT Modules */
#include "pv_module_esc.h"
#include "pv_module_serial.h"
#include "pv_module_servo.h"
#include "pv_module_imu.h"

/* Common Components, FOR TESTING */
#include "c_common_gpio.h"
#include "c_io_blctrl.h"
#include "c_io_sonar.h"
#include "c_io_rx24f.h"

/** @addtogroup ProVANT_Modules
  * \brief Ponto de entrada do software geral do VANT.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void vApplicationTickHook() {};
void vApplicationIdleHook() {};
void vApplicationStackOverflowHook() {};
void vApplicationMallocFailedHook() {};


void FPU_init(){
	/* Enable FPU.*/
	__asm("  LDR.W R0, =0xE000ED88\n"
		"  LDR R1, [R0]\n"
		"  ORR R1, R1, #(0xF << 20)\n"
		"  STR R1, [R0]");

	//#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	//#endif
}

/* Tasks ----------------------------------------------------------------------*/

// `I'm alive` kind of task
void blink_led_task(void *pvParameters)
{
  GPIOPin LED_builtin;
  GPIOPin LED_builtin2;
  #ifdef STM32F4_H407
    LED_builtin = c_common_gpio_init(GPIOC, GPIO_Pin_13, GPIO_Mode_OUT);
  #else
    LED_builtin = c_common_gpio_init(GPIOA, GPIO_Pin_7, GPIO_Mode_OUT);
    LED_builtin2 = c_common_gpio_init(GPIOB, GPIO_Pin_1, GPIO_Mode_OUT);
    c_common_gpio_toggle(LED_builtin2);
  #endif
    while(1) {
    	c_common_gpio_toggle(LED_builtin);
      #ifdef STM32F4_DISCOVERY
        c_common_gpio_toggle(LED_builtin2);
      #endif
      vTaskDelay(500/portTICK_RATE_MS);
    }
}

void arduino_i2c_task() {
	char buffer[20];//create a temporary buffer with size of IMU_MSG struct
	uint8_t device = 5;
	uint8_t address = 0;
	char byteToWrite = 'P';
	portTickType lastWakeTime;/* Leitura do numero de ciclos atuais */
	//lastWakeTime = xTaskGetTickCount();

	//send a byte to arduino
	while(1) {
		lastWakeTime = xTaskGetTickCount();
		c_common_i2c_writeByte(I2C1,device,address, byteToWrite);
		vTaskDelayUntil( &lastWakeTime, (100 / portTICK_RATE_MS));//periodo de 100 ms
	}
}

void module_esc_task() {
	module_esc_run();
}

void module_serial_task() {
	module_serial_run();
}

void module_servo_task() {
	module_servo_run();
}

void module_imu_task() {
	module_imu_run();
}

/* Main ----------------------------------------------------------------------*/
int main(void)
{
	/* Init system and trace */

	SystemInit();
	FPU_init();

	vTraceInitTraceData();
	vTraceConsoleMessage("Starting application...");
	if (! uiTraceStart() )
		vTraceConsoleMessage("Could not start recorder!");


#if !SERIAL_TEST
	/* Init modules */
	//module_esc_init();
	//module_servo_init();
	module_imu_init();
#endif
#if !SERVO_IN_TEST
	//module_serial_init();
#endif
	/* Connect modules: interface1.o* = interface2.i* */
	//pv_interface_do.iInputData  = pv_interface_in.oInputData;
	//pv_interface_co.iInputData  = pv_interface_in.oInputData;
	//pv_interface_do.iControlOutputData  = pv_interface_co.oControlOutputData;
#if !SERVO_IN_TEST && !SERIAL_TEST
	//pv_interface_serial.iServoOutput=pv_interface_servo.oServoOutput;
#endif
	//iEscQueueData = oEscQueueData;
	/* create tasks
	 * Prioridades - quanto maior o valor, maior a prioridade
	 */
	xTaskCreate(blink_led_task, (signed char *)"Blink led", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
#if !SERIAL_TEST
	//xTaskCreate(arduino_i2c_task, (signed char *)"Arduino", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+2, NULL);
	//xTaskCreate(module_esc_task, (signed char *)"ESC", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+3, NULL);
	//xTaskCreate(module_servo_task, (signed char *)"Servo", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+2, NULL);
	xTaskCreate(module_imu_task, (signed char *)"IMU", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+2, NULL);
#endif
#if !SERVO_IN_TEST
	//xTaskCreate(module_serial_task, (signed char *)"Serial", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+2, NULL);
#endif
	//xTaskCreate(sonar_task, (signed char *)"Sonar task", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* should never reach here! */
	for(;;);
}

/**
  * @}
  */
