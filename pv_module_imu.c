/**
  ******************************************************************************
  * @file    modules/io/pv_module_io.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do módulo de gerenciamento de sensores.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_imu.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_in
  * \brief Componentes para o sensoriamento do VANT.
  *
  * Reunião de todos os componentes relacionados às operações de input do VANT.
  * Leituras de todos os sensores. O processamento destes
  * dados brutos é feito neste módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   1000//ms
#define BAUDRATE		115200

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
char str[256];
GPIOPin LED4;
int32_t bmp = 18001;
struct L3GD20 gyro;
static USART_TypeDef *USARTn = USART6;
static I2C_TypeDef *I2Cn;

float attitude_quaternion[4]={1,0,0,0};


/* Output Message */
pv_msg_input oInputData;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*
void displaySensorDetails(void)
{
  sensor_t sensor;
  char tmp[20];

  c_io_lsm303Accel_getSensor(&accel, &sensor);

  c_common_usart_puts(USARTn,"----------- ACCELEROMETER ----------\n");
  c_common_usart_puts(USARTn,"Sensor:       ");
  c_common_usart_puts(USARTn,sensor.name);
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Driver Ver:   ");
  c_common_utils_floatToString((float)sensor.version,tmp,0);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Unique ID:    ");
  c_common_utils_floatToString((float)sensor.sensor_id,tmp,0);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Max Value:    ");
  c_common_utils_floatToString(sensor.max_value,tmp,2);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_puts(USARTn," m/s^2");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Min Value:    ");
  c_common_utils_floatToString(sensor.min_value,tmp,2);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_puts(USARTn," m/s^2");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Resolution:   ");
  c_common_utils_floatToString(sensor.resolution,tmp,2);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_puts(USARTn," m/s^2");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"------------------------------------");
  c_common_usart_puts(USARTn,"\n\n");

  c_io_l3gd20_getSensor(&gyro,&sensor);
  c_common_usart_puts(USARTn,"------------- GYROSCOPE -----------");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Sensor:       ");
  c_common_usart_puts(USARTn,sensor.name);
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Driver Ver:   ");
  c_common_utils_floatToString((float)sensor.version,tmp,0);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Unique ID:    ");
  c_common_utils_floatToString((float)sensor.sensor_id,tmp,0);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_putchar(USARTn,'\n');


  c_common_usart_puts(USARTn,"Max Value:    ");
  c_common_utils_floatToString(sensor.max_value,tmp,2);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_puts(USARTn," rad/s");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Min Value:    ");
  c_common_utils_floatToString(sensor.min_value,tmp,2);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_puts(USARTn," rad/s");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Resolution:   ");
  c_common_utils_floatToString(sensor.resolution,tmp,2);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_puts(USARTn," rad/s");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"------------------------------------");
  c_common_usart_puts(USARTn,"\n\n");

  c_io_lsm303Mag_getSensor(&mag, &sensor);
  c_common_usart_puts(USARTn,"----------- MAGNETOMETER -----------");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Sensor:       ");
  c_common_usart_puts(USARTn,sensor.name);
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Driver Ver:   ");
  c_common_utils_floatToString((float)sensor.version,tmp,0);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Unique ID:    ");
  c_common_utils_floatToString((float)sensor.sensor_id,tmp,0);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Max Value:    ");
  c_common_utils_floatToString(sensor.max_value,tmp,2);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_puts(USARTn," uT");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Min Value:    ");
  c_common_utils_floatToString((float)sensor.min_value,tmp,2);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_puts(USARTn," uT");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Resolution:   ");
  c_common_utils_floatToString((float)sensor.resolution,tmp,2);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_puts(USARTn," uT");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"------------------------------------");
  c_common_usart_puts(USARTn,"\n\n");

  c_io_bmp180_getSensor(bmp,&sensor);
  c_common_usart_puts(USARTn,"-------- PRESSURE/ALTITUDE ---------");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Sensor:       ");
  c_common_usart_puts(USARTn,sensor.name);
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Driver Ver:   ");
  c_common_utils_floatToString((float)sensor.version,tmp,0);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Unique ID:    ");
  c_common_utils_floatToString((float)sensor.sensor_id,tmp,0);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Max Value:    ");
  c_common_utils_floatToString(sensor.max_value,tmp,2);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_puts(USARTn," hPa");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Min Value:    ");
  c_common_utils_floatToString(sensor.min_value,tmp,2);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_puts(USARTn," hPa");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"Resolution:   ");
  c_common_utils_floatToString(sensor.resolution,tmp,2);
  c_common_usart_puts(USARTn,tmp);
  c_common_usart_puts(USARTn," hPa");
  c_common_usart_putchar(USARTn,'\n');

  c_common_usart_puts(USARTn,"------------------------------------");
  c_common_usart_puts(USARTn,"\n\n");

}
*/
void displaySensorData(void) {

}
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao componentes de IO.
  *
  * Incializa o hardware para comunicar com os sensores. Rotinas de teste
  * ainda precisam ser executadas.
  * @param  None
  * @retval None
  */
void module_imu_init()
{
	I2Cn=I2C1;
	/* Inicialização do hardware do módulo */
	LED4 = c_common_gpio_init(GPIOD, GPIO_Pin_12, GPIO_Mode_OUT); //LED4

	/* Inicialização da imu */
	//c_common_i2c_init(I2Cn);
	c_io_imuAdafruit_init(I2Cn);
	c_common_usart6_init(BAUDRATE);

	c_io_l3gd20_init(&gyro,20);
	c_common_usart_puts(USARTn,"Adafruit 10DOF Tester");
	c_common_usart_putchar(USARTn,'\n');

	/* Initialise the sensors */
	if(!c_io_lsm303_initAccel(I2Cn)) {
		/* There was a problem detecting the ADXL345 ... check your connections */
		c_common_usart_puts(USARTn,"Ooops, no LSM303 detected ... Check your wiring!");
		c_common_usart_putchar(USARTn,'\n');
		while(1);
	}
	if(!c_io_lsm303_initMag(I2Cn)) {
		/* There was a problem detecting the LSM303 ... check your connections */
		c_common_usart_puts(USARTn,"Ooops, no LSM303 detected ... Check your wiring!");
		c_common_usart_putchar(USARTn,'\n');
		while(1);
	}
	if(!c_io_bmp180_init(I2Cn,bmp)) {
		/* There was a problem detecting the BMP085 ... check your connections */
		c_common_usart_puts(USARTn,"Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}
	if(!c_io_l3gd20_begin(&gyro,GYRO_RANGE_250DPS)) {
		/* There was a problem detecting the L3GD20 ... check your connections */
		c_common_usart_puts(USARTn,"Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}

	/* Display some basic information on this sensor */
	//displaySensorDetails();
}

/** \brief Função principal do módulo de IO.
  * @param  None
  * @retval None
  *
  * Loop que amostra sensores como necessário.
  *
  */
void module_imu_run()
{
	unsigned int heartBeat=0;
	char tmp[20];
  	/*Dados usados no sonar*/


  	while(1) {

		/* Leitura do numero de ciclos atuais */
		lastWakeTime = xTaskGetTickCount();

		oInputData.heartBeat=heartBeat++;

		/*----------------------Tratamento da IMU---------------------*/
		/* Pega e trata os valores da imu */
		//c_io_imu_getRaw(oInputData.imuOutput.accRaw, oInputData.imuOutput.gyrRaw, oInputData.imuOutput.magRaw,sample_time_gyro_us);
		/* Get a new sensor event */
		sensors_event_t event;

		/* Display the results (acceleration is measured in m/s^2) */
		c_io_lsm303_getDataAccel();
		c_common_usart_puts(USARTn,"ACCEL ");
		c_common_usart_puts(USARTn,"X: ");
		c_common_utils_floatToString(event.acceleration.x,tmp,2);
		c_common_usart_puts(USARTn,tmp);
		c_common_usart_puts(USARTn,"  ");
		c_common_usart_puts(USARTn,"Y: ");
		c_common_utils_floatToString(event.acceleration.y,tmp,2);
		c_common_usart_puts(USARTn,tmp);
		c_common_usart_puts(USARTn,"  ");
		c_common_usart_puts(USARTn,"Z: ");
		c_common_utils_floatToString(event.acceleration.z,tmp,2);
		c_common_usart_puts(USARTn,tmp);
		c_common_usart_puts(USARTn,"  ");
		c_common_usart_puts(USARTn,"m/s^2 ");
		c_common_usart_putchar(USARTn,'\n');

		/* Display the results (magnetic vector values are in micro-Tesla (uT)) */
		c_io_lsm303_getMagData();
		c_common_usart_puts(USARTn,"MAG   ");
		c_common_usart_puts(USARTn,"X: ");
		c_common_utils_floatToString(event.magnetic.x,tmp,2);
		c_common_usart_puts(USARTn,tmp);
		c_common_usart_puts(USARTn,"  ");
		c_common_usart_puts(USARTn,"Y: ");
		c_common_utils_floatToString(event.magnetic.y,tmp,2);
		c_common_usart_puts(USARTn,tmp);
		c_common_usart_puts(USARTn,"  ");
		c_common_usart_puts(USARTn,"Z: ");
		c_common_utils_floatToString(event.magnetic.z,tmp,2);
		c_common_usart_puts(USARTn,tmp);
		c_common_usart_puts(USARTn,"  ");
		c_common_usart_puts(USARTn,"uT");
		c_common_usart_putchar(USARTn,'\n');

		/* Display the results (gyrocope values in rad/s) */
		c_io_l3gd20_getEvent(&gyro, &event);
		c_common_usart_puts(USARTn,"GYRO  ");
		c_common_usart_puts(USARTn,"X: ");
		c_common_utils_floatToString(event.gyro.x,tmp,2);
		c_common_usart_puts(USARTn,tmp);
		c_common_usart_puts(USARTn,"  ");
		c_common_usart_puts(USARTn,"Y: ");
		c_common_utils_floatToString(event.gyro.y,tmp,2);
		c_common_usart_puts(USARTn,tmp);
		c_common_usart_puts(USARTn,"  ");
		c_common_usart_puts(USARTn,"Z: ");
		c_common_utils_floatToString(event.gyro.z,tmp,2);
		c_common_usart_puts(USARTn,tmp);
		c_common_usart_puts(USARTn,"  ");
		c_common_usart_puts(USARTn,"rad/s ");
		c_common_usart_putchar(USARTn,'\n');

		/* Display the pressure sensor results (barometric pressure is measure in hPa) */
		c_io_bmp180_getData(bmp,&event);
		if (event.pressure)
		{
			/* Display atmospheric pressure in hPa */
			c_common_usart_puts(USARTn,"PRESS ");
			c_common_utils_floatToString(event.pressure,tmp,2);
			c_common_usart_puts(USARTn,tmp);
			c_common_usart_puts(USARTn," hPa, ");
			/* Display ambient temperature in C */
			float temperature;
			c_io_bmp180_getTemperature(&temperature);
			c_common_utils_floatToString(event.temperature,tmp,2);
			c_common_usart_puts(USARTn,tmp);
			c_common_usart_puts(USARTn," C, ");
			/* Then convert the atmospheric pressure, SLP and temp to altitude    */
			/* Update this next line with the current SLP for better results      */
			float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

			float fl_tmp = c_io_bmp180_pressureToAltitude(seaLevelPressure,event.pressure);
			c_common_utils_floatToString(fl_tmp,tmp,2);
			c_common_usart_puts(USARTn,tmp);
			c_common_usart_puts(USARTn," m");
			c_common_usart_putchar(USARTn,'\n');
		}

		c_common_usart_putchar(USARTn,'\n');

		/* toggle pin for debug */
		c_common_gpio_toggle(LED4);

		/* Realiza o trabalho de mutex */
		//if(pv_interface_in.oInputData != 0)
			//xQueueOverwrite(pv_interface_in.oInputData, &oInputData);

		/* A thread dorme ate o tempo final ser atingido */
		vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */



