/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm303agr_interface.h"
#include "lsm6dsl_interface.h"
#include "lps22hb_interface.h"
#include "delays.h"
#include "accel.h"
#include "attitude.h"
#include "baro.h"
#include "control_motor.h"
#include "gyro.h"
#include "mag.h"
#include "bin_semaphores.h"
#include "global_variables.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
acc_mean accellerometer_mean;
/* USER CODE END Variables */
/* Definitions for tControlMotor */
osThreadId_t tControlMotorHandle;
const osThreadAttr_t tControlMotor_attributes = {
  .name = "tControlMotor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for tAttitude */
osThreadId_t tAttitudeHandle;
const osThreadAttr_t tAttitude_attributes = {
  .name = "tAttitude",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for tAltitude */
osThreadId_t tAltitudeHandle;
const osThreadAttr_t tAltitude_attributes = {
  .name = "tAltitude",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for LBS22HB_pressio */
osThreadId_t LBS22HB_pressioHandle;
const osThreadAttr_t LBS22HB_pressio_attributes = {
  .name = "LBS22HB_pressio",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for LSM303AGR_magne */
osThreadId_t LSM303AGR_magneHandle;
const osThreadAttr_t LSM303AGR_magne_attributes = {
  .name = "LSM303AGR_magne",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for Media_Accellero */
osThreadId_t Media_AccelleroHandle;
const osThreadAttr_t Media_Accellero_attributes = {
  .name = "Media_Accellero",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for LSM6DSL_girosco */
osThreadId_t LSM6DSL_giroscoHandle;
const osThreadAttr_t LSM6DSL_girosco_attributes = {
  .name = "LSM6DSL_girosco",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for Display */
osThreadId_t DisplayHandle;
const osThreadAttr_t Display_attributes = {
  .name = "Display",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for binSemaphoreAcc1 */
osSemaphoreId_t binSemaphoreAcc1Handle;
const osSemaphoreAttr_t binSemaphoreAcc1_attributes = {
  .name = "binSemaphoreAcc1"
};
/* Definitions for binSemaphoreAcc2 */
osSemaphoreId_t binSemaphoreAcc2Handle;
const osSemaphoreAttr_t binSemaphoreAcc2_attributes = {
  .name = "binSemaphoreAcc2"
};
/* Definitions for binSemaphoreMag */
osSemaphoreId_t binSemaphoreMagHandle;
const osSemaphoreAttr_t binSemaphoreMag_attributes = {
  .name = "binSemaphoreMag"
};
/* Definitions for binSemaphoreGyr */
osSemaphoreId_t binSemaphoreGyrHandle;
const osSemaphoreAttr_t binSemaphoreGyr_attributes = {
  .name = "binSemaphoreGyr"
};
/* Definitions for binSemaphorePres */
osSemaphoreId_t binSemaphorePresHandle;
const osSemaphoreAttr_t binSemaphorePres_attributes = {
  .name = "binSemaphorePres"
};
/* Definitions for binSemaphoreI2C */
osSemaphoreId_t binSemaphoreI2CHandle;
const osSemaphoreAttr_t binSemaphoreI2C_attributes = {
  .name = "binSemaphoreI2C"
};
/* Definitions for binSemaphoreAccMean */
osSemaphoreId_t binSemaphoreAccMeanHandle;
const osSemaphoreAttr_t binSemaphoreAccMean_attributes = {
  .name = "binSemaphoreAccMean"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void startTaskControlMotor(void *argument);
void startTaskAttitude(void *argument);
void startTaskAltitude(void *argument);
void getPressure(void *argument);
void getMagnetometer(void *argument);
void getAccMean(void *argument);
void getGyroscope(void *argument);
void displayData(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of binSemaphoreAcc1 */
  binSemaphoreAcc1Handle = osSemaphoreNew(1, 1, &binSemaphoreAcc1_attributes);

  /* creation of binSemaphoreAcc2 */
  binSemaphoreAcc2Handle = osSemaphoreNew(1, 1, &binSemaphoreAcc2_attributes);

  /* creation of binSemaphoreMag */
  binSemaphoreMagHandle = osSemaphoreNew(1, 1, &binSemaphoreMag_attributes);

  /* creation of binSemaphoreGyr */
  binSemaphoreGyrHandle = osSemaphoreNew(1, 1, &binSemaphoreGyr_attributes);

  /* creation of binSemaphorePres */
  binSemaphorePresHandle = osSemaphoreNew(1, 1, &binSemaphorePres_attributes);

  /* creation of binSemaphoreI2C */
  binSemaphoreI2CHandle = osSemaphoreNew(1, 1, &binSemaphoreI2C_attributes);

  /* creation of binSemaphoreAccMean */
  binSemaphoreAccMeanHandle = osSemaphoreNew(1, 1, &binSemaphoreAccMean_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of tControlMotor */
  tControlMotorHandle = osThreadNew(startTaskControlMotor, NULL, &tControlMotor_attributes);

  /* creation of tAttitude */
  tAttitudeHandle = osThreadNew(startTaskAttitude, NULL, &tAttitude_attributes);

  /* creation of tAltitude */
  tAltitudeHandle = osThreadNew(startTaskAltitude, NULL, &tAltitude_attributes);

  /* creation of LBS22HB_pressio */
  LBS22HB_pressioHandle = osThreadNew(getPressure, NULL, &LBS22HB_pressio_attributes);

  /* creation of LSM303AGR_magne */
  LSM303AGR_magneHandle = osThreadNew(getMagnetometer, NULL, &LSM303AGR_magne_attributes);

  /* creation of Media_Accellero */
  Media_AccelleroHandle = osThreadNew(getAccMean, NULL, &Media_Accellero_attributes);

  /* creation of LSM6DSL_girosco */
  LSM6DSL_giroscoHandle = osThreadNew(getGyroscope, NULL, &LSM6DSL_girosco_attributes);

  /* creation of Display */
  DisplayHandle = osThreadNew(displayData, NULL, &Display_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_startTaskControlMotor */
/**
  * @brief  Function implementing the tControlMotor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startTaskControlMotor */
void startTaskControlMotor(void *argument)
{
  /* USER CODE BEGIN startTaskControlMotor */
  /* Infinite loop */
  for(;;)
  {
	gyroUpdate();
	controlMotorUpdate();
    osDelay(TASK_CONTROL_MOTOR_PERIOD);
  }
  /* USER CODE END startTaskControlMotor */
}

/* USER CODE BEGIN Header_startTaskAttitude */
/**
* @brief Function implementing the tAttitude thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTaskAttitude */
void startTaskAttitude(void *argument)
{
  /* USER CODE BEGIN startTaskAttitude */
  /* Infinite loop */
  for(;;)
  {
	accUpdate();
	magUpdate();

	attitudeUpdate();
    osDelay(TASK_ASSETTO_PERIOD);
  }
  /* USER CODE END startTaskAttitude */
}

/* USER CODE BEGIN Header_startTaskAltitude */
/**
* @brief Function implementing the tAltitude thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTaskAltitude */
void startTaskAltitude(void *argument)
{
  /* USER CODE BEGIN startTaskAltitude */
  /* Infinite loop */
  for(;;)
  {
	baroUpdate();

    osDelay(TASK_ALTITUDINE_PERIOD);
  }
  /* USER CODE END startTaskAltitude */
}

/* USER CODE BEGIN Header_getPressure */
/**
* @brief Function implementing the LBS22HB_pressio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_getPressure */
void getPressure(void *argument)
{
  /* USER CODE BEGIN getPressure */
  /* Infinite loop */
  for(;;)
  {
   if(osSemaphoreAcquire(binSemaphoreI2CHandle, 0)==osOK){

	   LPS22HB_dataRead();

	   osSemaphoreRelease(binSemaphoreI2CHandle);
	  }

    osDelay(TASK_PRESSIONE_PERIOD);
  }
  /* USER CODE END getPressure */
}

/* USER CODE BEGIN Header_getMagnetometer */
/**
* @brief Function implementing the LSM303AGR_magne thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_getMagnetometer */
void getMagnetometer(void *argument)
{
  /* USER CODE BEGIN getMagnetometer */
  /* Infinite loop */
  for(;;)
  {
   if(osSemaphoreAcquire(binSemaphoreI2CHandle, 0)==osOK){

	   LSM303AGR_dataReadMag();

	   osSemaphoreRelease(binSemaphoreI2CHandle);
	}

    osDelay(TASK_MAGNETOMETRO_PERIOD);
  }
  /* USER CODE END getMagnetometer */
}

/* USER CODE BEGIN Header_getAccMean */
/**
* @brief Function implementing the Media_Accellero thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_getAccMean */
void getAccMean(void *argument)
{
  /* USER CODE BEGIN getAccMean */
  /* Infinite loop */
  for(;;)
  {
	if(osSemaphoreAcquire(binSemaphoreI2CHandle, 0)==osOK){

		LSM303AGR_dataReadAcc();
		LSM6DSL_dataReadAcc();

		osSemaphoreRelease(binSemaphoreI2CHandle);
	}

	if(osSemaphoreAcquire(binSemaphoreAccMeanHandle, 0)==osOK){

		accellerometer_mean.x = (accellerometer1.x + accellerometer2.x)/2;
		accellerometer_mean.y = (accellerometer1.y + accellerometer2.y)/2;
		accellerometer_mean.z = (accellerometer1.z + accellerometer2.z)/2;

		osSemaphoreRelease(binSemaphoreAccMeanHandle);
	}

    osDelay(TASK_ACCELLEROMETRI_PERIOD);
  }
  /* USER CODE END getAccMean */
}

/* USER CODE BEGIN Header_getGyroscope */
/**
* @brief Function implementing the LSM6DSL_girosco thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_getGyroscope */
void getGyroscope(void *argument)
{
  /* USER CODE BEGIN getGyroscope */
  /* Infinite loop */
  for(;;)
  {
	if(osSemaphoreAcquire(binSemaphoreI2CHandle, 0)==osOK){

			LSM6DSL_dataReadGyro();

			osSemaphoreRelease(binSemaphoreI2CHandle);
	}

    osDelay(TASK_GIROSCOPIO_PERIOD);
  }
  /* USER CODE END getGyroscope */
}

/* USER CODE BEGIN Header_displayData */
/**
* @brief Function implementing the DIsplay thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displayData */
void displayData(void *argument)
{
  /* USER CODE BEGIN displayData */
  /* Infinite loop */
  for(;;)
  {
   //NB: nel caso in cui non riesco ad acquisire anche solo un semaforo la stampa non avviene
   char MSG1[80] = {"\0"};
   char MSG2[80] = {"\0"};
   char MSG3[80] = {"\0"};
   char MSG4[80] = {"\0"};
   char MSG5[80] = {"\0"};
   char MSG6[80] = {"\0"};
   if(osSemaphoreAcquire(binSemaphorePresHandle, 0) == osOK){
   	   if(osSemaphoreAcquire(binSemaphoreAcc1Handle, 0) == osOK){
	   	   if(osSemaphoreAcquire(binSemaphoreAcc2Handle, 0) == osOK){
	   		   if(osSemaphoreAcquire(binSemaphoreAccMeanHandle, 0) == osOK){
	   			   if(osSemaphoreAcquire(binSemaphoreMagHandle, 0) == osOK){
	   				   if(osSemaphoreAcquire(binSemaphoreGyrHandle, 0) == osOK){
	   					   sprintf(MSG1, "dati velocita' angolare       x=%.2f y=%.2f z=%.2f\n\r", gyroscope.x, gyroscope.y, gyroscope.z);
	   					   sprintf(MSG2, "dati accellerometro 1         x=%.2f y=%.2f z=%.2f\n\r", accellerometer1.x, accellerometer1.y, accellerometer1.z);
	   					   sprintf(MSG3, "dati accellerometro 2         x=%.2f y=%.2f z=%.2f\n\r", accellerometer2.x, accellerometer2.y, accellerometer2.z);
	   					   sprintf(MSG4, "media dati accellerometri     x=%.2f y=%.2f z=%.2f\n\r", accellerometer_mean.x, accellerometer_mean.y, accellerometer_mean.z);
	   					   sprintf(MSG5, "dati flusso magnetico         x=%.2f y=%.2f z=%.2f\n\r", magnetometer.x, magnetometer.y, magnetometer.z);
	   					   sprintf(MSG6, "dati pressione                p=%.2f\n\n\r", pressure);
	   				 	   osSemaphoreRelease(binSemaphoreGyrHandle);
	   				   }
	   			 	   osSemaphoreRelease(binSemaphoreMagHandle);
	   			   }
	   		  	   osSemaphoreRelease(binSemaphoreAccMeanHandle);
	   		   }
	   	   	osSemaphoreRelease(binSemaphoreAcc2Handle);
	   	   }
	   	osSemaphoreRelease(binSemaphoreAcc1Handle);
   	   }
   	osSemaphoreRelease(binSemaphorePresHandle);
   }

   HAL_UART_Transmit(&huart3, MSG1, sizeof(MSG1), 100);
   HAL_UART_Transmit(&huart3, MSG2, sizeof(MSG2), 100);
   HAL_UART_Transmit(&huart3, MSG3, sizeof(MSG3), 100);
   HAL_UART_Transmit(&huart3, MSG4, sizeof(MSG4), 100);
   HAL_UART_Transmit(&huart3, MSG5, sizeof(MSG5), 100);
   HAL_UART_Transmit(&huart3, MSG6, sizeof(MSG6), 100);

   osDelay(TASK_DISPLAY_PERIOD);
  }
  /* USER CODE END displayData */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

