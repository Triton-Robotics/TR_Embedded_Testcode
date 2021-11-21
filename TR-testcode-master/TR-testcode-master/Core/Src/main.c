/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include "rm_uart.h"
#include "rm_gpio.h"
#include "rm_can.h"
#include "rm_imu.h"
#include "rm_pwm.h"
#include "rm_ctrl.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INF_2
//#define SENTRY

// default positions
//#define YAW_ECD_DEFAULT (6115)
//#define PIT_ECD_DEFAULT (5350)
#define YAW_ECD_DEFAULT (291)
#define PIT_ECD_DEFAULT (3056)
#define INDEXER_SPEED_DEFAULT 125
#define INIT_TIMER_VALUE 15000
#define YAW_OSCILLATION 750
#define PIT_OSCILLATION 1000
#define PIXEL_TO_ESC 1000


#define DRIVE_LF_ID 1
#define DRIVE_LB_ID 2
#define DRIVE_RF_ID 3
#define DRIVE_RB_ID 4
#define RPM_SCALE 10
#define JOY_TO_RPM (1 / 660.0f) // max 660, 660 / 660.0f = 1 rpm
#define JOY_TO_ECD (4096 / 660.0f) // max 660, 660 * (4096 / 660.0f = 4096
#define DRIVE_SPEED_DEFAULT 450 // 450 rpm
#define FLYWHEEL_OFF_PERCENT (0.4f)
#define FLYWHEEL_ON_PERCENT (0.417f)
#define MOTOR_BOUNDS 4500
#define GIMBAL_RPM_LIMIT 3200

#define DELAY_MS 5
#define PI 3.14159265f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
char buf[200];
int count;

double minDistance, maxDistance, centerDistance;
unsigned long start_t, current_t, currentCount;

short sentryInit = 0;
short timerInit = 0;
short cvDetected = 0;

double cvX, cvY;
typedef struct {
	struct {
		short LF_rpm,LB_rpm,RF_rpm,RB_rpm;
	} chassis;
	struct {
		short yaw_ecd, pitch_ecd;
		short yaw_rpm, pit_rpm;
		short flywheel_speed;
		float indexer_speed;
		short yaw_ecd_target;
		short pit_ecd_target;
	} gimbal;
	short isKB;
} signal_t;
float radian;
float x;
float y;

signal_t signal;

float imu_yaw_offset = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void processMKB();
void processController();
void processSentry();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void signalInit(signal_t* sig){
	sig->isKB = 0;
	sig->gimbal.yaw_ecd_target = YAW_ECD_DEFAULT;
	sig->gimbal.pit_ecd_target = PIT_ECD_DEFAULT;
}
void processSentry(signal_t* sig){

	current_t = HAL_GetTick();
	double currentESCVal = 0;
	count = 0;
	if (!sentryInit){

		minDistance = 0; //Replace with DOUBLE.MAXVAL
		maxDistance = 0; //Replace with DOUBLE.MINVAL
		cvX = 0;
		cvY = 0;

		if (!timerInit && !sentryInit){
			start_t = HAL_GetTick();
			timerInit = 1;
		}

		//Calibration period for INIT_TIMER_VALUE milliseconds
		if (HAL_GetTick() - start_t < INIT_TIMER_VALUE){


			if (currentESCVal > maxDistance){
				maxDistance = currentESCVal;
			}
			else if (currentESCVal < minDistance){
				minDistance = currentESCVal;
			}
		}
		else {
			sentryInit = 1;
			timerInit = 0;
			centerDistance = maxDistance + minDistance;
		}
	}
	else {
		if (cvDetected){
			sig->gimbal.yaw_ecd = PIXEL_TO_ESC * cvX;
			sig->gimbal.pitch_ecd = PIXEL_TO_ESC * cvY;
		}
		else {

			//sig->gimbal.yaw_ecd = (short)((sin(current_t / 175) * YAW_OSCILLATION));
			sig->gimbal.pitch_ecd = (short)((sin(current_t / 225) * PIT_OSCILLATION) / 3);

			if (current_t % 5000 < 2500){
				sig->gimbal.yaw_ecd = 0;
			}
			else {
				sig->gimbal.yaw_ecd = -YAW_OSCILLATION;
			}

			if (current_t % 12000 < 3000){
				sig->chassis.LF_rpm = (short) (75 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
				sig->chassis.RF_rpm = (short) (75 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
				sig->chassis.LB_rpm = (short) (75 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
				sig->chassis.RB_rpm = (short) (75 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
			}
			else if (current_t % 12000 > 3000 && current_t % 12000 < 6000){
				sig->chassis.LF_rpm = (short) (0 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
				sig->chassis.RF_rpm = (short) (0 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
				sig->chassis.LB_rpm = (short) (0 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
				sig->chassis.RB_rpm = (short) (0 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
			}
			else if (current_t % 12000 > 6000 && current_t % 12000 < 10000){
				sig->chassis.LF_rpm = (short) (-110 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
				sig->chassis.RF_rpm = (short) (-110 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
				sig->chassis.LB_rpm = (short) (-110 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
				sig->chassis.RB_rpm = (short) (-110 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
			}
			else{
				sig->chassis.LF_rpm = (short) (0 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
				sig->chassis.RF_rpm = (short) (0 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
				sig->chassis.LB_rpm = (short) (0 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
				sig->chassis.RB_rpm = (short) (0 * JOY_TO_RPM * RPM_SCALE * DRIVE_SPEED_DEFAULT);
			}
		}
	}
	currentCount++;
}

void processCV(){

}

void processMKB(rc_info_t* rcPtr, signal_t* sig){
	// Axis y-> up, x -> right, motors going forward when pos (left CCW, right CW)
	short kbX = rcPtr->kb.bit.D - rcPtr->kb.bit.A;
	short kbY = rcPtr->kb.bit.W - rcPtr->kb.bit.S;
	short kbRotation = rcPtr->kb.bit.E - rcPtr->kb.bit.Q;
	short mouseLC = rcPtr->mouse.l;
	short mouseRC = rcPtr->mouse.r;
	short mouseSpeedX = rcPtr->mouse.x;
	short mouseSpeedY = rcPtr->mouse.y;

	sig->gimbal.yaw_rpm = fmax(fmin(mouseSpeedX * 10, GIMBAL_RPM_LIMIT), -GIMBAL_RPM_LIMIT);
	sig->gimbal.pit_rpm = fmax(fmin(mouseSpeedY * 10, GIMBAL_RPM_LIMIT), -GIMBAL_RPM_LIMIT);

	sig->gimbal.yaw_ecd += mouseSpeedX * 10;
	sig->gimbal.pitch_ecd += mouseSpeedY * 10;

	if (mouseRC) {
		sig->gimbal.flywheel_speed = (short) (PWM_RESOLUTION * FLYWHEEL_ON_PERCENT);
	} else {
		sig->gimbal.flywheel_speed = (short) (PWM_RESOLUTION * FLYWHEEL_OFF_PERCENT);
	}

	if (mouseLC) {
		sig->gimbal.indexer_speed = INDEXER_SPEED_DEFAULT * RPM_SCALE;
	} else {
		sig->gimbal.indexer_speed = 0 * RPM_SCALE;
	}

	sig->chassis.LF_rpm = (short) ((- kbY + kbX + kbRotation) * RPM_SCALE * DRIVE_SPEED_DEFAULT);
	sig->chassis.RF_rpm = (short) ((kbY + kbX + kbRotation) * RPM_SCALE * DRIVE_SPEED_DEFAULT);
	sig->chassis.LB_rpm = (short) ((- kbY - kbX + kbRotation) * RPM_SCALE * DRIVE_SPEED_DEFAULT);
	sig->chassis.RB_rpm = (short) ((kbY - kbX + kbRotation) * RPM_SCALE * DRIVE_SPEED_DEFAULT);
	
}

void processController(rc_info_t* rcPtr, signal_t* sig){
	// Axis y-> up, x -> right, motors going forward when pos (left CCW, right CW)
	if (rcPtr->sw1 == 2 && rcPtr->sw2 == 2){
		if (!sig->isKB){ // last time not kb
			// transition from controller to KB
			gimbal_pid_clear();
		}
		sig->isKB = 1;
		return processMKB(rcPtr, sig);
	}
	if (sig->isKB) { // last time kb
		// transition from KB to controller
		gimbal_pid_clear();
	}
	sig->isKB = 0;
	//yaw_ecd = 0;
	//pitch_ecd = 0;
	short joyLeftX = (short)(rcPtr->ch3); // positive direction stay at right
	short joyLeftY = (short)(rcPtr->ch4); // change positive direction to up
	short joyRightX = (short)(rcPtr->ch1); // positive direction stay at right
	short joyRightY = (short)(-rcPtr->ch2); // change positive direction to up
	short baybladeRPM = DRIVE_SPEED_DEFAULT / 2;
	short baybladeRotation = 0;
	// hard coded compensation for yaw motor to rotate
	float compensation = (23.2f / (60.0f * 1000.0f / DELAY_MS)) * ECD_PERIOD;
	switch(rcPtr->sw1){
		case 1: //left up (left stick strafe, right stick bot rotation)
			//joyRotation = joyRightX;
			baybladeRotation = - baybladeRPM;
			sig->gimbal.yaw_ecd -= (short)(joyRightX * JOY_TO_ECD * 0.05f + compensation);
			sig->gimbal.indexer_speed = 0 * RPM_SCALE;
			break;
		case 3: //left middle (left stick strafe, right stick aim)
			sig->gimbal.yaw_ecd -= (short)(joyRightX * JOY_TO_ECD * 0.05f);
			sig->gimbal.indexer_speed = 0 * RPM_SCALE;
			break;
		case 2: // reserve for keyboard
			baybladeRotation = baybladeRPM;
			sig->gimbal.yaw_ecd -= (short)(joyRightX * JOY_TO_ECD * 0.05f - compensation);
			if (rcPtr->sw2 == 1){ // flywheel on
				//sig->gimbal.yaw_ecd += (short)(joyRightX * JOY_TO_ECD * 0.05f);
				sig->gimbal.indexer_speed = INDEXER_SPEED_DEFAULT * RPM_SCALE;
			}
			break;
		default: // when remote not connected
			sig->gimbal.yaw_ecd -= (short)(joyRightX * JOY_TO_ECD * 0.05f);
			sig->gimbal.indexer_speed = 0 * RPM_SCALE;

	}
	switch(rcPtr->sw2){
		case 1:
			sig->gimbal.flywheel_speed = (short) (PWM_RESOLUTION * FLYWHEEL_ON_PERCENT);
			break;
		case 3:
			sig->gimbal.flywheel_speed = (short) (PWM_RESOLUTION * FLYWHEEL_OFF_PERCENT);
			sig->gimbal.indexer_speed = 0 * RPM_SCALE;
			break;
		case 2: // reserve for keyboard
		default: // when remote not connected
			break;

	}
	sig->gimbal.pitch_ecd += (short)((joyRightY * JOY_TO_ECD) * 0.05f); // pitch don't have the same range as yaw, now limit to 20 degrees
	float raw_x = joyLeftX * JOY_TO_RPM * DRIVE_SPEED_DEFAULT;
	float raw_y = joyLeftY * JOY_TO_RPM * DRIVE_SPEED_DEFAULT;

	// change of angle from the original position
	radian = sig->gimbal.yaw_ecd / ECD_PERIOD * 2 * PI;

	// apply rotational matrix
	x = (float) (raw_x * cos(radian) - raw_y * sin(radian));
	y = (float) (raw_x * sin(radian) + raw_y * cos(radian));

	sig->chassis.LF_rpm = (short) ((y + x + baybladeRotation) * RPM_SCALE);
	sig->chassis.RF_rpm = (short) ((-y + x + baybladeRotation) * RPM_SCALE);
	sig->chassis.LB_rpm = (short) ((y - x + baybladeRotation) * RPM_SCALE);
	sig->chassis.RB_rpm = (short) ((-y - x + baybladeRotation) * RPM_SCALE);

}

void outINF(signal_t* sig, CAN_HandleTypeDef *hcan) {
	static int imu_zeroed = 0;
	// output variables
	float yaw_output;
	float pit_output;
	float wheels_output[4];
	float indexer_output;

	signal.gimbal.yaw_ecd_target = mapPeriod(
			YAW_ECD_DEFAULT + signal.gimbal.yaw_ecd, ECD_PERIOD);

	signal.gimbal.pit_ecd_target = mapPeriod(
			PIT_ECD_DEFAULT + signal.gimbal.pitch_ecd, ECD_PERIOD);

	if (signal.isKB){
		//yaw_output = yaw_rpm_pid_ctrl(yaw_rpm);
		//pit_output = pit_rpm_pid_ctrl(pit_rpm);
		//yaw_output = yaw_ecd_cascade_ctrl(yaw_ecd_target);
		//pit_output = pit_ecd_cascade_ctrl(pit_ecd_target);
		yaw_output = yaw_ecd_direct_ctrl(signal.gimbal.yaw_ecd_target);
		pit_output = pit_ecd_direct_ctrl(signal.gimbal.pit_ecd_target);
	} else {
		//yaw_output = yaw_ecd_cascade_ctrl(signal.gimbal.yaw_ecd_target);
		//pit_output = pit_ecd_cascade_ctrl(signal.gimbal.pit_ecd_target);
		int yaw_centered = fabs(motors[4].ecd - YAW_ECD_DEFAULT) < 1;
		if (!imu_zeroed){
			yaw_output = yaw_ecd_direct_ctrl(YAW_ECD_DEFAULT);
			if (yaw_centered){
				imu_yaw_offset = YAW_ECD_DEFAULT * (ANGLE_PERIOD / ECD_PERIOD) - imu.yaw;
			}
		} else {
			float deg_target = signal.gimbal.yaw_ecd_target * (ANGLE_PERIOD / ECD_PERIOD);
			yaw_output = yaw_imu_deg_ctrl(imu.yaw + imu_yaw_offset, deg_target);
		}

		pit_output = pit_ecd_direct_ctrl(signal.gimbal.pit_ecd_target);
	}

	sig->chassis.LF_rpm = fmax(fmin(sig->chassis.LF_rpm,MOTOR_BOUNDS),-MOTOR_BOUNDS);
	sig->chassis.RF_rpm = fmax(fmin(sig->chassis.RF_rpm,MOTOR_BOUNDS),-MOTOR_BOUNDS);
	sig->chassis.LB_rpm = fmax(fmin(sig->chassis.LB_rpm,MOTOR_BOUNDS),-MOTOR_BOUNDS);
	sig->chassis.RB_rpm = fmax(fmin(sig->chassis.RB_rpm,MOTOR_BOUNDS),-MOTOR_BOUNDS);

	wheels_rpm_ctrl_calc(
			signal.chassis.LF_rpm,
			signal.chassis.RF_rpm,
			signal.chassis.LB_rpm,
			signal.chassis.RB_rpm,
			wheels_output);

	indexer_output = indexer_rpm_ctrl_calc(signal.gimbal.indexer_speed);

	can_transmit(hcan, CAN_CHASSIS_ALL_ID,
			wheels_output[0],
			wheels_output[1],
			wheels_output[2],
			wheels_output[3]);

	can_transmit(hcan, CAN_GIMBAL_ALL_ID,
			yaw_output,
			pit_output,
			indexer_output, 0);
	//flywheel_speed = (rc.ch4 / 660.0f + 1) * 1000;
	set_pwm_flywheel(signal.gimbal.flywheel_speed);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  count = 0; // keep count in while loop and reset every 10s
  signalInit(&signal);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI5_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  led_off();
  power_on();
  dbus_uart_init();
  //can_filter_init();
  pwm_imu_start();
  pwm_buzzer_start();

  mpu_device_init();
  init_quaternion();

  pwm_flywheel_start();

  grand_pid_init();
  //imu_calibration();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	mpu_get_data();
	imu_ahrs_update();
	imu_attitude_update();
	imu_temp_pid_ctrl(imu.temp, DESIRED_TEMPERATURE);

	// process the inputs from the rc into targets for each pid module
	//processSentry(&signal);
	processController(&rc, &signal);
	processCV();

	//outINF(&signal, &hcan1);

	if (count%100 == 0)
	{
		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	}

	/* reset count every 10s */
	if (count == 2000)
	{
		count = 0;
	}
	count++;

	HAL_Delay(DELAY_MS);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = TIM_PSC_APB1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = PWM_RESOLUTION-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_DEFAULT_DUTY;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = TIM_PSC_APB1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = PWM_RESOLUTION-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_DEFAULT_DUTY;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 3;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 7777;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 100000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, IST_INT_Pin|IST_RST_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IST_INT_Pin IST_RST_Pin */
  GPIO_InitStruct.Pin = IST_INT_Pin|IST_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PH2 PH3 PH4 PH5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PF6 LED_GREEN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
