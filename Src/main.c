/* USER CODE BEGIN Header */

/**

 ******************************************************************************

 * @file           : main.c

 * @brief          : Main program body

 ******************************************************************************

 * @attention

 *

 * Copyright (c) 2026 STMicroelectronics.

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

#include "main.h"

/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include <string.h>

#include <stdlib.h>

#include "visualizer.h"

#include "encoder.h"

#include "motor.h"

#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

#define CONTROL_100HZ_PERIOD_US    10000U

#define CONTROL_100HZ_PERIOD_S    0.01f

#define CONTROL_10HZ_PERIOD_US    100000U

#define CONTROL_10HZ_PERIOD_S    0.1f

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;

TIM_HandleTypeDef htim2;

TIM_HandleTypeDef htim3;

TIM_HandleTypeDef htim4;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/*

 * Control scheduler flags.

 *

 * These are set by the (32-bit) timer interrupt and consumed

 * by the main control task.

 */

volatile uint8_t sched_100hz = 0;

volatile uint8_t sched_10hz = 0;

PID_Config_t vel_config = {

        .Kp = 30.0f,

        .Ki = 300.0f,

        .Kd = 0.0f,

        .Ts = 0.01f,

        .output_min = -2000.0f,

        .output_max = 2000.0f

    };

PID_Config_t pos_config = {

        .Kp = 0.01f,

        .Ki = 0.0f,

        .Kd = 0.0f,

        .Ts = 0.1f,

        .output_min = -100.0f,

        .output_max = 100.0f

    };

PID_Controller_t vel;

PID_Controller_t pos;

float setpointPosition = -1000.0f;

float setpointSpeed = 10.0f;

int setpointCounter = 0;

float pos_m = 0.0f;

typedef struct

{

    float alpha;

    float beta;

} Observer_Config_t;

static Observer_Config_t obs_config =

{

	.alpha = 0.9f,

	.beta = 0.05f

};

typedef struct

{

    Observer_Config_t *config;

    float x_hat;

    float v_hat;

    float error;

} VelocityObserver;

VelocityObserver velocity_obs;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_TIM1_Init(void);

static void MX_TIM2_Init(void);

static void MX_TIM3_Init(void);

static void MX_TIM4_Init(void);

static void MX_TIM5_Init(void);

static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

void Send_UART(const char *text);

void Observer_Update(VelocityObserver *obs,

                     float x_meas,

                     float OBS_TS);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)

{

	if (htim->Instance == TIM2) {

        Encoder_CaptureCallback(htim);

    }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

{

	if (htim == &htim4) {

		Encoder_PeriodicUpdate(htim);

		UpdateVelocityEstimate();

	}

}

/* USER CODE END 0 */

/**

  * @brief  The application entry point.

  * @retval int

  */

int main(void)

{

  /* USER CODE BEGIN 1 */

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

  MX_TIM1_Init();

  MX_TIM2_Init();

  MX_TIM3_Init();

  MX_TIM4_Init();

  MX_TIM5_Init();

  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

 Encoder_Init(&htim2, &htim3, &htim4, 0.8f, 5.0f, 1);

 Motor_Init(&htim1);

 PID_Init(&vel, &vel_config);

 PID_SetSetpoint(&vel, setpointSpeed);

 PID_Init(&pos, &pos_config);

 //PID_SetSetpoint(&pos, setpointPosition);

 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

 Motor_Set_Pwm(MOTOR_DIR_Pin,

  	 				MOTOR_DIR_GPIO_Port,

  					0,

  	 							1,

								TIM_CHANNEL_1);

 HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);

 HAL_TIM_Base_Start(&htim3);

 HAL_TIM_Base_Start_IT(&htim4);

 /*

  * ----------------------------------------------------------

  * Start TIM5

  * ----------------------------------------------------------

  *

  * TIM5 runs continuously at 1 MHz.

  */

 HAL_TIM_Base_Start(&htim5);

 /* Read current counter value */

 uint32_t now = __HAL_TIM_GET_COUNTER(&htim5);

 /*

  * Schedule first events.

  *

  * Channel 1 -> 100 Hz

  * Channel 2 -> 10 Hz

  */

 __HAL_TIM_SET_COMPARE(

	 &htim5,

	 TIM_CHANNEL_3,

	 now + CONTROL_100HZ_PERIOD_US

 );

 __HAL_TIM_SET_COMPARE(

	 &htim5,

	 TIM_CHANNEL_4,

	 now + CONTROL_10HZ_PERIOD_US

 );

 /* Enable output compare interrupts */

 HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_3);

 HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_4);

 const char *visualizer_names[] =

   {

       "setpoint",

	   "motor velocity",

	   "obs. velocity",

   };

   Visualizer_Init(

       &huart1,

       3,

       visualizer_names

   );

    PID_Update(&vel, Encoder_GetVelocity());

    Motor_Set_Pwm(MOTOR_DIR_Pin,

     	 				MOTOR_DIR_GPIO_Port,

						(int16_t)vel.output,

     	 							1,

									TIM_CHANNEL_1);

    velocity_obs.config = &obs_config;

    velocity_obs.x_hat = 0.0f;

    velocity_obs.v_hat = 0.0f;

    velocity_obs.error = 0.0f;

  /* USER CODE END 2 */

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

 while (1)

 {

	 if (sched_100hz)

	 	      {

	 	          sched_100hz = 0;

		 	          setpointCounter++;

		 	         if (setpointCounter % 300 == 0) {

		 	             setpointSpeed = -setpointSpeed;

		 	             PID_SetSetpoint(&vel, setpointSpeed);

		 	         }

	 	          // 100 Hz task

		 	      pos_m = Encoder_GetCount() / 390.0f;

		 	      Observer_Update(&velocity_obs, pos_m, 0.01f);

	 	          //PID_Update(&vel, Encoder_GetVelocity());

		 	      PID_Update(&vel, velocity_obs.v_hat);

	 	          Motor_Set_Pwm(MOTOR_DIR_Pin,

	 	           	 				MOTOR_DIR_GPIO_Port,

									(int16_t)vel.output,

	 	           	 							1,

												TIM_CHANNEL_1);

	 	      }

	 	      if (sched_10hz)

	 	      {

	 	          sched_10hz = 0;

	 	        /*  setpointCounter++;

	 	         if (setpointCounter % 30 == 0) {

	 	             setpointPosition = -setpointPosition;

	 	             PID_SetSetpoint(&pos, setpointPosition);

	 	         }*/

	 	    	  int32_t values[3];

	 	    	  values[0] = (int32_t)(setpointSpeed * 1000);

				  values[1] = (int32_t)(Encoder_GetVelocity() * 1000);

				  values[2] = (int32_t)(velocity_obs.v_hat * 1000);

				  Visualizer_Send(

					  values,

					  3,

					  100 * 1000

				  );

				  // 10 Hz task

				  //PID_Update(&pos, Encoder_GetCount());

				  //PID_SetSetpoint(&vel, pos.output);*/

	 	      }

	 	//if (control_loop_flag == 1) {

	 		//UpdateVelocityEstimate();

	 	//}

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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

  RCC_OscInitStruct.HSIState = RCC_HSI_ON;

  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

  RCC_OscInitStruct.PLL.PLLM = 8;

  RCC_OscInitStruct.PLL.PLLN = 50;

  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;

  RCC_OscInitStruct.PLL.PLLQ = 4;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != *HAL_OK*)

  {

    Error_Handler();

  }

  /** Initializes the CPU, AHB and APB buses clocks

  */

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK

                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != *HAL_OK*)

  {

    Error_Handler();

  }

}

/**

  * @brief TIM1 Initialization Function

  * @param None

  * @retval None

  */

static void MX_TIM1_Init(void)

{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  TIM_OC_InitTypeDef sConfigOC = {0};

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */

  htim1.Instance = TIM1;

  htim1.Init.Prescaler = 0;

  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;

  htim1.Init.Period = 2499;

  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim1.Init.RepetitionCounter = 0;

  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim1) != *HAL_OK*)

  {

    Error_Handler();

  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != *HAL_OK*)

  {

    Error_Handler();

  }

  if (HAL_TIM_PWM_Init(&htim1) != *HAL_OK*)

  {

    Error_Handler();

  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != *HAL_OK*)

  {

    Error_Handler();

  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;

  sConfigOC.Pulse = 0;

  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;

  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;

  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;

  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != *HAL_OK*)

  {

    Error_Handler();

  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;

  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;

  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;

  sBreakDeadTimeConfig.DeadTime = 0;

  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;

  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;

  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != *HAL_OK*)

  {

    Error_Handler();

  }

  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */

  htim2.Instance = TIM2;

  htim2.Init.Prescaler = 0;

  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;

  htim2.Init.Period = 4294967295;

  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;

  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;

  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;

  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;

  sConfig.IC1Filter = 0;

  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;

  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;

  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;

  sConfig.IC2Filter = 0;

  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != *HAL_OK*)

  {

    Error_Handler();

  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != *HAL_OK*)

  {

    Error_Handler();

  }

  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */

  htim3.Instance = TIM3;

  htim3.Init.Prescaler = 49;

  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;

  htim3.Init.Period = 65535;

  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim3) != *HAL_OK*)

  {

    Error_Handler();

  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != *HAL_OK*)

  {

    Error_Handler();

  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != *HAL_OK*)

  {

    Error_Handler();

  }

  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */

  htim4.Instance = TIM4;

  htim4.Init.Prescaler = 49;

  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;

  htim4.Init.Period = 9999;

  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim4) != *HAL_OK*)

  {

    Error_Handler();

  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != *HAL_OK*)

  {

    Error_Handler();

  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != *HAL_OK*)

  {

    Error_Handler();

  }

  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */

  htim5.Instance = TIM5;

  htim5.Init.Prescaler = 49;

  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;

  htim5.Init.Period = 4294967295;

  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim5) != *HAL_OK*)

  {

    Error_Handler();

  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != *HAL_OK*)

  {

    Error_Handler();

  }

  if (HAL_TIM_OC_Init(&htim5) != *HAL_OK*)

  {

    Error_Handler();

  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != *HAL_OK*)

  {

    Error_Handler();

  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;

  sConfigOC.Pulse = 0;

  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;

  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != *HAL_OK*)

  {

    Error_Handler();

  }

  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != *HAL_OK*)

  {

    Error_Handler();

  }

  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

  HAL_TIM_MspPostInit(&htim5);

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

  huart1.Init.BaudRate = 115200;

  huart1.Init.WordLength = UART_WORDLENGTH_8B;

  huart1.Init.StopBits = UART_STOPBITS_1;

  huart1.Init.Parity = UART_PARITY_NONE;

  huart1.Init.Mode = UART_MODE_TX_RX;

  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;

  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != *HAL_OK*)

  {

    Error_Handler();

  }

  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**

  * @brief GPIO Initialization Function

  * @param None

  * @retval None

  */

static void MX_GPIO_Init(void)

{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */

  __HAL_RCC_GPIOA_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */

  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, *GPIO_PIN_RESET*);

  /*Configure GPIO pin : MOTOR_DIR_Pin */

  GPIO_InitStruct.Pin = MOTOR_DIR_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(MOTOR_DIR_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */

}

/* USER CODE BEGIN 4 */

/**

 * @brief TIM5 output compare callback for control loop timing

 */

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)

{

    if (htim->Instance != TIM5)

        return;

    if (htim->Channel == *HAL_TIM_ACTIVE_CHANNEL_3*)

    {

         /* Schedule next 500 Hz event.

         * IMPORTANT:

         * Add to the previous compare value rather than

         * using CNT + period. This eliminates accumulated

         * scheduler drift.

         * */

        __HAL_TIM_SET_COMPARE(

            &htim5,

            TIM_CHANNEL_3,

            __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_3)

                \+ CONTROL_100HZ_PERIOD_US

        );

        sched_100hz = 1;

    }

    else if (htim->Channel == *HAL_TIM_ACTIVE_CHANNEL_4*)

    {

         //Schedule next 100 Hz event

        __HAL_TIM_SET_COMPARE(

            &htim5,

            TIM_CHANNEL_4,

            __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_4)

                \+ CONTROL_10HZ_PERIOD_US

        );

        sched_10hz = 1;

    }

}

void Observer_Update(VelocityObserver *obs,

                     float x_meas,

                     float OBS_TS)

{

    if (obs == NULL || obs->config == NULL || OBS_TS <= 0.0f)

        return;

    // Position prediction error

    obs->error = x_meas - obs->x_hat;

	// Prediction step

	float x_pred = obs->x_hat + OBS_TS * obs->v_hat;

	float v_pred = obs->v_hat;

	// Innovation (measurement residual)

	float residual = x_meas - x_pred;

	// Correction step

	obs->x_hat = x_pred +

				 obs->config->alpha * residual;

	obs->v_hat = v_pred +

				 (obs->config->beta / OBS_TS) * residual;

}

/* USER CODE END 4 */

/**

  * @brief  This function is executed in case of error occurrence.

  * @retval None

  */

void Error_Handler(void)

{

  /* USER CODE BEGIN Error_Handler_Debug */

 /* User can add his own implementation to report the HAL error return state */

 __disable_irq();

 while (1)

 {

 }

  /* USER CODE END Error_Handler_Debug */

}

#ifdef USE_FULL_ASSERT

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

    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* USER CODE END 6 */

}

#endif /* USE_FULL_ASSERT */
