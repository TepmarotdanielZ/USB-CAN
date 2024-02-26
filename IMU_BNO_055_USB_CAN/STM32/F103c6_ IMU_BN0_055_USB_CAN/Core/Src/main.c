/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bno055_stm32.h"
#include "bno055.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


	/* IMU BNO 055 */

		/* EULER */

			bno055_vector_t E;

		/* QUATERNION */

			bno055_vector_t Q;

		/* ACCELEROMETER */

			bno055_vector_t A;

		/* MAGNETORMETER */

			bno055_vector_t M;

		/* CALIBRATION STATE */

			bno055_vector_t C;

		 /* GRAVITY */

			bno055_vector_t Gra;

		 /* GYROSCOPE */

			bno055_vector_t Gyr;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


			/* CAN */

				CAN_RxHeaderTypeDef RxHeader;
				uint8_t RxData[8];
				CAN_TxHeaderTypeDef TxHeader;
				uint8_t TxData[8];
				uint32_t TxMailbox;
				uint16_t ReadValue;

				int datacheck = 0;
				float pwm_M1 = 0;
				float pwm_M2 = 0;

				float RxData1 = 0;
				float RxData2 = 0;
				float RxData3 = 0;
				float RxData4 = 0;

			/* TAGET SPEED OF RGB */

				float V1 = 0;
				float V2 = 0;
				float V3 = 0;
				float V4 = 0;

			/* TAGET SPEED OF MOTOR */

				float M1 ;
				float M2 ;
				float M3;
				float M4 ;

			/* V OUT RGB */

				uint16_t V=255;
				uint16_t V1_out = 0;
				uint16_t V2_out = 0;
				uint16_t V3_out = 0;
				uint16_t V4_out = 0;

			/* M OUT MOTOR */

				uint16_t M_1=255;
				uint16_t M1_out = 0;
				uint16_t M2_out = 0;
				uint16_t M3_out = 0;
				uint16_t M4_out = 0;

				uint8_t flag = 0;
				uint8_t cntt;

			/* EULER */

				uint16_t heading = 0;
				uint16_t pitch   = 0;
				uint16_t roll    = 0;
				uint16_t y =0;

			/* QUATERNION */

				uint16_t QuaX = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* MAP */

	float map(float Input, float Min_Input, float Max_Input, float Min_Output, float Max_Output) {
		return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
	}


/* STM32 INTERUPP RECEIVER FROM USB CAN */

	void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
		cntt++;
		while (cntt - 100 > 0) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			cntt = 0;
		}

	/* DATA RECIEVER FROM USB CAN */

		if (RxHeader.DLC == 8) {
		RxData1 = (RxData[0] << 8) | RxData[1];
		RxData2 = (RxData[2] << 8) | RxData[3];
		RxData3 = (RxData[4] << 8) | RxData[5];
		RxData4 = (RxData[6] << 8) | RxData[7];

	/* TRANFER TO SPEED */

		/* V RGB */

			V1 = RxData1;
			V2 = RxData2;
			V3 = RxData3;
			V4 = RxData4;

		/* M MOTOR */

			M1 = RxData1;
			M2 = RxData2;
			M3 = RxData3;
			M4 = RxData4;

			/* MAP TX DATA FROM 8BIT TO 16BIT (MOTOR ) */

				/* MAPPING MOTOR */

					M1_out = map(M1,0,65535,0,999);
					M2_out = map(M2,0,65535,0,999);

				/* MAPPING RGB ) */

					M3_out = map(M3,0,65535,0,65535);
					M4_out = map(M4,0,65535,0,65535);

					flag = 1;

	}
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* IMU BNO 055 */

	  bno055_assignI2C(&hi2c1);
	  bno055_setup();
	  bno055_setOperationModeNDOF();

  	/* CAN */

		HAL_CAN_Start(&hcan);

	/* STRUCTUR TRANSMITTER DATA */

		HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
		TxHeader.DLC = 8;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.StdId = 0x103; //0b11001010001

	/* TIMER RGB */

		HAL_TIM_Base_Start_IT(&htim2);


		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);


		HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	/* TIMER 3 MOTOR */

		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);


	/* MAP TX DATA FROM 8BIT TO 16BIT (RGB) */

		V1_out = map(V,0,255,0,65535);
		V2_out = map(V,0,255,0,65535);
		V3_out = map(V,0,255,0,65535);
		V4_out = map(V,0,255,0,65535);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* MAKER TIMER INTERRUPP 1mS */

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		if (htim->Instance == TIM2) {

			TIM3->CCR1 = M1_out; // PWM MOTOR
			TIM3->CCR2 = M2_out; // PWM MOTOR
			TIM1->CCR2 = M3_out; // PWM BLINK LED
			TIM1->CCR3 = M4_out; // PWM BLINK LED

		/* TX DATA SEND TO PC MAP 16BIT TO 8BIT COZ DATA CAN 1DATA=8BIT */

			/* EULER */

			 	 E	  = bno055_getVectorEuler();

			 /* QUATERNION */

			 	 Q    = bno055_getVectorQuaternion();

			 /* MAP HEADING (Y) QUATERNION */

				 if (E.y>0)
				 {
					 y = E.y;
				 }
				 else if (E.y<0)
				 {
					 y = 360-(-1*E.y);
				 }

			 /* EULER */

				heading = map (E.x, 0, 360, 0, 65535);
				pitch   = map (E.y, 0, 360, 0, 65535);
				roll    = map (E.z, 0, 360, 0, 65535);

			/* QUATERNION */

				QuaX    = map (Q.x, 0, 360, 0, 65535);

			/* TxData SEND IMU EULER QUATERNION */

				TxData[0] = ((heading  & 0xFF00) >> 8);
				TxData[1] = (heading   & 0x00FF);
				TxData[2] = ((pitch    & 0xFF00) >> 8);
				TxData[3] = (pitch     & 0x00FF);
				TxData[4] = ((roll     & 0xFF00) >> 8);
				TxData[5] = (roll      & 0x00FF);
				TxData[6] = ((-20     & 0xFF00) >> 8);
				TxData[7] = (-20      & 0x00FF);


			if (flag ==1) {
				HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

			}
		}

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
