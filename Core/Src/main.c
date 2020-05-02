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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// hardware
#include "potentiometer.h"
#include "motor.h"

// system io
#include "serialPrint.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

Motor_t gLowerMotor;
Motor_t gUpperMotor;

Potentiometer_t gLowerPot;
Potentiometer_t gUpperPot;

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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();

	/* USER CODE BEGIN 2 */

	/* Initialize Motors */
	uartprintf("Initializing Motors ..................... ");

	/* Lower Motor Config
	 * RED -> OUT1
	 * BLACK -> OUT2
	 * */
	gLowerMotor.channel = TIM_CHANNEL_1; // PA8 -> ENA
	gLowerMotor.voltage = 0.0;
	gLowerMotor.dir1 = GPIO_PIN_6; // PA6 -> IN1
	gLowerMotor.dir2 = GPIO_PIN_7; // PA7 -> IN2

	/* Lower Motor Init */
	Motor_init(&gLowerMotor);

	/* Upper Motor
	 * RED -> OUT3
	 * BLACK -> OUT4
	 * */
	gUpperMotor.channel = TIM_CHANNEL_2; // PA9 -> ENB
	gUpperMotor.voltage = 0.0;
	gUpperMotor.dir1 = GPIO_PIN_10; // PA10 -> IN3
	gUpperMotor.dir2 = GPIO_PIN_11; // PA11 -> IN4

	/* Upper Motor Init */
	Motor_init(&gUpperMotor);

	uartprintf("OK\r\n");

	/* Initialize potentiometer values */
	uartprintf("Initializing Potentiometer .............. ");

	/* Lower Potentiometer */
	/* zero: 1895 lsb */
	gLowerPot.cal.zero = 1895;
	/* +90: 3450 lsb */
	uint16_t p90 = 3450;
	float pConv = (float)(p90 - gLowerPot.cal.zero)/90; // lsb/deg
	/* -90: 435 lsb */
	uint16_t m90 = 435;
	float mConv = (float)(gLowerPot.cal.zero - m90)/90; // lsb/deg
	gLowerPot.cal.convFactor = (pConv + mConv)/2;

	/* Upper Potentiometer */
	/* zero: 1360 lsb */
	gUpperPot.cal.zero = 1360;
	/* +90: 2700 lsb */
	p90 = 2700;
	pConv = (float)(p90 - gUpperPot.cal.zero)/90; // lsb/deg
	/* -90: 215 lsb */
	m90 = 215;
	mConv = (float)(gUpperPot.cal.zero - m90)/90; // lsb/deg
	gUpperPot.cal.convFactor = (pConv + mConv)/2;

	uartprintf("OK\r\n");

	float speedDown = 8.0;
	float speedUp = -speedDown;
	float speedStopped = 0;

	float upperPos = -20;
	float lastUpperPos = 0;

	/* time variables */
	uint32_t t = 0;
	uint32_t lastT = HAL_GetTick();
	uint32_t dt = 0;

	uint32_t hist[1024];
	uint32_t i = 0;

	/* errors */
	float pErr = 0;
	float dErr = 0;
	float iErr = 0;

	float pErrHist[1024];

	/* control gains */
	float Kp = 4;

	/* control input */
	float u = 0;

	/* USER CODE END 2 */



	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		/* read potentiometers */
		Potentiometer_read();
		Potentiometer_getPosition(&gLowerPot);
		Potentiometer_getPosition(&gUpperPot);

		/* calculate time step */
		t = HAL_GetTick();
		dt = t - lastT;

		/* calculate errors */
		pErr = upperPos - gUpperPot.pos;
		iErr += pErr * dt/1000;
		dErr = (gUpperPot.pos - lastUpperPos)/(dt/1000);

		/* calculate control input */
		u = Kp * pErr;

		/* check bounds on control input */
		if(u > 12){
			u = 12.0;
		}else if(u < -12){
			u = -12.0;
		}

		/* move arm */
		Motor_setSpeed(&gUpperMotor, u);

		/* update last values */
		lastT = t;
		lastUpperPos = gUpperPot.pos;

		//uartprintf("Timestep: %d\t", dt);
		//uartprintf("Upper Pot Position: %d\r\n", gUpperPot.rawPot);
		HAL_Delay(4);

		hist[i] = dt;
		pErrHist[i] = pErr;
		i++;

		i = i % 1024;


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
