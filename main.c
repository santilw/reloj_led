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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
uint8_t hora = 0;
uint8_t minuto = 0;
uint8_t segundos = 0;

#define DEBOUNCE_TIME 50

#define SET_MODE 0
#define RUN_MODE 1

uint32_t mode = RUN_MODE;
uint32_t buttonPressedTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void update_LEDs(uint8_t hora, uint8_t minuto, uint8_t segundos) {
    uint8_t hora_1 = hora / 10;
    uint8_t hora_2 = hora % 10;
    uint8_t minuto_1 = minuto / 10;
    uint8_t minuto_2 = minuto % 10;
    uint8_t segundos_1 = segundos / 10;
    uint8_t segundos_2 = segundos % 10;

    HAL_GPIO_WritePin(GPIOA, hora_1, hora_1);
    HAL_GPIO_WritePin(GPIOA, hora_2, hora_2);
    HAL_GPIO_WritePin(GPIOA, minuto_1, minuto_1);
    HAL_GPIO_WritePin(GPIOA, minuto_2, minuto_2);
    HAL_GPIO_WritePin(GPIOA, segundos_1, segundos_1);
    HAL_GPIO_WritePin(GPIOA, segundos_2, segundos_2);
}

void delay_ms(uint32_t ms) {
    uint32_t i;
    for(i=0; i<ms; i++) {
        volatile uint32_t j = 1000;
        while(j--);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	      if (HAL_GPIO_ReadPin(GPIOB, boton_Pin) == 1)
	      {
	        buttonPressedTime += 1;
	      }
	      else
	          {
	            if (buttonPressedTime > DEBOUNCE_TIME)
	            {
	              if (mode == RUN_MODE)
	              {
	                // Cambiar a modo SET
	                HAL_GPIO_WritePin(GPIOA, hora_1_Pin, GPIO_PIN_SET);
	                HAL_Delay(300);
	                HAL_GPIO_WritePin(GPIOA, hora_1_Pin, GPIO_PIN_RESET);
	                HAL_Delay(300);
	                HAL_GPIO_WritePin(GPIOA, hora_1_Pin, GPIO_PIN_SET);
	                HAL_Delay(300);
	                HAL_GPIO_WritePin(GPIOA, hora_1_Pin, GPIO_PIN_RESET);
	                HAL_Delay(300);
	                mode = SET_MODE;
	              }
	              else if (mode == SET_MODE)
	                     {
	                       // Volver a modo RUN
	                       HAL_GPIO_WritePin(GPIOA, hora_1_Pin, GPIO_PIN_RESET);
	                       HAL_Delay(2000);
	                       mode = RUN_MODE;
	                     }
	                     buttonPressedTime = 0;
	                   }
	                 }
	      if (mode == SET_MODE)
	          {
	            uint32_t min = minuto / 5;
	            switch (buttonPressedTime)
	            {
	              case 0 ... DEBOUNCE_TIME - 1:
	                break;
	              case DEBOUNCE_TIME ... 5000 - 1:
	                if (hora < 23)
	                {
	                  hora++;
	                  HAL_GPIO_WritePin(GPIOA, hora_1_Pin, GPIO_PIN_SET);
	                  HAL_Delay(200);
	                  HAL_GPIO_WritePin(GPIOA, hora_1_Pin, GPIO_PIN_RESET);
	                  HAL_Delay(200);
	                }
	                buttonPressedTime = 0;
	                break;
	              case 5000 ... 10000 - 1:
	                        if (minuto < 11)
	                        {
	                          minuto++;
	                          min = minuto * 5;
	                          if (minuto == 11)
	                          {
	                            min = 0;
	                            if (hora < 23)
	                            {
	                              hora++;
	                              HAL_GPIO_WritePin(GPIOA, hora_1_Pin, GPIO_PIN_SET);
	                              HAL_Delay(200);
	                              HAL_GPIO_WritePin(GPIOA, hora_1_Pin, GPIO_PIN_RESET);
	                              HAL_Delay(200);
	                            }
	                          }
	                          else
	                          {
	                            if (minuto == 10)
	                            {
	                            	HAL_GPIO_WritePin(GPIOA, hora_1_Pin, GPIO_PIN_SET);
	                                HAL_Delay(200);
	                            	HAL_GPIO_WritePin(GPIOA, hora_1_Pin, GPIO_PIN_RESET);
	                            	HAL_Delay(200);

	  update_LEDs(hora, minuto, segundos);

	  	        delay_ms(1000);

	  	        segundos++;
	  	        if(segundos == 60) {
	  	            segundos = 0;
	  	            minuto++;
	  	        }
	  	        if(minuto == 60) {
	  	            minuto = 0;
	  	            hora++;
	  	        }
	  	        if(hora == 24) {
	  	            hora = 0;
	  	        }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
	                        }}}}}
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, hora_1_Pin|hora_2_Pin|minuto_1_Pin|minuto_2_Pin
                          |segundos_1_Pin|segundos_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : hora_1_Pin hora_2_Pin minuto_1_Pin minuto_2_Pin
                           segundos_1_Pin segundos_2_Pin */
  GPIO_InitStruct.Pin = hora_1_Pin|hora_2_Pin|minuto_1_Pin|minuto_2_Pin
                          |segundos_1_Pin|segundos_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : boton_Pin buzzer_Pin */
  GPIO_InitStruct.Pin = boton_Pin|buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
