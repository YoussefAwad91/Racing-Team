/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
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
UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
typedef enum {
  SRC_COUNTER = 0,
  SRC_SHARED  = 1
} DataSource_t;

typedef struct {
  DataSource_t source;
  uint16_t value;
} UARTData_t;

volatile uint8_t counter = 0;
static uint16_t shared = 100;

SemaphoreHandle_t xButtonSemaphore;
QueueHandle_t xUARTDataQueue;
SemaphoreHandle_t xSharedMutex;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (GPIO_Pin == GPIO_PIN_0) /* PA0 */
    {
        counter++;
        if (counter >= 9U) counter = 0U;

        /* Give semaphore from ISR */
        if (xButtonSemaphore != NULL)
        {
            xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  void CalcSwitch1_Task(void *pvParameters);
  void IncreaseVar_Task(void *pvParameters);
  void DecreaseVar_Task(void *pvParameters);
  void UART_Task(void *pvParameters);

  xSharedMutex = xSemaphoreCreateMutex();
  xUARTDataQueue = xQueueCreate(10, sizeof(UARTData_t));
  xButtonSemaphore = xSemaphoreCreateBinary();

  xTaskCreate(CalcSwitch1_Task, "CalcSwitch1", 128, NULL, 1, NULL);
  xTaskCreate(IncreaseVar_Task, "IncreaseVar", 128, NULL, 1, NULL);
  xTaskCreate(DecreaseVar_Task, "DecreaseVar", 128, NULL, 1, NULL);
  xTaskCreate(UART_Task, "UARTTask", 128, NULL, 1, NULL);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
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

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
	void CalcSwitch1_Task(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t waitTicks = 2;
    UARTData_t msg;
    while(1)
    {
        if (xSemaphoreTake(xButtonSemaphore, waitTicks) == pdTRUE)
        {
            uint8_t local_cnt;
            local_cnt = counter;
            msg.source = SRC_COUNTER;
            msg.value = (uint16_t)local_cnt * 10U;
            xQueueSend(xUARTDataQueue, &msg, pdMS_TO_TICKS(10));
        }
    }
}

	void IncreaseVar_Task(void *pvParameters)
	  {
	      (void)pvParameters;
	      const TickType_t period = 10; /* 10 ms */
	      TickType_t lastWake = xTaskGetTickCount();
	      UARTData_t msg;
	      while(1)
	      {
	          vTaskDelayUntil(&lastWake, period);
	          if (xSemaphoreTake(xSharedMutex, pdMS_TO_TICKS(5)) == pdTRUE)
	          {
	              shared++;
	              if (shared > 200U) shared = 100U;
	              msg.source = SRC_SHARED;
	              msg.value = shared;
	              xSemaphoreGive(xSharedMutex);
	              xQueueSend(xUARTDataQueue, &msg, pdMS_TO_TICKS(10));
	          }
	      }
	  }

	  void DecreaseVar_Task(void *pvParameters)
	  {
	      (void)pvParameters;
	      const TickType_t period = 12; /* 12 ms */
	      TickType_t lastWake = xTaskGetTickCount();
	      UARTData_t msg;
	      while(1)
	      {
	          vTaskDelayUntil(&lastWake, period);
	          if (xSemaphoreTake(xSharedMutex, pdMS_TO_TICKS(5)) == pdTRUE)
	          {
	        	  shared--;
	        	  if (shared < 100U) shared = 200U;
	              msg.source = SRC_SHARED;
	              msg.value = shared;
	              xSemaphoreGive(xSharedMutex);
	              xQueueSend(xUARTDataQueue, &msg, pdMS_TO_TICKS(10));
	          }
	      }
	  }


	  void UART_Task(void *pvParameters)
	  {
	      (void)pvParameters;
	      const TickType_t period = 5; /* 5 ms */
	      TickType_t lastWake = xTaskGetTickCount();
	      UARTData_t recv;
	      char outbuf[64];
	      while(1)
	      {
	          vTaskDelayUntil(&lastWake, period);
	          while (xQueueReceive(xUARTDataQueue, &recv, 0) == pdPASS)
	          {
	              if (recv.source == SRC_SHARED)
	                  snprintf(outbuf, sizeof(outbuf), "shared: %u\r\n", (unsigned)recv.value);
	              else
	                  snprintf(outbuf, sizeof(outbuf), "counterx10: %u\r\n", (unsigned)recv.value);
	              HAL_UART_Transmit(&huart1, (uint8_t*)outbuf, strlen(outbuf), HAL_MAX_DELAY);
	          }
	      }
	  }

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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
