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
//TODO: Define variables you think you might need
// - Performance timing variables (e.g execution time, throughput, pixels per second, clock cycles)
#define MAX_ITER 100
int dimensions[] = {128, 160, 192, 224, 256};
uint64_t checksum0;
uint64_t checksum1;
uint64_t checksum2;
uint64_t checksum3;
uint64_t checksum4;
int execution_time0;
int execution_time1;
int execution_time2;
int execution_time3;
int execution_time4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
//TODO: Define any function prototypes you might need such as the calculate Mandelbrot function among others
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //TODO: Visual indicator: Turn on LED0 to signal processing start
  GPIOB->ODR |= 1;

  //TODO: Benchmark and Profile Performance
  int checksums[5];
  int execution_times[5];
  int start_time;
  int end_time;
  int width[] = {256, 500, 1000, 1500, 1920};
  int height[] = {256, 500, 500, 750, 1080};
//  for (int i=0; i<5; i++) {
//	  start_time = HAL_GetTick();
//	  checksums[i] = calculate_mandelbrot_fixed_point_arithmetic(dimensions[4], dimensions[4], max_iter[i]);
//	  end_time = HAL_GetTick();
//	  execution_times[i] = end_time-start_time;
//  }
  start_time = HAL_GetTick();
  checksums[0] = calculate_mandelbrot_fixed_point_arithmetic(width[0], height[0], MAX_ITER);
  end_time = HAL_GetTick();
  execution_times[0] = end_time-start_time;
  checksum0 = checksums[0];
//  checksum1 = checksums[1];
//  checksum2 = checksums[2];
//  checksum3 = checksums[3];
//  checksum4 = checksums[4];
  execution_time0 = execution_times[0];
//  execution_time1 = execution_times[1];
//  execution_time2 = execution_times[2];
//  execution_time3 = execution_times[3];
//  execution_time4 = execution_times[4];

  //TODO: Visual indicator: Turn on LED1 to signal processing start
  GPIOB->ODR |= 2;

  //TODO: Keep the LEDs ON for 2s
  HAL_Delay(2000);

  // TODO: Turn OFF LEDs
  GPIOB->ODR &= ~0xFF;
  while (1)//relocated by the student
  {//relocated by the student


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Function signatures you defined previously , implement them here
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
  uint64_t mandelbrot_sum = 0;
    int64_t s = (1<<20); //scaling factor

    int64_t x_scale = 3.5*s;
    int64_t x_shift = 2.5*s;
    int64_t y_scale = 2.0*s;
    int64_t y_shift = 1.0*s;

    for (int y=0; y<height; y++) {
    	for (int x=0; x<width; x++) {
    		int64_t x0 = (x_scale*x)/width-x_shift;
    		int64_t y0 = (y_scale*y)/height-y_shift;
    		int64_t xi = 0;
    		int64_t yi = 0;
    		int iteration = 0;
    		while (iteration<max_iterations && (xi*xi+yi*yi)/s<=(4*s)) {
    			int64_t temp = (xi*xi-yi*yi)/s;
    			yi = 2*xi*yi/s+y0;
    			xi = temp+x0;
    			iteration++;
    		}
    		mandelbrot_sum += iteration;
    	}
    }

    return mandelbrot_sum;
}

uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations){
  uint64_t mandelbrot_sum = 0;
    for (int y=0; y<height; y++) {
      for (int x=0; x<width; x++) {
    	  double x0 = 3.5*x/width-2.5;
    	  double y0 = 2.0*y/height-1.0;
    	  double xi = 0;
    	  double yi = 0;
    	  int iteration = 0;
    	  while (iteration<max_iterations && (xi*xi+yi*yi)<=4) {
    		  double temp = xi*xi-yi*yi;
    		  yi = 2*xi*yi+y0;
    		  xi = temp+x0;
    		  iteration++;
    	  }
    	  mandelbrot_sum += iteration;
      }
    }

    return mandelbrot_sum;
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
