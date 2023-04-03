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
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "utils.h"
#include "arm_math.h"
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
uint16_t adc_buffer[30] = {0};
#define adc_average(a, b, c,d,e) (((float)((a) + (b) + (c) + (d) + (e))) / 5.0f)
float VBus_Voltage = 0;
float V_battery = 0;
float I_battery = 0;
float duty_inspector=0;
float error_inspector=0;
int counter=0;
int perfom_counter=0;

arm_pid_instance_f32 PID_Voltage = {
    .Kp = -100,
    .Ki = 100,
    .Kd = 0
};

int deadtime_rise = 40;
int deadtime_fall = 40;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float pid_process(arm_pid_instance_f32 *pid, float aim, float current, float out_max, float out_min)
{
  float out = 0;
  
arm_pid_init_f32(pid, 0);
	
	error_inspector=(current - aim);
	
  out = arm_pid_f32(pid, (current - aim));

  // 进行pwm输出限幅

  if (out > out_max)
  {
    out = out_max;
  }
  else if (out < out_min)
  {
    out = out_min;
  }

  return out;
}
float mqtt_fake_value(float V, float I)
{
  return V / I;
}
typedef struct{
	enum{
		MQTT_rising_step_len=1,
		MQTT_falling_step_len=1,
		MQTT_stable_step_len=1,
	}step_len; //步长
	
}MQTT_t;
float mppt_process(){
	
}
float duty = 0.5;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	
  VBus_Voltage = adc_average(adc_buffer[0], adc_buffer[0 + 3], adc_buffer[0 + 2 * 3], adc_buffer[0 + 3 * 3] , adc_buffer[0 + 4 * 3]) * 3.3f / 4096.0f * 21.8184f;
    hhrtim1.Instance->sTimerxRegs[0].CMP1xR =(uint16_t)(pid_process(&PID_Voltage, 30, VBus_Voltage,
                              0.98f * (hhrtim1.Instance->sTimerxRegs[0].PERxR),
                             0.35f * (hhrtim1.Instance->sTimerxRegs[0].PERxR)));

	
//  	 hhrtim1.Instance->sTimerxRegs[0].CMP1xR =duty*65503;
  //  hhrtim1.Instance->sTimerxRegs[1].CMP1xR =\
//													pid_process(&mqtt_pid, 10, mqtt_fake_value(adc_buffer[1],adc_buffer[2]),\
//																					0.98f * (hhrtim1.Instance->sTimerxRegs[0].PERxR),\
//																					0.02f * (hhrtim1.Instance->sTimerxRegs[0].PERxR));

 // HAL_GPIO_TogglePin(check_GPIO_Port, check_Pin);
  //  UART_printf(&huart1, "ADC:%f,%f,%f,%f\n", ((float)adc_buffer[0])*3.3f/4096.0f,((float)adc_buffer[1])*3.3f/4096.0f*1,((float)adc_buffer[2])*3.3f/4096.0f*1,((float)adc_buffer[3])*3.3f/4096.0f*1);
  // UART_printf(&huart1, "ADC:%f,%f\n",3.3f*((float)(adc_buffer[0]))/4096.0f,3.3f*((float)(adc_buffer[3]))/4096.0f);
  // UART_printf(&huart1, "ADC:%f\n", VBus_Voltage);
	
		duty_inspector=(float)hhrtim1.Instance->sTimerxRegs[0].CMP1xR/65503.0f;
	perfom_counter=counter;
	counter=0;
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
  MX_DMA_Init();
  MX_HRTIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	//hrtim A
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
  hhrtim1.Instance->sTimerxRegs[0].PERxR = 65503;
  hhrtim1.Instance->sTimerxRegs[0].CMP1xR = (int)(0.98f * (65503.0f));

	//hrtim D
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_D);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
  hhrtim1.Instance->sTimerxRegs[3].PERxR = 65503;
  hhrtim1.Instance->sTimerxRegs[3].CMP1xR = (int)(0.6f * (65503.0f));

  // TIM+ADC

  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, 3*5);
  TIM2->ARR = 10 - 1;
  TIM2->CCR2 = 2;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_GPIO_WritePin(check_GPIO_Port, check_Pin, 1);

  // Dead time
  HRTIM1->sTimerxRegs[0].DTxR = ((deadtime_fall) << 16) | (deadtime_rise);//chanel A
	HRTIM1->sTimerxRegs[3].DTxR = ((deadtime_fall) << 16) | (deadtime_rise);//chanel D
	
	arm_pid_init_f32(&PID_Voltage, 1);
  while (1)
  {
		counter=counter+1;
    // HRTIM1->sTimerxRegs[0].DTxR = ((deadtime_fall) << 16) | (deadtime_rise);

    // hhrtim1.Instance->sTimerxRegs[0].PERxR = 65503;
    // hhrtim1.Instance->sTimerxRegs[0].CMP1xR = 0.5f*(65503);
    // HAL_GPIO_TogglePin(check_GPIO_Port,check_Pin);
    // HAL_Delay(1000);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
