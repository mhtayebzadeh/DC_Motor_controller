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
#define ENC_PULSE_PER_ROUND 6125
#define PI 3.14159
#define ENC_PULSE_2_RAD 0.00102582 // 2*PI/ENC_PULSE_PER_ROUND

#define CURRENT_CONTROL_MODE 1
#define POSITION_CONTROL_MODE 2
#define SPEED_CONTROL_MODE 3

#define TOTAL_PACKET_SIZE 8

#define LOADCELL_DATA_ID 1

#define SET_MOTOR_POSITION_SP_ID 21
#define SET_MOTOR_SPEED_SP_ID 22
#define SET_MOTOR_CURRENT_SP_ID 23
#define SET_MOTOR_EMERGENCY_STOP_ID 24
#define EMERGENCY_STOP_MOTOR_ID 27

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint32_t adc_buff[2];
float vsense = 3.3/4095;

double MOTOR_SIGN = 1;
double I_motor_acs712 , I_motor_MAX4080 =0;
double I_motor_acs712_filtered , I_motor_MAX4080_filtered =0 , alpha_adc_acs = 0.8 ,alpha_adc_max4080 = 0.6;

int32_t encoder_pulse = 0;
double sp_pos_rad , sp_speed , sp_current;
double encoder_rad = 0 , encoder_rad_last , encoder_speed , encoder_speed_filtered , encoder_speed_last;
double alpha_speed = 0.8;
uint32_t dt = 0, last_t1 = 0; 

volatile uint8_t convCompleted = 0;

double motor_voltage_dir = 1.0;
int32_t pwm_i = 0;
uint16_t p1,p2;
float p = 0;

double control_sigal,ff_curr=0.5,err_curr_dot,pid_out,last_err_curr,err_curr_dot,err_curr,fb_curr,sum_err_curr=0 , current_sign=1;
double K_d_curr = 0.0 ,K_p_curr = 2.5 ,K_i_curr = 0.1;

double err_pos,err_pos_dot,sum_err_pos,ff_pos ;
double K_p_pos = 10 , K_d_pos = 0.1 , K_i_pos = 0;

double err_speed , err_speed_dot , last_err_speed , sum_err_speed , ff_speed = 0 ; 
double K_p_speed = 2.0 , K_d_speed = 0.00 , K_i_speed = 2.0;




uint8_t UART1_rxBuffer[TOTAL_PACKET_SIZE], buff_1[TOTAL_PACKET_SIZE];
int is_uart1_single_byte_recieved = 1;
int32_t i, ii , i2;



double POWER_SUPLY_MAX_VOLTAGE = 16.0;
uint8_t control_mode = CURRENT_CONTROL_MODE;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_12) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
			if(HAL_GPIO_ReadPin(Enc_B_GPIO_Port , Enc_B_Pin) == 1)
				encoder_pulse++;
			else
				encoder_pulse--;
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	I_motor_acs712 = 0.004959*((double)adc_buff[0]) -13.61;		// ASC is faster but not precise
	I_motor_MAX4080 = 0.0008734*((double)adc_buff[1]) -0.01224 ;	// MAX4080 is more precise but has RC filter
	I_motor_acs712_filtered = alpha_adc_acs*I_motor_acs712_filtered + (1.0 - alpha_adc_acs)*I_motor_acs712;
	I_motor_MAX4080_filtered = alpha_adc_max4080*I_motor_MAX4080_filtered + (1.0 - alpha_adc_max4080)*I_motor_MAX4080;
}

double saturation_f(double a , double min_ , double max_)
{
	if (a > max_)
		return max_;
	else if (a < min_)
		return  min_;
	else 
		return a;
}
int32_t saturation_i(int32_t a , int32_t min_ , int32_t max_)
{
	if (a > max_)
		return max_;
	else if (a < min_)
		return  min_;
	else 
		return a;
}
double abs_f(double a)
{
		if (a<0)
			return -a;
		return a;
}

void set_motor_percent(double p )	// p --> -100 to 100
{
		if (p > 0)
			motor_voltage_dir = 1;
		else if (p < 0)
			motor_voltage_dir = -1;
		
	double pwm = p*2.0;		// p --> -200 to 200
	if(pwm > 0){
			pwm_i = (int32_t)pwm;
			pwm_i = saturation_i(pwm_i , 0 , 200);	// PWM  0 --> 200
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1 , pwm_i);		// R_PWM  0 --> 200
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2 , 0);		// L_PWM  0 --> 200
	}else{
		pwm_i = (int32_t)-pwm;
		pwm_i = saturation_i(pwm_i , 0 , 200);	// PWM  0 --> 200
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1 , 0);		// R_PWM  0 --> 200
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2 , pwm_i);		// L_PWM  0 --> 200
	}
}
void set_motor_voltage(double v)		// voltage to percent 
{
	set_motor_percent(100.0*v/POWER_SUPLY_MAX_VOLTAGE);
}	

void set_motor_enable(int en)
{
	if (en == 1)
	{
		HAL_GPIO_WritePin(Motor_LEn_GPIO_Port , Motor_LEn_Pin , GPIO_PIN_SET);
		HAL_GPIO_WritePin(Motor_REn_GPIO_Port , Motor_REn_Pin , GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(Motor_LEn_GPIO_Port , Motor_LEn_Pin , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Motor_REn_GPIO_Port , Motor_REn_Pin , GPIO_PIN_RESET);
	}
}

void push_to_arr(uint8_t* arr , uint8_t  len , uint8_t c)
{
	for(int i = 0; i<len-1 ;i++)
	{
		arr[i] = arr[i+1];
	}
	arr[len-1] = c;
}
uint8_t check_packet(uint8_t* buf)
{
	if (buf[0]==0xAA && buf[1]==0xAA && buf[7]==0xBB)
		return 1;
	else
		return 0;
}
void read_packet_data(uint8_t* buf)
{
	uint8_t command = buf[2] ;
	int32_t value = (int32_t)((buf[3]<<24) | (buf[4]<<16) | (buf[5]<<8) | (buf[6]));
	if (command == SET_MOTOR_POSITION_SP_ID)
	{
		control_mode = POSITION_CONTROL_MODE;
		sp_pos_rad = ((double)(value))/1000.0;
	}
	else if(command == SET_MOTOR_SPEED_SP_ID)
	{
		control_mode =  SPEED_CONTROL_MODE;
		sp_speed = ((double)(value))/1000.0;
	}
	else if(command == SET_MOTOR_CURRENT_SP_ID)
	{
		control_mode = CURRENT_CONTROL_MODE;
		sp_current = ((double)(value))/1000.0;
	}
	else if(command == EMERGENCY_STOP_MOTOR_ID)
	{
		set_motor_voltage(0);
		set_motor_enable(0);
	}
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if (huart->Instance == USART1) { 		// MCU
		if (is_uart1_single_byte_recieved)
		{
			
			push_to_arr(buff_1 , TOTAL_PACKET_SIZE , UART1_rxBuffer[0]);

			if (check_packet(buff_1))	// correct packet recieved
			{	
				read_packet_data(buff_1);
				is_uart1_single_byte_recieved = 0;
				HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, TOTAL_PACKET_SIZE);
				return ;
			}
			is_uart1_single_byte_recieved = 1;
			HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 1);
		} 
		else {
			if (check_packet(UART1_rxBuffer))	// correct packet recieved
			{	
				i++;
				read_packet_data(UART1_rxBuffer);
				HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, TOTAL_PACKET_SIZE);
				return ;
			}
			is_uart1_single_byte_recieved = 1;
			HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 1);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim4);
	HAL_ADC_Start_DMA(&hadc1 , adc_buff , 2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1 , 0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2 , 0);
	 
	HAL_UART_Receive_DMA(&huart1 , UART1_rxBuffer , 1);
	//HAL_UART_Receive_IT(&huart1 , UART1_rxBuffer , 1);
	
	set_motor_enable(1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//HAL_UART_Receive_IT(&huart1 , UART1_rxBuffer , 1);
	
		//set_motor_voltage(p);
		
		// loop time
		dt = __HAL_TIM_GET_COUNTER(&htim4) - last_t1;
		__HAL_TIM_SetCounter(&htim4 , 0);
		
		// update variable
		encoder_rad_last = encoder_rad;
		encoder_rad = encoder_pulse*ENC_PULSE_2_RAD;
		encoder_speed = 1000000.0*(encoder_rad - encoder_rad_last)/((double)dt);
		encoder_speed_filtered = alpha_speed*encoder_speed_filtered + (1.0 - alpha_speed)*encoder_speed;
		

		// Control
		if (control_mode == CURRENT_CONTROL_MODE){
			if (sp_current >= 0)
				current_sign = 1.0;
			else 
				current_sign = -1.0;
			
			//motor_voltage_dir
			fb_curr = current_sign*(0.0*I_motor_acs712_filtered + 1.0*I_motor_MAX4080_filtered);
			//err_curr = abs_f(sp_current) - fb_curr;
			err_curr = sp_current - fb_curr;
			err_curr_dot = 1000000.0*(err_curr - last_err_curr)/((double)dt);
			last_err_curr = err_curr;
			sum_err_curr = saturation_f(sum_err_curr + err_curr*((double)dt)/1000000.0 , -1.0 , 1.0);
			
			pid_out = K_p_curr*err_curr + K_d_curr*err_curr_dot + K_i_curr*sum_err_curr;
			ff_curr = (sp_current>0.01)*(0.9542*sp_current + 1.694) + (sp_current < -0.01)*(0.8944*sp_current - 1.708) ;
			
			//TODO:  bug on change set point sign
			control_sigal = pid_out + ff_curr;		// volt 
			set_motor_voltage(MOTOR_SIGN*control_sigal);
			
		}
		else if (control_mode == POSITION_CONTROL_MODE){
			err_pos = sp_pos_rad - encoder_rad;
			err_pos_dot = -encoder_speed;
			sum_err_pos = saturation_f(sum_err_pos + err_pos*((double)dt)/1000000.0 , -1.0 , 1.0);
			
			pid_out = K_p_pos*err_pos + K_d_pos*err_pos_dot + K_i_pos*sum_err_pos;
			ff_pos = 0;
			
			control_sigal = pid_out + ff_pos;		// volt 
			set_motor_voltage(MOTOR_SIGN*control_sigal);
		}
		else if (control_mode == SPEED_CONTROL_MODE){
		
			err_speed = sp_speed - encoder_speed_filtered;
			err_speed_dot = 1000000.0*(err_speed - last_err_speed)/((double)dt);
			last_err_speed = err_speed;
			sum_err_speed = saturation_f(sum_err_speed + err_speed*((double)dt)/1000000.0 , -1.0 , 1.0);
			
			pid_out = K_p_speed*err_speed + K_d_speed*err_speed_dot + K_i_speed*sum_err_speed;
			ff_speed = (sp_speed > 0.2)*(0.3785*(sp_speed) + 1.36) + (sp_speed < -0.2)*(0.3667*(sp_speed) - 1.07); 
			
			control_sigal = pid_out + ff_speed;		// volt 
			set_motor_voltage(MOTOR_SIGN*control_sigal);
		}
		
		
		// loop delay
		HAL_Delay(0);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Motor_REn_Pin|Motor_LEn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Enc_Z_Pin */
  GPIO_InitStruct.Pin = Enc_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Enc_Z_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_REn_Pin Motor_LEn_Pin */
  GPIO_InitStruct.Pin = Motor_REn_Pin|Motor_LEn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Enc_A_Pin */
  GPIO_InitStruct.Pin = Enc_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Enc_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Enc_B_Pin */
  GPIO_InitStruct.Pin = Enc_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Enc_B_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
