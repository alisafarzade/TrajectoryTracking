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
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRESCALER 	  49
#define PWM_PERIOD 		999
#define TIM1_PERIOD 	59
#define NUM_OF_HOLES    12100

#define LED1_ON				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)		
#define LED1_OFF			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)		
#define LED2_ON				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)		
#define LED2_OFF			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define LED1_TGL			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12)
#define LED2_TGL			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13)
#define USART_BUFFER_SIZE 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// ------- Defines Variables
#define pi 3.14159265359 //3.142 
#define M_PI pi
#define M_2PI 2*pi
// ------- ADC Variables

int  contL = 0, contR = 0;
uint16_t readValueL, readValueR;
float rawVoltageL, rawVoltageR;
float sumL = 0, sumR = 0, currentL, currentR;
uint32_t AD_RES=0;

// ------- Motor PID Variables
float kpR  = 0.6  , kpL = 0.6;
float ki_R = 0.01 , ki_L = 0.01;
float kd_R = 5.5, kd_L = 5.5;
char forwardR, forwardL, backwardR, backwardL, increasingSpeed=1;
char speedFlag=0, currentFlag=0;
int pid_last_time=0;
double integralError_r = 0, difrentialError_r = 0, lastErrorRight = 0;
double integralError_l = 0, difrentialError_l = 0, lastErrorLeftt = 0;
float counterRight, counterLeft, rpmRight, rpmLeft, counterBackRight, counterBackLeft;
float rpmRightD, rpmLeftD, errorRight, errorLeft;
int rPWM = 0;
int lPWM = 0;
int delta_rPWM, delta_lPWM;

// ------- Motor Torque Variables
float KTL = 1, KTR = 1;
int errorTorRight=0, errorTorLeft=0;
int torRightD, torLeftD, torRight, torLeft;

// ------- USART Variables
uint8_t Rx_data[USART_BUFFER_SIZE];
char locFlag = 0;
int got_x, got_y, got_angle;

// ------- Kinematic Variables
float time=0;
char derivationFlag=0;
int cnt = 0;
/*
float x = 0.55, y = 0, angle=3.14/2;
float Xi1, Xi2;
float lastalpha=0, alpha, z1, z2;
float alphadot=0;
float ls1 = 0.55,ls2 = 1.35,ls31 = 2.75,ls32 = 1.65,le1 = 0.35,le2 = 0.35,le31 = 1.5,le32 = 1.2;
float k1 = 50.0,k2 = 50.0,k3 = 2.0, k4 = 1.0, k5 = 1.0, k41 = 2, k42 = 2;
float V, w;
float width = 0.125, r = 0.06;
float _xcd, _ycd, _thetad, _wd, _x2d, _x3d, _x2, _x3;
float A, B;
float speed = 0.1;
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Motor_PWM_Right(int PWM){

	if(PWM >= 0){
	forwardR = 1;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM);
	
	}
	else{
		forwardR = 0;
		PWM = -PWM;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM);		
	
	}

}

void Motor_PWM_Left(int PWM){

	if(PWM >= 0){
	forwardL = 1;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM);
	
	}
	else{
		forwardL = 0;
		PWM = -PWM;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM);		
	
	}

}
int turn;
// ------- Math Additional Functions
inline double constrainAngle(double x){
    x = fmod(x + M_PI,M_2PI);
    if (x < 0) x += M_2PI;
    return x - M_PI;
}
inline double angleConv(double angle){
    return fmod(constrainAngle(angle),M_2PI);
}
inline double angleDiff(double a,double b){
    double dif = fmod(b - a + M_PI,M_2PI);
    if (dif < 0)
        dif += M_2PI;
    return dif - M_PI;
}
inline double unwrap(double previousAngle,double newAngle){
    return previousAngle - angleDiff(newAngle,angleConv(previousAngle));
}
float sin2(float val){
	return pow(sin(val), 2);
}
float cos2(float val){
	return pow(cos(val), 2);
}
double absf(double input){
	if(input >= 0) return input;
	return -input;
}



// ------- Kinematic Functions
/*
float xcd(float time){
	return 0.45 * cos(time*speed);// + 2.2;
}
float ycd(float time){
	return 0.30 * sin(time*speed);// + 1.7;
}
float thetad(float time){
	float theta = atan2(0.30*cos(time*speed), -0.45*sin(time*speed));
	return unwrap(_thetad, theta);
}
float wd(float time){
	return (6*speed * (sin2(time*speed) +   cos2(time*speed))) /	(9*sin2(time*speed) + 4*cos2(time*speed));
}
float x2d(){
	return (_xcd * cos(_thetad)) + (_ycd * sin(_thetad));
}
float x3d(){
	return (_xcd * sin(_thetad)) - (_ycd * cos(_thetad));
}
float x2(float xc, float yc){
	return (xc * cos(angle)) + (yc * sin(angle));
}
float x3(float xc, float yc){
	return (xc * sin(angle)) - (yc * cos(angle));
}
*/

// ------- Interrupt CallBack Functions
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	locFlag = 1;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim -> Instance == TIM1){
		speedFlag = 1;
	}
	/*
	if(htim -> Instance == TIM4){
		time += 0.2;
		derivationFlag = 1;
	}
	*/
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	/////////// Right //////////
	if(GPIO_Pin == GPIO_PIN_7){
		if(forwardR == 1){ //HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)){
			counterRight++;
//			forwardR = 1;
		}
		else{
			counterBackRight++;
//			forwardR = 0;
		}
	}
	/////////// Left //////////
	if(GPIO_Pin == GPIO_PIN_0){
		if(forwardL == 1){ //!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)){
			counterLeft++;
//			forwardL = 1;
		}
		else{
			counterBackLeft++;
//			forwardL = 0;
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	
	
	//uint8_t temp_data;
	//HAL_UART_Receive(&huart1, &temp_data, USART_BUFFER_SIZE, HAL_MAX_DELAY);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
	
	HAL_TIM_Base_Start_IT(&htim1);
//	HAL_TIM_Base_Start_IT(&htim4);
	
//	HAL_ADC_Start_DMA(&hadc1, &AD_RES, 1);
	
	////////////// left /////////////////////////////
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	
	////////////// Right /////////////////////////////
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	
	
	HAL_UART_Receive_IT(&huart5, Rx_data, USART_BUFFER_SIZE);
	
	LED1_OFF;
	LED2_OFF;
	for(int i=0; i<65535; i+=10){
		TIM5->CCR2 = 65535 - i;			//G
		TIM5->CCR3 = 65535;			//R
		TIM5->CCR4 = 65535; //B
	}
	TIM5->CCR2 = 65535;			//G
	TIM5->CCR3 = 65535;			//R
	TIM5->CCR4 = 65535; //B
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
//		LED1_ON;
//		HAL_Delay(3000);
//		LED1_OFF;
//		HAL_Delay(1000);
//		LED2_ON;
//		HAL_Delay(3000);
//		LED2_OFF;
//		HAL_Delay(1000);
		if(speedFlag){
			if(forwardR)
				rpmRight = (counterRight * 60000/(TIM1_PERIOD+1))/NUM_OF_HOLES;
			if(forwardL)
				rpmLeft = (counterLeft *  60000/(TIM1_PERIOD+1))/NUM_OF_HOLES;	
			if(!forwardR)
				rpmRight = -(counterBackRight *  60000/(TIM1_PERIOD+1))/NUM_OF_HOLES;
			if(!forwardL)
				rpmLeft = -(counterBackLeft *  60000/(TIM1_PERIOD+1))/NUM_OF_HOLES;	
			counterBackRight = 0;
			counterBackLeft = 0;
			counterRight = 0;
			counterLeft = 0;
			speedFlag = 0;

		}

		if(locFlag){
			
			LED1_OFF;
			rpmLeftD = ((Rx_data[0] | (Rx_data[1]<<8)) - 9000)/100.0;
			rpmRightD= ((Rx_data[2] | (Rx_data[3]<<8)) - 9000)/100.0;
			if(rpmRight > 5)  rpmRightD -= 5;
			if(rpmRight < -5) rpmRightD += 5;
			/*
			got_x = Rx_data[0];
			got_x |= Rx_data[1]<<8;
			x = (got_x/100.0) - 1.55;
			got_y = Rx_data[2];
			got_y |= Rx_data[3]<<8;
			y = (got_y/100.0) - 0.90;
			got_angle = Rx_data[4];
			got_angle |= Rx_data[5]<<8;
			angle = unwrap(angle, ((float) got_angle) * pi / 180);
			*/
			HAL_UART_Receive_IT(&huart5, Rx_data, USART_BUFFER_SIZE);
			locFlag = 0;
		}
		
		
		


/*
		_xcd = xcd(time);
		_ycd = ycd(time);
		_thetad = thetad(time);
		_wd = wd(time);
		_x2d = x2d();
		_x3d = x3d();
		_x2 = x2(x, y);
		_x3 = x3(x, y);
		
		Xi1 = _wd + (k3 * (_thetad - angle));
		z1 = _x3d - _x3;
		A = pow(le1,2) - pow(z1,2);
		alpha = _x2d + (A * k1 * z1 * _wd);
		z2 = alpha - _x2;
		B = pow(le2,2) - pow(z2,2);

		if(derivationFlag){
			alphadot = (alpha - lastalpha)/0.2; //((TIM4_PERIOD+1)/1000);
			derivationFlag = 0;
			lastalpha = alpha;
		}	
		Xi2 = alphadot + (B * k2 * z2 * pow(_wd,2)) + ((B / A) * k5 * z1 * _wd);
		V = _x3 * Xi1 + k4 * Xi2;
		w = Xi1;
		
		rpmRightD = (V + (width*w))/r;
		rpmLeftD = (V - (width*w))/r;
		*/
		

//		
		if(rpmRightD > 90) LED2_ON;
		else							 LED2_OFF;
		if(rpmLeftD > 90)  LED1_ON;
		else							 LED1_OFF;
		
		
//		rpmRightD = 55;
//		rpmLeftD = 60;
//		if(cnt < 100000){
//			rpmRightD = 15;
//			rpmLeftD = 15;
//		}
//		else if(cnt < 200000){
//			rpmRightD = 80;
//			rpmLeftD = 80;
//		}
//		else cnt = 0;
//		cnt++;
		
		
		/////////////////////////////////////////////////// PID Farhan
		if(HAL_GetTick() - pid_last_time > 5){
			errorRight = rpmRightD - rpmRight;
			errorLeft = rpmLeftD - rpmLeft; 
			integralError_r   += (errorRight - lastErrorRight);
			difrentialError_r = (errorRight - lastErrorRight);
			integralError_l   += (errorLeft  - lastErrorLeftt);
			difrentialError_l = (errorLeft  - lastErrorLeftt);
			lastErrorLeftt = errorLeft;
			lastErrorRight = errorRight;
			pid_last_time = HAL_GetTick();
			delta_rPWM = kpR*errorRight + ki_R*integralError_r + kd_R*difrentialError_r;
			delta_lPWM = kpL*errorLeft  + ki_L*integralError_l + kd_L*difrentialError_l;
			rPWM = rPWM + delta_rPWM;
			lPWM = lPWM + delta_lPWM;
		}
		
		/////////////////////////////////////////////////// END
		
		if(rPWM > PWM_PERIOD) 				 rPWM = PWM_PERIOD;
		else if(rPWM < -PWM_PERIOD)		 rPWM = -PWM_PERIOD;		
		if(rpmRightD == 0 && rPWM !=0) rPWM = 0;
		Motor_PWM_Right(rPWM);
		
		if(lPWM > PWM_PERIOD)					lPWM = PWM_PERIOD;
		else if(lPWM < -PWM_PERIOD)		lPWM = -PWM_PERIOD;
		if(rpmLeftD == 0 && lPWM !=0)	lPWM = 0;
		Motor_PWM_Left(lPWM);

		

//Motor_PWM_Right(700);
//Motor_PWM_Left(-700);

//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		
		
/////////////////// End of Speed Control ///////////////////////////////
		
		
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 23999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TIM1_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = PRESCALER;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = PWM_PERIOD;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
  htim5.Init.Period = 65535;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
