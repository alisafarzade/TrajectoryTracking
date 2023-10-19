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

#define PRESCALER 	  99
#define TIM1_PERIOD 	9
#define PWM_PERIOD 	499
#define TIM4_PERIOD 1399

#define NUM_OF_HOLES    12000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

volatile int counterRight, counterLeft, rpmRight, rpmLeft, counterBackRight, counterBackLeft;
int rpmRightD, rpmLeftD, errorRight, errorLeft;
//char flagARight, flagBRight, flagALeft, flagBLeft;
int rPWM = 0;
int lPWM = 0;
int delta_rPWM, delta_lPWM;
int  contL = 0, contR = 0;
uint16_t readValueL, readValueR;
float sensitivity = 0.185; // 0.185 for 5A Model
float rawVoltageL, rawVoltageR, kpR = 0.5, kpL = 0.5;
float sumL = 0, sumR = 0, currentL, currentR;
char forwardR, forwardL, backwardR, backwardL, increasingSpeed=1;
int second=0;
char speedFlag=0, currentFlag=0;
uint32_t AD_RES=0;
int loopNumber=0;
float KTL = 1, KTR = 1;
int errorTorRight=0, errorTorLeft=0;
int torRightD, torLeftD, torRight, torLeft;

int speedAngle;
uint8_t Rx_data[6];
#define totalError 15
#define circlePoint 20
int loopNumberPoint =0;
int allError[totalError];
char locFlag = 0;
//uint8_t x, y, angle;
int x, y, angle;
int dangle =0;
char locState = 's';

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ADC_Select_CHA3 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_Select_CHC0 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


void Motor_PWM_Right(int PWM){

	if(PWM >= 0){
	
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM);
	
	}
	else{
		
		PWM = -PWM;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM);		
	
	}

}

void Motor_PWM_Left(int PWM){

	if(PWM >= 0){
	
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM);
	
	}
	else{
		
		PWM = -PWM;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM);		
	
	}

}
int gox,goy;
struct position{
	int x;
	int y;
	int angle;
};
float nearPoint(struct position point1,struct position point2)
{
	float dist = sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2));
	return dist;
}

struct position circle(int r,struct position robot,struct position center,int point)
{
	struct position goPoint;
	goPoint.x = sin(point) * r + center.x;
	goPoint.y = cos(point) * r + center.y;
	goPoint.angle = (atan2(goPoint.y - robot.y, goPoint.x - robot.x)*180.0/3.14);
	if(goPoint.angle < 0 )
		goPoint.angle = goPoint.angle + 360;
	return goPoint;
}
int turnPID(float kc,float kp,int dangle,int angleRobot)
{	
float error = dangle-angleRobot;
	if(error > 180)
		error = -360 + error;
	else if (error < -180)
		error = error + 360;
	if(error > 45 || error < -45)
		kc /= 2;
allError[loopNumber] = error;

float sum=0;
for(int i=0;i<totalError;i++)
{
	sum = allError[i]+sum;
}
int turn = (error * kc) + (kp *sum);
return turn;
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
//
//
//	  /////////////////// Current ADC ///////////////////////////////
//
//	  		// If rawVoltage is not 2.5Volt, multiply by a factor.In my case it is 1.035
//	  //		HAL_ADC_PollForConversion(&hadc1,100);
//	  //    readValue = HAL_ADC_GetValue(&hadc1);
//	  //    sum += (float) readValue * 3.3 * 2 / 4095;
//	  //		cont++;
//
//	  //		if(cont >= 5000){
//	  //			rawVoltage = (float)sum / 5000;
//	  //			cont = 0;
//	  //			sum = 0;
//	  //		}
//	  //    current =(rawVoltage - 2.5)/sensitivity;
//
//
//	readValue = HAL_ADC_GetValue(&hadc1);
//	
//	currentFlag = 1;
//		
//}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	locFlag = 1;

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
if(htim -> Instance == TIM1){
	speedFlag = 1;
}

if(htim -> Instance == TIM4){

		second++;
	
////////////// forward straight line ///////////////
//	if(increasingSpeed){
//		rpmRightD += 15;
//		rpmLeftD += 15;
//	}
//	if(!increasingSpeed){
//		rpmRightD -= 15;
//		rpmLeftD -= 15;	
//	}
//	if(rpmRightD >= 69 || rpmLeftD >= 69){
//		increasingSpeed = 0;
//  }
//	if(rpmRightD <= 1 || rpmLeftD <= 1){
//		increasingSpeed = 1;
//  }


////////////// backward straight line ///////////////	
//	if(increasingSpeed){
//		rpmRightD -= 15;
//		rpmLeftD -= 15;
//	}
//	if(!increasingSpeed){
//		rpmRightD += 15;
//		rpmLeftD += 15;	
//	}
//	if(rpmRightD <= -69 || rpmLeftD <= -69){
//		increasingSpeed = 0;
//  }
//	if(rpmRightD >= -1 || rpmLeftD >= -1){
//		increasingSpeed = 1;
//  }


///////////////// T shape profile ///////////////////
//	if(second == 1){
//	
//		rpmRightD = 30;
//		rpmLeftD = 30;
//	
//	}
//	if(second == 3){
//		
//		rpmRightD = 0;
//		rpmLeftD = 0;

//	
//	}
//	if(second == 4){
//		
//		rpmRightD = 11;
//		rpmLeftD = -11;
//	
//	}
//	if(second == 7){
//		
//		rpmRightD = 0;
//		rpmLeftD = 0;

//	
//	}
//	if(second == 8){
//		
//		rpmRightD = 30;
//		rpmLeftD = 30;

//	
//	}
//	if(second == 9){
//		
//		rpmRightD = -30;
//		rpmLeftD = -30;

//	
//	}
//	if(second == 12){
//		
//		rpmRightD = 0;
//		rpmLeftD = 0;

//	
//	}

//////////////////// 8 ///////////////
//	if(second == 1){
//		
//		rpmRightD = 30;
//		rpmLeftD = 10;
//}
//	if(second == 11){
//		
//		rpmRightD = 10;
//		rpmLeftD = 30;
//  }
//	if(second == 21){
//		
//		rpmRightD = 0;
//		rpmLeftD = 0;
//  }
	
	
}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	/////////// Right //////////
	if(GPIO_Pin == GPIO_PIN_6){
		 
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)){
			counterRight++;
			forwardR = 1;
		}
		else{
			counterBackRight++;
			forwardR = 0;
		}
	}
	
	
	/////////// Left //////////

	if(GPIO_Pin == GPIO_PIN_7){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)){
			counterLeft++;
			forwardL = 1;
		}
		else{
			counterBackLeft++;
			forwardL = 0;
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
//	int a = 0;
	lPWM = 0;
//	rPWM = 1.3*lPWM;
	rPWM = 0;
	
	rpmRightD = 30;
	rpmLeftD = 30;
	
	torRightD = 120;
	torLeftD = 120;
	
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim4);
	
//	HAL_ADC_Start_DMA(&hadc1, &AD_RES, 1);
	
	////////////// left /////////////////////////////
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	
	////////////// Right /////////////////////////////
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	
	
	HAL_UART_Receive_IT(&huart1, Rx_data, 6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		ADC_Select_CHA3();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		readValueL = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		
		currentL = (float) readValueL * 3.3 / 4095 ;
//		sumL += (float) readValueL * 3.3 / 4095 ;
//		contL++;
//		if(contL >= 5){
//			rawVoltageL = (float)sumL / 5;
//			contL = 0;
//			sumL = 0;
//			currentL = rawVoltageL;
//		}
		
		ADC_Select_CHC0();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		readValueR = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);		
		
		currentR = (float) readValueR * 3.3 / 4095 ;
//		sumR += (float) readValueR * 3.3 / 4095 ;
//		contR++;
//		if(contR >= 5){
//			rawVoltageR = (float)sumR / 5;
//			contR = 0;
//			sumR = 0;
//			currentR = rawVoltageR;
//		}
//		
		
if(speedFlag){

	//60000/(TIM1_PERIOD+1)
	if(forwardR){	
	rpmRight = (counterRight * 60000/(TIM1_PERIOD+1))/NUM_OF_HOLES;
	}
	if(forwardL){
	rpmLeft = (counterLeft *  60000/(TIM1_PERIOD+1))/NUM_OF_HOLES;	
	}
	
	if(!forwardR){	
	rpmRight = -(counterBackRight *  60000/(TIM1_PERIOD+1))/NUM_OF_HOLES;
	}
	if(!forwardL){
	rpmLeft = -(counterBackLeft *  60000/(TIM1_PERIOD+1))/NUM_OF_HOLES;	
	}		
	
	
	counterBackRight = 0;
	counterBackLeft = 0;
	counterRight = 0;
	counterLeft = 0;
	speedFlag = 0;

}

if(locFlag){

	x = Rx_data[0];
	x |= Rx_data[1]<<8;
	y = Rx_data[2];
	y |= Rx_data[3]<<8;
	angle = Rx_data[4];
	angle |= Rx_data[5]<<8;
	
//	HAL_UART_Transmit(&huart1, Rx_data, sizeof(Rx_data), 10);
	HAL_UART_Receive_IT(&huart1, Rx_data, 6);
	locFlag = 0;

//	if(locState == 's'){
//		if(y >= 500){
//			HAL_Delay(100);
//			rpmRightD = 10;
//			rpmLeftD = -10;
//			locState = 'a';
//		}		
//	}
//	if(locState == 'a'){
//		if(angle <= 2 | angle >= 358){
//			HAL_Delay(100);
//			rpmRightD = 30;
//			rpmLeftD = 30;
//			locState = 'b';
//		}		
//	}
//	if(locState == 'b'){
//		if(x >= 450){
//			HAL_Delay(100);
//			rpmRightD = 10;
//			rpmLeftD = -10;
//			locState = 'c';
//		}		
//	}	
//	if(locState == 'c'){
//		if(angle <= 270){
//			HAL_Delay(100);
//			rpmRightD = 30;
//			rpmLeftD = 30;
//			locState = 'd';
//		}		
//	}
//	if(locState == 'd'){
//		if(y <= 350){
//			HAL_Delay(100);
//			rpmRightD = 10;
//			rpmLeftD = -10;
//			locState = 'e';
//		}		
//	}
//	if(locState == 'e'){
//		if(angle <= 180){
//			HAL_Delay(100);
//			rpmRightD = 30;
//			rpmLeftD = 30;
//			locState = 'f';
//		}		
//	}
//	if(locState == 'f'){
//		if(x <= 300){
//			HAL_Delay(100);
//			rpmRightD = 10;
//			rpmLeftD = -10;
//			locState = 'g';
//		}		
//	}
//	if(locState == 'g'){
//		if(angle <= 90){
//			HAL_Delay(100);
//			rpmRightD = 30;
//			rpmLeftD = 30;
//			locState = 's';
//		}		
	}	
	/*
	if(y >= 520){
	
		rpmRightD = -30;
		rpmLeftD = -30;
	}
	else if(y <= 240){
	
		rpmRightD = 30;
		rpmLeftD = 30;
	}*/
//}
		
/////////////////// Speed Control ///////////////////////////////
//Motor_PWM_Right(250);
//Motor_PWM_Left(250);
//		int a = 300;




//dangle = (atan2((370.0-y),(470.0-x)))*180.0/3.14;
//	if(dangle < 0 )
//		dangle = dangle + 360;
//speedAngle = turnPID(0.6,0.0015,dangle,angle);
loopNumber = loopNumber+1;
if(loopNumber>=totalError)
	loopNumber=0;
struct position robot;
robot.x =x;
robot.y = y;
robot.angle=angle;
struct position center;
center.x = 470;
center.y = 370;


struct position goPoint = circle(100,robot,center,loopNumberPoint*(2*3.14/circlePoint));
if(nearPoint(goPoint,robot)<20)
{
	loopNumberPoint = loopNumberPoint+1;
}
if(loopNumberPoint>circlePoint)
{
	loopNumberPoint = 0;
}
speedAngle = turnPID(0.6,0.0015,goPoint.angle,robot.angle);

//loopNumber = loopNumber+1;
//if(loopNumber>=totalError)
//	loopNumber=0;
//Motor_PWM_Right(speedAngle);
//Motor_PWM_Left(-speedAngle);
int addSpeed = 30;
//if(nearPoint(x,y,470,370)>20)
//{
//	addSpeed = 30;
//}
//else
//{
//		speedAngle=0;

//}
gox = goPoint.x;
goy = goPoint.y;
			rpmRightD = -speedAngle + addSpeed;
			rpmLeftD = speedAngle + addSpeed;

		errorRight = rpmRightD - rpmRight;
		
		delta_rPWM = kpR*errorRight;

		rPWM = rPWM + delta_rPWM;
		if(rPWM > PWM_PERIOD)
			rPWM = PWM_PERIOD;
		else if(rPWM < -PWM_PERIOD)
			rPWM = -PWM_PERIOD;		
		HAL_Delay(5);
		Motor_PWM_Right(rPWM);
	//	Motor_PWM_Right(250);
		
		errorLeft = rpmLeftD - rpmLeft; 
		
		delta_lPWM = kpL*errorLeft;

		lPWM = lPWM + delta_lPWM;
		if(lPWM > PWM_PERIOD)
			lPWM = PWM_PERIOD;
		else if(lPWM < -PWM_PERIOD)
			lPWM = -PWM_PERIOD;
		HAL_Delay(5);
		Motor_PWM_Left(lPWM);
		//Motor_PWM_Left(250);

		
		if(rpmRightD == 0 && rPWM !=0)
			rPWM = 0;
		if(rpmLeftD == 0 && lPWM !=0)
			lPWM = 0;

//Motor_PWM_Right(300);
		
		
/////////////////// End of Speed Control ///////////////////////////////





/////////////////// Torque Control /////////////////////////////////////
/*
		if(rPWM < 0)
			currentR = -currentR;
		torRight = currentR * KTR;
		errorTorRight = torRightD - torRight;
		
		delta_rPWM = 0.1*errorTorRight;

		rPWM = rPWM + delta_rPWM;
		if(rPWM > PWM_PERIOD)
			rPWM = PWM_PERIOD;
		else if(rPWM < -PWM_PERIOD)
			rPWM = -PWM_PERIOD;
		
		HAL_Delay(3);
		Motor_PWM_Right(rPWM); 

		
		if(lPWM < 0)
			currentL = -currentL;
		torLeft = currentL * KTL;
		errorTorLeft = torLeftD - torLeft;
		
		delta_lPWM = 0.1*errorTorLeft;

		lPWM = lPWM + delta_lPWM;
		if(lPWM > PWM_PERIOD)
			lPWM = PWM_PERIOD;
		else if(lPWM < -PWM_PERIOD)
			lPWM = -PWM_PERIOD;
		
		HAL_Delay(3);
		Motor_PWM_Left(lPWM); 		
*/		
/////////////////// USART /////////////////////////////////////
		
		
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = PRESCALER;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = PWM_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = PRESCALER;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 23999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = TIM4_PERIOD;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
