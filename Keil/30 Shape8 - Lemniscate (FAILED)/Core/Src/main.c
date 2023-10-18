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
#define TIM1_PERIOD 	59
#define PWM_PERIOD 	499
#define TIM4_PERIOD 99

#define NUM_OF_HOLES    12100

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
float rawVoltageL, rawVoltageR;
float kpR  = 0.6  , kpL = 0.6;
float ki_R = 0.01 , ki_L = 0.01;
float kd_R = 5.5, kd_L = 5.5;
float sumL = 0, sumR = 0, currentL, currentR;
char forwardR, forwardL, backwardR, backwardL, increasingSpeed=1;
float time=0;
int timerCounter = 0;
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
int got_x, got_y, got_angle;
float x = 0.6, y = 0, angle=3.14/2;
int dangle =0;
char locState = 's';

#define pi 3.14159265359 //3.142 
#define M_PI pi
#define M_2PI 2*pi
char derivationFlag=0;
float Xi1, Xi2;
float lastalpha=0, alpha, z1, z2;
float alphadot=0;
float ls1 = 0.55,ls2 = 1.35,ls31 = 2.75,ls32 = 1.65,le1 = 0.35,le2 = 0.35,le31 = 1.5,le32 = 1.2;
float k1 = 3.0,k2 = 3.0,k3 = 2.5, k4 = 1.0, k5 = 1.0, k41 = 2, k42 = 2;
float k1_temp, k2_temp;
float V, w;
float width = 0.125, r = 0.06;
float _xcd, _ycd, _thetad, _wd, _x2d, _x3d, _x2, _x3;
float A, B;
int pid_last_time=0;
double integralError_r = 0, difrentialError_r = 0, lastErrorRight = 0;
double integralError_l = 0, difrentialError_l = 0, lastErrorLeftt = 0;
double theta_Integral_Error = 0, K_I_theta = 0.05;
int theta_pid_last_time = 0;
double dest_angle = 0;
double dest_distance = 0;
int while_last_time=0;
int while_run_time=0;
uint8_t arived_to_dest = 0;
int gox,goy;
struct position{
	int x;
	int y;
	int angle;
};

#define LED_GREEN_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0)
#define LED_RED_OFF 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0)
#define LED_GREEN_ON 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1)
#define LED_RED_ON 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1)
float speed = 0.1;
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
void ADC_Select_CHA3 (void){
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

void ADC_Select_CHC0 (void){
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
int turn;
/*float unwrap(float previous_angle, float new_angle) {
	turn = (previous_angle / (pi)) + 1;
	float d = new_angle - previous_angle;
	d = d > pi ? d - turn * pi : (d < -pi ? d + turn * pi : d);
	return previous_angle + d;
}*/
//Normalize to [-180,180):
inline double constrainAngle(double x){
    x = fmod(x + M_PI,M_2PI);
    if (x < 0) x += M_2PI;
    return x - M_PI;
}
// convert to [-360,360]
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
float sin4(float val){
	return pow(sin(val), 4);
}
float sin6(float val){
	return pow(sin(val), 6);
}
float cos4(float val){
	return pow(cos(val), 4);
}
float sin8(float val){
	return pow(sin(val), 8);
}
float cos8(float val){
	return pow(cos(val), 8);
}
float xcd(float time){
	return (0.6 * cos(time*speed)) / (sin2(time*speed) + 1);
}
float ycd(float time){
	return (0.9 * cos(time*speed) * sin(time*speed)) / (sin2(time*speed) + 1);
}
float thetad(float time){
	float t = time*speed;
	float theta = atan2(-(9*(sin4(t)+(cos2(t)+1)*sin2(t)-cos2(t)))/(10*pow(sin2(t)+1, 2)), -(3*sin(t)*(sin2(t)+2*cos2(t)+1))/(5*pow((sin2(t)+1),2)));
	return unwrap(_thetad, theta);
}
float wd(float time){
	float t = time*speed;
	// Shape8
	return (18*cos(t)*sin4(t)+36*cos(t)*sin2(t)+18*cos(t))/(4*sin6(t)+57*sin4(t)-18*sin2(t)+9);
	
	// Ellipse
	//return (6*speed * (sin2(time*speed) +   cos2(time*speed)))  /	(9*sin2(time*speed) + 4*cos2(time*speed));
}
float x2d(float time){
	//return (xcd(time) * cos(thetad(time))) + (ycd(time) * sin(thetad(time)));
	return (_xcd * cos(_thetad)) + (_ycd * sin(_thetad));
}
float x3d(float time){
	//return (xcd(time) * sin(thetad(time))) - (ycd(time) * cos(thetad(time)));
	return (_xcd * sin(_thetad)) - (_ycd * cos(_thetad));
}
float x2(float xc, float yc, float time){
	//return (xc * cos(thetad(time))) + (yc * sin(thetad(time)));
	return (xc * cos(angle)) + (yc * sin(angle));
}
float x3(float xc, float yc, float time){
	//return (xc * sin(thetad(time))) - (yc * cos(thetad(time)));
	return (xc * sin(angle)) - (yc * cos(angle));
}
float nearPoint(struct position point1,struct position point2){
	float dist = sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2));
	return dist;
}
struct position circle(int r,struct position robot,struct position center,int point){
	struct position goPoint;
	goPoint.x = sin(point) * r + center.x;
	goPoint.y = cos(point) * r + center.y;
	goPoint.angle = (atan2(goPoint.y - robot.y, goPoint.x - robot.x)*180.0/3.14);
	if(goPoint.angle < 0 )
		goPoint.angle = goPoint.angle + 360;
	return goPoint;
}
int turnPID(float kc,float kp,int dangle,int angleRobot){	
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	locFlag = 1;

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
if(htim -> Instance == TIM1){
	speedFlag = 1;
}

if(htim -> Instance == TIM4){
	timerCounter++;
	time += 0.1;
	derivationFlag = 1;
}

}
double absf(double input){
	if(input >= 0) return input;
	return -input;
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
	
	/// Configure GPIOB
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	LED_GREEN_OFF;
	LED_RED_OFF;
	
	
	
	uint8_t temp_data;
	HAL_UART_Receive(&huart1, &temp_data, 6, HAL_MAX_DELAY);
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
	k1_temp = k1;
	k2_temp = k2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//while_run_time = HAL_GetTick() - while_last_time;
		//while_last_time = HAL_GetTick();
		
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
				rpmRight = ((counterRight * 60000)/(TIM1_PERIOD+1))/NUM_OF_HOLES;
			}
			if(forwardL){
				rpmLeft = ((counterLeft *  60000)/(TIM1_PERIOD+1))/NUM_OF_HOLES;	
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
			got_x = Rx_data[0];
			got_x |= Rx_data[1]<<8;
			x = (got_x/100.0) - 1.55;
		//	x -= 543;
			got_y = Rx_data[2];
			got_y |= Rx_data[3]<<8;
			y = (got_y/100.0) - 1.15;
		//	y -= 328;
			got_angle = Rx_data[4];
			got_angle |= Rx_data[5]<<8;
			angle = unwrap(angle, ((float) got_angle) * pi / 180);
			//if(angle < -pi) angle+=2*pi;
			//if(angle > pi)  angle-=2*pi;
		//	HAL_UART_Transmit(&huart1, Rx_data, sizeof(Rx_data), 10);
			HAL_UART_Receive_IT(&huart1, Rx_data, 6);
			locFlag = 0;
		}
		x = _xcd;
		y = _ycd;
		angle = _thetad;
		
		
		
		/*



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

		//if(nearPoint(x,y,470,370)>20)
		//{
		//}
		//else
		//{
		//		speedAngle=0;

		//}
		gox = goPoint.x;
		goy = goPoint.y;
		*/


		//Xi1 = wd(time) + (k3 * (thetad(time) - angle));
		//z1 = x3d(time) - x3(x, y, time);
		//alpha = x2d(time) + ((pow(le1,2) - pow(z1,2)) * k1 * z1 * wd(time));
		//z2 = alpha - x2(x, y, time);
		//if(derivationFlag){  alphadot = (alpha - lastalpha)/0.5;  }	
		//lastalpha = alpha;
		//Xi2 = alphadot + (pow(le2,2) - pow(z2,2)) * k2 * z2 * pow(wd(time),2) + ((pow(le2,2) - pow(z2,2))/(pow(le1,2) - pow(z1,2))) * z1 * wd(time);
		//V = x3(x, y, time) * Xi1 + Xi2;
		//w = Xi1;

		_xcd = xcd(time);
		_ycd = ycd(time);
		_thetad = thetad(time);
		_wd = wd(time);
		_x2d = x2d(time);
		_x3d = x3d(time);
		_x2 = x2(x, y, time);
		_x3 = x3(x, y, time);

		if(HAL_GetTick() - theta_pid_last_time > 5000){
			theta_Integral_Error = (theta_Integral_Error + (_thetad - angle));
			theta_pid_last_time = HAL_GetTick();
		}
		
		Xi1 = _wd + (k3 * (_thetad - angle));// + K_I_theta * theta_Integral_Error;
		z1 = _x3d - _x3;
		A = pow(le1,2) - pow(z1,2);
		
		if(_wd < 0.03 && _wd > -0.03){
			k1 = 4.0 * k1_temp;
			k2 = 4.0 * k2_temp;
		}else{
			k1 = k1_temp;
			k2 = k2_temp;
		}
		alpha = _x2d + (A * k1 * z1 * _wd);
		z2 = alpha - _x2;
		B = pow(le2,2) - pow(z2,2);

		if(derivationFlag){
			alphadot = (alpha - lastalpha)/0.1; //((TIM4_PERIOD+1)/1000);
			derivationFlag = 0;
			lastalpha = alpha;
		}	
		Xi2 = alphadot + (B * k2 * z2 * pow(_wd,2)) + ((B / A) * k5 * z1 * _wd);
		V = _x3 * Xi1 + k4 * Xi2;
		w = Xi1;
		//x = x + 0.002 * V * cos(angle);
		//y = y + 0.002 * V * sin(angle);
		//angle = angle + 0.002 * w;
		
		if(_thetad > 3.7)
			_thetad = _thetad;
		rpmRightD = (V + (width*w))/r;
		rpmLeftD = (V - (width*w))/r;
		/*
		// Farhan - move in path		(OK)
		_xcd = 0;
		_ycd = 0;
		_thetad = pi/2;
		dest_angle = atan2(_ycd - y, _xcd - x) - angle;
		if(dest_angle < -pi) dest_angle += 2*pi;
		dest_distance = sqrt(pow(_xcd - x, 2) + pow(_ycd - y, 2));
		
		//rpmRightD = (V + (width*w))/r;
		//rpmLeftD = (V - (width*w))/r;
		if(arived_to_dest < 0.15 && dest_distance < 0.3){
			dest_angle = _thetad - angle;
			if(dest_angle < 0.2 && dest_angle > -0.2){
				rpmRightD	= 0;
				rpmLeftD	= 0;
			}
			else{
				if(dest_angle < -pi) dest_angle += 2*pi;
				rpmRightD	= -1 -dest_angle*2;
				rpmLeftD	= -1 +dest_angle*2;
			}
			time += 0.1;
		}
		if(dest_distance < 0.15) arived_to_dest = 1;
		else{
			rpmRightD	= 10*dest_distance - dest_angle * 5;
			rpmLeftD	= 10*dest_distance + dest_angle * 5;
		}
		
		*/
		
		
		

		rpmRightD *= 9.55;
		rpmLeftD *= 9.55;
		if(rpmRightD > 90) LED_GREEN_ON;
		else							 LED_GREEN_OFF;
		if(rpmLeftD > 90)  LED_RED_ON;
		else							 LED_RED_OFF;
		/////////////////// Speed Control ///////////////////////////////
		//Motor_PWM_Right(250);
		//Motor_PWM_Left(250);
		//		int a = 300;
		//int addSpeed = 0;

//		rpmRightD = -speedAngle + addSpeed;
//		rpmLeftD = speedAngle + addSpeed;

		
		
		
		/////////////////////////////////////////////////// PID Farhan
		if(HAL_GetTick() - pid_last_time > 5){
			errorRight = rpmRightD - rpmRight;
			errorLeft = rpmLeftD - rpmLeft; 
			//if(rpmRight != rpmRightD) {
				integralError_r   += (errorRight - lastErrorRight);//*0.02;
				difrentialError_r = (errorRight - lastErrorRight);
			//}else{
			//	integralError_r   = 0;
			//	difrentialError_r = 0;
			//}
			//if(rpmLeft != rpmLeftD) {
				integralError_l   += (errorLeft  - lastErrorLeftt);//*0.02;
				difrentialError_l = (errorLeft  - lastErrorLeftt);
			//}else{
			//	integralError_l   = 0;
			//	difrentialError_l = 0;
			//}
			lastErrorLeftt = errorLeft;
			lastErrorRight = errorRight;
			pid_last_time = HAL_GetTick();
			delta_rPWM = kpR*errorRight + ki_R*integralError_r + kd_R*difrentialError_r;
			delta_lPWM = kpL*errorLeft  + ki_L*integralError_l + kd_L*difrentialError_l;
			rPWM = rPWM + delta_rPWM;
			lPWM = lPWM + delta_lPWM;
		}
		/////////////////////////////////////////////////// END
		
		if(rPWM > PWM_PERIOD)
			rPWM = PWM_PERIOD;
		else if(rPWM < -PWM_PERIOD)
			rPWM = -PWM_PERIOD;		
		//HAL_Delay(5);
		Motor_PWM_Right(rPWM);
	//	Motor_PWM_Right(250);

		
		if(lPWM > PWM_PERIOD)
			lPWM = PWM_PERIOD;
		else if(lPWM < -PWM_PERIOD)
			lPWM = -PWM_PERIOD;
		//HAL_Delay(5);
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
