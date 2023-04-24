/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c-lcd.h"
#include "PS2/PS2.h"
#include "rc522.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*---------------------- Khai Bao RC522 ----------------------*/
uint8_t CardID[5];
uint8_t MyID[5] = {0x7A, 0x92, 0xB4, 0x01, 0x5D};
uint8_t ZoneA[5] = {0x7A, 0x92, 0xB4, 0x01, 0x5D};
uint8_t ZoneB[5] = {0x7A, 0x92, 0xB4, 0x01, 0x5D};
char szBuff[100];
uint8_t Status=0;
int mode=0;
/*----------------------- Khai Bao PS2 -----------------------*/
PS2Buttons PS2;

/*----------------------- Khai Bao PID -----------------------*/
int read = 0x00000000;
int position;
int actives = 0;

float Kp = 0.002; //set up the constants value
float Ki = 0.001;
float Kd = 10;
float Kr = 0;

int P, I, D, R;
int lastError = 0;
int errors[10] = {0,0,0,0,0,0,0,0,0,0};
int error_sum = 0;
int last_end = 0;	// 0 -> Left, 1 -> Right 
int last_idle = 0;
	
const uint8_t maxspeedr = 50;
const uint8_t maxspeedl = 50;
const uint8_t basespeedr = 30;
const uint8_t basespeedl = 30;
const int ARR = 10;

#define TRIG_PIN GPIO_PIN_11
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_10
#define ECHO_PORT GPIOA
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;  // cm
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t u8_Status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ------------------------------Delay us------------------------------ */
void delay(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}
/*-------------------------------    PID   -----------------------------*/

void motor_control(double pos_left, double pos_right) 
{
	if (pos_left < 0 )
	{
		__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_1, ARR*0);
		__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_2, ARR*(-pos_left));
	} 
	else if(pos_left > 0)
	{
		__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_1, ARR*pos_left);
		__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_2, ARR*0);
	}
	else if (pos_left == 0)
	{
		__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_1, ARR*0);
		__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_2, ARR*0);
	}
	if (pos_right < 0 )
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*0);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*(-pos_right));
	} 
	else if (pos_right > 0 )
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*pos_right);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*0);
	}
	else if (pos_right == 0 )
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*0);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*0);
	}
}

uint8_t CamBien(void){
		HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

    pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
    // wait for the echo pin to go high
    while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
    Value1 = __HAL_TIM_GET_COUNTER (&htim1);

    pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
    // wait for the echo pin to go low
    while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
    Value2 = __HAL_TIM_GET_COUNTER (&htim1);

    Distance = (Value2-Value1)* 0.034/2;
		return Distance;
}

uint16_t Read_Line(void){
	if (HAL_GPIO_ReadPin(Line1_GPIO_Port, Line1_Pin)==0){
			read |= 0x00000001;
	}
	else read &= 0x11111110;
	if (HAL_GPIO_ReadPin(Line2_GPIO_Port, Line2_Pin)==0){
			read |= 0x00000010;
		}
	else read &= 0x11111101;
	if (HAL_GPIO_ReadPin(Line3_GPIO_Port, Line3_Pin)==0){
			read |= 0x00000100;
		}
	else read &= 0x11111011;
	if (HAL_GPIO_ReadPin(Line4_GPIO_Port, Line4_Pin)==0){
			read |= 0x00001000;
		}
	else read &= 0x11110111;
	if (HAL_GPIO_ReadPin(Line5_GPIO_Port, Line5_Pin)==0){
			read |= 0x00010000;
		}
	else read &= 0x11101111;
	return read;
	HAL_Delay(20);
}
int value_line(){
	//Read_Line();
	int pos = 0;
  int active = 0;
	switch(read){
		case 0x00000001:
		{
			pos += 1000;
			active++;
			last_end = 1;
		}
		break;
		case 0x00000011:
		{
			pos += 2000;
			active++;
		}
		break;
		case 0x00000010:
		{
			pos += 3000;
			active++;
		}
		break;
		case 0x00000110:
		{
			pos += 4000;
			active++;
		}
		break;
		case 0x00000100:
		{
			pos += 5000;
			active++;
		}
		break;
		case 0x00001100:
		{
			pos += 6000;
			active++;
		}
		break;
		case 0x00001000:
		{
			pos += 7000;
			active++;
		}
		break;
		case 0x00011000:
		{
			pos += 8000;
			active++;
		}
		break;
		case 0x00010000:
		{
			pos += 9000;
			active++;
			last_end=0;
		}
		break;
	}
	
  actives = active;
	position = pos/active;
	
	if (actives == 0)
		last_idle++;
	else
		last_idle = 0;
	return pos/active;
}

void sharp_turn () {
	if (last_idle < 25)
	{
		if (last_end == 1)
			motor_control(-40, 80);
		else
			motor_control(80, -40);
	}
	else 
	{
		if (last_end == 1)
			motor_control(-40, 60);
		else
			motor_control(60, -40);
	}
}
void motor_balance(double x, double y){
	if (y < 100){
				if (y > 0){
					motor_control(x,y-10);
				}
				else if (y < 0){
					motor_control(x,y+10);
				}
				else if (y == 0){
					motor_control(x,0);
				}
				else if (x==0 && y==0){
					motor_control(0,0);
				}
	}
	else if (y == 100){
		motor_control(x,100);
	}
}
void forward_brake(int pos_right, int pos_left) 
{
	if (actives == 0)
		sharp_turn();
	else
	  motor_balance(pos_right, pos_left);
}

void past_errors (int error) 
{
  for (int i = 9; i > 0; i--) 
      errors[i] = errors[i-1];
  errors[0] = error;
}

int errors_sum (int index, int abs) 
{
  int sum = 0;
  for (int i = 0; i < index; i++)
  {
    if (abs == 1 & errors[i] < 0)
      sum += -errors[i]; 
    else
      sum += errors[i];
  }
  return sum;
}

void PID_control() {
	uint16_t position = value_line();	
  int error = 5000 - position;
	past_errors(error);

  P = error;
  I = errors_sum(5, 0);
  D = error - lastError;
  R = errors_sum(5, 1);
  lastError = error;
	
  int motorspeed = P*Kp + I*Ki + D*Kd;
  
  int motorspeedl = basespeedl + motorspeed - R*Kr;
  int motorspeedr = basespeedr - motorspeed - R*Kr;
  
  if (motorspeedl > maxspeedl)
    motorspeedl = maxspeedl;
  if (motorspeedr > maxspeedr)
    motorspeedr = maxspeedr;
	
	forward_brake(motorspeedr, motorspeedl);
}
/*----------------------------------------------------------------------------*/
void LCD(void){
	lcd_init();
	lcd_clear();
	lcd_put_cur(0,0);
	lcd_send_string("BAI TAP LON");
	
	//Chay chu
	char DuLieu[]={"                 FORKLIFT - XE NANG HANG TU HANH "};
	char HienThi[100];
	unsigned char i,j;
	for(j=1;j <= strlen(DuLieu)-16;j++)
	{
		for(i=1;i<=16;i++)
		{
			lcd_put_cur(1,i);
			printf(HienThi,"%c",DuLieu[i+j-1]);
			lcd_send_string(HienThi);
		}
		HAL_Delay(100);
	}
}

/*-----------------------------------STEPMOTOR--------------------------------*/
#define stepsperrev 4096
void stepper_set_rpm (int rpm)  // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	delay(60000000/stepsperrev/rpm);
}
void stepper_half_drive (int step)
{
	switch (step){
		case 0:
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   // IN4
			  break;

		case 1:
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   // IN4
			  break;

		case 2:
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   // IN4
			  break;

		case 3:
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   // IN4
			  break;

		case 4:
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   // IN4
			  break;

		case 5:
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // IN4
			  break;

		case 6:
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // IN4
			  break;

		case 7:
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // IN4
			  break;
		}
}
void SM_UP(int x){ // Stepmotor up
	for(int i=0; i<512;i++){
		for(int j=0; j<8;j++){
			stepper_half_drive(j);
			stepper_set_rpm(x);
		}
	}
}
void SM_DOWN(int x){ // Stepmotor down
	for(int i=0; i<512;i++){
		for(int j=7; j>=0;j--){
			stepper_half_drive(j);
			stepper_set_rpm(x);
		}
	}
}

/*----------------------------------RC522-------------------------------*/
void RFID(){
	if (TM_MFRC522_Check(CardID) == MI_OK) {
			sprintf(szBuff, "ID: 0x%02X%02X%02X%02X%02X", CardID[0], CardID[1], CardID[2], CardID[3], CardID[4]);
			lcd_clear();
			lcd_put_cur(0,1);
			lcd_send_string("STM32 - MFRC522");
			lcd_put_cur(1,0);
			lcd_send_string(szBuff);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
			HAL_Delay(2000);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	}
}
void RFID_A(){
	lcd_put_cur(0,0);
	lcd_send_string("HAY QUET THE!");
	if (TM_MFRC522_Check(CardID) == MI_OK){
	lcd_clear();
	if (TM_MFRC522_Compare(CardID,MyID) == MI_OK) {
			sprintf(szBuff, "ID: 0x%02X%02X%02X%02X%02X", CardID[0], CardID[1], CardID[2], CardID[3], CardID[4]);
			lcd_put_cur(0,0);
			lcd_send_string("Xin chao Tuan!");
			lcd_put_cur(1,0);
			lcd_send_string(szBuff);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
			HAL_Delay(2000);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
			lcd_clear();
	}
	else{
		lcd_put_cur(0,0);
		lcd_send_string("THE KHONG PHU HOP!");
		lcd_put_cur(1,0);
		lcd_send_string("VUI LONG THU LAI ...");
		HAL_Delay(2000);
		lcd_clear();
	}
}
}
/*------------------------------ZONE------------------------------------*/
void ZONE(int *z){
	Read_Line();
	int a=0;
	if (read == 0x00011111){
		a++;
		motor_balance(0,0);
		HAL_Delay(100);
		if (a == *z){
			motor_balance(80,80);
			HAL_Delay(1000);
			SM_DOWN(13);
			motor_balance(-80,-80);
			HAL_Delay(1000);
			SM_UP(13);
			*z=0;
		}
		else if (a != *z){
			motor_balance(60,60);
			HAL_Delay(500);
		}
	}
}
/*----------------------------------------------------------------------*/
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	TM_MFRC522_Init();
	PS2_Init(&htim1, &PS2);
	lcd_init();
	LCD();
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  /* USER CODE END 2 */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
      {
    /* USER CODE END WHILE */
			mode = 0;
			int z = 0;
			motor_balance(0,0);
			PS2_Update();
			if(PS2.START){
			/* Mode 1 */
				mode = 1;
				while(mode == 1){
				PS2_Update();
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
				if (PS2.UP){
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					motor_balance(80,80);
					HAL_Delay(50);
				}
				else if(PS2.L1){HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					motor_balance(-50,50);
					HAL_Delay(50);
				}
				else if(PS2.R1){HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					motor_balance(50,-50);
					HAL_Delay(50);
				}
				else if(PS2.L2){HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					motor_balance(0,0);
					HAL_Delay(50);
				}
				else if(PS2.DOWN){
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					motor_balance(-60,-80);
					HAL_Delay(50);
				}
				else if(PS2.LEFT){
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					motor_balance(40,80);
					HAL_Delay(50);
				}
				else if(PS2.RIGHT){
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					motor_balance(80,40);
					HAL_Delay(50);
				}
				else if(PS2.TRIANGLE){
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					SM_UP(12);
					HAL_Delay(10);
				}
				else if(PS2.CROSS){
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					SM_DOWN(12);
					HAL_Delay(10);
				}
				else if(PS2.SELECT){
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					mode--;
					HAL_Delay(20);
				}
				else{
					motor_balance(0,0);
					HAL_Delay(50);
				}
				HAL_Delay(100);
			}
			}
			
			/* Mode 2 */
			
			else if(PS2.SELECT){
				mode =2;
				while(mode ==2){
				PS2_Update();
				Read_Line();
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
				if (read == 0x00011111){
					CamBien();
					if (Distance < 10){
						SM_DOWN(13);
						motor_balance(30,30);
						if (TM_MFRC522_Check(CardID) == MI_OK){
							motor_balance(0,0);
							HAL_Delay(50);
							SM_UP(13);
							/*---------------------------------------------------*/
							if (TM_MFRC522_Compare(CardID,ZoneA) == MI_OK) {
								z=1;
								while(z==1){
								PID_control();
								HAL_Delay(5);
								ZONE(&z);
								}
							}
							else if (TM_MFRC522_Compare(CardID,ZoneB) == MI_OK) {
								z=2;
								while(z==2){
								PID_control();
								HAL_Delay(5);
								ZONE(&z);
								}
							}
							/*---------------------------------------------------*/
						}
					}
					else if(Distance > 10){
						motor_balance(0,0);
						HAL_Delay(50);
					}
				}
				else if(PS2.START){
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					mode=mode-2;
					HAL_Delay(20);
				}
				PID_control();
				HAL_Delay(10);
			}
			}
			HAL_Delay(100);
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SS2_Pin|SCK2_Pin|MOSI2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB11 SS2_Pin SCK2_Pin
                           MOSI2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_11|SS2_Pin|SCK2_Pin
                          |MOSI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 MISO2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|MISO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Line5_Pin Line4_Pin Line3_Pin Line2_Pin
                           Line1_Pin */
  GPIO_InitStruct.Pin = Line5_Pin|Line4_Pin|Line3_Pin|Line2_Pin
                          |Line1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
