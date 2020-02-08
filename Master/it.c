/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "ctype.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
char gidecekdeger[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t data[] = "t";
uint8_t sicaklik_nem_buffer[5],toprak_buffer[4],rx_data[400];
uint8_t syc=0;
uint8_t sicaklik_veriler[2],nem_veriler[2],ts_veriler[2],checksum_slave,checksum_master;
uint8_t rx_index=0;
uint8_t baglanti_yapilamadi=0;
char str[100];
uint16_t nem,sicaklik,toprak_sicaklik,toprak_nem;
int nm;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart2;

void delay_ticks(uint32_t ticks)

 {

 SysTick->LOAD = ticks;

 SysTick->VAL = 0;

 SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;

 SysTick->CTRL &= ~SYSTICK_CLKSOURCE_HCLK_DIV8;
 SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;






 while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

 SysTick->CTRL = 0;

 }


  void delay_ms(uint32_t ms)
 {

 delay_ticks((ms * 16000)); // orjinal hali ((ms * 48000000)/2000)

 }

void ilet(int sicaklik,int nem,int toprak_sicaklik,int toprak_nem)
 {

 	HAL_UART_Transmit(&huart2,(uint8_t*)"AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n",
 										sizeof("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n")-1,100);

	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	delay_ms(2000);
	//	char yollanacakkomut[]=("GET /update?api_key=A0AISRD6SB8SKH4J&field1=%s\r\n",veriler);
	//	length = strlen(yollanacakkomut);
	HAL_UART_Transmit(&huart2,(uint8_t*)"AT+CIPSEND=83\r\n",
									sizeof("AT+CIPSEND=83\r\n")-1,100);

	delay_ms(2000);
	delay_ms(2000);

	sprintf(gidecekdeger,"GET /update?api_key=A0AISRD6SB8SKH4J&field1=%.1f&field2=%.1f&field3=%.1f&field4=%d\r\n",((float)sicaklik)/10,((float)nem)/10,((float)toprak_sicaklik)/16,toprak_nem);

	delay_ms(2000);
	delay_ms(2000);

	HAL_UART_Transmit(&huart2, (uint8_t *)gidecekdeger,strlen(gidecekdeger), 500);




 }

void sicaklik_nem_sensorune_baglan(void)
{	baglanti_yapilamadi=0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	delay_ms(200);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
	delay_ms(200);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	delay_ms(750);//3000
	HAL_UART_Transmit(&huart6,(uint8_t*)"AT+ROLE=1\r\n",sizeof("AT+ROLE=1\r\n")-1,200);
	delay_ms(750);
	HAL_UART_Transmit(&huart6,(uint8_t*)"AT+CMODE=0\r\n",sizeof("AT+CMODE=0\r\n")-1,200);
	delay_ms(750);
	HAL_UART_Transmit(&huart6,(uint8_t*)"AT+BIND=FCA8,9A,003D22\r\n",sizeof("AT+BIND=FCA8,9A,003D22\r\n")-1,200);
	delay_ms(500);
	HAL_UART_Transmit(&huart6,(uint8_t*)"AT+LINK=FCA8,9A,003D22\r\n",sizeof("AT+LINK=FCA8,9A,003D22\r\n")-1,200);
	delay_ms(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	delay_ms(200);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
	delay_ms(200);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	delay_ms(750);
}

void toprak_sicakik_nem_sensorune_baglan(void)
{		baglanti_yapilamadi=0;
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		delay_ms(100);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		delay_ms(100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		delay_ms(750);//3000
		HAL_UART_Transmit(&huart6,(uint8_t*)"AT+ROLE=1\r\n",sizeof("AT+ROLE=1\r\n")-1,100);
		delay_ms(750);
		HAL_UART_Transmit(&huart6,(uint8_t*)"AT+CMODE=0\r\n",sizeof("AT+CMODE=0\r\n")-1,100);
		delay_ms(750);
		HAL_UART_Transmit(&huart6,(uint8_t*)"AT+BIND=FCA8,9A,003B92\r\n",sizeof("AT+BIND=FCA8,9A,003B92\r\n")-1,100);
		delay_ms(500);
		HAL_UART_Transmit(&huart6,(uint8_t*)"AT+LINK=FCA8,9A,003B92\r\n",sizeof("AT+LINK=FCA8,9A,003B92\r\n")-1,100);
		delay_ms(500);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		delay_ms(100);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		delay_ms(100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		delay_ms(750);
}


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */
int baglanti_bayrak = 1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);

  if(baglanti_bayrak == 1)
  {
	sicaklik_nem_sensorune_baglan();//hc 05 den gelen veriler toprak buffer a doluyor
	baglanti_bayrak = 2;
  }

  if(baglanti_bayrak == 2)
   {
	  HAL_UART_Transmit(&huart6,(uint8_t*)"s",sizeof("s")-1,100);
	  baglanti_yapilamadi++;
	  if(baglanti_yapilamadi>=8)
	  {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		delay_ms(100);

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		baglanti_yapilamadi=0;

	  }

   }


  if(baglanti_bayrak == 3)
  {

		toprak_sicakik_nem_sensorune_baglan();//hc 05 den gelen veriler sicaklik_buffer a doluyor
		baglanti_bayrak = 4;

  }


  if(baglanti_bayrak == 4)
   {
	  HAL_UART_Transmit(&huart6,(uint8_t*)"t",sizeof("t")-1,100);
	  baglanti_yapilamadi++;
	  baglanti_yapilamadi++;
	  if(baglanti_yapilamadi>=8)
	  {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		delay_ms(100);

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		baglanti_yapilamadi=0;

	  }
   }

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  HAL_UART_Receive_IT(&huart2,rx_data,400);


  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  //if(rx_data[0]!=79)
  //{
  if(baglanti_bayrak == 2 )
{
HAL_UART_Receive_IT(&huart6,sicaklik_nem_buffer,5);

	if(sicaklik_nem_buffer[4]!=0 && sicaklik_nem_buffer[0]!=79)
	{
		sicaklik = ((sicaklik_nem_buffer[0]<<8)|sicaklik_nem_buffer[1]);
		nem = ((sicaklik_nem_buffer[2]<<8)|sicaklik_nem_buffer[3]);
		checksum_slave=sicaklik_nem_buffer[4];
		checksum_master=sicaklik_nem_buffer[0]+sicaklik_nem_buffer[1]+sicaklik_nem_buffer[2]+sicaklik_nem_buffer[3];
			if(checksum_master==checksum_slave)
			{
			sicaklik_nem_buffer[4]=0;
			baglanti_bayrak=3;
			}

	}



}

		if(baglanti_bayrak == 4 )
		{
		HAL_UART_Receive_IT(&huart6,toprak_buffer,4);

			if(toprak_buffer[3]!=0 )
			{
			toprak_sicaklik = ((toprak_buffer[0]<<8)|toprak_buffer[1]);
			toprak_nem = toprak_buffer[2];
			checksum_slave=toprak_buffer[3];
			checksum_master=toprak_buffer[0]+toprak_buffer[1]+toprak_buffer[2];


				if(checksum_master==checksum_slave)
				{

				ilet(sicaklik,nem,toprak_sicaklik,toprak_nem);
				toprak_buffer[3]=0;
				baglanti_bayrak = 1;

				}
			}



		}





  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
