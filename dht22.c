/*
 * dht22.c
 *
 *  Created on: Nov 27, 2019
 *      Author: mustafa
 */

#include "main.h" //buraya don
#include "stm32f0xx_hal.h"
#include "dht22.h"

uint8_t nem_byte1, nem_byte2, sicaklik_byte1, sicaklik_byte2,checksum, verilerin_toplami;
uint16_t  nem, sicaklik,sicaklik1,nem1,timer_sayac_degeri;


void delay_ticks(uint32_t ticks)
 { //SYSTICK TIMER 24 BIT
     SysTick->LOAD = ticks; //girilen sÃ¼re iÃ§in gerekli tick sayÄ±sÄ±
     SysTick->VAL = 0;
     SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
     SysTick->CTRL &= ~SYSTICK_CLKSOURCE_HCLK_DIV8;
     SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;
     //COUNTFLAG,counter 0 oldugunda 1 oluyo.
     //okundugunda otomatik oalrak temizleniyo
     while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
     SysTick->CTRL = 0;
 }

void delay_us(uint32_t us)
{
    delay_ticks(us * 8);
}

static inline void delay_ms(uint32_t ms)
 {
     delay_ticks(ms * 8000);
 }

GPIO_InitTypeDef GPIO_InitStruct = {0};
 void pini_giris_yap (void)
  {
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

  /*Configure GPIO pin : PA2 */
  void pini_cikis_yap (void)
  {
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }


void DHT22_baslat (void)
{
	pini_cikis_yap();
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5, 0);   // baslangica sinyali
	delay_us(1100);
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5, 1);   // sensorden gelecek cevabi bekle
	delay_us(30);
	pini_giris_yap();
}

void sensorden_yanit_al (void)
{
	 delay_us(120); //datasheete gore 80-160 arasi bir deger
	while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_5)));
}

uint8_t veri_oku(void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_5)));   // pin 1 olana keder bekle
		delay_us(40);   // 50 yapinca calismiyo
		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_5)) == 0)
		{
			i&= ~(1<<(7-j));   // alinan bit 0
		}
		else i|= (1<<(7-j));  // alinan bit 1

		while (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_5 && ((TIM3->CNT-timer_sayac_degeri)==1600) )); //problem olursa 200 us sonra devam et
	}
	return i;
}


void oku(void){
DHT22_baslat ();
	sensorden_yanit_al ();
	nem_byte1 = veri_oku();
	nem_byte2 = veri_oku();
	sicaklik_byte1 = veri_oku();
	sicaklik_byte2 = veri_oku();
	checksum = veri_oku();// check-sum should be the last 8 bit of "8 bit integral RH data+8 bit decimal RH
	//data+8 bit integral T data+8 bit decimal T data".


	verilerin_toplami=(nem_byte1+nem_byte2+sicaklik_byte1+sicaklik_byte2);
	//		 HAL_UART_Transmit(&huart1,(uint8_t*)"resul\r\n",sizeof("resul\r\n")-1,100);

	if ((checksum) == verilerin_toplami)
	{
	sicaklik = ((sicaklik_byte1<<8)|sicaklik_byte2);
	nem = ((nem_byte1<<8)|nem_byte2);

	}

}
