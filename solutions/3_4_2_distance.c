/*
Distance sensor 3.4.2 code.
*/

/* Includes */

#include "stm32l1xx.h"
#define HSI_VALUE    ((uint32_t)16000000)
#include "nucleo152start.h"
#include <stdio.h>
#include <string.h>

/* Private typedef */
/* Private define  */
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */
void delay_ms(unsigned long delay);
void delay_10us(void);
void USART_write(char data);
void USART2_Init(void);
void delay_Ms(int delay);
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	__disable_irq();			//global disable IRQs, M3_Generic_User_Guide p135.
	USART2_Init();

  /* Configure the system clock to 32 MHz and update SystemCoreClock */
  SetSysClock();
  SystemCoreClockUpdate();

  /* TODO - Add your application code here */

RCC->AHBENR|=1; 				//GPIOA ABH bus clock ON. p154
RCC->AHBENR|=4;					//GPIOC ABH bus clock ON. p154
RCC->APB2ENR |=1;				//Bit 0 SYSCFGEN: System configuration controller clock enable. p157

GPIOA->MODER&=~0x00000C00;		//clear (input reset state for PA5). p184
GPIOA->MODER|=0x00000400; 		//GPIOA pin 5 (PA5) to output. p184

GPIOC->MODER &= ~0x0C000000;	//clear (input state for PC13). p184
SYSCFG->EXTICR[3] &= ~0x00f0;	//clear port selection for EXTI13. p223
SYSCFG->EXTICR[3] |= 0x0020;	//select port C (PC13) for EXTI13. p223
EXTI->IMR |= 0x2000;			//unmask EXTI13 (MR13), p242
EXTI->FTSR |= 0x2000;			//select falling edge trigger, p243
//EXTI->RTSR |= 0x2000;			//select risign edge trigger, p243

//PA6 to external interrupt
GPIOA->MODER &=~0x3000;			//clear (input state for PA6). p184
SYSCFG->EXTICR[1] &=~0x0F00;	//select port A (PA6) for EXTI6. p223
EXTI->IMR |= (1<<6);			//unmask EXTI6 (MR6), p242
EXTI->RTSR |= (1<<6);			//select risign edge trigger PA6, p243
EXTI->FTSR |= (1<<6);			//select falling edge trigger, p243

NVIC_EnableIRQ(EXTI15_10_IRQn); //enable IRQ M3_Generic_User_Guide p130
NVIC_EnableIRQ(EXTI9_5_IRQn); 	//enable IRQ M3_Generic_User_Guide p130

__enable_irq();					//global enable IRQs, M3_Generic_User_Guide p135

GPIOA->MODER|=0x400; 	//GPIOA pin 5 to output. p184
GPIOA->MODER|=0x4000;	//GPIOA pin 7 to output.
GPIOA->ODR^=0x20;		//0010 0000 xor bit 5. p186
delay_ms(1000);
GPIOA->ODR^=0x20;		//0010 0000 xor bit 5. p186
delay_ms(1000);

  /* Infinite loop */
   while (1)
  {
		GPIOA->ODR|=0x20;		//0010 0000 xor bit 5. p186
		GPIOA->ODR|=0x80;		//0010 0000 xor bit 7. p186
		delay_10us();			//this have to be calibrated
		delay_10us();			//this have to be calibrated
		GPIOA->ODR&=~0x20;		//0010 0000 xor bit 5. p186
		GPIOA->ODR&=~0x80;		//0010 0000 xor bit 7. p186
		delay_ms(1000);			//this have to be calibrated
  }
  return 0;
}

void delay_ms(unsigned long delay)
{
	unsigned long i=0;
	RCC->APB1ENR|=(1<<1); 	//TIM3EN: Timer 5 clock enable. p160
	TIM3->PSC=32-1; 		//32 000 000 MHz / 32 = 1 000 000 Hz. p435
	TIM3->ARR=1000-1; 		//TIM3 counter. 1 000 000 Hz / 1000 = 1000 Hz ~ 1ms. p435
	TIM3->CNT=0;			//counter start value = 0
	TIM3->CR1=1; 			//TIM3 Counter enabled. p421

	  while(i<delay)
	  {
		  while(!((TIM3->SR)&1)){} //Update interrupt flag. p427
		  i++;
		  TIM3->SR &= ~1; 	//flag cleared. p427
		  TIM3->CNT=0;	  	//counter start value = 0
	  }
	TIM3->CR1=0; 		//TIM3 Counter disabled. p421
}

void delay_10us(void)
{
	//unsigned long i=0;//,delay=3;
	RCC->APB1ENR|=(1<<4); 	//TIM6EN: Timer 6 clock enable. p160
	TIM6->PSC=130; 			//value from ocilloscope meauremt for 10 us
	TIM6->ARR=1; 			//TIM6 counter. 1 000 000 Hz / 1000 = 1000 Hz ~ 1ms. p435
	TIM6->CNT=0;			//counter start value = 0
	TIM6->CR1=1; 			//TIM6 Counter enabled. p421

	 // while(i<delay)
	 // {
		  while(!((TIM6->SR)&1)){} //Update interrupt flag. p427
		  //i++;
		  TIM6->SR &= ~1; 	//flag cleared. p427
		  TIM6->CNT=0;	  	//counter start value = 0
	  //}
	TIM6->CR1=0; 		//TIM6 Counter disabled. p421
}

void USART2_Init(void)
{
	RCC->APB1ENR|=0x00020000; 	//set bit 17 (USART2 EN)
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[0]=0x00000700;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->MODER|=0x00000020; 	//MODER2=PA2(TX) to mode 10=alternate function mode. p184

	USART2->BRR = 0x00000116;	//115200 BAUD and crystal 32MHz. p710, 116
	USART2->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART2->DR=(data);			//p739
}

void EXTI15_10_IRQHandler(void)
{

	if((GPIOC->IDR & 0x2000)==0) //if input PC13 state logical 0 = 0V
	{
		char buf[]="falling edge interrupt";

		for(int i=0;i<strlen(buf);i++)
		{
			USART_write(buf[i]);
		}

		USART_write('\n');
		USART_write('\r');
	}
	else
	{
		char buf[]="risign edge interrupt";

		for(int i=0;i<strlen(buf);i++)
		{
			USART_write(buf[i]);
		}

		USART_write('\n');
		USART_write('\r');
	}

	EXTI->PR=0x2000;		//Pending, This bit is cleared by writing a ‘1’ to the bit. p245
}


void EXTI9_5_IRQHandler(void)
{
	if((GPIOA->IDR & (1<<6))==(1<<6)) //if input PA6 state logical 1 = 3.3V
	{
		RCC->APB1ENR|=(1<<2); 	//TIM4EN: Timer 4 clock enable. p160
		TIM4->PSC=32-1; 		//32 000 000 MHz / 32 = 1 000 000 Hz. p435
		//TIM3->ARR=32-1; 		//TIM3 counter. 1 000 000 Hz / 1000 = 1000 Hz ~ 1ms. p435
		TIM4->CNT=0;			//counter start value = 0
		TIM4->CR1=1; 			//TIM3 Counter enabled. p421
	}
	else
	{
		int timer4=TIM4->CNT;
		TIM4->CR1=0; 		//TIM4 Counter disabled. p421
		int distance=timer4/58;
		char buf[100]="";
		sprintf(buf,"%d",distance);

		for(int i=0;i<5;i++)
		{
			USART_write(buf[i]);
		}

		USART_write('\n');
		USART_write('\r');
	}

	EXTI->PR=(1<<6);		//Pending, This bit is cleared by writing a ‘1’ to the bit. p245
}

void delay_Ms(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<2460;i++); 	//measured with oscilloscope
}
