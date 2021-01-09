/*
basic DAC for PA4 (A2). DAC gives 0-3.3V and uses 12-bit mode.
PA6 external interrupt

Atollic: File/New/(C/C++ project)/C Manged build/Executable Embedded C project/
STM32L1 Boards NUCLEO-L152RE/next/next/next/finish
*/

/* Includes */

#include "stm32l1xx.h"
#define HSI_VALUE    ((uint32_t)16000000)
#include "nucleo152start.h"

/* Private typedef */
/* Private define  */
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

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
	/* Configure the system clock to 32 MHz and update SystemCoreClock */
  SetSysClock();
  SystemCoreClockUpdate();

  /* TODO - Add your application code here */

  RCC->AHBENR|=1; //GPIOA ABH bus clock ON. p154
  GPIOA->MODER|=0x400; //GPIOA pin 5 to output. p184

  //PA6 to external interrupt
  GPIOA->MODER &=~0x3000;		//clear (input state for PA6). p184
  SYSCFG->EXTICR[1] &=~0x0F00;	//select port A (PA6) for EXTI6. p223
  EXTI->IMR |= (1<<6);			//unmask EXTI6 (MR6), p242
  EXTI->RTSR |= (1<<6);			//select risign edge trigger PA6, p243
  //EXTI->FTSR |= (1<<6);			//select falling edge trigger, p243

  //PA7 to input
  GPIOA->MODER &=~0xC000;		//clear (input state for PA7). p184

		int data=0;
		GPIOA->MODER |= 0x00000300;		//PA4 analog
		RCC->APB1ENR |=1<<29; 			//enable DAC clock
		DAC->CR|=1;						//enable DAC
		DAC->CR|=2;						//disable Buffer

NVIC_EnableIRQ(EXTI9_5_IRQn); 	//enable IRQ M3_Generic_User_Guide p130
  /* Infinite loop */

__enable_irq();					//global enable IRQs, M3_Generic_User_Guide p135

  while (1)
  {
	  DAC->DHR12R1=data++;
	  delay_Ms(10);

	  if(data>=4095)
	  {
		  data=0;
	  }
  }
  return 0;
}

void delay_Ms(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<2460;i++); //measured with oscilloscope
}

void EXTI9_5_IRQHandler(void)
{
	if(GPIOA->IDR & 0x80) //if PA7 has high state
	{
	GPIOA->ODR|=0x20;		//0010 0000 xor bit 5. p186
	}

	EXTI->PR=(1<<6);		//Pending, This bit is cleared by writing a ‘1’ to the bit. p245
}
