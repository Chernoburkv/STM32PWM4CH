#include "stm32f10x.h"
#include "inttypes.h"


#define LED_OFF (GPIOC->BSRR = GPIO_BSRR_BS13)
#define LED_ON (GPIOC->BSRR = GPIO_BSRR_BR13)
//----------------------------------------------------------

#define TIM_EnableIT_UPDATE(TIMx) SET_BIT(TIMx->DIER, TIM_DIER_UIE)
#define TIM_EnableCounter(TIMx) SET_BIT(TIMx->CR1, TIM_CR1_CEN)
#define TIM_DisableCounter(TIMx) CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN)
#define TIM_CC_EnableChannel(TIMx, Channels) SET_BIT(TIMx->CCER, Channels);
//----------------------------------------------------------

__IO uint32_t SysTick_CNT = 0;


void SysTick_Init(void)
{
  //MODIFY_REG(SysTick->LOAD,SysTick_LOAD_RELOAD_Msk, SystemCoreClock  / 1000 - 1);
  //CLEAR_BIT(SysTick->VAL, SysTick_VAL_CURRENT_Msk);
	
	SysTick->LOAD = SysTick_LOAD_RELOAD_Msk & (SystemCoreClock  / 1000 - 1); //<= 0X00FFFFFF
	SysTick->VAL = 0;
  SysTick->CTRL|= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
}

void delay_ms(uint32_t ms)
{
  SysTick->VAL = SysTick->LOAD;
  SysTick_CNT = ms;
  while(SysTick_CNT) {}
}




/*
static void TIM3_Init(void)
{

  NVIC_EnableIRQ(TIM3_IRQn);
  TIM3->PSC = 3599;
  TIM3->ARR = 2000;
  TIM3->DIER|= TIM_DIER_UIE;
  TIM3->CR1|= TIM_CR1_CEN;
	
}
*/


//----------------------------------------------------------
static void TIM3_Init(void)
{
  uint32_t tmpcr1, tmpcr2, tmpccer, tmpccmr1, tmpccmr2;
  tmpcr1 = TIM3->CR1;
  tmpcr1 &= ~( TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD);
  TIM3->CR1 = tmpcr1 ;
  //Set the auto-reload value
  TIM3->ARR = 65535;
  //Set the prescaler value
  TIM3->PSC = 10;
  //Generate an update event to reload the Prescaler
  //and the repetition counter value (if applicable) immediately
  TIM3->EGR|= TIM_EGR_UG;
  //Disable auto-reload
  CLEAR_BIT(TIM3->CR1, TIM_CR1_ARPE);
  //Set clock source internal
  CLEAR_BIT(TIM3->SMCR, TIM_SMCR_SMS | TIM_SMCR_ECE);
  //CH1, CH2, CH3 AND CH4 Enable Preload
  TIM3->CCMR1|= TIM_CCMR1_OC2PE | TIM_CCMR1_OC1PE;
  TIM3->CCMR2|= TIM_CCMR2_OC4PE | TIM_CCMR2_OC3PE;
  //Disable the Channel 1, 2, 3 and 4: Reset the CC3E and CC4E Bits
  CLEAR_BIT(TIM3->CCER, TIM_CCER_CC4E | TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E);
  //Get the TIM3 CCER register value
  tmpccer = TIM3->CCER;
  //Get the TIM3 CR2 register value
  tmpcr2 = TIM3->CR2;
  //Get the TIM3 CCMR1 register value
  tmpccmr1 = TIM3->CCMR1;
  //Get the TIM3 CCMR2 register value
  tmpccmr2 = TIM3->CCMR2;
  //Reset Capture/Compare selection Bits
  CLEAR_BIT(tmpccmr1, TIM_CCMR1_CC2S | TIM_CCMR1_CC1S);
  CLEAR_BIT(tmpccmr2, TIM_CCMR2_CC4S | TIM_CCMR2_CC3S);
  //Select the Output Compare Mode
  MODIFY_REG(tmpccmr1, TIM_CCMR1_OC2M | TIM_CCMR1_OC1M, TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
  MODIFY_REG(tmpccmr2, TIM_CCMR2_OC4M | TIM_CCMR2_OC3M, TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);
  //Set the Output Compare Polarity
  CLEAR_BIT(tmpccer, TIM_CCER_CC4P | TIM_CCER_CC3P | TIM_CCER_CC2P | TIM_CCER_CC1P);
  //Set the Output State
  CLEAR_BIT(tmpccer, TIM_CCER_CC4E | TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E);
  //Write to TIM2 CR2
  TIM3->CR2 = tmpcr2;
  //Write to TIM2 CCMR1
  TIM3->CCMR1 = tmpccmr1;
  //Write to TIM2 CCMR2
  TIM3->CCMR2 = tmpccmr2;
  //Set the Capture Compare Registers value
  TIM3->CCR1 = 0;
  TIM3->CCR2 = 0;
  TIM3->CCR3 = 0;
  TIM3->CCR4 = 0;
  //Write to TIM2 CCER
  TIM3->CCER = tmpccer;
  //TIM2 OC Disable Fast
  CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC2FE | TIM_CCMR1_OC1FE );
  CLEAR_BIT(TIM3->CCMR2, TIM_CCMR2_OC4FE | TIM_CCMR2_OC3FE );
  //Disable Master Mode Selection
  CLEAR_BIT(TIM3->CR2, TIM_CR2_MMS);
  //Disable Master/Slave mode
  CLEAR_BIT(TIM3->SMCR, TIM_SMCR_MSM);

}
//----------------------------------------------------------







int main (void) {
	
SysTick_Init();	
uint32_t i, k ;
RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;
RCC->APB2ENR|=RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN|RCC_APB2ENR_IOPCEN|RCC_APB2ENR_AFIOEN;

LED_OFF;
TIM3_Init();
TIM_CC_EnableChannel(TIM3, TIM_CCER_CC4E | TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E);
TIM_EnableCounter(TIM3);
	

GPIOC->CRH	= (GPIOC->CRH & (~(GPIO_CRH_CNF13_1)))|GPIO_CRH_MODE13| GPIO_CRH_CNF13_0;
MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF7_0 | GPIO_CRL_CNF6_0 | GPIO_CRL_MODE7_0 | GPIO_CRL_MODE6_0,\
             GPIO_CRL_CNF7_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_MODE7_1 | GPIO_CRL_MODE6_1);
MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF1_0 | GPIO_CRL_CNF0_0 | GPIO_CRL_MODE1_0 | GPIO_CRL_MODE0_0,\
             GPIO_CRL_CNF1_1 | GPIO_CRL_CNF0_1 | GPIO_CRL_MODE1_1 | GPIO_CRL_MODE0_1);

delay_ms(1000);
LED_ON; 
delay_ms(2000);
	
	while (1) {	   
	
	for(k=10;k<70;k++) 
    {
   for(i=0;i<524288;i++) 
    {
      if(i<65536) TIM3->CCR1 = i;
      else if ((i>65535)&&(i<131072)) TIM3->CCR1 = 131071-i;
      else if((i>131071)&&(i<196608)) TIM3->CCR2 = i-131072;
      else if ((i>196607)&&(i<262144)) TIM3->CCR2 = 262143-i;
      else if((i>262143)&&(i<327680)) TIM3->CCR3 = i-262144;
      else if ((i>327679)&&(i<393216))  TIM3->CCR3 = 393215-i;
      else if((i>393215)&&(i<458752)) TIM3->CCR4 = i-393216;
      else TIM3->CCR4 = 524287-i;
     
       delay_ms(1);
            }

     }



   }

 }

 
 void SysTick_Handler(void)
{
  if(SysTick_CNT > 0)  SysTick_CNT--;
}
