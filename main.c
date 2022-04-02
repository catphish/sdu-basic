#include "stm32f103xb.h"

void SystemInit (void)
{
  // Enable HSE
  RCC->CR |= (uint32_t)0x00010000;
  // Wait for HSE to be stable
  while((RCC->CR & 0x00020000) == 0);
  // Enable flash prefetch
  FLASH->ACR = 0x00000012;
  // Configure PLL
  RCC->CFGR |= 0x001D0400; // PLL=72Mhz APB1=36Mhz AHB=72Mhz
  // Enable PLL
  RCC->CR |= 0x01000000;
  // Wait for PLL to be stable
  while((RCC->CR & 0x03000000) == 0);
  /* Set SYSCLK as PLL */
  RCC->CFGR |= 0x00000002;
  // Wait for SYSCLK to switch
  while((RCC->CFGR & 0x00000008) == 0);

  // Enable GPIOA, GPIOB, GPIOC
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
  // Enable AF IO
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  // Enable TIM1, TIM3
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  // Set PORTA modes.
  GPIOA->CRL = GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1;
  GPIOA->ODR = 0xFFFFFFFF;
  GPIOA->CRH = GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1 | GPIO_CRH_CNF8_1 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_1;
  // Set PORTB modes.
  GPIOB->CRH = GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1 | GPIO_CRH_CNF13_1 | GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1 | GPIO_CRH_CNF14_1 | GPIO_CRH_MODE15_0 | GPIO_CRH_MODE15_1 | GPIO_CRH_CNF15_1;

  // TIM1 is used to generate 3-phase PWM outputs

  // Disable timer, clear all settings
  TIM1->CR1 = 0;
  // No prescale
  TIM1->PSC = 0;
  // Count fo 0xFFF (12 bits), 17.5kHz
  TIM1->ARR = 0x0FFF;
  // OUT1,2,3
  TIM1->CCR1 = 0x0;
  TIM1->CCR2 = 0x0;
  TIM1->CCR3 = 0x0;
  // Configure output comapre to PWM
  TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;  
  TIM1->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
  // Enable PWM output
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
  // Set deadtime
  TIM1->BDTR = TIM_BDTR_MOE | 108; // 108 cycles @ 72MHz = 1500ns
  TIM1->CR1 |= TIM_CR1_CEN;

  // TIM3 is used to count encoder pulses
  // TODO: enable digital noise filtering

  // Disable timer
  TIM3->CR1 = 0;
  // Configure encoder mode
  TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
  // Configure two inputs
  TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  // Enable timer
  TIM3->CR1 |= TIM_CR1_CEN;
}

int main() {
  // Nothing to do yet
  while(1);
  // During the main program execution, most work will be done by
  // a 17.5Khz interrupt. This will advance the PWM sine wave output
  // by the rotation indicated by the encoder, plus slip indicated
  // by the throttle. It will additionally calculate the correct
  // amplitude for the output based on a V/Hz ratio.

  // A major challenge is that in order to calculate V/Hz, we need
  // to calculate the instantaneous rotational velocity.
  // Unfortunately this means we need to make a decision as to wether
  // we use TIM3 as an encoder input, which will not give us velovity
  // or a timer, which will give us veocity, but not direction.

  // We will use the same 17.5kHz interrupt to read ADC data (current,
  // thrrote) and digital inputs (direction, fault states, etc).
}
