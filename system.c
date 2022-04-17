// This file contains hardware initialization functions

#include "stm32f103xb.h"

// Data from the ADC {il1, il2, throttle}
volatile uint16_t adc_data[3];

void configure_system_clock() {
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
}

void configure_ocurlim_pwm(uint16_t limit) {
  // Calculate current sensor offset
  while(!adc_data[1]); // Wait for data from ADC
  // Give time to settle
  volatile uint32_t delay = 1000000; while (delay--);
  // Calclate offset
  uint16_t offset = (adc_data[0] + adc_data[1]) >> 1;
  // Enable GPIOB clocks
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
  // Enable alternate function IO clock
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  // Enable TIM4 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  // Configure GPIOB pins 7,8 as AF push-pull output
  GPIOB->CRL &= ~(GPIO_CRL_MODE7_Msk | GPIO_CRL_CNF7_Msk);
  GPIOB->CRH &= ~(GPIO_CRH_MODE8_Msk | GPIO_CRH_CNF8_Msk);
  GPIOB->CRL |= GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1;
  GPIOB->CRH |= GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1 | GPIO_CRH_CNF8_1;

  // Disable TIM1, clear all settings
  TIM4->CR1 = 0;
  // No prescale
  TIM4->PSC = 0;
  // Count fo 0xFFF (12 bits), 17.5kHz
  TIM4->ARR = 0x0FFF;
  // Set output states, units are believed to be 1.5 per amp
  TIM4->CCR2 = offset - limit;
  TIM4->CCR3 = offset + limit;
  // Configure PWM output compare
  TIM4->CCMR1 = TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
  TIM4->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
  TIM4->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E;
  // Enable timer
  TIM4->CR1 |= TIM_CR1_CEN;
}

void configure_three_phase_pwm() {
  // Enable GPIOA and GPIOB clocks
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
  // Enable alternate function IO clock
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  // Enable TIM1 clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  // Configure GPIOA pins 8,9,10 as AF push-pull output
  GPIOA->CRH &= ~(GPIO_CRH_MODE8_Msk  | GPIO_CRH_CNF8_Msk);
  GPIOA->CRH &= ~(GPIO_CRH_MODE9_Msk  | GPIO_CRH_CNF9_Msk);
  GPIOA->CRH &= ~(GPIO_CRH_MODE10_Msk | GPIO_CRH_CNF10_Msk);
  GPIOA->CRH |= GPIO_CRH_MODE8_0  | GPIO_CRH_MODE8_1  | GPIO_CRH_CNF8_1;
  GPIOA->CRH |= GPIO_CRH_MODE9_0  | GPIO_CRH_MODE9_1  | GPIO_CRH_CNF9_1;
  GPIOA->CRH |= GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_1;
  // Configure GPIOB pins 13,15,15 as AF push-pull output
  GPIOB->CRH &= ~(GPIO_CRH_MODE13_Msk | GPIO_CRH_CNF13_Msk);
  GPIOB->CRH &= ~(GPIO_CRH_MODE14_Msk | GPIO_CRH_CNF14_Msk);
  GPIOB->CRH &= ~(GPIO_CRH_MODE15_Msk | GPIO_CRH_CNF15_Msk);
  GPIOB->CRH |= GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1 | GPIO_CRH_CNF13_1;
  GPIOB->CRH |= GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1 | GPIO_CRH_CNF14_1;
  GPIOB->CRH |= GPIO_CRH_MODE15_0 | GPIO_CRH_MODE15_1 | GPIO_CRH_CNF15_1;
  // Configure B12, floating input, pwm_inhibit
  GPIOB->CRH &= ~(GPIO_CRH_MODE12_Msk  | GPIO_CRH_CNF12_Msk);
  GPIOB->CRH |= GPIO_CRH_CNF12_0;

  // Disable TIM1, clear all settings
  TIM1->CR1 = 0;
  // No prescale
  TIM1->PSC = 0;
  // Count fo 0xFFF (12 bits), 17.5kHz
  TIM1->ARR = 0x0FFF;
  // Set initial output states, these are only for testing
  TIM1->CCR1 = 2047;
  TIM1->CCR2 = 2047;
  TIM1->CCR3 = 2047;
  // Configure PWM output compare
  TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;  
  TIM1->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
  // Set deadtime
  TIM1->BDTR = 108; // 108 cycles @ 72MHz = 1500ns
  // Enable reload interrupt
  TIM1->DIER |= TIM_DIER_UIE;
  // Global interrupt config
  NVIC->ISER[0] |= (1 << TIM1_UP_TIM16_IRQn);
  // Enable timer
  TIM1->CR1 |= TIM_CR1_CEN;
  // Clear error conditions
  TIM1->SR = ~TIM_SR_BIF;
}

void configure_gpio() {
  // Enable GPIOA, GPIOB, GPIOC clocks
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

  // Configure A1, floating input, ocur_in
  GPIOA->CRL &= ~(GPIO_CRL_MODE1_Msk  | GPIO_CRL_CNF1_Msk);
  GPIOA->CRL |= GPIO_CRL_CNF1_0;
  // Configure A2, floating input, brake_in
  GPIOA->CRL &= ~(GPIO_CRL_MODE2_Msk  | GPIO_CRL_CNF2_Msk);
  GPIOA->CRL |= GPIO_CRL_CNF2_0;
  // Configure A4, floating input, forward_in
  GPIOA->CRL &= ~(GPIO_CRL_MODE4_Msk  | GPIO_CRL_CNF4_Msk);
  GPIOA->CRL |= GPIO_CRL_CNF4_0;

  // Configure B1, push-pull output, precharge_out
  GPIOB->CRL &= ~(GPIO_CRL_MODE1_Msk  | GPIO_CRL_CNF1_Msk);
  GPIOB->CRL |= GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1;
  // Configure B5, floating input, cruise_in
  GPIOB->CRL &= ~(GPIO_CRL_MODE5_Msk  | GPIO_CRL_CNF5_Msk);
  GPIOB->CRL |= GPIO_CRL_CNF5_0;
  // Configure B6, floating input, start_in
  GPIOB->CRL &= ~(GPIO_CRL_MODE6_Msk  | GPIO_CRL_CNF6_Msk);
  GPIOB->CRL |= GPIO_CRL_CNF6_0;

  // Configure C6, floating input, reverse_in
  GPIOC->CRL &= ~(GPIO_CRL_MODE6_Msk  | GPIO_CRL_CNF6_Msk);
  GPIOC->CRL |= GPIO_CRL_CNF6_0;
  // Configure C12, push-pull output, led_out
  GPIOC->CRH &= ~(GPIO_CRH_MODE12_Msk  | GPIO_CRH_CNF12_Msk);
  GPIOC->CRH |= GPIO_CRH_MODE12_0 | GPIO_CRH_MODE12_1;
  GPIOC->ODR |= 1<<12;
  // Configure C13, push-pull output, dcsw_out
  GPIOC->CRH &= ~(GPIO_CRH_MODE13_Msk  | GPIO_CRH_CNF13_Msk);
  GPIOC->CRH |= GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1;
}

void configure_encoder_input() {
  // Enable GPIOA clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  // Enable alternate function IO clock
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  // Enable TIM3 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  // Configure PORTA pins 6,7 for AF input with pulls
  GPIOA->CRL &= ~(GPIO_CRL_MODE6_Msk  | GPIO_CRL_CNF6_Msk);
  GPIOA->CRL &= ~(GPIO_CRL_MODE7_Msk  | GPIO_CRL_CNF7_Msk);
  GPIOA->CRL |= GPIO_CRL_CNF6_1;
  GPIOA->CRL |= GPIO_CRL_CNF7_1;
  // Configure GPIO6,GPIO7 with pull-up
  GPIOA->ODR = (1<<6) | (1<<7);

  // Disable timer
  TIM3->CR1 = 0;
  // Configure encoder mode
  TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
  // Configure two inputs
  TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  // Enable timer
  TIM3->CR1 |= TIM_CR1_CEN;
  // TODO: enable digital noise filtering
}

void configure_adc() {
  // Enable GPIOA, GPIOB, GPIOC clocks
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
  // Enable ADC1 clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  // Enable DMA1 clock
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  // Configure A5, analog input, il1
  GPIOA->CRL &= ~(GPIO_CRL_MODE5_Msk  | GPIO_CRL_CNF5_Msk);
  // Configure B0, analog input, il2
  GPIOB->CRL &= ~(GPIO_CRL_MODE0_Msk  | GPIO_CRL_CNF0_Msk);
  // Configure C1, analog input, throttle
  GPIOC->CRL &= ~(GPIO_CRL_MODE1_Msk  | GPIO_CRL_CNF1_Msk);
  // Set ADC clock prescaler to 6 (12MHz)
  RCC->CFGR |= (2<<14);
  
  // Set scan mode
  ADC1->CR1 = ADC_CR1_SCAN;
  // SWSTART to trigger conversions
  ADC1->CR2 |= ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;
  
  // Set the sampling Time for the channels to 41.5 cycles
  // Total sampling time is 12MHz / (41.5+12.5) / 3 = ~74kHz
  ADC1->SMPR2 = ADC_SMPR2_SMP0_2;
  ADC1->SMPR2 = ADC_SMPR2_SMP1_2;
  ADC1->SMPR2 = ADC_SMPR2_SMP2_2;
  // Set the sequence length to 3 channels
  ADC1->SQR1 |= (2<<20);
  // Enable DMA for ADC
  ADC1->CR2 |= (1<<8);
  // Select channels
  ADC1->SQR3 = (5<<0 | 8<<5 | 11<<10); // Channels 5(A5), 8(B0), 11(PC1)
  // Enable ADC and wait for settle
  ADC1->CR2 |= 1<<0;
  volatile uint32_t delay = 10000;
  while (delay--);

  //  Reset DMA config
  DMA1_Channel1->CCR = 0;
  // Enable the circular mode (CIRC)
  DMA1_Channel1->CCR |= DMA_CCR_CIRC;
  // Enable the Memory Increment (MINC)
  DMA1_Channel1->CCR |= DMA_CCR_MINC;
  // Set the Peripheral data size (PSIZE) to 16 bits
  DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;
  // 5. Set the Memory data size (MSIZE) to 16 bits
  DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;
  // Set the total size of the transfer (3 words)
  DMA1_Channel1->CNDTR = 3;
  // Source address is ADC data
  DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
  // Destination address in memory
  DMA1_Channel1->CMAR = (uint32_t)adc_data;
  // Enable the DMA Stream
  DMA1_Channel1->CCR |= DMA_CCR_EN;

  // Clear status register
  ADC1->SR = 0;
  // Enable external trigger
  ADC1->CR2 |= ADC_CR2_EXTTRIG;
  // Enable continuous conversion
  ADC1->CR2 |= ADC_CR2_CONT;
  // Start conversions
  ADC1->CR2 |= ADC_CR2_SWSTART;
}
