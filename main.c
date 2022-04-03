#include "stm32f103xb.h"

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
  TIM1->CCR1 = 0x400;
  TIM1->CCR2 = 0x800;
  TIM1->CCR3 = 0xA00;
  // Configure PWM output compare
  TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;  
  TIM1->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;

  // Set deadtime
  TIM1->BDTR = 108; // 108 cycles @ 72MHz = 1500ns
  // Enable break on BKIN pin
  TIM1->BDTR |= TIM_BDTR_BKE;
  // Enable timer
  TIM1->CR1 |= TIM_CR1_CEN;
  // Enable output
  TIM1->BDTR |= TIM_BDTR_MOE;
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

uint16_t adc_data[3];

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
    // Set continuous conversion mode
  ADC1->CR2 = ADC_CR2_CONT;
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
  // Start conversions!
  ADC1->CR2 |= ADC_CR2_SWSTART;
}

void SystemInit (void)
{
  // Set system clock to 72MHz
  configure_system_clock();

  // Configure three phase PWM
  configure_three_phase_pwm();

  // Configure encoder input
  configure_encoder_input();

  // Configure digital IO pins
  configure_gpio();

  // Set up ADC, currently reads il21, il2, and throttle
  configure_adc();

  // TODO: PWM configuration for current limit outputs
}

// Storage of previous encoder deltas
uint16_t encoder_values[256];
uint8_t encoder_value_pos;

// One full rotation is 72 << 24 (1207959552)
// This absurdly large  allows us to be reasonably precise without the need for FPU
uint32_t stator_angle;

// This function runs at 17.5kHz intervals and handles all calculations
// required to generate the PWM sine wave outputs.
void pwm_interval() {
  // Cache encoder values to allow delta calcuation over any period
  encoder_values[encoder_value_pos++] = TIM3->CNT;
  // Calculate the delta over the previous 16 periods
  int16_t encoder_delta = encoder_values[encoder_value_pos - 1] - encoder_values[encoder_value_pos - 1 - 16];
  // Increment stator angle by the encoder delta. A full rotation is be 72 << 24, we scale down by 16.
  stator_angle += encoder_delta << 20;

  // TODO: Add a small fixed slip towards zero to ensure the neutral state is torque toward stationary
  // TODO: Direction input
  // TODO: Throttle input
  // TODO: Increment the angle by the throttle position
  // TODO: Calculate voltage based on sum of encoder and throttle increments
  // TODO: Current limit output
  // TODO: Error state input
}

int main() {
  // Nothing to do yet
  while(1);
}
