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

  // Disable TIM1, clear all settings
  TIM1->CR1 = 0;
  // No prescale
  TIM1->PSC = 0;
  // Count fo 0xFFF (12 bits), 17.5kHz
  TIM1->ARR = 0x0FFF;
  // Set initial output states
  TIM1->CCR1 = 0x400;
  TIM1->CCR2 = 0x800;
  TIM1->CCR3 = 0xA00;
  // Configure PWM output compare
  TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;  
  TIM1->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
  // Set deadtime
  TIM1->BDTR = TIM_BDTR_MOE | 108; // 108 cycles @ 72MHz = 1500ns
  // Enable timer
  TIM1->CR1 |= TIM_CR1_CEN;
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
  // Configure B12, floating input, pwm_inhibit
  GPIOB->CRH &= ~(GPIO_CRH_MODE12_Msk  | GPIO_CRH_CNF12_Msk);
  GPIOB->CRH |= GPIO_CRH_CNF12_0;

  // Configure C6, floating input, reverse_in
  GPIOC->CRL &= ~(GPIO_CRL_MODE6_Msk  | GPIO_CRL_CNF6_Msk);
  GPIOC->CRL |= GPIO_CRL_CNF6_0;
  // Configure C12, push-pull output, led_out
  GPIOB->CRH &= ~(GPIO_CRH_MODE12_Msk  | GPIO_CRH_CNF12_Msk);
  GPIOB->CRH |= GPIO_CRH_MODE12_0 | GPIO_CRH_MODE12_1;
  // Configure C13, push-pull output, dcsw_out
  GPIOB->CRH &= ~(GPIO_CRH_MODE13_Msk  | GPIO_CRH_CNF13_Msk);
  GPIOB->CRH |= GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1;
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

  // TODO: Throttle ADC configuration
  // TODO: PWM configuration for current limit outputs
  // TODO: Digital inputs for direction and error states
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
  // Cache encoder deltas to allow averaging
  encoder_deltas[encoder_value_pos++] = TIM3->CNT;
  // Calculate sum of encoder deltas over previous 16 periods
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
