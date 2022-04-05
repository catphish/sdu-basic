#include "stm32f103xb.h"
#include "system.h"
#include "table.h"

#define POT_MIN 100
#define POT_MAX 4000
#define SLIP_MAX 5.f
#define FWEAK 180.f;

// Storage of previous encoder deltas
uint16_t encoder_values[256];
uint8_t encoder_value_pos;

// Data from the ADC {il1, il2, throttle}
extern uint16_t adc_data[3];

// One full rotation is 2^32
// This large number of steps allows us to mix resolutions with good precision
uint32_t stator_angle;

void SystemInit (void)
{
  // Set system clock to 72MHz
  configure_system_clock();
  // Configure encoder input
  configure_encoder_input();
  // Configure digital IO pins
  configure_gpio();
  // Configure ADC, currently reads il21, il2, and throttle
  configure_adc();
  // Calculate current sensor offset
  while(!adc_data[1]); // Wait for data from ADC
  uint16_t offset = (adc_data[0] + adc_data[1]) >> 1;
  // Configure overcurrent limit PWM output
  configure_ocurlim_pwm(offset, 150); // 100A
  // Configure three phase PWM
  configure_three_phase_pwm();
}

// Fetch the throttle input and normalize to 1.0
float get_throttle() {
  // Fetch ADC data
  uint16_t pot = adc_data[2];
  // Limit to allowed scale
  if(pot < POT_MIN) pot = POT_MIN;
  if(pot > POT_MAX) pot = POT_MAX;
  pot -= POT_MIN;
  // Scale to 1.0 in FWD, return 0.0 otherwise
  // TODO: Reverse
  if(GPIOA->IDR & (1<<4))
    return (float) pot / (float) (POT_MAX - POT_MIN);
  else
    return 0.f;
}

// This function runs at 17.5kHz intervals and handles all calculations
// required to generate the PWM sine wave outputs.
void TIM1_UP_TIM16_IRQHandler(void) {
  uint16_t encoder_value = TIM3->CNT;
  // Calculate the delta over the previous 256 periods (14.6ms moving average)
  // Subtract oldest value from current value
  // This value is proportional to RPM: 1.053257143 = 1Hz
  int16_t encoder_delta = encoder_value - encoder_values[encoder_value_pos];
  // Generate a signal 1Hz slower than the input to generate some natural braking
  int16_t encoder_delta_braked = 0;
  if(encoder_delta > 0) encoder_delta_braked = encoder_delta - 1;
  if(encoder_delta < 0) encoder_delta_braked = encoder_delta + 1;

  // Store value for future calculations
  encoder_values[encoder_value_pos++] = TIM3->CNT;
  // Calculate the stator increment based on rotor speed and throttle requested slip
  int32_t increment = 0;
  increment += encoder_delta_braked * 233016;         // 2^32 / 72 / 256 = 233016
  increment += get_throttle() * SLIP_MAX * 245426.7f; // 2^32 / 17500 = 245426.7
  // Apply to stator orientation
  stator_angle += increment;

  // Generate a fault on cruise control input (B5) for testing
  if(GPIOB->IDR & (1<<5)) TIM1->EGR = TIM_EGR_BG;
  // Generate a fault on overspeed for now, approx 560Hz, 17,000RPM
  if(encoder_delta > 600) TIM1->EGR = TIM_EGR_BG;

  // Calculate voltage (sine amplitude)
  // Multiple by full PWM scale (+/-1935), divide by full-voltage frequency (FWEAK) and units per Hz (245426.7)
  float voltage = (float)increment * 1935.f / 245426.7f / FWEAK;
  if(voltage > 1.f)  voltage = 1.f;
  if(voltage < -1.f) voltage = -1.f;

  // Output the PWM
  TIM1->CCR1 = (int16_t)(voltage * table1[stator_angle >> 19]) + 2047;
  TIM1->CCR2 = (int16_t)(voltage * table2[stator_angle >> 19]) + 2047;
  TIM1->CCR3 = (int16_t)(voltage * table3[stator_angle >> 19]) + 2047;

  // Illuminate LED if fault has occurred
  if(TIM1->SR & TIM_SR_BIF) {
    GPIOC->ODR &= ~(1<<12);
  }
  // Clear interrupt
  TIM1->SR = ~TIM_SR_UIF;
}

int main() {
  // Nothing to do yet
  while(1);
}
