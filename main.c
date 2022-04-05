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
  // Configure three phase PWM
  configure_three_phase_pwm();
  // Configure encoder input
  configure_encoder_input();
  // Configure digital IO pins
  configure_gpio();
  // Configure ADC, currently reads il21, il2, and throttle
  configure_adc();
  // Configure overcurrent limit PWM output
  configure_ocurlim_pwm();
}

// Fetch the throttle input and normalize to 1.0
float get_throttle() {
  // Fetch ADC data
  uint16_t pot = adc_data[2];
  // Limit to allowed scale
  if(pot < POT_MIN) pot = POT_MIN;
  if(pot > POT_MAX) pot = POT_MAX;
  // Scale to 1.0
  pot -= POT_MIN;
  return (float) pot / (float) (POT_MAX - POT_MIN);
}

// This function runs at 17.5kHz intervals and handles all calculations
// required to generate the PWM sine wave outputs.
void TIM1_UP_TIM16_IRQHandler(void) {
  // Cache encoder values to allow delta calcuation over any period
  encoder_values[encoder_value_pos++] = TIM3->CNT;
  // Calculate the delta over the previous 255 periods (14.47ms moving average)
  // This value is proportional to RPM so can be converted when required
  int16_t encoder_delta = encoder_values[(uint8_t)(encoder_value_pos - 1)] - encoder_values[encoder_value_pos];
  // Calculate the stator increment based on rotor speed and throttle requested slip
  int32_t increment = 0;
  increment += encoder_delta * 233930;                // 2^32 / 72 / 255 = 3728270
  increment += get_throttle() * SLIP_MAX * 245426.7f; // 2^32 / 17500 = 245426.7
  // Apply to stator orientation
  stator_angle += increment;

  // Calculate voltage (sine amplitude)
  // Multiple by full PWM scale (3850), divide by full-voltage frequency (FWEAK) and units per Hz (245426.7)
  float voltage = (float)increment * 3850.f / 245426.7f / FWEAK;

  // Output the PWM
  TIM1->CCR1 = (int16_t)(voltage * table1[stator_angle >> 19]) + 2047;
  TIM1->CCR2 = (int16_t)(voltage * table2[stator_angle >> 19]) + 2047;
  TIM1->CCR3 = (int16_t)(voltage * table3[stator_angle >> 19]) + 2047;

  // Illuminate LED if fault has occurred
  if((TIM1->BDTR & TIM_BDTR_MOE) == 0) {
    GPIOC->ODR &= ~(1<<12);
  }
  // TODO: Add a small fixed slip towards zero to ensure the neutral state is torque toward stationary
  // TODO: Direction input

  // Clear interrupt
  TIM1->SR = ~TIM_SR_UIF;
}

int main() {
  // Nothing to do yet
  while(1);
}
