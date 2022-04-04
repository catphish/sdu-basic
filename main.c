#include "stm32f103xb.h"
#include "system.h"
#include "table.h"

#define POT_MIN 1000
#define POT_MAX 4095
#define SLIP_MAX 5.f
#define FWEAK 220.f

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
  void configure_ocurlim_pwm();
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
void pwm_interval() {
  // Cache encoder values to allow delta calcuation over any period
  encoder_values[encoder_value_pos++] = TIM3->CNT;
  // Calculate the delta over the previous 16 periods
  int16_t encoder_delta = encoder_values[encoder_value_pos - 1] - encoder_values[encoder_value_pos - 1 - 16];
  // Calculate the stator increment based on rotor speed and throttle requested slip
  int32_t increment = 0;
  increment += encoder_delta * 3728270;               // 2^32 / 72 / 16 = 3728270
  increment += get_throttle() * SLIP_MAX * 245426.7f; // 2^32 / 17500 = 245426.7
  // Apply to stator orientation
  stator_angle += increment;
  // Calculate voltage
  float voltage = (float)increment / 245426.f / FWEAK;
  if(voltage >  1) voltage =  1;
  if(voltage < -1) voltage = -1;
  // Output the PWM
  TIM1->CCR1 = (voltage * table1[stator_angle >> 19]) * 2047 + 2047;
  TIM1->CCR2 = (voltage * table2[stator_angle >> 19]) * 2047 + 2047;
  TIM1->CCR3 = (voltage * table3[stator_angle >> 19]) * 2047 + 2047;

  // TODO: Add a small fixed slip towards zero to ensure the neutral state is torque toward stationary
  // TODO: Direction input
  // TODO: Current limit output

  // Start ADC conversions!
  ADC1->CR2 |= ADC_CR2_SWSTART;

}

int main() {
  // Nothing to do yet
  while(1);
}
