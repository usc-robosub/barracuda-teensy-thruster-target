#include <Arduino.h>
#include <i2c_register_slave.h>
#include <cmath>

#include "barracuda-teensy.h"

// Thruster registers (registers 0, 4, 8, 12)
float thruster_registers[NTHRUSTERS];

I2CRegisterSlave registerSlave = I2CRegisterSlave(
    Slave1, (uint8_t *)&thruster_registers, sizeof(thruster_registers), nullptr, 0);

// Callback
void on_write_isr(uint8_t reg_num, size_t num_bytes);

void setup() {
  // Enable the serial port for debugging
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  for (float &t : thruster_registers) {
    t = 0;
  }

  analogWriteResolution(PWM_BIT_RES);
  for (size_t i = 0; i < NTHRUSTERS; i++) {
    analogWriteFrequency(PWM_PINS[i], PWM_FREQ);
    analogWrite(PWM_PINS[i], f_to_dc(thruster_registers[i]));
  }

  // assign 0x2e as i2c address if pin 2 not connected to gnd, 0x2d if pin 2 is
  // connected to gnd 
  pinMode(2, INPUT_PULLUP); 
  delay(5); // for pin 2 to stabilize 
  if (digitalRead(2)) {
    registerSlave.listen(0x2e);
    Serial.println("2E");
  } else {
    registerSlave.listen(0x2d);
    Serial.println("2D");
  }

  // Start listening
  registerSlave.after_write(on_write_isr);

  Serial.printf("Init duty cycle : %d\n", INIT_DC);

  for (int i = 0; i < NTHRUSTERS; i++) {
    Serial.printf("thruster reg %d: %f, dc: %d\n", i, thruster_registers[i], f_to_dc(thruster_registers[i]));
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

uint16_t f_to_dc(float force) {
  if (force == 0) return INIT_DC;
  if (force <= f_vals[0]) return dc_vals[0];
  if (force >= f_vals[LUT_LEN - 1]) return dc_vals[LUT_LEN - 1];

  // binary search
  int lo = 0;
  int hi = LUT_LEN - 1;
  int mid;
  while (hi - lo > 1) {
    mid = (lo + hi) / 2;
    if (force == f_vals[mid]) return dc_vals[mid];
    if (force < f_vals[mid]) hi = mid;
    if (force > f_vals[mid]) lo = mid;
  }
  return (fabsf(force - f_vals[lo]) < fabsf(force - f_vals[hi])) ? dc_vals[lo] : dc_vals[hi];
}

void handle_thruster(int thruster_idx) {
  analogWrite(PWM_PINS[thruster_idx], f_to_dc(thruster_registers[thruster_idx]));
  Serial.printf("Thruster reg % written to: \n", thruster_idx);
  Serial.println(thruster_registers[thruster_idx]);
}

void on_write_isr(uint8_t reg_num, size_t num_bytes) {
  switch (reg_num) {
  case 0:
    handle_thruster(0);
    break;
  case 4:
    handle_thruster(1);
    break;
  case 8:
    handle_thruster(2);
    break;
  case 12:
    handle_thruster(3);
    break;
  default:
    Serial.println("Non-thruster register");
  }
}
