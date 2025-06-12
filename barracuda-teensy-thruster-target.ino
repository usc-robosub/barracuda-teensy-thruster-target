#include <Arduino.h>
#include <i2c_register_slave.h>

int PWM_PINS[4] = { 0, 1, 4, 5 };
int frequency = 400;  // Hz
int bit_resolution = 15;
int default_duty_cycle_val = (1U << (bit_resolution - 1));
double period = (double)1 / frequency;


// Thruster registers (caller writes to them)
struct ThrusterRegisters {
  uint16_t thruster_reg_0 = default_duty_cycle_val;  // Register 0
  uint16_t thruster_reg_2 = default_duty_cycle_val;  // Register 2
  uint16_t thruster_reg_4 = default_duty_cycle_val;  // Register 4
  uint16_t thruster_reg_6 = default_duty_cycle_val;  // Register 6
};
// TODO
// Config registers
struct Config {
  uint16_t frequency = 400;     // Register 8
  uint8_t bit_resolution = 15;  // Register 10
};

ThrusterRegisters thruster_registers;
Config config;
I2CRegisterSlave registerSlave = I2CRegisterSlave(Slave1, (uint8_t*)&thruster_registers, sizeof(thruster_registers), (uint8_t*)&config, sizeof(config));

// Callback
void on_write_isr(uint8_t reg_num, size_t num_bytes);

void setup() {
  for (int pin_idx = 0; pin_idx < sizeof(PWM_PINS) / sizeof(PWM_PINS[0]); pin_idx++) {
    analogWriteFrequency(pin_idx, frequency);
  }

  // assign 0x2d as i2c address if pin 2 not connected to gnd, 0x2e if pin 2 is connected to gnd
  pinMode(2, INPUT_PULLUP);
  if (digitalRead(2)) {
    registerSlave.listen(0x2d);
  } else {
    registerSlave.listen(0x2e);
  }
  analogWriteResolution(15);
  analogWrite(PWM_PINS[0], thruster_registers.thruster_reg_0);
  analogWrite(PWM_PINS[1], thruster_registers.thruster_reg_2);
  analogWrite(PWM_PINS[2], thruster_registers.thruster_reg_4);
  analogWrite(PWM_PINS[3], thruster_registers.thruster_reg_6);

  // Start listening
  registerSlave.after_write(on_write_isr);

  // Enable the serial port for debugging
  Serial.begin(9600);
  Serial.println("Registers: 0, 2, 4, 6");
}

void loop() {
}

void on_write_isr(uint8_t reg_num, size_t num_bytes) {
  switch (reg_num) {
    case 0:
      analogWrite(PWM_PINS[0], thruster_registers.thruster_reg_0);
      Serial.println("Thruster reg 0 written to: ");
      Serial.println(thruster_registers.thruster_reg_0);
      break;
    case 2:
      Serial.println("Thruster reg 2 written to: ");
      analogWrite(PWM_PINS[1], thruster_registers.thruster_reg_2);
      Serial.println(thruster_registers.thruster_reg_2);
      break;
    case 4:
      Serial.println("Thruster reg 4 written to: ");
      analogWrite(PWM_PINS[2], thruster_registers.thruster_reg_4);
      Serial.println(thruster_registers.thruster_reg_4);
      break;
    case 6:
      Serial.println("Thruster reg 6 written to: ");
      analogWrite(PWM_PINS[3], thruster_registers.thruster_reg_6);
      Serial.println(thruster_registers.thruster_reg_6);
      break;
    default:
      Serial.println("Bad register");
  }
}
