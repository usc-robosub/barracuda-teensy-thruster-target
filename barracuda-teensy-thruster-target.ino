#include <Arduino.h>
#include <i2c_register_slave.h>

int PWM_PINS[] = {0, 1, 4, 5};

// Config registers
struct Config {
  uint16_t freq = 333;                       // Register 0
  uint16_t bit_res = 8;                      // Register 2
  uint16_t t200_init = 1500;                 // Register 4
};
Config config;

// Thruster registers (registers 6, 8 , 10, 12)
uint16_t thruster_registers[sizeof(PWM_PINS) / sizeof(*PWM_PINS)];

I2CRegisterSlave registerSlave = I2CRegisterSlave(
    Slave1, (uint8_t *)&config, sizeof(config), (uint8_t *)&thruster_registers, sizeof(thruster_registers));

// Callback
void on_write_isr(uint8_t reg_num, size_t num_bytes);

void setup() {
  // Enable the serial port for debugging
  Serial.begin(9600);

  double period = (double)1 / config.freq;
  int default_duty_cycle_val =
      (((double)config.t200_init / 1000000) / period) *
      pow(2, config.bit_res);
  for (size_t i = 0;
       i < sizeof(thruster_registers) / sizeof(*thruster_registers); i++) {
    thruster_registers[i] = default_duty_cycle_val;
  }

  pinMode(LED_BUILTIN, OUTPUT);
  analogWriteResolution(config.bit_res);
  for (size_t i = 0; i < sizeof(PWM_PINS) / sizeof(*PWM_PINS); i++) {
    analogWriteFrequency(PWM_PINS[i], config.freq);
  }

  // assign 0x2d as i2c address if pin 2 not connected to gnd, 0x2e if pin 2 is
  // connected to gnd 
  pinMode(2, INPUT_PULLUP); if (digitalRead(2)) {
    registerSlave.listen(0x2d);
    Serial.println("2D");
  } else {
    registerSlave.listen(0x2e);
    Serial.println("2E");
  }

  analogWrite(PWM_PINS[0], thruster_registers[0]);

  analogWrite(PWM_PINS[1], thruster_registers[1]);

  analogWrite(PWM_PINS[2], thruster_registers[2]);

  analogWrite(PWM_PINS[3], thruster_registers[3]);

  // Start listening
  registerSlave.after_write(on_write_isr);


  Serial.println("Default duty cycle val:");
  Serial.println(default_duty_cycle_val);

  Serial.println("thruster reg 0:");
  Serial.println(thruster_registers[0]);

  Serial.println("thruster reg 1:");
  Serial.println(thruster_registers[1]);

  Serial.println("thruster reg 2:");
  Serial.println(thruster_registers[2]);

  Serial.println("thruster reg 3:");
  Serial.println(thruster_registers[3]);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

void on_write_isr(uint8_t reg_num, size_t num_bytes) {
  switch (reg_num) {
  case 6:
    analogWrite(PWM_PINS[0], thruster_registers[0]);
    Serial.println("Thruster reg 0 written to: ");
    Serial.println(thruster_registers[0]);
    break;
  case 8:
    Serial.println("Thruster reg 1 written to: ");
    analogWrite(PWM_PINS[1], thruster_registers[1]);
    Serial.println(thruster_registers[1]);
    break;
  case 10:
    Serial.println("Thruster reg 2 written to: ");
    analogWrite(PWM_PINS[2], thruster_registers[2]);
    Serial.println(thruster_registers[2]);
    break;
  case 12:
    Serial.println("Thruster reg 3 written to: ");
    analogWrite(PWM_PINS[3], thruster_registers[3]);
    Serial.println(thruster_registers[3]);
    break;
  default:
    Serial.println("Bad register");
  }
}
