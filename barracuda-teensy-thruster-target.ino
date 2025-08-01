#include <Arduino.h>
#include <i2c_register_slave.h>

int PWM_PINS[] = { 0, 1, 4, 5 };
int frequency = 333;  // Hz
int bit_resolution = 8;
int default_pwm_width = 1500; // microseconds
double period = (double)1 / frequency;
int default_duty_cycle_val = (((double)default_pwm_width / 1000000) / period) * pow(2, bit_resolution);


// chosen min/max pwm width vals in microseconds
int MIN_PWM_WIDTH = 1300; // acutal min on t200 thruster datasheet is 1100
int MAX_PWM_WIDTH = 1700; // actual max on t200 thruster datasheet is 1900


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
  uint8_t bit_resolution = 8;  // Register 10
};

ThrusterRegisters thruster_registers;
Config config;
I2CRegisterSlave registerSlave = I2CRegisterSlave(Slave1, (uint8_t*)&thruster_registers, sizeof(thruster_registers), (uint8_t*)&config, sizeof(config));

// Callback
void on_write_isr(uint8_t reg_num, size_t num_bytes);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  for (unsigned int i = 0; i < sizeof(PWM_PINS) / sizeof(*PWM_PINS); i++) {
    analogWriteFrequency(PWM_PINS[i], frequency);
  }

  // assign 0x2d as i2c address if pin 2 not connected to gnd, 0x2e if pin 2 is connected to gnd
  /*
  pinMode(2, INPUT_PULLUP);
  if (digitalRead(2)) {
    registerSlave.listen(0x2d);
  } else {
    registerSlave.listen(0x2e);
  }
  */
  registerSlave.listen(0x2d);
  
  analogWriteResolution(bit_resolution);
  analogWrite(PWM_PINS[0], thruster_registers.thruster_reg_0);


  analogWrite(PWM_PINS[1], thruster_registers.thruster_reg_2);


  analogWrite(PWM_PINS[2], thruster_registers.thruster_reg_4);

  analogWrite(PWM_PINS[3], thruster_registers.thruster_reg_6);


  // Start listening
  registerSlave.after_write(on_write_isr);

  // Enable the serial port for debugging
  Serial.begin(9600);
  Serial.println("Registers: 0, 2, 4, 6");

  Serial.println("Default duty cycle val:");
  Serial.println(default_duty_cycle_val);

  Serial.println("thruster reg 0:");
  Serial.println(thruster_registers.thruster_reg_0);

  Serial.println("thruster reg 2:");
  Serial.println(thruster_registers.thruster_reg_2);

  Serial.println("thruster reg 4:");
  Serial.println(thruster_registers.thruster_reg_4);

  Serial.println("thruster reg 6:");
  Serial.println(thruster_registers.thruster_reg_6);


}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
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
