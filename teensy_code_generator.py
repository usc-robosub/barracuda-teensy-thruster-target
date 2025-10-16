import csv
import os
from scipy.interpolate import CubicSpline

# cubic spline interpolation of t200 pulse width (us) vs force (kgF) plot
# set TEENSY_PWM_FREQ, TEENSY_PWM_BIT_RES, TEENSY_T200_INIT_PWM_US
# use those values to create array of duty cycle vals, corresponding forces 
# from function given by interpolation - force = f(width_us(duty_cycle_val)) for each (1100us <= valid <= 1900) duty cycle val

# in microseconds
T200_INIT_PW = 1500
T200_MIN_PW = 1100
T200_MAX_PW = 1900
T200_MIN_ZERO_PW = 1472
T200_MAX_ZERO_PW = 1528

TEENSY_NTHRUSTERS = 4
TEENSY_PWM_PINS = [0, 1, 4, 5]
TEENSY_PWM_FREQ = 500
TEENSY_PWM_BIT_RES = 8
TEENSY_INIT_DC = round((T200_INIT_PW * (TEENSY_PWM_FREQ / 1e6)) * 2**TEENSY_PWM_BIT_RES)

NEWTONS_PER_KGF = 9.80665

datasheet_pw_vals = [] # pw_vals: pulse widths
datasheet_f_vals = [] # fs: force values
with open('t200_18v_data.csv') as f:
    reader = csv.DictReader(f)
    for row in reader:
        pw, kgF = int(row['PWM (µs)']), float(row['Force (Kg f)'])
        datasheet_pw_vals.append(pw)
        datasheet_f_vals.append(kgF * NEWTONS_PER_KGF)
assert len(datasheet_pw_vals) == len(datasheet_f_vals), 'datasheet_pw_vals must be same length as datasheet_f_vals'

spline = CubicSpline(datasheet_pw_vals, datasheet_f_vals)
pw_to_f = lambda pw: 0 if T200_MIN_ZERO_PW <= pw <= T200_MAX_ZERO_PW else spline(pw)

# dc_vals: duty cycle values (between 0 and 2^(pwm bit resolution))
dc_to_us = lambda dc: dc / (2**TEENSY_PWM_BIT_RES) / (TEENSY_PWM_FREQ / 1e6)

teensy_dc_vals = [dc for dc in range(2**TEENSY_PWM_BIT_RES) if T200_MIN_PW <= dc_to_us(dc) <= T200_MAX_PW]
teensy_pw_vals = [dc_to_us(dc) for dc in teensy_dc_vals]
teensy_f_vals = [pw_to_f(pw) for pw in teensy_pw_vals]

# for dc, pw, f in zip(teensy_dc_vals, teensy_pw_vals, teensy_f_vals):
#     print(dc, pw, f)


out = f'''\
constexpr int NTHRUSTERS = {TEENSY_NTHRUSTERS};
constexpr int PWM_PINS[] = {{ {','.join(map(str, TEENSY_PWM_PINS))} }};
constexpr float PWM_FREQ = {TEENSY_PWM_FREQ};
constexpr int PWM_BIT_RES = {TEENSY_PWM_BIT_RES};
constexpr uint16_t INIT_DC = {TEENSY_INIT_DC};

constexpr int LUT_LEN = {len(teensy_dc_vals)};
constexpr uint16_t dc_vals[] = {{ {','.join(map(str, teensy_dc_vals))} }};
constexpr float f_vals[] = {{ {','.join([f'{val:.2f}f' for val in teensy_f_vals])} }};
'''

with open('barracuda-teensy.h', 'w') as f:
    f.write(out)