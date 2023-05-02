/**************************************************
File:        Arduino_Actuators_PCB_Validation.INO
Description: Trucky Project Arduino code for Actuators PCB Validation
Author:      MCI. Aldo Iván Aguilar Aldecoa
Contact:     aldo_aldecoa@tec.mx
Date:        April 2023
***************************************************/

#include <Wire.h>

#define DEBUG_PIN          A6
#define INTERRUPT_PIN      2
#define INTERRUPT_INT      0
#define MOTOR_PIN          11
#define SERVO_PIN          12
#define LEFT_ENCODER_PIN   4
#define RIGHT_ENCODER_PIN  5

#define AUTO_MODE     HIGH
#define MANUAL_MODE   LOW

#define I2C_ID        1

#define SAMPLING_PERIOD_MS                  200
#define ENCODER_COUNT_TO_WHEEL_TURNS_CONST  20
const float WHEEL_RADIUS_MM = 52.5;

// Encoders
volatile bool left_encoder_state = LOW;
volatile bool right_encoder_state = LOW;

volatile unsigned int left_encoder_pulses_cnt;
volatile unsigned int right_encoder_pulses_cnt;

volatile unsigned int prev_left_encoder_pulses_cnt;
volatile unsigned int prev_right_encoder_pulses_cnt;

volatile float right_wheel_angular_velocity = 0.0;
volatile float left_wheel_angular_velocity = 0.0;

volatile float right_wheel_linear_velocity = 0.0;
volatile float left_wheel_linear_velocity = 0.0;

volatile unsigned int encoder_timer_elapsed_time_ms = 0;

volatile float current_speed = 0;

// PWM High Times
volatile bool servo_pwm_state = LOW;
volatile unsigned int servo_pwm_high_state_init_time_us;
volatile int servo_pwm_high_time;

volatile bool motor_pwm_state = LOW;
volatile unsigned int motor_pwm_high_state_init_time_us;
volatile int motor_pwm_high_time;
volatile bool operating_mode;

volatile bool i2c_send;

long init_time;

/* 
  Initializes the Atmega328P Timer (Counter 2)
  Clear Timer on Compare Match (CTC) Operation Mode:
    - Timer Frequency set at 500HZ (2 ms period).
    - f = f_clk / (2 * N * (1 + OCR))
      Where:
        f     -> PWM  Frequency
        f_clk -> Atmega328P Clock Frequency (16MHz)
        N     -> Is the clock prescaler value (128)
        OCR   -> Is the timer maximum counter value (124)
      f = 16E6 / (2 * 128 * (1 + 124)) = 500Hz = 2 ms
    - As the TOP value (OCR) is set to 124, the period is set to 2 ms.
      Therefore a Timer Interrupt is toggled each 1 ms (half the period).
*/
void initTimer()
{
  TCCR2A = _BV(WGM21);
  TCCR2B =  _BV(CS22) | _BV(CS20);
  OCR2A  = 124;
  TIMSK2 = (1 << OCIE2A);  
}

/*
  Enables Pin Change Interrupts for the Encoder and Acutator's PWM signals
  Acquisition.
*/
void initPinChangeInterrupt()
{
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT3); // Motor Pin Interrupt PCINT3 -> D11
  PCMSK0 |= (1 << PCINT4); // Servo Pin Interrupt PCINT4 -> D12

  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20); // Left Encoder Interrupt PCINT20 -> D4
  PCMSK2 |= (1 << PCINT21); // Right Encoder Interrupt PCINT21 -> D5
}

/*
  Sets the robot operating mode by getting the INTERRUPT_PIN digital state
*/
void setOperatingMode()
{
  operating_mode = (PIND & B00000100); // PD2 -> D2
}

/*
  Gets the Wheel Linear Velocity based on its angular velocity estimation.
*/
float getWheelSpeed(float wheel_angular_velocity)
{
  return ((float) WHEEL_RADIUS_MM / 1000.0) * wheel_angular_velocity;
}

/*
  Gets the Wheel Angular Velocity based on the current detected encoder counts.
*/
float getWheelAngularVelocity(unsigned int encoder_cnt)
{
  return (((2.0 * PI) * (float) encoder_cnt) / ENCODER_COUNT_TO_WHEEL_TURNS_CONST) / ((float) SAMPLING_PERIOD_MS / 1000.0);
}

/*
  Timer Interrupt Function used to obtain the current robot speed based on the Encoder pulse counts.
*/
ISR(TIMER2_COMPA_vect)
{
  encoder_timer_elapsed_time_ms++;

  if (encoder_timer_elapsed_time_ms != SAMPLING_PERIOD_MS) return;

  unsigned int left_encoder_cnt_diff  = left_encoder_pulses_cnt - prev_left_encoder_pulses_cnt;
  unsigned int right_encoder_cnt_diff = right_encoder_pulses_cnt - prev_right_encoder_pulses_cnt;

  unsigned int left_wheel_encoder_ok = left_encoder_cnt_diff != 0 ? 1 : 0;
  unsigned int right_wheel_encoder_ok = right_encoder_cnt_diff != 0 ? 1 : 0;

  digitalWrite(LED_BUILTIN, left_wheel_encoder_ok && right_wheel_encoder_ok);

  left_wheel_angular_velocity = getWheelAngularVelocity(left_encoder_cnt_diff);
  right_wheel_angular_velocity = getWheelAngularVelocity(right_encoder_cnt_diff);

  left_wheel_linear_velocity = getWheelSpeed(left_wheel_angular_velocity);
  right_wheel_linear_velocity = getWheelSpeed(right_wheel_angular_velocity);

  unsigned int wheel_ok_cnt = left_wheel_encoder_ok + right_wheel_encoder_ok;

  current_speed = wheel_ok_cnt > 0 ? (right_wheel_linear_velocity + left_wheel_linear_velocity) /  wheel_ok_cnt : 0;

  prev_left_encoder_pulses_cnt = left_encoder_pulses_cnt;
  prev_right_encoder_pulses_cnt = right_encoder_pulses_cnt;

  encoder_timer_elapsed_time_ms = 0;

  i2c_send = true;
}

/*
  Pin Change Interrupt function used to obtain Encoder pulse counts 
*/
ISR(PCINT2_vect) {
  if((PIND & B00010000) >> 4 != left_encoder_state)
  {
    left_encoder_pulses_cnt++;
    left_encoder_state = !left_encoder_state;
  }
  if((PIND & B00100000) >> 5 != right_encoder_state)
  {
    right_encoder_pulses_cnt++;
    right_encoder_state = !right_encoder_state;
  }
}

/*
  Pin Change Interrupt function used to obtain PWM High Time values in microseconds
*/
ISR(PCINT0_vect) {
  unsigned int current_time_us = micros();
  if((PINB & B00010000) >> 4 != servo_pwm_state)
  {
    if((PINB & B00010000) >> 4 == HIGH)
    {
      servo_pwm_high_state_init_time_us = current_time_us;
    }
    else
    {
      unsigned int new_servo_pwm_high_time = current_time_us - servo_pwm_high_state_init_time_us;
      servo_pwm_high_time = new_servo_pwm_high_time > 2000 ? servo_pwm_high_time : new_servo_pwm_high_time;
      servo_pwm_high_state_init_time_us = 0;
    }
    servo_pwm_state = !servo_pwm_state;
  }
  if((PINB & B00001000) >> 3 != motor_pwm_state)
  {
    if((PINB & B00001000) >> 3 == HIGH)
    {
      motor_pwm_high_state_init_time_us = current_time_us;  
    }
    else
    {
      unsigned int new_motor_pwm_high_time = current_time_us - motor_pwm_high_state_init_time_us;
      motor_pwm_high_time = new_motor_pwm_high_time > 2000 ? motor_pwm_high_time : new_motor_pwm_high_time;
      motor_pwm_high_state_init_time_us = 0;
    }
    motor_pwm_state = !motor_pwm_state;
  }
}

/* 
  Sends I2C Messages to the Arduino Actuators board to communicate the current robot speed.
*/
void sendI2CMessage()
{
  if (!i2c_send) return;
  char i2c_speed_str[10];
  dtostrf(current_speed, 6, 3, i2c_speed_str);

  Wire.beginTransmission(I2C_ID);
  Wire.write(i2c_speed_str);
  Wire.endTransmission();

  i2c_send = false;
}

void setup()
{
  pinMode(DEBUG_PIN, INPUT);
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);
  pinMode(SERVO_PIN, INPUT);
  pinMode(MOTOR_PIN, INPUT);
  pinMode(INTERRUPT_PIN, INPUT);   

  Wire.begin();

  initTimer();
  initPinChangeInterrupt();

  setOperatingMode();
  attachInterrupt(INTERRUPT_INT, setOperatingMode, CHANGE);

  Serial.begin(9600);

  init_time = millis();
}

void loop()
{
  sendI2CMessage();

  if (millis() - init_time < 1000) return;

  Serial.print("OP: ");
  Serial.print(operating_mode);
  Serial.print(",");
  Serial.print("wl: ");
  Serial.print(left_wheel_angular_velocity);
  Serial.print(",");
  Serial.print("wr: ");
  Serial.print(right_wheel_angular_velocity);
  Serial.print(",");
  Serial.print("motor: ");
  Serial.print(motor_pwm_high_time);
  Serial.print(",");
  Serial.print("servo: ");
  Serial.println(servo_pwm_high_time);

  init_time = millis();
}