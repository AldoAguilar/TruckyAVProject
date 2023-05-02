/**************************************************
File:        arduino_sensors.cpp
Description: Trucky Project Arduino code for Sensors integration with ROS
Author:      MCI. Aldo Iván Aguilar Aldecoa
Contact:     aldo_aldecoa@tec.mx
Date:        April 2023
***************************************************/

#define PWM_CONTROL    true

#include "ros/node_handle.h"
#include "ArduinoHardware.h"

namespace ros
{
#if PWM_CONTROL == true
  #define PUBLISHERS 2
  #define SUBSCRIBERS 0
#else
  #define PUBLISHERS 3
  #define SUBSCRIBERS 0
#endif
  typedef NodeHandle_<ArduinoHardware, SUBSCRIBERS, PUBLISHERS, 280, 280> NodeHandle;
}

#include <std_msgs/Float32.h>

#if PWM_CONTROL == true
  #include <custom_msgs/ActuatorsState.h>
#else
  #include <Wire.h>
#endif

#include <Arduino.h>

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

/*--------------------------------------------------
          ---  Configuration Parameters  ---

These parameters must be replaced for the correct operation of the robot.

  - ROS_PERIOD_MS                       -> ROS Node Publishing Period in miliseconds.

  - SAMPLING_PERIOD_MS                  -> Encoder Velocity sampling period in miliseconds.
  - ENCODER_COUNT_TO_WHEEL_TURNS_CONST  -> Number of encoder counts per wheel turn.
  - WHEEL_RADIUS_MM                     -> Robot Wheel Radius in milimeters.

  - SERVO_CHARACTERIZATION_SLOPE_CONSTANT     -> Is the Servo Motor Characterization Slope constant for the steering angle calculation.
  - SERVO_CHARACTERIZATION_INTERCEPT_CONSTANT -> Is the Servo Motor Characterization Intercept constant for the steering angle calculation.
--------------------------------------------------*/

#define ROS_PERIOD_MS 100

#define SAMPLING_PERIOD_MS                  200
#define ENCODER_COUNT_TO_WHEEL_TURNS_CONST  20
const float WHEEL_RADIUS_MM = 52.5;


#if PWM_CONTROL == false
  const float SERVO_CHARACTERIZATION_SLOPE_CONSTANT = 1.25;
  const float SERVO_CHARACTERIZATION_INTERCEPT_CONSTANT = 0.25;
#endif

//--------------------------------------------------

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

#if PWM_CONTROL == true
  volatile bool motor_pwm_state = LOW;
  volatile unsigned int motor_pwm_high_state_init_time_us;
  volatile int motor_pwm_high_time;
  volatile bool operating_mode;
#else
  // I2C
  volatile bool i2c_send;
#endif

// ROS
ros::NodeHandle nh;

unsigned long ros_time;

#if PWM_CONTROL == false
  std_msgs::Float32 wr_msg;
  std_msgs::Float32 wl_msg;

  std_msgs::Float32 steering_angle_msg;
#else
  custom_msgs::ActuatorsState actuators_state_msg;
  std_msgs::Float32 speed_msg;
#endif

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
  #if PWM_CONTROL == true
    PCMSK0 |= (1 << PCINT3); // Motor Pin Interrupt PCINT3 -> D11
  #endif
  PCMSK0 |= (1 << PCINT4); // Servo Pin Interrupt PCINT4 -> D12

  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20); // Left Encoder Interrupt PCINT20 -> D4
  PCMSK2 |= (1 << PCINT21); // Right Encoder Interrupt PCINT21 -> D5
  // PCMSK2 |= (1 << PCINT22);
  // PCMSK2 |= (1 << PCINT23);  
}

#if PWM_CONTROL == true
  /*
    Sets the robot operating mode by getting the INTERRUPT_PIN digital state
  */
  void setOperatingMode()
  {
    operating_mode = (PIND & B00000100); // PD2 -> D2
  }
#endif

#if PWM_CONTROL == false
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
#endif

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

  #if PWM_CONTROL == false
    i2c_send = true;
  #endif
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
  #if PWM_CONTROL == true
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
  #endif
}

/*
ROS Publishers for rosserial
*/
#if PWM_CONTROL == false
  ros::Publisher wr("/wr", &wr_msg);
  ros::Publisher wl("/wl", &wl_msg);

  ros::Publisher steering_angle("/steering_angle", &steering_angle_msg);
#else
  ros::Publisher actuators_state("/actuators_state", &actuators_state_msg);
  ros::Publisher speed("/speed", &speed_msg);
#endif

/*
  The code will get blocked if the digital state on the DEBUG pin is set LOW.
  This means that the incorrect code has been loaded to the board, if so the board LED will blink.
*/
void setup()
{
  pinMode(DEBUG_PIN, INPUT);
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);
  pinMode(SERVO_PIN, INPUT);
  pinMode(MOTOR_PIN, INPUT);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  if (digitalRead(DEBUG_PIN) == LOW)
  {
    bool LED_VAL = HIGH;
    while (true)
    {
      digitalWrite(LED_BUILTIN, LED_VAL);
      LED_VAL = !LED_VAL;
      delay(500);
    }
  }

  nh.getHardware()->setBaud(115200);
  nh.initNode();   
  #if PWM_CONTROL == false 
    nh.advertise(wr);
    nh.advertise(wl);
    nh.advertise(steering_angle);
  #else
    nh.advertise(actuators_state);
    nh.advertise(speed);
  #endif

  while(!nh.connected())
  {
    nh.spinOnce();
  }

  #if PWM_CONTROL == false
    Wire.begin();
  #endif

  initTimer();
  initPinChangeInterrupt();

  #if PWM_CONTROL == true
    setOperatingMode();
    attachInterrupt(INTERRUPT_INT, setOperatingMode, CHANGE);
  #endif

  ros_time = millis();
}

/*
  Infinite loop responsible of transmiting the robot sensors state.
*/
void loop()
{
  #if PWM_CONTROL == false
    sendI2CMessage();
  #endif

  if (millis() - ros_time < ROS_PERIOD_MS) return;

  #if PWM_CONTROL == true
    actuators_state_msg.motor_pwm_high_time = motor_pwm_high_time;
    actuators_state_msg.servo_pwm_high_time = servo_pwm_high_time;
    actuators_state_msg.output_mode = operating_mode == AUTO_MODE ? "AUTO" : "MANUAL";

    speed_msg.data = current_speed;

    actuators_state.publish( &actuators_state_msg );
    speed.publish( &speed_msg );
  #else
    wr_msg.data = right_wheel_angular_velocity;
    wl_msg.data = left_wheel_angular_velocity;
    steering_angle_msg.data = ((float) servo_pwm_high_time * SERVO_CHARACTERIZATION_SLOPE_CONSTANT) +  SERVO_CHARACTERIZATION_INTERCEPT_CONSTANT;

    wl.publish( &wl_msg );
    wr.publish( &wr_msg );
    steering_angle.publish( &steering_angle_msg );
  #endif

  nh.spinOnce();

  ros_time = millis();
}