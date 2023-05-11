/**************************************************
File:        arduino_actuators.cpp
Description: Trucky Project Arduino code for Actuators Control with ROS
Author:      MCI. Aldo Iván Aguilar Aldecoa
Contact:     aldo_aldecoa@tec.mx
Date:        April 2023
***************************************************/

#define PWM_CONTROL    true
#define PID_ADJUST     false

#include "ros/node_handle.h"
#include "ArduinoHardware.h"

namespace ros
{
#if PWM_CONTROL == true
  #define PUBLISHERS 0
  #define SUBSCRIBERS 2
#else
  #if PID_ADJUST == true
    #define PUBLISHERS 0
    #define SUBSCRIBERS 3
  #else
    #define PUBLISHERS 0
    #define SUBSCRIBERS 2
  #endif
#endif
  typedef NodeHandle_<ArduinoHardware, SUBSCRIBERS, PUBLISHERS, 280, 280> NodeHandle;
}

#if PWM_CONTROL == true
  #include <trucky_custom_msgs/ActuatorsState.h>
#else
  #include <Wire.h>
  #include <ackermann_msgs/AckermannDrive.h>
  #if PID_ADJUST == true
    #include <trucky_custom_msgs/PIDGains.h> // **
  #endif
#endif

#include <Arduino.h>

#define DEBUG_PIN      A6
#define INTERRUPT_PIN  8
#define MOTOR_PIN      9
#define SERVO_PIN      10

#define AUTO_MODE     HIGH
#define MANUAL_MODE   LOW

#define I2C_ID        1


/*--------------------------------------------------
          ---  Configuration Parameters  ---

These parameters must be replaced for the correct operation of the robot.

  - MOTOR_PAIRING_PWM_HIGH_TIME  -> Is the BLDC Motor PWM High Time (us) required for RF pairing.
  - MOTOR_MIN_MOVE_PWM_HIGH_TIME -> Is the minimum BLDC Motor PWM High Time (us) required to move the robot.
  - MOTOR_MAX_PWM_HIGH_TIME      -> Is the maximum desired BLDC Motor PWM High Time (us). It restricts the maximum speed of the robot.

  - SERVO_MIN_PWM_HIGH_TIME    -> Is the minimum desired Servo Motor PWM High Time (us).
  - SERVO_CENTER_PWM_HIGH_TIME -> Is the Servo Motor PWM High Time (us) required to place the steering wheels at the center position.
  - SERVO_MAX_PWM_HIGH_TIME    -> Is the maximum desired Servo Motor PWM High Time (us)

  - SERVO_CHARACTERIZATION_SLOPE_CONSTANT     -> Is the Servo Motor Characterization Slope constant for the steering angle calculation.
  - SERVO_CHARACTERIZATION_INTERCEPT_CONSTANT -> Is the Servo Motor Characterization Intercept constant for the steering angle calculation.

  - KP -> PID controller Proportional Gain.
  - KI -> PID controller Integral Gain.
  - KD -> PID controller Derivative Gain.
--------------------------------------------------*/

#if PWM_CONTROL == false
  #define MOTOR_PAIRING_PWM_HIGH_TIME   1478
  #define MOTOR_MIN_MOVE_PWM_HIGH_TIME  1546
  #define MOTOR_MAX_PWM_HIGH_TIME       1700

  #define SERVO_MIN_PWM_HIGH_TIME       1060
  #define SERVO_CENTER_PWM_HIGH_TIME    1475
  #define SERVO_MAX_PWM_HIGH_TIME       1897

  const float SERVO_CHARACTERIZATION_SLOPE_CONSTANT = 1.25;
  const float SERVO_CHARACTERIZATION_INTERCEPT_CONSTANT = 0.25;

  volatile float KP = 0.0;
  volatile float KI = 0.0;
  volatile float KD = 0.0;
#endif

//--------------------------------------------------


#if PWM_CONTROL == false
  // PID 
  float PID_P = 0; 
  float PID_I = 0; 
  float PID_D = 0;

  const float PID_THRESH = 100.0;
  float PID_ERROR = 0;
  float PID_PREV_ERROR = 0;

  volatile bool RUN_PID_ROUTINE = false;

  volatile float  PID_SAMPLING_PERIOD_SEC  = 0;
  volatile unsigned int PID_LAST_SAMPLING_TIME  = 0;

  // Ackermann control
  volatile float desired_speed;
  volatile float desired_steering_angle;

  // I2C
  volatile float current_speed = 0;
  volatile char i2c_speed_str[10];
  volatile char i2c_input_char;
  volatile int  i2c_msg_id;

#endif

// PWM control
volatile int servo_pwm_high_time;
volatile int motor_pwm_high_time;

// ROS
ros::NodeHandle nh;

#if PWM_CONTROL == false
  /* 
    Receives I2C Messages from the Arduino Sensors board to determine the current robot speed.
    The PID Sampling Period is defined based on the reception time of the speed estimation. 
  */
  void receiveI2CMessage(int num_bytes)
  {
    i2c_input_char = -1;
    i2c_msg_id = 0;

    for (byte i = 0; i < 10; i++) i2c_speed_str[i] = 0;

    for (byte i = 0; i < num_bytes; i++)
    {
      i2c_input_char = Wire.read();
      i2c_speed_str[i2c_msg_id] = i2c_input_char;
      i2c_msg_id++;
      i2c_speed_str[i2c_msg_id] = '\0';
    }

    current_speed = atof((const char*)i2c_speed_str);

    PID_SAMPLING_PERIOD_SEC = (float)(millis() - PID_LAST_SAMPLING_TIME) / 1000.0;
    PID_LAST_SAMPLING_TIME = millis();
    RUN_PID_ROUTINE = true;
  }
#endif

/* 
  Initializes the Atmega328P PWM Timer (Counter 1)
  Phase Correct PWM Operation Mode:
    - PWM Frequency set at 50HZ (20 ms period).
    - f = f_clk / (2 * N * TOP)
      Where:
        f     -> PWM  Frequency
        f_clk -> Atmega328P Clock Frequency (16MHz)
        N     -> Is the clock prescaler value (8)
        TOP   -> Is the timer maximum counter value (20000)
      f = 16E6 / (2 * 8 * 20000) = 50Hz = 20 ms
    - As the TOP value (ICR1) is set to 20000 and the period is set to 20 ms, 
      its possible to establish a minimum PWM High Time of 1us.
*/
void initPWMTimer() 
{
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS11);
  ICR1 = 20000;
}

/*
  Gets the Robot operation Mode (AUTO or MANUAL) by obtaining the INTERRUPT Pin digital state.
*/
bool getOperatingMode()
{
  return PINB & B00000001; // digitalRead(INTERRUPT_PIN);
}

/*
  Sets the BLDC Motor PWM High Time in us.
*/
void setMotorPWM(unsigned int motor_pwm_high_time_us)
{
  OCR1A = motor_pwm_high_time_us;
}

/*
  Sets the Servo Motor PWM High Time in us.
*/
void setServoPWM(int servo_pwm_high_time_us)
{
  OCR1B = servo_pwm_high_time_us;
}

#if PWM_CONTROL == false
  /*
    Ouputs the desired PWM High Time values to the BLDC and Servo Motors.
    Compares the desired PWM values to the Minimum and Maximum operation values of each actuator
    to prevent non desired behaviour. 
  */
  void outputPWM(unsigned int motor_pwm_high_time_us, unsigned int servo_pwm_high_time_us)
  {
    if (motor_pwm_high_time_us <= MOTOR_PAIRING_PWM_HIGH_TIME || desired_speed <= 0)
    {
      motor_pwm_high_time_us = MOTOR_PAIRING_PWM_HIGH_TIME;
    }
    else if (motor_pwm_high_time_us <= MOTOR_MIN_MOVE_PWM_HIGH_TIME)
    {
      motor_pwm_high_time_us = MOTOR_MIN_MOVE_PWM_HIGH_TIME;
    }
    else if (motor_pwm_high_time_us >= MOTOR_MAX_PWM_HIGH_TIME)
    {
      motor_pwm_high_time_us = MOTOR_MAX_PWM_HIGH_TIME;
    }
    
    if (servo_pwm_high_time_us <= SERVO_MIN_PWM_HIGH_TIME)
    {
      servo_pwm_high_time_us = SERVO_MIN_PWM_HIGH_TIME;
    }

    else if (servo_pwm_high_time_us >= SERVO_MAX_PWM_HIGH_TIME)
    {
      servo_pwm_high_time_us = SERVO_MAX_PWM_HIGH_TIME;
    }

    setMotorPWM(motor_pwm_high_time_us);
    setServoPWM(servo_pwm_high_time_us);
  }
#endif

/*
  Forces the desired PWM High Time values to the BLDC and Servo Motors.
  NOTE: Does not consider the Minimum and Maximum operation values of each actuator
  to prevent non desired behaviour. USE CAREFULY!
*/
void forcePWM(unsigned int motor_pwm_high_time_us, unsigned int servo_pwm_high_time_us)
{
  setMotorPWM(motor_pwm_high_time_us);
  setServoPWM(servo_pwm_high_time_us);  
}

/*
  Function used to control the robot actuators based on the ackermann or PWM control modes.
  Implements the PID contoller for the desired speed and uses the servo characterization for steering angle.
*/
void controlActuators()
{
  #if PWM_CONTROL == true
    forcePWM(motor_pwm_high_time, servo_pwm_high_time);
  #else
    if (getOperatingMode() == MANUAL_MODE)
    {
      PID_P = 0; PID_I = 0; PID_D = 0;

      PID_ERROR      = 0;
      PID_PREV_ERROR = 0;

      return;
    }

    servo_pwm_high_time = ((float) desired_steering_angle - SERVO_CHARACTERIZATION_INTERCEPT_CONSTANT) / SERVO_CHARACTERIZATION_SLOPE_CONSTANT;

    if (RUN_PID_ROUTINE)
    {
      RUN_PID_ROUTINE = false;

      PID_PREV_ERROR = PID_ERROR;
      PID_ERROR      = (float)(desired_speed - current_speed);

      PID_P = KP * PID_ERROR;
      PID_I = max(- PID_THRESH , min(PID_I + KI * (PID_SAMPLING_PERIOD_SEC * PID_ERROR), PID_THRESH));
      PID_D = KD * (PID_ERROR - PID_PREV_ERROR) / PID_SAMPLING_PERIOD_SEC;

      motor_pwm_high_time = PID_P + PID_I + PID_D + MOTOR_MIN_MOVE_PWM_HIGH_TIME;
    }
    
    outputPWM(motor_pwm_high_time, servo_pwm_high_time);
  #endif
}

#if PWM_CONTROL == false
  /*
    Callback function used for Ackermann Commands reception.
  */
  void ackermann_cmd_cb( const ackermann_msgs::AckermannDrive& ackermann_cmd)
  {
    desired_speed = ackermann_cmd.speed;
    desired_steering_angle = ackermann_cmd.steering_angle;
  }

  #if PID_ADJUST == true
    /*
      Callback function used for PID Gains values reception.
    */
    void pid_gains_cb( const trucky_custom_msgs::PIDGains& pid_gains) // **
    {
      KP = pid_gains.kp;
      KD = pid_gains.kd;
      KI = pid_gains.ki;
    }
  #endif
#else
  /*
    Callback function used for Actuators State Commands (PWM High Times) reception.
  */
  void actuators_cmd_cb( const trucky_custom_msgs::ActuatorsState& actuators_cmd)
  {
    servo_pwm_high_time = actuators_cmd.servo_pwm_high_time;
    motor_pwm_high_time = actuators_cmd.motor_pwm_high_time; 
  }
#endif

/*
ROS Subscribers for rosserial
*/
#if PWM_CONTROL == false
  ros::Subscriber<ackermann_msgs::AckermannDrive> ackermann_cmd_sub("/ackermann_cmd", &ackermann_cmd_cb );
  #if PID_ADJUST == true
    ros::Subscriber<trucky_custom_msgs::PIDGains> pid_gains_sub("/pid_gains", &pid_gains_cb); // **
  #endif
#else
  ros::Subscriber<trucky_custom_msgs::ActuatorsState> actuators_cmd_sub("/actuators_cmd", &actuators_cmd_cb);
#endif

/*
  The code will get blocked if the digital state on the DEBUG pin is set HIGH.
  This means that the incorrect code has been loaded to the board, if so the board LED will blink.
*/
void setup()
{
  pinMode(DEBUG_PIN, INPUT);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  if (analogRead(DEBUG_PIN) != LOW)
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
    nh.subscribe(ackermann_cmd_sub);
    #if PID_ADJUST == true
      nh.subscribe(pid_gains_sub); // **
    #endif
  #else
    nh.subscribe(actuators_cmd_sub);
  #endif

  while(!nh.connected())
  {
    nh.spinOnce();
  }

  #if PWM_CONTROL == false
    Wire.begin(I2C_ID);
    Wire.onReceive(receiveI2CMessage);
  #endif

  initPWMTimer();
}

/*
  Infinite loop responsible of controlling the robot actuators.
  The board LED will follow the current robot operation mode (AUTO or MANUAL)
*/
void loop()
{
  nh.spinOnce(); 
  
  digitalWrite(LED_BUILTIN, getOperatingMode());
  controlActuators();
}