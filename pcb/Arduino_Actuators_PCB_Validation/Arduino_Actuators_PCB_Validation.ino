/**************************************************
File:        Arduino_Actuators_PCB_Validation.INO
Description: Trucky Project Arduino code for Actuators PCB Validation
Author:      MCI. Aldo Iván Aguilar Aldecoa
Contact:     aldo_aldecoa@tec.mx
Date:        April 2023
***************************************************/

#include <Wire.h>

#define DEBUG_PIN      A6
#define INTERRUPT_PIN  8
#define MOTOR_PIN      9
#define SERVO_PIN      10

#define AUTO_MODE     HIGH
#define MANUAL_MODE   LOW

#define I2C_ID        1

const unsigned int MOTOR_IDLE_CNT = 1478;
const unsigned int SERVO_IDLE_CNT = 1475;

const unsigned int MOTOR_OP_CNT  = 1546;
const unsigned int SERVO_OP_CNT  = 1600;

bool PREV_OPERATING_MODE = MANUAL_MODE;
bool OPERATING_MODE = MANUAL_MODE;
unsigned int MOTOR_PWM_HIGH_TIME_US;
unsigned int SERVO_PWM_HIGH_TIME_US;

volatile float I2C_MESSAGE = 0.0;

volatile char i2c_message_str[10];
volatile char i2c_input_char;
volatile int  i2c_msg_id;

/*
  Gets the Robot operation Mode (AUTO or MANUAL) by obtaining the INTERRUPT Pin digital state.
*/
bool getOperatingMode()
{
  return PINB & B00000001; // digitalRead(INTERRUPT_PIN);
}

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

/* 
  Receives I2C Messages from the Arduino Sensors board to determine the current robot speed.
  The PID Sampling Period is defined based on the reception time of the speed estimation. 
*/
void receiveI2CMessage(int num_bytes)
{
  i2c_input_char = -1;
  i2c_msg_id = 0;

  for (byte i = 0; i < 10; i++) i2c_message_str[i] = 0; 

  for (byte i = 0; i < num_bytes; i++)
  {
    i2c_input_char = Wire.read();
    i2c_message_str[i2c_msg_id] = i2c_input_char;
    i2c_msg_id++;
    i2c_message_str[i2c_msg_id] = '\0';
  }

  I2C_MESSAGE = atof((const char*)i2c_message_str);
}

void controlActuators()
{
  if (PREV_OPERATING_MODE == OPERATING_MODE) return;
  if (OPERATING_MODE == MANUAL_MODE)
  {
    setServoPWM(0);
    setMotorPWM(0);
    PREV_OPERATING_MODE = OPERATING_MODE;
    return;
  }
  long init_time = millis();
  while (millis() - init_time < 4000)
  {
    setServoPWM(0);
    setMotorPWM(0);
  }

  init_time = millis();
  while (millis() - init_time < 4000)
  {
    setServoPWM(SERVO_IDLE_CNT);
    setMotorPWM(MOTOR_IDLE_CNT);
  }
  setServoPWM(SERVO_OP_CNT);
  setMotorPWM(MOTOR_OP_CNT);
}

void setup()
{
  pinMode(DEBUG_PIN, INPUT);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);  

  initPWMTimer();

  Wire.begin(I2C_ID);
  Wire.onReceive(receiveI2CMessage);

  Serial.begin(9600);
}

/*
  Infinite loop responsible of controlling the robot actuators.
  The board LED will follow the current robot operation mode (AUTO or MANUAL)
*/
void loop()
{
  OPERATING_MODE = getOperatingMode();
  controlActuators();
  Serial.print(OPERATING_MODE == AUTO_MODE ? "AUTO" : "MANUAL");
  Serial.print(",");
  Serial.println(I2C_MESSAGE);
}
