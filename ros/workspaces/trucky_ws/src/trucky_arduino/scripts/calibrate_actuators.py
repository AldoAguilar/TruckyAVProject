#!/usr/bin/env python

import rospy
import rosnode
import rostopic
import time
import numpy as np
import matplotlib.pyplot as plt
from trucky_custom_msgs.msg import ActuatorsState
from std_msgs.msg import Float32

CALIBRATION_TIME = 5

def validateInitialConditions():
    if output_mode == "MANUAL":
        rospy.logwarn("Set the vehicle Automated Mode using the RF Control.")

    while output_mode == "MANUAL": continue

    rospy.loginfo("Correct Pairing and Automated Operation Mode.")

    actuators_cmd = ActuatorsState()
    actuators_cmd.motor_pwm_high_time = 0
    actuators_cmd.servo_pwm_high_time = 0

    actuators_state_pub.publish(actuators_cmd)

    start_time = time.time()
    while time.time() - start_time <= CALIBRATION_TIME: continue

    if output_mode == "AUTO" or motor_pwm_high_time == 0 or servo_pwm_high_time == 0:
        rospy.logwarn("Set the vehicle Manual Mode using the RF Control.")
        rospy.logwarn("WARNING: Do not move any stick on your RF Control.")

    while output_mode == "AUTO" or motor_pwm_high_time == 0 or servo_pwm_high_time == 0: continue

    rospy.loginfo("Correct RF Pairing and Manual Operation Mode.")
    rospy.logwarn("WARNING: Do not turn off your RF Controller!")

def getActuatorsMeanState():
    rospy.loginfo("Press enter to continue...")
    while raw_input() != "": continue

    rospy.loginfo("Calibrating for " + str(CALIBRATION_TIME) + " seconds...")

    mean_servo_pwm_high_time = 0
    mean_motor_pwm_high_time = 0
    measurement_count = 0
    start_time = time.time()
    while time.time() - start_time <= CALIBRATION_TIME:
        mean_servo_pwm_high_time += servo_pwm_high_time
        mean_motor_pwm_high_time += motor_pwm_high_time
        measurement_count += 1

    mean_servo_pwm_high_time = round(mean_servo_pwm_high_time / measurement_count)
    mean_motor_pwm_high_time = round(mean_motor_pwm_high_time / measurement_count)

    return int(mean_motor_pwm_high_time), int(mean_servo_pwm_high_time)

def getMotorDeadZoneState():
    if output_mode == "MANUAL":
        rospy.logwarn("Set the vehicle Automated Mode using the RF Control.")

    while output_mode == "MANUAL": continue

    rospy.loginfo("Correct Pairing and Automated Operation Mode.")

    mean_motor_dead_zone_pwm_high_time = 0
    for _ in range(10):
        curr_motor_pwm_high_time = motor_pairing_pwm_high_time

        actuators_cmd = ActuatorsState()
        actuators_cmd.motor_pwm_high_time = 0
        actuators_cmd.servo_pwm_high_time = 0

        actuators_state_pub.publish(actuators_cmd)

        start_time = time.time()
        while time.time() - start_time <= 3: continue

        actuators_cmd = ActuatorsState()
        actuators_cmd.motor_pwm_high_time = curr_motor_pwm_high_time
        actuators_cmd.servo_pwm_high_time = servo_center_pwm_high_time
    
        actuators_state_pub.publish(actuators_cmd)

        start_time = time.time()

        while speed == 0:
            if time.time() - start_time < 3: continue

            curr_motor_pwm_high_time = curr_motor_pwm_high_time + 1

            actuators_cmd = ActuatorsState()
            actuators_cmd.motor_pwm_high_time = curr_motor_pwm_high_time
            actuators_cmd.servo_pwm_high_time = servo_center_pwm_high_time

            actuators_state_pub.publish(actuators_cmd)
            
            start_time = time.time()

        mean_motor_dead_zone_pwm_high_time += curr_motor_pwm_high_time

    actuators_cmd = ActuatorsState()
    actuators_cmd.motor_pwm_high_time = 0
    actuators_cmd.servo_pwm_high_time = 0

    actuators_state_pub.publish(actuators_cmd)

    mean_motor_dead_zone_pwm_high_time = round(mean_motor_dead_zone_pwm_high_time / 10)

    return int(mean_motor_dead_zone_pwm_high_time)

def getServoCharacterization():
    servo_min_pwm_high_time = min(servo_left_pwm_high_time, servo_right_pwm_high_time)
    servo_max_pwm_high_time = max(servo_left_pwm_high_time, servo_right_pwm_high_time)

    PWM_HIGH_TIME_DIFF = 2

    if output_mode == "MANUAL":
        rospy.logwarn("Set the vehicle Automated Mode using the RF Control.")

    while output_mode == "MANUAL": continue

    rospy.loginfo("Correct Pairing and Automated Operation Mode.")

    actuators_cmd = ActuatorsState()
    actuators_cmd.motor_pwm_high_time = 0
    actuators_cmd.servo_pwm_high_time = 0

    actuators_state_pub.publish(actuators_cmd)

    start_time = time.time()
    while time.time() - start_time <= 3: continue


    curr_servo_pwm_high_time = servo_center_pwm_high_time

    actuators_cmd = ActuatorsState()
    actuators_cmd.motor_pwm_high_time = motor_pairing_pwm_high_time
    actuators_cmd.servo_pwm_high_time = curr_servo_pwm_high_time

    actuators_state_pub.publish(actuators_cmd)

    characterization_list = [(curr_servo_pwm_high_time, 90.0)]

    rospy.logwarn("Servo Characterization Process: Measure and register the vehicle steering angle and its corresponding PWM High Time.")

    for i in range(2):
        characterization_done = False
        if i == 0: rospy.logwarn("Characterization for first angle direction...")
        else: rospy.logwarn("Characterization for second angle direction...")

        while True:
            rospy.loginfo("Press enter to modify PWM High Time. Press 'S' to change calibration mode...")
            while True:
                value = raw_input()
                if value == "": break
                elif not(value == "s" or value == "S"): continue 
                characterization_done = True 
                break

            if curr_servo_pwm_high_time < servo_min_pwm_high_time or curr_servo_pwm_high_time > servo_max_pwm_high_time:
                curr_servo_pwm_high_time = servo_min_pwm_high_time if curr_servo_pwm_high_time < servo_min_pwm_high_time else servo_max_pwm_high_time

            elif characterization_done:
                curr_servo_pwm_high_time = servo_center_pwm_high_time

            else:
                curr_servo_pwm_high_time = curr_servo_pwm_high_time - PWM_HIGH_TIME_DIFF if i == 0 else curr_servo_pwm_high_time + PWM_HIGH_TIME_DIFF

            actuators_cmd = ActuatorsState()
            actuators_cmd.motor_pwm_high_time = motor_pairing_pwm_high_time
            actuators_cmd.servo_pwm_high_time = curr_servo_pwm_high_time

            actuators_state_pub.publish(actuators_cmd)

            if characterization_done: break

            rospy.loginfo("Servo PWM High Time (ms): " + str(curr_servo_pwm_high_time))
            rospy.loginfo("Introduce the corresponding angle in degrees:")
            while True:
                value = raw_input()
                try:
                    angle = float(value)
                    characterization_list.append((curr_servo_pwm_high_time, angle))
                    break
                except ValueError:
                    rospy.logerr("Error determining the introduced angle. Try again...")

            rospy.loginfo("PWM High Time (ms): %f, Angle (degrees): %f", characterization_list.pop()[0], characterization_list.pop()[1])

    actuators_cmd = ActuatorsState()
    actuators_cmd.motor_pwm_high_time = 0
    actuators_cmd.servo_pwm_high_time = 0

    actuators_state_pub.publish(actuators_cmd)

    characterization_list.sort(key = lambda x : x[1])

    x = []
    y = []

    for tuple in characterization_list:
        x.append(tuple[0])
        y.append(np.radians(tuple[1] - 90))

    model = np.polyfit(x, y, 1)
    slope = model[0]
    intercept = model[1]

    def linealRegFunc(x):
        return slope * x + intercept

    regModel = list(map(linealRegFunc, x))

    plt.grid()
    plt.scatter(x, y)
    plt.plot(x, regModel, c = 'r')
    plt.text(min(x), max(y), "y = " + str(slope) + " * x  " + ("- " if intercept < 0 else "+ " ) + str(-1 * intercept if intercept < 0 else intercept))
    plt.xlabel("PWM High Time (us)")
    plt.ylabel("Steering Angle (rads)")
    plt.show()     

    return slope, intercept

def actuatorsStateCallback(data):
    global servo_pwm_high_time
    global motor_pwm_high_time
    global output_mode
    
    servo_pwm_high_time = data.servo_pwm_high_time
    motor_pwm_high_time = data.motor_pwm_high_time
    output_mode = data.output_mode

def speedCallback(msg):
    global speed
    
    speed = msg.data

if __name__ == '__main__':
    servo_pwm_high_time = None
    motor_pwm_high_time = None
    output_mode = None

    speed = None

    rospy.init_node("actuators_calibrator")

    if "/arduino_1" not in rosnode.get_node_names() or "/arduino_2" not in rosnode.get_node_names():
        rospy.logerr("ERROR: arduino_driver.launch not running, cannot communicate with driver board!")
        exit()

    sensors_op_mode_success = False
    actuators_op_mode_success = False
    for topic_data in rostopic.get_topic_list()[1]:
        if '/actuators_cmd' == topic_data[0]:
            actuators_op_mode_success = True
        if '/speed' == topic_data[0]:
            sensors_op_mode_success = True
        if actuators_op_mode_success and sensors_op_mode_success:
            break
    
    if not sensors_op_mode_success:
        rospy.logerr("ERROR: Incorrect Arduino Sensors Operation Mode, set Operation Mode to PWM_CONTROL = true")

    if not actuators_op_mode_success:
        rospy.logerr("ERROR: Incorrect Arduino Actuators Operation Mode, set Operation Mode to PWM_CONTROL = true")

    if not actuators_op_mode_success or not sensors_op_mode_success:
        exit()

    rospy.Subscriber("/actuators_state", ActuatorsState, actuatorsStateCallback)
    rospy.Subscriber("/speed", Float32, speedCallback)

    actuators_state_pub = rospy.Publisher("/actuators_cmd", ActuatorsState, queue_size=10)

    while output_mode is None: continue

    rospy.loginfo("Starting calibration process.")
    validateInitialConditions()

    motor_pairing_pwm_high_time, servo_center_pwm_high_time = getActuatorsMeanState()
    rospy.loginfo("Success Calibrating Motors.")
    rospy.loginfo("BLDC Motor Pairing PWM High Time (ms): " + str(motor_pairing_pwm_high_time))
    rospy.loginfo("Servo Centered PWM High Time (ms): " + str(servo_center_pwm_high_time))

    rospy.logwarn("WARNING: Set and hold the vehicle steering to the maximum right angle until the calibration is finished.")
    rospy.logwarn("WARNING: Do not release the RF Controller stick!")

    _, servo_right_pwm_high_time = getActuatorsMeanState()
    rospy.loginfo("Success Calibrating Motors.")
    rospy.loginfo("Servo Max Right PWM High Time (ms): " + str(servo_right_pwm_high_time))

    rospy.logwarn("WARNING: Set and hold the vehicle steering to the maximum left angle until the calibration is finished.")
    rospy.logwarn("WARNING: Do not release the RF Controller stick!")

    _, servo_left_pwm_high_time = getActuatorsMeanState()
    rospy.loginfo("Success Calibrating Motors.")
    rospy.loginfo("Servo Max Left PWM High Time (ms): " + str(servo_left_pwm_high_time))

    rospy.logwarn("WARNING: Place the vehicle on the floor and assure that it has enough space to move freely.")

    motor_dead_zone_pwm_high_time = getMotorDeadZoneState()
    rospy.loginfo("Success Calibrating Motors.")
    rospy.loginfo("BLDC Motor Dead Zone PWM High Time (ms): " + str(motor_dead_zone_pwm_high_time))

    rospy.logwarn("WARNING: Get close to the vehicle for the servo characterization. You must be ready to measure the wheels steering angle!")
    rospy.logwarn("WARNING: Note that the measured angles must be provided in degrees, where max left steer -> 180, centered steer -> 90, max right steer -> 0")
    servo_characterization_slope_constant, servo_characterization_intercept_constant = getServoCharacterization()
    rospy.loginfo("Success Servo Motor Characterization.")
    rospy.loginfo("Servo Characterization Slope Constant: " + str(servo_characterization_slope_constant))
    rospy.loginfo("Servo Characterization Intercept Constant: " + str(servo_characterization_intercept_constant))
    rospy.loginfo("Steering Angle = Servo Characterization Slope Constant * PWM High Time + Servo Characterization Intercept Constant")

    rospy.logwarn("Write down the next constanst and substitute them in their declaration on the Arduino codes: 'trucky_arduino/firmware/arduino_actuators.cpp' and 'trucky_arduino/firmware/arduino_sensors.cpp'")
    rospy.logwarn("------------------------------------------")
    rospy.loginfo("MOTOR_PAIRING_PWM_HIGH_TIME  -> " + str(motor_pairing_pwm_high_time))
    rospy.loginfo("MOTOR_MIN_MOVE_PWM_HIGH_TIME -> " + str(motor_dead_zone_pwm_high_time))
    rospy.loginfo("MOTOR_MAX_PWM_HIGH_TIME      -> " + str(motor_dead_zone_pwm_high_time + 200))
    rospy.loginfo()
    rospy.loginfo("SERVO_MIN_PWM_HIGH_TIME    -> " + str(min([servo_right_pwm_high_time, servo_left_pwm_high_time, servo_center_pwm_high_time])))
    rospy.loginfo("SERVO_CENTER_PWM_HIGH_TIME -> " + str(servo_center_pwm_high_time))
    rospy.loginfo("SERVO_MAX_PWM_HIGH_TIME    -> " + str(max([servo_right_pwm_high_time, servo_left_pwm_high_time, servo_center_pwm_high_time])))
    rospy.loginfo()
    rospy.loginfo("SERVO_CHARACTERIZATION_SLOPE_CONSTANT     -> " + str(servo_characterization_slope_constant))
    rospy.loginfo("SERVO_CHARACTERIZATION_INTERCEPT_CONSTANT -> " + str(servo_characterization_intercept_constant))
    rospy.logwarn("------------------------------------------")
