#!/usr/bin/env python

import time
import rospy
import rosnode
import rostopic

from trucky_custom_msgs.msg import ActuatorsState
from ackermann_msgs.msg import AckermannDrive

op_mode = "None"

def actuatorsCmdCb(data):
    global op_mode
    motor_pwm = data.motor_pwm_high_time
    servo_pwm = data.servo_pwm_high_time
    rospy.loginfo("New Actuators State Command:")
    rospy.loginfo("\t - Motor PWM High Time: " + str(motor_pwm))
    rospy.loginfo("\t - Servo PWM High Time: " + str(servo_pwm))
    if op_mode != "Actuators":
        rospy.logwarn("The robot will now set its actuators based on the PWM High Time. If you want to control the robot in terms of speed and steering angle use the '/ackermann_cmd' topic now.")
    op_mode = "Actuators"

def ackermannCmdCb(data):
    global op_mode
    speed = data.speed
    steering = data.steering_angle
    if op_mode != "Ackermann":
        rospy.loginfo("New Ackermann Command:")
        rospy.loginfo("\t - Speed: " + str(speed))
        rospy.loginfo("\t - Steering Angle: " + str(steering))
        rospy.logwarn("The robot will now set its actuators based on speed and steering values. If you want to control the robot in terms of the actuators PWM High Times use the '/arduino_actuators/actuators_cmd' topic now.")
    op_mode = "Ackermann"

if __name__ == "__main__":
    rospy.init_node('arduino_logger', anonymous=True)

    failed = False

    while "/arduino_1" not in rosnode.get_node_names(): continue    
    while "/arduino_2" not in rosnode.get_node_names(): continue
    
    pwm_control = False
    t0 = time.time()
    while True:
        connected = False
        for topic_data in rostopic.get_topic_list()[1]:
            if '/ackermann_cmd' == topic_data[0] or '/actuators_cmd' == topic_data[0]: 
                rospy.logwarn("Arduino Actuators -> Connection established successfuly!")
                connected = True
                if '/actuators_cmd' == topic_data[0]: pwm_control = True
                break
        if connected: break

        if time.time() - t0 > 10:
            rospy.logerr("ERROR: Failed to communicate with Arduino Actuators:")
            rospy.logerr("\t - Check if the Arduino Actuators board is connected.")
            rospy.logerr("\t - Check if the Arduino Actuators board's LED is constantly flashing. This means that the wrong code has been loaded to the board.")
            rospy.logerr("\t - Review the 'arduino/config/config.yaml' file. If necessary change the serial port assigned to the board.")
            failed = True
            break

    t0 = time.time()
    while True:
        connected = False
        for topic_data in rostopic.get_topic_list()[0]:
            if '/actuators_state' == topic_data[0] or '/steering_angle' == topic_data[0]: 
                rospy.logwarn("Arduino Sensors   -> Connection established successfuly!")
                connected = True
                break
        if connected: break

        if time.time() - t0 > 10:
            rospy.logerr("ERROR: Failed to communicate with Arduino Sensors:")
            rospy.logerr("\t - Check if the Arduino Sensors board is connected.")
            rospy.logerr("\t - Check if the Arduino Sensors board's LED is constantly flashing. This means that the wrong code has been loaded to the board.")
            rospy.logerr("\t - Review the 'arduino/config/config.yaml' file. If necessary change the serial port assigned to the board.")
            failed = True
            break
    
    if not failed:
        if pwm_control: rospy.Subscriber("/actuators_cmd", ActuatorsState, actuatorsCmdCb)
        else: rospy.Subscriber("/ackermann_cmd", AckermannDrive, ackermannCmdCb)

    while not rospy.is_shutdown():
        rospy.spin()


    