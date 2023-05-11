import os
import rospy
import subprocess

def validateUSBDevices():
    rospy.logwarn("WARNING: Assure that the only USB-serial device connected to the computer is the Arduino board you want to upload code to.")
    rospy.loginfo("Press enter to continue...")
    while raw_input() != "": continue
    ls_process = subprocess.Popen(['ls', '/dev'], stdout=subprocess.PIPE)
    grep_process = subprocess.Popen(['grep', 'USB'], stdin = ls_process.stdout, stdout=subprocess.PIPE, stderr = subprocess.PIPE)
    stdout, _ = grep_process.communicate()
    devices = stdout.split('\n') if '\n' in stdout else ['']
    if len(devices) > 1: devices.pop()
    if len(devices) == 1 and 'USB' in devices[0]: return True
    rospy.logerr("ERROR: Multiple ttyUSB devices detected" + str(devices))
    rospy.logerr("\t - Check if multiple USB-serial devices are connected (Maybe the two Arduino Boards are connected? Connect only the Arduino board you want to upload code to).")
    return False
        

if __name__ == '__main__':
    code_type = rospy.get_param("code_type")
    rospy.init_node('upload_arduino_code', anonymous = True)
    rospy.loginfo("ARDUINO SENSORS UPLOAD" if code_type == "sensors" else "ARDUINO ACTUATORS UPLOAD")
    while not validateUSBDevices(): continue
    rospy.loginfo("Arduino Board detected succesfully!")
    if code_type == "actuators":
        os.system('rosrun trucky_arduino upload_arduino_actuators_code.sh')
    else:
        os.system('rosrun trucky_arduino upload_arduino_sensors_code.sh') 