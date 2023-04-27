// Auto-generated. Do not edit!

// (in-package custom_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ActuatorsState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.servo_pwm_high_time = null;
      this.motor_pwm_high_time = null;
      this.output_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('servo_pwm_high_time')) {
        this.servo_pwm_high_time = initObj.servo_pwm_high_time
      }
      else {
        this.servo_pwm_high_time = 0;
      }
      if (initObj.hasOwnProperty('motor_pwm_high_time')) {
        this.motor_pwm_high_time = initObj.motor_pwm_high_time
      }
      else {
        this.motor_pwm_high_time = 0;
      }
      if (initObj.hasOwnProperty('output_mode')) {
        this.output_mode = initObj.output_mode
      }
      else {
        this.output_mode = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActuatorsState
    // Serialize message field [servo_pwm_high_time]
    bufferOffset = _serializer.int64(obj.servo_pwm_high_time, buffer, bufferOffset);
    // Serialize message field [motor_pwm_high_time]
    bufferOffset = _serializer.int64(obj.motor_pwm_high_time, buffer, bufferOffset);
    // Serialize message field [output_mode]
    bufferOffset = _serializer.string(obj.output_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActuatorsState
    let len;
    let data = new ActuatorsState(null);
    // Deserialize message field [servo_pwm_high_time]
    data.servo_pwm_high_time = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [motor_pwm_high_time]
    data.motor_pwm_high_time = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [output_mode]
    data.output_mode = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.output_mode.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msgs/ActuatorsState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b99dd1ce38e6a3931dbead99360d717';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 servo_pwm_high_time 
    int64 motor_pwm_high_time
    string output_mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActuatorsState(null);
    if (msg.servo_pwm_high_time !== undefined) {
      resolved.servo_pwm_high_time = msg.servo_pwm_high_time;
    }
    else {
      resolved.servo_pwm_high_time = 0
    }

    if (msg.motor_pwm_high_time !== undefined) {
      resolved.motor_pwm_high_time = msg.motor_pwm_high_time;
    }
    else {
      resolved.motor_pwm_high_time = 0
    }

    if (msg.output_mode !== undefined) {
      resolved.output_mode = msg.output_mode;
    }
    else {
      resolved.output_mode = ''
    }

    return resolved;
    }
};

module.exports = ActuatorsState;
