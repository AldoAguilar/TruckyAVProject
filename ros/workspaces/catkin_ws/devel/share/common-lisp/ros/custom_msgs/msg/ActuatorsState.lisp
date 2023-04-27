; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude ActuatorsState.msg.html

(cl:defclass <ActuatorsState> (roslisp-msg-protocol:ros-message)
  ((servo_pwm_high_time
    :reader servo_pwm_high_time
    :initarg :servo_pwm_high_time
    :type cl:integer
    :initform 0)
   (motor_pwm_high_time
    :reader motor_pwm_high_time
    :initarg :motor_pwm_high_time
    :type cl:integer
    :initform 0)
   (output_mode
    :reader output_mode
    :initarg :output_mode
    :type cl:string
    :initform ""))
)

(cl:defclass ActuatorsState (<ActuatorsState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActuatorsState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActuatorsState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<ActuatorsState> is deprecated: use custom_msgs-msg:ActuatorsState instead.")))

(cl:ensure-generic-function 'servo_pwm_high_time-val :lambda-list '(m))
(cl:defmethod servo_pwm_high_time-val ((m <ActuatorsState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:servo_pwm_high_time-val is deprecated.  Use custom_msgs-msg:servo_pwm_high_time instead.")
  (servo_pwm_high_time m))

(cl:ensure-generic-function 'motor_pwm_high_time-val :lambda-list '(m))
(cl:defmethod motor_pwm_high_time-val ((m <ActuatorsState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:motor_pwm_high_time-val is deprecated.  Use custom_msgs-msg:motor_pwm_high_time instead.")
  (motor_pwm_high_time m))

(cl:ensure-generic-function 'output_mode-val :lambda-list '(m))
(cl:defmethod output_mode-val ((m <ActuatorsState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:output_mode-val is deprecated.  Use custom_msgs-msg:output_mode instead.")
  (output_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActuatorsState>) ostream)
  "Serializes a message object of type '<ActuatorsState>"
  (cl:let* ((signed (cl:slot-value msg 'servo_pwm_high_time)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor_pwm_high_time)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'output_mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'output_mode))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActuatorsState>) istream)
  "Deserializes a message object of type '<ActuatorsState>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo_pwm_high_time) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor_pwm_high_time) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'output_mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'output_mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActuatorsState>)))
  "Returns string type for a message object of type '<ActuatorsState>"
  "custom_msgs/ActuatorsState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorsState)))
  "Returns string type for a message object of type 'ActuatorsState"
  "custom_msgs/ActuatorsState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActuatorsState>)))
  "Returns md5sum for a message object of type '<ActuatorsState>"
  "4b99dd1ce38e6a3931dbead99360d717")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActuatorsState)))
  "Returns md5sum for a message object of type 'ActuatorsState"
  "4b99dd1ce38e6a3931dbead99360d717")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActuatorsState>)))
  "Returns full string definition for message of type '<ActuatorsState>"
  (cl:format cl:nil "int64 servo_pwm_high_time ~%int64 motor_pwm_high_time~%string output_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActuatorsState)))
  "Returns full string definition for message of type 'ActuatorsState"
  (cl:format cl:nil "int64 servo_pwm_high_time ~%int64 motor_pwm_high_time~%string output_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActuatorsState>))
  (cl:+ 0
     8
     8
     4 (cl:length (cl:slot-value msg 'output_mode))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActuatorsState>))
  "Converts a ROS message object to a list"
  (cl:list 'ActuatorsState
    (cl:cons ':servo_pwm_high_time (servo_pwm_high_time msg))
    (cl:cons ':motor_pwm_high_time (motor_pwm_high_time msg))
    (cl:cons ':output_mode (output_mode msg))
))
