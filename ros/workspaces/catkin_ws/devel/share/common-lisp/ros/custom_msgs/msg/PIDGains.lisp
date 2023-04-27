; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude PIDGains.msg.html

(cl:defclass <PIDGains> (roslisp-msg-protocol:ros-message)
  ((kp
    :reader kp
    :initarg :kp
    :type cl:float
    :initform 0.0)
   (kd
    :reader kd
    :initarg :kd
    :type cl:float
    :initform 0.0)
   (ki
    :reader ki
    :initarg :ki
    :type cl:float
    :initform 0.0))
)

(cl:defclass PIDGains (<PIDGains>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PIDGains>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PIDGains)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<PIDGains> is deprecated: use custom_msgs-msg:PIDGains instead.")))

(cl:ensure-generic-function 'kp-val :lambda-list '(m))
(cl:defmethod kp-val ((m <PIDGains>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:kp-val is deprecated.  Use custom_msgs-msg:kp instead.")
  (kp m))

(cl:ensure-generic-function 'kd-val :lambda-list '(m))
(cl:defmethod kd-val ((m <PIDGains>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:kd-val is deprecated.  Use custom_msgs-msg:kd instead.")
  (kd m))

(cl:ensure-generic-function 'ki-val :lambda-list '(m))
(cl:defmethod ki-val ((m <PIDGains>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:ki-val is deprecated.  Use custom_msgs-msg:ki instead.")
  (ki m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PIDGains>) ostream)
  "Serializes a message object of type '<PIDGains>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'kp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'kd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ki))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PIDGains>) istream)
  "Deserializes a message object of type '<PIDGains>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ki) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PIDGains>)))
  "Returns string type for a message object of type '<PIDGains>"
  "custom_msgs/PIDGains")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PIDGains)))
  "Returns string type for a message object of type 'PIDGains"
  "custom_msgs/PIDGains")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PIDGains>)))
  "Returns md5sum for a message object of type '<PIDGains>"
  "8dfae169c05c8647d6140e2a6c279a75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PIDGains)))
  "Returns md5sum for a message object of type 'PIDGains"
  "8dfae169c05c8647d6140e2a6c279a75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PIDGains>)))
  "Returns full string definition for message of type '<PIDGains>"
  (cl:format cl:nil "float32 kp ~%float32 kd~%float32 ki~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PIDGains)))
  "Returns full string definition for message of type 'PIDGains"
  (cl:format cl:nil "float32 kp ~%float32 kd~%float32 ki~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PIDGains>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PIDGains>))
  "Converts a ROS message object to a list"
  (cl:list 'PIDGains
    (cl:cons ':kp (kp msg))
    (cl:cons ':kd (kd msg))
    (cl:cons ':ki (ki msg))
))
