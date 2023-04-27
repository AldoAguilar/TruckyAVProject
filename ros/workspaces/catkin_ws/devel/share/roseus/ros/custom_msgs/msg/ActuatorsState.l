;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::ActuatorsState)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'ActuatorsState (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::ACTUATORSSTATE")
  (make-package "CUSTOM_MSGS::ACTUATORSSTATE"))

(in-package "ROS")
;;//! \htmlinclude ActuatorsState.msg.html


(defclass custom_msgs::ActuatorsState
  :super ros::object
  :slots (_servo_pwm_high_time _motor_pwm_high_time _output_mode ))

(defmethod custom_msgs::ActuatorsState
  (:init
   (&key
    ((:servo_pwm_high_time __servo_pwm_high_time) 0)
    ((:motor_pwm_high_time __motor_pwm_high_time) 0)
    ((:output_mode __output_mode) "")
    )
   (send-super :init)
   (setq _servo_pwm_high_time (round __servo_pwm_high_time))
   (setq _motor_pwm_high_time (round __motor_pwm_high_time))
   (setq _output_mode (string __output_mode))
   self)
  (:servo_pwm_high_time
   (&optional __servo_pwm_high_time)
   (if __servo_pwm_high_time (setq _servo_pwm_high_time __servo_pwm_high_time)) _servo_pwm_high_time)
  (:motor_pwm_high_time
   (&optional __motor_pwm_high_time)
   (if __motor_pwm_high_time (setq _motor_pwm_high_time __motor_pwm_high_time)) _motor_pwm_high_time)
  (:output_mode
   (&optional __output_mode)
   (if __output_mode (setq _output_mode __output_mode)) _output_mode)
  (:serialization-length
   ()
   (+
    ;; int64 _servo_pwm_high_time
    8
    ;; int64 _motor_pwm_high_time
    8
    ;; string _output_mode
    4 (length _output_mode)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _servo_pwm_high_time
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _servo_pwm_high_time (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _servo_pwm_high_time) (= (length (_servo_pwm_high_time . bv)) 2)) ;; bignum
              (write-long (ash (elt (_servo_pwm_high_time . bv) 0) 0) s)
              (write-long (ash (elt (_servo_pwm_high_time . bv) 1) -1) s))
             ((and (class _servo_pwm_high_time) (= (length (_servo_pwm_high_time . bv)) 1)) ;; big1
              (write-long (elt (_servo_pwm_high_time . bv) 0) s)
              (write-long (if (>= _servo_pwm_high_time 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _servo_pwm_high_time s)(write-long (if (>= _servo_pwm_high_time 0) 0 #xffffffff) s)))
     ;; int64 _motor_pwm_high_time
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _motor_pwm_high_time (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _motor_pwm_high_time) (= (length (_motor_pwm_high_time . bv)) 2)) ;; bignum
              (write-long (ash (elt (_motor_pwm_high_time . bv) 0) 0) s)
              (write-long (ash (elt (_motor_pwm_high_time . bv) 1) -1) s))
             ((and (class _motor_pwm_high_time) (= (length (_motor_pwm_high_time . bv)) 1)) ;; big1
              (write-long (elt (_motor_pwm_high_time . bv) 0) s)
              (write-long (if (>= _motor_pwm_high_time 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _motor_pwm_high_time s)(write-long (if (>= _motor_pwm_high_time 0) 0 #xffffffff) s)))
     ;; string _output_mode
       (write-long (length _output_mode) s) (princ _output_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _servo_pwm_high_time
#+(or :alpha :irix6 :x86_64)
      (setf _servo_pwm_high_time (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _servo_pwm_high_time (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _motor_pwm_high_time
#+(or :alpha :irix6 :x86_64)
      (setf _motor_pwm_high_time (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _motor_pwm_high_time (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _output_mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _output_mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get custom_msgs::ActuatorsState :md5sum-) "4b99dd1ce38e6a3931dbead99360d717")
(setf (get custom_msgs::ActuatorsState :datatype-) "custom_msgs/ActuatorsState")
(setf (get custom_msgs::ActuatorsState :definition-)
      "int64 servo_pwm_high_time 
int64 motor_pwm_high_time
string output_mode
")



(provide :custom_msgs/ActuatorsState "4b99dd1ce38e6a3931dbead99360d717")


