;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::PIDGains)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'PIDGains (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::PIDGAINS")
  (make-package "CUSTOM_MSGS::PIDGAINS"))

(in-package "ROS")
;;//! \htmlinclude PIDGains.msg.html


(defclass custom_msgs::PIDGains
  :super ros::object
  :slots (_kp _kd _ki ))

(defmethod custom_msgs::PIDGains
  (:init
   (&key
    ((:kp __kp) 0.0)
    ((:kd __kd) 0.0)
    ((:ki __ki) 0.0)
    )
   (send-super :init)
   (setq _kp (float __kp))
   (setq _kd (float __kd))
   (setq _ki (float __ki))
   self)
  (:kp
   (&optional __kp)
   (if __kp (setq _kp __kp)) _kp)
  (:kd
   (&optional __kd)
   (if __kd (setq _kd __kd)) _kd)
  (:ki
   (&optional __ki)
   (if __ki (setq _ki __ki)) _ki)
  (:serialization-length
   ()
   (+
    ;; float32 _kp
    4
    ;; float32 _kd
    4
    ;; float32 _ki
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _kp
       (sys::poke _kp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _kd
       (sys::poke _kd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ki
       (sys::poke _ki (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _kp
     (setq _kp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _kd
     (setq _kd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ki
     (setq _ki (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get custom_msgs::PIDGains :md5sum-) "8dfae169c05c8647d6140e2a6c279a75")
(setf (get custom_msgs::PIDGains :datatype-) "custom_msgs/PIDGains")
(setf (get custom_msgs::PIDGains :definition-)
      "float32 kp 
float32 kd
float32 ki
")



(provide :custom_msgs/PIDGains "8dfae169c05c8647d6140e2a6c279a75")


