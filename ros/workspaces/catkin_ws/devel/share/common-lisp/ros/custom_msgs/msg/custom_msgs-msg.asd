
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ActuatorsState" :depends-on ("_package_ActuatorsState"))
    (:file "_package_ActuatorsState" :depends-on ("_package"))
    (:file "PIDGains" :depends-on ("_package_PIDGains"))
    (:file "_package_PIDGains" :depends-on ("_package"))
  ))