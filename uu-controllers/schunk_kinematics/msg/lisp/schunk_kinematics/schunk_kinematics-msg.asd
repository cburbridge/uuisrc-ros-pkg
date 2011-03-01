
(in-package :asdf)

(defsystem "schunk_kinematics-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
 :motion_planning_msgs-msg
 :roslib-msg
 :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "VelocityIKRequest" :depends-on ("_package"))
    (:file "_package_VelocityIKRequest" :depends-on ("_package"))
    ))
