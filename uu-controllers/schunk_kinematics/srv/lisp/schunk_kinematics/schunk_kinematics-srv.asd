
(in-package :asdf)

(defsystem "schunk_kinematics-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg :motion_planning_msgs-msg :roslib-msg :schunk_kinematics-msg :sensor_msgs-msg)
  :components ((:file "_package")
    (:file "GetVelocityIK" :depends-on ("_package"))
    (:file "_package_GetVelocityIK" :depends-on ("_package"))
    ))
