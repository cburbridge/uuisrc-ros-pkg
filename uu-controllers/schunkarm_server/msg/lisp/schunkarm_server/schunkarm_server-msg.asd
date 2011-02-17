
(in-package :asdf)

(defsystem "schunkarm_server-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
 :geometry_msgs-msg
 :roslib-msg
)
  :components ((:file "_package")
    (:file "movearmFeedback" :depends-on ("_package"))
    (:file "_package_movearmFeedback" :depends-on ("_package"))
    (:file "movearmAction" :depends-on ("_package"))
    (:file "_package_movearmAction" :depends-on ("_package"))
    (:file "movearmActionResult" :depends-on ("_package"))
    (:file "_package_movearmActionResult" :depends-on ("_package"))
    (:file "movearmActionGoal" :depends-on ("_package"))
    (:file "_package_movearmActionGoal" :depends-on ("_package"))
    (:file "movearmResult" :depends-on ("_package"))
    (:file "_package_movearmResult" :depends-on ("_package"))
    (:file "movearmGoal" :depends-on ("_package"))
    (:file "_package_movearmGoal" :depends-on ("_package"))
    (:file "movearmActionFeedback" :depends-on ("_package"))
    (:file "_package_movearmActionFeedback" :depends-on ("_package"))
    ))
