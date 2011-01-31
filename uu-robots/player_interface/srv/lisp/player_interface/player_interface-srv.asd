
(in-package :asdf)

(defsystem "player_interface-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "GoToEnable" :depends-on ("_package"))
    (:file "_package_GoToEnable" :depends-on ("_package"))
    (:file "GoTo" :depends-on ("_package"))
    (:file "_package_GoTo" :depends-on ("_package"))
    ))
