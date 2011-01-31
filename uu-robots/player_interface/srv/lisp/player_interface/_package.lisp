(defpackage player_interface-srv
  (:use cl
        roslisp-msg-protocol)
  (:export
   "GOTOENABLE"
   "<GOTOENABLE-REQUEST>"
   "<GOTOENABLE-RESPONSE>"
   "GOTO"
   "<GOTO-REQUEST>"
   "<GOTO-RESPONSE>"
  ))

