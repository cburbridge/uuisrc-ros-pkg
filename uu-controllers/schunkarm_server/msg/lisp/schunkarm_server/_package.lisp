(defpackage schunkarm_server-msg
  (:use cl
        roslisp-msg-protocol)
  (:export
   "<MOVEARMFEEDBACK>"
   "<MOVEARMACTION>"
   "<MOVEARMACTIONRESULT>"
   "<MOVEARMACTIONGOAL>"
   "<MOVEARMRESULT>"
   "<MOVEARMGOAL>"
   "<MOVEARMACTIONFEEDBACK>"
  ))

