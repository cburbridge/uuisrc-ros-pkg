; Auto-generated. Do not edit!


(in-package player_interface-srv)


;//! \htmlinclude GoToEnable-request.msg.html

(defclass <GoToEnable-request> (ros-message)
  ((enable
    :reader enable-val
    :initarg :enable
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <GoToEnable-request>) ostream)
  "Serializes a message object of type '<GoToEnable-request>"
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'enable) 1 0)) ostream)
)
(defmethod deserialize ((msg <GoToEnable-request>) istream)
  "Deserializes a message object of type '<GoToEnable-request>"
  (setf (slot-value msg 'enable) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<GoToEnable-request>)))
  "Returns string type for a service object of type '<GoToEnable-request>"
  "player_interface/GoToEnableRequest")
(defmethod md5sum ((type (eql '<GoToEnable-request>)))
  "Returns md5sum for a message object of type '<GoToEnable-request>"
  "8c1211af706069c994c06e00eb59ac9e")
(defmethod message-definition ((type (eql '<GoToEnable-request>)))
  "Returns full string definition for message of type '<GoToEnable-request>"
  (format nil "bool enable~%~%"))
(defmethod serialization-length ((msg <GoToEnable-request>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <GoToEnable-request>))
  "Converts a ROS message object to a list"
  (list '<GoToEnable-request>
    (cons ':enable (enable-val msg))
))
;//! \htmlinclude GoToEnable-response.msg.html

(defclass <GoToEnable-response> (ros-message)
  ()
)
(defmethod serialize ((msg <GoToEnable-response>) ostream)
  "Serializes a message object of type '<GoToEnable-response>"
)
(defmethod deserialize ((msg <GoToEnable-response>) istream)
  "Deserializes a message object of type '<GoToEnable-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<GoToEnable-response>)))
  "Returns string type for a service object of type '<GoToEnable-response>"
  "player_interface/GoToEnableResponse")
(defmethod md5sum ((type (eql '<GoToEnable-response>)))
  "Returns md5sum for a message object of type '<GoToEnable-response>"
  "8c1211af706069c994c06e00eb59ac9e")
(defmethod message-definition ((type (eql '<GoToEnable-response>)))
  "Returns full string definition for message of type '<GoToEnable-response>"
  (format nil "~%~%~%"))
(defmethod serialization-length ((msg <GoToEnable-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <GoToEnable-response>))
  "Converts a ROS message object to a list"
  (list '<GoToEnable-response>
))
(defmethod service-request-type ((msg (eql 'GoToEnable)))
  '<GoToEnable-request>)
(defmethod service-response-type ((msg (eql 'GoToEnable)))
  '<GoToEnable-response>)
(defmethod ros-datatype ((msg (eql 'GoToEnable)))
  "Returns string type for a service object of type '<GoToEnable>"
  "player_interface/GoToEnable")
