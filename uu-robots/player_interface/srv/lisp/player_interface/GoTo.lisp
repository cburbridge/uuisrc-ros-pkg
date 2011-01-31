; Auto-generated. Do not edit!


(in-package player_interface-srv)


;//! \htmlinclude GoTo-request.msg.html

(defclass <GoTo-request> (ros-message)
  ((x
    :reader x-val
    :initarg :x
    :type float
    :initform 0.0)
   (y
    :reader y-val
    :initarg :y
    :type float
    :initform 0.0)
   (th
    :reader th-val
    :initarg :th
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <GoTo-request>) ostream)
  "Serializes a message object of type '<GoTo-request>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'x))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'y))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'th))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <GoTo-request>) istream)
  "Deserializes a message object of type '<GoTo-request>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'th) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<GoTo-request>)))
  "Returns string type for a service object of type '<GoTo-request>"
  "player_interface/GoToRequest")
(defmethod md5sum ((type (eql '<GoTo-request>)))
  "Returns md5sum for a message object of type '<GoTo-request>"
  "000e435776f4fd6ba555d25d7a61ed8f")
(defmethod message-definition ((type (eql '<GoTo-request>)))
  "Returns full string definition for message of type '<GoTo-request>"
  (format nil "float64 x~%float64 y~%float64 th~%~%"))
(defmethod serialization-length ((msg <GoTo-request>))
  (+ 0
     8
     8
     8
))
(defmethod ros-message-to-list ((msg <GoTo-request>))
  "Converts a ROS message object to a list"
  (list '<GoTo-request>
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
    (cons ':th (th-val msg))
))
;//! \htmlinclude GoTo-response.msg.html

(defclass <GoTo-response> (ros-message)
  ()
)
(defmethod serialize ((msg <GoTo-response>) ostream)
  "Serializes a message object of type '<GoTo-response>"
)
(defmethod deserialize ((msg <GoTo-response>) istream)
  "Deserializes a message object of type '<GoTo-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<GoTo-response>)))
  "Returns string type for a service object of type '<GoTo-response>"
  "player_interface/GoToResponse")
(defmethod md5sum ((type (eql '<GoTo-response>)))
  "Returns md5sum for a message object of type '<GoTo-response>"
  "000e435776f4fd6ba555d25d7a61ed8f")
(defmethod message-definition ((type (eql '<GoTo-response>)))
  "Returns full string definition for message of type '<GoTo-response>"
  (format nil "~%~%~%"))
(defmethod serialization-length ((msg <GoTo-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <GoTo-response>))
  "Converts a ROS message object to a list"
  (list '<GoTo-response>
))
(defmethod service-request-type ((msg (eql 'GoTo)))
  '<GoTo-request>)
(defmethod service-response-type ((msg (eql 'GoTo)))
  '<GoTo-response>)
(defmethod ros-datatype ((msg (eql 'GoTo)))
  "Returns string type for a service object of type '<GoTo>"
  "player_interface/GoTo")
