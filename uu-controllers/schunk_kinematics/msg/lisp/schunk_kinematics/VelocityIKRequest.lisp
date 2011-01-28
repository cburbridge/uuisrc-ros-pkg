; Auto-generated. Do not edit!


(in-package schunk_kinematics-msg)


;//! \htmlinclude VelocityIKRequest.msg.html

(defclass <VelocityIKRequest> (ros-message)
  ((ik_link_name
    :reader ik_link_name-val
    :initarg :ik_link_name
    :type string
    :initform "")
   (twist
    :reader twist-val
    :initarg :twist
    :type geometry_msgs-msg:<Twist>
    :initform (make-instance 'geometry_msgs-msg:<Twist>))
   (robot_state
    :reader robot_state-val
    :initarg :robot_state
    :type motion_planning_msgs-msg:<RobotState>
    :initform (make-instance 'motion_planning_msgs-msg:<RobotState>)))
)
(defmethod serialize ((msg <VelocityIKRequest>) ostream)
  "Serializes a message object of type '<VelocityIKRequest>"
  (let ((__ros_str_len (length (slot-value msg 'ik_link_name))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'ik_link_name))
  (serialize (slot-value msg 'twist) ostream)
  (serialize (slot-value msg 'robot_state) ostream)
)
(defmethod deserialize ((msg <VelocityIKRequest>) istream)
  "Deserializes a message object of type '<VelocityIKRequest>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'ik_link_name) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'ik_link_name) __ros_str_idx) (code-char (read-byte istream)))))
  (deserialize (slot-value msg 'twist) istream)
  (deserialize (slot-value msg 'robot_state) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<VelocityIKRequest>)))
  "Returns string type for a message object of type '<VelocityIKRequest>"
  "schunk_kinematics/VelocityIKRequest")
(defmethod md5sum ((type (eql '<VelocityIKRequest>)))
  "Returns md5sum for a message object of type '<VelocityIKRequest>"
  "facf8a45f952b874757dc2ae035fa257")
(defmethod message-definition ((type (eql '<VelocityIKRequest>)))
  "Returns full string definition for message of type '<VelocityIKRequest>"
  (format nil "# A Velocity IK request message~%# The name of the link for which we are computing IK~%string ik_link_name~%~%# The twist of the link~%geometry_msgs/Twist twist~%~%# Here specify the starting positions ~%# of joints/links on the robot.~%motion_planning_msgs/RobotState robot_state~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: motion_planning_msgs/RobotState~%# This message contains information about the robot state, i.e. the positions of its joints and links~%sensor_msgs/JointState joint_state~%motion_planning_msgs/MultiDOFJointState multi_dof_joint_state~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: motion_planning_msgs/MultiDOFJointState~%#A representation of a multi-dof joint state~%time stamp~%string joint_name~%string frame_id~%string child_frame_id~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(defmethod serialization-length ((msg <VelocityIKRequest>))
  (+ 0
     4 (length (slot-value msg 'ik_link_name))
     (serialization-length (slot-value msg 'twist))
     (serialization-length (slot-value msg 'robot_state))
))
(defmethod ros-message-to-list ((msg <VelocityIKRequest>))
  "Converts a ROS message object to a list"
  (list '<VelocityIKRequest>
    (cons ':ik_link_name (ik_link_name-val msg))
    (cons ':twist (twist-val msg))
    (cons ':robot_state (robot_state-val msg))
))
