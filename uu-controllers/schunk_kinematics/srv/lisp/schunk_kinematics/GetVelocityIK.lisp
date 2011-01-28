; Auto-generated. Do not edit!


(in-package schunk_kinematics-srv)


;//! \htmlinclude GetVelocityIK-request.msg.html

(defclass <GetVelocityIK-request> (ros-message)
  ((ik_request
    :reader ik_request-val
    :initarg :ik_request
    :type schunk_kinematics-msg:<VelocityIKRequest>
    :initform (make-instance 'schunk_kinematics-msg:<VelocityIKRequest>))
   (timeout
    :reader timeout-val
    :initarg :timeout
    :type real
    :initform 0))
)
(defmethod serialize ((msg <GetVelocityIK-request>) ostream)
  "Serializes a message object of type '<GetVelocityIK-request>"
  (serialize (slot-value msg 'ik_request) ostream)
  (let ((__sec (floor (slot-value msg 'timeout)))
        (__nsec (round (* 1e9 (- (slot-value msg 'timeout) (floor (slot-value msg 'timeout)))))))
    (write-byte (ldb (byte 8 0) __sec) ostream)
    (write-byte (ldb (byte 8 8) __sec) ostream)
    (write-byte (ldb (byte 8 16) __sec) ostream)
    (write-byte (ldb (byte 8 24) __sec) ostream)
    (write-byte (ldb (byte 8 0) __nsec) ostream)
    (write-byte (ldb (byte 8 8) __nsec) ostream)
    (write-byte (ldb (byte 8 16) __nsec) ostream)
    (write-byte (ldb (byte 8 24) __nsec) ostream))
)
(defmethod deserialize ((msg <GetVelocityIK-request>) istream)
  "Deserializes a message object of type '<GetVelocityIK-request>"
  (deserialize (slot-value msg 'ik_request) istream)
  (let ((__sec 0) (__nsec 0))
    (setf (ldb (byte 8 0) __sec) (read-byte istream))
    (setf (ldb (byte 8 8) __sec) (read-byte istream))
    (setf (ldb (byte 8 16) __sec) (read-byte istream))
    (setf (ldb (byte 8 24) __sec) (read-byte istream))
    (setf (ldb (byte 8 0) __nsec) (read-byte istream))
    (setf (ldb (byte 8 8) __nsec) (read-byte istream))
    (setf (ldb (byte 8 16) __nsec) (read-byte istream))
    (setf (ldb (byte 8 24) __nsec) (read-byte istream))
    (setf (slot-value msg 'timeout) (+ (coerce __sec 'double-float) (/ __nsec 1e9))))
  msg
)
(defmethod ros-datatype ((msg (eql '<GetVelocityIK-request>)))
  "Returns string type for a service object of type '<GetVelocityIK-request>"
  "schunk_kinematics/GetVelocityIKRequest")
(defmethod md5sum ((type (eql '<GetVelocityIK-request>)))
  "Returns md5sum for a message object of type '<GetVelocityIK-request>"
  "a916ab3d9e042d3490ff22dc06b05611")
(defmethod message-definition ((type (eql '<GetVelocityIK-request>)))
  "Returns full string definition for message of type '<GetVelocityIK-request>"
  (format nil "# A service call to carry out an inverse kinematics computation for velocity~%# The inverse kinematics request~%schunk_kinematics/VelocityIKRequest ik_request~%# Maximum allowed time for IK calculation~%duration timeout~%~%"))
(defmethod serialization-length ((msg <GetVelocityIK-request>))
  (+ 0
     (serialization-length (slot-value msg 'ik_request))
     8
))
(defmethod ros-message-to-list ((msg <GetVelocityIK-request>))
  "Converts a ROS message object to a list"
  (list '<GetVelocityIK-request>
    (cons ':ik_request (ik_request-val msg))
    (cons ':timeout (timeout-val msg))
))
;//! \htmlinclude GetVelocityIK-response.msg.html

(defclass <GetVelocityIK-response> (ros-message)
  ((solution
    :reader solution-val
    :initarg :solution
    :type motion_planning_msgs-msg:<RobotState>
    :initform (make-instance 'motion_planning_msgs-msg:<RobotState>))
   (error_code
    :reader error_code-val
    :initarg :error_code
    :type motion_planning_msgs-msg:<ArmNavigationErrorCodes>
    :initform (make-instance 'motion_planning_msgs-msg:<ArmNavigationErrorCodes>)))
)
(defmethod serialize ((msg <GetVelocityIK-response>) ostream)
  "Serializes a message object of type '<GetVelocityIK-response>"
  (serialize (slot-value msg 'solution) ostream)
  (serialize (slot-value msg 'error_code) ostream)
)
(defmethod deserialize ((msg <GetVelocityIK-response>) istream)
  "Deserializes a message object of type '<GetVelocityIK-response>"
  (deserialize (slot-value msg 'solution) istream)
  (deserialize (slot-value msg 'error_code) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<GetVelocityIK-response>)))
  "Returns string type for a service object of type '<GetVelocityIK-response>"
  "schunk_kinematics/GetVelocityIKResponse")
(defmethod md5sum ((type (eql '<GetVelocityIK-response>)))
  "Returns md5sum for a message object of type '<GetVelocityIK-response>"
  "a916ab3d9e042d3490ff22dc06b05611")
(defmethod message-definition ((type (eql '<GetVelocityIK-response>)))
  "Returns full string definition for message of type '<GetVelocityIK-response>"
  (format nil "# The returned solution ~%# (in the same order as the list of joints specified in the IKRequest message)~%motion_planning_msgs/RobotState solution~%~%motion_planning_msgs/ArmNavigationErrorCodes error_code~%~%~%================================================================================~%MSG: schunk_kinematics/VelocityIKRequest~%# A Velocity IK request message~%# The name of the link for which we are computing IK~%string ik_link_name~%~%# The twist of the link~%geometry_msgs/Twist twist~%~%# Here specify the starting positions ~%# of joints/links on the robot.~%motion_planning_msgs/RobotState robot_state~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: motion_planning_msgs/RobotState~%# This message contains information about the robot state, i.e. the positions of its joints and links~%sensor_msgs/JointState joint_state~%motion_planning_msgs/MultiDOFJointState multi_dof_joint_state~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: motion_planning_msgs/MultiDOFJointState~%#A representation of a multi-dof joint state~%time stamp~%string joint_name~%string frame_id~%string child_frame_id~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: motion_planning_msgs/ArmNavigationErrorCodes~%int32 val~%~%# overall behavior~%int32 PLANNING_FAILED=-1~%int32 SUCCESS=1~%int32 TIMED_OUT=-2~%~%# start state errors~%int32 START_STATE_IN_COLLISION=-3~%int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-4~%~%# goal errors~%int32 GOAL_IN_COLLISION=-5~%int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-6~%~%# robot state~%int32 INVALID_ROBOT_STATE=-7~%int32 INCOMPLETE_ROBOT_STATE=-8~%~%# planning request errors~%int32 INVALID_PLANNER_ID=-9~%int32 INVALID_NUM_PLANNING_ATTEMPTS=-10~%int32 INVALID_ALLOWED_PLANNING_TIME=-11~%int32 INVALID_GROUP_NAME=-12~%int32 INVALID_GOAL_JOINT_CONSTRAINTS=-13~%int32 INVALID_GOAL_POSITION_CONSTRAINTS=-14~%int32 INVALID_GOAL_ORIENTATION_CONSTRAINTS=-15~%int32 INVALID_PATH_JOINT_CONSTRAINTS=-16~%int32 INVALID_PATH_POSITION_CONSTRAINTS=-17~%int32 INVALID_PATH_ORIENTATION_CONSTRAINTS=-18~%~%# state/trajectory monitor errors~%int32 INVALID_TRAJECTORY=-19~%int32 INVALID_INDEX=-20~%int32 JOINT_LIMITS_VIOLATED=-21~%int32 PATH_CONSTRAINTS_VIOLATED=-22~%int32 COLLISION_CONSTRAINTS_VIOLATED=-23~%int32 GOAL_CONSTRAINTS_VIOLATED=-24~%int32 JOINTS_NOT_MOVING=-25~%int32 TRAJECTORY_CONTROLLER_FAILED=-26~%~%# system errors~%int32 FRAME_TRANSFORM_FAILURE=-27~%int32 COLLISION_CHECKING_UNAVAILABLE=-28~%int32 ROBOT_STATE_STALE=-29~%int32 SENSOR_INFO_STALE=-30~%~%# kinematics errors~%int32 NO_IK_SOLUTION=-31~%int32 INVALID_LINK_NAME=-32~%int32 IK_LINK_IN_COLLISION=-33~%int32 NO_FK_SOLUTION=-34~%int32 KINEMATICS_STATE_IN_COLLISION=-35~%~%# general errors~%int32 INVALID_TIMEOUT=-36~%~%~%~%"))
(defmethod serialization-length ((msg <GetVelocityIK-response>))
  (+ 0
     (serialization-length (slot-value msg 'solution))
     (serialization-length (slot-value msg 'error_code))
))
(defmethod ros-message-to-list ((msg <GetVelocityIK-response>))
  "Converts a ROS message object to a list"
  (list '<GetVelocityIK-response>
    (cons ':solution (solution-val msg))
    (cons ':error_code (error_code-val msg))
))
(defmethod service-request-type ((msg (eql 'GetVelocityIK)))
  '<GetVelocityIK-request>)
(defmethod service-response-type ((msg (eql 'GetVelocityIK)))
  '<GetVelocityIK-response>)
(defmethod ros-datatype ((msg (eql 'GetVelocityIK)))
  "Returns string type for a service object of type '<GetVelocityIK>"
  "schunk_kinematics/GetVelocityIK")
