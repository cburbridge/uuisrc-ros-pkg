; Auto-generated. Do not edit!


(in-package schunkarm_server-msg)


;//! \htmlinclude movearmActionGoal.msg.html

(defclass <movearmActionGoal> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (goal_id
    :reader goal_id-val
    :initarg :goal_id
    :type actionlib_msgs-msg:<GoalID>
    :initform (make-instance 'actionlib_msgs-msg:<GoalID>))
   (goal
    :reader goal-val
    :initarg :goal
    :type schunkarm_server-msg:<movearmGoal>
    :initform (make-instance 'schunkarm_server-msg:<movearmGoal>)))
)
(defmethod serialize ((msg <movearmActionGoal>) ostream)
  "Serializes a message object of type '<movearmActionGoal>"
  (serialize (slot-value msg 'header) ostream)
  (serialize (slot-value msg 'goal_id) ostream)
  (serialize (slot-value msg 'goal) ostream)
)
(defmethod deserialize ((msg <movearmActionGoal>) istream)
  "Deserializes a message object of type '<movearmActionGoal>"
  (deserialize (slot-value msg 'header) istream)
  (deserialize (slot-value msg 'goal_id) istream)
  (deserialize (slot-value msg 'goal) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<movearmActionGoal>)))
  "Returns string type for a message object of type '<movearmActionGoal>"
  "schunkarm_server/movearmActionGoal")
(defmethod md5sum ((type (eql '<movearmActionGoal>)))
  "Returns md5sum for a message object of type '<movearmActionGoal>"
  "5d6b6045f8bb6b2628ea044e5e36ee6d")
(defmethod message-definition ((type (eql '<movearmActionGoal>)))
  "Returns full string definition for message of type '<movearmActionGoal>"
  (format nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%movearmGoal goal~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: schunkarm_server/movearmGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#Goal~%geometry_msgs/Pose goal_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(defmethod serialization-length ((msg <movearmActionGoal>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     (serialization-length (slot-value msg 'goal_id))
     (serialization-length (slot-value msg 'goal))
))
(defmethod ros-message-to-list ((msg <movearmActionGoal>))
  "Converts a ROS message object to a list"
  (list '<movearmActionGoal>
    (cons ':header (header-val msg))
    (cons ':goal_id (goal_id-val msg))
    (cons ':goal (goal-val msg))
))
