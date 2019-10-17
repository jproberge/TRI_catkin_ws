; Auto-generated. Do not edit!


(cl:in-package ur_msgs-msg)


;//! \htmlinclude FollowCartesianTrajectoryGoal.msg.html

(cl:defclass <FollowCartesianTrajectoryGoal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (poses
    :reader poses
    :initarg :poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0))
)

(cl:defclass FollowCartesianTrajectoryGoal (<FollowCartesianTrajectoryGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FollowCartesianTrajectoryGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FollowCartesianTrajectoryGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_msgs-msg:<FollowCartesianTrajectoryGoal> is deprecated: use ur_msgs-msg:FollowCartesianTrajectoryGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FollowCartesianTrajectoryGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-msg:header-val is deprecated.  Use ur_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <FollowCartesianTrajectoryGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-msg:poses-val is deprecated.  Use ur_msgs-msg:poses instead.")
  (poses m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <FollowCartesianTrajectoryGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-msg:velocity-val is deprecated.  Use ur_msgs-msg:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FollowCartesianTrajectoryGoal>) ostream)
  "Serializes a message object of type '<FollowCartesianTrajectoryGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FollowCartesianTrajectoryGoal>) istream)
  "Deserializes a message object of type '<FollowCartesianTrajectoryGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FollowCartesianTrajectoryGoal>)))
  "Returns string type for a message object of type '<FollowCartesianTrajectoryGoal>"
  "ur_msgs/FollowCartesianTrajectoryGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FollowCartesianTrajectoryGoal)))
  "Returns string type for a message object of type 'FollowCartesianTrajectoryGoal"
  "ur_msgs/FollowCartesianTrajectoryGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FollowCartesianTrajectoryGoal>)))
  "Returns md5sum for a message object of type '<FollowCartesianTrajectoryGoal>"
  "c216c7e2f7bdd5dcc81619d395471cdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FollowCartesianTrajectoryGoal)))
  "Returns md5sum for a message object of type 'FollowCartesianTrajectoryGoal"
  "c216c7e2f7bdd5dcc81619d395471cdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FollowCartesianTrajectoryGoal>)))
  "Returns full string definition for message of type '<FollowCartesianTrajectoryGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%Header header~%geometry_msgs/Pose[] poses~%float64 velocity~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FollowCartesianTrajectoryGoal)))
  "Returns full string definition for message of type 'FollowCartesianTrajectoryGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%Header header~%geometry_msgs/Pose[] poses~%float64 velocity~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FollowCartesianTrajectoryGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FollowCartesianTrajectoryGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'FollowCartesianTrajectoryGoal
    (cl:cons ':header (header msg))
    (cl:cons ':poses (poses msg))
    (cl:cons ':velocity (velocity msg))
))
