; Auto-generated. Do not edit!


(cl:in-package ur_msgs-srv)


;//! \htmlinclude GripperMove-request.msg.html

(cl:defclass <GripperMove-request> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type cl:fixnum
    :initform 0)
   (force
    :reader force
    :initarg :force
    :type cl:fixnum
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0)
   (position_compensation
    :reader position_compensation
    :initarg :position_compensation
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GripperMove-request (<GripperMove-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperMove-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperMove-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_msgs-srv:<GripperMove-request> is deprecated: use ur_msgs-srv:GripperMove-request instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <GripperMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:pos-val is deprecated.  Use ur_msgs-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <GripperMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:force-val is deprecated.  Use ur_msgs-srv:force instead.")
  (force m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <GripperMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:speed-val is deprecated.  Use ur_msgs-srv:speed instead.")
  (speed m))

(cl:ensure-generic-function 'position_compensation-val :lambda-list '(m))
(cl:defmethod position_compensation-val ((m <GripperMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:position_compensation-val is deprecated.  Use ur_msgs-srv:position_compensation instead.")
  (position_compensation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperMove-request>) ostream)
  "Serializes a message object of type '<GripperMove-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'force)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'position_compensation) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperMove-request>) istream)
  "Deserializes a message object of type '<GripperMove-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'force)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position_compensation) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperMove-request>)))
  "Returns string type for a service object of type '<GripperMove-request>"
  "ur_msgs/GripperMoveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperMove-request)))
  "Returns string type for a service object of type 'GripperMove-request"
  "ur_msgs/GripperMoveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperMove-request>)))
  "Returns md5sum for a message object of type '<GripperMove-request>"
  "d1d55f75257890b063dd76689fbde353")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperMove-request)))
  "Returns md5sum for a message object of type 'GripperMove-request"
  "d1d55f75257890b063dd76689fbde353")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperMove-request>)))
  "Returns full string definition for message of type '<GripperMove-request>"
  (cl:format cl:nil "uint8 pos~%uint8 force~%uint8 speed~%bool position_compensation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperMove-request)))
  "Returns full string definition for message of type 'GripperMove-request"
  (cl:format cl:nil "uint8 pos~%uint8 force~%uint8 speed~%bool position_compensation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperMove-request>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperMove-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperMove-request
    (cl:cons ':pos (pos msg))
    (cl:cons ':force (force msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':position_compensation (position_compensation msg))
))
;//! \htmlinclude GripperMove-response.msg.html

(cl:defclass <GripperMove-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (has_object
    :reader has_object
    :initarg :has_object
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GripperMove-response (<GripperMove-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperMove-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperMove-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_msgs-srv:<GripperMove-response> is deprecated: use ur_msgs-srv:GripperMove-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GripperMove-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:success-val is deprecated.  Use ur_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'has_object-val :lambda-list '(m))
(cl:defmethod has_object-val ((m <GripperMove-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:has_object-val is deprecated.  Use ur_msgs-srv:has_object instead.")
  (has_object m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperMove-response>) ostream)
  "Serializes a message object of type '<GripperMove-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_object) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperMove-response>) istream)
  "Deserializes a message object of type '<GripperMove-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'has_object) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperMove-response>)))
  "Returns string type for a service object of type '<GripperMove-response>"
  "ur_msgs/GripperMoveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperMove-response)))
  "Returns string type for a service object of type 'GripperMove-response"
  "ur_msgs/GripperMoveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperMove-response>)))
  "Returns md5sum for a message object of type '<GripperMove-response>"
  "d1d55f75257890b063dd76689fbde353")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperMove-response)))
  "Returns md5sum for a message object of type 'GripperMove-response"
  "d1d55f75257890b063dd76689fbde353")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperMove-response>)))
  "Returns full string definition for message of type '<GripperMove-response>"
  (cl:format cl:nil "bool success~%bool has_object~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperMove-response)))
  "Returns full string definition for message of type 'GripperMove-response"
  (cl:format cl:nil "bool success~%bool has_object~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperMove-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperMove-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperMove-response
    (cl:cons ':success (success msg))
    (cl:cons ':has_object (has_object msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GripperMove)))
  'GripperMove-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GripperMove)))
  'GripperMove-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperMove)))
  "Returns string type for a service object of type '<GripperMove>"
  "ur_msgs/GripperMove")