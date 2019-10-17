; Auto-generated. Do not edit!


(cl:in-package vision_based_picking-srv)


;//! \htmlinclude Acquire-request.msg.html

(cl:defclass <Acquire-request> (roslisp-msg-protocol:ros-message)
  ((the_request
    :reader the_request
    :initarg :the_request
    :type cl:string
    :initform "")
   (cam_index
    :reader cam_index
    :initarg :cam_index
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Acquire-request (<Acquire-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Acquire-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Acquire-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision_based_picking-srv:<Acquire-request> is deprecated: use vision_based_picking-srv:Acquire-request instead.")))

(cl:ensure-generic-function 'the_request-val :lambda-list '(m))
(cl:defmethod the_request-val ((m <Acquire-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_based_picking-srv:the_request-val is deprecated.  Use vision_based_picking-srv:the_request instead.")
  (the_request m))

(cl:ensure-generic-function 'cam_index-val :lambda-list '(m))
(cl:defmethod cam_index-val ((m <Acquire-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_based_picking-srv:cam_index-val is deprecated.  Use vision_based_picking-srv:cam_index instead.")
  (cam_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Acquire-request>) ostream)
  "Serializes a message object of type '<Acquire-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'the_request))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'the_request))
  (cl:let* ((signed (cl:slot-value msg 'cam_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Acquire-request>) istream)
  "Deserializes a message object of type '<Acquire-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'the_request) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'the_request) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cam_index) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Acquire-request>)))
  "Returns string type for a service object of type '<Acquire-request>"
  "vision_based_picking/AcquireRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Acquire-request)))
  "Returns string type for a service object of type 'Acquire-request"
  "vision_based_picking/AcquireRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Acquire-request>)))
  "Returns md5sum for a message object of type '<Acquire-request>"
  "4cd58e4f9987399415b3ab1f2afe4e12")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Acquire-request)))
  "Returns md5sum for a message object of type 'Acquire-request"
  "4cd58e4f9987399415b3ab1f2afe4e12")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Acquire-request>)))
  "Returns full string definition for message of type '<Acquire-request>"
  (cl:format cl:nil "string the_request~%int16 cam_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Acquire-request)))
  "Returns full string definition for message of type 'Acquire-request"
  (cl:format cl:nil "string the_request~%int16 cam_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Acquire-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'the_request))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Acquire-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Acquire-request
    (cl:cons ':the_request (the_request msg))
    (cl:cons ':cam_index (cam_index msg))
))
;//! \htmlinclude Acquire-response.msg.html

(cl:defclass <Acquire-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Acquire-response (<Acquire-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Acquire-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Acquire-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision_based_picking-srv:<Acquire-response> is deprecated: use vision_based_picking-srv:Acquire-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Acquire-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_based_picking-srv:status-val is deprecated.  Use vision_based_picking-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Acquire-response>) ostream)
  "Serializes a message object of type '<Acquire-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Acquire-response>) istream)
  "Deserializes a message object of type '<Acquire-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Acquire-response>)))
  "Returns string type for a service object of type '<Acquire-response>"
  "vision_based_picking/AcquireResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Acquire-response)))
  "Returns string type for a service object of type 'Acquire-response"
  "vision_based_picking/AcquireResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Acquire-response>)))
  "Returns md5sum for a message object of type '<Acquire-response>"
  "4cd58e4f9987399415b3ab1f2afe4e12")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Acquire-response)))
  "Returns md5sum for a message object of type 'Acquire-response"
  "4cd58e4f9987399415b3ab1f2afe4e12")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Acquire-response>)))
  "Returns full string definition for message of type '<Acquire-response>"
  (cl:format cl:nil "int16 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Acquire-response)))
  "Returns full string definition for message of type 'Acquire-response"
  (cl:format cl:nil "int16 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Acquire-response>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Acquire-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Acquire-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Acquire)))
  'Acquire-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Acquire)))
  'Acquire-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Acquire)))
  "Returns string type for a service object of type '<Acquire>"
  "vision_based_picking/Acquire")