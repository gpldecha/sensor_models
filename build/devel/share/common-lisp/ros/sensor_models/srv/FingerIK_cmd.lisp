; Auto-generated. Do not edit!


(cl:in-package sensor_models-srv)


;//! \htmlinclude FingerIK_cmd-request.msg.html

(cl:defclass <FingerIK_cmd-request> (roslisp-msg-protocol:ros-message)
  ((str
    :reader str
    :initarg :str
    :type cl:string
    :initform "")
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass FingerIK_cmd-request (<FingerIK_cmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FingerIK_cmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FingerIK_cmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_models-srv:<FingerIK_cmd-request> is deprecated: use sensor_models-srv:FingerIK_cmd-request instead.")))

(cl:ensure-generic-function 'str-val :lambda-list '(m))
(cl:defmethod str-val ((m <FingerIK_cmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_models-srv:str-val is deprecated.  Use sensor_models-srv:str instead.")
  (str m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <FingerIK_cmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_models-srv:x-val is deprecated.  Use sensor_models-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <FingerIK_cmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_models-srv:y-val is deprecated.  Use sensor_models-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <FingerIK_cmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_models-srv:z-val is deprecated.  Use sensor_models-srv:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FingerIK_cmd-request>) ostream)
  "Serializes a message object of type '<FingerIK_cmd-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'str))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'str))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FingerIK_cmd-request>) istream)
  "Deserializes a message object of type '<FingerIK_cmd-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'str) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'str) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FingerIK_cmd-request>)))
  "Returns string type for a service object of type '<FingerIK_cmd-request>"
  "sensor_models/FingerIK_cmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FingerIK_cmd-request)))
  "Returns string type for a service object of type 'FingerIK_cmd-request"
  "sensor_models/FingerIK_cmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FingerIK_cmd-request>)))
  "Returns md5sum for a message object of type '<FingerIK_cmd-request>"
  "903d48d0627c46a85e18c737ad1a5a14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FingerIK_cmd-request)))
  "Returns md5sum for a message object of type 'FingerIK_cmd-request"
  "903d48d0627c46a85e18c737ad1a5a14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FingerIK_cmd-request>)))
  "Returns full string definition for message of type '<FingerIK_cmd-request>"
  (cl:format cl:nil "string str~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FingerIK_cmd-request)))
  "Returns full string definition for message of type 'FingerIK_cmd-request"
  (cl:format cl:nil "string str~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FingerIK_cmd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'str))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FingerIK_cmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FingerIK_cmd-request
    (cl:cons ':str (str msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
;//! \htmlinclude FingerIK_cmd-response.msg.html

(cl:defclass <FingerIK_cmd-response> (roslisp-msg-protocol:ros-message)
  ((str
    :reader str
    :initarg :str
    :type cl:string
    :initform ""))
)

(cl:defclass FingerIK_cmd-response (<FingerIK_cmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FingerIK_cmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FingerIK_cmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_models-srv:<FingerIK_cmd-response> is deprecated: use sensor_models-srv:FingerIK_cmd-response instead.")))

(cl:ensure-generic-function 'str-val :lambda-list '(m))
(cl:defmethod str-val ((m <FingerIK_cmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_models-srv:str-val is deprecated.  Use sensor_models-srv:str instead.")
  (str m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FingerIK_cmd-response>) ostream)
  "Serializes a message object of type '<FingerIK_cmd-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'str))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'str))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FingerIK_cmd-response>) istream)
  "Deserializes a message object of type '<FingerIK_cmd-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'str) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'str) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FingerIK_cmd-response>)))
  "Returns string type for a service object of type '<FingerIK_cmd-response>"
  "sensor_models/FingerIK_cmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FingerIK_cmd-response)))
  "Returns string type for a service object of type 'FingerIK_cmd-response"
  "sensor_models/FingerIK_cmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FingerIK_cmd-response>)))
  "Returns md5sum for a message object of type '<FingerIK_cmd-response>"
  "903d48d0627c46a85e18c737ad1a5a14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FingerIK_cmd-response)))
  "Returns md5sum for a message object of type 'FingerIK_cmd-response"
  "903d48d0627c46a85e18c737ad1a5a14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FingerIK_cmd-response>)))
  "Returns full string definition for message of type '<FingerIK_cmd-response>"
  (cl:format cl:nil "string str~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FingerIK_cmd-response)))
  "Returns full string definition for message of type 'FingerIK_cmd-response"
  (cl:format cl:nil "string str~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FingerIK_cmd-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'str))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FingerIK_cmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FingerIK_cmd-response
    (cl:cons ':str (str msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FingerIK_cmd)))
  'FingerIK_cmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FingerIK_cmd)))
  'FingerIK_cmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FingerIK_cmd)))
  "Returns string type for a service object of type '<FingerIK_cmd>"
  "sensor_models/FingerIK_cmd")