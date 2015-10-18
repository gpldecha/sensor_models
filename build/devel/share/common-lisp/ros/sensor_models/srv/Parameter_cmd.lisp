; Auto-generated. Do not edit!


(cl:in-package sensor_models-srv)


;//! \htmlinclude Parameter_cmd-request.msg.html

(cl:defclass <Parameter_cmd-request> (roslisp-msg-protocol:ros-message)
  ((str
    :reader str
    :initarg :str
    :type cl:string
    :initform "")
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0))
)

(cl:defclass Parameter_cmd-request (<Parameter_cmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Parameter_cmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Parameter_cmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_models-srv:<Parameter_cmd-request> is deprecated: use sensor_models-srv:Parameter_cmd-request instead.")))

(cl:ensure-generic-function 'str-val :lambda-list '(m))
(cl:defmethod str-val ((m <Parameter_cmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_models-srv:str-val is deprecated.  Use sensor_models-srv:str instead.")
  (str m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <Parameter_cmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_models-srv:theta-val is deprecated.  Use sensor_models-srv:theta instead.")
  (theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Parameter_cmd-request>) ostream)
  "Serializes a message object of type '<Parameter_cmd-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'str))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'str))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Parameter_cmd-request>) istream)
  "Deserializes a message object of type '<Parameter_cmd-request>"
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
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Parameter_cmd-request>)))
  "Returns string type for a service object of type '<Parameter_cmd-request>"
  "sensor_models/Parameter_cmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Parameter_cmd-request)))
  "Returns string type for a service object of type 'Parameter_cmd-request"
  "sensor_models/Parameter_cmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Parameter_cmd-request>)))
  "Returns md5sum for a message object of type '<Parameter_cmd-request>"
  "5c79c82d6c588d5669b0f1cbf1ff2ad0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Parameter_cmd-request)))
  "Returns md5sum for a message object of type 'Parameter_cmd-request"
  "5c79c82d6c588d5669b0f1cbf1ff2ad0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Parameter_cmd-request>)))
  "Returns full string definition for message of type '<Parameter_cmd-request>"
  (cl:format cl:nil "string str~%float32 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Parameter_cmd-request)))
  "Returns full string definition for message of type 'Parameter_cmd-request"
  (cl:format cl:nil "string str~%float32 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Parameter_cmd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'str))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Parameter_cmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Parameter_cmd-request
    (cl:cons ':str (str msg))
    (cl:cons ':theta (theta msg))
))
;//! \htmlinclude Parameter_cmd-response.msg.html

(cl:defclass <Parameter_cmd-response> (roslisp-msg-protocol:ros-message)
  ((str
    :reader str
    :initarg :str
    :type cl:string
    :initform ""))
)

(cl:defclass Parameter_cmd-response (<Parameter_cmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Parameter_cmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Parameter_cmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_models-srv:<Parameter_cmd-response> is deprecated: use sensor_models-srv:Parameter_cmd-response instead.")))

(cl:ensure-generic-function 'str-val :lambda-list '(m))
(cl:defmethod str-val ((m <Parameter_cmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_models-srv:str-val is deprecated.  Use sensor_models-srv:str instead.")
  (str m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Parameter_cmd-response>) ostream)
  "Serializes a message object of type '<Parameter_cmd-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'str))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'str))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Parameter_cmd-response>) istream)
  "Deserializes a message object of type '<Parameter_cmd-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Parameter_cmd-response>)))
  "Returns string type for a service object of type '<Parameter_cmd-response>"
  "sensor_models/Parameter_cmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Parameter_cmd-response)))
  "Returns string type for a service object of type 'Parameter_cmd-response"
  "sensor_models/Parameter_cmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Parameter_cmd-response>)))
  "Returns md5sum for a message object of type '<Parameter_cmd-response>"
  "5c79c82d6c588d5669b0f1cbf1ff2ad0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Parameter_cmd-response)))
  "Returns md5sum for a message object of type 'Parameter_cmd-response"
  "5c79c82d6c588d5669b0f1cbf1ff2ad0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Parameter_cmd-response>)))
  "Returns full string definition for message of type '<Parameter_cmd-response>"
  (cl:format cl:nil "string str~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Parameter_cmd-response)))
  "Returns full string definition for message of type 'Parameter_cmd-response"
  (cl:format cl:nil "string str~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Parameter_cmd-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'str))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Parameter_cmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Parameter_cmd-response
    (cl:cons ':str (str msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Parameter_cmd)))
  'Parameter_cmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Parameter_cmd)))
  'Parameter_cmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Parameter_cmd)))
  "Returns string type for a service object of type '<Parameter_cmd>"
  "sensor_models/Parameter_cmd")