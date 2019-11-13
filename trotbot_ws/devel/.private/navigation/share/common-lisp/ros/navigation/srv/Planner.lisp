; Auto-generated. Do not edit!


(cl:in-package navigation-srv)


;//! \htmlinclude Planner-request.msg.html

(cl:defclass <Planner-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type (cl:vector std_msgs-msg:Float32)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:Float32 :initial-element (cl:make-instance 'std_msgs-msg:Float32)))
   (goal
    :reader goal
    :initarg :goal
    :type (cl:vector std_msgs-msg:Float32)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:Float32 :initial-element (cl:make-instance 'std_msgs-msg:Float32))))
)

(cl:defclass Planner-request (<Planner-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Planner-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Planner-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation-srv:<Planner-request> is deprecated: use navigation-srv:Planner-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <Planner-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-srv:start-val is deprecated.  Use navigation-srv:start instead.")
  (start m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <Planner-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-srv:goal-val is deprecated.  Use navigation-srv:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Planner-request>) ostream)
  "Serializes a message object of type '<Planner-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'start))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'start))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'goal))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Planner-request>) istream)
  "Deserializes a message object of type '<Planner-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'start) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'start)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:Float32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'goal) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'goal)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:Float32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Planner-request>)))
  "Returns string type for a service object of type '<Planner-request>"
  "navigation/PlannerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Planner-request)))
  "Returns string type for a service object of type 'Planner-request"
  "navigation/PlannerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Planner-request>)))
  "Returns md5sum for a message object of type '<Planner-request>"
  "e099e7e7ce9aa3ba23a5f56a66d6f6e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Planner-request)))
  "Returns md5sum for a message object of type 'Planner-request"
  "e099e7e7ce9aa3ba23a5f56a66d6f6e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Planner-request>)))
  "Returns full string definition for message of type '<Planner-request>"
  (cl:format cl:nil "std_msgs/Float32[] start~%std_msgs/Float32[] goal~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Planner-request)))
  "Returns full string definition for message of type 'Planner-request"
  (cl:format cl:nil "std_msgs/Float32[] start~%std_msgs/Float32[] goal~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Planner-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'start) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'goal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Planner-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Planner-request
    (cl:cons ':start (start msg))
    (cl:cons ':goal (goal msg))
))
;//! \htmlinclude Planner-response.msg.html

(cl:defclass <Planner-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type navigation-msg:PointArray
    :initform (cl:make-instance 'navigation-msg:PointArray))
   (ack
    :reader ack
    :initarg :ack
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Planner-response (<Planner-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Planner-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Planner-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation-srv:<Planner-response> is deprecated: use navigation-srv:Planner-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <Planner-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-srv:path-val is deprecated.  Use navigation-srv:path instead.")
  (path m))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <Planner-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-srv:ack-val is deprecated.  Use navigation-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Planner-response>) ostream)
  "Serializes a message object of type '<Planner-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ack) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Planner-response>) istream)
  "Deserializes a message object of type '<Planner-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
    (cl:setf (cl:slot-value msg 'ack) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Planner-response>)))
  "Returns string type for a service object of type '<Planner-response>"
  "navigation/PlannerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Planner-response)))
  "Returns string type for a service object of type 'Planner-response"
  "navigation/PlannerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Planner-response>)))
  "Returns md5sum for a message object of type '<Planner-response>"
  "e099e7e7ce9aa3ba23a5f56a66d6f6e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Planner-response)))
  "Returns md5sum for a message object of type 'Planner-response"
  "e099e7e7ce9aa3ba23a5f56a66d6f6e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Planner-response>)))
  "Returns full string definition for message of type '<Planner-response>"
  (cl:format cl:nil "navigation/PointArray path~%bool ack~%~%================================================================================~%MSG: navigation/PointArray~%std_msgs/Header header~%geometry_msgs/Point[] points~%  ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Planner-response)))
  "Returns full string definition for message of type 'Planner-response"
  (cl:format cl:nil "navigation/PointArray path~%bool ack~%~%================================================================================~%MSG: navigation/PointArray~%std_msgs/Header header~%geometry_msgs/Point[] points~%  ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Planner-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Planner-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Planner-response
    (cl:cons ':path (path msg))
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Planner)))
  'Planner-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Planner)))
  'Planner-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Planner)))
  "Returns string type for a service object of type '<Planner>"
  "navigation/Planner")