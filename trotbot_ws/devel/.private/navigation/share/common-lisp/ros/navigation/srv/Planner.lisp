; Auto-generated. Do not edit!


(cl:in-package navigation-srv)


;//! \htmlinclude Planner-request.msg.html

(cl:defclass <Planner-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type navigation-msg:Point_xy
    :initform (cl:make-instance 'navigation-msg:Point_xy))
   (goal
    :reader goal
    :initarg :goal
    :type navigation-msg:Point_xy
    :initform (cl:make-instance 'navigation-msg:Point_xy))
   (obstacle_list
    :reader obstacle_list
    :initarg :obstacle_list
    :type navigation-msg:PolyArray
    :initform (cl:make-instance 'navigation-msg:PolyArray)))
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

(cl:ensure-generic-function 'obstacle_list-val :lambda-list '(m))
(cl:defmethod obstacle_list-val ((m <Planner-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-srv:obstacle_list-val is deprecated.  Use navigation-srv:obstacle_list instead.")
  (obstacle_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Planner-request>) ostream)
  "Serializes a message object of type '<Planner-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obstacle_list) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Planner-request>) istream)
  "Deserializes a message object of type '<Planner-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obstacle_list) istream)
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
  "0f8ba09d5a21e9916f0e3bf633247872")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Planner-request)))
  "Returns md5sum for a message object of type 'Planner-request"
  "0f8ba09d5a21e9916f0e3bf633247872")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Planner-request>)))
  "Returns full string definition for message of type '<Planner-request>"
  (cl:format cl:nil "navigation/Point_xy start~%navigation/Point_xy goal~%navigation/PolyArray obstacle_list~%~%================================================================================~%MSG: navigation/Point_xy~%float32[] point~%================================================================================~%MSG: navigation/PolyArray~%navigation/PointArray[] polygons~%~%================================================================================~%MSG: navigation/PointArray~%navigation/Point_xy[] points~%  ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Planner-request)))
  "Returns full string definition for message of type 'Planner-request"
  (cl:format cl:nil "navigation/Point_xy start~%navigation/Point_xy goal~%navigation/PolyArray obstacle_list~%~%================================================================================~%MSG: navigation/Point_xy~%float32[] point~%================================================================================~%MSG: navigation/PolyArray~%navigation/PointArray[] polygons~%~%================================================================================~%MSG: navigation/PointArray~%navigation/Point_xy[] points~%  ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Planner-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obstacle_list))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Planner-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Planner-request
    (cl:cons ':start (start msg))
    (cl:cons ':goal (goal msg))
    (cl:cons ':obstacle_list (obstacle_list msg))
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
  "0f8ba09d5a21e9916f0e3bf633247872")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Planner-response)))
  "Returns md5sum for a message object of type 'Planner-response"
  "0f8ba09d5a21e9916f0e3bf633247872")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Planner-response>)))
  "Returns full string definition for message of type '<Planner-response>"
  (cl:format cl:nil "navigation/PointArray path~%bool ack~%~%================================================================================~%MSG: navigation/PointArray~%navigation/Point_xy[] points~%  ~%================================================================================~%MSG: navigation/Point_xy~%float32[] point~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Planner-response)))
  "Returns full string definition for message of type 'Planner-response"
  (cl:format cl:nil "navigation/PointArray path~%bool ack~%~%================================================================================~%MSG: navigation/PointArray~%navigation/Point_xy[] points~%  ~%================================================================================~%MSG: navigation/Point_xy~%float32[] point~%~%"))
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