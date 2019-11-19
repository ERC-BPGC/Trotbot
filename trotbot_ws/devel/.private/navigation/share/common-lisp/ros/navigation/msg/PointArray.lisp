; Auto-generated. Do not edit!


(cl:in-package navigation-msg)


;//! \htmlinclude PointArray.msg.html

(cl:defclass <PointArray> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector navigation-msg:Point_xy)
   :initform (cl:make-array 0 :element-type 'navigation-msg:Point_xy :initial-element (cl:make-instance 'navigation-msg:Point_xy))))
)

(cl:defclass PointArray (<PointArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PointArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PointArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation-msg:<PointArray> is deprecated: use navigation-msg:PointArray instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <PointArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-msg:points-val is deprecated.  Use navigation-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PointArray>) ostream)
  "Serializes a message object of type '<PointArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PointArray>) istream)
  "Deserializes a message object of type '<PointArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'navigation-msg:Point_xy))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PointArray>)))
  "Returns string type for a message object of type '<PointArray>"
  "navigation/PointArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointArray)))
  "Returns string type for a message object of type 'PointArray"
  "navigation/PointArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PointArray>)))
  "Returns md5sum for a message object of type '<PointArray>"
  "9c2c9f601cd3038d8ee0a839b54636d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PointArray)))
  "Returns md5sum for a message object of type 'PointArray"
  "9c2c9f601cd3038d8ee0a839b54636d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PointArray>)))
  "Returns full string definition for message of type '<PointArray>"
  (cl:format cl:nil "navigation/Point_xy[] points~%  ~%================================================================================~%MSG: navigation/Point_xy~%float32[] point~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PointArray)))
  "Returns full string definition for message of type 'PointArray"
  (cl:format cl:nil "navigation/Point_xy[] points~%  ~%================================================================================~%MSG: navigation/Point_xy~%float32[] point~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PointArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PointArray>))
  "Converts a ROS message object to a list"
  (cl:list 'PointArray
    (cl:cons ':points (points msg))
))
