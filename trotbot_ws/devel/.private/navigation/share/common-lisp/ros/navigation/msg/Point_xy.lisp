; Auto-generated. Do not edit!


(cl:in-package navigation-msg)


;//! \htmlinclude Point_xy.msg.html

(cl:defclass <Point_xy> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Point_xy (<Point_xy>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Point_xy>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Point_xy)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation-msg:<Point_xy> is deprecated: use navigation-msg:Point_xy instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <Point_xy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-msg:point-val is deprecated.  Use navigation-msg:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Point_xy>) ostream)
  "Serializes a message object of type '<Point_xy>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'point))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'point))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Point_xy>) istream)
  "Deserializes a message object of type '<Point_xy>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'point) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'point)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Point_xy>)))
  "Returns string type for a message object of type '<Point_xy>"
  "navigation/Point_xy")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Point_xy)))
  "Returns string type for a message object of type 'Point_xy"
  "navigation/Point_xy")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Point_xy>)))
  "Returns md5sum for a message object of type '<Point_xy>"
  "318ea976b093c91a3f95a8e83351f8ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Point_xy)))
  "Returns md5sum for a message object of type 'Point_xy"
  "318ea976b093c91a3f95a8e83351f8ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Point_xy>)))
  "Returns full string definition for message of type '<Point_xy>"
  (cl:format cl:nil "float32[] point~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Point_xy)))
  "Returns full string definition for message of type 'Point_xy"
  (cl:format cl:nil "float32[] point~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Point_xy>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'point) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Point_xy>))
  "Converts a ROS message object to a list"
  (cl:list 'Point_xy
    (cl:cons ':point (point msg))
))
