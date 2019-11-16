; Auto-generated. Do not edit!


(cl:in-package navigation-msg)


;//! \htmlinclude PolyArray.msg.html

(cl:defclass <PolyArray> (roslisp-msg-protocol:ros-message)
  ((polygons
    :reader polygons
    :initarg :polygons
    :type (cl:vector navigation-msg:PointArray)
   :initform (cl:make-array 0 :element-type 'navigation-msg:PointArray :initial-element (cl:make-instance 'navigation-msg:PointArray))))
)

(cl:defclass PolyArray (<PolyArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PolyArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PolyArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation-msg:<PolyArray> is deprecated: use navigation-msg:PolyArray instead.")))

(cl:ensure-generic-function 'polygons-val :lambda-list '(m))
(cl:defmethod polygons-val ((m <PolyArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-msg:polygons-val is deprecated.  Use navigation-msg:polygons instead.")
  (polygons m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PolyArray>) ostream)
  "Serializes a message object of type '<PolyArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'polygons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'polygons))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PolyArray>) istream)
  "Deserializes a message object of type '<PolyArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'polygons) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'polygons)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'navigation-msg:PointArray))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PolyArray>)))
  "Returns string type for a message object of type '<PolyArray>"
  "navigation/PolyArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PolyArray)))
  "Returns string type for a message object of type 'PolyArray"
  "navigation/PolyArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PolyArray>)))
  "Returns md5sum for a message object of type '<PolyArray>"
  "49d5f4357e9c5ac38b54c91ae2b7c9f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PolyArray)))
  "Returns md5sum for a message object of type 'PolyArray"
  "49d5f4357e9c5ac38b54c91ae2b7c9f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PolyArray>)))
  "Returns full string definition for message of type '<PolyArray>"
  (cl:format cl:nil "navigation/PointArray[] polygons~%~%================================================================================~%MSG: navigation/PointArray~%navigation/Point_xy[] points~%  ~%================================================================================~%MSG: navigation/Point_xy~%float32[] point~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PolyArray)))
  "Returns full string definition for message of type 'PolyArray"
  (cl:format cl:nil "navigation/PointArray[] polygons~%~%================================================================================~%MSG: navigation/PointArray~%navigation/Point_xy[] points~%  ~%================================================================================~%MSG: navigation/Point_xy~%float32[] point~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PolyArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'polygons) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PolyArray>))
  "Converts a ROS message object to a list"
  (cl:list 'PolyArray
    (cl:cons ':polygons (polygons msg))
))
