;; Auto-generated. Do not edit!


(when (boundp 'navigation::Planner)
  (if (not (find-package "NAVIGATION"))
    (make-package "NAVIGATION"))
  (shadow 'Planner (find-package "NAVIGATION")))
(unless (find-package "NAVIGATION::PLANNER")
  (make-package "NAVIGATION::PLANNER"))
(unless (find-package "NAVIGATION::PLANNERREQUEST")
  (make-package "NAVIGATION::PLANNERREQUEST"))
(unless (find-package "NAVIGATION::PLANNERRESPONSE")
  (make-package "NAVIGATION::PLANNERRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass navigation::PlannerRequest
  :super ros::object
  :slots (_start _goal ))

(defmethod navigation::PlannerRequest
  (:init
   (&key
    ((:start __start) (let (r) (dotimes (i 0) (push (instance std_msgs::Float32 :init) r)) r))
    ((:goal __goal) (let (r) (dotimes (i 0) (push (instance std_msgs::Float32 :init) r)) r))
    )
   (send-super :init)
   (setq _start __start)
   (setq _goal __goal)
   self)
  (:start
   (&rest __start)
   (if (keywordp (car __start))
       (send* _start __start)
     (progn
       (if __start (setq _start (car __start)))
       _start)))
  (:goal
   (&rest __goal)
   (if (keywordp (car __goal))
       (send* _goal __goal)
     (progn
       (if __goal (setq _goal (car __goal)))
       _goal)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float32[] _start
    (apply #'+ (send-all _start :serialization-length)) 4
    ;; std_msgs/Float32[] _goal
    (apply #'+ (send-all _goal :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float32[] _start
     (write-long (length _start) s)
     (dolist (elem _start)
       (send elem :serialize s)
       )
     ;; std_msgs/Float32[] _goal
     (write-long (length _goal) s)
     (dolist (elem _goal)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float32[] _start
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _start (let (r) (dotimes (i n) (push (instance std_msgs::Float32 :init) r)) r))
     (dolist (elem- _start)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/Float32[] _goal
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _goal (let (r) (dotimes (i n) (push (instance std_msgs::Float32 :init) r)) r))
     (dolist (elem- _goal)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass navigation::PlannerResponse
  :super ros::object
  :slots (_path _ack ))

(defmethod navigation::PlannerResponse
  (:init
   (&key
    ((:path __path) (instance std_msgs::Float32MultiArray :init))
    ((:ack __ack) nil)
    )
   (send-super :init)
   (setq _path __path)
   (setq _ack __ack)
   self)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:ack
   (&optional __ack)
   (if __ack (setq _ack __ack)) _ack)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float32MultiArray _path
    (send _path :serialization-length)
    ;; bool _ack
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float32MultiArray _path
       (send _path :serialize s)
     ;; bool _ack
       (if _ack (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float32MultiArray _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;; bool _ack
     (setq _ack (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass navigation::Planner
  :super ros::object
  :slots ())

(setf (get navigation::Planner :md5sum-) "59d496e471e2ce1a2e0dd836fb1cdfb4")
(setf (get navigation::Planner :datatype-) "navigation/Planner")
(setf (get navigation::Planner :request) navigation::PlannerRequest)
(setf (get navigation::Planner :response) navigation::PlannerResponse)

(defmethod navigation::PlannerRequest
  (:response () (instance navigation::PlannerResponse :init)))

(setf (get navigation::PlannerRequest :md5sum-) "59d496e471e2ce1a2e0dd836fb1cdfb4")
(setf (get navigation::PlannerRequest :datatype-) "navigation/PlannerRequest")
(setf (get navigation::PlannerRequest :definition-)
      "std_msgs/Float32[] start
std_msgs/Float32[] goal

================================================================================
MSG: std_msgs/Float32
---
std_msgs/Float32MultiArray path
bool ack

================================================================================
MSG: std_msgs/Float32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
")

(setf (get navigation::PlannerResponse :md5sum-) "59d496e471e2ce1a2e0dd836fb1cdfb4")
(setf (get navigation::PlannerResponse :datatype-) "navigation/PlannerResponse")
(setf (get navigation::PlannerResponse :definition-)
      "std_msgs/Float32[] start
std_msgs/Float32[] goal

================================================================================
MSG: std_msgs/Float32
---
std_msgs/Float32MultiArray path
bool ack

================================================================================
MSG: std_msgs/Float32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
")



(provide :navigation/Planner "59d496e471e2ce1a2e0dd836fb1cdfb4")


