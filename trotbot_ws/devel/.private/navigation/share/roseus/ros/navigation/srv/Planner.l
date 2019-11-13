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
    ((:path __path) (instance navigation::PointArray :init))
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
    ;; navigation/PointArray _path
    (send _path :serialization-length)
    ;; bool _ack
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; navigation/PointArray _path
       (send _path :serialize s)
     ;; bool _ack
       (if _ack (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; navigation/PointArray _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;; bool _ack
     (setq _ack (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass navigation::Planner
  :super ros::object
  :slots ())

(setf (get navigation::Planner :md5sum-) "e099e7e7ce9aa3ba23a5f56a66d6f6e9")
(setf (get navigation::Planner :datatype-) "navigation/Planner")
(setf (get navigation::Planner :request) navigation::PlannerRequest)
(setf (get navigation::Planner :response) navigation::PlannerResponse)

(defmethod navigation::PlannerRequest
  (:response () (instance navigation::PlannerResponse :init)))

(setf (get navigation::PlannerRequest :md5sum-) "e099e7e7ce9aa3ba23a5f56a66d6f6e9")
(setf (get navigation::PlannerRequest :datatype-) "navigation/PlannerRequest")
(setf (get navigation::PlannerRequest :definition-)
      "std_msgs/Float32[] start
std_msgs/Float32[] goal

================================================================================
MSG: std_msgs/Float32
---
navigation/PointArray path
bool ack

================================================================================
MSG: navigation/PointArray
std_msgs/Header header
geometry_msgs/Point[] points
  
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get navigation::PlannerResponse :md5sum-) "e099e7e7ce9aa3ba23a5f56a66d6f6e9")
(setf (get navigation::PlannerResponse :datatype-) "navigation/PlannerResponse")
(setf (get navigation::PlannerResponse :definition-)
      "std_msgs/Float32[] start
std_msgs/Float32[] goal

================================================================================
MSG: std_msgs/Float32
---
navigation/PointArray path
bool ack

================================================================================
MSG: navigation/PointArray
std_msgs/Header header
geometry_msgs/Point[] points
  
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :navigation/Planner "e099e7e7ce9aa3ba23a5f56a66d6f6e9")


