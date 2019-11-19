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





(defclass navigation::PlannerRequest
  :super ros::object
  :slots (_start _goal _obstacle_list ))

(defmethod navigation::PlannerRequest
  (:init
   (&key
    ((:start __start) (instance navigation::Point_xy :init))
    ((:goal __goal) (instance navigation::Point_xy :init))
    ((:obstacle_list __obstacle_list) (instance navigation::PolyArray :init))
    )
   (send-super :init)
   (setq _start __start)
   (setq _goal __goal)
   (setq _obstacle_list __obstacle_list)
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
  (:obstacle_list
   (&rest __obstacle_list)
   (if (keywordp (car __obstacle_list))
       (send* _obstacle_list __obstacle_list)
     (progn
       (if __obstacle_list (setq _obstacle_list (car __obstacle_list)))
       _obstacle_list)))
  (:serialization-length
   ()
   (+
    ;; navigation/Point_xy _start
    (send _start :serialization-length)
    ;; navigation/Point_xy _goal
    (send _goal :serialization-length)
    ;; navigation/PolyArray _obstacle_list
    (send _obstacle_list :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; navigation/Point_xy _start
       (send _start :serialize s)
     ;; navigation/Point_xy _goal
       (send _goal :serialize s)
     ;; navigation/PolyArray _obstacle_list
       (send _obstacle_list :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; navigation/Point_xy _start
     (send _start :deserialize buf ptr-) (incf ptr- (send _start :serialization-length))
   ;; navigation/Point_xy _goal
     (send _goal :deserialize buf ptr-) (incf ptr- (send _goal :serialization-length))
   ;; navigation/PolyArray _obstacle_list
     (send _obstacle_list :deserialize buf ptr-) (incf ptr- (send _obstacle_list :serialization-length))
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

(setf (get navigation::Planner :md5sum-) "0f8ba09d5a21e9916f0e3bf633247872")
(setf (get navigation::Planner :datatype-) "navigation/Planner")
(setf (get navigation::Planner :request) navigation::PlannerRequest)
(setf (get navigation::Planner :response) navigation::PlannerResponse)

(defmethod navigation::PlannerRequest
  (:response () (instance navigation::PlannerResponse :init)))

(setf (get navigation::PlannerRequest :md5sum-) "0f8ba09d5a21e9916f0e3bf633247872")
(setf (get navigation::PlannerRequest :datatype-) "navigation/PlannerRequest")
(setf (get navigation::PlannerRequest :definition-)
      "navigation/Point_xy start
navigation/Point_xy goal
navigation/PolyArray obstacle_list

================================================================================
MSG: navigation/Point_xy
float32[] point
================================================================================
MSG: navigation/PolyArray
navigation/PointArray[] polygons

================================================================================
MSG: navigation/PointArray
navigation/Point_xy[] points
---
navigation/PointArray path
bool ack

================================================================================
MSG: navigation/PointArray
navigation/Point_xy[] points
  
================================================================================
MSG: navigation/Point_xy
")

(setf (get navigation::PlannerResponse :md5sum-) "0f8ba09d5a21e9916f0e3bf633247872")
(setf (get navigation::PlannerResponse :datatype-) "navigation/PlannerResponse")
(setf (get navigation::PlannerResponse :definition-)
      "navigation/Point_xy start
navigation/Point_xy goal
navigation/PolyArray obstacle_list

================================================================================
MSG: navigation/Point_xy
float32[] point
================================================================================
MSG: navigation/PolyArray
navigation/PointArray[] polygons

================================================================================
MSG: navigation/PointArray
navigation/Point_xy[] points
---
navigation/PointArray path
bool ack

================================================================================
MSG: navigation/PointArray
navigation/Point_xy[] points
  
================================================================================
MSG: navigation/Point_xy
")



(provide :navigation/Planner "0f8ba09d5a21e9916f0e3bf633247872")


