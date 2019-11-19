;; Auto-generated. Do not edit!


(when (boundp 'navigation::PointArray)
  (if (not (find-package "NAVIGATION"))
    (make-package "NAVIGATION"))
  (shadow 'PointArray (find-package "NAVIGATION")))
(unless (find-package "NAVIGATION::POINTARRAY")
  (make-package "NAVIGATION::POINTARRAY"))

(in-package "ROS")
;;//! \htmlinclude PointArray.msg.html


(defclass navigation::PointArray
  :super ros::object
  :slots (_points ))

(defmethod navigation::PointArray
  (:init
   (&key
    ((:points __points) (let (r) (dotimes (i 0) (push (instance navigation::Point_xy :init) r)) r))
    )
   (send-super :init)
   (setq _points __points)
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; navigation/Point_xy[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; navigation/Point_xy[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; navigation/Point_xy[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance navigation::Point_xy :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get navigation::PointArray :md5sum-) "9c2c9f601cd3038d8ee0a839b54636d2")
(setf (get navigation::PointArray :datatype-) "navigation/PointArray")
(setf (get navigation::PointArray :definition-)
      "navigation/Point_xy[] points
  
================================================================================
MSG: navigation/Point_xy
float32[] point
")



(provide :navigation/PointArray "9c2c9f601cd3038d8ee0a839b54636d2")


