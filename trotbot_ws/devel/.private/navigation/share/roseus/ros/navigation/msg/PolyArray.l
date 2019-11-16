;; Auto-generated. Do not edit!


(when (boundp 'navigation::PolyArray)
  (if (not (find-package "NAVIGATION"))
    (make-package "NAVIGATION"))
  (shadow 'PolyArray (find-package "NAVIGATION")))
(unless (find-package "NAVIGATION::POLYARRAY")
  (make-package "NAVIGATION::POLYARRAY"))

(in-package "ROS")
;;//! \htmlinclude PolyArray.msg.html


(defclass navigation::PolyArray
  :super ros::object
  :slots (_polygons ))

(defmethod navigation::PolyArray
  (:init
   (&key
    ((:polygons __polygons) (let (r) (dotimes (i 0) (push (instance navigation::PointArray :init) r)) r))
    )
   (send-super :init)
   (setq _polygons __polygons)
   self)
  (:polygons
   (&rest __polygons)
   (if (keywordp (car __polygons))
       (send* _polygons __polygons)
     (progn
       (if __polygons (setq _polygons (car __polygons)))
       _polygons)))
  (:serialization-length
   ()
   (+
    ;; navigation/PointArray[] _polygons
    (apply #'+ (send-all _polygons :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; navigation/PointArray[] _polygons
     (write-long (length _polygons) s)
     (dolist (elem _polygons)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; navigation/PointArray[] _polygons
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _polygons (let (r) (dotimes (i n) (push (instance navigation::PointArray :init) r)) r))
     (dolist (elem- _polygons)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get navigation::PolyArray :md5sum-) "49d5f4357e9c5ac38b54c91ae2b7c9f3")
(setf (get navigation::PolyArray :datatype-) "navigation/PolyArray")
(setf (get navigation::PolyArray :definition-)
      "navigation/PointArray[] polygons

================================================================================
MSG: navigation/PointArray
navigation/Point_xy[] points
  
================================================================================
MSG: navigation/Point_xy
float32[] point
")



(provide :navigation/PolyArray "49d5f4357e9c5ac38b54c91ae2b7c9f3")


