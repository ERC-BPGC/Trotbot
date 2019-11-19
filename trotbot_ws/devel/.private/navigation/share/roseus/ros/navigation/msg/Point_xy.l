;; Auto-generated. Do not edit!


(when (boundp 'navigation::Point_xy)
  (if (not (find-package "NAVIGATION"))
    (make-package "NAVIGATION"))
  (shadow 'Point_xy (find-package "NAVIGATION")))
(unless (find-package "NAVIGATION::POINT_XY")
  (make-package "NAVIGATION::POINT_XY"))

(in-package "ROS")
;;//! \htmlinclude Point_xy.msg.html


(defclass navigation::Point_xy
  :super ros::object
  :slots (_point ))

(defmethod navigation::Point_xy
  (:init
   (&key
    ((:point __point) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _point __point)
   self)
  (:point
   (&optional __point)
   (if __point (setq _point __point)) _point)
  (:serialization-length
   ()
   (+
    ;; float32[] _point
    (* 4    (length _point)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _point
     (write-long (length _point) s)
     (dotimes (i (length _point))
       (sys::poke (elt _point i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _point
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _point (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _point i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get navigation::Point_xy :md5sum-) "318ea976b093c91a3f95a8e83351f8ad")
(setf (get navigation::Point_xy :datatype-) "navigation/Point_xy")
(setf (get navigation::Point_xy :definition-)
      "float32[] point
")



(provide :navigation/Point_xy "318ea976b093c91a3f95a8e83351f8ad")


