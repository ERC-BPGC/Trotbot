
(cl:in-package :asdf)

(defsystem "navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PointArray" :depends-on ("_package_PointArray"))
    (:file "_package_PointArray" :depends-on ("_package"))
    (:file "PointArray" :depends-on ("_package_PointArray"))
    (:file "_package_PointArray" :depends-on ("_package"))
    (:file "Point_xy" :depends-on ("_package_Point_xy"))
    (:file "_package_Point_xy" :depends-on ("_package"))
    (:file "Point_xy" :depends-on ("_package_Point_xy"))
    (:file "_package_Point_xy" :depends-on ("_package"))
    (:file "PolyArray" :depends-on ("_package_PolyArray"))
    (:file "_package_PolyArray" :depends-on ("_package"))
    (:file "PolyArray" :depends-on ("_package_PolyArray"))
    (:file "_package_PolyArray" :depends-on ("_package"))
  ))