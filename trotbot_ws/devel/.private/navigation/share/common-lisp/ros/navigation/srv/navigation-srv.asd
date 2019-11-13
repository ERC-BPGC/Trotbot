
(cl:in-package :asdf)

(defsystem "navigation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :navigation-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Planner" :depends-on ("_package_Planner"))
    (:file "_package_Planner" :depends-on ("_package"))
  ))