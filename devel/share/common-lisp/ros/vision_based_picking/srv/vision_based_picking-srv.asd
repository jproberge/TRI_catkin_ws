
(cl:in-package :asdf)

(defsystem "vision_based_picking-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Acquire" :depends-on ("_package_Acquire"))
    (:file "_package_Acquire" :depends-on ("_package"))
    (:file "Calibrate" :depends-on ("_package_Calibrate"))
    (:file "_package_Calibrate" :depends-on ("_package"))
  ))