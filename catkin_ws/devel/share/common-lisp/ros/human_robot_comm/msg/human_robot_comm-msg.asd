
(cl:in-package :asdf)

(defsystem "human_robot_comm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "floatlist" :depends-on ("_package_floatlist"))
    (:file "_package_floatlist" :depends-on ("_package"))
  ))