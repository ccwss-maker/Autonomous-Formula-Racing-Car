
(cl:in-package :asdf)

(defsystem "kbd_ros_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "kbd" :depends-on ("_package_kbd"))
    (:file "_package_kbd" :depends-on ("_package"))
  ))