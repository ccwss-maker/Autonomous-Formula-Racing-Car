
(cl:in-package :asdf)

(defsystem "box_ros_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BoundingBox" :depends-on ("_package_BoundingBox"))
    (:file "_package_BoundingBox" :depends-on ("_package"))
    (:file "BoundingBoxes" :depends-on ("_package_BoundingBoxes"))
    (:file "_package_BoundingBoxes" :depends-on ("_package"))
    (:file "Image" :depends-on ("_package_Image"))
    (:file "_package_Image" :depends-on ("_package"))
    (:file "Odom" :depends-on ("_package_Odom"))
    (:file "_package_Odom" :depends-on ("_package"))
  ))