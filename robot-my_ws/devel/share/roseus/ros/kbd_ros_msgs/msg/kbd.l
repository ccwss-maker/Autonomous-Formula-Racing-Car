;; Auto-generated. Do not edit!


(when (boundp 'kbd_ros_msgs::kbd)
  (if (not (find-package "KBD_ROS_MSGS"))
    (make-package "KBD_ROS_MSGS"))
  (shadow 'kbd (find-package "KBD_ROS_MSGS")))
(unless (find-package "KBD_ROS_MSGS::KBD")
  (make-package "KBD_ROS_MSGS::KBD"))

(in-package "ROS")
;;//! \htmlinclude kbd.msg.html


(defclass kbd_ros_msgs::kbd
  :super ros::object
  :slots (_w _a _s _d ))

(defmethod kbd_ros_msgs::kbd
  (:init
   (&key
    ((:w __w) 0)
    ((:a __a) 0)
    ((:s __s) 0)
    ((:d __d) 0)
    )
   (send-super :init)
   (setq _w (round __w))
   (setq _a (round __a))
   (setq _s (round __s))
   (setq _d (round __d))
   self)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:d
   (&optional __d)
   (if __d (setq _d __d)) _d)
  (:serialization-length
   ()
   (+
    ;; int16 _w
    2
    ;; int16 _a
    2
    ;; int16 _s
    2
    ;; int16 _d
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _w
       (write-word _w s)
     ;; int16 _a
       (write-word _a s)
     ;; int16 _s
       (write-word _s s)
     ;; int16 _d
       (write-word _d s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _w
     (setq _w (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _a
     (setq _a (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _s
     (setq _s (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _d
     (setq _d (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get kbd_ros_msgs::kbd :md5sum-) "2a1716104afcb5f80164140a20101628")
(setf (get kbd_ros_msgs::kbd :datatype-) "kbd_ros_msgs/kbd")
(setf (get kbd_ros_msgs::kbd :definition-)
      "int16 w
int16 a
int16 s
int16 d

")



(provide :kbd_ros_msgs/kbd "2a1716104afcb5f80164140a20101628")


