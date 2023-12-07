;; Auto-generated. Do not edit!


(when (boundp 'box_ros_msgs::Image)
  (if (not (find-package "BOX_ROS_MSGS"))
    (make-package "BOX_ROS_MSGS"))
  (shadow 'Image (find-package "BOX_ROS_MSGS")))
(unless (find-package "BOX_ROS_MSGS::IMAGE")
  (make-package "BOX_ROS_MSGS::IMAGE"))

(in-package "ROS")
;;//! \htmlinclude Image.msg.html


(defclass box_ros_msgs::Image
  :super ros::object
  :slots (_height _width _encoding _is_bigendian _step _data ))

(defmethod box_ros_msgs::Image
  (:init
   (&key
    ((:height __height) 0)
    ((:width __width) 0)
    ((:encoding __encoding) "")
    ((:is_bigendian __is_bigendian) 0)
    ((:step __step) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _height (round __height))
   (setq _width (round __width))
   (setq _encoding (string __encoding))
   (setq _is_bigendian (round __is_bigendian))
   (setq _step (round __step))
   (setq _data __data)
   self)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:encoding
   (&optional __encoding)
   (if __encoding (setq _encoding __encoding)) _encoding)
  (:is_bigendian
   (&optional __is_bigendian)
   (if __is_bigendian (setq _is_bigendian __is_bigendian)) _is_bigendian)
  (:step
   (&optional __step)
   (if __step (setq _step __step)) _step)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; uint32 _height
    4
    ;; uint32 _width
    4
    ;; string _encoding
    4 (length _encoding)
    ;; uint8 _is_bigendian
    1
    ;; uint32 _step
    4
    ;; uint8[] _data
    (* 1    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _height
       (write-long _height s)
     ;; uint32 _width
       (write-long _width s)
     ;; string _encoding
       (write-long (length _encoding) s) (princ _encoding s)
     ;; uint8 _is_bigendian
       (write-byte _is_bigendian s)
     ;; uint32 _step
       (write-long _step s)
     ;; uint8[] _data
     (write-long (length _data) s)
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _encoding
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _encoding (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _is_bigendian
     (setq _is_bigendian (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _step
     (setq _step (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[] _data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _data (make-array n :element-type :char))
     (replace _data buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get box_ros_msgs::Image :md5sum-) "585ac495236028e67bb0be633f129591")
(setf (get box_ros_msgs::Image :datatype-) "box_ros_msgs/Image")
(setf (get box_ros_msgs::Image :definition-)
      "uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data

")



(provide :box_ros_msgs/Image "585ac495236028e67bb0be633f129591")


