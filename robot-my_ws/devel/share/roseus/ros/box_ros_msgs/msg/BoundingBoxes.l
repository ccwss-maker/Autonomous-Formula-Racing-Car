;; Auto-generated. Do not edit!


(when (boundp 'box_ros_msgs::BoundingBoxes)
  (if (not (find-package "BOX_ROS_MSGS"))
    (make-package "BOX_ROS_MSGS"))
  (shadow 'BoundingBoxes (find-package "BOX_ROS_MSGS")))
(unless (find-package "BOX_ROS_MSGS::BOUNDINGBOXES")
  (make-package "BOX_ROS_MSGS::BOUNDINGBOXES"))

(in-package "ROS")
;;//! \htmlinclude BoundingBoxes.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass box_ros_msgs::BoundingBoxes
  :super ros::object
  :slots (_header _img _odom _count _bounding_boxes ))

(defmethod box_ros_msgs::BoundingBoxes
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:img __img) (instance box_ros_msgs::Image :init))
    ((:odom __odom) (instance box_ros_msgs::Odom :init))
    ((:count __count) 0)
    ((:bounding_boxes __bounding_boxes) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _img __img)
   (setq _odom __odom)
   (setq _count (round __count))
   (setq _bounding_boxes __bounding_boxes)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:img
   (&rest __img)
   (if (keywordp (car __img))
       (send* _img __img)
     (progn
       (if __img (setq _img (car __img)))
       _img)))
  (:odom
   (&rest __odom)
   (if (keywordp (car __odom))
       (send* _odom __odom)
     (progn
       (if __odom (setq _odom (car __odom)))
       _odom)))
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:bounding_boxes
   (&rest __bounding_boxes)
   (if (keywordp (car __bounding_boxes))
       (send* _bounding_boxes __bounding_boxes)
     (progn
       (if __bounding_boxes (setq _bounding_boxes (car __bounding_boxes)))
       _bounding_boxes)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; box_ros_msgs/Image _img
    (send _img :serialization-length)
    ;; box_ros_msgs/Odom _odom
    (send _odom :serialization-length)
    ;; int8 _count
    1
    ;; box_ros_msgs/BoundingBox[] _bounding_boxes
    (apply #'+ (send-all _bounding_boxes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; box_ros_msgs/Image _img
       (send _img :serialize s)
     ;; box_ros_msgs/Odom _odom
       (send _odom :serialize s)
     ;; int8 _count
       (write-byte _count s)
     ;; box_ros_msgs/BoundingBox[] _bounding_boxes
     (write-long (length _bounding_boxes) s)
     (dolist (elem _bounding_boxes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; box_ros_msgs/Image _img
     (send _img :deserialize buf ptr-) (incf ptr- (send _img :serialization-length))
   ;; box_ros_msgs/Odom _odom
     (send _odom :deserialize buf ptr-) (incf ptr- (send _odom :serialization-length))
   ;; int8 _count
     (setq _count (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _count 127) (setq _count (- _count 256)))
   ;; box_ros_msgs/BoundingBox[] _bounding_boxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bounding_boxes (let (r) (dotimes (i n) (push (instance box_ros_msgs::BoundingBox :init) r)) r))
     (dolist (elem- _bounding_boxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get box_ros_msgs::BoundingBoxes :md5sum-) "7e7ec4dfc26f8bdc8856533651e3a989")
(setf (get box_ros_msgs::BoundingBoxes :datatype-) "box_ros_msgs/BoundingBoxes")
(setf (get box_ros_msgs::BoundingBoxes :definition-)
      "Header header
Image img
Odom odom
int8 count
BoundingBox[] bounding_boxes

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: box_ros_msgs/Image
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data

================================================================================
MSG: box_ros_msgs/Odom
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: box_ros_msgs/BoundingBox
float64 probability
int64 xmin
int64 ymin
int64 xmax
int64 ymax
int16 id
string Class

")



(provide :box_ros_msgs/BoundingBoxes "7e7ec4dfc26f8bdc8856533651e3a989")


