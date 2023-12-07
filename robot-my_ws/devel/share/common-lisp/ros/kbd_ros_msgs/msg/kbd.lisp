; Auto-generated. Do not edit!


(cl:in-package kbd_ros_msgs-msg)


;//! \htmlinclude kbd.msg.html

(cl:defclass <kbd> (roslisp-msg-protocol:ros-message)
  ((w
    :reader w
    :initarg :w
    :type cl:fixnum
    :initform 0)
   (a
    :reader a
    :initarg :a
    :type cl:fixnum
    :initform 0)
   (s
    :reader s
    :initarg :s
    :type cl:fixnum
    :initform 0)
   (d
    :reader d
    :initarg :d
    :type cl:fixnum
    :initform 0))
)

(cl:defclass kbd (<kbd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <kbd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'kbd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kbd_ros_msgs-msg:<kbd> is deprecated: use kbd_ros_msgs-msg:kbd instead.")))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <kbd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kbd_ros_msgs-msg:w-val is deprecated.  Use kbd_ros_msgs-msg:w instead.")
  (w m))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <kbd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kbd_ros_msgs-msg:a-val is deprecated.  Use kbd_ros_msgs-msg:a instead.")
  (a m))

(cl:ensure-generic-function 's-val :lambda-list '(m))
(cl:defmethod s-val ((m <kbd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kbd_ros_msgs-msg:s-val is deprecated.  Use kbd_ros_msgs-msg:s instead.")
  (s m))

(cl:ensure-generic-function 'd-val :lambda-list '(m))
(cl:defmethod d-val ((m <kbd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kbd_ros_msgs-msg:d-val is deprecated.  Use kbd_ros_msgs-msg:d instead.")
  (d m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <kbd>) ostream)
  "Serializes a message object of type '<kbd>"
  (cl:let* ((signed (cl:slot-value msg 'w)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 's)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <kbd>) istream)
  "Deserializes a message object of type '<kbd>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'w) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 's) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'd) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<kbd>)))
  "Returns string type for a message object of type '<kbd>"
  "kbd_ros_msgs/kbd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'kbd)))
  "Returns string type for a message object of type 'kbd"
  "kbd_ros_msgs/kbd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<kbd>)))
  "Returns md5sum for a message object of type '<kbd>"
  "2a1716104afcb5f80164140a20101628")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'kbd)))
  "Returns md5sum for a message object of type 'kbd"
  "2a1716104afcb5f80164140a20101628")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<kbd>)))
  "Returns full string definition for message of type '<kbd>"
  (cl:format cl:nil "int16 w~%int16 a~%int16 s~%int16 d~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'kbd)))
  "Returns full string definition for message of type 'kbd"
  (cl:format cl:nil "int16 w~%int16 a~%int16 s~%int16 d~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <kbd>))
  (cl:+ 0
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <kbd>))
  "Converts a ROS message object to a list"
  (cl:list 'kbd
    (cl:cons ':w (w msg))
    (cl:cons ':a (a msg))
    (cl:cons ':s (s msg))
    (cl:cons ':d (d msg))
))
