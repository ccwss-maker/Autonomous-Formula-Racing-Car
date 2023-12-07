; Auto-generated. Do not edit!


(cl:in-package box_ros_msgs-msg)


;//! \htmlinclude Image.msg.html

(cl:defclass <Image> (roslisp-msg-protocol:ros-message)
  ((height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (encoding
    :reader encoding
    :initarg :encoding
    :type cl:string
    :initform "")
   (is_bigendian
    :reader is_bigendian
    :initarg :is_bigendian
    :type cl:fixnum
    :initform 0)
   (step
    :reader step
    :initarg :step
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Image (<Image>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Image>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Image)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name box_ros_msgs-msg:<Image> is deprecated: use box_ros_msgs-msg:Image instead.")))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <Image>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader box_ros_msgs-msg:height-val is deprecated.  Use box_ros_msgs-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <Image>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader box_ros_msgs-msg:width-val is deprecated.  Use box_ros_msgs-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'encoding-val :lambda-list '(m))
(cl:defmethod encoding-val ((m <Image>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader box_ros_msgs-msg:encoding-val is deprecated.  Use box_ros_msgs-msg:encoding instead.")
  (encoding m))

(cl:ensure-generic-function 'is_bigendian-val :lambda-list '(m))
(cl:defmethod is_bigendian-val ((m <Image>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader box_ros_msgs-msg:is_bigendian-val is deprecated.  Use box_ros_msgs-msg:is_bigendian instead.")
  (is_bigendian m))

(cl:ensure-generic-function 'step-val :lambda-list '(m))
(cl:defmethod step-val ((m <Image>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader box_ros_msgs-msg:step-val is deprecated.  Use box_ros_msgs-msg:step instead.")
  (step m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Image>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader box_ros_msgs-msg:data-val is deprecated.  Use box_ros_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Image>) ostream)
  "Serializes a message object of type '<Image>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'encoding))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'encoding))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'is_bigendian)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'step)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'step)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'step)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'step)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Image>) istream)
  "Deserializes a message object of type '<Image>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'encoding) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'encoding) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'is_bigendian)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'step)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'step)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'step)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'step)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Image>)))
  "Returns string type for a message object of type '<Image>"
  "box_ros_msgs/Image")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Image)))
  "Returns string type for a message object of type 'Image"
  "box_ros_msgs/Image")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Image>)))
  "Returns md5sum for a message object of type '<Image>"
  "585ac495236028e67bb0be633f129591")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Image)))
  "Returns md5sum for a message object of type 'Image"
  "585ac495236028e67bb0be633f129591")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Image>)))
  "Returns full string definition for message of type '<Image>"
  (cl:format cl:nil "uint32 height~%uint32 width~%string encoding~%uint8 is_bigendian~%uint32 step~%uint8[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Image)))
  "Returns full string definition for message of type 'Image"
  (cl:format cl:nil "uint32 height~%uint32 width~%string encoding~%uint8 is_bigendian~%uint32 step~%uint8[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Image>))
  (cl:+ 0
     4
     4
     4 (cl:length (cl:slot-value msg 'encoding))
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Image>))
  "Converts a ROS message object to a list"
  (cl:list 'Image
    (cl:cons ':height (height msg))
    (cl:cons ':width (width msg))
    (cl:cons ':encoding (encoding msg))
    (cl:cons ':is_bigendian (is_bigendian msg))
    (cl:cons ':step (step msg))
    (cl:cons ':data (data msg))
))
