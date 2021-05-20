; Auto-generated. Do not edit!


(cl:in-package crazyflie_scripts-msg)


;//! \htmlinclude camera_msg.msg.html

(cl:defclass <camera_msg> (roslisp-msg-protocol:ros-message)
  ((ext_x
    :reader ext_x
    :initarg :ext_x
    :type cl:integer
    :initform 0)
   (ext_y
    :reader ext_y
    :initarg :ext_y
    :type cl:integer
    :initform 0)
   (t_x
    :reader t_x
    :initarg :t_x
    :type cl:integer
    :initform 0)
   (t_y
    :reader t_y
    :initarg :t_y
    :type cl:integer
    :initform 0)
   (x_dir_tt
    :reader x_dir_tt
    :initarg :x_dir_tt
    :type cl:integer
    :initform 0)
   (y_dir_tt
    :reader y_dir_tt
    :initarg :y_dir_tt
    :type cl:integer
    :initform 0))
)

(cl:defclass camera_msg (<camera_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <camera_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'camera_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crazyflie_scripts-msg:<camera_msg> is deprecated: use crazyflie_scripts-msg:camera_msg instead.")))

(cl:ensure-generic-function 'ext_x-val :lambda-list '(m))
(cl:defmethod ext_x-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:ext_x-val is deprecated.  Use crazyflie_scripts-msg:ext_x instead.")
  (ext_x m))

(cl:ensure-generic-function 'ext_y-val :lambda-list '(m))
(cl:defmethod ext_y-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:ext_y-val is deprecated.  Use crazyflie_scripts-msg:ext_y instead.")
  (ext_y m))

(cl:ensure-generic-function 't_x-val :lambda-list '(m))
(cl:defmethod t_x-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:t_x-val is deprecated.  Use crazyflie_scripts-msg:t_x instead.")
  (t_x m))

(cl:ensure-generic-function 't_y-val :lambda-list '(m))
(cl:defmethod t_y-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:t_y-val is deprecated.  Use crazyflie_scripts-msg:t_y instead.")
  (t_y m))

(cl:ensure-generic-function 'x_dir_tt-val :lambda-list '(m))
(cl:defmethod x_dir_tt-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:x_dir_tt-val is deprecated.  Use crazyflie_scripts-msg:x_dir_tt instead.")
  (x_dir_tt m))

(cl:ensure-generic-function 'y_dir_tt-val :lambda-list '(m))
(cl:defmethod y_dir_tt-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:y_dir_tt-val is deprecated.  Use crazyflie_scripts-msg:y_dir_tt instead.")
  (y_dir_tt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <camera_msg>) ostream)
  "Serializes a message object of type '<camera_msg>"
  (cl:let* ((signed (cl:slot-value msg 'ext_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ext_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 't_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 't_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x_dir_tt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y_dir_tt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <camera_msg>) istream)
  "Deserializes a message object of type '<camera_msg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ext_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ext_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 't_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 't_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x_dir_tt) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y_dir_tt) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<camera_msg>)))
  "Returns string type for a message object of type '<camera_msg>"
  "crazyflie_scripts/camera_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'camera_msg)))
  "Returns string type for a message object of type 'camera_msg"
  "crazyflie_scripts/camera_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<camera_msg>)))
  "Returns md5sum for a message object of type '<camera_msg>"
  "0528b01776a9faff38d86476b22e6f80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'camera_msg)))
  "Returns md5sum for a message object of type 'camera_msg"
  "0528b01776a9faff38d86476b22e6f80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<camera_msg>)))
  "Returns full string definition for message of type '<camera_msg>"
  (cl:format cl:nil "int32 ext_x~%int32 ext_y~%int32 t_x~%int32 t_y~%int32 x_dir_tt~%int32 y_dir_tt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'camera_msg)))
  "Returns full string definition for message of type 'camera_msg"
  (cl:format cl:nil "int32 ext_x~%int32 ext_y~%int32 t_x~%int32 t_y~%int32 x_dir_tt~%int32 y_dir_tt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <camera_msg>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <camera_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'camera_msg
    (cl:cons ':ext_x (ext_x msg))
    (cl:cons ':ext_y (ext_y msg))
    (cl:cons ':t_x (t_x msg))
    (cl:cons ':t_y (t_y msg))
    (cl:cons ':x_dir_tt (x_dir_tt msg))
    (cl:cons ':y_dir_tt (y_dir_tt msg))
))
