; Auto-generated. Do not edit!


(cl:in-package crazyflie_scripts-msg)


;//! \htmlinclude camera_msg.msg.html

(cl:defclass <camera_msg> (roslisp-msg-protocol:ros-message)
  ((ext_x_cam
    :reader ext_x_cam
    :initarg :ext_x_cam
    :type cl:float
    :initform 0.0)
   (ext_y_cam
    :reader ext_y_cam
    :initarg :ext_y_cam
    :type cl:float
    :initform 0.0)
   (track_x
    :reader track_x
    :initarg :track_x
    :type cl:float
    :initform 0.0)
   (track_y
    :reader track_y
    :initarg :track_y
    :type cl:float
    :initform 0.0)
   (track_z
    :reader track_z
    :initarg :track_z
    :type cl:float
    :initform 0.0)
   (x_dir_tt
    :reader x_dir_tt
    :initarg :x_dir_tt
    :type cl:float
    :initform 0.0)
   (y_dir_tt
    :reader y_dir_tt
    :initarg :y_dir_tt
    :type cl:float
    :initform 0.0)
   (tg_yaw_t
    :reader tg_yaw_t
    :initarg :tg_yaw_t
    :type cl:float
    :initform 0.0))
)

(cl:defclass camera_msg (<camera_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <camera_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'camera_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crazyflie_scripts-msg:<camera_msg> is deprecated: use crazyflie_scripts-msg:camera_msg instead.")))

(cl:ensure-generic-function 'ext_x_cam-val :lambda-list '(m))
(cl:defmethod ext_x_cam-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:ext_x_cam-val is deprecated.  Use crazyflie_scripts-msg:ext_x_cam instead.")
  (ext_x_cam m))

(cl:ensure-generic-function 'ext_y_cam-val :lambda-list '(m))
(cl:defmethod ext_y_cam-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:ext_y_cam-val is deprecated.  Use crazyflie_scripts-msg:ext_y_cam instead.")
  (ext_y_cam m))

(cl:ensure-generic-function 'track_x-val :lambda-list '(m))
(cl:defmethod track_x-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:track_x-val is deprecated.  Use crazyflie_scripts-msg:track_x instead.")
  (track_x m))

(cl:ensure-generic-function 'track_y-val :lambda-list '(m))
(cl:defmethod track_y-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:track_y-val is deprecated.  Use crazyflie_scripts-msg:track_y instead.")
  (track_y m))

(cl:ensure-generic-function 'track_z-val :lambda-list '(m))
(cl:defmethod track_z-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:track_z-val is deprecated.  Use crazyflie_scripts-msg:track_z instead.")
  (track_z m))

(cl:ensure-generic-function 'x_dir_tt-val :lambda-list '(m))
(cl:defmethod x_dir_tt-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:x_dir_tt-val is deprecated.  Use crazyflie_scripts-msg:x_dir_tt instead.")
  (x_dir_tt m))

(cl:ensure-generic-function 'y_dir_tt-val :lambda-list '(m))
(cl:defmethod y_dir_tt-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:y_dir_tt-val is deprecated.  Use crazyflie_scripts-msg:y_dir_tt instead.")
  (y_dir_tt m))

(cl:ensure-generic-function 'tg_yaw_t-val :lambda-list '(m))
(cl:defmethod tg_yaw_t-val ((m <camera_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyflie_scripts-msg:tg_yaw_t-val is deprecated.  Use crazyflie_scripts-msg:tg_yaw_t instead.")
  (tg_yaw_t m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <camera_msg>) ostream)
  "Serializes a message object of type '<camera_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ext_x_cam))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ext_y_cam))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'track_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'track_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'track_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_dir_tt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_dir_tt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tg_yaw_t))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <camera_msg>) istream)
  "Deserializes a message object of type '<camera_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ext_x_cam) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ext_y_cam) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'track_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'track_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'track_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_dir_tt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_dir_tt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tg_yaw_t) (roslisp-utils:decode-single-float-bits bits)))
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
  "512cd6d5f46f8407166dee36dcd1fc07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'camera_msg)))
  "Returns md5sum for a message object of type 'camera_msg"
  "512cd6d5f46f8407166dee36dcd1fc07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<camera_msg>)))
  "Returns full string definition for message of type '<camera_msg>"
  (cl:format cl:nil "float32 ext_x_cam~%float32 ext_y_cam~%float32 track_x~%float32 track_y~%float32 track_z~%float32 x_dir_tt~%float32 y_dir_tt~%float32 tg_yaw_t~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'camera_msg)))
  "Returns full string definition for message of type 'camera_msg"
  (cl:format cl:nil "float32 ext_x_cam~%float32 ext_y_cam~%float32 track_x~%float32 track_y~%float32 track_z~%float32 x_dir_tt~%float32 y_dir_tt~%float32 tg_yaw_t~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <camera_msg>))
  (cl:+ 0
     4
     4
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
    (cl:cons ':ext_x_cam (ext_x_cam msg))
    (cl:cons ':ext_y_cam (ext_y_cam msg))
    (cl:cons ':track_x (track_x msg))
    (cl:cons ':track_y (track_y msg))
    (cl:cons ':track_z (track_z msg))
    (cl:cons ':x_dir_tt (x_dir_tt msg))
    (cl:cons ':y_dir_tt (y_dir_tt msg))
    (cl:cons ':tg_yaw_t (tg_yaw_t msg))
))
