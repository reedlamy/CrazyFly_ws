; Auto-generated. Do not edit!


(cl:in-package car-msg)


;//! \htmlinclude pid.msg.html

(cl:defclass <pid> (roslisp-msg-protocol:ros-message)
  ((setpoint
    :reader setpoint
    :initarg :setpoint
    :type cl:float
    :initform 0.0)
   (laster
    :reader laster
    :initarg :laster
    :type cl:float
    :initform 0.0)
   (err
    :reader err
    :initarg :err
    :type cl:float
    :initform 0.0)
   (wndup
    :reader wndup
    :initarg :wndup
    :type cl:float
    :initform 0.0)
   (kp
    :reader kp
    :initarg :kp
    :type cl:float
    :initform 0.0)
   (kd
    :reader kd
    :initarg :kd
    :type cl:float
    :initform 0.0)
   (ki
    :reader ki
    :initarg :ki
    :type cl:float
    :initform 0.0)
   (out
    :reader out
    :initarg :out
    :type cl:float
    :initform 0.0)
   (fdbck
    :reader fdbck
    :initarg :fdbck
    :type cl:float
    :initform 0.0)
   (iterm
    :reader iterm
    :initarg :iterm
    :type cl:float
    :initform 0.0)
   (pterm
    :reader pterm
    :initarg :pterm
    :type cl:float
    :initform 0.0)
   (dterm
    :reader dterm
    :initarg :dterm
    :type cl:float
    :initform 0.0)
   (delterr
    :reader delterr
    :initarg :delterr
    :type cl:float
    :initform 0.0)
   (delttime
    :reader delttime
    :initarg :delttime
    :type cl:float
    :initform 0.0))
)

(cl:defclass pid (<pid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name car-msg:<pid> is deprecated: use car-msg:pid instead.")))

(cl:ensure-generic-function 'setpoint-val :lambda-list '(m))
(cl:defmethod setpoint-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:setpoint-val is deprecated.  Use car-msg:setpoint instead.")
  (setpoint m))

(cl:ensure-generic-function 'laster-val :lambda-list '(m))
(cl:defmethod laster-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:laster-val is deprecated.  Use car-msg:laster instead.")
  (laster m))

(cl:ensure-generic-function 'err-val :lambda-list '(m))
(cl:defmethod err-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:err-val is deprecated.  Use car-msg:err instead.")
  (err m))

(cl:ensure-generic-function 'wndup-val :lambda-list '(m))
(cl:defmethod wndup-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:wndup-val is deprecated.  Use car-msg:wndup instead.")
  (wndup m))

(cl:ensure-generic-function 'kp-val :lambda-list '(m))
(cl:defmethod kp-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:kp-val is deprecated.  Use car-msg:kp instead.")
  (kp m))

(cl:ensure-generic-function 'kd-val :lambda-list '(m))
(cl:defmethod kd-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:kd-val is deprecated.  Use car-msg:kd instead.")
  (kd m))

(cl:ensure-generic-function 'ki-val :lambda-list '(m))
(cl:defmethod ki-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:ki-val is deprecated.  Use car-msg:ki instead.")
  (ki m))

(cl:ensure-generic-function 'out-val :lambda-list '(m))
(cl:defmethod out-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:out-val is deprecated.  Use car-msg:out instead.")
  (out m))

(cl:ensure-generic-function 'fdbck-val :lambda-list '(m))
(cl:defmethod fdbck-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:fdbck-val is deprecated.  Use car-msg:fdbck instead.")
  (fdbck m))

(cl:ensure-generic-function 'iterm-val :lambda-list '(m))
(cl:defmethod iterm-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:iterm-val is deprecated.  Use car-msg:iterm instead.")
  (iterm m))

(cl:ensure-generic-function 'pterm-val :lambda-list '(m))
(cl:defmethod pterm-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:pterm-val is deprecated.  Use car-msg:pterm instead.")
  (pterm m))

(cl:ensure-generic-function 'dterm-val :lambda-list '(m))
(cl:defmethod dterm-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:dterm-val is deprecated.  Use car-msg:dterm instead.")
  (dterm m))

(cl:ensure-generic-function 'delterr-val :lambda-list '(m))
(cl:defmethod delterr-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:delterr-val is deprecated.  Use car-msg:delterr instead.")
  (delterr m))

(cl:ensure-generic-function 'delttime-val :lambda-list '(m))
(cl:defmethod delttime-val ((m <pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car-msg:delttime-val is deprecated.  Use car-msg:delttime instead.")
  (delttime m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pid>) ostream)
  "Serializes a message object of type '<pid>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'setpoint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'laster))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'err))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wndup))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'kp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'kd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ki))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'out))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'fdbck))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'iterm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pterm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dterm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delterr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delttime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pid>) istream)
  "Deserializes a message object of type '<pid>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'setpoint) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'laster) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'err) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wndup) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ki) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'out) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fdbck) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'iterm) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pterm) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dterm) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delterr) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delttime) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pid>)))
  "Returns string type for a message object of type '<pid>"
  "car/pid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pid)))
  "Returns string type for a message object of type 'pid"
  "car/pid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pid>)))
  "Returns md5sum for a message object of type '<pid>"
  "87da0e9e14c5b44deff5947cc352d507")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pid)))
  "Returns md5sum for a message object of type 'pid"
  "87da0e9e14c5b44deff5947cc352d507")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pid>)))
  "Returns full string definition for message of type '<pid>"
  (cl:format cl:nil "float32 setpoint~%float32 laster~%float32 err~%float32 wndup~%float32 kp~%float32 kd~%float32 ki~%float32 out~%float64 fdbck~%float32 iterm~%float32 pterm~%float32 dterm~%float32 delterr~%float32 delttime~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pid)))
  "Returns full string definition for message of type 'pid"
  (cl:format cl:nil "float32 setpoint~%float32 laster~%float32 err~%float32 wndup~%float32 kp~%float32 kd~%float32 ki~%float32 out~%float64 fdbck~%float32 iterm~%float32 pterm~%float32 dterm~%float32 delterr~%float32 delttime~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pid>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     8
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pid>))
  "Converts a ROS message object to a list"
  (cl:list 'pid
    (cl:cons ':setpoint (setpoint msg))
    (cl:cons ':laster (laster msg))
    (cl:cons ':err (err msg))
    (cl:cons ':wndup (wndup msg))
    (cl:cons ':kp (kp msg))
    (cl:cons ':kd (kd msg))
    (cl:cons ':ki (ki msg))
    (cl:cons ':out (out msg))
    (cl:cons ':fdbck (fdbck msg))
    (cl:cons ':iterm (iterm msg))
    (cl:cons ':pterm (pterm msg))
    (cl:cons ':dterm (dterm msg))
    (cl:cons ':delterr (delterr msg))
    (cl:cons ':delttime (delttime msg))
))
