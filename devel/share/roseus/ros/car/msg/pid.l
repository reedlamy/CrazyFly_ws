;; Auto-generated. Do not edit!


(when (boundp 'car::pid)
  (if (not (find-package "CAR"))
    (make-package "CAR"))
  (shadow 'pid (find-package "CAR")))
(unless (find-package "CAR::PID")
  (make-package "CAR::PID"))

(in-package "ROS")
;;//! \htmlinclude pid.msg.html


(defclass car::pid
  :super ros::object
  :slots (_setpoint _laster _err _wndup _kp _kd _ki _out _fdbck _iterm _pterm _dterm _delterr _delttime ))

(defmethod car::pid
  (:init
   (&key
    ((:setpoint __setpoint) 0.0)
    ((:laster __laster) 0.0)
    ((:err __err) 0.0)
    ((:wndup __wndup) 0.0)
    ((:kp __kp) 0.0)
    ((:kd __kd) 0.0)
    ((:ki __ki) 0.0)
    ((:out __out) 0.0)
    ((:fdbck __fdbck) 0.0)
    ((:iterm __iterm) 0.0)
    ((:pterm __pterm) 0.0)
    ((:dterm __dterm) 0.0)
    ((:delterr __delterr) 0.0)
    ((:delttime __delttime) 0.0)
    )
   (send-super :init)
   (setq _setpoint (float __setpoint))
   (setq _laster (float __laster))
   (setq _err (float __err))
   (setq _wndup (float __wndup))
   (setq _kp (float __kp))
   (setq _kd (float __kd))
   (setq _ki (float __ki))
   (setq _out (float __out))
   (setq _fdbck (float __fdbck))
   (setq _iterm (float __iterm))
   (setq _pterm (float __pterm))
   (setq _dterm (float __dterm))
   (setq _delterr (float __delterr))
   (setq _delttime (float __delttime))
   self)
  (:setpoint
   (&optional __setpoint)
   (if __setpoint (setq _setpoint __setpoint)) _setpoint)
  (:laster
   (&optional __laster)
   (if __laster (setq _laster __laster)) _laster)
  (:err
   (&optional __err)
   (if __err (setq _err __err)) _err)
  (:wndup
   (&optional __wndup)
   (if __wndup (setq _wndup __wndup)) _wndup)
  (:kp
   (&optional __kp)
   (if __kp (setq _kp __kp)) _kp)
  (:kd
   (&optional __kd)
   (if __kd (setq _kd __kd)) _kd)
  (:ki
   (&optional __ki)
   (if __ki (setq _ki __ki)) _ki)
  (:out
   (&optional __out)
   (if __out (setq _out __out)) _out)
  (:fdbck
   (&optional __fdbck)
   (if __fdbck (setq _fdbck __fdbck)) _fdbck)
  (:iterm
   (&optional __iterm)
   (if __iterm (setq _iterm __iterm)) _iterm)
  (:pterm
   (&optional __pterm)
   (if __pterm (setq _pterm __pterm)) _pterm)
  (:dterm
   (&optional __dterm)
   (if __dterm (setq _dterm __dterm)) _dterm)
  (:delterr
   (&optional __delterr)
   (if __delterr (setq _delterr __delterr)) _delterr)
  (:delttime
   (&optional __delttime)
   (if __delttime (setq _delttime __delttime)) _delttime)
  (:serialization-length
   ()
   (+
    ;; float32 _setpoint
    4
    ;; float32 _laster
    4
    ;; float32 _err
    4
    ;; float32 _wndup
    4
    ;; float32 _kp
    4
    ;; float32 _kd
    4
    ;; float32 _ki
    4
    ;; float32 _out
    4
    ;; float64 _fdbck
    8
    ;; float32 _iterm
    4
    ;; float32 _pterm
    4
    ;; float32 _dterm
    4
    ;; float32 _delterr
    4
    ;; float32 _delttime
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _setpoint
       (sys::poke _setpoint (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _laster
       (sys::poke _laster (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _err
       (sys::poke _err (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _wndup
       (sys::poke _wndup (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _kp
       (sys::poke _kp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _kd
       (sys::poke _kd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ki
       (sys::poke _ki (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _out
       (sys::poke _out (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _fdbck
       (sys::poke _fdbck (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _iterm
       (sys::poke _iterm (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pterm
       (sys::poke _pterm (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dterm
       (sys::poke _dterm (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _delterr
       (sys::poke _delterr (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _delttime
       (sys::poke _delttime (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _setpoint
     (setq _setpoint (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _laster
     (setq _laster (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _err
     (setq _err (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _wndup
     (setq _wndup (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _kp
     (setq _kp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _kd
     (setq _kd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ki
     (setq _ki (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _out
     (setq _out (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _fdbck
     (setq _fdbck (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _iterm
     (setq _iterm (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pterm
     (setq _pterm (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dterm
     (setq _dterm (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _delterr
     (setq _delterr (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _delttime
     (setq _delttime (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get car::pid :md5sum-) "87da0e9e14c5b44deff5947cc352d507")
(setf (get car::pid :datatype-) "car/pid")
(setf (get car::pid :definition-)
      "float32 setpoint
float32 laster
float32 err
float32 wndup
float32 kp
float32 kd
float32 ki
float32 out
float64 fdbck
float32 iterm
float32 pterm
float32 dterm
float32 delterr
float32 delttime

")



(provide :car/pid "87da0e9e14c5b44deff5947cc352d507")


