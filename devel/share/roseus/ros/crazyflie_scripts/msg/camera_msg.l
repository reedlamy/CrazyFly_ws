;; Auto-generated. Do not edit!


(when (boundp 'crazyflie_scripts::camera_msg)
  (if (not (find-package "CRAZYFLIE_SCRIPTS"))
    (make-package "CRAZYFLIE_SCRIPTS"))
  (shadow 'camera_msg (find-package "CRAZYFLIE_SCRIPTS")))
(unless (find-package "CRAZYFLIE_SCRIPTS::CAMERA_MSG")
  (make-package "CRAZYFLIE_SCRIPTS::CAMERA_MSG"))

(in-package "ROS")
;;//! \htmlinclude camera_msg.msg.html


(defclass crazyflie_scripts::camera_msg
  :super ros::object
  :slots (_ext_x _ext_y _t_x _t_y _x_dir_tt _y_dir_tt ))

(defmethod crazyflie_scripts::camera_msg
  (:init
   (&key
    ((:ext_x __ext_x) 0)
    ((:ext_y __ext_y) 0)
    ((:t_x __t_x) 0)
    ((:t_y __t_y) 0)
    ((:x_dir_tt __x_dir_tt) 0)
    ((:y_dir_tt __y_dir_tt) 0)
    )
   (send-super :init)
   (setq _ext_x (round __ext_x))
   (setq _ext_y (round __ext_y))
   (setq _t_x (round __t_x))
   (setq _t_y (round __t_y))
   (setq _x_dir_tt (round __x_dir_tt))
   (setq _y_dir_tt (round __y_dir_tt))
   self)
  (:ext_x
   (&optional __ext_x)
   (if __ext_x (setq _ext_x __ext_x)) _ext_x)
  (:ext_y
   (&optional __ext_y)
   (if __ext_y (setq _ext_y __ext_y)) _ext_y)
  (:t_x
   (&optional __t_x)
   (if __t_x (setq _t_x __t_x)) _t_x)
  (:t_y
   (&optional __t_y)
   (if __t_y (setq _t_y __t_y)) _t_y)
  (:x_dir_tt
   (&optional __x_dir_tt)
   (if __x_dir_tt (setq _x_dir_tt __x_dir_tt)) _x_dir_tt)
  (:y_dir_tt
   (&optional __y_dir_tt)
   (if __y_dir_tt (setq _y_dir_tt __y_dir_tt)) _y_dir_tt)
  (:serialization-length
   ()
   (+
    ;; int32 _ext_x
    4
    ;; int32 _ext_y
    4
    ;; int32 _t_x
    4
    ;; int32 _t_y
    4
    ;; int32 _x_dir_tt
    4
    ;; int32 _y_dir_tt
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _ext_x
       (write-long _ext_x s)
     ;; int32 _ext_y
       (write-long _ext_y s)
     ;; int32 _t_x
       (write-long _t_x s)
     ;; int32 _t_y
       (write-long _t_y s)
     ;; int32 _x_dir_tt
       (write-long _x_dir_tt s)
     ;; int32 _y_dir_tt
       (write-long _y_dir_tt s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _ext_x
     (setq _ext_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ext_y
     (setq _ext_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _t_x
     (setq _t_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _t_y
     (setq _t_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _x_dir_tt
     (setq _x_dir_tt (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y_dir_tt
     (setq _y_dir_tt (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get crazyflie_scripts::camera_msg :md5sum-) "0528b01776a9faff38d86476b22e6f80")
(setf (get crazyflie_scripts::camera_msg :datatype-) "crazyflie_scripts/camera_msg")
(setf (get crazyflie_scripts::camera_msg :definition-)
      "int32 ext_x
int32 ext_y
int32 t_x
int32 t_y
int32 x_dir_tt
int32 y_dir_tt

")



(provide :crazyflie_scripts/camera_msg "0528b01776a9faff38d86476b22e6f80")


