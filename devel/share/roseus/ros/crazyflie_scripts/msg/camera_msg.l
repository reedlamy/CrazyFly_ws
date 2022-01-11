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
  :slots (_ext_x_cam _ext_y_cam _track_x _track_y _track_z _x_dir_tt _y_dir_tt _tg_yaw_t ))

(defmethod crazyflie_scripts::camera_msg
  (:init
   (&key
    ((:ext_x_cam __ext_x_cam) 0.0)
    ((:ext_y_cam __ext_y_cam) 0.0)
    ((:track_x __track_x) 0.0)
    ((:track_y __track_y) 0.0)
    ((:track_z __track_z) 0.0)
    ((:x_dir_tt __x_dir_tt) 0.0)
    ((:y_dir_tt __y_dir_tt) 0.0)
    ((:tg_yaw_t __tg_yaw_t) 0.0)
    )
   (send-super :init)
   (setq _ext_x_cam (float __ext_x_cam))
   (setq _ext_y_cam (float __ext_y_cam))
   (setq _track_x (float __track_x))
   (setq _track_y (float __track_y))
   (setq _track_z (float __track_z))
   (setq _x_dir_tt (float __x_dir_tt))
   (setq _y_dir_tt (float __y_dir_tt))
   (setq _tg_yaw_t (float __tg_yaw_t))
   self)
  (:ext_x_cam
   (&optional __ext_x_cam)
   (if __ext_x_cam (setq _ext_x_cam __ext_x_cam)) _ext_x_cam)
  (:ext_y_cam
   (&optional __ext_y_cam)
   (if __ext_y_cam (setq _ext_y_cam __ext_y_cam)) _ext_y_cam)
  (:track_x
   (&optional __track_x)
   (if __track_x (setq _track_x __track_x)) _track_x)
  (:track_y
   (&optional __track_y)
   (if __track_y (setq _track_y __track_y)) _track_y)
  (:track_z
   (&optional __track_z)
   (if __track_z (setq _track_z __track_z)) _track_z)
  (:x_dir_tt
   (&optional __x_dir_tt)
   (if __x_dir_tt (setq _x_dir_tt __x_dir_tt)) _x_dir_tt)
  (:y_dir_tt
   (&optional __y_dir_tt)
   (if __y_dir_tt (setq _y_dir_tt __y_dir_tt)) _y_dir_tt)
  (:tg_yaw_t
   (&optional __tg_yaw_t)
   (if __tg_yaw_t (setq _tg_yaw_t __tg_yaw_t)) _tg_yaw_t)
  (:serialization-length
   ()
   (+
    ;; float32 _ext_x_cam
    4
    ;; float32 _ext_y_cam
    4
    ;; float32 _track_x
    4
    ;; float32 _track_y
    4
    ;; float32 _track_z
    4
    ;; float32 _x_dir_tt
    4
    ;; float32 _y_dir_tt
    4
    ;; float32 _tg_yaw_t
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _ext_x_cam
       (sys::poke _ext_x_cam (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ext_y_cam
       (sys::poke _ext_y_cam (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _track_x
       (sys::poke _track_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _track_y
       (sys::poke _track_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _track_z
       (sys::poke _track_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_dir_tt
       (sys::poke _x_dir_tt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_dir_tt
       (sys::poke _y_dir_tt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tg_yaw_t
       (sys::poke _tg_yaw_t (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _ext_x_cam
     (setq _ext_x_cam (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ext_y_cam
     (setq _ext_y_cam (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _track_x
     (setq _track_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _track_y
     (setq _track_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _track_z
     (setq _track_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_dir_tt
     (setq _x_dir_tt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_dir_tt
     (setq _y_dir_tt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tg_yaw_t
     (setq _tg_yaw_t (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get crazyflie_scripts::camera_msg :md5sum-) "512cd6d5f46f8407166dee36dcd1fc07")
(setf (get crazyflie_scripts::camera_msg :datatype-) "crazyflie_scripts/camera_msg")
(setf (get crazyflie_scripts::camera_msg :definition-)
      "float32 ext_x_cam
float32 ext_y_cam
float32 track_x
float32 track_y
float32 track_z
float32 x_dir_tt
float32 y_dir_tt
float32 tg_yaw_t

")



(provide :crazyflie_scripts/camera_msg "512cd6d5f46f8407166dee36dcd1fc07")


