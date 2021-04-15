;; Auto-generated. Do not edit!


(when (boundp 'crazyflie_driver::NameArray)
  (if (not (find-package "CRAZYFLIE_DRIVER"))
    (make-package "CRAZYFLIE_DRIVER"))
  (shadow 'NameArray (find-package "CRAZYFLIE_DRIVER")))
(unless (find-package "CRAZYFLIE_DRIVER::NAMEARRAY")
  (make-package "CRAZYFLIE_DRIVER::NAMEARRAY"))

(in-package "ROS")
;;//! \htmlinclude NameArray.msg.html


(defclass crazyflie_driver::NameArray
  :super ros::object
  :slots (_names ))

(defmethod crazyflie_driver::NameArray
  (:init
   (&key
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _names __names)
   self)
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:serialization-length
   ()
   (+
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get crazyflie_driver::NameArray :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get crazyflie_driver::NameArray :datatype-) "crazyflie_driver/NameArray")
(setf (get crazyflie_driver::NameArray :definition-)
      "string[] names

")



(provide :crazyflie_driver/NameArray "dc7ae3609524b18034e49294a4ce670e")


