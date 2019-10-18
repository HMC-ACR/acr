;; Auto-generated. Do not edit!


(when (boundp 'low_level::theta_dot_lr)
  (if (not (find-package "LOW_LEVEL"))
    (make-package "LOW_LEVEL"))
  (shadow 'theta_dot_lr (find-package "LOW_LEVEL")))
(unless (find-package "LOW_LEVEL::THETA_DOT_LR")
  (make-package "LOW_LEVEL::THETA_DOT_LR"))

(in-package "ROS")
;;//! \htmlinclude theta_dot_lr.msg.html


(defclass low_level::theta_dot_lr
  :super ros::object
  :slots (_theta_dot_left _theta_dot_right ))

(defmethod low_level::theta_dot_lr
  (:init
   (&key
    ((:theta_dot_left __theta_dot_left) 0.0)
    ((:theta_dot_right __theta_dot_right) 0.0)
    )
   (send-super :init)
   (setq _theta_dot_left (float __theta_dot_left))
   (setq _theta_dot_right (float __theta_dot_right))
   self)
  (:theta_dot_left
   (&optional __theta_dot_left)
   (if __theta_dot_left (setq _theta_dot_left __theta_dot_left)) _theta_dot_left)
  (:theta_dot_right
   (&optional __theta_dot_right)
   (if __theta_dot_right (setq _theta_dot_right __theta_dot_right)) _theta_dot_right)
  (:serialization-length
   ()
   (+
    ;; float32 _theta_dot_left
    4
    ;; float32 _theta_dot_right
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _theta_dot_left
       (sys::poke _theta_dot_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta_dot_right
       (sys::poke _theta_dot_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _theta_dot_left
     (setq _theta_dot_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta_dot_right
     (setq _theta_dot_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get low_level::theta_dot_lr :md5sum-) "bd5df01c85a05c89a29d456cc9a16601")
(setf (get low_level::theta_dot_lr :datatype-) "low_level/theta_dot_lr")
(setf (get low_level::theta_dot_lr :definition-)
      "float32 theta_dot_left
float32 theta_dot_right


")



(provide :low_level/theta_dot_lr "bd5df01c85a05c89a29d456cc9a16601")


