;; Auto-generated. Do not edit!


(when (boundp 'me212bot::WheelVelCmd)
  (if (not (find-package "ME212BOT"))
    (make-package "ME212BOT"))
  (shadow 'WheelVelCmd (find-package "ME212BOT")))
(unless (find-package "ME212BOT::WHEELVELCMD")
  (make-package "ME212BOT::WHEELVELCMD"))

(in-package "ROS")
;;//! \htmlinclude WheelVelCmd.msg.html


(defclass me212bot::WheelVelCmd
  :super ros::object
  :slots (_desiredWV_R _desiredWV_L ))

(defmethod me212bot::WheelVelCmd
  (:init
   (&key
    ((:desiredWV_R __desiredWV_R) 0.0)
    ((:desiredWV_L __desiredWV_L) 0.0)
    )
   (send-super :init)
   (setq _desiredWV_R (float __desiredWV_R))
   (setq _desiredWV_L (float __desiredWV_L))
   self)
  (:desiredWV_R
   (&optional __desiredWV_R)
   (if __desiredWV_R (setq _desiredWV_R __desiredWV_R)) _desiredWV_R)
  (:desiredWV_L
   (&optional __desiredWV_L)
   (if __desiredWV_L (setq _desiredWV_L __desiredWV_L)) _desiredWV_L)
  (:serialization-length
   ()
   (+
    ;; float32 _desiredWV_R
    4
    ;; float32 _desiredWV_L
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _desiredWV_R
       (sys::poke _desiredWV_R (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _desiredWV_L
       (sys::poke _desiredWV_L (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _desiredWV_R
     (setq _desiredWV_R (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _desiredWV_L
     (setq _desiredWV_L (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get me212bot::WheelVelCmd :md5sum-) "428fbbfd1f38717ca7baa73045b4efaa")
(setf (get me212bot::WheelVelCmd :datatype-) "me212bot/WheelVelCmd")
(setf (get me212bot::WheelVelCmd :definition-)
      "float32 desiredWV_R
float32 desiredWV_L

")



(provide :me212bot/WheelVelCmd "428fbbfd1f38717ca7baa73045b4efaa")


