;; Auto-generated. Do not edit!


(when (boundp 'course_project::Landmark)
  (if (not (find-package "COURSE_PROJECT"))
    (make-package "COURSE_PROJECT"))
  (shadow 'Landmark (find-package "COURSE_PROJECT")))
(unless (find-package "COURSE_PROJECT::LANDMARK")
  (make-package "COURSE_PROJECT::LANDMARK"))

(in-package "ROS")
;;//! \htmlinclude Landmark.msg.html


(defclass course_project::Landmark
  :super ros::object
  :slots (_x _y _distance _variance ))

(defmethod course_project::Landmark
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:distance __distance) 0.0)
    ((:variance __variance) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _distance (float __distance))
   (setq _variance (float __variance))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:variance
   (&optional __variance)
   (if __variance (setq _variance __variance)) _variance)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _distance
    4
    ;; float32 _variance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _variance
       (sys::poke _variance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _variance
     (setq _variance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get course_project::Landmark :md5sum-) "d57b053e39926628bfa2041c6ea64533")
(setf (get course_project::Landmark :datatype-) "course_project/Landmark")
(setf (get course_project::Landmark :definition-)
      "float32 x
float32 y
float32 distance
float32 variance

")



(provide :course_project/Landmark "d57b053e39926628bfa2041c6ea64533")


