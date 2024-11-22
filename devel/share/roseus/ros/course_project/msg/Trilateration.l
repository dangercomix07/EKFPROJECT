;; Auto-generated. Do not edit!


(when (boundp 'course_project::Trilateration)
  (if (not (find-package "COURSE_PROJECT"))
    (make-package "COURSE_PROJECT"))
  (shadow 'Trilateration (find-package "COURSE_PROJECT")))
(unless (find-package "COURSE_PROJECT::TRILATERATION")
  (make-package "COURSE_PROJECT::TRILATERATION"))

(in-package "ROS")
;;//! \htmlinclude Trilateration.msg.html


(defclass course_project::Trilateration
  :super ros::object
  :slots (_landmarkA _landmarkB _landmarkC ))

(defmethod course_project::Trilateration
  (:init
   (&key
    ((:landmarkA __landmarkA) (instance course_project::Landmark :init))
    ((:landmarkB __landmarkB) (instance course_project::Landmark :init))
    ((:landmarkC __landmarkC) (instance course_project::Landmark :init))
    )
   (send-super :init)
   (setq _landmarkA __landmarkA)
   (setq _landmarkB __landmarkB)
   (setq _landmarkC __landmarkC)
   self)
  (:landmarkA
   (&rest __landmarkA)
   (if (keywordp (car __landmarkA))
       (send* _landmarkA __landmarkA)
     (progn
       (if __landmarkA (setq _landmarkA (car __landmarkA)))
       _landmarkA)))
  (:landmarkB
   (&rest __landmarkB)
   (if (keywordp (car __landmarkB))
       (send* _landmarkB __landmarkB)
     (progn
       (if __landmarkB (setq _landmarkB (car __landmarkB)))
       _landmarkB)))
  (:landmarkC
   (&rest __landmarkC)
   (if (keywordp (car __landmarkC))
       (send* _landmarkC __landmarkC)
     (progn
       (if __landmarkC (setq _landmarkC (car __landmarkC)))
       _landmarkC)))
  (:serialization-length
   ()
   (+
    ;; course_project/Landmark _landmarkA
    (send _landmarkA :serialization-length)
    ;; course_project/Landmark _landmarkB
    (send _landmarkB :serialization-length)
    ;; course_project/Landmark _landmarkC
    (send _landmarkC :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; course_project/Landmark _landmarkA
       (send _landmarkA :serialize s)
     ;; course_project/Landmark _landmarkB
       (send _landmarkB :serialize s)
     ;; course_project/Landmark _landmarkC
       (send _landmarkC :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; course_project/Landmark _landmarkA
     (send _landmarkA :deserialize buf ptr-) (incf ptr- (send _landmarkA :serialization-length))
   ;; course_project/Landmark _landmarkB
     (send _landmarkB :deserialize buf ptr-) (incf ptr- (send _landmarkB :serialization-length))
   ;; course_project/Landmark _landmarkC
     (send _landmarkC :deserialize buf ptr-) (incf ptr- (send _landmarkC :serialization-length))
   ;;
   self)
  )

(setf (get course_project::Trilateration :md5sum-) "45e1ed04607c6f7e36ae2697ced8826f")
(setf (get course_project::Trilateration :datatype-) "course_project/Trilateration")
(setf (get course_project::Trilateration :definition-)
      "Landmark landmarkA
Landmark landmarkB
Landmark landmarkC

================================================================================
MSG: course_project/Landmark
float32 x
float32 y
float32 distance
float32 variance

")



(provide :course_project/Trilateration "45e1ed04607c6f7e36ae2697ced8826f")


