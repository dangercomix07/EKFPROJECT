; Auto-generated. Do not edit!


(cl:in-package course_project-msg)


;//! \htmlinclude Trilateration.msg.html

(cl:defclass <Trilateration> (roslisp-msg-protocol:ros-message)
  ((landmarkA
    :reader landmarkA
    :initarg :landmarkA
    :type course_project-msg:Landmark
    :initform (cl:make-instance 'course_project-msg:Landmark))
   (landmarkB
    :reader landmarkB
    :initarg :landmarkB
    :type course_project-msg:Landmark
    :initform (cl:make-instance 'course_project-msg:Landmark))
   (landmarkC
    :reader landmarkC
    :initarg :landmarkC
    :type course_project-msg:Landmark
    :initform (cl:make-instance 'course_project-msg:Landmark)))
)

(cl:defclass Trilateration (<Trilateration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trilateration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trilateration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name course_project-msg:<Trilateration> is deprecated: use course_project-msg:Trilateration instead.")))

(cl:ensure-generic-function 'landmarkA-val :lambda-list '(m))
(cl:defmethod landmarkA-val ((m <Trilateration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader course_project-msg:landmarkA-val is deprecated.  Use course_project-msg:landmarkA instead.")
  (landmarkA m))

(cl:ensure-generic-function 'landmarkB-val :lambda-list '(m))
(cl:defmethod landmarkB-val ((m <Trilateration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader course_project-msg:landmarkB-val is deprecated.  Use course_project-msg:landmarkB instead.")
  (landmarkB m))

(cl:ensure-generic-function 'landmarkC-val :lambda-list '(m))
(cl:defmethod landmarkC-val ((m <Trilateration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader course_project-msg:landmarkC-val is deprecated.  Use course_project-msg:landmarkC instead.")
  (landmarkC m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trilateration>) ostream)
  "Serializes a message object of type '<Trilateration>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'landmarkA) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'landmarkB) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'landmarkC) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trilateration>) istream)
  "Deserializes a message object of type '<Trilateration>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'landmarkA) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'landmarkB) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'landmarkC) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trilateration>)))
  "Returns string type for a message object of type '<Trilateration>"
  "course_project/Trilateration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trilateration)))
  "Returns string type for a message object of type 'Trilateration"
  "course_project/Trilateration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trilateration>)))
  "Returns md5sum for a message object of type '<Trilateration>"
  "45e1ed04607c6f7e36ae2697ced8826f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trilateration)))
  "Returns md5sum for a message object of type 'Trilateration"
  "45e1ed04607c6f7e36ae2697ced8826f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trilateration>)))
  "Returns full string definition for message of type '<Trilateration>"
  (cl:format cl:nil "Landmark landmarkA~%Landmark landmarkB~%Landmark landmarkC~%~%================================================================================~%MSG: course_project/Landmark~%float32 x~%float32 y~%float32 distance~%float32 variance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trilateration)))
  "Returns full string definition for message of type 'Trilateration"
  (cl:format cl:nil "Landmark landmarkA~%Landmark landmarkB~%Landmark landmarkC~%~%================================================================================~%MSG: course_project/Landmark~%float32 x~%float32 y~%float32 distance~%float32 variance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trilateration>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'landmarkA))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'landmarkB))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'landmarkC))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trilateration>))
  "Converts a ROS message object to a list"
  (cl:list 'Trilateration
    (cl:cons ':landmarkA (landmarkA msg))
    (cl:cons ':landmarkB (landmarkB msg))
    (cl:cons ':landmarkC (landmarkC msg))
))
