
(cl:in-package :asdf)

(defsystem "course_project-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Landmark" :depends-on ("_package_Landmark"))
    (:file "_package_Landmark" :depends-on ("_package"))
    (:file "Trilateration" :depends-on ("_package_Trilateration"))
    (:file "_package_Trilateration" :depends-on ("_package"))
  ))