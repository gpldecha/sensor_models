
(cl:in-package :asdf)

(defsystem "sensor_models-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FingerIK_cmd" :depends-on ("_package_FingerIK_cmd"))
    (:file "_package_FingerIK_cmd" :depends-on ("_package"))
    (:file "String_cmd" :depends-on ("_package_String_cmd"))
    (:file "_package_String_cmd" :depends-on ("_package"))
    (:file "Parameter_cmd" :depends-on ("_package_Parameter_cmd"))
    (:file "_package_Parameter_cmd" :depends-on ("_package"))
  ))