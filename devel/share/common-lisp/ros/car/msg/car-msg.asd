
(cl:in-package :asdf)

(defsystem "car-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "pid" :depends-on ("_package_pid"))
    (:file "_package_pid" :depends-on ("_package"))
  ))