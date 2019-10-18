
(cl:in-package :asdf)

(defsystem "low_level-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "theta_dot_lr" :depends-on ("_package_theta_dot_lr"))
    (:file "_package_theta_dot_lr" :depends-on ("_package"))
  ))