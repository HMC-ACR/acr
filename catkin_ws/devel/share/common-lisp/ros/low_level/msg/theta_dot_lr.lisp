; Auto-generated. Do not edit!


(cl:in-package low_level-msg)


;//! \htmlinclude theta_dot_lr.msg.html

(cl:defclass <theta_dot_lr> (roslisp-msg-protocol:ros-message)
  ((theta_dot_left
    :reader theta_dot_left
    :initarg :theta_dot_left
    :type cl:float
    :initform 0.0)
   (theta_dot_right
    :reader theta_dot_right
    :initarg :theta_dot_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass theta_dot_lr (<theta_dot_lr>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <theta_dot_lr>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'theta_dot_lr)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name low_level-msg:<theta_dot_lr> is deprecated: use low_level-msg:theta_dot_lr instead.")))

(cl:ensure-generic-function 'theta_dot_left-val :lambda-list '(m))
(cl:defmethod theta_dot_left-val ((m <theta_dot_lr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader low_level-msg:theta_dot_left-val is deprecated.  Use low_level-msg:theta_dot_left instead.")
  (theta_dot_left m))

(cl:ensure-generic-function 'theta_dot_right-val :lambda-list '(m))
(cl:defmethod theta_dot_right-val ((m <theta_dot_lr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader low_level-msg:theta_dot_right-val is deprecated.  Use low_level-msg:theta_dot_right instead.")
  (theta_dot_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <theta_dot_lr>) ostream)
  "Serializes a message object of type '<theta_dot_lr>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_dot_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_dot_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <theta_dot_lr>) istream)
  "Deserializes a message object of type '<theta_dot_lr>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_dot_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_dot_right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<theta_dot_lr>)))
  "Returns string type for a message object of type '<theta_dot_lr>"
  "low_level/theta_dot_lr")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'theta_dot_lr)))
  "Returns string type for a message object of type 'theta_dot_lr"
  "low_level/theta_dot_lr")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<theta_dot_lr>)))
  "Returns md5sum for a message object of type '<theta_dot_lr>"
  "bd5df01c85a05c89a29d456cc9a16601")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'theta_dot_lr)))
  "Returns md5sum for a message object of type 'theta_dot_lr"
  "bd5df01c85a05c89a29d456cc9a16601")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<theta_dot_lr>)))
  "Returns full string definition for message of type '<theta_dot_lr>"
  (cl:format cl:nil "float32 theta_dot_left~%float32 theta_dot_right~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'theta_dot_lr)))
  "Returns full string definition for message of type 'theta_dot_lr"
  (cl:format cl:nil "float32 theta_dot_left~%float32 theta_dot_right~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <theta_dot_lr>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <theta_dot_lr>))
  "Converts a ROS message object to a list"
  (cl:list 'theta_dot_lr
    (cl:cons ':theta_dot_left (theta_dot_left msg))
    (cl:cons ':theta_dot_right (theta_dot_right msg))
))
