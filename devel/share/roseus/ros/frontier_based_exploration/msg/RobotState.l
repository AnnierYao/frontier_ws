;; Auto-generated. Do not edit!


(when (boundp 'frontier_based_exploration::RobotState)
  (if (not (find-package "FRONTIER_BASED_EXPLORATION"))
    (make-package "FRONTIER_BASED_EXPLORATION"))
  (shadow 'RobotState (find-package "FRONTIER_BASED_EXPLORATION")))
(unless (find-package "FRONTIER_BASED_EXPLORATION::ROBOTSTATE")
  (make-package "FRONTIER_BASED_EXPLORATION::ROBOTSTATE"))

(in-package "ROS")
;;//! \htmlinclude RobotState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass frontier_based_exploration::RobotState
  :super ros::object
  :slots (_header _x _y _theta _linear_velocity _angular_velocity ))

(defmethod frontier_based_exploration::RobotState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    ((:linear_velocity __linear_velocity) 0.0)
    ((:angular_velocity __angular_velocity) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   (setq _linear_velocity (float __linear_velocity))
   (setq _angular_velocity (float __angular_velocity))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:linear_velocity
   (&optional __linear_velocity)
   (if __linear_velocity (setq _linear_velocity __linear_velocity)) _linear_velocity)
  (:angular_velocity
   (&optional __angular_velocity)
   (if __angular_velocity (setq _angular_velocity __angular_velocity)) _angular_velocity)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _theta
    4
    ;; float32 _linear_velocity
    4
    ;; float32 _angular_velocity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linear_velocity
       (sys::poke _linear_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular_velocity
       (sys::poke _angular_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta
     (setq _theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linear_velocity
     (setq _linear_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular_velocity
     (setq _angular_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get frontier_based_exploration::RobotState :md5sum-) "3b7f69e8b3ca09af6164794ff2eacaae")
(setf (get frontier_based_exploration::RobotState :datatype-) "frontier_based_exploration/RobotState")
(setf (get frontier_based_exploration::RobotState :definition-)
      "Header header
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :frontier_based_exploration/RobotState "3b7f69e8b3ca09af6164794ff2eacaae")


