;; Auto-generated. Do not edit!


(when (boundp 'ur_msgs::GripperMove)
  (if (not (find-package "UR_MSGS"))
    (make-package "UR_MSGS"))
  (shadow 'GripperMove (find-package "UR_MSGS")))
(unless (find-package "UR_MSGS::GRIPPERMOVE")
  (make-package "UR_MSGS::GRIPPERMOVE"))
(unless (find-package "UR_MSGS::GRIPPERMOVEREQUEST")
  (make-package "UR_MSGS::GRIPPERMOVEREQUEST"))
(unless (find-package "UR_MSGS::GRIPPERMOVERESPONSE")
  (make-package "UR_MSGS::GRIPPERMOVERESPONSE"))

(in-package "ROS")





(defclass ur_msgs::GripperMoveRequest
  :super ros::object
  :slots (_pos _force _speed _position_compensation ))

(defmethod ur_msgs::GripperMoveRequest
  (:init
   (&key
    ((:pos __pos) 0)
    ((:force __force) 0)
    ((:speed __speed) 0)
    ((:position_compensation __position_compensation) nil)
    )
   (send-super :init)
   (setq _pos (round __pos))
   (setq _force (round __force))
   (setq _speed (round __speed))
   (setq _position_compensation __position_compensation)
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:force
   (&optional __force)
   (if __force (setq _force __force)) _force)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:position_compensation
   (&optional __position_compensation)
   (if __position_compensation (setq _position_compensation __position_compensation)) _position_compensation)
  (:serialization-length
   ()
   (+
    ;; uint8 _pos
    1
    ;; uint8 _force
    1
    ;; uint8 _speed
    1
    ;; bool _position_compensation
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _pos
       (write-byte _pos s)
     ;; uint8 _force
       (write-byte _force s)
     ;; uint8 _speed
       (write-byte _speed s)
     ;; bool _position_compensation
       (if _position_compensation (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _pos
     (setq _pos (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _force
     (setq _force (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _speed
     (setq _speed (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _position_compensation
     (setq _position_compensation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ur_msgs::GripperMoveResponse
  :super ros::object
  :slots (_success _has_object ))

(defmethod ur_msgs::GripperMoveResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:has_object __has_object) nil)
    )
   (send-super :init)
   (setq _success __success)
   (setq _has_object __has_object)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:has_object
   (&optional __has_object)
   (if __has_object (setq _has_object __has_object)) _has_object)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; bool _has_object
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; bool _has_object
       (if _has_object (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _has_object
     (setq _has_object (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ur_msgs::GripperMove
  :super ros::object
  :slots ())

(setf (get ur_msgs::GripperMove :md5sum-) "d1d55f75257890b063dd76689fbde353")
(setf (get ur_msgs::GripperMove :datatype-) "ur_msgs/GripperMove")
(setf (get ur_msgs::GripperMove :request) ur_msgs::GripperMoveRequest)
(setf (get ur_msgs::GripperMove :response) ur_msgs::GripperMoveResponse)

(defmethod ur_msgs::GripperMoveRequest
  (:response () (instance ur_msgs::GripperMoveResponse :init)))

(setf (get ur_msgs::GripperMoveRequest :md5sum-) "d1d55f75257890b063dd76689fbde353")
(setf (get ur_msgs::GripperMoveRequest :datatype-) "ur_msgs/GripperMoveRequest")
(setf (get ur_msgs::GripperMoveRequest :definition-)
      "uint8 pos
uint8 force
uint8 speed
bool position_compensation
---
bool success
bool has_object

")

(setf (get ur_msgs::GripperMoveResponse :md5sum-) "d1d55f75257890b063dd76689fbde353")
(setf (get ur_msgs::GripperMoveResponse :datatype-) "ur_msgs/GripperMoveResponse")
(setf (get ur_msgs::GripperMoveResponse :definition-)
      "uint8 pos
uint8 force
uint8 speed
bool position_compensation
---
bool success
bool has_object

")



(provide :ur_msgs/GripperMove "d1d55f75257890b063dd76689fbde353")


