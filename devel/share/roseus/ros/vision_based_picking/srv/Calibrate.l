;; Auto-generated. Do not edit!


(when (boundp 'vision_based_picking::Calibrate)
  (if (not (find-package "VISION_BASED_PICKING"))
    (make-package "VISION_BASED_PICKING"))
  (shadow 'Calibrate (find-package "VISION_BASED_PICKING")))
(unless (find-package "VISION_BASED_PICKING::CALIBRATE")
  (make-package "VISION_BASED_PICKING::CALIBRATE"))
(unless (find-package "VISION_BASED_PICKING::CALIBRATEREQUEST")
  (make-package "VISION_BASED_PICKING::CALIBRATEREQUEST"))
(unless (find-package "VISION_BASED_PICKING::CALIBRATERESPONSE")
  (make-package "VISION_BASED_PICKING::CALIBRATERESPONSE"))

(in-package "ROS")





(defclass vision_based_picking::CalibrateRequest
  :super ros::object
  :slots (_the_request _cam_index ))

(defmethod vision_based_picking::CalibrateRequest
  (:init
   (&key
    ((:the_request __the_request) "")
    ((:cam_index __cam_index) 0)
    )
   (send-super :init)
   (setq _the_request (string __the_request))
   (setq _cam_index (round __cam_index))
   self)
  (:the_request
   (&optional __the_request)
   (if __the_request (setq _the_request __the_request)) _the_request)
  (:cam_index
   (&optional __cam_index)
   (if __cam_index (setq _cam_index __cam_index)) _cam_index)
  (:serialization-length
   ()
   (+
    ;; string _the_request
    4 (length _the_request)
    ;; int16 _cam_index
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _the_request
       (write-long (length _the_request) s) (princ _the_request s)
     ;; int16 _cam_index
       (write-word _cam_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _the_request
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _the_request (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _cam_index
     (setq _cam_index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass vision_based_picking::CalibrateResponse
  :super ros::object
  :slots (_x _y _z ))

(defmethod vision_based_picking::CalibrateResponse
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass vision_based_picking::Calibrate
  :super ros::object
  :slots ())

(setf (get vision_based_picking::Calibrate :md5sum-) "d402ff8b5a8f9b73b3f2511623d98c8d")
(setf (get vision_based_picking::Calibrate :datatype-) "vision_based_picking/Calibrate")
(setf (get vision_based_picking::Calibrate :request) vision_based_picking::CalibrateRequest)
(setf (get vision_based_picking::Calibrate :response) vision_based_picking::CalibrateResponse)

(defmethod vision_based_picking::CalibrateRequest
  (:response () (instance vision_based_picking::CalibrateResponse :init)))

(setf (get vision_based_picking::CalibrateRequest :md5sum-) "d402ff8b5a8f9b73b3f2511623d98c8d")
(setf (get vision_based_picking::CalibrateRequest :datatype-) "vision_based_picking/CalibrateRequest")
(setf (get vision_based_picking::CalibrateRequest :definition-)
      "string the_request
int16 cam_index
---
float64 x
float64 y
float64 z

")

(setf (get vision_based_picking::CalibrateResponse :md5sum-) "d402ff8b5a8f9b73b3f2511623d98c8d")
(setf (get vision_based_picking::CalibrateResponse :datatype-) "vision_based_picking/CalibrateResponse")
(setf (get vision_based_picking::CalibrateResponse :definition-)
      "string the_request
int16 cam_index
---
float64 x
float64 y
float64 z

")



(provide :vision_based_picking/Calibrate "d402ff8b5a8f9b73b3f2511623d98c8d")


