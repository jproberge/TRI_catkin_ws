;; Auto-generated. Do not edit!


(when (boundp 'vision_based_picking::Acquire)
  (if (not (find-package "VISION_BASED_PICKING"))
    (make-package "VISION_BASED_PICKING"))
  (shadow 'Acquire (find-package "VISION_BASED_PICKING")))
(unless (find-package "VISION_BASED_PICKING::ACQUIRE")
  (make-package "VISION_BASED_PICKING::ACQUIRE"))
(unless (find-package "VISION_BASED_PICKING::ACQUIREREQUEST")
  (make-package "VISION_BASED_PICKING::ACQUIREREQUEST"))
(unless (find-package "VISION_BASED_PICKING::ACQUIRERESPONSE")
  (make-package "VISION_BASED_PICKING::ACQUIRERESPONSE"))

(in-package "ROS")





(defclass vision_based_picking::AcquireRequest
  :super ros::object
  :slots (_the_request ))

(defmethod vision_based_picking::AcquireRequest
  (:init
   (&key
    ((:the_request __the_request) "")
    )
   (send-super :init)
   (setq _the_request (string __the_request))
   self)
  (:the_request
   (&optional __the_request)
   (if __the_request (setq _the_request __the_request)) _the_request)
  (:serialization-length
   ()
   (+
    ;; string _the_request
    4 (length _the_request)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _the_request
       (write-long (length _the_request) s) (princ _the_request s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _the_request
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _the_request (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass vision_based_picking::AcquireResponse
  :super ros::object
  :slots (_status ))

(defmethod vision_based_picking::AcquireResponse
  (:init
   (&key
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; int16 _status
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _status
       (write-word _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _status
     (setq _status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass vision_based_picking::Acquire
  :super ros::object
  :slots ())

(setf (get vision_based_picking::Acquire :md5sum-) "0472ad368258a75cc77be2698c6aee5a")
(setf (get vision_based_picking::Acquire :datatype-) "vision_based_picking/Acquire")
(setf (get vision_based_picking::Acquire :request) vision_based_picking::AcquireRequest)
(setf (get vision_based_picking::Acquire :response) vision_based_picking::AcquireResponse)

(defmethod vision_based_picking::AcquireRequest
  (:response () (instance vision_based_picking::AcquireResponse :init)))

(setf (get vision_based_picking::AcquireRequest :md5sum-) "0472ad368258a75cc77be2698c6aee5a")
(setf (get vision_based_picking::AcquireRequest :datatype-) "vision_based_picking/AcquireRequest")
(setf (get vision_based_picking::AcquireRequest :definition-)
      "string the_request
---
int16 status

")

(setf (get vision_based_picking::AcquireResponse :md5sum-) "0472ad368258a75cc77be2698c6aee5a")
(setf (get vision_based_picking::AcquireResponse :datatype-) "vision_based_picking/AcquireResponse")
(setf (get vision_based_picking::AcquireResponse :definition-)
      "string the_request
---
int16 status

")



(provide :vision_based_picking/Acquire "0472ad368258a75cc77be2698c6aee5a")


