(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
(jaxon-init)
(objects *jaxon*)

;; pose init
(send *jaxon* :reset-pose)
;; (send *jaxon* :reset-manip-pose)
(send *ri* :angle-vector (send *jaxon* :angle-vector) 5000)
(send *ri* :wait-interpolation)

;; locomotion init
(send *ri* :start-auto-balancer)
;; (send *ri* :start-st) ;; ST does not work in gazebo
(send *ri* :wait-interpolation)

;; locomotion test
(send *ri* :go-pos 0 0 0)
(send *ri* :wait-interpolation)

;; main program
(defun main ()
  ;; main program
     )
(main)
