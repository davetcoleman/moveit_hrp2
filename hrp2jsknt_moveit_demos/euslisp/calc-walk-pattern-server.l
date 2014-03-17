#!/usr/bin/env roseus

(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "hrp2_moveit_msgs")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "trajectory_msgs")

(defun setup-hrp2jsknt ()
  (require :hrp2jsknt-utils "package://hrp2jsknt_description/euslisp/models/hrp2jsknt-utils.l")
  (if (not (boundp '*hrp2jsknt*))
      (setq *hrp2jsknt* (hrp2jsknt)))
  (send (car (send *hrp2jsknt* :links)) :newcoords (make-coords))
  (mapcar
   #'(lambda (l)
       (let* ((ec (send *hrp2jsknt* l :end-coords))
              (pl (send ec :parent)))
         (send pl :dissoc ec)
         (send (send pl :parent) :assoc ec)))
   '(:rleg :lleg))
  (print "create hrp2jsknt model")
  (objects (list *hrp2jsknt*)))

(defun apply-robot-states-to-robot (rs robot)
  (let* ((joint-name-list (send (send rs :joint_state) :name))
         (joint-angle-list (send (send rs :joint_state) :position))
         (sorted-joint-angle-list
          (mapcar #'(lambda (name) (rad2deg (elt joint-angle-list (position name joint-name-list :test #'equal))))
                  (send-all (send robot :joint-list) :name)))
         (virtual-joint-tf (elt (send (send rs :multi_dof_joint_state) :transforms) 0))
         (rc (make-coords :pos (scale 1e3 (float-vector (send (send virtual-joint-tf :translation) :x)
                                                        (send (send virtual-joint-tf :translation) :y)
                                                        (send (send virtual-joint-tf :translation) :z)))
                          :rot (quaternion2matrix
                                (float-vector (send (send virtual-joint-tf :rotation) :w)
                                              (send (send virtual-joint-tf :rotation) :x)
                                              (send (send virtual-joint-tf :rotation) :y)
                                              (send (send virtual-joint-tf :rotation) :z))))))
    ;; set angle-vector
    (send robot :angle-vector (concatenate float-vector sorted-joint-angle-list))
    ;; set root-link-coords
    (send (car (send robot :links)) :move-to rc :world)
    ))

(defun calc-foot-midcroods (rs robot)
  (apply-robot-states-to-robot rs robot)
  (apply #'midcoords 0.5 (send robot :legs :end-coords)))

(defun set-JointTrajectory-msg (msg robot avs dt)
  (send msg :joint_names (send-all (send robot :joint-list) :name))
  (let ((point-list)
        (time_from_start 0.0))
    (dolist (av avs)
      (let ((point (instance trajectory_msgs::JointTrajectoryPoint :init)))
        (send point :positions av)
        (send (send point :time_from_start) :sec time_from_start)
        (push point point-list))
      (setq time_from_start (+ time_from_start dt)))
    (send msg :points (reverse point-list)))
  (send (send msg :header) :frame_id "/odom"))

(defun set-MultiDOFJointTrajectory-msg (msg robot rcs dt)
  (send msg :joint_names (list "virtual_joint"))
  (let ((point-list)
        (time_from_start 0.0))
    (dolist (rc rcs)
      (let ((rc-point (instance trajectory_msgs::MultiDOFJointTrajectoryPoint :init)))
        (let ((tf (instance geometry_msgs::Transform :init))
              (v3 (instance geometry_msgs::Vector3 :init))
              (q4 (instance geometry_msgs::Quaternion :init))
              (axis-for-v3 '(:x :y :z))
              (axis-for-q4 '(:w :x :y :z)))
          (dolist (axis axis-for-v3)
            (send v3 axis (* 1e-3 (elt (send rc :worldpos) (position axis axis-for-v3)))))
          (dolist (axis axis-for-q4)
            (send q4 axis (elt (matrix2quaternion (send rc :worldrot)) (position axis axis-for-q4))))
          (send tf :translation v3)
          (send tf :rotation q4)
          (send rc-point :transforms (list tf))
          (send (send rc-point :time_from_start) :sec time_from_start))
        (push rc-point point-list))
      (setq time_from_start (+ time_from_start dt)))
    (send msg :points (reverse point-list)))
  (send (send msg :header) :frame_id "/odom"))


(defun calc-walk-pattern (req)
  (let* ((m (send req :response))
         (start-rs (send req :start_state))
         (goal-rs (send req :goal_state))
         ;; call calc-foot-midcoords for start after call it for goal, because start pose and position depends on the last one.
         (goal-foot-midcoords (calc-foot-midcroods goal-rs *hrp2jsknt*))
         (start-foot-midcoords (calc-foot-midcroods start-rs *hrp2jsknt*))
         (from-start-to-goal (send start-foot-midcoords :transformation goal-foot-midcoords))
         (dt 0.1)
         (walk-pattern))
    (objects (list *hrp2jsknt*))

    ;; calc walk pattern
    (let ((x (elt (send from-start-to-goal :worldpos) 0))
          (y (elt (send from-start-to-goal :worldpos) 1))
          (theta (rad2deg (elt (car (send from-start-to-goal :rpy-angle)) 0))) ;yaw. I'm not sure about 'car'.
          (ik-args
           (list
            ;; :additional-weight-list (list (list (car (last (send *hrp2jsknt* :rleg :links))) 0.0)
            ;;                               (list (car (last (send *hrp2jsknt* :lleg :links))) 0.0))
            :centroid-thre 15
            :init nil)))
      (format t "x : ~A[mm], y : ~A[mm], theta : ~A[deg]~%" x y theta)
      (setq walk-pattern (send *hrp2jsknt* :calc-walk-pattern-from-footstep-list
                               (send *hrp2jsknt* :go-pos-params->footstep-list x y theta)
                               :dt dt                 ;[s] time interval of preview-control
                               :default-step-time 1    ;[s]
                               :default-step-height 50 ;[mm]
                               :solve-angle-vector-args ik-args
                               :init-pose-function
                               #'(lambda () ())
                               :debug-view :no-message
                               ))
      (print "finish calculation of walking pattern"))

    ;; set response
    (let ((avs (mapcar #'(lambda (one-tick)
                           (mapcar #'deg2rad (concatenate cons (cadr (memq :angle-vector one-tick))))) walk-pattern))
          (rcs (mapcar #'(lambda (one-tick)
                           (cadr (memq :root-coords one-tick))) walk-pattern))
          (jt-msg (instance trajectory_msgs::JointTrajectory :init))
          (mdjt-msg (instance trajectory_msgs::MultiDOFJointTrajectory :init)))
      (set-JointTrajectory-msg jt-msg *hrp2jsknt* avs dt)
      (set-MultiDOFJointTrajectory-msg mdjt-msg *hrp2jsknt* rcs dt)
      (send (send m :trajectory) :joint_trajectory jt-msg)
      (send (send m :trajectory) :multi_dof_joint_trajectory mdjt-msg)
      (print "finish setting values to response"))
    m))


(setup-hrp2jsknt)
(ros::roseus "calc_walk_pattern_server")
(print "Starting walking server")
(ros::advertise-service "/generate_walking_service" hrp2_moveit_msgs::GetWalkingMotion #'calc-walk-pattern)
;; (ros::advertise-service "calc_walk_pattern" moveit_msgs::GetWalkingMotion #'calc-walk-pattern)
(do-until-key
 (x::window-main-one)
 (ros::spin-once))
