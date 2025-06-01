(define (domain gantry_robot)
  (:requirements :strips :typing :durative-actions)

  (:types
    gantry robot tool location
  )

  (:constants
    none unknown - tool
  )

  (:predicates
    (gantry_at ?g - gantry ?l - location)
    (robot_at ?r - robot ?l - location)
    (tool_at ?t - tool ?l - location)
    (calibrated ?g - gantry)
    (uncalibrated ?g - gantry)
    (locked ?g - gantry)
    (unlocked ?g - gantry)
    (mounted ?r - robot)
    (unmounted ?r - robot)
    (tool_taken ?t - tool)
    (tool_free ?t - tool)
    (attached ?r - robot ?t - tool)
  )

  (:durative-action move_gantry
    :parameters (?g - gantry ?from ?to - location)
    :duration (= ?duration 1)
    :condition (and
      (at start (gantry_at ?g ?from))
      (over all (calibrated ?g))
      (over all (unlocked ?g))
    )
    :effect (and
      (at start (not (gantry_at ?g ?from)))
      (at end (gantry_at ?g ?to))
    )
  )

  (:durative-action calibrate
    :parameters (?g - gantry)
    :duration (= ?duration 1)
    :condition (and
      (over all (unlocked ?g))
      (at start (uncalibrated ?g))
    )
    :effect (and
      (at start (not (uncalibrated ?g)))
      (at end (calibrated ?g))
    )
  )

  (:durative-action lock
    :parameters (?g - gantry)
    :duration (= ?duration 1)
    :condition (and
      (at start (unlocked ?g))
    )
    :effect (and
      (at start (not (unlocked ?g)))
      (at end (locked ?g))
    )
  )

  (:durative-action unlock
    :parameters (?g - gantry)
    :duration (= ?duration 1)
    :condition (and
      (at start (locked ?g))
    )
    :effect (and
      (at start (not (locked ?g)))
      (at end (unlocked ?g))
    )
  )

  (:durative-action move_robot
    :parameters (?g - gantry ?r - robot ?from ?to - location)
    :duration (= ?duration 1)
    :condition (and
      (at start (robot_at ?r ?from))
      (over all (calibrated ?g))
      (over all (locked ?g))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
    )
  )

  (:durative-action mount
    :parameters (?r - robot ?t - tool ?g - gantry ?loc - location)
    :duration (= ?duration 1)
    :condition (and
      (over all (robot_at ?r ?loc))
      (over all (tool_at ?t ?loc))
      (over all (locked ?g))
      (at start (unmounted ?r))
      (at start (tool_free ?t))
      (at start (attached ?r none))
    )
    :effect (and
      (at start (not (unmounted ?r)))
      (at start (not (tool_free ?t)))
      (at start (not (attached ?r none)))
      (at end (attached ?r ?t))
      (at end (mounted ?r))
      (at end (tool_taken ?t))
    )
  )

  (:durative-action unmount
    :parameters (?r - robot ?t - tool ?g - gantry ?loc - location)
    :duration (= ?duration 1)
    :condition (and
      (over all (robot_at ?r ?loc))
      (over all (tool_at ?t ?loc))
      (over all (locked ?g))
      (at start (mounted ?r))
      (at start (tool_taken ?t))
      (at start (attached ?r ?t))
    )
    :effect (and
      (at start (not (mounted ?r)))
      (at start (not (tool_taken ?t)))
      (at start (not (attached ?r ?t)))
      (at end (attached ?r none))
      (at end (unmounted ?r))
      (at end (tool_free ?t))
    )
  )

  (:durative-action check_mounted
    :parameters (?r - robot)
    :duration (= ?duration 1)
    :condition (and
      (at start (attached ?r unknown))
    )
    :effect (and
      (at start (not (attached ?r unknown)))
      (at end (attached ?r none))
    )
  )
)