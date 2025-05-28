(define (domain gantry_robot)
  (:requirements :strips :typing :durative-actions)

  (:types
    gantry robot tool location
  )

  (:predicates
    (at_gantry ?g - gantry ?l - location)
    (at_robot ?r - robot ?l - location)
    (calibrated ?g - gantry)
    (locked ?g - gantry)
    (mounted ?r - robot ?t - tool)
  )

  (:durative-action move_gantry
    :parameters (?g - gantry ?from - location ?to - location)
    :duration (= ?duration 1)
    :condition (and
      (at start (at_gantry ?g ?from))
    )
    :effect (and
      (at start (not (at_gantry ?g ?from)))
      (at end (at_gantry ?g ?to))
    )
  )

  (:durative-action calibrate
    :parameters (?g - gantry)
    :duration (= ?duration 1)
    :effect (and
      (at end (calibrated ?g))
    )
  )

  (:durative-action lock
    :parameters (?g - gantry)
    :duration (= ?duration 1)
    :effect (and
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
      (at end (not (locked ?g)))
    )
  )

  (:durative-action move_robot
    :parameters (?r - robot ?from - location ?to - location)
    :duration (= ?duration 1)
    :condition (and
      (at start (at_robot ?r ?from))
    )
    :effect (and
      (at start (not (at_robot ?r ?from)))
      (at end (at_robot ?r ?to))
    )
  )

  (:durative-action mount
    :parameters (?r - robot ?t - tool)
    :duration (= ?duration 1)
    :effect (and
      (at end (mounted ?r ?t))
    )
  )

  (:durative-action unmount
    :parameters (?r - robot ?t - tool)
    :duration (= ?duration 1)
    :condition (and
      (at start (mounted ?r ?t))
    )
    :effect (and
      (at end (not (mounted ?r ?t)))
    )
  )
)