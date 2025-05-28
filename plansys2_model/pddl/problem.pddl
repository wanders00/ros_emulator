(define (problem gantry_robot_problem)
  (:domain gantry_robot)
  (:objects
    gantry1 - gantry
    robot1 - robot
    tool1 - tool
    a1 b1 - location
  )
  (:init
    (at_robot robot1 a1)
    (at_gantry gantry1 a1)
  )
  (:goal (and
    (at_robot robot1 b1)
    (mounted robot1 tool1)
    (calibrated gantry1)
    (locked gantry1)
  ))
)