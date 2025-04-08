(define (domain robplan)
  (:requirements :typing :durative-actions :strips :fluents)
  
  (:types  
     turtlebot robot camera camera_eo camera_ir robo_arm charger - vehicle
     vehicle photo valve pump pipe sound gas_ind obj battery - subject
     city_location city - location
     waypoint battery_station - city_location
     route
  )

  (:predicates
    (at ?physical_obj1 - subject ?location1 - location)
    (available ?vehicle1 - vehicle)
    (available ?camera1 - camera)
    (available ?arm - robo_arm)
    (connects ?route1 - route ?location1 - location ?location2 - location)
    (route_available ?route1 - route)
    (in_city ?location1 - location ?city1 - city)

    ;; inspection status
    (no_photo ?subject1 - subject)
    (photo ?subject1 - subject)

    (no_seals_check ?subject1 - subject)
    (seals_check ?subject1 - subject)

    (charged ?v - robot)
    (needs_charge ?v - robot)
  )

  (:functions 
    (distance ?O - location ?L - location)
    (route-length ?R - route)
    (speed ?V - vehicle)
    (battery-level ?V - robot)
  )

  ;; Move between waypoints
  (:durative-action move_robot
    :parameters (?V - robot ?O - location ?L - location ?R - route)
    :duration (= ?duration (/ (route-length ?R) (speed ?V)))
    :condition (and 
      (at start (at ?V ?O))
      (at start (connects ?R ?O ?L))
    )
    :effect (and 
      (at start (not (at ?V ?O)))
      (at end (at ?V ?L))
    )
  )

  ;; Inspect valve with EO camera
  (:durative-action check_seals_valve_picture_EO
    :parameters (?V - robot ?L - location ?G - camera_eo ?B - valve)
    :duration (= ?duration 10)
    :condition (and 
      (over all (at ?V ?L))
      (at start (at ?B ?L))
      (at start (available ?G))
      (at start (no_seals_check ?B))
    )
    :effect (and 
      (at start (not (no_seals_check ?B)))
      (at end (seals_check ?B))
    )
  )

  ;; Take photo of pump with IR camera
  (:durative-action take_pump_picture_IR
    :parameters (?V - robot ?L - location ?G - camera_ir ?P - pump)
    :duration (= ?duration 5)
    :condition (and 
      (over all (at ?V ?L))
      (at start (at ?P ?L))
      (at start (available ?G))
      (at start (no_photo ?P))
    )
    :effect (and 
      (at start (not (no_photo ?P)))
      (at end (photo ?P))
    )
  )

  ;; Charge robot at battery station
  (:durative-action charge_robot
    :parameters (?V - robot ?L - location ?C - charger)
    :duration (= ?duration 15)
    :condition (and
      (over all (at ?V ?L))
      (at start (at ?C ?L))
      (at start (needs_charge ?V))
    )
    :effect (and
      (at end (charged ?V))
      (at end (not (needs_charge ?V)))
    )
  )
)
