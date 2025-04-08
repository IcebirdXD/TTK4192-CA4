(define (problem robplan-problem)
  (:domain robplan)

  (:objects
    turtlebot0 - robot
    camera0 - camera
    camera_eo0 - camera_eo
    camera_ir0 - camera_ir
    charger0 charger1 - charger
    robo_arm0 - robo_arm
    battery0 - battery

    valve0 valve1 - valve
    pump0 pump1 - pump

    waypoint0 waypoint1 waypoint2 waypoint3 waypoint4 waypoint5 waypoint6 waypoint7 - waypoint

    d01 d02 d03 d04 d05 d06 d07 - route
    d10 d12 d13 d14 d15 d16 d17 - route
    d20 d21 d23 d24 d25 d26 d27 - route
    d30 d31 d32 d34 d35 d36 d37 - route
    d40 d41 d42 d43 d45 d46 d47 - route
    d50 d51 d52 d53 d54 d56 d57 - route
    d60 d61 d62 d63 d64 d65 d67 - route
    d70 d71 d72 d73 d74 d75 d76 - route
  )

(:init
(= (speed turtlebot0) 0.90)

(connects d01 waypoint0 waypoint1)
(connects d02 waypoint0 waypoint2)
(connects d03 waypoint0 waypoint3)
(connects d04 waypoint0 waypoint4)
(connects d05 waypoint0 waypoint5)
(connects d06 waypoint0 waypoint6)
(connects d07 waypoint0 waypoint7)

(connects d10 waypoint1 waypoint0)
(connects d12 waypoint1 waypoint2)
(connects d13 waypoint1 waypoint3)
(connects d14 waypoint1 waypoint4)
(connects d15 waypoint1 waypoint5)
(connects d16 waypoint1 waypoint6)
(connects d17 waypoint1 waypoint7)

(connects d20 waypoint2 waypoint0)
(connects d21 waypoint2 waypoint1)
(connects d23 waypoint2 waypoint3)
(connects d24 waypoint2 waypoint4)
(connects d25 waypoint2 waypoint5)
(connects d26 waypoint2 waypoint6)
(connects d27 waypoint2 waypoint7)

(connects d30 waypoint3 waypoint0)
(connects d31 waypoint3 waypoint1)
(connects d32 waypoint3 waypoint2)
(connects d34 waypoint3 waypoint4)
(connects d35 waypoint3 waypoint5)
(connects d36 waypoint3 waypoint6)
(connects d37 waypoint3 waypoint7)

(connects d40 waypoint4 waypoint0)
(connects d41 waypoint4 waypoint1)
(connects d42 waypoint4 waypoint2)
(connects d43 waypoint4 waypoint3)
(connects d45 waypoint4 waypoint5)
(connects d46 waypoint4 waypoint6)
(connects d47 waypoint4 waypoint7)

(connects d50 waypoint5 waypoint0)
(connects d51 waypoint5 waypoint1)
(connects d52 waypoint5 waypoint2)
(connects d53 waypoint5 waypoint3)
(connects d54 waypoint5 waypoint4)
(connects d56 waypoint5 waypoint6)
(connects d57 waypoint5 waypoint7)

(connects d60 waypoint6 waypoint0)
(connects d61 waypoint6 waypoint1)
(connects d62 waypoint6 waypoint2)
(connects d63 waypoint6 waypoint3)
(connects d64 waypoint6 waypoint4)
(connects d65 waypoint6 waypoint5)
(connects d67 waypoint6 waypoint7)

(connects d70 waypoint7 waypoint0)
(connects d71 waypoint7 waypoint1)
(connects d72 waypoint7 waypoint2)
(connects d73 waypoint7 waypoint3)
(connects d74 waypoint7 waypoint4)
(connects d75 waypoint7 waypoint5)
(connects d76 waypoint7 waypoint6)

(= (route-length d01)   2.78   )
(= (route-length d02)   9.36   )
(= (route-length d03)   6.63   )
(= (route-length d04)   12.27  )
(= (route-length d05)   4.68   )
(= (route-length d06)   8.01   )
(= (route-length d07)   8.73   )

(= (route-length d10)   2.78   )
(= (route-length d12)   9.45   )
(= (route-length d13)   5.48   )
(= (route-length d14)   10.40   )
(= (route-length d15)   5.30   )
(= (route-length d16)   6.18   )
(= (route-length d17)   9.18   )


(= (route-length d20)   9.36   )
(= (route-length d21)   9.45   )
(= (route-length d23)   4.83   )
(= (route-length d24)   8.00   )
(= (route-length d25)   4.70   )
(= (route-length d26)   6.40   )
(= (route-length d27)   1.29   )

(= (route-length d30)   6.63   )
(= (route-length d31)   5.48   )
(= (route-length d32)   4.83   )
(= (route-length d34)   5.92   )
(= (route-length d35)   3.69   )
(= (route-length d36)   2.15   )
(= (route-length d37)   5.19   )

(= (route-length d40)   12.27   )
(= (route-length d41)   10.40   )
(= (route-length d42)   8.00   )
(= (route-length d43)   5.92   )
(= (route-length d45)   9.48   )
(= (route-length d46)   4.27   )
(= (route-length d47)   9.11   )


(= (route-length d50)   4.68   )
(= (route-length d51)   5.30   )
(= (route-length d52)   4.70   )
(= (route-length d53)   3.69   )
(= (route-length d54)   9.48   )
(= (route-length d56)   5.80   )
(= (route-length d57)   4.07   )


(= (route-length d60)   8.01   )
(= (route-length d61)   6.18   )
(= (route-length d62)   6.40   )
(= (route-length d63)   2.15   )
(= (route-length d64)   4.27   )
(= (route-length d65)   5.80   )
(= (route-length d67)   7.03   )


(= (route-length d70)   8.73   )
(= (route-length d71)   9.18   )
(= (route-length d72)   1.29   )
(= (route-length d73)   5.19   )
(= (route-length d74)   9.11   )
(= (route-length d75)   4.07   )
(= (route-length d76)   7.03   )


 ;; initial positions and availability
    (at turtlebot0 waypoint0)
    (at valve0 waypoint1)
    (at valve1 waypoint2)
    (at pump0 waypoint5)
    (at pump1 waypoint6)

    (available turtlebot0)
    (available camera_eo0)
    (available camera_ir0)
    (available robo_arm0)

    (no_seals_check valve0)
    (no_seals_check valve1)
    (no_photo pump0)
    (no_photo pump1)
    
    (needs_charge turtlebot0)
  )

  (:goal
    (and
      (at turtlebot0 waypoint4)
      (seals_check valve0)
      (seals_check valve1)
      (photo pump0)
      (photo pump1)
    )
  )

  (:metric minimize (total-time))
)
