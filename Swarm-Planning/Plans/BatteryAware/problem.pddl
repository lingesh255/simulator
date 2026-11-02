;; One drone flies from the source to the destination, charging on the way if
;; it has to. The mission program fills in this example before planning: the
;; picked source and destination, up to 5 charging stations, and every hop's
;; distance and battery use for the drone that flies it.

(define (problem battery-aware-mission-001)
    (:domain battery-aware-drone-mission)

    ;; Drones and other objects in this example
    (:objects
        drone1      - drone
        ground-controller - controller
        source      - location
        destination - location
    )

    ;; Starting facts and values
    (:init

        ;; Where the drone is
        (at drone1 source)

        ;; The drone is on the ground
        (landed drone1)

        ;; Where the mission starts
        (source source)

        ;; Where the mission goes
        (destination destination)

        ;; The drone may fly straight from one place to the other
        (hop source destination)

        ;; Straight-line distance of one hop, in metres
        (= (distance source destination) 10000.00)

        ;; Battery used by one hop: takeoff, climb, cruise and landing
        (= (hop-energy source destination) 30.000)

        ;; The drone has a GPS fix
        (gps-ok drone1)

        ;; The model treats the ground-controller link as working
        (communication-ok drone1)

        ;; The drone passes the health check
        (drone-healthy drone1)

        ;; Current health score
        (= (health drone1) 100)

        ;; Lowest allowed health score
        (= (minimum-health drone1) 40)

        ;; Remaining battery, in percent
        (= (battery drone1) 100)

        ;; Full battery, in percent
        (= (max-battery drone1) 100)

        ;; Charge this much more than a hop needs - it is left on landing
        (= (charge-margin drone1) 10)

        ;; Metres flown so far
        (= (total-distance) 0)

        ;; Number of stations landed at
        (= (charge-stops) 0)

        ;; North-south map coordinate
        (= (latitude source)       18.3901547)

        ;; East-west map coordinate
        (= (longitude source)      79.0495640)

        ;; North-south map coordinate
        (= (latitude destination)  18.3876026)

        ;; East-west map coordinate
        (= (longitude destination) 79.1440000)
    )

    ;; Facts that must be true when the mission finishes
    (:goal
        (and

            ;; Where the drone is
            (at drone1 destination)

            ;; The drone is on the ground
            (landed drone1)

            ;; The drone has finished its mission
            (mission-completed drone1)
        )
    )

    ;; The shortest route in metres flown, plus 1 for every station landed
    ;; at - so the drone never stops where it does not have to
    (:metric minimize (+ (total-distance) (charge-stops)))
)
