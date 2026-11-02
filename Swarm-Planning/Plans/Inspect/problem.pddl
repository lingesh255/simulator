;; One drone inspects the road and follows it back to the source.
;; The mission program fills in this example before planning.

(define (problem road-inspection-mission-001)
    (:domain road-inspection-drone-mission)

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

        ;; The drone assigned to inspect the road
        (inspector drone1)

        ;; Current height of the drone
        (= (altitude drone1) 0)

        ;; Where the mission starts
        (source source)

        ;; Where the mission goes
        (destination destination)

        ;; A road section to inspect
        (road-segment source destination)

        ;; Distance along this route leg
        (= (distance source destination) 1000.00)
        (= (distance destination source) 1000.00)

        ;; Battery needed for this route leg
        (= (energy-required source destination) 20.00)
        (= (energy-required destination source) 20.00)

        ;; Number of road sections to inspect
        (= (segments-total) 1)

        ;; Number of road sections already inspected
        (= (segments-inspected) 0)

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

        ;; Remaining battery in planning units
        (= (battery drone1) 100)

        ;; Full battery in planning units
        (= (max-battery drone1) 100)

        ;; Battery reserve for returning
        (= (minimum-return-battery drone1) 30)

        ;; Height just after takeoff
        (= (hover-altitude drone1) 2)

        ;; Assigned cruise height
        (= (target-altitude drone1) 50)

        ;; Battery used to reach cruise height
        (= (climb-energy drone1) 4)

        ;; North-south map coordinate
        (= (latitude source)       18.3901547)

        ;; East-west map coordinate
        (= (longitude source)      79.0495640)

        ;; North-south map coordinate
        (= (latitude destination)  18.3876026)

        ;; East-west map coordinate
        (= (longitude destination) 79.0816646)
    )

    ;; Facts that must be true when the mission finishes
    (:goal
        (and

            ;; The whole road has been inspected
            (road-inspected)

            ;; Where the drone is
            (at drone1 source)

            ;; The drone is on the ground
            (landed drone1)

            ;; The drone has finished its mission
            (mission-completed drone1)
        )
    )
)
