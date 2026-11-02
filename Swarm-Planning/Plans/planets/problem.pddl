;; Drones fly out in turn, circle on their assigned orbits and return.
;; The mission program fills in this example before planning.
;; Keep each starting fact on one line so added drones receive the full fact.

(define (problem planets-formation-mission-001)
    (:domain planets-formation-drone-mission)

    ;; Drones and other objects in this example
    (:objects
        drone-mercury - drone
        drone-venus   - drone
        ground-controller - controller
        source        - location
        destination   - location
    )

    ;; Starting facts and values
    (:init

        ;; Where the drone is
        (at drone-mercury source)
        (at drone-venus source)

        ;; The drone is on the ground
        (landed drone-mercury)
        (landed drone-venus)

        ;; Current height of the drone
        (= (altitude drone-mercury) 0)
        (= (altitude drone-venus) 0)

        ;; Where the mission starts
        (source source)

        ;; Where the mission goes
        (destination destination)

        ;; The drone leads the formation
        (formation-leader drone-mercury)

        ;; The drone flies one of the orbits
        (planet drone-venus)

        ;; The drone has an assigned orbit
        (has-orbit drone-mercury)
        (has-orbit drone-venus)

        ;; Distance from the orbit centre
        (= (orbit-radius drone-mercury) 20)
        (= (orbit-radius drone-venus) 30)

        ;; Speed along the orbit
        (= (orbit-speed drone-mercury) 5)
        (= (orbit-speed drone-venus) 6)

        ;; The drone turns anticlockwise when viewed from above
        (orbits-anticlockwise drone-mercury)

        ;; The drone turns clockwise when viewed from above
        (orbits-clockwise drone-venus)

        ;; Two drones have neighbouring orbits
        (orbit-neighbor drone-mercury drone-venus)
        (orbit-neighbor drone-venus drone-mercury)

        ;; The model treats the required neighbour links as working
        (neighbor-comm-ok drone-mercury)
        (neighbor-comm-ok drone-venus)

        ;; It is this drone's turn to fly out
        (out-turn drone-mercury)

        ;; The second drone flies out after the first
        (out-after drone-mercury drone-venus)

        ;; It is this drone's turn to fly back
        (back-turn drone-venus)

        ;; The second drone flies back after the first
        (back-after drone-venus drone-mercury)

        ;; Number of laps to fly
        (= (laps-required) 1)

        ;; Number of laps already counted as complete
        (= (laps-completed) 0)

        ;; The drone has a GPS fix
        (gps-ok drone-mercury)

        ;; The model treats the ground-controller link as working
        (communication-ok drone-mercury)

        ;; The drone passes the health check
        (drone-healthy drone-mercury)

        ;; The drone has a GPS fix
        (gps-ok drone-venus)

        ;; The model treats the ground-controller link as working
        (communication-ok drone-venus)

        ;; The drone passes the health check
        (drone-healthy drone-venus)

        ;; Current health score
        (= (health drone-mercury) 100)
        (= (health drone-venus) 100)

        ;; Lowest allowed health score
        (= (minimum-health drone-mercury) 40)
        (= (minimum-health drone-venus) 40)

        ;; Remaining battery in planning units
        (= (battery drone-mercury) 100)
        (= (battery drone-venus) 100)

        ;; Full battery in planning units
        (= (max-battery drone-mercury) 100)
        (= (max-battery drone-venus) 100)

        ;; Battery reserve for returning
        (= (minimum-return-battery drone-mercury) 30)
        (= (minimum-return-battery drone-venus) 30)

        ;; Height just after takeoff
        (= (hover-altitude drone-mercury) 2)
        (= (hover-altitude drone-venus) 2)

        ;; Assigned cruise height
        (= (target-altitude drone-mercury) 50)
        (= (target-altitude drone-venus) 50)

        ;; Battery used to reach cruise height
        (= (climb-energy drone-mercury) 4)
        (= (climb-energy drone-venus) 4)

        ;; Battery used for a flight between source and show
        (= (transit-energy drone-mercury) 10)
        (= (transit-energy drone-venus) 10)

        ;; North-south map coordinate
        (= (latitude source)  18.3901547)

        ;; East-west map coordinate
        (= (longitude source) 79.0495640)

        ;; North-south map coordinate
        (= (latitude destination)  18.3928547)

        ;; East-west map coordinate
        (= (longitude destination) 79.0495640)
    )

    ;; Facts that must be true when the mission finishes
    (:goal
        (and

            ;; The orbit show mission is complete
            (planets-mission-complete)
            (>= (laps-completed) (laps-required))

            ;; The drone has finished its mission
            (mission-completed drone-mercury)
            (mission-completed drone-venus)
        )
    )
)
