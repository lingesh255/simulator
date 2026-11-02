;; Drones fly out in turn, perform the star show and return in turn.
;; The mission program fills in this example before planning.
;; Keep each starting fact on one line so added drones receive the full fact.

(define (problem star-formation-mission-001)
    (:domain star-formation-drone-mission)

    ;; Drones and other objects in this example
    (:objects
        drone-lead  - drone
        drone-m1    - drone
        ground-controller - controller
        source      - location
        destination - location
    )

    ;; Starting facts and values
    (:init

        ;; Where the drone is
        (at drone-lead source)
        (at drone-m1 source)

        ;; The drone is on the ground
        (landed drone-lead)
        (landed drone-m1)

        ;; Current height of the drone
        (= (altitude drone-lead) 0)
        (= (altitude drone-m1) 0)

        ;; Where the mission starts
        (source source)

        ;; Where the mission goes
        (destination destination)

        ;; The drone leads the formation
        (formation-leader drone-lead)

        ;; The drone belongs to the star show
        (star-member drone-m1)

        ;; The drone has an assigned formation position
        (has-slot drone-lead)
        (has-slot drone-m1)

        ;; Angle of the assigned position around the show centre
        (= (slot-angle drone-lead) 0)
        (= (slot-angle drone-m1) 60)

        ;; Two drones are neighbours around the show
        (ring-neighbor drone-lead drone-m1)
        (ring-neighbor drone-m1 drone-lead)

        ;; The model treats the required neighbour links as working
        (neighbor-comm-ok drone-lead)
        (neighbor-comm-ok drone-m1)

        ;; It is this drone's turn to fly out
        (out-turn drone-lead)

        ;; The second drone flies out after the first
        (out-after drone-lead drone-m1)

        ;; It is this drone's turn to fly back
        (back-turn drone-m1)

        ;; The second drone flies back after the first
        (back-after drone-m1 drone-lead)

        ;; Number of laps to fly
        (= (laps-required) 1)

        ;; Number of laps already counted as complete
        (= (laps-completed) 0)

        ;; The drone has a GPS fix
        (gps-ok drone-lead)

        ;; The model treats the ground-controller link as working
        (communication-ok drone-lead)

        ;; The drone passes the health check
        (drone-healthy drone-lead)

        ;; The drone has a GPS fix
        (gps-ok drone-m1)

        ;; The model treats the ground-controller link as working
        (communication-ok drone-m1)

        ;; The drone passes the health check
        (drone-healthy drone-m1)

        ;; Current health score
        (= (health drone-lead) 100)
        (= (health drone-m1) 100)

        ;; Lowest allowed health score
        (= (minimum-health drone-lead) 40)
        (= (minimum-health drone-m1) 40)

        ;; Remaining battery in planning units
        (= (battery drone-lead) 100)
        (= (battery drone-m1) 100)

        ;; Full battery in planning units
        (= (max-battery drone-lead) 100)
        (= (max-battery drone-m1) 100)

        ;; Battery reserve for returning
        (= (minimum-return-battery drone-lead) 30)
        (= (minimum-return-battery drone-m1) 30)

        ;; Height just after takeoff
        (= (hover-altitude drone-lead) 2)
        (= (hover-altitude drone-m1) 2)

        ;; Assigned cruise height
        (= (target-altitude drone-lead) 50)
        (= (target-altitude drone-m1) 50)

        ;; Battery used to reach cruise height
        (= (climb-energy drone-lead) 4)
        (= (climb-energy drone-m1) 4)

        ;; Battery used for a flight between source and show
        (= (transit-energy drone-lead) 10)
        (= (transit-energy drone-m1) 10)

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

            ;; The star show mission is complete
            (star-mission-complete)
            (>= (laps-completed) (laps-required))

            ;; The drone has finished its mission
            (mission-completed drone-lead)
            (mission-completed drone-m1)
        )
    )
)
