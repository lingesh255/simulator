;; Drones follow a route with a leader and two wings.
;; The mission program fills in this example before planning.
;; Keep each starting fact on one line so added drones receive the full fact.

(define (problem v-formation-mission-001)
    (:domain v-formation-drone-mission)

    ;; Drones and other objects in this example
    (:objects
        drone-lead  - drone
        drone-left  - drone
        drone-right - drone
        ground-controller - controller
        source      - location
        destination - location
        north       - direction
        north-east  - direction
        east        - direction
        south-east  - direction
        south       - direction
        south-west  - direction
        west        - direction
        north-west  - direction
    )

    ;; Starting facts and values
    (:init

        ;; Where the drone is
        (at drone-lead  source)
        (at drone-left  source)
        (at drone-right source)

        ;; The drone is on the ground
        (landed drone-lead)
        (landed drone-left)
        (landed drone-right)

        ;; Current height of the drone
        (= (altitude drone-lead)  0)
        (= (altitude drone-left)  0)
        (= (altitude drone-right) 0)

        ;; Where the mission starts
        (source source)

        ;; Where the mission goes
        (destination destination)

        ;; A route connects these locations
        (connected source destination)
        (connected destination source)

        ;; This route is allowed
        (safe-route source destination)
        (safe-route destination source)

        ;; The drone leads the formation
        (formation-leader drone-lead)

        ;; The drone takes the front of the V
        (slot-apex drone-lead)

        ;; The side of the V assigned to the drone
        (slot-direction drone-left  north-west)
        (slot-direction drone-right south-west)

        ;; The drone belongs to a wing
        (wing-drone drone-left)
        (wing-drone drone-right)

        ;; The drone has an assigned formation position
        (has-slot drone-lead)
        (has-slot drone-left)
        (has-slot drone-right)

        ;; Forward or backward distance from the leader
        (= (slot-along-offset drone-lead)    0)

        ;; Sideways distance from the leader
        (= (slot-cross-offset drone-lead)    0)

        ;; Forward or backward distance from the leader
        (= (slot-along-offset drone-left)   -17.320508)

        ;; Sideways distance from the leader
        (= (slot-cross-offset drone-left)   -10)

        ;; Forward or backward distance from the leader
        (= (slot-along-offset drone-right)  -17.320508)

        ;; Sideways distance from the leader
        (= (slot-cross-offset drone-right)   10)

        ;; Two drones are neighbours along a wing
        (wing-neighbor drone-lead drone-left)
        (wing-neighbor drone-left drone-lead)
        (wing-neighbor drone-lead drone-right)
        (wing-neighbor drone-right drone-lead)

        ;; The model treats the required neighbour links as working
        (neighbor-comm-ok drone-lead)
        (neighbor-comm-ok drone-left)
        (neighbor-comm-ok drone-right)

        ;; Waiting time counted after the leader takes off
        (= (seconds-since-leader-airborne) 0)

        ;; How long wing drones wait before takeoff
        (= (wing-launch-delay) 5)

        ;; The drone has a GPS fix
        (gps-ok drone-lead)

        ;; The model treats the ground-controller link as working
        (communication-ok drone-lead)

        ;; The drone passes the health check
        (drone-healthy drone-lead)

        ;; The drone has a GPS fix
        (gps-ok drone-left)

        ;; The model treats the ground-controller link as working
        (communication-ok drone-left)

        ;; The drone passes the health check
        (drone-healthy drone-left)

        ;; The drone has a GPS fix
        (gps-ok drone-right)

        ;; The model treats the ground-controller link as working
        (communication-ok drone-right)

        ;; The drone passes the health check
        (drone-healthy drone-right)

        ;; Current health score
        (= (health drone-lead)  100)
        (= (health drone-left)  100)
        (= (health drone-right) 100)

        ;; Lowest allowed health score
        (= (minimum-health drone-lead)  40)
        (= (minimum-health drone-left)  40)
        (= (minimum-health drone-right) 40)

        ;; Remaining battery in planning units
        (= (battery drone-lead)  100)
        (= (battery drone-left)  100)
        (= (battery drone-right) 100)

        ;; Full battery in planning units
        (= (max-battery drone-lead)  100)
        (= (max-battery drone-left)  100)
        (= (max-battery drone-right) 100)

        ;; Battery reserve for returning
        (= (minimum-return-battery drone-lead)  30)
        (= (minimum-return-battery drone-left)  30)
        (= (minimum-return-battery drone-right) 30)

        ;; Height just after takeoff
        (= (hover-altitude drone-lead)  2)
        (= (hover-altitude drone-left)  2)
        (= (hover-altitude drone-right) 2)

        ;; Assigned cruise height
        (= (target-altitude drone-lead)  50)
        (= (target-altitude drone-left)  50)
        (= (target-altitude drone-right) 50)

        ;; Battery used to reach cruise height
        (= (climb-energy drone-lead)  4)
        (= (climb-energy drone-left) 4)
        (= (climb-energy drone-right) 4)

        ;; Battery used by a member for one route leg
        (= (cruise-leg-energy drone-lead)  12)
        (= (cruise-leg-energy drone-left)  14)
        (= (cruise-leg-energy drone-right) 14)

        ;; North-south map coordinate
        (= (latitude source)       18.3901547)

        ;; East-west map coordinate
        (= (longitude source)      79.0495640)

        ;; North-south map coordinate
        (= (latitude destination)  18.3876026)

        ;; East-west map coordinate
        (= (longitude destination) 79.0816646)

        ;; Distance along this route leg
        (= (distance source destination) 3399.03)
        (= (distance destination source) 3399.03)

        ;; Battery needed for this route leg
        (= (energy-required source destination) 10.00)
        (= (energy-required destination source) 10.00)
    )

    ;; Facts that must be true when the mission finishes
    (:goal
        (and

            ;; The V formation has been set up
            (v-formation-established)

            ;; The drone has joined the formation
            (in-formation drone-lead)
            (in-formation drone-left)
            (in-formation drone-right)

            ;; The drone has reached its assigned height
            (at-cruise-altitude drone-lead)
            (at-cruise-altitude drone-left)
            (at-cruise-altitude drone-right)
            (>= (altitude drone-lead)  50)
            (>= (altitude drone-left)  50)
            (>= (altitude drone-right) 50)

            ;; Where the formation is
            (formation-at destination)

            ;; Where the drone is
            (at drone-lead  destination)
            (at drone-left  destination)
            (at drone-right destination)

            ;; The formation mission is complete
            (formation-mission-complete)

            ;; The drone has finished its mission
            (mission-completed drone-lead)
            (mission-completed drone-left)
            (mission-completed drone-right)
        )
    )
)
