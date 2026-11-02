;; Drones follow a route in rows, columns and height layers.
;; The mission program fills in this example before planning.
;; Keep each starting fact on one line so added drones receive the full fact.

(define (problem cube-grid-formation-mission-001)
    (:domain cube-grid-formation-drone-mission)

    ;; Drones and other objects in this example
    (:objects
        drone-lead   - drone
        drone-member - drone
        ground-controller - controller
        source      - location
        destination - location
        layer0 - layer
    )

    ;; Starting facts and values
    (:init

        ;; Where the drone is
        (at drone-lead   source)
        (at drone-member source)

        ;; The drone is on the ground
        (landed drone-lead)
        (landed drone-member)

        ;; Current height of the drone
        (= (altitude drone-lead)   0)
        (= (altitude drone-member) 0)

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

        ;; The drone belongs to the cube
        (cube-member drone-member)

        ;; The drone has an assigned formation position
        (has-slot drone-lead)
        (has-slot drone-member)

        ;; Forward or backward distance from the leader
        (= (slot-along-offset drone-lead)     0)

        ;; Sideways distance from the leader
        (= (slot-cross-offset drone-lead)     0)

        ;; Layer number of the assigned position
        (= (slot-layer drone-lead)            0)

        ;; Extra height above the bottom layer
        (= (slot-lift-offset drone-lead)      0)

        ;; Forward or backward distance from the leader
        (= (slot-along-offset drone-member)   0)

        ;; Sideways distance from the leader
        (= (slot-cross-offset drone-member)  10)

        ;; Layer number of the assigned position
        (= (slot-layer drone-member)          0)

        ;; Extra height above the bottom layer
        (= (slot-lift-offset drone-member)    0)

        ;; Two drones are neighbours in the cube
        (cube-neighbor drone-lead drone-member)
        (cube-neighbor drone-member drone-lead)

        ;; The model treats the required neighbour links as working
        (neighbor-comm-ok drone-lead)
        (neighbor-comm-ok drone-member)

        ;; The layer assigned to the drone
        (in-layer drone-lead   layer0)
        (in-layer drone-member layer0)

        ;; The layer has been released for launch
        (layer-launched layer0)

        ;; The drone is allowed to take off
        (cleared-to-launch drone-lead)
        (cleared-to-launch drone-member)

        ;; Waiting time counted after the leader takes off
        (= (seconds-since-leader-airborne) 0)

        ;; How long cube members wait before takeoff
        (= (cube-launch-delay) 5)

        ;; The drone has a GPS fix
        (gps-ok drone-lead)

        ;; The model treats the ground-controller link as working
        (communication-ok drone-lead)

        ;; The drone passes the health check
        (drone-healthy drone-lead)

        ;; The drone has a GPS fix
        (gps-ok drone-member)

        ;; The model treats the ground-controller link as working
        (communication-ok drone-member)

        ;; The drone passes the health check
        (drone-healthy drone-member)

        ;; Current health score
        (= (health drone-lead)    100)
        (= (health drone-member)  100)

        ;; Lowest allowed health score
        (= (minimum-health drone-lead)    40)
        (= (minimum-health drone-member)  40)

        ;; Remaining battery in planning units
        (= (battery drone-lead)    100)
        (= (battery drone-member)  100)

        ;; Full battery in planning units
        (= (max-battery drone-lead)    100)
        (= (max-battery drone-member)  100)

        ;; Battery reserve for returning
        (= (minimum-return-battery drone-lead)    30)
        (= (minimum-return-battery drone-member)  30)

        ;; Height just after takeoff
        (= (hover-altitude drone-lead)    2)
        (= (hover-altitude drone-member)  2)

        ;; Assigned cruise height
        (= (target-altitude drone-lead)    50)
        (= (target-altitude drone-member)  50)

        ;; Battery used to reach cruise height
        (= (climb-energy drone-lead)    4)
        (= (climb-energy drone-member)  4)

        ;; Battery used by a member for one route leg
        (= (cruise-leg-energy drone-lead)    12)
        (= (cruise-leg-energy drone-member)  14)

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

            ;; The cube formation has been set up
            (cube-formation-established)

            ;; The drone has joined the formation
            (in-formation drone-lead)
            (in-formation drone-member)

            ;; The drone has reached its assigned height
            (at-cruise-altitude drone-lead)
            (at-cruise-altitude drone-member)

            ;; Where the formation is
            (formation-at destination)

            ;; Where the drone is
            (at drone-lead   destination)
            (at drone-member destination)

            ;; The formation mission is complete
            (formation-mission-complete)

            ;; The drone has finished its mission
            (mission-completed drone-lead)
            (mission-completed drone-member)
        )
    )
)
