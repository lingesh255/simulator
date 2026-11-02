;; A middle drone and two higher arm drones follow one route.
;; The mission program fills in this example before planning.

(define (problem parabola-formation-mission-001)
    (:domain parabola-formation-drone-mission)

    ;; Drones and other objects in this example
    (:objects
        drone-vertex - drone
        drone-left   - drone
        drone-right  - drone
        ground-controller - controller
        source       - location
        destination  - location
    )

    ;; Starting facts and values
    (:init

        ;; Where the drone is
        (at drone-vertex source)
        (at drone-left   source)
        (at drone-right  source)

        ;; The drone is on the ground
        (landed drone-vertex)
        (landed drone-left)
        (landed drone-right)

        ;; Current height of the drone
        (= (altitude drone-vertex) 0)
        (= (altitude drone-left)   0)
        (= (altitude drone-right)  0)

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
        (formation-leader drone-vertex)

        ;; The drone takes the lower middle position
        (slot-vertex drone-vertex)

        ;; The drone takes a raised side position
        (arm-drone drone-left)
        (arm-drone drone-right)

        ;; The drone has an assigned formation position
        (has-slot drone-vertex)
        (has-slot drone-left)
        (has-slot drone-right)

        ;; Forward or backward distance from the leader
        (= (slot-along-offset drone-vertex) 0)

        ;; Sideways distance from the leader
        (= (slot-cross-offset drone-vertex) 0)

        ;; Forward or backward distance from the leader
        (= (slot-along-offset drone-left)   0)

        ;; Sideways distance from the leader
        (= (slot-cross-offset drone-left)  -10)

        ;; Forward or backward distance from the leader
        (= (slot-along-offset drone-right)  0)

        ;; Sideways distance from the leader
        (= (slot-cross-offset drone-right)  10)

        ;; Required height difference between an arm and the middle drone
        (= (arm-rise) 10)

        ;; Smallest allowed gap between drones
        (= (min-separation) 10)

        ;; Two drones need a link in the parabola
        (parabola-neighbor drone-vertex drone-left)
        (parabola-neighbor drone-left drone-vertex)
        (parabola-neighbor drone-vertex drone-right)
        (parabola-neighbor drone-right drone-vertex)
        (parabola-neighbor drone-left drone-right)
        (parabola-neighbor drone-right drone-left)

        ;; The model treats the required neighbour links as working
        (neighbor-comm-ok drone-vertex)
        (neighbor-comm-ok drone-left)
        (neighbor-comm-ok drone-right)

        ;; Distance between two drones in the formation
        (= (link-distance drone-vertex drone-left)  14.142136)
        (= (link-distance drone-left drone-vertex)  14.142136)
        (= (link-distance drone-vertex drone-right) 14.142136)
        (= (link-distance drone-right drone-vertex) 14.142136)
        (= (link-distance drone-left drone-right)   20)
        (= (link-distance drone-right drone-left)   20)

        ;; Largest allowed distance for a neighbour link
        (= (comm-range) 100)

        ;; The drone has a GPS fix
        (gps-ok drone-vertex)

        ;; The model treats the ground-controller link as working
        (communication-ok drone-vertex)

        ;; The drone passes the health check
        (drone-healthy drone-vertex)

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
        (= (health drone-vertex) 100)
        (= (health drone-left)   100)
        (= (health drone-right)  100)

        ;; Lowest allowed health score
        (= (minimum-health drone-vertex) 40)
        (= (minimum-health drone-left)   40)
        (= (minimum-health drone-right)  40)

        ;; Remaining battery in planning units
        (= (battery drone-vertex) 100)
        (= (battery drone-left)   100)
        (= (battery drone-right)  100)

        ;; Full battery in planning units
        (= (max-battery drone-vertex) 100)
        (= (max-battery drone-left)   100)
        (= (max-battery drone-right)  100)

        ;; Battery reserve for returning
        (= (minimum-return-battery drone-vertex) 30)
        (= (minimum-return-battery drone-left)   30)
        (= (minimum-return-battery drone-right)  30)

        ;; Height just after takeoff
        (= (hover-altitude drone-vertex) 2)
        (= (hover-altitude drone-left)   2)
        (= (hover-altitude drone-right)  2)

        ;; Assigned cruise height
        (= (target-altitude drone-vertex) 50)
        (= (target-altitude drone-left)   60)
        (= (target-altitude drone-right)  60)

        ;; Battery used to reach cruise height
        (= (climb-energy drone-vertex) 4)
        (= (climb-energy drone-left)   5)
        (= (climb-energy drone-right)  5)

        ;; Battery used by a member for one route leg
        (= (cruise-leg-energy drone-vertex) 12)
        (= (cruise-leg-energy drone-left)   12)
        (= (cruise-leg-energy drone-right)  12)

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

            ;; The parabola formation has been set up
            (parabola-established)

            ;; The drone has joined the formation
            (in-formation drone-vertex)
            (in-formation drone-left)
            (in-formation drone-right)

            ;; The drone has links to all its required neighbours
            (swarm-comm-ready drone-vertex)
            (swarm-comm-ready drone-left)
            (swarm-comm-ready drone-right)

            ;; The drone has reached its assigned height
            (at-cruise-altitude drone-vertex)
            (at-cruise-altitude drone-left)
            (at-cruise-altitude drone-right)
            (>= (altitude drone-vertex) 50)
            (>= (altitude drone-left)   60)
            (>= (altitude drone-right)  60)

            ;; Where the formation is
            (formation-at destination)

            ;; Where the drone is
            (at drone-vertex destination)
            (at drone-left   destination)
            (at drone-right  destination)

            ;; The parabola mission is complete
            (parabola-mission-complete)

            ;; The drone has finished its mission
            (mission-completed drone-vertex)
            (mission-completed drone-left)
            (mission-completed drone-right)
        )
    )
)
