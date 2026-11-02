(define (problem parabola-formation-mission-001)
    (:domain parabola-formation-drone-mission)
    ;; Objects used in the parabola mission
    (:objects
        drone-vertex - drone
        drone-left   - drone
        drone-right  - drone
        ground-controller - controller
        source       - location
        destination  - location
    )
    (:init
        ;; All drones start landed at the source
        (at drone-vertex source)
        (at drone-left   source)
        (at drone-right  source)
        (landed drone-vertex)
        (landed drone-left)
        (landed drone-right)
        (= (altitude drone-vertex) 0)
        (= (altitude drone-left)   0)
        (= (altitude drone-right)  0)
        (source source)
        (destination destination)
        ;; Direct route between source and destination
        (connected source destination)
        (connected destination source)
        (safe-route source destination)
        (safe-route destination source)
        ;; The vertex leads from the middle; one arm drone either side
        (formation-leader drone-vertex)
        (slot-vertex drone-vertex)
        (arm-drone drone-left)
        (arm-drone drone-right)
        (has-slot drone-vertex)
        (has-slot drone-left)
        (has-slot drone-right)
        ;; All three on one line across the route, 10 m apart
        (= (slot-along-offset drone-vertex) 0)
        (= (slot-cross-offset drone-vertex) 0)
        (= (slot-along-offset drone-left)   0)
        (= (slot-cross-offset drone-left)  -10)
        (= (slot-along-offset drone-right)  0)
        (= (slot-cross-offset drone-right)  10)
        ;; Parabola shape - each arm 10 m above the vertex (50 m -> 60 m),
        ;; no two drones closer than 10 m
        (= (arm-rise) 10)
        (= (min-separation) 10)
        ;; Communication - every drone talks to both of the others: each
        ;; arm to the vertex, and the two arms to each other
        (parabola-neighbor drone-vertex drone-left)
        (parabola-neighbor drone-left drone-vertex)
        (parabola-neighbor drone-vertex drone-right)
        (parabola-neighbor drone-right drone-vertex)
        (parabola-neighbor drone-left drone-right)
        (parabola-neighbor drone-right drone-left)
        (neighbor-comm-ok drone-vertex)
        (neighbor-comm-ok drone-left)
        (neighbor-comm-ok drone-right)
        ;; Straight-line gaps in the formation: vertex to an arm is 10 m
        ;; across and 10 m up, arm to arm is 20 m across at the same height
        (= (link-distance drone-vertex drone-left)  14.142136)
        (= (link-distance drone-left drone-vertex)  14.142136)
        (= (link-distance drone-vertex drone-right) 14.142136)
        (= (link-distance drone-right drone-vertex) 14.142136)
        (= (link-distance drone-left drone-right)   20)
        (= (link-distance drone-right drone-left)   20)
        ;; Drone-to-drone link range
        (= (comm-range) 100)
        ;; All drones start with normal health and system status
        (gps-ok drone-vertex)
        (communication-ok drone-vertex)
        (drone-healthy drone-vertex)
        (gps-ok drone-left)
        (communication-ok drone-left)
        (drone-healthy drone-left)
        (gps-ok drone-right)
        (communication-ok drone-right)
        (drone-healthy drone-right)
        (= (health drone-vertex) 100)
        (= (health drone-left)   100)
        (= (health drone-right)  100)
        (= (minimum-health drone-vertex) 40)
        (= (minimum-health drone-left)   40)
        (= (minimum-health drone-right)  40)
        ;; Initial battery
        (= (battery drone-vertex) 100)
        (= (battery drone-left)   100)
        (= (battery drone-right)  100)
        (= (max-battery drone-vertex) 100)
        (= (max-battery drone-left)   100)
        (= (max-battery drone-right)  100)
        ;; Battery level for emergency return
        (= (minimum-return-battery drone-vertex) 30)
        (= (minimum-return-battery drone-left)   30)
        (= (minimum-return-battery drone-right)  30)
        ;; Drones start at 2 m; the vertex climbs to 50 m, the arms to 60 m
        (= (hover-altitude drone-vertex) 2)
        (= (hover-altitude drone-left)   2)
        (= (hover-altitude drone-right)  2)
        (= (target-altitude drone-vertex) 50)
        (= (target-altitude drone-left)   60)
        (= (target-altitude drone-right)  60)
        ;; Battery used during the climb (the arms climb 10 m further)
        (= (climb-energy drone-vertex) 4)
        (= (climb-energy drone-left)   5)
        (= (climb-energy drone-right)  5)
        ;; Battery used by each drone for one cruise leg
        (= (cruise-leg-energy drone-vertex) 12)
        (= (cruise-leg-energy drone-left)   12)
        (= (cruise-leg-energy drone-right)  12)
        ;; Source and destination coordinates
        (= (latitude source)       18.3901547)
        (= (longitude source)      79.0495640)
        (= (latitude destination)  18.3876026)
        (= (longitude destination) 79.0816646)
        ;; Direct route distance and energy cost
        (= (distance source destination) 3399.03)
        (= (distance destination source) 3399.03)
        (= (energy-required source destination) 10.00)
        (= (energy-required destination source) 10.00)
    )
    (:goal
        (and
            ;; Parabola is established
            (parabola-established)
            (in-formation drone-vertex)
            (in-formation drone-left)
            (in-formation drone-right)
            ;; Every drone has a working link with both of the others
            (swarm-comm-ready drone-vertex)
            (swarm-comm-ready drone-left)
            (swarm-comm-ready drone-right)
            ;; Vertex at 50 m, both arms at 60 m
            (at-cruise-altitude drone-vertex)
            (at-cruise-altitude drone-left)
            (at-cruise-altitude drone-right)
            (>= (altitude drone-vertex) 50)
            (>= (altitude drone-left)   60)
            (>= (altitude drone-right)  60)
            ;; Formation reached the destination
            (formation-at destination)
            (at drone-vertex destination)
            (at drone-left   destination)
            (at drone-right  destination)
            ;; Mission is completed
            (parabola-mission-complete)
            (mission-completed drone-vertex)
            (mission-completed drone-left)
            (mission-completed drone-right)
        )
    )
)
