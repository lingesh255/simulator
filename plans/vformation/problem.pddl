(define (problem v-formation-mission-001)

    (:domain v-formation-drone-mission)

    ;; Objects used in the V-formation mission
    (:objects

        drone-lead  - drone
        drone-left  - drone
        drone-right - drone
        ground-controller - controller
        source      - location
        destination - location

        ;; Possible compass directions for wing positions
        north       - direction
        north-east  - direction
        east        - direction
        south-east  - direction
        south       - direction
        south-west  - direction
        west        - direction
        north-west  - direction
    )


    (:init

        ;; All drones start landed at the source
        (at drone-lead  source)
        (at drone-left  source)
        (at drone-right source)

        (landed drone-lead)
        (landed drone-left)
        (landed drone-right)

        (= (altitude drone-lead)  0)
        (= (altitude drone-left)  0)
        (= (altitude drone-right) 0)

        (source source)
        (destination destination)


        ;; Direct route between source and destination
        (connected source destination)
        (connected destination source)

        (safe-route source destination)
        (safe-route destination source)


        ;; Assign the leader and two wing positions
        (formation-leader drone-lead)

        (slot-apex drone-lead)
        (slot-direction drone-left  north-west)
        (slot-direction drone-right south-west)

        (wing-drone drone-left)
        (wing-drone drone-right)

        (has-slot drone-lead)
        (has-slot drone-left)
        (has-slot drone-right)

        ;; V-formation slot offsets
        (= (slot-along-offset drone-lead)    0)
        (= (slot-cross-offset drone-lead)    0)
        (= (slot-along-offset drone-left)   -17.320508)
        (= (slot-cross-offset drone-left)   -10)
        (= (slot-along-offset drone-right)  -17.320508)
        (= (slot-cross-offset drone-right)   10)


        ;; Neighbor communication - each drone talks to its own left/right
        ;; neighbor along its wing (here, rank 1 either side of the leader)
        (wing-neighbor drone-lead drone-left)
        (wing-neighbor drone-left drone-lead)
        (wing-neighbor drone-lead drone-right)
        (wing-neighbor drone-right drone-lead)

        (neighbor-comm-ok drone-lead)
        (neighbor-comm-ok drone-left)
        (neighbor-comm-ok drone-right)


        ;; Leader waits 5 seconds before wing drones take off
        (= (seconds-since-leader-airborne) 0)
        (= (wing-launch-delay) 5)


        ;; All drones start with normal health and system status
        (gps-ok drone-lead)
        (communication-ok drone-lead)
        (drone-healthy drone-lead)

        (gps-ok drone-left)
        (communication-ok drone-left)
        (drone-healthy drone-left)

        (gps-ok drone-right)
        (communication-ok drone-right)
        (drone-healthy drone-right)

        (= (health drone-lead)  100)
        (= (health drone-left)  100)
        (= (health drone-right) 100)

        (= (minimum-health drone-lead)  40)
        (= (minimum-health drone-left)  40)
        (= (minimum-health drone-right) 40)


        ;; Initial battery
        (= (battery drone-lead)  100)
        (= (battery drone-left)  100)
        (= (battery drone-right) 100)

        (= (max-battery drone-lead)  100)
        (= (max-battery drone-left)  100)
        (= (max-battery drone-right) 100)

        ;; Battery level for emergency return
        (= (minimum-return-battery drone-lead)  30)
        (= (minimum-return-battery drone-left)  30)
        (= (minimum-return-battery drone-right) 30)


        ;; Drones start at 2 m and climb to 50 m
        (= (hover-altitude drone-lead)  2)
        (= (hover-altitude drone-left)  2)
        (= (hover-altitude drone-right) 2)

        (= (target-altitude drone-lead)  50)
        (= (target-altitude drone-left)  50)
        (= (target-altitude drone-right) 50)

        ;; Battery used during the climb
        (= (climb-energy drone-lead)  4)
        (= (climb-energy drone-left) 4)
        (= (climb-energy drone-right) 4)


        ;; Battery used by each drone for one cruise leg
        (= (cruise-leg-energy drone-lead)  12)
        (= (cruise-leg-energy drone-left)  14)
        (= (cruise-leg-energy drone-right) 14)


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

            ;; V-formation is established
            (v-formation-established)
            (in-formation drone-lead)
            (in-formation drone-left)
            (in-formation drone-right)

            ;; All drones reached 50 m cruise altitude
            (at-cruise-altitude drone-lead)
            (at-cruise-altitude drone-left)
            (at-cruise-altitude drone-right)

            (>= (altitude drone-lead)  50)
            (>= (altitude drone-left)  50)
            (>= (altitude drone-right) 50)

            ;; Formation reached the destination
            (formation-at destination)
            (at drone-lead  destination)
            (at drone-left  destination)
            (at drone-right destination)

            ;; Mission is completed
            (formation-mission-complete)
            (mission-completed drone-lead)
            (mission-completed drone-left)
            (mission-completed drone-right)
        )
    )
)