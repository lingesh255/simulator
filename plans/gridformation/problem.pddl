(define (problem grid-formation-mission-001)

    (:domain grid-formation-drone-mission)

    ;; Objects used in the mission
    (:objects

        drone-lead   - drone
        drone-member - drone

        ground-controller - controller

        source      - location
        destination - location
    )

    ;; Initial state
    (:init

        ;; Both drones start at the source
        (at drone-lead   source)
        (at drone-member source)

        (landed drone-lead)
        (landed drone-member)

        (= (altitude drone-lead)   0)
        (= (altitude drone-member) 0)

        (source source)
        (destination destination)


        ;; Direct route between source and destination
        (connected source destination)
        (connected destination source)

        (safe-route source destination)
        (safe-route destination source)


        ;; Assign formation leader and grid member
        (formation-leader drone-lead)

        (grid-member drone-member)

        (has-slot drone-lead)
        (has-slot drone-member)

        ;; Grid slot positions
        (= (slot-along-offset drone-lead)     0)
        (= (slot-cross-offset drone-lead)     0)
        (= (slot-along-offset drone-member)   0)
        (= (slot-cross-offset drone-member)  10)


        ;; Neighbor communication
        (grid-neighbor drone-lead drone-member)
        (grid-neighbor drone-member drone-lead)

        (neighbor-comm-ok drone-lead)
        (neighbor-comm-ok drone-member)


        ;; Staggered launch settings
        (= (seconds-since-leader-airborne) 0)
        (= (grid-launch-delay) 5)


        ;; System health starts normal
        (gps-ok drone-lead)
        (communication-ok drone-lead)
        (drone-healthy drone-lead)

        (gps-ok drone-member)
        (communication-ok drone-member)
        (drone-healthy drone-member)

        (= (health drone-lead)    100)
        (= (health drone-member)  100)

        (= (minimum-health drone-lead)    40)
        (= (minimum-health drone-member)  40)


        ;; Initial battery
        (= (battery drone-lead)    100)
        (= (battery drone-member)  100)

        (= (max-battery drone-lead)    100)
        (= (max-battery drone-member)  100)

        (= (minimum-return-battery drone-lead)    30)
        (= (minimum-return-battery drone-member)  30)


        ;; Altitude settings
        (= (hover-altitude drone-lead)    2)
        (= (hover-altitude drone-member)  2)

        (= (target-altitude drone-lead)    50)
        (= (target-altitude drone-member)  50)

        ;; Battery used during climb
        (= (climb-energy drone-lead)    4)
        (= (climb-energy drone-member)  4)


        ;; Battery used for each cruise leg
        (= (cruise-leg-energy drone-lead)    12)
        (= (cruise-leg-energy drone-member)  14)


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


    ;; Mission goal
    (:goal

        (and

            ;; Grid formation is established
            (grid-formation-established)
            (in-formation drone-lead)
            (in-formation drone-member)

            ;; Both drones reach cruise altitude
            (at-cruise-altitude drone-lead)
            (at-cruise-altitude drone-member)

            (>= (altitude drone-lead)    50)
            (>= (altitude drone-member)  50)

            ;; Formation reaches the destination
            (formation-at destination)
            (at drone-lead   destination)
            (at drone-member destination)

            ;; Mission is completed
            (formation-mission-complete)
            (mission-completed drone-lead)
            (mission-completed drone-member)
        )
    )
)