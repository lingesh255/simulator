(define (problem star-formation-mission-001)

    (:domain star-formation-drone-mission)

    ;; Objects used in the star drone show
    (:objects

        drone-lead  - drone
        drone-m1    - drone
        ground-controller - controller
        source      - location
    )


    (:init

        ;; All drones start landed at the source
        (at drone-lead source)
        (at drone-m1 source)

        (landed drone-lead)
        (landed drone-m1)

        (= (altitude drone-lead) 0)
        (= (altitude drone-m1) 0)

        (source source)


        ;; Formation slots: the leader on the star's top tip
        (formation-leader drone-lead)
        (star-member drone-m1)

        (has-slot drone-lead)
        (has-slot drone-m1)

        (= (slot-angle drone-lead) 0)
        (= (slot-angle drone-m1) 60)


        ;; Neighbor communication - each drone and the two either side of it
        ;; round the circle (rebuilt for the real drone count at run time)
        (ring-neighbor drone-lead drone-m1)
        (ring-neighbor drone-m1 drone-lead)

        (neighbor-comm-ok drone-lead)
        (neighbor-comm-ok drone-m1)


        ;; Laps of the show (set from the operator's input at run time)
        (= (laps-required) 1)
        (= (laps-completed) 0)


        ;; All drones start with normal health and system status
        (gps-ok drone-lead)
        (communication-ok drone-lead)
        (drone-healthy drone-lead)

        (gps-ok drone-m1)
        (communication-ok drone-m1)
        (drone-healthy drone-m1)

        (= (health drone-lead) 100)
        (= (health drone-m1) 100)

        (= (minimum-health drone-lead) 40)
        (= (minimum-health drone-m1) 40)


        ;; Initial battery
        (= (battery drone-lead) 100)
        (= (battery drone-m1) 100)

        (= (max-battery drone-lead) 100)
        (= (max-battery drone-m1) 100)

        ;; Battery level for emergency return
        (= (minimum-return-battery drone-lead) 30)
        (= (minimum-return-battery drone-m1) 30)


        ;; Drones start at 2 m and climb to 50 m
        (= (hover-altitude drone-lead) 2)
        (= (hover-altitude drone-m1) 2)

        (= (target-altitude drone-lead) 50)
        (= (target-altitude drone-m1) 50)

        ;; Battery used during the climb
        (= (climb-energy drone-lead) 4)
        (= (climb-energy drone-m1) 4)


        ;; Where the show is performed
        (= (latitude source)  18.3901547)
        (= (longitude source) 79.0495640)
    )


    (:goal

        (and

            ;; Every lap flown, every drone landed back at the source
            (star-mission-complete)
            (>= (laps-completed) (laps-required))

            (at drone-lead source)
            (at drone-m1 source)

            (mission-completed drone-lead)
            (mission-completed drone-m1)

            (landed drone-lead)
            (landed drone-m1)
        )
    )
)
