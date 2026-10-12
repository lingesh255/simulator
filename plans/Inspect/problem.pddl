(define (problem road-inspection-mission-001)

    (:domain road-inspection-drone-mission)

    ;; Objects used in the road inspection
    (:objects

        drone1      - drone
        ground-controller - controller

        ;; The road's two ends (the waypoints between them are added at run
        ;; time from the real road's route)
        source      - location
        destination - location
    )


    (:init

        ;; The drone starts landed at the source end of the road
        (at drone1 source)
        (landed drone1)
        (inspector drone1)

        (= (altitude drone1) 0)

        (source source)
        (destination destination)


        ;; The road, as one segment here - replaced at run time by the real
        ;; road's segments
        (road-segment source destination)

        (= (distance source destination) 1000.00)
        (= (distance destination source) 1000.00)

        (= (energy-required source destination) 20.00)
        (= (energy-required destination source) 20.00)

        (= (segments-total) 1)
        (= (segments-inspected) 0)


        ;; The drone starts with normal health and system status
        (gps-ok drone1)
        (communication-ok drone1)
        (drone-healthy drone1)

        (= (health drone1) 100)
        (= (minimum-health drone1) 40)


        ;; Initial battery
        (= (battery drone1) 100)
        (= (max-battery drone1) 100)

        ;; Battery level for emergency return
        (= (minimum-return-battery drone1) 30)


        ;; The drone starts at 2 m and climbs to 50 m to inspect
        (= (hover-altitude drone1) 2)
        (= (target-altitude drone1) 50)

        ;; Battery used during the climb
        (= (climb-energy drone1) 4)


        ;; The road's two ends
        (= (latitude source)       18.3901547)
        (= (longitude source)      79.0495640)

        (= (latitude destination)  18.3876026)
        (= (longitude destination) 79.0816646)
    )


    (:goal

        (and

            ;; The whole road inspected, and the drone back and landed at the source
            (road-inspected)
            (at drone1 source)
            (landed drone1)
            (mission-completed drone1)
        )
    )
)
