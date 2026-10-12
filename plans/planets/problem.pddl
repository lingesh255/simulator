(define (problem planets-formation-mission-001)

    (:domain planets-formation-drone-mission)

    ;; Objects used in the solar-system show
    (:objects

        drone-mercury - drone
        drone-venus   - drone
        ground-controller - controller
        source        - location
    )


    (:init

        ;; All drones start landed at the source
        (at drone-mercury source)
        (at drone-venus source)

        (landed drone-mercury)
        (landed drone-venus)

        (= (altitude drone-mercury) 0)
        (= (altitude drone-venus) 0)

        (source source)


        ;; Orbits: the leader on the innermost, 50 m further out per drone
        ;; (radii and directions rebuilt for the real drone count at run time)
        (formation-leader drone-mercury)
        (planet drone-venus)

        (has-orbit drone-mercury)
        (has-orbit drone-venus)

        (= (orbit-radius drone-mercury) 50)
        (= (orbit-radius drone-venus) 100)

        (orbits-anticlockwise drone-mercury)
        (orbits-clockwise drone-venus)


        ;; Neighbouring orbits
        (orbit-neighbor drone-mercury drone-venus)
        (orbit-neighbor drone-venus drone-mercury)

        (neighbor-comm-ok drone-mercury)
        (neighbor-comm-ok drone-venus)


        ;; Laps of the outermost orbit (set from the operator's input at run time)
        (= (laps-required) 1)
        (= (laps-completed) 0)


        ;; All drones start with normal health and system status
        (gps-ok drone-mercury)
        (communication-ok drone-mercury)
        (drone-healthy drone-mercury)

        (gps-ok drone-venus)
        (communication-ok drone-venus)
        (drone-healthy drone-venus)

        (= (health drone-mercury) 100)
        (= (health drone-venus) 100)

        (= (minimum-health drone-mercury) 40)
        (= (minimum-health drone-venus) 40)


        ;; Initial battery
        (= (battery drone-mercury) 100)
        (= (battery drone-venus) 100)

        (= (max-battery drone-mercury) 100)
        (= (max-battery drone-venus) 100)

        ;; Battery level for emergency return
        (= (minimum-return-battery drone-mercury) 30)
        (= (minimum-return-battery drone-venus) 30)


        ;; Drones start at 2 m and climb to 50 m
        (= (hover-altitude drone-mercury) 2)
        (= (hover-altitude drone-venus) 2)

        (= (target-altitude drone-mercury) 50)
        (= (target-altitude drone-venus) 50)

        ;; Battery used during the climb
        (= (climb-energy drone-mercury) 4)
        (= (climb-energy drone-venus) 4)


        ;; Where the sun is
        (= (latitude source)  18.3901547)
        (= (longitude source) 79.0495640)
    )


    (:goal

        (and

            ;; Every lap flown, every drone landed back at the source
            (planets-mission-complete)
            (>= (laps-completed) (laps-required))

            (at drone-mercury source)
            (at drone-venus source)

            (mission-completed drone-mercury)
            (mission-completed drone-venus)

            (landed drone-mercury)
            (landed drone-venus)
        )
    )
)
