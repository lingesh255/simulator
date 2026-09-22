(define (problem forest-search-001)

    (:domain forest-drone-search)
    
    ;; Objects used in the search mission
    (:objects
        ;; Drones
        drone1 - drone
        drone2 - drone

        ;; Ground controller
        ground-controller - controller

        ;; One search area for each drone
        forest-lane1 - forest-area
        forest-lane2 - forest-area

        ;; Common starting and return location
        base - location
    )


    (:init

        ;; Forest search areas
        (forest forest-lane1)
        (forest forest-lane2)

        ;; Both drones start at the base
        (at drone1 base)
        (at drone2 base)
        (is-base base)

        ;; Assign one search area to each drone
        (assigned drone1 forest-lane1)
        (assigned drone2 forest-lane2)

        ;; Both drones start with normal system status
        (gps-ok drone1)
        (communication-ok drone1)
        (drone-healthy drone1)

        (gps-ok drone2)
        (communication-ok drone2)
        (drone-healthy drone2)

        ;; Initial battery
        (= (battery drone1) 100)
        (= (max-battery drone1) 100)

        (= (battery drone2) 100)
        (= (max-battery drone2) 100)

        ;; Battery level that triggers emergency return
        (= (minimum-return-battery drone1) 30)
        (= (minimum-return-battery drone2) 30)

        ;; Drone size and search coverage
        (= (drone-width drone1) 50)
        (= (drone-width drone2) 50)

        (= (coverage-width drone1) 50)
        (= (coverage-width drone2) 50)

        ;; Minimum distance required between the drones
        (= (minimum-drone-separation drone1 drone2) 10)
        (= (minimum-drone-separation drone2 drone1) 10)

        ;; Initial drone health
        (= (health drone1) 100)
        (= (health drone2) 100)

        (= (minimum-health drone1) 40)
        (= (minimum-health drone2) 40)

        ;; Battery used while hovering
        (= (hover-energy drone1) 2)
        (= (hover-energy drone2) 2)

        ;; Allow the first movement from the common base
        (separation-ok drone1)
        (separation-ok drone2)

        ;; Base coordinates
        (= (latitude base) 13.0827)
        (= (longitude base) 80.2707)
    )


    (:goal
        (and
            ;; Entire forest must be searched
            (forest-search-completed)

            ;; Both drone missions must be completed
            (mission-completed drone1)
            (mission-completed drone2)

            ;; Both drones must return to the base
            (at drone1 base)
            (at drone2 base)
        )
    )
)
