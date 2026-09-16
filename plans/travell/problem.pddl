(define (problem drone-mission-001)

    (:domain swarm-drone-mission)

    ;; Objects used in the mission
    (:objects

        drone1 - drone

        ground-controller - controller

        source - location
        destination - location

        waypoint1 - location
        waypoint2 - location
    )


    (:init

        ;; Drone starts at the source
        (at drone1 source)

        (source source)
        (destination destination)


        ;; Locations connected by possible routes
        (connected source destination)

        (connected source waypoint1)
        (connected waypoint1 waypoint2)
        (connected waypoint2 destination)

        (connected waypoint1 source)
        (connected waypoint2 waypoint1)
        (connected destination waypoint2)


        ;; Safe routes avoid the restricted direct route
        (safe-route source waypoint1)
        (safe-route waypoint1 waypoint2)
        (safe-route waypoint2 destination)

        (safe-route waypoint1 source)
        (safe-route waypoint2 waypoint1)
        (safe-route destination waypoint2)


        ;; Direct route is restricted
        (restricted-route source destination)


        ;; Drone starts in a normal operating state
        (gps-ok drone1)
        (communication-ok drone1)
        (drone-healthy drone1)


        ;; Geographic coordinates
        (= (latitude source) 13.0827)
        (= (longitude source) 80.2707)

        (= (latitude destination) 13.0674)
        (= (longitude destination) 80.2376)

        (= (latitude waypoint1) 13.0830)
        (= (longitude waypoint1) 80.2250)

        (= (latitude waypoint2) 13.0650)
        (= (longitude waypoint2) 80.2250)


        ;; Distances are provided by the mission-generation program
        (= (distance source destination) 4300)

        (= (distance source waypoint1) 3900)
        (= (distance waypoint1 waypoint2) 2100)
        (= (distance waypoint2 destination) 1800)

        (= (distance waypoint1 source) 3900)
        (= (distance waypoint2 waypoint1) 2100)
        (= (distance destination waypoint2) 1800)


        ;; Energy required for each route
        (= (energy-required source destination) 45)

        (= (energy-required source waypoint1) 20)
        (= (energy-required waypoint1 waypoint2) 12)
        (= (energy-required waypoint2 destination) 10)

        (= (energy-required waypoint1 source) 20)
        (= (energy-required waypoint2 waypoint1) 12)
        (= (energy-required destination waypoint2) 10)


        ;; Initial battery and battery limits
        (= (battery drone1) 85)
        (= (max-battery drone1) 100)

        ;; Start emergency return at or below 30% battery
        (= (minimum-return-battery drone1) 30)

        ;; Battery used for each hover action
        (= (hover-energy drone1) 2)


        ;; Initial drone health and minimum health
        (= (health drone1) 95)
        (= (minimum-health drone1) 40)


        ;; Waypoint 1 is inside the restricted area
        (no-fly-zone waypoint1)
    )


    (:goal

        (and

            ;; Mission is complete when the drone reaches the destination
            (mission-completed drone1)

            (at drone1 destination)
        )
    )
)