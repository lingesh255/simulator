;; Drones search their assigned forest areas and return to base.
;; The mission program fills in this example before planning.
;; Keep each starting fact on one line so added drones receive the full fact.

(define (problem forest-search-001)
    (:domain forest-drone-search)

    ;; Drones and other objects in this example
    (:objects
        drone1 - drone
        drone2 - drone
        ground-controller - controller
        forest-lane1 - forest-area
        forest-lane2 - forest-area
        base - location
    )

    ;; Starting facts and values
    (:init

        ;; An area that must be searched
        (forest forest-lane1)
        (forest forest-lane2)

        ;; Where the drone is
        (at drone1 base)
        (at drone2 base)

        ;; The return base
        (is-base base)

        ;; The search area assigned to the drone
        (assigned drone1 forest-lane1)
        (assigned drone2 forest-lane2)

        ;; The drone has a GPS fix
        (gps-ok drone1)

        ;; The model treats the ground-controller link as working
        (communication-ok drone1)

        ;; The drone passes the health check
        (drone-healthy drone1)

        ;; The drone has a GPS fix
        (gps-ok drone2)

        ;; The model treats the ground-controller link as working
        (communication-ok drone2)

        ;; The drone passes the health check
        (drone-healthy drone2)

        ;; Remaining battery in planning units
        (= (battery drone1) 100)

        ;; Full battery in planning units
        (= (max-battery drone1) 100)

        ;; Remaining battery in planning units
        (= (battery drone2) 100)

        ;; Full battery in planning units
        (= (max-battery drone2) 100)

        ;; Battery reserve for returning
        (= (minimum-return-battery drone1) 30)
        (= (minimum-return-battery drone2) 30)

        ;; Width of the drone
        (= (drone-width drone1) 50)
        (= (drone-width drone2) 50)

        ;; Width covered in one search pass
        (= (coverage-width drone1) 50)
        (= (coverage-width drone2) 50)

        ;; Smallest allowed gap between two drones
        (= (minimum-drone-separation drone1 drone2) 10)
        (= (minimum-drone-separation drone2 drone1) 10)

        ;; Current health score
        (= (health drone1) 100)
        (= (health drone2) 100)

        ;; Lowest allowed health score
        (= (minimum-health drone1) 40)
        (= (minimum-health drone2) 40)

        ;; Battery used by one hover step
        (= (hover-energy drone1) 2)
        (= (hover-energy drone2) 2)

        ;; The drone has passed the separation check
        (separation-ok drone1)
        (separation-ok drone2)

        ;; North-south map coordinate
        (= (latitude base) 13.0827)

        ;; East-west map coordinate
        (= (longitude base) 80.2707)
    )

    ;; Facts that must be true when the mission finishes
    (:goal
        (and

            ;; All required forest areas have been searched
            (forest-search-completed)

            ;; The drone has finished its mission
            (mission-completed drone1)
            (mission-completed drone2)

            ;; Where the drone is
            (at drone1 base)
            (at drone2 base)
        )
    )
)
