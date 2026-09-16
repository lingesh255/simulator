(define (domain forest-drone-search)

    (:requirements
        :strips
        :typing
        :numeric-fluents
        :negative-preconditions
        :conditional-effects
    )

    ;; Types used in the mission
    (:types
        drone
        location
        forest-area
        controller
    )

    ;; Facts that describe the drone and search mission
    (:predicates

        ;; Drone position
        (at ?d - drone ?l - location)

        ;; Forest areas and their coverage
        (forest ?f - forest-area)
        (area-location ?a - forest-area ?l - location)
        (area-covered ?a - forest-area)

        ;; Drone-to-area assignment
        (assigned ?d - drone ?a - forest-area)

        ;; Location connectivity
        (connected ?from - location ?to - location)

        ;; Base location
        (is-base ?l - location)

        ;; Safe routes
        (safe-route ?from - location ?to - location)

        ;; Drone states
        (flying ?d - drone)
        (hovering ?d - drone)
        (searching ?d - drone)
        (search-completed ?d - drone)
        (returning ?d - drone)

        ;; System health
        (gps-ok ?d - drone)
        (communication-ok ?d - drone)
        (drone-healthy ?d - drone)

        ;; Failure states
        (gps-lost ?d - drone)
        (communication-lost ?d - drone)
        (health-failure ?d - drone)

        ;; Controller commands
        (controller-notified ?d - drone)
        (controller-return ?d - drone)
        (controller-hover ?d - drone)

        ;; Battery status
        (battery-insufficient ?d - drone)
        (battery-checked ?d - drone)

        ;; Collision avoidance
        (safe-separation ?d1 - drone ?d2 - drone)
        (separation-ok ?d - drone)
        (collision-warning ?d1 - drone ?d2 - drone)

        ;; Mission status
        (forest-search-completed)
        (mission-completed ?d - drone)
    )

    ;; Numeric values used by the planner
    (:functions

        ;; Battery information
        (battery ?d - drone)
        (max-battery ?d - drone)
        (minimum-return-battery ?d - drone)

        ;; Energy and distance
        (energy-required ?from - location ?to - location)
        (distance ?from - location ?to - location)

        ;; Drone size and separation
        (drone-width ?d - drone)
        (minimum-drone-separation ?d1 - drone ?d2 - drone)

        ;; Forest dimensions
        (forest-length ?f - forest-area)
        (forest-width ?f - forest-area)

        ;; Search coverage width
        (coverage-width ?d - drone)

        ;; Drone health
        (health ?d - drone)
        (minimum-health ?d - drone)

        ;; Energy used while hovering
        (hover-energy ?d - drone)

        ;; Location coordinates
        (latitude ?l - location)
        (longitude ?l - location)
    )


    ;; Check if the drone has enough battery for a move
    (:action check-battery

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (connected ?from ?to)
                (> (battery ?d)
                   (energy-required ?from ?to))
            )

        :effect
            (battery-checked ?d)
    )


    ;; Record that the drone does not have enough battery
    (:action notify-insufficient-battery

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (connected ?from ?to)
                (<= (battery ?d)
                    (energy-required ?from ?to))
            )

        :effect
            (battery-insufficient ?d)
    )


    ;; Check the required distance between two drones
    (:action check-drone-separation

        :parameters
            (?d1 - drone
             ?d2 - drone
             ?l1 - location
             ?l2 - location)

        :precondition
            (and
                (at ?d1 ?l1)
                (at ?d2 ?l2)
                (not (= ?d1 ?d2))

                (>=
                    (distance ?l1 ?l2)
                    (minimum-drone-separation ?d1 ?d2)
                )
            )

        :effect
            (and
                (safe-separation ?d1 ?d2)
                (separation-ok ?d1)
                (separation-ok ?d2)
            )
    )


    ;; Start searching an assigned forest area
    (:action start-search

        :parameters
            (?d - drone
             ?a - forest-area
             ?l - location)

        :precondition
            (and
                (at ?d ?l)
                (forest ?a)
                (area-location ?a ?l)
                (assigned ?d ?a)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (not (area-covered ?a))
            )

        :effect
            (and
                (searching ?d)
                (flying ?d)
            )
    )


    ;; Mark the assigned forest area as searched
    (:action cover-area

        :parameters
            (?d - drone
             ?a - forest-area
             ?l - location)

        :precondition
            (and
                (at ?d ?l)
                (forest ?a)
                (area-location ?a ?l)
                (assigned ?d ?a)
                (searching ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (not (area-covered ?a))
            )

        :effect
            (and

                ;; Mark the forest area as covered
                (area-covered ?a)

                ;; Mark the drone's search as completed
                (search-completed ?d)

                (not (searching ?d))
            )
    )


    ;; Move the drone to the next search location
    (:action move-search

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (connected ?from ?to)
                (safe-route ?from ?to)
                (separation-ok ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (battery-checked ?d)
                (> (battery ?d)
                   (energy-required ?from ?to))
            )

        :effect
            (and

                ;; Move drone
                (at ?d ?to)
                (not (at ?d ?from))

                ;; Drone is flying
                (flying ?d)

                ;; Consume battery
                (decrease
                    (battery ?d)
                    (energy-required ?from ?to)
                )

                ;; Require fresh checks before the next move
                (not (battery-checked ?d))
                (not (separation-ok ?d))
            )
    )


    ;; Mark the forest search as complete
    (:action complete-forest-search

        :parameters
            (?a1 - forest-area
             ?a2 - forest-area)

        :precondition
            (and
                (area-covered ?a1)
                (area-covered ?a2)
                (not (= ?a1 ?a2))
            )

        :effect
            (forest-search-completed)
    )


    ;; Return the drone to the base after searching
    (:action return-to-base

        :parameters
            (?d - drone
             ?current - location
             ?base - location)

        :precondition
            (and
                (at ?d ?current)
                (is-base ?base)
                (connected ?current ?base)
                (safe-route ?current ?base)
                (separation-ok ?d)
                (forest-search-completed)
                (> (battery ?d)
                   (energy-required ?current ?base))
            )

        :effect
            (and
                (at ?d ?base)
                (not (at ?d ?current))
                (not (flying ?d))
                (returning ?d)

                (decrease
                    (battery ?d)
                    (energy-required ?current ?base)
                )
            )
    )


    ;; Complete the drone mission after returning to base
    (:action complete-drone-mission

        :parameters
            (?d - drone
             ?base - location)

        :precondition
            (and
                (at ?d ?base)
                (returning ?d)
                (forest-search-completed)
            )

        :effect
            (and
                (mission-completed ?d)
                (not (returning ?d))
            )
    )


    ;; Start returning when the battery is too low
    (:action emergency-low-battery-return

        :parameters
            (?d - drone)

        :precondition
            (and
                (<=
                    (battery ?d)
                    (minimum-return-battery ?d)
                )
            )

        :effect
            (returning ?d)
    )


    ;; Detect GPS failure during flight
    (:action detect-gps-loss

        :parameters
            (?d - drone)

        :precondition
            (and
                (flying ?d)
                (not (gps-ok ?d))
            )

        :effect
            (and
                (gps-lost ?d)
                (not (flying ?d))
            )
    )


    ;; Detect communication failure during flight
    (:action detect-communication-loss

        :parameters
            (?d - drone)

        :precondition
            (and
                (flying ?d)
                (not (communication-ok ?d))
            )

        :effect
            (and
                (communication-lost ?d)
                (not (flying ?d))
            )
    )


    ;; Detect when drone health becomes too low
    (:action detect-health-failure

        :parameters
            (?d - drone)

        :precondition
            (and
                (flying ?d)
                (<
                    (health ?d)
                    (minimum-health ?d)
                )
            )

        :effect
            (and
                (health-failure ?d)
                (not (flying ?d))
            )
    )

)