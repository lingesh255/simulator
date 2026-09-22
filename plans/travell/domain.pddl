(define (domain swarm-drone-mission)

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
        waypoint
        controller
    )

    ;; Facts that describe the drone and mission state
    (:predicates

        ;; Drone position
        (at ?d - drone ?l - location)

        ;; Mission locations
        (source ?l - location)
        (destination ?l - location)

        ;; Waypoint information
        (waypoint-location ?w - waypoint ?l - location)

        ;; Route information
        (connected ?from - location ?to - location)
        (safe-route ?from - location ?to - location)
        (restricted-route ?from - location ?to - location)

        ;; Drone state
        (flying ?d - drone)
        (hovering ?d - drone)
        (mission-completed ?d - drone)

        ;; System health
        (gps-ok ?d - drone)
        (communication-ok ?d - drone)
        (drone-healthy ?d - drone)

        ;; Failure states
        (gps-lost ?d - drone)
        (communication-lost ?d - drone)
        (health-failure ?d - drone)

        ;; Controller commands and status
        (controller-notified ?d - drone)
        (controller-return ?d - drone)
        (controller-hover ?d - drone)
        (controller-sacrifice ?d - drone)

        ;; Warning messages
        (battery-insufficient ?d - drone)
        (health-warning ?d - drone)
        (gps-warning ?d - drone)
        (communication-warning ?d - drone)

        ;; Emergency states
        (emergency-return ?d - drone)
        (sacrificed ?d - drone)

        ;; Restricted area
        (no-fly-zone ?w - waypoint)

        ;; Mission checks
        (route-planned ?d - drone)
        (distance-checked ?d - drone)
        (battery-checked ?d - drone)
    )

    ;; Numeric values used by the planner
    (:functions

        ;; Battery information
        (battery ?d - drone)
        (max-battery ?d - drone)

        ;; Energy needed for a route
        (energy-required ?from - location ?to - location)

        ;; Distance between two locations
        (distance ?from - location ?to - location)

        ;; Drone health
        (health ?d - drone)
        (minimum-health ?d - drone)

        ;; Battery needed for emergency return
        (minimum-return-battery ?d - drone)

        ;; Battery used while hovering
        (hover-energy ?d - drone)

        ;; Energy used by the current route
        (route-energy ?d - drone)

        ;; Location coordinates
        (latitude ?l - location)
        (longitude ?l - location)
    )

    ;; Check the distance before planning the route
    (:action check-distance

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (connected ?from ?to)
            )

        :effect
            (distance-checked ?d)
    )

    ;; Check whether the drone has enough battery
    (:action check-battery

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (distance-checked ?d)
                (> (battery ?d)
                   (energy-required ?from ?to))
            )

        :effect
            (battery-checked ?d)
    )

    ;; Record that the battery is not enough
    (:action notify-insufficient-battery

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (distance-checked ?d)
                (<= (battery ?d)
                    (energy-required ?from ?to))
            )

        :effect
            (battery-insufficient ?d)
    )

    ;; Create a route only when all safety checks pass
    (:action plan-safe-route

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (connected ?from ?to)
                (safe-route ?from ?to)
                (battery-checked ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
            )

        :effect
            (route-planned ?d)
    )

    ;; Move the drone and reduce its battery
    (:action travel

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (at ?d ?from)
                (route-planned ?d)
                (safe-route ?from ?to)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (> (battery ?d)
                   (energy-required ?from ?to))
            )

        :effect
            (and

                ;; Move drone to the new location
                (at ?d ?to)
                (not (at ?d ?from))

                ;; Drone is now flying
                (flying ?d)

                ;; Reduce battery by route energy
                (decrease
                    (battery ?d)
                    (energy-required ?from ?to)
                )

                ;; Route must be planned again for the next move
                (not (route-planned ?d))
            )
    )

    ;; Complete the mission when the destination is reached
    (:action reach-destination

        :parameters
            (?d - drone
             ?dest - location)

        :precondition
            (and
                (at ?d ?dest)
                (destination ?dest)
            )

        :effect
            (and
                (mission-completed ?d)
                (not (flying ?d))
            )
    )

    ;; Detect a GPS failure during flight
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
                (gps-warning ?d)
                (not (flying ?d))
            )
    )

    ;; Detect a communication failure during flight
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
                (communication-warning ?d)
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
                (< (health ?d)
                   (minimum-health ?d))
            )

        :effect
            (and
                (health-failure ?d)
                (health-warning ?d)
                (not (flying ?d))
            )
    )

    ;; Inform the ground controller about a failure
    (:action notify-ground-controller

        :parameters
            (?d - drone
             ?c - controller)

        :precondition
            (or
                (gps-lost ?d)
                (communication-lost ?d)
                (health-failure ?d)
            )

        :effect
            (controller-notified ?d)
    )

    ;; Follow the controller's return command
    (:action controller-order-return

        :parameters
            (?d - drone)

        :precondition
            (and
                (controller-notified ?d)
                (controller-return ?d)
            )

        :effect
            (emergency-return ?d)
    )

    ;; Return the drone safely to the source
    (:action return-to-source

        :parameters
            (?d - drone
             ?current - location
             ?src - location)

        :precondition
            (and
                (at ?d ?current)
                (source ?src)
                (emergency-return ?d)
                (safe-route ?current ?src)
                (> (battery ?d)
                   (energy-required ?current ?src))
            )

        :effect
            (and
                (at ?d ?src)
                (not (at ?d ?current))
                (not (emergency-return ?d))

                (decrease
                    (battery ?d)
                    (energy-required ?current ?src)
                )
            )
    )

    ;; Follow the controller's hover command
    (:action controller-order-hover

        :parameters
            (?d - drone)

        :precondition
            (and
                (controller-notified ?d)
                (controller-hover ?d)
            )

        :effect
            (hovering ?d)
    )

    ;; Keep the drone hovering while enough battery remains
    (:action hover

        :parameters
            (?d - drone)

        :precondition
            (and
                (hovering ?d)
                (> (battery ?d)
                   (minimum-return-battery ?d))
            )

        :effect
            (decrease
                (battery ?d)
                (hover-energy ?d)
            )
    )

    ;; Start emergency return when battery becomes low
    (:action initiate-low-battery-return

        :parameters
            (?d - drone)

        :precondition
            (and
                (hovering ?d)
                (<= (battery ?d)
                    (minimum-return-battery ?d))
            )

        :effect
            (and
                (emergency-return ?d)
                (not (hovering ?d))
            )
    )

    ;; Emergency landing when the controller orders sacrifice
    (:action sacrifice-drone

        :parameters
            (?d - drone)

        :precondition
            (and
                (controller-notified ?d)
                (controller-sacrifice ?d)
            )

        :effect
            (and
                (sacrificed ?d)
                (not (flying ?d))
                (not (hovering ?d))
            )
    )

)