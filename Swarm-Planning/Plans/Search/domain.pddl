;; Drones search their assigned forest areas and return to base.

(define (domain forest-drone-search)

    ;; Planning features used by these actions
    (:requirements
        :strips ;; Use actions with conditions and effects
        :typing ;; Give objects types such as drone and location
        :numeric-fluents ;; Use numbers such as battery, distance and height
        :negative-preconditions ;; Allow a condition to require that a fact is false
        :conditional-effects ;; Apply an effect only when its condition holds
    )

    ;; Types of objects used in this mission
    (:types
        drone
        location
        forest-area
        controller
    )

    ;; Facts that can be true or false
    (:predicates
        (at ?d - drone ?l - location) ;; Where the drone is
        (forest ?f - forest-area) ;; An area that must be searched
        (area-location ?a - forest-area ?l - location) ;; The location linked to a search area
        (area-covered ?a - forest-area) ;; The area has been searched
        (assigned ?d - drone ?a - forest-area) ;; The search area assigned to the drone
        (connected ?from - location ?to - location) ;; A route connects these locations
        (is-base ?l - location) ;; The return base
        (safe-route ?from - location ?to - location) ;; This route is allowed
        (flying ?d - drone) ;; The drone is flying
        (hovering ?d - drone) ;; The drone is holding its position
        (searching ?d - drone) ;; The drone is searching
        (search-completed ?d - drone) ;; The drone has finished its search
        (returning ?d - drone) ;; The drone is returning to base
        (gps-ok ?d - drone) ;; The drone has a GPS fix
        (communication-ok ?d - drone) ;; The model treats the ground-controller link as working
        (drone-healthy ?d - drone) ;; The drone passes the health check
        (gps-lost ?d - drone) ;; GPS loss has been recorded
        (communication-lost ?d - drone) ;; A communication failure has been recorded
        (health-failure ?d - drone) ;; Low drone health has been recorded
        (controller-notified ?d - drone) ;; The controller has been told about a failure
        (controller-return ?d - drone) ;; The controller has ordered a return
        (controller-hover ?d - drone) ;; The controller has ordered a hover
        (battery-insufficient ?d - drone) ;; The battery is not enough for the route leg
        (battery-checked ?d - drone) ;; The battery check has passed
        (safe-separation ?d1 - drone ?d2 - drone) ;; The gap between these drones has passed the check
        (separation-ok ?d - drone) ;; The drone has passed the separation check
        (collision-warning ?d1 - drone ?d2 - drone) ;; A collision warning has been recorded
        (forest-search-completed) ;; All required forest areas have been searched
        (mission-completed ?d - drone) ;; The drone has finished its mission
    )

    ;; Numbers used by the planner
    (:functions
        (battery ?d - drone) ;; Remaining battery in planning units
        (max-battery ?d - drone) ;; Full battery in planning units
        (minimum-return-battery ?d - drone) ;; Battery reserve for returning
        (energy-required ?from - location ?to - location) ;; Battery needed for this route leg
        (distance ?from - location ?to - location) ;; Distance along this route leg
        (drone-width ?d - drone) ;; Width of the drone
        (minimum-drone-separation ?d1 - drone ?d2 - drone) ;; Smallest allowed gap between two drones
        (forest-length ?f - forest-area) ;; Length of the search area
        (forest-width ?f - forest-area) ;; Width of the search area
        (coverage-width ?d - drone) ;; Width covered in one search pass
        (health ?d - drone) ;; Current health score
        (minimum-health ?d - drone) ;; Lowest allowed health score
        (hover-energy ?d - drone) ;; Battery used by one hover step
        (latitude ?l - location) ;; North-south map coordinate
        (longitude ?l - location) ;; East-west map coordinate
    )

    ;; Check that the battery is enough for the next move
    (:action check-battery
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?from - location
             ?to - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (at ?d ?from)
                (connected ?from ?to)
                (> (battery ?d)
                   (energy-required ?from ?to))
            )
        ;; These facts and values change after the action
        :effect
            (battery-checked ?d)
    )

    ;; Record that the battery is not enough for the next move
    (:action notify-insufficient-battery
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?from - location
             ?to - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (at ?d ?from)
                (connected ?from ?to)
                (<= (battery ?d)
                    (energy-required ?from ?to))
            )
        ;; These facts and values change after the action
        :effect
            (battery-insufficient ?d)
    )

    ;; Check that two drones are far enough apart
    (:action check-drone-separation
        ;; Objects used by this action
        :parameters
            (?d1 - drone
             ?d2 - drone
             ?l1 - location
             ?l2 - location)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
        :effect
            (and
                (safe-separation ?d1 ?d2)
                (separation-ok ?d1)
                (separation-ok ?d2)
            )
    )

    ;; Start searching the area assigned to the drone
    (:action start-search
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?a - forest-area
             ?l - location)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
        :effect
            (and
                (searching ?d)
                (flying ?d)
            )
    )

    ;; Mark the assigned area and the drone search as complete
    (:action cover-area
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?a - forest-area
             ?l - location)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
        :effect
            (and
                (area-covered ?a)
                (search-completed ?d)
                (not (searching ?d))
            )
    )

    ;; Move to the next search location and use the route battery
    (:action move-search
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?from - location
             ?to - location)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
        :effect
            (and
                (at ?d ?to)
                (not (at ?d ?from))
                (flying ?d)
                (decrease
                    (battery ?d)
                    (energy-required ?from ?to)
                )
                (not (battery-checked ?d))
                (not (separation-ok ?d))
            )
    )

    ;; Mark the forest search complete after all areas are covered
    (:action complete-forest-search
        ;; Objects used by this action
        :parameters
            (?a1 - forest-area
             ?a2 - forest-area)
        ;; These conditions must hold before the action
        :precondition
            (and
                (area-covered ?a1)
                (area-covered ?a2)
                (not (= ?a1 ?a2))
            )
        ;; These facts and values change after the action
        :effect
            (forest-search-completed)
    )

    ;; Return to the base after finishing the route or search
    (:action return-to-base
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?current - location
             ?base - location)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
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

    ;; Finish the drone mission after it returns to base
    (:action complete-drone-mission
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?base - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (at ?d ?base)
                (returning ?d)
                (forest-search-completed)
            )
        ;; These facts and values change after the action
        :effect
            (and
                (mission-completed ?d)
                (not (returning ?d))
            )
    )

    ;; Mark the drone as returning when its battery reaches the reserve
    (:action emergency-low-battery-return
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (<=
                    (battery ?d)
                    (minimum-return-battery ?d)
                )
            )
        ;; These facts and values change after the action
        :effect
            (returning ?d)
    )

    ;; Record GPS loss during flight
    (:action detect-gps-loss
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (flying ?d)
                (not (gps-ok ?d))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (gps-lost ?d)
                (not (flying ?d))
            )
    )

    ;; Record loss of contact with the ground controller
    (:action detect-communication-loss
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (flying ?d)
                (not (communication-ok ?d))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (communication-lost ?d)
                (not (flying ?d))
            )
    )

    ;; Record a health score below the allowed limit
    (:action detect-health-failure
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (flying ?d)
                (<
                    (health ?d)
                    (minimum-health ?d)
                )
            )
        ;; These facts and values change after the action
        :effect
            (and
                (health-failure ?d)
                (not (flying ?d))
            )
    )
)
