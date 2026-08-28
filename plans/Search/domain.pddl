(define (domain forest-drone-search)

    (:requirements
        :strips
        :typing
        :numeric-fluents
        :negative-preconditions
        :conditional-effects
    )

    ;; ============================================================
    ;; TYPES
    ;; ============================================================

    (:types
        drone
        location
        forest-area
        controller
    )


    ;; ============================================================
    ;; PREDICATES
    ;; ============================================================

    (:predicates

        ;; --------------------------------------------------------
        ;; Drone position
        ;; --------------------------------------------------------

        (at ?d - drone ?l - location)

        ;; --------------------------------------------------------
        ;; Forest areas
        ;; --------------------------------------------------------

        (forest ?f - forest-area)

        (area-location ?a - forest-area ?l - location)

        (area-covered ?a - forest-area)

        ;; --------------------------------------------------------
        ;; Coverage assignment
        ;; --------------------------------------------------------

        (assigned ?d - drone ?a - forest-area)

        ;; --------------------------------------------------------
        ;; Connectivity
        ;; --------------------------------------------------------

        (connected ?from - location ?to - location)

        ;; The one location drones launch from and must actually return to -
        ;; without this, nothing stops return-to-base (below) from being
        ;; used for an ordinary interior hop, since otherwise its
        ;; precondition looks just like move-search's.

        (is-base ?l - location)

        ;; --------------------------------------------------------
        ;; Safe movement
        ;; --------------------------------------------------------

        (safe-route ?from - location ?to - location)

        ;; --------------------------------------------------------
        ;; Drone states
        ;; --------------------------------------------------------

        (flying ?d - drone)

        (hovering ?d - drone)

        (searching ?d - drone)

        (search-completed ?d - drone)

        (returning ?d - drone)

        ;; --------------------------------------------------------
        ;; System health
        ;; --------------------------------------------------------

        (gps-ok ?d - drone)

        (communication-ok ?d - drone)

        (drone-healthy ?d - drone)

        ;; --------------------------------------------------------
        ;; Fault states
        ;; --------------------------------------------------------

        (gps-lost ?d - drone)

        (communication-lost ?d - drone)

        (health-failure ?d - drone)

        ;; --------------------------------------------------------
        ;; Controller
        ;; --------------------------------------------------------

        (controller-notified ?d - drone)

        (controller-return ?d - drone)

        (controller-hover ?d - drone)

        ;; --------------------------------------------------------
        ;; Battery
        ;; --------------------------------------------------------

        (battery-insufficient ?d - drone)

        (battery-checked ?d - drone)

        ;; --------------------------------------------------------
        ;; Collision avoidance
        ;; --------------------------------------------------------

        ;; `separation-ok` is what actually gates movement (see move-search /
        ;; return-to-base below) - `safe-separation` is kept as the
        ;; informational per-pair record `check-drone-separation` produces,
        ;; the same way `battery-insufficient` records a battery check's
        ;; outcome without itself blocking anything.

        (safe-separation ?d1 - drone ?d2 - drone)

        (separation-ok ?d - drone)

        (collision-warning ?d1 - drone ?d2 - drone)

        ;; --------------------------------------------------------
        ;; Mission
        ;; --------------------------------------------------------

        (forest-search-completed)

        (mission-completed ?d - drone)
    )


    ;; ============================================================
    ;; NUMERIC FUNCTIONS
    ;; ============================================================

    (:functions

        ;; --------------------------------------------------------
        ;; Battery
        ;; --------------------------------------------------------

        (battery ?d - drone)

        (max-battery ?d - drone)

        (minimum-return-battery ?d - drone)

        ;; --------------------------------------------------------
        ;; Movement energy
        ;; --------------------------------------------------------

        (energy-required ?from - location ?to - location)

        ;; --------------------------------------------------------
        ;; Distance
        ;; --------------------------------------------------------

        (distance ?from - location ?to - location)

        ;; --------------------------------------------------------
        ;; Drone physical properties
        ;; --------------------------------------------------------

        (drone-width ?d - drone)

        (minimum-drone-separation ?d1 - drone ?d2 - drone)

        ;; --------------------------------------------------------
        ;; Forest dimensions
        ;; --------------------------------------------------------

        (forest-length ?f - forest-area)

        (forest-width ?f - forest-area)

        ;; --------------------------------------------------------
        ;; Coverage
        ;; --------------------------------------------------------

        (coverage-width ?d - drone)

        ;; --------------------------------------------------------
        ;; Health
        ;; --------------------------------------------------------

        (health ?d - drone)

        (minimum-health ?d - drone)

        ;; --------------------------------------------------------
        ;; Hover energy
        ;; --------------------------------------------------------

        (hover-energy ?d - drone)

        ;; --------------------------------------------------------
        ;; Latitude and longitude
        ;; --------------------------------------------------------

        (latitude ?l - location)

        (longitude ?l - location)
    )


    ;; ============================================================
    ;; ACTION 1: CHECK BATTERY
    ;; ============================================================

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


    ;; ============================================================
    ;; ACTION 2: REPORT INSUFFICIENT BATTERY
    ;; ============================================================

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


    ;; ============================================================
    ;; ACTION 3: CHECK SAFE SEPARATION
    ;; ============================================================

    ;; Records that ?d1/?d2 are currently far enough apart, from their
    ;; *actual* current locations - not assumed true from the start. Both
    ;; drones' `separation-ok` comes from the same check because in this
    ;; two-drone domain there is only ever one "other" drone to be separated
    ;; from; `move-search`/`return-to-base` each consume their own drone's
    ;; flag and require a fresh one before the next hop (see below), so the
    ;; two lanes' progress has to actually stay paired up distance-wise for
    ;; the plan to go through at all.

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


    ;; ============================================================
    ;; ACTION 4: START SEARCH
    ;; ============================================================

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


    ;; ============================================================
    ;; ACTION 5: SEARCH / COVER FOREST AREA
    ;; ============================================================

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

                ;; Forest region is now searched
                (area-covered ?a)

                ;; Drone completed this search section
                (search-completed ?d)

                (not (searching ?d))
            )
    )


    ;; ============================================================
    ;; ACTION 6: MOVE TO NEXT SEARCH LOCATION
    ;; ============================================================

    ;; Deliberately does NOT require `search-completed`: the drone's very
    ;; first hop (base -> its assigned area) necessarily happens *before*
    ;; `cover-area` can ever fire (that action itself requires already being
    ;; at the assigned location), so gating movement on it would make the
    ;; mission unsolvable before it starts. Reaching the goal still forces
    ;; `start-search`/`cover-area` to happen somewhere in the plan, because
    ;; `forest-search-completed` (required to return to base at all) depends
    ;; on it - just not as a per-hop gate on every individual move.

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

                ;; Reset checks for next movement - a fresh separation check
                ;; is required before the drone is allowed to hop again
                (not (battery-checked ?d))

                (not (separation-ok ?d))
            )
    )


    ;; ============================================================
    ;; ACTION 7: COMPLETE FOREST SEARCH
    ;; ============================================================

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


    ;; ============================================================
    ;; ACTION 8: RETURN TO BASE
    ;; ============================================================

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


    ;; ============================================================
    ;; ACTION 9: COMPLETE DRONE MISSION
    ;; ============================================================

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


    ;; ============================================================
    ;; ACTION 10: LOW BATTERY RETURN
    ;; ============================================================

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


    ;; ============================================================
    ;; ACTION 11: GPS FAILURE
    ;; ============================================================

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


    ;; ============================================================
    ;; ACTION 12: COMMUNICATION FAILURE
    ;; ============================================================

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


    ;; ============================================================
    ;; ACTION 13: HEALTH FAILURE
    ;; ============================================================

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
