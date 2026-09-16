(define (domain grid-formation-drone-mission)

    (:requirements
        :strips
        :typing
        :equality
        :numeric-fluents
        :negative-preconditions
        :disjunctive-preconditions
        :universal-preconditions
        :conditional-effects
    )

    ;; Types used in the mission
    (:types
        drone
        location
        controller
    )

    ;; Mission state and formation facts
    (:predicates

        ;; Drone position
        (at ?d - drone ?l - location)

        ;; Current formation position
        (formation-at ?l - location)

        ;; Mission locations and routes
        (source ?l - location)
        (destination ?l - location)

        (connected ?from - location ?to - location)
        (safe-route ?from - location ?to - location)

        ;; Flight state
        (landed ?d - drone)
        (airborne ?d - drone)
        (flying ?d - drone)
        (at-cruise-altitude ?d - drone)

        ;; Formation assignment
        (has-slot ?d - drone)
        (grid-member ?d - drone)
        (formation-leader ?d - drone)
        (in-formation ?d - drone)
        (grid-formation-established)
        (grid-closed)

        ;; Neighbor communication
        (grid-neighbor ?d1 - drone ?d2 - drone)
        (neighbor-comm-ok ?d - drone)

        ;; Pre-flight status
        (preflight-done ?d - drone)

        ;; System health
        (gps-ok ?d - drone)
        (communication-ok ?d - drone)
        (drone-healthy ?d - drone)

        ;; Failure states
        (gps-lost ?d - drone)
        (communication-lost ?d - drone)
        (health-failure ?d - drone)
        (controller-notified ?d - drone)

        ;; Mission status
        (mission-completed ?d - drone)
        (formation-mission-complete)
    )

    ;; Numeric values used by the planner
    (:functions

        ;; Battery
        (battery ?d - drone)
        (max-battery ?d - drone)
        (minimum-return-battery ?d - drone)

        ;; Altitude and energy
        (altitude ?d - drone)
        (hover-altitude ?d - drone)
        (target-altitude ?d - drone)
        (climb-energy ?d - drone)
        (cruise-leg-energy ?d - drone)
        (energy-required ?from - location ?to - location)
        (distance ?from - location ?to - location)

        ;; Health
        (health ?d - drone)
        (minimum-health ?d - drone)

        ;; Location coordinates
        (latitude ?l - location)
        (longitude ?l - location)

        ;; Grid slot offsets
        (slot-along-offset ?d - drone)
        (slot-cross-offset ?d - drone)

        ;; Staggered launch timing
        (seconds-since-leader-airborne)
        (grid-launch-delay)
    )

    ;; Check the drone before takeoff
    (:action pre-flight-check

        :parameters
            (?d - drone
             ?l - location)

        :precondition
            (and
                (at ?d ?l)
                (source ?l)
                (landed ?d)
                (has-slot ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (>= (health ?d) (minimum-health ?d))
                (> (battery ?d) (minimum-return-battery ?d))
            )

        :effect
            (preflight-done ?d)
    )

    ;; Take off from the source
    (:action takeoff

        :parameters
            (?d - drone
             ?l - location)

        :precondition
            (and
                (at ?d ?l)
                (source ?l)
                (landed ?d)
                (preflight-done ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)

                (or
                    (formation-leader ?d)
                    (>= (seconds-since-leader-airborne) (grid-launch-delay))
                )
            )

        :effect
            (and
                (not (landed ?d))
                (airborne ?d)
                (flying ?d)
                (assign (altitude ?d) (hover-altitude ?d))
            )
    )

    ;; Climb to the cruise altitude
    (:action climb-to-cruise-altitude

        :parameters
            (?d - drone)

        :precondition
            (and
                (airborne ?d)
                (not (at-cruise-altitude ?d))
                (< (altitude ?d) (target-altitude ?d))
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (> (battery ?d) (climb-energy ?d))
            )

        :effect
            (and
                (assign (altitude ?d) (target-altitude ?d))
                (at-cruise-altitude ?d)
                (decrease (battery ?d) (climb-energy ?d))
            )
    )

    ;; Leader waits for the other drones
    (:action hold-position

        :parameters
            (?lead - drone)

        :precondition
            (and
                (formation-leader ?lead)
                (airborne ?lead)
                (< (seconds-since-leader-airborne) (grid-launch-delay))
                (gps-ok ?lead)
                (communication-ok ?lead)
                (drone-healthy ?lead)
            )

        :effect
            (increase (seconds-since-leader-airborne) 1)
    )

    ;; Move into the assigned grid slot
    (:action take-grid-slot

        :parameters
            (?d - drone)

        :precondition
            (and
                (airborne ?d)
                (has-slot ?d)
                (at-cruise-altitude ?d)
                (>= (altitude ?d) (target-altitude ?d))
                (not (in-formation ?d))
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
            )

        :effect
            (in-formation ?d)
    )

    ;; Establish the grid formation
    (:action establish-grid-formation

        :parameters
            (?lead - drone
             ?l - location)

        :precondition
            (and
                (formation-leader ?lead)
                (in-formation ?lead)
                (at ?lead ?l)
                (at-cruise-altitude ?lead)
                (>= (altitude ?lead) (target-altitude ?lead))
                (neighbor-comm-ok ?lead)

                (forall (?m - drone)
                    (or
                        (not (grid-member ?m))
                        (and
                            (in-formation ?m)
                            (at ?m ?l)
                            (at-cruise-altitude ?m)
                            (>= (altitude ?m) (target-altitude ?m))
                            (neighbor-comm-ok ?m)
                        )
                    )
                )
            )

        :effect
            (and
                (grid-formation-established)
                (formation-at ?l)
                (grid-closed)
            )
    )

    ;; Move the formation to the next location
    (:action formation-cruise

        :parameters
            (?lead - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (grid-formation-established)
                (grid-closed)

                (formation-leader ?lead)
                (in-formation ?lead)

                (formation-at ?from)
                (at ?lead ?from)

                (connected ?from ?to)
                (safe-route ?from ?to)

                (at-cruise-altitude ?lead)
                (>= (altitude ?lead) (target-altitude ?lead))

                (gps-ok ?lead)
                (communication-ok ?lead)
                (drone-healthy ?lead)
                (neighbor-comm-ok ?lead)

                (> (battery ?lead) (energy-required ?from ?to))
            )

        :effect
            (and
                (not (formation-at ?from))
                (formation-at ?to)

                (not (at ?lead ?from))
                (at ?lead ?to)

                (not (grid-closed))

                (decrease (battery ?lead) (energy-required ?from ?to))
            )
    )

    ;; Move grid members back to the leader
    (:action close-up-grid

        :parameters
            (?lead - drone
             ?to - location)

        :precondition
            (and
                (grid-formation-established)
                (not (grid-closed))

                (formation-leader ?lead)
                (in-formation ?lead)

                (formation-at ?to)
                (at ?lead ?to)

                (forall (?m - drone)
                    (or
                        (not (grid-member ?m))
                        (and
                            (in-formation ?m)
                            (at-cruise-altitude ?m)
                            (>= (altitude ?m) (target-altitude ?m))
                            (gps-ok ?m)
                            (communication-ok ?m)
                            (drone-healthy ?m)
                            (neighbor-comm-ok ?m)
                            (> (battery ?m) (cruise-leg-energy ?m))
                        )
                    )
                )
            )

        :effect
            (and
                (forall (?m - drone) (forall (?x - location)
                    (when
                        (and
                            (grid-member ?m)
                            (at ?m ?x)
                        )
                        (not (at ?m ?x))
                    )
                ))

                (forall (?m - drone)
                    (when
                        (grid-member ?m)
                        (at ?m ?to)
                    )
                )

                (forall (?m - drone)
                    (when
                        (grid-member ?m)
                        (decrease
                            (battery ?m)
                            (cruise-leg-energy ?m)
                        )
                    )
                )

                (grid-closed)
            )
    )

    ;; Complete the mission at the destination
    (:action complete-grid-mission

        :parameters
            (?lead - drone
             ?dest - location)

        :precondition
            (and
                (destination ?dest)
                (grid-formation-established)
                (formation-at ?dest)

                (formation-leader ?lead)
                (in-formation ?lead)
                (at ?lead ?dest)

                (forall (?m - drone)
                    (or
                        (not (grid-member ?m))
                        (and
                            (in-formation ?m)
                            (at ?m ?dest)
                        )
                    )
                )
            )

        :effect
            (and
                (mission-completed ?lead)

                (forall (?m - drone)
                    (when
                        (grid-member ?m)
                        (mission-completed ?m)
                    )
                )

                (formation-mission-complete)
            )
    )

    ;; Detect GPS failure
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
                (not (in-formation ?d))
            )
    )

    ;; Detect communication failure
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
                (not (in-formation ?d))
            )
    )

    ;; Detect neighbor communication failure
    (:action detect-neighbor-link-loss

        :parameters
            (?d - drone)

        :precondition
            (and
                (flying ?d)
                (not (neighbor-comm-ok ?d))
            )

        :effect
            (and
                (communication-lost ?d)
                (not (flying ?d))
                (not (in-formation ?d))
            )
    )

    ;; Detect health failure
    (:action detect-health-failure

        :parameters
            (?d - drone)

        :precondition
            (and
                (flying ?d)
                (< (health ?d) (minimum-health ?d))
            )

        :effect
            (and
                (health-failure ?d)
                (not (flying ?d))
                (not (in-formation ?d))
            )
    )

    ;; Notify the ground controller
    (:action notify-ground-controller

        :parameters
            (?d - drone
             ?c - controller)

        :precondition
            (and
                (or
                    (gps-lost ?d)
                    (communication-lost ?d)
                    (health-failure ?d)
                )
            )

        :effect
            (controller-notified ?d)
    )

)