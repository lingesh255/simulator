(define (domain cube-grid-formation-drone-mission)

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
        layer
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
        (cube-member ?d - drone)
        (formation-leader ?d - drone)
        (in-formation ?d - drone)
        (cube-formation-established)
        (cube-closed)

        (cube-neighbor ?d1 - drone ?d2 - drone) ;; The link to the up-to-24 drones adjacent to this one in the cube
        (neighbor-comm-ok ?d - drone)     ;; The link to the up-to-24 drones adjacent to this one in the cube

        ;; Layer-by-layer ascent (see the header note)
        (in-layer ?d - drone ?l - layer)
        (layer-below ?lo - layer ?hi - layer)
        (layer-launched ?l - layer)
        (cleared-to-launch ?d - drone)

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

        (slot-along-offset ?d - drone) ;;along the direction of travel
        (slot-cross-offset ?d - drone) ;;perpendicular to the direction of travel
        (slot-layer ?d - drone)        ;;vertical layer in the cube formation
        (slot-lift-offset ?d - drone)  ;;vertical offset from the layer's base altitude

        ;; Staggered launch timing
        (seconds-since-leader-airborne)
        (cube-launch-delay)
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

    ;; Take off from the source.
    ;;
    ;; Two gates, both about keeping aircraft out of each other's way while
    ;; the stack is being built:
    ;;   - the leader goes first, everyone else waits out the launch delay;
    ;;   - no drone may leave the ground until every layer below its own has
    ;;     finished climbing (see `clear-next-layer`).
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

                (cleared-to-launch ?d)

                (or
                    (formation-leader ?d)
                    (>= (seconds-since-leader-airborne) (cube-launch-delay))
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

    ;; Climb to this drone's OWN cruise altitude - one per layer, so the
    ;; stack separates vertically as it forms.
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
                (< (seconds-since-leader-airborne) (cube-launch-delay))
                (gps-ok ?lead)
                (communication-ok ?lead)
                (drone-healthy ?lead)
            )

        :effect
            (increase (seconds-since-leader-airborne) 1)
    )

    ;; Release the next layer up.
    ;;
    ;; Only applicable once every drone in the layer below has reached its
    ;; cruise altitude, so exactly one layer is ever climbing at a time and no
    ;; drone ever ascends through another's slot. This is the vertical
    ;; counterpart of the staggered horizontal launch, and the second of the
    ;; three collision-avoidance mechanisms in the header note.
    ;;
    ;; The conditional effect hands the launch clearance to every drone in the
    ;; newly opened layer at once, which is what `takeoff` then checks.
    (:action open-next-layer

        :parameters
            (?lo - layer
             ?hi - layer)

        :precondition
            (and
                (layer-below ?lo ?hi)
                (layer-launched ?lo)
                (not (layer-launched ?hi))

                (forall (?d - drone)
                    (or
                        (not (in-layer ?d ?lo))
                        (at-cruise-altitude ?d)
                    )
                )
            )

        :effect
            (and
                (layer-launched ?hi)

                (forall (?d - drone)
                    (when
                        (in-layer ?d ?hi)
                        (cleared-to-launch ?d)
                    )
                )
            )
    )

    ;; Move into the assigned cube slot
    (:action take-cube-slot

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

    ;; Establish the cube formation
    (:action establish-cube-formation

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

                ;; Every layer has been released, so the whole stack is up.
                ;; NB the quantified variable must not be `?l` - that name is
                ;; already bound to this action's location parameter.
                (forall (?ly - layer) (layer-launched ?ly))

                (forall (?m - drone)
                    (or
                        (not (cube-member ?m))
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
                (cube-formation-established)
                (formation-at ?l)
                (cube-closed)
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
                (cube-formation-established)
                (cube-closed)

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

                (not (cube-closed))

                (decrease (battery ?lead) (energy-required ?from ?to))
            )
    )

    ;; Move every cube member up to the leader's new location.
    ;;
    ;; All-or-nothing: every member's battery and links are checked before
    ;; any of them moves, so the block never partially advances.
    (:action close-up-cube

        :parameters
            (?lead - drone
             ?to - location)

        :precondition
            (and
                (cube-formation-established)
                (not (cube-closed))

                (formation-leader ?lead)
                (in-formation ?lead)

                (formation-at ?to)
                (at ?lead ?to)

                (forall (?m - drone)
                    (or
                        (not (cube-member ?m))
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
                            (cube-member ?m)
                            (at ?m ?x)
                        )
                        (not (at ?m ?x))
                    )
                ))

                (forall (?m - drone)
                    (when
                        (cube-member ?m)
                        (at ?m ?to)
                    )
                )

                (forall (?m - drone)
                    (when
                        (cube-member ?m)
                        (decrease
                            (battery ?m)
                            (cruise-leg-energy ?m)
                        )
                    )
                )

                (cube-closed)
            )
    )

    ;; Complete the mission at the destination
    (:action complete-cube-mission

        :parameters
            (?lead - drone
             ?dest - location)

        :precondition
            (and
                (destination ?dest)
                (cube-formation-established)
                (formation-at ?dest)

                (formation-leader ?lead)
                (in-formation ?lead)
                (at ?lead ?dest)

                (forall (?m - drone)
                    (or
                        (not (cube-member ?m))
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
                        (cube-member ?m)
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

    ;; Detect communication failure (the link to the ground controller)
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

    ;; Detect neighbor communication failure (the links to the up-to-24
    ;; drones adjacent to this one in the cube)
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
