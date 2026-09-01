(define (domain v-formation-drone-mission)

    ;; ============================================================
    ;; Three drones launch from a common source, climb to a 50 m
    ;; cruise altitude, lock into a V-shape (one apex leader, two
    ;; wings) and then transit direct - source straight to
    ;; destination, no waypoint and no restricted-area handling for
    ;; this plan - as a rigid formation.
    ;;
    ;; A wing's position is labeled by real compass direction (one of
    ;; the 8 `direction` objects in problem.pddl - north/south/east/
    ;; west/north-east/north-west/south-east/south-west), not a fixed
    ;; "left"/"right": `slot-direction` is computed from the actual
    ;; source->destination bearing by
    ;; engine.vformation_problem.retarget_vformation_problem, the same
    ;; way engine.pddl_problem.retarget_problem fits `travell`'s route
    ;; to wherever you click. Both wings sit exactly 5 m from the
    ;; leader and from each other (an equilateral triangle - see
    ;; problem.pddl's `slot-along-offset`/`slot-cross-offset`).
    ;;
    ;; The V is kept honest three ways:
    ;;   * every drone must be assigned exactly one slot
    ;;     (apex, or a direction) up front - see `has-slot`;
    ;;   * `establish-v-formation` only fires once all three slots
    ;;     are individually filled AND the three drones sit at the
    ;;     same location and the same cruise altitude;
    ;;   * once the V exists, the only way it advances is the locked
    ;;     `formation-cruise` -> `close-up-wings` pair: the apex pulls
    ;;     the formation onto the next waypoint, then both wings tuck
    ;;     back onto it before the apex is allowed to move again, so no
    ;;     drone can wander off on its own and the wings never trail by
    ;;     more than the leg in progress.
    ;; ============================================================

    (:requirements
        :strips
        :typing
        :equality
        :numeric-fluents
        :negative-preconditions
        :disjunctive-preconditions
        :conditional-effects
    )

    ;; ============================================================
    ;; TYPES
    ;; ============================================================

    (:types
        drone
        location
        controller
        direction    ; one of the 8 compass points - see problem.pddl's objects
    )


    ;; ============================================================
    ;; PREDICATES
    ;; ============================================================

    (:predicates

        ;; --------------------------------------------------------
        ;; Position
        ;; --------------------------------------------------------

        (at ?d - drone ?l - location)

        ;; Where the V as a whole currently sits - only meaningful
        ;; once `v-formation-established` holds.
        (formation-at ?l - location)

        ;; --------------------------------------------------------
        ;; Mission geography
        ;; --------------------------------------------------------

        (source ?l - location)
        (destination ?l - location)

        (connected ?from - location ?to - location)
        (safe-route ?from - location ?to - location)

        ;; --------------------------------------------------------
        ;; Flight phase
        ;; --------------------------------------------------------

        (landed ?d - drone)
        (airborne ?d - drone)
        (flying ?d - drone)
        (at-cruise-altitude ?d - drone)

        ;; --------------------------------------------------------
        ;; Formation slots - asserted in the problem's :init.
        ;;
        ;; The leader is `slot-apex`. Each wing instead carries
        ;; `slot-direction`, the real compass point (one of the 8
        ;; objects declared in problem.pddl) its offset from the
        ;; leader actually points to - computed from the true
        ;; source->destination bearing (see
        ;; engine.vformation_problem.retarget_vformation_problem),
        ;; not a fixed "left"/"right" label baked into the template.
        ;; --------------------------------------------------------

        (slot-apex ?d - drone)
        (slot-direction ?d - drone ?dir - direction)
        (has-slot ?d - drone)

        (formation-leader ?d - drone)

        (in-formation ?d - drone)

        (v-formation-established)

        ;; True whenever both wings are tucked onto the apex. The apex
        ;; may only start the next leg while this holds; each leg clears
        ;; it and `close-up-wings` restores it - so the wings can never
        ;; fall more than one leg behind the leader.
        (wings-closed)

        ;; --------------------------------------------------------
        ;; Pre-flight / checks
        ;; --------------------------------------------------------

        (preflight-done ?d - drone)

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
        (controller-notified ?d - drone)

        ;; --------------------------------------------------------
        ;; Mission
        ;; --------------------------------------------------------

        (mission-completed ?d - drone)
        (formation-mission-complete)
    )


    ;; ============================================================
    ;; NUMERIC FUNCTIONS
    ;; ============================================================

    (:functions

        ;; Battery percentage
        (battery ?d - drone)
        (max-battery ?d - drone)
        (minimum-return-battery ?d - drone)

        ;; Altitude in metres above the launch point
        (altitude ?d - drone)

        ;; Altitude the drone rests at right after takeoff, before
        ;; the climb to cruise height
        (hover-altitude ?d - drone)

        ;; Target cruise altitude for this mission (50 m here)
        (target-altitude ?d - drone)

        ;; Battery spent climbing from hover to cruise altitude
        (climb-energy ?d - drone)

        ;; Battery a drone spends holding its slot over one cruise leg.
        ;; Modelled per drone (not per leg) so the wings, which fly a
        ;; slightly longer path around the outside of every turn, can be
        ;; given a higher burn than the apex.
        (cruise-leg-energy ?d - drone)

        ;; Battery spent flying a leg (used by the single-drone phases)
        (energy-required ?from - location ?to - location)

        ;; Leg length in metres
        (distance ?from - location ?to - location)

        ;; Health
        (health ?d - drone)
        (minimum-health ?d - drone)

        ;; Geographic coordinates
        (latitude ?l - location)
        (longitude ?l - location)

        ;; V geometry - slot offsets relative to the apex, in metres.
        ;; Along-track is negative for the wings (they trail the
        ;; leader); cross-track is -/+ for left/right. These are
        ;; descriptive: they pin down the actual shape of the V that
        ;; `establish-v-formation` locks in.
        (slot-along-offset ?d - drone)
        (slot-cross-offset ?d - drone)
    )


    ;; ============================================================
    ;; ACTION 1: PRE-FLIGHT CHECK
    ;; ============================================================

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


    ;; ============================================================
    ;; ACTION 2: TAKE OFF
    ;; ============================================================

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
            )

        :effect
            (and
                (not (landed ?d))
                (airborne ?d)
                (flying ?d)
                (assign (altitude ?d) (hover-altitude ?d))
            )
    )


    ;; ============================================================
    ;; ACTION 3: CLIMB TO CRUISE ALTITUDE (50 m)
    ;; ============================================================

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


    ;; ============================================================
    ;; ACTION 4: MOVE INTO ASSIGNED V SLOT
    ;; ============================================================

    (:action take-formation-slot

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


    ;; ============================================================
    ;; ACTION 5: ESTABLISH THE V FORMATION
    ;; ============================================================

    ;; Fires once each of the three slots is filled by a distinct
    ;; drone, all three share a location, and all three are at cruise
    ;; altitude. From here on the group moves only via
    ;; `formation-cruise`.

    (:action establish-v-formation

        :parameters
            (?lead - drone
             ?lw - drone
             ?rw - drone
             ?l - location
             ?dl - direction
             ?dr - direction)

        :precondition
            (and
                (not (= ?lead ?lw))
                (not (= ?lead ?rw))
                (not (= ?lw ?rw))
                (not (= ?dl ?dr))

                (formation-leader ?lead)
                (slot-apex ?lead)
                (slot-direction ?lw ?dl)
                (slot-direction ?rw ?dr)

                (in-formation ?lead)
                (in-formation ?lw)
                (in-formation ?rw)

                (at ?lead ?l)
                (at ?lw ?l)
                (at ?rw ?l)

                (at-cruise-altitude ?lead)
                (at-cruise-altitude ?lw)
                (at-cruise-altitude ?rw)

                (>= (altitude ?lead) (target-altitude ?lead))
                (>= (altitude ?lw) (target-altitude ?lw))
                (>= (altitude ?rw) (target-altitude ?rw))
            )

        :effect
            (and
                (v-formation-established)
                (formation-at ?l)
                (wings-closed)
            )
    )


    ;; ============================================================
    ;; ACTION 6: CRUISE ONE LEG - APEX LEADS
    ;; ============================================================

    ;; Once the V exists this is the only way it advances, and it comes
    ;; in a locked pair with ACTION 6b:
    ;;
    ;;   formation-cruise  - apex pulls the whole formation onto the
    ;;                       next waypoint and clears `wings-closed`
    ;;   close-up-wings    - both wings tuck back onto the apex and
    ;;                       `wings-closed` is restored
    ;;
    ;; The apex can't begin another leg until the wings have closed up
    ;; (it needs `wings-closed`), so the wings never trail the leader by
    ;; more than the leg in progress and the V stays intact.

    (:action formation-cruise

        :parameters
            (?lead - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (v-formation-established)
                (wings-closed)

                (formation-leader ?lead)
                (slot-apex ?lead)
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

                (> (battery ?lead) (energy-required ?from ?to))
            )

        :effect
            (and
                (not (formation-at ?from))
                (formation-at ?to)

                (not (at ?lead ?from))
                (at ?lead ?to)

                (not (wings-closed))

                (decrease (battery ?lead) (energy-required ?from ?to))
            )
    )


    ;; ============================================================
    ;; ACTION 6b: CLOSE THE WINGS BACK ONTO THE APEX
    ;; ============================================================

    (:action close-up-wings

        :parameters
            (?lead - drone
             ?lw - drone
             ?rw - drone
             ?to - location
             ?dl - direction
             ?dr - direction)

        :precondition
            (and
                (not (= ?lead ?lw))
                (not (= ?lead ?rw))
                (not (= ?lw ?rw))
                (not (= ?dl ?dr))

                (v-formation-established)
                (not (wings-closed))

                (formation-leader ?lead)
                (slot-direction ?lw ?dl)
                (slot-direction ?rw ?dr)

                (in-formation ?lead)
                (in-formation ?lw)
                (in-formation ?rw)

                (formation-at ?to)
                (at ?lead ?to)

                (at-cruise-altitude ?lw)
                (at-cruise-altitude ?rw)
                (>= (altitude ?lw) (target-altitude ?lw))
                (>= (altitude ?rw) (target-altitude ?rw))

                (gps-ok ?lw) (gps-ok ?rw)
                (communication-ok ?lw) (communication-ok ?rw)
                (drone-healthy ?lw) (drone-healthy ?rw)

                (> (battery ?lw) (cruise-leg-energy ?lw))
                (> (battery ?rw) (cruise-leg-energy ?rw))
            )

        :effect
            (and
                (forall (?x - location)
                    (when (at ?lw ?x) (not (at ?lw ?x))))
                (forall (?x - location)
                    (when (at ?rw ?x) (not (at ?rw ?x))))

                (at ?lw ?to)
                (at ?rw ?to)

                (wings-closed)

                (decrease (battery ?lw) (cruise-leg-energy ?lw))
                (decrease (battery ?rw) (cruise-leg-energy ?rw))
            )
    )


    ;; ============================================================
    ;; ACTION 7: COMPLETE THE FORMATION MISSION AT DESTINATION
    ;; ============================================================

    (:action complete-formation-mission

        :parameters
            (?lead - drone
             ?lw - drone
             ?rw - drone
             ?dest - location
             ?dl - direction
             ?dr - direction)

        :precondition
            (and
                (not (= ?lead ?lw))
                (not (= ?lead ?rw))
                (not (= ?lw ?rw))
                (not (= ?dl ?dr))

                (destination ?dest)
                (v-formation-established)
                (formation-at ?dest)

                (formation-leader ?lead)
                (slot-apex ?lead)
                (slot-direction ?lw ?dl)
                (slot-direction ?rw ?dr)

                (in-formation ?lead)
                (in-formation ?lw)
                (in-formation ?rw)

                (at ?lead ?dest)
                (at ?lw ?dest)
                (at ?rw ?dest)
            )

        :effect
            (and
                (mission-completed ?lead)
                (mission-completed ?lw)
                (mission-completed ?rw)
                (formation-mission-complete)
            )
    )


    ;; ============================================================
    ;; ACTION 8: GPS FAILURE
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
                (not (in-formation ?d))
            )
    )


    ;; ============================================================
    ;; ACTION 9: COMMUNICATION FAILURE
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
                (not (in-formation ?d))
            )
    )


    ;; ============================================================
    ;; ACTION 10: HEALTH FAILURE
    ;; ============================================================

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


    ;; ============================================================
    ;; ACTION 11: NOTIFY GROUND CONTROLLER
    ;; ============================================================

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
