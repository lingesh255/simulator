(define (domain star-formation-drone-mission)

    ;; Star drone show round one place: the drones take off, form a circle,
    ;; then each lap form a star, a circle, spin the circle, form a star,
    ;; spin the star and form a circle again - and finally land where they
    ;; took off
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


    ;; Facts that describe drone positions, states, and formation
    (:predicates

        ;; Drone position
        (at ?d - drone ?l - location)

        ;; The place the show is performed round
        (source ?l - location)

        ;; Flight states
        (landed ?d - drone)
        (airborne ?d - drone)
        (flying ?d - drone)
        (at-cruise-altitude ?d - drone)

        ;; Formation slots: the leader takes the star's top tip, every
        ;; other drone its own place round the star
        (formation-leader ?d - drone)
        (star-member ?d - drone)
        (has-slot ?d - drone)

        ;; The show's progress
        (show-started)
        (circle-formed)
        (star-formed)
        (lap-started)
        (circle-spun)
        (star-spun)

        ;; Neighbor communication - each drone and the two either side of it
        ;; round the circle
        (ring-neighbor ?d1 - drone ?d2 - drone)
        (neighbor-comm-ok ?d - drone)

        ;; Pre-flight check
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
        (star-mission-complete)
    )

    ;; Numeric values used by the planner
    (:functions

        ;; Battery
        (battery ?d - drone)
        (max-battery ?d - drone)
        (minimum-return-battery ?d - drone)

        ;; Drone altitude
        (altitude ?d - drone)
        (hover-altitude ?d - drone)
        (target-altitude ?d - drone)

        ;; Energy consumption
        (climb-energy ?d - drone)

        ;; Drone health
        (health ?d - drone)
        (minimum-health ?d - drone)

        ;; Location coordinates
        (latitude ?l - location)
        (longitude ?l - location)

        ;; Each drone's direction from the show's centre (degrees,
        ;; anticlockwise from the top tip) - descriptive
        (slot-angle ?d - drone)

        ;; Laps of the show: how many are asked for, how many are done
        (laps-required)
        (laps-completed)
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
                (not (show-started))
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


    ;; Take off from the drone's pad
    (:action takeoff

        :parameters
            (?d - drone
             ?l - location)

        :precondition
            (and
                (at ?d ?l)
                (source ?l)
                (landed ?d)
                (not (show-started))
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


    ;; Climb from hover height to the show's altitude
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


    ;; Every drone spreads out from its pad onto the circle
    (:action form-circle

        :parameters
            (?lead - drone)

        :precondition
            (and
                (formation-leader ?lead)
                (at-cruise-altitude ?lead)
                (neighbor-comm-ok ?lead)
                (not (show-started))

                (forall (?m - drone)
                    (or
                        (not (star-member ?m))
                        (and
                            (at-cruise-altitude ?m)
                            (neighbor-comm-ok ?m)
                        )
                    )
                )
            )

        :effect
            (and
                (show-started)
                (circle-formed)
            )
    )


    ;; Lap, step 1: the circle changes into the star
    (:action form-star

        :parameters
            (?lead - drone)

        :precondition
            (and
                (formation-leader ?lead)
                (circle-formed)
                (not (lap-started))
                (< (laps-completed) (laps-required))
                (gps-ok ?lead)
                (communication-ok ?lead)
            )

        :effect
            (and
                (not (circle-formed))
                (star-formed)
                (lap-started)
            )
    )


    ;; Lap, step 2: the star changes back into the circle
    (:action form-circle-in-lap

        :parameters
            (?lead - drone)

        :precondition
            (and
                (formation-leader ?lead)
                (star-formed)
                (lap-started)
                (not (circle-spun))
            )

        :effect
            (and
                (not (star-formed))
                (circle-formed)
            )
    )


    ;; Lap, step 3: the circle turns once round
    (:action spin-circle

        :parameters
            (?lead - drone)

        :precondition
            (and
                (formation-leader ?lead)
                (circle-formed)
                (lap-started)
                (not (circle-spun))
            )

        :effect
            (circle-spun)
    )


    ;; Lap, step 4: the circle changes into the star again
    (:action form-star-again

        :parameters
            (?lead - drone)

        :precondition
            (and
                (formation-leader ?lead)
                (circle-formed)
                (circle-spun)
                (not (star-spun))
            )

        :effect
            (and
                (not (circle-formed))
                (star-formed)
            )
    )


    ;; Lap, step 5: the whole star turns once round its centre
    (:action spin-star

        :parameters
            (?lead - drone)

        :precondition
            (and
                (formation-leader ?lead)
                (star-formed)
                (circle-spun)
                (not (star-spun))
            )

        :effect
            (star-spun)
    )


    ;; Lap, step 6: the star changes back into the circle - the lap is done
    (:action close-lap

        :parameters
            (?lead - drone)

        :precondition
            (and
                (formation-leader ?lead)
                (star-formed)
                (star-spun)
            )

        :effect
            (and
                (not (star-formed))
                (circle-formed)
                (not (lap-started))
                (not (circle-spun))
                (not (star-spun))
                (increase (laps-completed) 1)
            )
    )


    ;; Every lap flown: spread back in onto the pads and land
    (:action land-show

        :parameters
            (?lead - drone
             ?l - location)

        :precondition
            (and
                (source ?l)
                (formation-leader ?lead)
                (circle-formed)
                (not (lap-started))
                (>= (laps-completed) (laps-required))
            )

        :effect
            (and
                (not (circle-formed))
                (forall (?d - drone)
                    (when (has-slot ?d)
                        (and
                            (landed ?d)
                            (not (airborne ?d))
                            (not (flying ?d))
                            (at ?d ?l)
                            (mission-completed ?d))))

                (star-mission-complete)
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
            )
    )


    ;; Detect loss of the link to a neighbor round the circle
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
            )
    )


    ;; Detect when drone health becomes too low
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
            )
    )


    ;; Inform the ground controller about a failure
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
