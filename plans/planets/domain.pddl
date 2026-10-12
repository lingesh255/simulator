(define (domain planets-formation-drone-mission)

    ;; Solar-system drone show round one place (the sun): every drone flies
    ;; its own orbit, 50 m further out than the one inside it, neighbouring
    ;; orbits turning in opposite directions. A lap is one turn of the
    ;; OUTERMOST drone; inner drones go round more often in that time, and
    ;; every lap ends with all of them back on their starting line. After
    ;; the last lap every drone flies back in and lands where it took off
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


    ;; Facts that describe drone positions, states, and orbits
    (:predicates

        ;; Drone position
        (at ?d - drone ?l - location)

        ;; The place the drones orbit round (the sun)
        (source ?l - location)

        ;; Flight states
        (landed ?d - drone)
        (airborne ?d - drone)
        (flying ?d - drone)
        (at-cruise-altitude ?d - drone)

        ;; Orbits: the leader flies the innermost one, every other drone
        ;; its own orbit further out
        (formation-leader ?d - drone)
        (planet ?d - drone)
        (has-orbit ?d - drone)

        ;; Which way round each drone orbits (seen from above)
        (orbits-clockwise ?d - drone)
        (orbits-anticlockwise ?d - drone)

        ;; Drones on neighbouring orbits (one 50 m step apart)
        (orbit-neighbor ?d1 - drone ?d2 - drone)

        ;; The show's progress
        (orbits-entered)

        ;; Neighbor communication - each drone and the drones on the orbits
        ;; either side of it
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
        (planets-mission-complete)
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

        ;; Each drone's orbit radius round the source (metres) - descriptive
        (orbit-radius ?d - drone)

        ;; Laps (turns of the outermost orbit): asked for and done
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
                (not (orbits-entered))
                (has-orbit ?d)
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
                (not (orbits-entered))
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


    ;; Every drone flies out from its pad onto its own orbit - only once
    ;; every drone is up, and only if no two neighbouring orbits turn the
    ;; same way
    (:action enter-orbits

        :parameters
            (?lead - drone)

        :precondition
            (and
                (formation-leader ?lead)
                (at-cruise-altitude ?lead)
                (neighbor-comm-ok ?lead)
                (not (orbits-entered))

                (forall (?m - drone)
                    (or
                        (not (has-orbit ?m))
                        (and
                            (at-cruise-altitude ?m)
                            (neighbor-comm-ok ?m)
                        )
                    )
                )

                (forall (?a - drone ?b - drone)
                    (or
                        (not (orbit-neighbor ?a ?b))
                        (and (orbits-clockwise ?a) (orbits-anticlockwise ?b))
                        (and (orbits-anticlockwise ?a) (orbits-clockwise ?b))
                    )
                )
            )

        :effect
            (orbits-entered)
    )


    ;; One lap: the outermost drone goes once round its orbit, every inner
    ;; drone round its own as often as fits, and all are back on their
    ;; starting line together
    (:action fly-lap

        :parameters
            (?lead - drone)

        :precondition
            (and
                (formation-leader ?lead)
                (orbits-entered)
                (< (laps-completed) (laps-required))
                (gps-ok ?lead)
                (communication-ok ?lead)
            )

        :effect
            (increase (laps-completed) 1)
    )


    ;; Every lap flown: fly back in onto the pads and land
    (:action land-planets

        :parameters
            (?lead - drone
             ?l - location)

        :precondition
            (and
                (source ?l)
                (formation-leader ?lead)
                (orbits-entered)
                (>= (laps-completed) (laps-required))
            )

        :effect
            (and
                (not (orbits-entered))
                (forall (?d - drone)
                    (when (has-orbit ?d)
                        (and
                            (landed ?d)
                            (not (airborne ?d))
                            (not (flying ?d))
                            (at ?d ?l)
                            (mission-completed ?d))))

                (planets-mission-complete)
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


    ;; Detect loss of the link to a neighbouring orbit's drone
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
