(define (domain road-inspection-drone-mission)

    ;; Road inspection: a drone follows a road from the source to the
    ;; destination, inspecting every segment of it on the way, then flies
    ;; back along the same road and lands at the source
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


    ;; Facts that describe the drone, the road and the inspection
    (:predicates

        ;; Drone position
        (at ?d - drone ?l - location)

        ;; Mission locations
        (source ?l - location)
        (destination ?l - location)

        ;; The road, as segments from the source end towards the destination
        (road-segment ?from - location ?to - location)
        (segment-inspected ?from - location ?to - location)
        (road-inspected)

        ;; The drone doing the inspection
        (inspector ?d - drone)

        ;; Flight states
        (landed ?d - drone)
        (airborne ?d - drone)
        (flying ?d - drone)
        (at-cruise-altitude ?d - drone)

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
        (energy-required ?from - location ?to - location)

        ;; Length of a road segment
        (distance ?from - location ?to - location)

        ;; Drone health
        (health ?d - drone)
        (minimum-health ?d - drone)

        ;; Location coordinates
        (latitude ?l - location)
        (longitude ?l - location)

        ;; How much of the road is inspected
        (segments-total)
        (segments-inspected)
    )


    ;; Check the drone before takeoff
    (:action pre-flight-check

        :parameters
            (?d - drone
             ?l - location)

        :precondition
            (and
                (inspector ?d)
                (at ?d ?l)
                (source ?l)
                (landed ?d)
                (not (road-inspected))
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (>= (health ?d) (minimum-health ?d))
                (> (battery ?d) (minimum-return-battery ?d))
            )

        :effect
            (preflight-done ?d)
    )


    ;; Take off from the source end of the road
    (:action takeoff

        :parameters
            (?d - drone
             ?l - location)

        :precondition
            (and
                (inspector ?d)
                (at ?d ?l)
                (source ?l)
                (landed ?d)
                (preflight-done ?d)
                (not (road-inspected))
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


    ;; Climb from hover height to inspection altitude
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


    ;; Fly along the next segment of the road, inspecting it
    (:action inspect-segment

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (inspector ?d)
                (road-segment ?from ?to)
                (not (segment-inspected ?from ?to))
                (not (road-inspected))
                (at ?d ?from)
                (at-cruise-altitude ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (> (battery ?d) (energy-required ?from ?to))
            )

        :effect
            (and
                (not (at ?d ?from))
                (at ?d ?to)
                (segment-inspected ?from ?to)
                (increase (segments-inspected) 1)
                (decrease (battery ?d) (energy-required ?from ?to))
            )
    )


    ;; At the destination with every segment inspected: the road is done
    (:action finish-inspection

        :parameters
            (?d - drone
             ?l - location)

        :precondition
            (and
                (inspector ?d)
                (destination ?l)
                (at ?d ?l)
                (>= (segments-inspected) (segments-total))
            )

        :effect
            (road-inspected)
    )


    ;; Fly back along the same road, one segment at a time
    (:action return-along-road

        :parameters
            (?d - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (inspector ?d)
                (road-inspected)
                (road-segment ?to ?from)
                (at ?d ?from)
                (at-cruise-altitude ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (> (battery ?d) (energy-required ?from ?to))
            )

        :effect
            (and
                (not (at ?d ?from))
                (at ?d ?to)
                (decrease (battery ?d) (energy-required ?from ?to))
            )
    )


    ;; Land back at the source
    (:action land-at-source

        :parameters
            (?d - drone
             ?l - location)

        :precondition
            (and
                (inspector ?d)
                (road-inspected)
                (source ?l)
                (at ?d ?l)
                (airborne ?d)
            )

        :effect
            (and
                (not (airborne ?d))
                (not (flying ?d))
                (not (at-cruise-altitude ?d))
                (landed ?d)
                (mission-completed ?d)
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
