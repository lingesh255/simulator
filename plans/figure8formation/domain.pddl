(define (domain figure8-formation-drone-mission)

    ;; Figure-8 formation: a train of drones, one behind the other, flies
    ;; from its base to the middle of a figure-8 (two lobes crossing at a
    ;; middle point), round the 8 a number of times at a fixed spacing, and
    ;; back from the middle to the base
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

        ;; Current location of the whole train
        (formation-at ?l - location)

        ;; Mission locations: the base, and the figure-8 waypoint chain from
        ;; where a lap starts (the middle) to where it ends (the middle again)
        (base ?l - location)
        (lap-start ?l - location)
        (lap-end ?l - location)
        (connected ?from - location ?to - location)
        (safe-route ?from - location ?to - location)

        ;; The legs between the base and the middle of the 8
        (transit-route ?from - location ?to - location)

        ;; A waypoint where the two lobes of the 8 cross each other
        (crossing-point ?l - location)

        ;; Flight states
        (landed ?d - drone)
        (airborne ?d - drone)
        (flying ?d - drone)
        (at-cruise-altitude ?d - drone)

        ;; Train slots
        (train-leader ?d - drone)
        (train-follower ?d - drone)
        (has-slot ?d - drone)
        (in-formation ?d - drone)

        ;; Formation status
        (figure8-established)
        (train-closed)
        (pattern-entered)
        (laps-finished)

        ;; Neighbor communication - each drone talks to the drone directly
        ;; in front of it and directly behind it in the train
        (train-neighbor ?d1 - drone ?d2 - drone)
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
        (figure8-mission-complete)
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
        (cruise-leg-energy ?d - drone)
        (energy-required ?from - location ?to - location)

        ;; Distance between locations (along the curve of the 8)
        (distance ?from - location ?to - location)

        ;; Drone health
        (health ?d - drone)
        (minimum-health ?d - drone)

        ;; Location coordinates
        (latitude ?l - location)
        (longitude ?l - location)

        ;; Train slot: metres along the 8 behind the leader (negative = behind)
        (slot-arc-offset ?d - drone)

        ;; Train geometry
        (drone-spacing)
        (train-span)
        (min-lobe-length)
        (crossing-clearance)

        ;; Laps of the 8: how many the mission asks for, how many are done
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
                (base ?l)
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


    ;; Take off from the drone's own slot on the 8 - every drone already
    ;; starts the spacing apart, so no staggered launch is needed
    (:action takeoff

        :parameters
            (?d - drone
             ?l - location)

        :precondition
            (and
                (at ?d ?l)
                (base ?l)
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


    ;; Climb from hover height to cruise altitude
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


    ;; Move the drone into its assigned train slot
    (:action take-train-slot

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


    ;; Establish the figure-8 train when all drones are ready. The whole
    ;; train (front to back) must fit inside the smaller lobe with room to
    ;; spare at the crossing, otherwise the head of the train would reach
    ;; the crossing point while its tail is still passing through it
    (:action establish-figure8-formation

        :parameters
            (?lead - drone
             ?l - location)

        :precondition
            (and
                (train-leader ?lead)
                (in-formation ?lead)
                (at ?lead ?l)
                (at-cruise-altitude ?lead)
                (>= (altitude ?lead) (target-altitude ?lead))
                (neighbor-comm-ok ?lead)

                (<= (+ (train-span) (crossing-clearance)) (min-lobe-length))

                (forall (?f - drone)
                    (or
                        (not (train-follower ?f))
                        (and
                            (in-formation ?f)
                            (at ?f ?l)
                            (at-cruise-altitude ?f)
                            (>= (altitude ?f) (target-altitude ?f))
                            (neighbor-comm-ok ?f)
                        )
                    )
                )
            )

        :effect
            (and
                (figure8-established)
                (formation-at ?l)
                (train-closed)
            )
    )


    ;; Fly the train from the base to the middle of the 8
    (:action fly-to-pattern

        :parameters
            (?lead - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (figure8-established)
                (train-closed)
                (not (pattern-entered))

                (train-leader ?lead)
                (in-formation ?lead)

                (base ?from)
                (lap-start ?to)
                (transit-route ?from ?to)

                (formation-at ?from)
                (at ?lead ?from)

                (gps-ok ?lead)
                (communication-ok ?lead)
                (drone-healthy ?lead)
                (neighbor-comm-ok ?lead)

                (> (battery ?lead) (energy-required ?from ?to))
            )

        :effect
            (and
                (pattern-entered)

                (not (formation-at ?from))
                (formation-at ?to)

                (not (at ?lead ?from))
                (at ?lead ?to)

                ;; Followers must catch up before the next leg
                (not (train-closed))

                (decrease (battery ?lead) (energy-required ?from ?to))
            )
    )


    ;; Move the leader to the next waypoint of the 8
    (:action formation-cruise

        :parameters
            (?lead - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (figure8-established)
                (train-closed)
                (pattern-entered)
                (not (laps-finished))

                (train-leader ?lead)
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

                ;; Followers must catch up before the next leg
                (not (train-closed))

                (decrease (battery ?lead) (energy-required ?from ?to))
            )
    )


    ;; Move every follower along behind the leader
    (:action follow-leader

        :parameters
            (?lead - drone
             ?to - location)

        :precondition
            (and
                (figure8-established)
                (not (train-closed))

                (train-leader ?lead)
                (in-formation ?lead)

                (formation-at ?to)
                (at ?lead ?to)

                (forall (?f - drone)
                    (or
                        (not (train-follower ?f))
                        (and
                            (in-formation ?f)
                            (at-cruise-altitude ?f)
                            (>= (altitude ?f) (target-altitude ?f))
                            (gps-ok ?f)
                            (communication-ok ?f)
                            (drone-healthy ?f)
                            (neighbor-comm-ok ?f)
                            (> (battery ?f) (cruise-leg-energy ?f))
                        )
                    )
                )
            )

        :effect
            (and
                (forall (?f - drone) (forall (?x - location)
                    (when (and (train-follower ?f) (at ?f ?x))
                        (not (at ?f ?x)))))

                (forall (?f - drone)
                    (when (train-follower ?f) (at ?f ?to)))

                (forall (?f - drone)
                    (when (train-follower ?f)
                        (decrease (battery ?f) (cruise-leg-energy ?f))))

                (train-closed)
            )
    )


    ;; Finish a lap at the crossing and start the next one from the same
    ;; point - only while more laps remain after the one being finished
    (:action start-next-lap

        :parameters
            (?lead - drone
             ?dest - location
             ?start - location)

        :precondition
            (and
                (lap-end ?dest)
                (lap-start ?start)
                (figure8-established)
                (train-closed)
                (formation-at ?dest)

                (train-leader ?lead)
                (at ?lead ?dest)

                (< (+ (laps-completed) 1) (laps-required))
            )

        :effect
            (and
                (not (formation-at ?dest))
                (formation-at ?start)

                (not (at ?lead ?dest))
                (at ?lead ?start)

                (forall (?f - drone)
                    (when (and (train-follower ?f) (at ?f ?dest))
                        (and (not (at ?f ?dest)) (at ?f ?start))))

                (increase (laps-completed) 1)
            )
    )


    ;; Count the last lap as flown - the train is at the middle of the 8 and
    ;; no more laps remain
    (:action finish-laps

        :parameters
            (?lead - drone
             ?end - location)

        :precondition
            (and
                (lap-end ?end)
                (figure8-established)
                (train-closed)
                (not (laps-finished))
                (formation-at ?end)

                (train-leader ?lead)
                (at ?lead ?end)

                (>= (+ (laps-completed) 1) (laps-required))
            )

        :effect
            (and
                (laps-finished)
                (increase (laps-completed) 1)
            )
    )


    ;; Fly the train from the middle of the 8 back to the base
    (:action return-to-base

        :parameters
            (?lead - drone
             ?from - location
             ?to - location)

        :precondition
            (and
                (laps-finished)
                (train-closed)

                (train-leader ?lead)
                (in-formation ?lead)

                (lap-end ?from)
                (base ?to)
                (transit-route ?from ?to)

                (formation-at ?from)
                (at ?lead ?from)

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

                ;; Followers must catch up before the mission can complete
                (not (train-closed))

                (decrease (battery ?lead) (energy-required ?from ?to))
            )
    )


    ;; Complete the mission when the train has flown every lap and is back
    ;; at the base
    (:action complete-figure8-mission

        :parameters
            (?lead - drone
             ?home - location)

        :precondition
            (and
                (base ?home)
                (laps-finished)
                (train-closed)
                (formation-at ?home)

                (train-leader ?lead)
                (in-formation ?lead)
                (at ?lead ?home)

                (forall (?f - drone)
                    (or
                        (not (train-follower ?f))
                        (and
                            (in-formation ?f)
                            (at ?f ?home)
                        )
                    )
                )
            )

        :effect
            (and
                (mission-completed ?lead)

                (forall (?f - drone)
                    (when (train-follower ?f)
                        (mission-completed ?f)))

                (figure8-mission-complete)
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


    ;; Detect loss of the link to the front/back neighbor
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
                (not (in-formation ?d))
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
