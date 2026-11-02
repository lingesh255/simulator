;; Sequential task ownership; a replacement launches only after a failure.
(define (domain task-allocation)
    (:requirements :strips :typing :negative-preconditions :numeric-fluents)
    (:types drone location)
    (:predicates
        (at ?d - drone ?l - location)
        (source ?l - location)
        (destination ?l - location)
        (connected ?from - location ?to - location)
        (standby ?d - drone)
        (capable ?d - drone)
        (communication-ok ?d - drone)
        (responsible ?d - drone)
        (mission-data ?d - drone)
        (airborne ?d - drone)
        (en-route ?d - drone ?from - location ?to - location)
        ;; Scenario input: this drone develops a recoverable fault in flight.
        (will-fail ?d - drone)
        (failed ?d - drone)
        (notified ?old - drone ?new - drone)
        (replacement-launched ?old - drone ?new - drone)
        (retired ?d - drone)
        (mission-completed ?d - drone)
        (mission-started)
        (mission-finished)
    )

    (:functions (latitude ?l - location) (longitude ?l - location))

    (:action start-mission
        :parameters (?d - drone ?from - location ?to - location)
        :precondition (and
            (not (mission-started)) (not (mission-finished))
            (source ?from) (destination ?to) (connected ?from ?to)
            (at ?d ?from) (standby ?d) (capable ?d)
            (communication-ok ?d))
        :effect (and
            (mission-started) (responsible ?d) (mission-data ?d)
            (not (standby ?d)) (not (at ?d ?from))
            (airborne ?d) (en-route ?d ?from ?to))
    )

    (:action detect-problem
        :parameters (?d - drone)
        :precondition (and
            (not (mission-finished)) (responsible ?d) (airborne ?d)
            (mission-data ?d) (will-fail ?d) (not (failed ?d)))
        :effect (failed ?d)
    )

    ;; Consuming standby reserves exactly one replacement for this failure.
    (:action notify-capable-drone
        :parameters (?old - drone ?new - drone ?from - location)
        :precondition (and
            (not (mission-finished)) (responsible ?old) (failed ?old)
            (airborne ?old) (mission-data ?old)
            (communication-ok ?old) (communication-ok ?new)
            (source ?from) (at ?new ?from) (standby ?new) (capable ?new))
        :effect (and
            (not (responsible ?old)) (not (standby ?new))
            (notified ?old ?new))
    )

    (:action launch-replacement
        :parameters (?old - drone ?new - drone ?from - location ?to - location)
        :precondition (and
            (not (mission-finished)) (notified ?old ?new)
            (failed ?old) (airborne ?old)
            (source ?from) (destination ?to) (connected ?from ?to)
            (en-route ?old ?from ?to) (at ?new ?from) (capable ?new)
            (communication-ok ?old) (communication-ok ?new))
        :effect (and
            (not (notified ?old ?new)) (not (at ?new ?from))
            (airborne ?new) (en-route ?new ?from ?to)
            (replacement-launched ?old ?new))
    )

    ;; Both drones are airborne here. The old drone must retain its radio
    ;; and enough flight control to transmit the latest mission state.
    (:action share-data-and-transfer-task
        :parameters (?old - drone ?new - drone ?from - location ?to - location)
        :precondition (and
            (not (mission-finished)) (replacement-launched ?old ?new)
            (failed ?old) (mission-data ?old)
            (airborne ?old) (airborne ?new)
            (en-route ?old ?from ?to) (en-route ?new ?from ?to)
            (communication-ok ?old) (communication-ok ?new))
        :effect (and
            (not (replacement-launched ?old ?new))
            (mission-data ?new) (responsible ?new)
            (not (mission-data ?old)) (retired ?old))
    )

    ;; A configured failure cannot be bypassed by flying to the goal.
    ;; Arrival immediately closes the mission, disabling every other action.
    (:action travel-to-destination
        :parameters (?d - drone ?from - location ?to - location)
        :precondition (and
            (not (mission-finished)) (responsible ?d) (mission-data ?d)
            (airborne ?d) (en-route ?d ?from ?to) (destination ?to)
            (not (will-fail ?d)) (not (failed ?d)))
        :effect (and
            (not (en-route ?d ?from ?to)) (not (airborne ?d))
            (not (responsible ?d)) (at ?d ?to)
            (mission-completed ?d) (mission-finished))
    )
)
