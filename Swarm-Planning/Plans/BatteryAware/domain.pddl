;; One drone flies from the source to the destination on its battery.
;; If its current charge reaches the destination, it flies straight there
;; and no charging station is used. If not, it hops between charging
;; stations: it lands at a station, charges ONLY what the next hop needs
;; plus a margin (charge-margin, 10%), and takes off again from there.
;; From the first station the destination can be reached from, it charges
;; just enough for the destination and flies straight to it, skipping any
;; stations in between.

(define (domain battery-aware-drone-mission)

    ;; Planning features used by these actions
    (:requirements
        :strips ;; Use actions with conditions and effects
        :typing ;; Give objects types such as drone and location
        :equality ;; Compare whether two objects are the same
        :numeric-fluents ;; Use numbers such as battery, distance and height
        :negative-preconditions ;; Allow a condition to require that a fact is false
        :disjunctive-preconditions ;; Allow a choice between conditions
    )

    ;; Types of objects used in this mission
    (:types
        drone
        location
        controller
    )

    ;; Facts that can be true or false
    (:predicates
        (at ?d - drone ?l - location) ;; Where the drone is
        (source ?l - location) ;; Where the mission starts
        (destination ?l - location) ;; Where the mission goes
        (charging-station ?l - location) ;; A place the drone can land and charge
        (hop ?from - location ?to - location) ;; The drone may fly straight from one place to the other
        (landed ?d - drone) ;; The drone is on the ground
        (airborne ?d - drone) ;; The drone has taken off
        (hop-flown ?d - drone) ;; The drone has flown its hop and must now land
        (needs-charge ?d - drone) ;; The drone landed at a station and has not charged yet
        (charged-for ?d - drone ?from - location ?to - location) ;; Charged at ?from for the hop to ?to
        (preflight-done ?d - drone) ;; The drone has passed the checks before takeoff
        (gps-ok ?d - drone) ;; The drone has a GPS fix
        (communication-ok ?d - drone) ;; The model treats the ground-controller link as working
        (drone-healthy ?d - drone) ;; The drone passes the health check
        (gps-lost ?d - drone) ;; GPS loss has been recorded
        (communication-lost ?d - drone) ;; A communication failure has been recorded
        (controller-notified ?d - drone) ;; The controller has been told about a failure
        (mission-completed ?d - drone) ;; The drone has finished its mission
    )

    ;; Numbers used by the planner
    (:functions
        (battery ?d - drone) ;; Remaining battery, in percent
        (max-battery ?d - drone) ;; Full battery, in percent
        (charge-margin ?d - drone) ;; Charge this much more than a hop needs; it is left on landing
        (hop-energy ?from - location ?to - location) ;; Battery used by one hop: takeoff, climb, cruise and landing
        (distance ?from - location ?to - location) ;; Straight-line distance of one hop, in metres
        (health ?d - drone) ;; Current health score
        (minimum-health ?d - drone) ;; Lowest allowed health score
        (latitude ?l - location) ;; North-south map coordinate
        (longitude ?l - location) ;; East-west map coordinate
        (total-distance) ;; Metres flown so far - the planner keeps this as small as it can
        (charge-stops) ;; Number of stations landed at - each one also counts against the route
    )

    ;; Check the drone before the first takeoff
    (:action pre-flight-check
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?l - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (at ?d ?l)
                (source ?l)
                (landed ?d)
                (not (preflight-done ?d))
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (>= (health ?d) (minimum-health ?d))
                (> (battery ?d) (charge-margin ?d))
            )
        ;; These facts and values change after the action
        :effect
            (preflight-done ?d)
    )

    ;; Take off from the source, or from a station once the battery is charged
    (:action take-off
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?l - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (at ?d ?l)
                (landed ?d)
                (preflight-done ?d)
                (not (needs-charge ?d))
                (not (destination ?l))
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (landed ?d))
                (airborne ?d)
            )
    )

    ;; Fly one hop - only if the battery left on landing is at least the margin.
    ;; From a station, only the hop the drone was charged for.
    (:action fly-hop
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?from - location
             ?to - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (hop ?from ?to)
                (at ?d ?from)
                (airborne ?d)
                (not (hop-flown ?d))
                ;; At least one of the following conditions must hold
                (or
                    (source ?from)
                    (charged-for ?d ?from ?to)
                )
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (>= (- (battery ?d) (hop-energy ?from ?to)) (charge-margin ?d))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (at ?d ?from))
                (at ?d ?to)
                (hop-flown ?d)
                (not (charged-for ?d ?from ?to))
                (decrease (battery ?d) (hop-energy ?from ?to))
                (increase (total-distance) (distance ?from ?to))
            )
    )

    ;; Land at a charging station - the drone must charge before it takes off again
    (:action land-at-station
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?l - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (at ?d ?l)
                (charging-station ?l)
                (airborne ?d)
                (hop-flown ?d)
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (airborne ?d))
                (not (hop-flown ?d))
                (landed ?d)
                (needs-charge ?d)
                (increase (charge-stops) 1)
            )
    )

    ;; Charge, while landed at the station, to exactly what the next hop
    ;; (to ?to) needs plus the margin - not to full. If ?to is the
    ;; destination, that is just enough for the destination.
    (:action charge-battery
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?l - location
             ?to - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (at ?d ?l)
                (charging-station ?l)
                (landed ?d)
                (needs-charge ?d)
                (hop ?l ?to)
                ;; it really needs charging for that hop
                (< (battery ?d) (+ (hop-energy ?l ?to) (charge-margin ?d)))
                ;; and the hop fits in a full battery
                (<= (+ (hop-energy ?l ?to) (charge-margin ?d)) (max-battery ?d))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (needs-charge ?d))
                (charged-for ?d ?l ?to)
                (assign (battery ?d) (+ (hop-energy ?l ?to) (charge-margin ?d)))
            )
    )

    ;; Land at the destination
    (:action land-at-destination
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?l - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (at ?d ?l)
                (destination ?l)
                (airborne ?d)
                (hop-flown ?d)
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (airborne ?d))
                (not (hop-flown ?d))
                (landed ?d)
            )
    )

    ;; Mark the mission complete once the drone is down at the destination
    (:action complete-mission
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?l - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (at ?d ?l)
                (destination ?l)
                (landed ?d)
                (>= (battery ?d) (charge-margin ?d))
            )
        ;; These facts and values change after the action
        :effect
            (mission-completed ?d)
    )

    ;; Record GPS loss during flight
    (:action detect-gps-loss
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (airborne ?d)
                (not (gps-ok ?d))
            )
        ;; These facts and values change after the action
        :effect
            (gps-lost ?d)
    )

    ;; Record loss of contact with the ground controller
    (:action detect-communication-loss
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (airborne ?d)
                (not (communication-ok ?d))
            )
        ;; These facts and values change after the action
        :effect
            (communication-lost ?d)
    )

    ;; Record that the ground controller has been told about the failure
    (:action notify-ground-controller
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?c - controller)
        ;; These conditions must hold before the action
        :precondition
            (and
                ;; At least one of the following conditions must hold
                (or
                    (gps-lost ?d)
                    (communication-lost ?d)
                )
            )
        ;; These facts and values change after the action
        :effect
            (controller-notified ?d)
    )
)
