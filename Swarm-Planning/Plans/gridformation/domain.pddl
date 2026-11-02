;; Drones follow a route in a flat grid.

(define (domain grid-formation-drone-mission)

    ;; Planning features used by these actions
    (:requirements
        :strips ;; Use actions with conditions and effects
        :typing ;; Give objects types such as drone and location
        :equality ;; Compare whether two objects are the same
        :numeric-fluents ;; Use numbers such as battery, distance and height
        :negative-preconditions ;; Allow a condition to require that a fact is false
        :disjunctive-preconditions ;; Allow a choice between conditions
        :universal-preconditions ;; Check a condition for every matching object
        :conditional-effects ;; Apply an effect only when its condition holds
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
        (formation-at ?l - location) ;; Where the formation is
        (source ?l - location) ;; Where the mission starts
        (destination ?l - location) ;; Where the mission goes
        (connected ?from - location ?to - location) ;; A route connects these locations
        (safe-route ?from - location ?to - location) ;; This route is allowed
        (landed ?d - drone) ;; The drone is on the ground
        (airborne ?d - drone) ;; The drone has taken off
        (flying ?d - drone) ;; The drone is flying
        (at-cruise-altitude ?d - drone) ;; The drone has reached its assigned height
        (has-slot ?d - drone) ;; The drone has an assigned formation position
        (grid-member ?d - drone) ;; The drone belongs to the grid
        (formation-leader ?d - drone) ;; The drone leads the formation
        (in-formation ?d - drone) ;; The drone has joined the formation
        (grid-formation-established) ;; The grid formation has been set up
        (grid-closed) ;; The grid members have caught up
        (grid-neighbor ?d1 - drone ?d2 - drone) ;; Two drones occupy neighbouring grid cells
        (neighbor-comm-ok ?d - drone) ;; The model treats the required neighbour links as working
        (preflight-done ?d - drone) ;; The drone has passed the checks before takeoff
        (gps-ok ?d - drone) ;; The drone has a GPS fix
        (communication-ok ?d - drone) ;; The model treats the ground-controller link as working
        (drone-healthy ?d - drone) ;; The drone passes the health check
        (gps-lost ?d - drone) ;; GPS loss has been recorded
        (communication-lost ?d - drone) ;; A communication failure has been recorded
        (health-failure ?d - drone) ;; Low drone health has been recorded
        (controller-notified ?d - drone) ;; The controller has been told about a failure
        (mission-completed ?d - drone) ;; The drone has finished its mission
        (formation-mission-complete) ;; The formation mission is complete
    )

    ;; Numbers used by the planner
    (:functions
        (battery ?d - drone) ;; Remaining battery in planning units
        (max-battery ?d - drone) ;; Full battery in planning units
        (minimum-return-battery ?d - drone) ;; Battery reserve for returning
        (altitude ?d - drone) ;; Current height of the drone
        (hover-altitude ?d - drone) ;; Height just after takeoff
        (target-altitude ?d - drone) ;; Assigned cruise height
        (climb-energy ?d - drone) ;; Battery used to reach cruise height
        (cruise-leg-energy ?d - drone) ;; Battery used by a member for one route leg
        (energy-required ?from - location ?to - location) ;; Battery needed for this route leg
        (distance ?from - location ?to - location) ;; Distance along this route leg
        (health ?d - drone) ;; Current health score
        (minimum-health ?d - drone) ;; Lowest allowed health score
        (latitude ?l - location) ;; North-south map coordinate
        (longitude ?l - location) ;; East-west map coordinate
        (slot-along-offset ?d - drone) ;; Forward or backward distance from the leader
        (slot-cross-offset ?d - drone) ;; Sideways distance from the leader
        (seconds-since-leader-airborne) ;; Waiting time counted after the leader takes off
        (grid-launch-delay) ;; How long grid members wait before takeoff
    )

    ;; Check the drone before takeoff
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
                (has-slot ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (>= (health ?d) (minimum-health ?d))
                (> (battery ?d) (minimum-return-battery ?d))
            )
        ;; These facts and values change after the action
        :effect
            (preflight-done ?d)
    )

    ;; Take off and move to hover height
    (:action takeoff
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
                (preflight-done ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                ;; At least one of the following conditions must hold
                (or
                    (formation-leader ?d)
                    (>= (seconds-since-leader-airborne) (grid-launch-delay))
                )
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (landed ?d))
                (airborne ?d)
                (flying ?d)
                (assign (altitude ?d) (hover-altitude ?d))
            )
    )

    ;; Climb to the assigned height and use battery for the climb
    (:action climb-to-cruise-altitude
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
        :effect
            (and
                (assign (altitude ?d) (target-altitude ?d))
                (at-cruise-altitude ?d)
                (decrease (battery ?d) (climb-energy ?d))
            )
    )

    ;; Count one second of waiting after the leader takes off
    (:action hold-position
        ;; Objects used by this action
        :parameters
            (?lead - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (formation-leader ?lead)
                (airborne ?lead)
                (< (seconds-since-leader-airborne) (grid-launch-delay))
                (gps-ok ?lead)
                (communication-ok ?lead)
                (drone-healthy ?lead)
            )
        ;; These facts and values change after the action
        :effect
            (increase (seconds-since-leader-airborne) 1)
    )

    ;; Join the assigned grid position
    (:action take-grid-slot
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
        :effect
            (in-formation ?d)
    )

    ;; Set up the grid formation when the required drones are ready
    (:action establish-grid-formation
        ;; Objects used by this action
        :parameters
            (?lead - drone
             ?l - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (formation-leader ?lead)
                (in-formation ?lead)
                (at ?lead ?l)
                (at-cruise-altitude ?lead)
                (>= (altitude ?lead) (target-altitude ?lead))
                (neighbor-comm-ok ?lead)
                ;; Apply this check or update to every matching object
                (forall (?m - drone)
                    ;; At least one of the following conditions must hold
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
        ;; These facts and values change after the action
        :effect
            (and
                (grid-formation-established)
                (formation-at ?l)
                (grid-closed)
            )
    )

    ;; Move the leader to the next location and use the route battery
    (:action formation-cruise
        ;; Objects used by this action
        :parameters
            (?lead - drone
             ?from - location
             ?to - location)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
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

    ;; Move the grid members to the leader's new location
    (:action close-up-grid
        ;; Objects used by this action
        :parameters
            (?lead - drone
             ?to - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (grid-formation-established)
                (not (grid-closed))
                (formation-leader ?lead)
                (in-formation ?lead)
                (formation-at ?to)
                (at ?lead ?to)
                ;; Apply this check or update to every matching object
                (forall (?m - drone)
                    ;; At least one of the following conditions must hold
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
        ;; These facts and values change after the action
        :effect
            (and
                ;; Apply this check or update to every matching object
                (forall (?m - drone) (forall (?x - location)
                    ;; Make this change only for objects that match the condition
                    (when
                        (and
                            (grid-member ?m)
                            (at ?m ?x)
                        )
                        (not (at ?m ?x))
                    )
                ))
                ;; Apply this check or update to every matching object
                (forall (?m - drone)
                    ;; Make this change only for objects that match the condition
                    (when
                        (grid-member ?m)
                        (at ?m ?to)
                    )
                )
                ;; Apply this check or update to every matching object
                (forall (?m - drone)
                    ;; Make this change only for objects that match the condition
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

    ;; Finish the grid mission at the destination
    (:action complete-grid-mission
        ;; Objects used by this action
        :parameters
            (?lead - drone
             ?dest - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (destination ?dest)
                (grid-formation-established)
                (formation-at ?dest)
                (formation-leader ?lead)
                (in-formation ?lead)
                (at ?lead ?dest)
                ;; Apply this check or update to every matching object
                (forall (?m - drone)
                    ;; At least one of the following conditions must hold
                    (or
                        (not (grid-member ?m))
                        (and
                            (in-formation ?m)
                            (at ?m ?dest)
                        )
                    )
                )
            )
        ;; These facts and values change after the action
        :effect
            (and
                (mission-completed ?lead)
                ;; Apply this check or update to every matching object
                (forall (?m - drone)
                    ;; Make this change only for objects that match the condition
                    (when
                        (grid-member ?m)
                        (mission-completed ?m)
                    )
                )
                (formation-mission-complete)
            )
    )

    ;; Record GPS loss during flight
    (:action detect-gps-loss
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (flying ?d)
                (not (gps-ok ?d))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (gps-lost ?d)
                (not (flying ?d))
                (not (in-formation ?d))
            )
    )

    ;; Record loss of contact with the ground controller
    (:action detect-communication-loss
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (flying ?d)
                (not (communication-ok ?d))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (communication-lost ?d)
                (not (flying ?d))
                (not (in-formation ?d))
            )
    )

    ;; Record loss of the required neighbour links
    (:action detect-neighbor-link-loss
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (flying ?d)
                (not (neighbor-comm-ok ?d))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (communication-lost ?d)
                (not (flying ?d))
                (not (in-formation ?d))
            )
    )

    ;; Record a health score below the allowed limit
    (:action detect-health-failure
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (flying ?d)
                (< (health ?d) (minimum-health ?d))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (health-failure ?d)
                (not (flying ?d))
                (not (in-formation ?d))
            )
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
                    (health-failure ?d)
                )
            )
        ;; These facts and values change after the action
        :effect
            (controller-notified ?d)
    )
)
