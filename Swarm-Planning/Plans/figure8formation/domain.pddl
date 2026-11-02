;; Drones fly the figure-eight pattern for the required laps and return.

(define (domain figure8-formation-drone-mission)

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
        (base ?l - location) ;; Where drones leave and return
        (lap-start ?l - location) ;; Where a lap starts
        (lap-end ?l - location) ;; Where a lap ends
        (connected ?from - location ?to - location) ;; A route connects these locations
        (safe-route ?from - location ?to - location) ;; This route is allowed
        (transit-route ?from - location ?to - location) ;; A route between the base and the pattern
        (crossing-point ?l - location) ;; Where the two loops cross
        (landed ?d - drone) ;; The drone is on the ground
        (airborne ?d - drone) ;; The drone has taken off
        (flying ?d - drone) ;; The drone is flying
        (at-cruise-altitude ?d - drone) ;; The drone has reached its assigned height
        (train-leader ?d - drone) ;; The drone leads the train
        (train-follower ?d - drone) ;; The drone follows in the train
        (has-slot ?d - drone) ;; The drone has an assigned formation position
        (in-formation ?d - drone) ;; The drone has joined the formation
        (figure8-established) ;; The figure-eight formation has been set up
        (train-closed) ;; The followers have caught up
        (pattern-entered) ;; The train has reached the figure-eight route
        (laps-finished) ;; The required laps have been finished
        (train-neighbor ?d1 - drone ?d2 - drone) ;; Two drones are neighbours in the train
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
        (figure8-mission-complete) ;; The figure-eight mission is complete
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
        (slot-arc-offset ?d - drone) ;; Distance along the path relative to the leader
        (drone-spacing) ;; Required spacing along the pattern
        (crossing-gap) ;; Gap along the path between drones sharing the crossing
        (crossing-clearance) ;; Required gap at the crossing
        (laps-required) ;; Number of laps to fly
        (laps-completed) ;; Number of laps already counted as complete
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
                (base ?l)
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
                (base ?l)
                (landed ?d)
                (preflight-done ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
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

    ;; Join the assigned train position
    (:action take-train-slot
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

    ;; Set up the figure-eight formation when the required drones are ready
    (:action establish-figure8-formation
        ;; Objects used by this action
        :parameters
            (?lead - drone
             ?l - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (train-leader ?lead)
                (in-formation ?lead)
                (at ?lead ?l)
                (at-cruise-altitude ?lead)
                (>= (altitude ?lead) (target-altitude ?lead))
                (neighbor-comm-ok ?lead)
                (>= (crossing-gap) (crossing-clearance))
                ;; Apply this check or update to every matching object
                (forall (?f - drone)
                    ;; At least one of the following conditions must hold
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
        ;; These facts and values change after the action
        :effect
            (and
                (figure8-established)
                (formation-at ?l)
                (train-closed)
            )
    )

    ;; Move the leader from the base to the figure-eight route
    (:action fly-to-pattern
        ;; Objects used by this action
        :parameters
            (?lead - drone
             ?from - location
             ?to - location)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
        :effect
            (and
                (pattern-entered)
                (not (formation-at ?from))
                (formation-at ?to)
                (not (at ?lead ?from))
                (at ?lead ?to)
                (not (train-closed))
                (decrease (battery ?lead) (energy-required ?from ?to))
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
        ;; These facts and values change after the action
        :effect
            (and
                (not (formation-at ?from))
                (formation-at ?to)
                (not (at ?lead ?from))
                (at ?lead ?to)
                (not (train-closed))
                (decrease (battery ?lead) (energy-required ?from ?to))
            )
    )

    ;; Move the followers to the leader's new location
    (:action follow-leader
        ;; Objects used by this action
        :parameters
            (?lead - drone
             ?to - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (figure8-established)
                (not (train-closed))
                (train-leader ?lead)
                (in-formation ?lead)
                (formation-at ?to)
                (at ?lead ?to)
                ;; Apply this check or update to every matching object
                (forall (?f - drone)
                    ;; At least one of the following conditions must hold
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
        ;; These facts and values change after the action
        :effect
            (and
                ;; Apply this check or update to every matching object
                (forall (?f - drone) (forall (?x - location)
                    ;; Make this change only for objects that match the condition
                    (when (and (train-follower ?f) (at ?f ?x))
                        (not (at ?f ?x)))))
                ;; Apply this check or update to every matching object
                (forall (?f - drone)
                    ;; Make this change only for objects that match the condition
                    (when (train-follower ?f) (at ?f ?to)))
                ;; Apply this check or update to every matching object
                (forall (?f - drone)
                    ;; Make this change only for objects that match the condition
                    (when (train-follower ?f)
                        (decrease (battery ?f) (cruise-leg-energy ?f))))
                (train-closed)
            )
    )

    ;; Count the finished lap and prepare the next lap
    (:action start-next-lap
        ;; Objects used by this action
        :parameters
            (?lead - drone
             ?dest - location
             ?start - location)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
        :effect
            (and
                (not (formation-at ?dest))
                (formation-at ?start)
                (not (at ?lead ?dest))
                (at ?lead ?start)
                ;; Apply this check or update to every matching object
                (forall (?f - drone)
                    ;; Make this change only for objects that match the condition
                    (when (and (train-follower ?f) (at ?f ?dest))
                        (and (not (at ?f ?dest)) (at ?f ?start))))
                (increase (laps-completed) 1)
            )
    )

    ;; Record that the required figure-eight laps are finished
    (:action finish-laps
        ;; Objects used by this action
        :parameters
            (?lead - drone
             ?end - location)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
        :effect
            (and
                (laps-finished)
                (increase (laps-completed) 1)
            )
    )

    ;; Return to the base after finishing the route or search
    (:action return-to-base
        ;; Objects used by this action
        :parameters
            (?lead - drone
             ?from - location
             ?to - location)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
        :effect
            (and
                (not (formation-at ?from))
                (formation-at ?to)
                (not (at ?lead ?from))
                (at ?lead ?to)
                (not (train-closed))
                (decrease (battery ?lead) (energy-required ?from ?to))
            )
    )

    ;; Finish the figure-eight mission after returning to base
    (:action complete-figure8-mission
        ;; Objects used by this action
        :parameters
            (?lead - drone
             ?home - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (base ?home)
                (laps-finished)
                (train-closed)
                (formation-at ?home)
                (train-leader ?lead)
                (in-formation ?lead)
                (at ?lead ?home)
                ;; Apply this check or update to every matching object
                (forall (?f - drone)
                    ;; At least one of the following conditions must hold
                    (or
                        (not (train-follower ?f))
                        (and
                            (in-formation ?f)
                            (at ?f ?home)
                        )
                    )
                )
            )
        ;; These facts and values change after the action
        :effect
            (and
                (mission-completed ?lead)
                ;; Apply this check or update to every matching object
                (forall (?f - drone)
                    ;; Make this change only for objects that match the condition
                    (when (train-follower ?f)
                        (mission-completed ?f)))
                (figure8-mission-complete)
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
