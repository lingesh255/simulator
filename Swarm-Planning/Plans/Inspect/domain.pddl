;; One drone inspects the road and follows it back to the source.

(define (domain road-inspection-drone-mission)

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
        (source ?l - location) ;; Where the mission starts
        (destination ?l - location) ;; Where the mission goes
        (road-segment ?from - location ?to - location) ;; A road section to inspect
        (segment-inspected ?from - location ?to - location) ;; This road section has been inspected
        (road-inspected) ;; The whole road has been inspected
        (inspector ?d - drone) ;; The drone assigned to inspect the road
        (landed ?d - drone) ;; The drone is on the ground
        (airborne ?d - drone) ;; The drone has taken off
        (flying ?d - drone) ;; The drone is flying
        (at-cruise-altitude ?d - drone) ;; The drone has reached its assigned height
        (preflight-done ?d - drone) ;; The drone has passed the checks before takeoff
        (gps-ok ?d - drone) ;; The drone has a GPS fix
        (communication-ok ?d - drone) ;; The model treats the ground-controller link as working
        (drone-healthy ?d - drone) ;; The drone passes the health check
        (gps-lost ?d - drone) ;; GPS loss has been recorded
        (communication-lost ?d - drone) ;; A communication failure has been recorded
        (health-failure ?d - drone) ;; Low drone health has been recorded
        (controller-notified ?d - drone) ;; The controller has been told about a failure
        (mission-completed ?d - drone) ;; The drone has finished its mission
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
        (energy-required ?from - location ?to - location) ;; Battery needed for this route leg
        (distance ?from - location ?to - location) ;; Distance along this route leg
        (health ?d - drone) ;; Current health score
        (minimum-health ?d - drone) ;; Lowest allowed health score
        (latitude ?l - location) ;; North-south map coordinate
        (longitude ?l - location) ;; East-west map coordinate
        (segments-total) ;; Number of road sections to inspect
        (segments-inspected) ;; Number of road sections already inspected
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

    ;; Inspect the next road section and use the route battery
    (:action inspect-segment
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?from - location
             ?to - location)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
        :effect
            (and
                (not (at ?d ?from))
                (at ?d ?to)
                (segment-inspected ?from ?to)
                (increase (segments-inspected) 1)
                (decrease (battery ?d) (energy-required ?from ?to))
            )
    )

    ;; Mark the road inspected after all required sections are done
    (:action finish-inspection
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?l - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (inspector ?d)
                (destination ?l)
                (at ?d ?l)
                (>= (segments-inspected) (segments-total))
            )
        ;; These facts and values change after the action
        :effect
            (road-inspected)
    )

    ;; Fly back along one road section
    (:action return-along-road
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?from - location
             ?to - location)
        ;; These conditions must hold before the action
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
        ;; These facts and values change after the action
        :effect
            (and
                (not (at ?d ?from))
                (at ?d ?to)
                (decrease (battery ?d) (energy-required ?from ?to))
            )
    )

    ;; Land at the source and mark the mission complete
    (:action land-at-source
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?l - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (inspector ?d)
                (road-inspected)
                (source ?l)
                (at ?d ?l)
                (airborne ?d)
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (airborne ?d))
                (not (flying ?d))
                (not (at-cruise-altitude ?d))
                (landed ?d)
                (mission-completed ?d)
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
