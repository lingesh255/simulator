;; Drones fly out in turn, circle on their assigned orbits and return.

(define (domain planets-formation-drone-mission)

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
        (landed ?d - drone) ;; The drone is on the ground
        (airborne ?d - drone) ;; The drone has taken off
        (flying ?d - drone) ;; The drone is flying
        (at-cruise-altitude ?d - drone) ;; The drone has reached its assigned height
        (formation-leader ?d - drone) ;; The drone leads the formation
        (planet ?d - drone) ;; The drone flies one of the orbits
        (has-orbit ?d - drone) ;; The drone has an assigned orbit
        (orbits-clockwise ?d - drone) ;; The drone turns clockwise when viewed from above
        (orbits-anticlockwise ?d - drone) ;; The drone turns anticlockwise when viewed from above
        (orbit-neighbor ?d1 - drone ?d2 - drone) ;; Two drones have neighbouring orbits
        (on-orbit ?d - drone) ;; The drone has reached its orbit start position
        (returned ?d - drone) ;; The drone has flown back to the source
        (out-turn ?d - drone) ;; It is this drone's turn to fly out
        (out-after ?d1 - drone ?d2 - drone) ;; The second drone flies out after the first
        (back-turn ?d - drone) ;; It is this drone's turn to fly back
        (back-after ?d1 - drone ?d2 - drone) ;; The second drone flies back after the first
        (orbits-entered) ;; All drones have entered their orbits
        (show-ended) ;; The show is over and drones may return
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
        (planets-mission-complete) ;; The orbit show mission is complete
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
        (transit-energy ?d - drone) ;; Battery used for a flight between source and show
        (health ?d - drone) ;; Current health score
        (minimum-health ?d - drone) ;; Lowest allowed health score
        (latitude ?l - location) ;; North-south map coordinate
        (longitude ?l - location) ;; East-west map coordinate
        (orbit-radius ?d - drone) ;; Distance from the orbit centre
        (orbit-speed ?d - drone) ;; Speed along the orbit
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
                (not (orbits-entered))
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

    ;; Fly one drone to its orbit start when it is its turn
    (:action fly-to-orbit
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?from - location
             ?to - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (at ?d ?from)
                (source ?from)
                (destination ?to)
                (has-orbit ?d)
                (at-cruise-altitude ?d)
                (not (on-orbit ?d))
                (not (orbits-entered))
                (out-turn ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
                (> (battery ?d) (+ (transit-energy ?d) (minimum-return-battery ?d)))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (at ?d ?from))
                (at ?d ?to)
                (on-orbit ?d)
                (not (out-turn ?d))
                ;; Apply this check or update to every matching object
                (forall (?n - drone)
                    ;; Make this change only for objects that match the condition
                    (when (out-after ?d ?n) (out-turn ?n)))
                (decrease (battery ?d) (transit-energy ?d))
            )
    )

    ;; Start the orbit show when all drones are ready and neighbouring orbits turn opposite ways
    (:action enter-orbits
        ;; Objects used by this action
        :parameters
            (?lead - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (formation-leader ?lead)
                (neighbor-comm-ok ?lead)
                (not (orbits-entered))
                ;; Apply this check or update to every matching object
                (forall (?m - drone)
                    ;; At least one of the following conditions must hold
                    (or
                        (not (has-orbit ?m))
                        (and
                            (on-orbit ?m)
                            (neighbor-comm-ok ?m)
                        )
                    )
                )
                ;; Apply this check or update to every matching object
                (forall (?a - drone ?b - drone)
                    ;; At least one of the following conditions must hold
                    (or
                        (not (orbit-neighbor ?a ?b))
                        (and (orbits-clockwise ?a) (orbits-anticlockwise ?b))
                        (and (orbits-anticlockwise ?a) (orbits-clockwise ?b))
                    )
                )
            )
        ;; These facts and values change after the action
        :effect
            (orbits-entered)
    )

    ;; Count one completed orbit lap
    (:action fly-lap
        ;; Objects used by this action
        :parameters
            (?lead - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (formation-leader ?lead)
                (orbits-entered)
                (not (show-ended))
                (< (laps-completed) (laps-required))
                (gps-ok ?lead)
                (communication-ok ?lead)
            )
        ;; These facts and values change after the action
        :effect
            (increase (laps-completed) 1)
    )

    ;; End the show after the required laps
    (:action end-show
        ;; Objects used by this action
        :parameters
            (?lead - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (formation-leader ?lead)
                (orbits-entered)
                (not (show-ended))
                (>= (laps-completed) (laps-required))
            )
        ;; These facts and values change after the action
        :effect
            (show-ended)
    )

    ;; Fly one drone back to the source when it is its turn
    (:action return-to-source
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?from - location
             ?to - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (show-ended)
                (at ?d ?from)
                (destination ?from)
                (source ?to)
                (on-orbit ?d)
                (back-turn ?d)
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (at ?d ?from))
                (at ?d ?to)
                (not (on-orbit ?d))
                (returned ?d)
                (not (back-turn ?d))
                ;; Apply this check or update to every matching object
                (forall (?n - drone)
                    ;; Make this change only for objects that match the condition
                    (when (back-after ?d ?n) (back-turn ?n)))
                (decrease (battery ?d) (transit-energy ?d))
            )
    )

    ;; Land after flying back and mark the drone mission complete
    (:action land
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?l - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (returned ?d)
                (at ?d ?l)
                (source ?l)
                (airborne ?d)
            )
        ;; These facts and values change after the action
        :effect
            (and
                (landed ?d)
                (not (airborne ?d))
                (not (flying ?d))
                (not (at-cruise-altitude ?d))
                (assign (altitude ?d) 0)
                (mission-completed ?d)
            )
    )

    ;; Finish the orbit mission after every drone completes its return and landing
    (:action complete-planets
        ;; Objects used by this action
        :parameters
            (?l - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (source ?l)
                (show-ended)
                ;; Apply this check or update to every matching object
                (forall (?d - drone)
                    ;; At least one of the following conditions must hold
                    (or
                        (not (has-orbit ?d))
                        (mission-completed ?d)
                    )
                )
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (orbits-entered))
                (planets-mission-complete)
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
