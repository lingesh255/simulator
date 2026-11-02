;; A middle drone and two higher arm drones follow one route.

(define (domain parabola-formation-drone-mission)

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
        (formation-leader ?d - drone) ;; The drone leads the formation
        (slot-vertex ?d - drone) ;; The drone takes the lower middle position
        (arm-drone ?d - drone) ;; The drone takes a raised side position
        (has-slot ?d - drone) ;; The drone has an assigned formation position
        (in-formation ?d - drone) ;; The drone has joined the formation
        (parabola-established) ;; The parabola formation has been set up
        (shape-held) ;; The parabola shape is ready for the next move
        (parabola-neighbor ?d1 - drone ?d2 - drone) ;; Two drones need a link in the parabola
        (link-up ?d1 - drone ?d2 - drone) ;; A link has been set up in this direction
        (neighbor-comm-ok ?d - drone) ;; The model treats the required neighbour links as working
        (swarm-comm-ready ?d - drone) ;; The drone has links to all its required neighbours
        (preflight-done ?d - drone) ;; The drone has passed the checks before takeoff
        (gps-ok ?d - drone) ;; The drone has a GPS fix
        (communication-ok ?d - drone) ;; The model treats the ground-controller link as working
        (drone-healthy ?d - drone) ;; The drone passes the health check
        (gps-lost ?d - drone) ;; GPS loss has been recorded
        (communication-lost ?d - drone) ;; A communication failure has been recorded
        (health-failure ?d - drone) ;; Low drone health has been recorded
        (controller-notified ?d - drone) ;; The controller has been told about a failure
        (mission-completed ?d - drone) ;; The drone has finished its mission
        (parabola-mission-complete) ;; The parabola mission is complete
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
        (arm-rise) ;; Required height difference between an arm and the middle drone
        (min-separation) ;; Smallest allowed gap between drones
        (link-distance ?d1 - drone ?d2 - drone) ;; Distance between two drones in the formation
        (comm-range) ;; Largest allowed distance for a neighbour link
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

    ;; Set up one neighbour link when the drones are ready and within range
    (:action establish-comm-link
        ;; Objects used by this action
        :parameters
            (?d1 - drone
             ?d2 - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (parabola-neighbor ?d1 ?d2)
                (not (link-up ?d1 ?d2))
                (airborne ?d1)
                (airborne ?d2)
                (at-cruise-altitude ?d1)
                (at-cruise-altitude ?d2)
                (communication-ok ?d1)
                (communication-ok ?d2)
                (neighbor-comm-ok ?d1)
                (neighbor-comm-ok ?d2)
                (<= (link-distance ?d1 ?d2) (comm-range))
            )
        ;; These facts and values change after the action
        :effect
            (link-up ?d1 ?d2)
    )

    ;; Confirm that the drone has links to all its required neighbours
    (:action confirm-swarm-comm
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (airborne ?d)
                (not (swarm-comm-ready ?d))
                (communication-ok ?d)
                (neighbor-comm-ok ?d)
                ;; Apply this check or update to every matching object
                (forall (?n - drone)
                    ;; At least one of the following conditions must hold
                    (or
                        (not (parabola-neighbor ?d ?n))
                        (link-up ?d ?n)
                    )
                )
            )
        ;; These facts and values change after the action
        :effect
            (swarm-comm-ready ?d)
    )

    ;; Join the assigned parabola position
    (:action take-parabola-slot
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
                (swarm-comm-ready ?d)
                (gps-ok ?d)
                (communication-ok ?d)
                (drone-healthy ?d)
            )
        ;; These facts and values change after the action
        :effect
            (in-formation ?d)
    )

    ;; Set up the parabola formation when the required drones are ready
    (:action establish-parabola-formation
        ;; Objects used by this action
        :parameters
            (?vtx - drone
             ?l - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (formation-leader ?vtx)
                (slot-vertex ?vtx)
                (in-formation ?vtx)
                (at ?vtx ?l)
                (at-cruise-altitude ?vtx)
                (>= (altitude ?vtx) (target-altitude ?vtx))
                (swarm-comm-ready ?vtx)
                ;; Apply this check or update to every matching object
                (forall (?a - drone)
                    ;; At least one of the following conditions must hold
                    (or
                        (not (arm-drone ?a))
                        (and
                            (in-formation ?a)
                            (at ?a ?l)
                            (at-cruise-altitude ?a)
                            (>= (altitude ?a) (target-altitude ?a))
                            (= (- (target-altitude ?a) (target-altitude ?vtx)) (arm-rise))
                            (>= (link-distance ?vtx ?a) (min-separation))
                            (swarm-comm-ready ?a)
                        )
                    )
                )
            )
        ;; These facts and values change after the action
        :effect
            (and
                (parabola-established)
                (formation-at ?l)
                (shape-held)
            )
    )

    ;; Move the leader to the next location and use the route battery
    (:action formation-cruise
        ;; Objects used by this action
        :parameters
            (?vtx - drone
             ?from - location
             ?to - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (parabola-established)
                (shape-held)
                (formation-leader ?vtx)
                (slot-vertex ?vtx)
                (in-formation ?vtx)
                (formation-at ?from)
                (at ?vtx ?from)
                (connected ?from ?to)
                (safe-route ?from ?to)
                (at-cruise-altitude ?vtx)
                (>= (altitude ?vtx) (target-altitude ?vtx))
                (gps-ok ?vtx)
                (communication-ok ?vtx)
                (drone-healthy ?vtx)
                (neighbor-comm-ok ?vtx)
                (swarm-comm-ready ?vtx)
                (> (battery ?vtx) (energy-required ?from ?to))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (formation-at ?from))
                (formation-at ?to)
                (not (at ?vtx ?from))
                (at ?vtx ?to)
                (not (shape-held))
                (decrease (battery ?vtx) (energy-required ?from ?to))
            )
    )

    ;; Move the arm drones to the next location and check the shape and links
    (:action hold-parabola-shape
        ;; Objects used by this action
        :parameters
            (?vtx - drone
             ?to - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (parabola-established)
                (not (shape-held))
                (formation-leader ?vtx)
                (in-formation ?vtx)
                (formation-at ?to)
                (at ?vtx ?to)
                ;; Apply this check or update to every matching object
                (forall (?d1 - drone)
                    ;; Apply this check or update to every matching object
                    (forall (?d2 - drone)
                        ;; At least one of the following conditions must hold
                        (or
                            (not (parabola-neighbor ?d1 ?d2))
                            (link-up ?d1 ?d2)
                        )
                    )
                )
                ;; Apply this check or update to every matching object
                (forall (?a - drone)
                    ;; At least one of the following conditions must hold
                    (or
                        (not (arm-drone ?a))
                        (and
                            (in-formation ?a)
                            (at-cruise-altitude ?a)
                            (>= (altitude ?a) (target-altitude ?a))
                            (gps-ok ?a)
                            (communication-ok ?a)
                            (drone-healthy ?a)
                            (neighbor-comm-ok ?a)
                            (swarm-comm-ready ?a)
                            (> (battery ?a) (cruise-leg-energy ?a))
                        )
                    )
                )
            )
        ;; These facts and values change after the action
        :effect
            (and
                ;; Apply this check or update to every matching object
                (forall (?a - drone) (forall (?x - location)
                    ;; Make this change only for objects that match the condition
                    (when (and (arm-drone ?a) (at ?a ?x))
                        (not (at ?a ?x)))))
                ;; Apply this check or update to every matching object
                (forall (?a - drone)
                    ;; Make this change only for objects that match the condition
                    (when (arm-drone ?a) (at ?a ?to)))
                ;; Apply this check or update to every matching object
                (forall (?a - drone)
                    ;; Make this change only for objects that match the condition
                    (when (arm-drone ?a)
                        (decrease (battery ?a) (cruise-leg-energy ?a))))
                (shape-held)
            )
    )

    ;; Finish the parabola mission at the destination
    (:action complete-parabola-mission
        ;; Objects used by this action
        :parameters
            (?vtx - drone
             ?dest - location)
        ;; These conditions must hold before the action
        :precondition
            (and
                (destination ?dest)
                (parabola-established)
                (shape-held)
                (formation-at ?dest)
                (formation-leader ?vtx)
                (slot-vertex ?vtx)
                (in-formation ?vtx)
                (at ?vtx ?dest)
                ;; Apply this check or update to every matching object
                (forall (?a - drone)
                    ;; At least one of the following conditions must hold
                    (or
                        (not (arm-drone ?a))
                        (and
                            (in-formation ?a)
                            (at ?a ?dest)
                        )
                    )
                )
            )
        ;; These facts and values change after the action
        :effect
            (and
                (mission-completed ?vtx)
                ;; Apply this check or update to every matching object
                (forall (?a - drone)
                    ;; Make this change only for objects that match the condition
                    (when (arm-drone ?a)
                        (mission-completed ?a)))
                (parabola-mission-complete)
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
                (not (swarm-comm-ready ?d))
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
                (not (swarm-comm-ready ?d))
                ;; Apply this check or update to every matching object
                (forall (?n - drone)
                    ;; Make this change only for objects that match the condition
                    (when (parabola-neighbor ?d ?n)
                        (and
                            (not (link-up ?d ?n))
                            (not (link-up ?n ?d))
                        )
                    )
                )
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
