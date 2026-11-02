(define (domain parabola-formation-drone-mission)
    ;; Upward-parabola formation: three drones fly side by side on one line
    ;; across the direction of travel. The middle drone (the vertex) cruises
    ;; at 50 m and the two arm drones either side of it at 60 m, so seen
    ;; from behind the three make a "U" - an upward-opening parabola - and
    ;; fly it from the source to the destination. Every drone keeps a live
    ;; link with each of the other two the whole way
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
        ;; Current location of the whole formation
        (formation-at ?l - location)
        ;; Mission locations and routes
        (source ?l - location)
        (destination ?l - location)
        (connected ?from - location ?to - location)
        (safe-route ?from - location ?to - location)
        ;; Flight states
        (landed ?d - drone)
        (airborne ?d - drone)
        (flying ?d - drone)
        (at-cruise-altitude ?d - drone)
        ;; Formation slots: the vertex in the middle (lowest point of the
        ;; parabola) and an arm drone either side of it on the same line
        (formation-leader ?d - drone)
        (slot-vertex ?d - drone)
        (arm-drone ?d - drone)
        (has-slot ?d - drone)
        (in-formation ?d - drone)
        ;; Formation status
        (parabola-established)
        (shape-held)
        ;; Communication between the drones. parabola-neighbor lists every
        ;; pair that must talk (all three pairs here - the two arms talk to
        ;; the vertex and to each other); link-up is a link that has been
        ;; set up and is in range; swarm-comm-ready means a drone has a
        ;; working link with every one of its neighbors
        (parabola-neighbor ?d1 - drone ?d2 - drone)
        (link-up ?d1 - drone ?d2 - drone)
        (neighbor-comm-ok ?d - drone)
        (swarm-comm-ready ?d - drone)
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
        (parabola-mission-complete)
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
        ;; Distance between locations
        (distance ?from - location ?to - location)
        ;; Drone health
        (health ?d - drone)
        (minimum-health ?d - drone)
        ;; Location coordinates
        (latitude ?l - location)
        (longitude ?l - location)
        ;; Parabola slot offsets: metres behind the vertex (negative =
        ;; behind) and metres to its right (negative = left). Both arms sit
        ;; level with the vertex, on the same line across the route
        (slot-along-offset ?d - drone)
        (slot-cross-offset ?d - drone)
        ;; Parabola shape: how much higher each arm flies than the vertex
        ;; (it must be exactly this, the same for both arms, so the shape is
        ;; a symmetric "U"), and the smallest 3-D gap any two drones may have
        (arm-rise)
        (min-separation)
        ;; Communication: straight-line distance between two drones in the
        ;; formation, and how far a drone-to-drone link can reach
        (link-distance ?d1 - drone ?d2 - drone)
        (comm-range)
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
    ;; Take off from the drone's own pad at the source - the pads are
    ;; already on the formation's line, one slot spacing apart, so the
    ;; three drones can lift off together without crossing
    (:action takeoff
        :parameters
            (?d - drone
             ?l - location)
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
        :effect
            (and
                (not (landed ?d))
                (airborne ?d)
                (flying ?d)
                (assign (altitude ?d) (hover-altitude ?d))
            )
    )
    ;; Climb from hover height to the drone's own cruise altitude - 50 m for
    ;; the vertex, 60 m for each arm
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
    ;; Set up the link from one drone to a neighbor - both must be up at
    ;; cruise altitude with their radios working, and close enough for the
    ;; link to reach
    (:action establish-comm-link
        :parameters
            (?d1 - drone
             ?d2 - drone)
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
        :effect
            (link-up ?d1 ?d2)
    )
    ;; A drone is ready to fly in the swarm once it has a link to every one
    ;; of its neighbors
    (:action confirm-swarm-comm
        :parameters
            (?d - drone)
        :precondition
            (and
                (airborne ?d)
                (not (swarm-comm-ready ?d))
                (communication-ok ?d)
                (neighbor-comm-ok ?d)
                (forall (?n - drone)
                    (or
                        (not (parabola-neighbor ?d ?n))
                        (link-up ?d ?n)
                    )
                )
            )
        :effect
            (swarm-comm-ready ?d)
    )
    ;; Move the drone into its assigned parabola slot
    (:action take-parabola-slot
        :parameters
            (?d - drone)
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
        :effect
            (in-formation ?d)
    )
    ;; Establish the parabola when all three drones are in their slots and
    ;; talking to each other. The shape must open upward: each arm flies
    ;; exactly arm-rise above the vertex (so both arms are level with each
    ;; other), and no two drones are closer than the minimum separation
    (:action establish-parabola-formation
        :parameters
            (?vtx - drone
             ?l - location)
        :precondition
            (and
                (formation-leader ?vtx)
                (slot-vertex ?vtx)
                (in-formation ?vtx)
                (at ?vtx ?l)
                (at-cruise-altitude ?vtx)
                (>= (altitude ?vtx) (target-altitude ?vtx))
                (swarm-comm-ready ?vtx)
                (forall (?a - drone)
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
        :effect
            (and
                (parabola-established)
                (formation-at ?l)
                (shape-held)
            )
    )
    ;; Move the vertex to the next location - the arms hold their slots
    ;; either side of it
    (:action formation-cruise
        :parameters
            (?vtx - drone
             ?from - location
             ?to - location)
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
        :effect
            (and
                (not (formation-at ?from))
                (formation-at ?to)
                (not (at ?vtx ?from))
                (at ?vtx ?to)
                ;; The arms must re-form the parabola before the next leg
                (not (shape-held))
                (decrease (battery ?vtx) (energy-required ?from ?to))
            )
    )
    ;; Bring both arms level with the vertex again. Only possible while
    ;; every link in the formation is still up - the arms hold their slots
    ;; from the positions they share with the vertex and with each other
    (:action hold-parabola-shape
        :parameters
            (?vtx - drone
             ?to - location)
        :precondition
            (and
                (parabola-established)
                (not (shape-held))
                (formation-leader ?vtx)
                (in-formation ?vtx)
                (formation-at ?to)
                (at ?vtx ?to)
                (forall (?d1 - drone)
                    (forall (?d2 - drone)
                        (or
                            (not (parabola-neighbor ?d1 ?d2))
                            (link-up ?d1 ?d2)
                        )
                    )
                )
                (forall (?a - drone)
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
        :effect
            (and
                (forall (?a - drone) (forall (?x - location)
                    (when (and (arm-drone ?a) (at ?a ?x))
                        (not (at ?a ?x)))))
                (forall (?a - drone)
                    (when (arm-drone ?a) (at ?a ?to)))
                (forall (?a - drone)
                    (when (arm-drone ?a)
                        (decrease (battery ?a) (cruise-leg-energy ?a))))
                (shape-held)
            )
    )
    ;; Complete the mission when the whole parabola reaches the destination
    (:action complete-parabola-mission
        :parameters
            (?vtx - drone
             ?dest - location)
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
                (forall (?a - drone)
                    (or
                        (not (arm-drone ?a))
                        (and
                            (in-formation ?a)
                            (at ?a ?dest)
                        )
                    )
                )
            )
        :effect
            (and
                (mission-completed ?vtx)
                (forall (?a - drone)
                    (when (arm-drone ?a)
                        (mission-completed ?a)))
                (parabola-mission-complete)
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
                (not (swarm-comm-ready ?d))
            )
    )
    ;; Detect loss of the drone-to-drone links - every link the drone has
    ;; drops with it, so the shape can't be re-formed until it is restored
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
                (not (swarm-comm-ready ?d))
                (forall (?n - drone)
                    (when (parabola-neighbor ?d ?n)
                        (and
                            (not (link-up ?d ?n))
                            (not (link-up ?n ?d))
                        )
                    )
                )
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
