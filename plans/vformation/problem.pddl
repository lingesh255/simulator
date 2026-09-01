(define (problem v-formation-mission-001)

    (:domain v-formation-drone-mission)

    ;; ============================================================
    ;; Three drones - drone-lead (apex), drone-left and drone-right
    ;; (the two wings) - launch together from `source`, climb to
    ;; 50 m, lock into a V and cruise direct, source straight to
    ;; destination, as one formation. Just the two endpoints - no
    ;; waypoint and no restricted-area handling for this plan.
    ;; ============================================================

    ;; ============================================================
    ;; OBJECTS
    ;; ============================================================

    (:objects

        drone-lead  - drone
        drone-left  - drone
        drone-right - drone

        ground-controller - controller

        source      - location
        destination - location

        ;; The 8 compass points a wing's `slot-direction` can be. Which
        ;; two the wings actually get is computed from the real
        ;; source->destination bearing - see
        ;; engine.vformation_problem.retarget_vformation_problem.
        north      - direction
        north-east - direction
        east       - direction
        south-east - direction
        south      - direction
        south-west - direction
        west       - direction
        north-west - direction
    )


    ;; ============================================================
    ;; INITIAL STATE
    ;; ============================================================

    (:init

        ;; --------------------------------------------------------
        ;; Everyone starts landed, stacked at the source pad
        ;; --------------------------------------------------------

        (at drone-lead  source)
        (at drone-left  source)
        (at drone-right source)

        (landed drone-lead)
        (landed drone-left)
        (landed drone-right)

        (= (altitude drone-lead)  0)
        (= (altitude drone-left)  0)
        (= (altitude drone-right) 0)

        (source source)
        (destination destination)


        ;; --------------------------------------------------------
        ;; Route: source <-> destination direct, both ways. No
        ;; waypoint and no restricted area for this plan - just the
        ;; two endpoints.
        ;; --------------------------------------------------------

        (connected source destination)
        (connected destination source)

        (safe-route source destination)
        (safe-route destination source)


        ;; --------------------------------------------------------
        ;; Formation slot assignment - the V geometry
        ;;
        ;;              drone-lead              (apex, 0 / 0)
        ;;             /          \
        ;;      drone-left      drone-right     (wings, ~17.32 m behind,
        ;;                                       10 m either side)
        ;;
        ;; Along/cross offsets of +-10 m cross-track and ~17.32 m
        ;; along-track put the leader and both wings at the corners of
        ;; an equilateral triangle with 20 m sides - every drone is
        ;; exactly 20 m from both of the other two.
        ;;
        ;; `slot-direction` below is the real compass point that
        ;; offset actually points to for *this* source->destination
        ;; bearing (east, here) - retargeting to a different bearing
        ;; recomputes it; the along/cross offsets themselves don't
        ;; change; see engine.vformation_problem.
        ;; --------------------------------------------------------

        (formation-leader drone-lead)

        (slot-apex  drone-lead)
        (slot-direction drone-left  north-west)
        (slot-direction drone-right south-west)

        (has-slot drone-lead)
        (has-slot drone-left)
        (has-slot drone-right)

        (= (slot-along-offset drone-lead)    0)
        (= (slot-cross-offset drone-lead)    0)
        (= (slot-along-offset drone-left)   -17.320508)
        (= (slot-cross-offset drone-left)   -10)
        (= (slot-along-offset drone-right)  -17.320508)
        (= (slot-cross-offset drone-right)   10)


        ;; --------------------------------------------------------
        ;; Staggered launch: the leader takes off alone and holds
        ;; position (`hold-position`) for this many seconds before the
        ;; wings' own `takeoff` unlocks - see domain.pddl ACTION 2 /
        ;; ACTION 3b.
        ;; --------------------------------------------------------

        (= (seconds-since-leader-airborne) 0)
        (= (wing-launch-delay) 5)


        ;; --------------------------------------------------------
        ;; System health - all nominal
        ;; --------------------------------------------------------

        (gps-ok drone-lead)   (communication-ok drone-lead)   (drone-healthy drone-lead)
        (gps-ok drone-left)   (communication-ok drone-left)   (drone-healthy drone-left)
        (gps-ok drone-right)  (communication-ok drone-right)  (drone-healthy drone-right)

        (= (health drone-lead)  100)
        (= (health drone-left)  100)
        (= (health drone-right) 100)

        (= (minimum-health drone-lead)  40)
        (= (minimum-health drone-left)  40)
        (= (minimum-health drone-right) 40)


        ;; --------------------------------------------------------
        ;; Battery
        ;; --------------------------------------------------------

        (= (battery drone-lead)  100)
        (= (battery drone-left)  100)
        (= (battery drone-right) 100)

        (= (max-battery drone-lead)  100)
        (= (max-battery drone-left)  100)
        (= (max-battery drone-right) 100)

        (= (minimum-return-battery drone-lead)  30)
        (= (minimum-return-battery drone-left)  30)
        (= (minimum-return-battery drone-right) 30)


        ;; --------------------------------------------------------
        ;; Altitude profile: rest at 2 m after takeoff, then climb
        ;; to the 50 m cruise altitude (4% battery for the climb).
        ;; --------------------------------------------------------

        (= (hover-altitude drone-lead)  2)
        (= (hover-altitude drone-left)  2)
        (= (hover-altitude drone-right) 2)

        (= (target-altitude drone-lead)  50)
        (= (target-altitude drone-left)  50)
        (= (target-altitude drone-right) 50)

        (= (climb-energy drone-lead)  4)
        (= (climb-energy drone-left)  4)
        (= (climb-energy drone-right) 4)


        ;; Per-leg cruise burn: the apex flies the short inside line,
        ;; the two wings a little more as they sweep the outside.
        (= (cruise-leg-energy drone-lead)  12)
        (= (cruise-leg-energy drone-left)  14)
        (= (cruise-leg-energy drone-right) 14)


        ;; --------------------------------------------------------
        ;; Geographic coordinates
        ;; --------------------------------------------------------

        (= (latitude source)       18.3901547)
        (= (longitude source)      79.0495640)

        (= (latitude destination)  18.3876026)
        (= (longitude destination) 79.0816646)


        ;; --------------------------------------------------------
        ;; Direct leg distance (m) and energy cost (% battery)
        ;; --------------------------------------------------------

        (= (distance source destination) 3399.03)
        (= (distance destination source) 3399.03)

        (= (energy-required source destination) 35.88)
        (= (energy-required destination source) 35.88)
    )


    ;; ============================================================
    ;; GOAL
    ;; ============================================================

    (:goal

        (and

            ;; V-shape actually formed and held
            (v-formation-established)
            (in-formation drone-lead)
            (in-formation drone-left)
            (in-formation drone-right)

            ;; Cruise altitude of 50 m reached and maintained
            (at-cruise-altitude drone-lead)
            (at-cruise-altitude drone-left)
            (at-cruise-altitude drone-right)
            (>= (altitude drone-lead)  50)
            (>= (altitude drone-left)  50)
            (>= (altitude drone-right) 50)

            ;; Formation flew source -> destination
            (formation-at destination)
            (at drone-lead  destination)
            (at drone-left  destination)
            (at drone-right destination)

            ;; Mission closed out
            (formation-mission-complete)
            (mission-completed drone-lead)
            (mission-completed drone-left)
            (mission-completed drone-right)
        )
    )
)
