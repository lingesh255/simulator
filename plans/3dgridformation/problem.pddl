(define (problem cube-grid-formation-mission-001)

    (:domain cube-grid-formation-drone-mission)

    ;; This file is a TEMPLATE, not the mission that flies. It describes the
    ;; smallest possible case - the leader plus one member beside it, a
    ;; single-layer 1x1x2 "cube" - and `engine.pddl_problem.expand_cube_drones`
    ;; rebuilds it into whatever layers x rows x columns block the checked
    ;; drone count calls for, then `retarget_cube_problem` relocates it onto
    ;; the picked source/destination.
    ;;
    ;; FORMATTING RULE: keep every `(= (fluent object) value)` fact on ONE
    ;; LINE. The expansion machinery clones facts line by line
    ;; (`_clone_named_lines`), so a fact split across several lines is cloned
    ;; into a valueless fragment and the planner then reports the problem
    ;; unsolvable - exactly the defect that broke plans/gridformation/ on
    ;; 2026-09-16.

    ;; Objects used in the mission
    (:objects

        drone-lead   - drone
        drone-member - drone

        ground-controller - controller

        source      - location
        destination - location

        ;; One object per horizontal layer, bottom first. The template is a
        ;; single layer; expand_cube_drones widens this to layer0..layerN-1.
        layer0 - layer
    )

    ;; Initial state
    (:init

        ;; Both drones start at the source
        (at drone-lead   source)
        (at drone-member source)

        (landed drone-lead)
        (landed drone-member)

        (= (altitude drone-lead)   0)
        (= (altitude drone-member) 0)

        (source source)
        (destination destination)


        ;; Direct route between source and destination
        (connected source destination)
        (connected destination source)

        (safe-route source destination)
        (safe-route destination source)


        ;; Assign formation leader and cube member
        (formation-leader drone-lead)

        (cube-member drone-member)

        (has-slot drone-lead)
        (has-slot drone-member)

        ;; Cube slot positions - horizontal pair, then the vertical pair.
        ;; The template is a single layer, so both drones sit at layer 0 with
        ;; no lift above the bottom layer's cruise altitude.
        (= (slot-along-offset drone-lead)     0)
        (= (slot-cross-offset drone-lead)     0)
        (= (slot-layer drone-lead)            0)
        (= (slot-lift-offset drone-lead)      0)

        (= (slot-along-offset drone-member)   0)
        (= (slot-cross-offset drone-member)  10)
        (= (slot-layer drone-member)          0)
        (= (slot-lift-offset drone-member)    0)


        ;; Neighbor communication. For this single-layer template the 24-way
        ;; relation collapses to one symmetric pair; `expand_cube_drones`
        ;; strips these and rebuilds the full 24-connected mesh for the real
        ;; block.
        (cube-neighbor drone-lead drone-member)
        (cube-neighbor drone-member drone-lead)

        (neighbor-comm-ok drone-lead) ;; communications are OK at the start of the mission
        (neighbor-comm-ok drone-member);; communication with 24 neighbors.


        ;; Layer-by-layer ascent. Both template drones sit in the bottom
        ;; layer, which starts already launched - so they are cleared to take
        ;; off immediately. A single-layer template has nothing above layer 0,
        ;; so there are no `layer-below` facts and `open-next-layer` never
        ;; fires; expand_cube_drones strips and rebuilds all four fact kinds
        ;; for the real block.
        (in-layer drone-lead   layer0)
        (in-layer drone-member layer0)

        (layer-launched layer0)

        (cleared-to-launch drone-lead)
        (cleared-to-launch drone-member)

        ;; Staggered launch settings
        (= (seconds-since-leader-airborne) 0)
        (= (cube-launch-delay) 5)


        ;; System health starts normal
        (gps-ok drone-lead)
        (communication-ok drone-lead)
        (drone-healthy drone-lead)

        (gps-ok drone-member)
        (communication-ok drone-member)
        (drone-healthy drone-member)

        (= (health drone-lead)    100)
        (= (health drone-member)  100)

        (= (minimum-health drone-lead)    40)
        (= (minimum-health drone-member)  40)


        ;; Initial battery
        (= (battery drone-lead)    100)
        (= (battery drone-member)  100)

        (= (max-battery drone-lead)    100)
        (= (max-battery drone-member)  100)

        (= (minimum-return-battery drone-lead)    30)
        (= (minimum-return-battery drone-member)  30)


        ;; Altitude settings. `target-altitude` is per drone because it is
        ;; per LAYER - expand_cube_drones rewrites it as
        ;; base + layer * layer-spacing for every cell.
        (= (hover-altitude drone-lead)    2)
        (= (hover-altitude drone-member)  2)

        (= (target-altitude drone-lead)    50)
        (= (target-altitude drone-member)  50)

        ;; Battery used during climb
        (= (climb-energy drone-lead)    4)
        (= (climb-energy drone-member)  4)


        ;; Battery used for each cruise leg
        (= (cruise-leg-energy drone-lead)    12)
        (= (cruise-leg-energy drone-member)  14)


        ;; Source and destination coordinates
        (= (latitude source)       18.3901547)
        (= (longitude source)      79.0495640)

        (= (latitude destination)  18.3876026)
        (= (longitude destination) 79.0816646)


        ;; Direct route distance and energy cost
        (= (distance source destination) 3399.03)
        (= (distance destination source) 3399.03)

        (= (energy-required source destination) 10.00)
        (= (energy-required destination source) 10.00)
    )


    ;; Mission goal
    (:goal

        (and

            ;; Cube formation is established
            (cube-formation-established)
            (in-formation drone-lead)
            (in-formation drone-member)

            ;; Both drones reached their own layer's cruise altitude
            (at-cruise-altitude drone-lead)
            (at-cruise-altitude drone-member)

            ;; Formation reaches the destination
            (formation-at destination)
            (at drone-lead   destination)
            (at drone-member destination)

            ;; Mission is completed
            (formation-mission-complete)
            (mission-completed drone-lead)
            (mission-completed drone-member)
        )
    )
)
