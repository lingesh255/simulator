(define (problem forest-search-001)

    (:domain forest-drone-search)


    ;; ============================================================
    ;; OBJECTS
    ;; ============================================================

    ;; No search waypoints are declared here - how many a mission needs
    ;; depends on the marked search area's actual size relative to each
    ;; drone's coverage width, so they don't exist until a real area is
    ;; marked. engine/search_problem.retarget_search_problem generates a
    ;; full lawnmower sweep of location objects (named "drone1-wp1",
    ;; "drone1-wp2", ... / "drone2-wp1", ...) for the actual marked area and
    ;; splices them - and the connected/safe-route/distance/energy-required/
    ;; area-location facts wiring them up - into this file's :objects/:init
    ;; sections before ENHSP ever sees it. This template alone is therefore
    ;; NOT independently solvable (there is nowhere for either drone to
    ;; search yet) - it is only ever run after retargeting.

    (:objects

        ;; --------------------------------------------------------
        ;; Drones
        ;; --------------------------------------------------------

        drone1 - drone
        drone2 - drone

        ;; --------------------------------------------------------
        ;; Controller
        ;; --------------------------------------------------------

        ground-controller - controller

        ;; --------------------------------------------------------
        ;; Forest - one area per drone's lane, so "covered" is tracked
        ;; per-lane and the two drones can never be credited with covering
        ;; the same ground (see ACTION 7 in domain.pddl).
        ;; --------------------------------------------------------

        forest-lane1 - forest-area
        forest-lane2 - forest-area

        ;; --------------------------------------------------------
        ;; Base - the one fixed location; every search waypoint is
        ;; generated relative to it (see above).
        ;; --------------------------------------------------------

        base - location
    )


    ;; ============================================================
    ;; INITIAL STATE
    ;; ============================================================

    (:init

        ;; --------------------------------------------------------
        ;; Forest
        ;; --------------------------------------------------------

        (forest forest-lane1)
        (forest forest-lane2)


        ;; --------------------------------------------------------
        ;; Drone starting positions
        ;; --------------------------------------------------------

        (at drone1 base)
        (at drone2 base)

        (is-base base)


        ;; --------------------------------------------------------
        ;; Search strip assignments
        ;; --------------------------------------------------------

        ;; Drone 1 owns lane 1 (one side of the search area)
        (assigned drone1 forest-lane1)

        ;; Drone 2 owns lane 2 (the other side - disjoint from lane 1, so
        ;; the two drones never need to cover the same ground)
        (assigned drone2 forest-lane2)


        ;; --------------------------------------------------------
        ;; GPS / Communication / Health
        ;; --------------------------------------------------------

        (gps-ok drone1)
        (communication-ok drone1)
        (drone-healthy drone1)

        (gps-ok drone2)
        (communication-ok drone2)
        (drone-healthy drone2)


        ;; ========================================================
        ;; BATTERY
        ;; ========================================================

        (= (battery drone1) 100)
        (= (max-battery drone1) 100)

        (= (battery drone2) 100)
        (= (max-battery drone2) 100)


        ;; Minimum battery for emergency return
        (= (minimum-return-battery drone1) 30)
        (= (minimum-return-battery drone2) 30)


        ;; ========================================================
        ;; DRONE WIDTH / COVERAGE
        ;; ========================================================

        ;; Each drone's real sensor/coverage swath - this is what
        ;; engine/search_problem spaces sweep passes by, so the lawnmower
        ;; route it generates actually matches what these two facts claim.

        (= (drone-width drone1) 50)
        (= (drone-width drone2) 50)

        (= (coverage-width drone1) 50)
        (= (coverage-width drone2) 50)


        ;; ========================================================
        ;; MINIMUM SEPARATION
        ;; ========================================================

        ;; Drones must remain at least 10 m apart

        (= (minimum-drone-separation drone1 drone2) 10)
        (= (minimum-drone-separation drone2 drone1) 10)


        ;; ========================================================
        ;; HEALTH
        ;; ========================================================

        (= (health drone1) 100)
        (= (health drone2) 100)

        (= (minimum-health drone1) 40)
        (= (minimum-health drone2) 40)


        ;; ========================================================
        ;; HOVER ENERGY
        ;; ========================================================

        (= (hover-energy drone1) 2)
        (= (hover-energy drone2) 2)


        ;; ========================================================
        ;; INITIAL SEPARATION
        ;; ========================================================

        ;; Both drones start stacked at the same base point (distance 0),
        ;; which is not itself >= the minimum separation - real takeoff is
        ;; naturally staggered, so the very first hop out of base is let
        ;; through for free. Every hop after that (see move-search /
        ;; return-to-base in domain.pddl) consumes this and requires a
        ;; freshly-checked one in its place, using the real distances
        ;; generated for the marked area.

        (separation-ok drone1)
        (separation-ok drone2)


        ;; ========================================================
        ;; GEOGRAPHIC COORDINATES
        ;; ========================================================

        ;; Placeholder - overwritten with the real picked base point at
        ;; runtime (see engine/search_problem.retarget_search_problem).

        (= (latitude base) 13.0827)
        (= (longitude base) 80.2707)
    )


    ;; ============================================================
    ;; GOAL
    ;; ============================================================

    (:goal

        (and

            ;; Entire forest searched
            (forest-search-completed)

            ;; Both drones returned
            (mission-completed drone1)
            (mission-completed drone2)

            ;; Both drones are at base
            (at drone1 base)
            (at drone2 base)
        )
    )

)
