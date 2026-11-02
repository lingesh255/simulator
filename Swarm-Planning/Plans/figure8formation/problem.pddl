;; Drones fly the figure-eight pattern for the required laps and return.
;; The mission program fills in this example before planning.
;; Keep each starting fact on one line so added drones receive the full fact.

(define (problem figure8-formation-mission-001)
    (:domain figure8-formation-drone-mission)

    ;; Drones and other objects in this example
    (:objects
        drone-lead  - drone
        drone-f1    - drone
        ground-controller - controller
        base         - location
        crossing-in  - location
        lobe1-a      - location
        lobe1-end    - location
        lobe1-b      - location
        crossing-mid - location
        lobe2-a      - location
        lobe2-end    - location
        lobe2-b      - location
        crossing-out - location
    )

    ;; Starting facts and values
    (:init

        ;; Where the drone is
        (at drone-lead base)
        (at drone-f1 base)

        ;; The drone is on the ground
        (landed drone-lead)
        (landed drone-f1)

        ;; Current height of the drone
        (= (altitude drone-lead) 0)
        (= (altitude drone-f1) 0)

        ;; Where drones leave and return
        (base base)

        ;; Where a lap starts
        (lap-start crossing-in)

        ;; Where a lap ends
        (lap-end crossing-out)

        ;; Where the two loops cross
        (crossing-point crossing-in)
        (crossing-point crossing-mid)
        (crossing-point crossing-out)

        ;; A route between the base and the pattern
        (transit-route base crossing-in)
        (transit-route crossing-out base)

        ;; A route connects these locations
        (connected crossing-in lobe1-a)
        (connected lobe1-a lobe1-end)
        (connected lobe1-end lobe1-b)
        (connected lobe1-b crossing-mid)
        (connected crossing-mid lobe2-a)
        (connected lobe2-a lobe2-end)
        (connected lobe2-end lobe2-b)
        (connected lobe2-b crossing-out)

        ;; This route is allowed
        (safe-route crossing-in lobe1-a)
        (safe-route lobe1-a lobe1-end)
        (safe-route lobe1-end lobe1-b)
        (safe-route lobe1-b crossing-mid)
        (safe-route crossing-mid lobe2-a)
        (safe-route lobe2-a lobe2-end)
        (safe-route lobe2-end lobe2-b)
        (safe-route lobe2-b crossing-out)

        ;; The drone leads the train
        (train-leader drone-lead)

        ;; The drone follows in the train
        (train-follower drone-f1)

        ;; The drone has an assigned formation position
        (has-slot drone-lead)
        (has-slot drone-f1)

        ;; Distance along the path relative to the leader
        (= (slot-arc-offset drone-lead) 0)
        (= (slot-arc-offset drone-f1) -10)

        ;; Two drones are neighbours in the train
        (train-neighbor drone-lead drone-f1)
        (train-neighbor drone-f1 drone-lead)

        ;; The model treats the required neighbour links as working
        (neighbor-comm-ok drone-lead)
        (neighbor-comm-ok drone-f1)

        ;; Required spacing along the pattern
        (= (drone-spacing) 304.86)

        ;; Gap along the path between drones sharing the crossing
        (= (crossing-gap) 152.43)

        ;; Required gap at the crossing
        (= (crossing-clearance) 14.142136)

        ;; Number of laps to fly
        (= (laps-required) 1)

        ;; Number of laps already counted as complete
        (= (laps-completed) 0)

        ;; The drone has a GPS fix
        (gps-ok drone-lead)

        ;; The model treats the ground-controller link as working
        (communication-ok drone-lead)

        ;; The drone passes the health check
        (drone-healthy drone-lead)

        ;; The drone has a GPS fix
        (gps-ok drone-f1)

        ;; The model treats the ground-controller link as working
        (communication-ok drone-f1)

        ;; The drone passes the health check
        (drone-healthy drone-f1)

        ;; Current health score
        (= (health drone-lead) 100)
        (= (health drone-f1) 100)

        ;; Lowest allowed health score
        (= (minimum-health drone-lead) 40)
        (= (minimum-health drone-f1) 40)

        ;; Remaining battery in planning units
        (= (battery drone-lead) 100)
        (= (battery drone-f1) 100)

        ;; Full battery in planning units
        (= (max-battery drone-lead) 100)
        (= (max-battery drone-f1) 100)

        ;; Battery reserve for returning
        (= (minimum-return-battery drone-lead) 30)
        (= (minimum-return-battery drone-f1) 30)

        ;; Height just after takeoff
        (= (hover-altitude drone-lead) 2)
        (= (hover-altitude drone-f1) 2)

        ;; Assigned cruise height
        (= (target-altitude drone-lead) 50)
        (= (target-altitude drone-f1) 50)

        ;; Battery used to reach cruise height
        (= (climb-energy drone-lead) 4)
        (= (climb-energy drone-f1) 4)

        ;; Battery used by a member for one route leg
        (= (cruise-leg-energy drone-lead) 4)
        (= (cruise-leg-energy drone-f1) 4)

        ;; North-south map coordinate
        (= (latitude base)          18.3888072)

        ;; East-west map coordinate
        (= (longitude base)         79.0495640)

        ;; North-south map coordinate
        (= (latitude crossing-in)   18.3901547)

        ;; East-west map coordinate
        (= (longitude crossing-in)  79.0495640)

        ;; North-south map coordinate
        (= (latitude lobe1-a)       18.3906039)

        ;; East-west map coordinate
        (= (longitude lobe1-a)      79.0502334)

        ;; North-south map coordinate
        (= (latitude lobe1-end)     18.3901547)

        ;; East-west map coordinate
        (= (longitude lobe1-end)    79.0505107)

        ;; North-south map coordinate
        (= (latitude lobe1-b)       18.3897055)

        ;; East-west map coordinate
        (= (longitude lobe1-b)      79.0502334)

        ;; North-south map coordinate
        (= (latitude crossing-mid)  18.3901547)

        ;; East-west map coordinate
        (= (longitude crossing-mid) 79.0495640)

        ;; North-south map coordinate
        (= (latitude lobe2-a)       18.3906039)

        ;; East-west map coordinate
        (= (longitude lobe2-a)      79.0488946)

        ;; North-south map coordinate
        (= (latitude lobe2-end)     18.3901547)

        ;; East-west map coordinate
        (= (longitude lobe2-end)    79.0486173)

        ;; North-south map coordinate
        (= (latitude lobe2-b)       18.3897055)

        ;; East-west map coordinate
        (= (longitude lobe2-b)      79.0488946)

        ;; North-south map coordinate
        (= (latitude crossing-out)  18.3901547)

        ;; East-west map coordinate
        (= (longitude crossing-out) 79.0495640)

        ;; Distance along this route leg
        (= (distance base crossing-in) 152.16)
        (= (distance crossing-in lobe1-a) 88.33)
        (= (distance lobe1-a lobe1-end) 64.10)
        (= (distance lobe1-end lobe1-b) 64.10)
        (= (distance lobe1-b crossing-mid) 88.33)
        (= (distance crossing-mid lobe2-a) 88.33)
        (= (distance lobe2-a lobe2-end) 64.10)
        (= (distance lobe2-end lobe2-b) 64.10)
        (= (distance lobe2-b crossing-out) 88.33)
        (= (distance crossing-out base) 198.21)

        ;; Battery needed for this route leg
        (= (energy-required base crossing-in) 6.15)
        (= (energy-required crossing-in lobe1-a) 3.57)
        (= (energy-required lobe1-a lobe1-end) 2.59)
        (= (energy-required lobe1-end lobe1-b) 2.59)
        (= (energy-required lobe1-b crossing-mid) 3.57)
        (= (energy-required crossing-mid lobe2-a) 3.57)
        (= (energy-required lobe2-a lobe2-end) 2.59)
        (= (energy-required lobe2-end lobe2-b) 2.59)
        (= (energy-required lobe2-b crossing-out) 3.57)
        (= (energy-required crossing-out base) 8.01)
    )

    ;; Facts that must be true when the mission finishes
    (:goal
        (and

            ;; The figure-eight formation has been set up
            (figure8-established)

            ;; The drone has joined the formation
            (in-formation drone-lead)
            (in-formation drone-f1)

            ;; The drone has reached its assigned height
            (at-cruise-altitude drone-lead)
            (at-cruise-altitude drone-f1)
            (>= (altitude drone-lead) 50)
            (>= (altitude drone-f1) 50)

            ;; The required laps have been finished
            (laps-finished)

            ;; Where the formation is
            (formation-at base)

            ;; Where the drone is
            (at drone-lead base)
            (at drone-f1 base)

            ;; The figure-eight mission is complete
            (figure8-mission-complete)

            ;; The drone has finished its mission
            (mission-completed drone-lead)
            (mission-completed drone-f1)
        )
    )
)
