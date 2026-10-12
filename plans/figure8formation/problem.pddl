(define (problem figure8-formation-mission-001)

    (:domain figure8-formation-drone-mission)

    ;; Objects used in the figure-8 mission
    (:objects

        drone-lead  - drone
        drone-f1    - drone
        ground-controller - controller

        ;; The base the drones fly from and back to
        base         - location

        ;; Key waypoints of the 8, in flying order: out of the middle into
        ;; the first lobe, round it, back through the middle, round the
        ;; second lobe and back to the middle (crossing-in and crossing-out
        ;; are both the middle point - the start and end of one lap)
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


    (:init

        ;; All drones start landed at the base
        (at drone-lead base)
        (at drone-f1 base)

        (landed drone-lead)
        (landed drone-f1)

        (= (altitude drone-lead) 0)
        (= (altitude drone-f1) 0)

        (base base)
        (lap-start crossing-in)
        (lap-end crossing-out)

        (crossing-point crossing-in)
        (crossing-point crossing-mid)
        (crossing-point crossing-out)


        ;; Legs between the base and the middle of the 8
        (transit-route base crossing-in)
        (transit-route crossing-out base)


        ;; One-way chain of waypoints round the 8
        (connected crossing-in lobe1-a)
        (connected lobe1-a lobe1-end)
        (connected lobe1-end lobe1-b)
        (connected lobe1-b crossing-mid)
        (connected crossing-mid lobe2-a)
        (connected lobe2-a lobe2-end)
        (connected lobe2-end lobe2-b)
        (connected lobe2-b crossing-out)

        (safe-route crossing-in lobe1-a)
        (safe-route lobe1-a lobe1-end)
        (safe-route lobe1-end lobe1-b)
        (safe-route lobe1-b crossing-mid)
        (safe-route crossing-mid lobe2-a)
        (safe-route lobe2-a lobe2-end)
        (safe-route lobe2-end lobe2-b)
        (safe-route lobe2-b crossing-out)


        ;; Train slots: the leader in front, followers one behind another
        (train-leader drone-lead)
        (train-follower drone-f1)

        (has-slot drone-lead)
        (has-slot drone-f1)

        ;; Metres along the path behind the leader
        (= (slot-arc-offset drone-lead) 0)
        (= (slot-arc-offset drone-f1) -10)


        ;; Neighbor communication - each drone talks to the drone directly
        ;; in front of it and directly behind it (here, just the one pair)
        (train-neighbor drone-lead drone-f1)
        (train-neighbor drone-f1 drone-lead)

        (neighbor-comm-ok drone-lead)
        (neighbor-comm-ok drone-f1)


        ;; Train geometry - the train (front to back) plus the crossing
        ;; clearance must fit inside the smaller lobe
        (= (drone-spacing) 10)
        (= (train-span) 10)
        (= (min-lobe-length) 304.86)
        (= (crossing-clearance) 14.142136)

        ;; Laps of the 8 to fly (set from the operator's input at run time)
        (= (laps-required) 1)
        (= (laps-completed) 0)


        ;; All drones start with normal health and system status
        (gps-ok drone-lead)
        (communication-ok drone-lead)
        (drone-healthy drone-lead)

        (gps-ok drone-f1)
        (communication-ok drone-f1)
        (drone-healthy drone-f1)

        (= (health drone-lead) 100)
        (= (health drone-f1) 100)

        (= (minimum-health drone-lead) 40)
        (= (minimum-health drone-f1) 40)


        ;; Initial battery
        (= (battery drone-lead) 100)
        (= (battery drone-f1) 100)

        (= (max-battery drone-lead) 100)
        (= (max-battery drone-f1) 100)

        ;; Battery level for emergency return
        (= (minimum-return-battery drone-lead) 30)
        (= (minimum-return-battery drone-f1) 30)


        ;; Drones start at 2 m and climb to 50 m
        (= (hover-altitude drone-lead) 2)
        (= (hover-altitude drone-f1) 2)

        (= (target-altitude drone-lead) 50)
        (= (target-altitude drone-f1) 50)

        ;; Battery used during the climb
        (= (climb-energy drone-lead) 4)
        (= (climb-energy drone-f1) 4)


        ;; Battery used by each follower for one leg
        (= (cruise-leg-energy drone-lead) 4)
        (= (cruise-leg-energy drone-f1) 4)


        ;; Coordinates (a 200 m-wide 8, the base 150 m below its middle)
        (= (latitude base)          18.3888072)
        (= (longitude base)         79.0495640)

        (= (latitude crossing-in)   18.3901547)
        (= (longitude crossing-in)  79.0495640)

        (= (latitude lobe1-a)       18.3906039)
        (= (longitude lobe1-a)      79.0502334)

        (= (latitude lobe1-end)     18.3901547)
        (= (longitude lobe1-end)    79.0505107)

        (= (latitude lobe1-b)       18.3897055)
        (= (longitude lobe1-b)      79.0502334)

        (= (latitude crossing-mid)  18.3901547)
        (= (longitude crossing-mid) 79.0495640)

        (= (latitude lobe2-a)       18.3906039)
        (= (longitude lobe2-a)      79.0488946)

        (= (latitude lobe2-end)     18.3901547)
        (= (longitude lobe2-end)    79.0486173)

        (= (latitude lobe2-b)       18.3897055)
        (= (longitude lobe2-b)      79.0488946)

        (= (latitude crossing-out)  18.3901547)
        (= (longitude crossing-out) 79.0495640)


        ;; Leg lengths along the path, and their energy cost
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


    (:goal

        (and

            ;; Figure-8 train is established
            (figure8-established)
            (in-formation drone-lead)
            (in-formation drone-f1)

            ;; All drones reached 50 m cruise altitude
            (at-cruise-altitude drone-lead)
            (at-cruise-altitude drone-f1)

            (>= (altitude drone-lead) 50)
            (>= (altitude drone-f1) 50)

            ;; Every lap flown, and the train back at the base
            (laps-finished)
            (formation-at base)
            (at drone-lead base)
            (at drone-f1 base)

            ;; Mission is completed
            (figure8-mission-complete)
            (mission-completed drone-lead)
            (mission-completed drone-f1)
        )
    )
)
