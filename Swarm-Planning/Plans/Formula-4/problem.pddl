;; Formula-4: four drones swap places all the way to the destination,
;; each following its own formula, with no communication between them.
;; The mission program fills in this example before planning: every slot's
;; position from the picked source and destination, the swap rate and the
;; number of swaps for the route's length. (See domain.pddl for the formula.)

(define (problem formula4-mission-001)
    (:domain formula4-drone-mission)

    ;; Drones and other objects in this example
    (:objects
        drone1 drone2 drone3 drone4 - drone
        ;; Each drone's own take-off point (the formula at tau = 0)
        src-1 src-2 src-3 src-4 - slot
        ;; Each drone's own landing point (the formula at tau = T)
        dst-1 dst-2 dst-3 dst-4 - slot
    )

    ;; Starting facts and values
    (:init

        ;; ---- The formula's numbers ----------------------------------------

        ;; V: the formation moves 6 m per second of the shared clock
        (= (formation-speed) 6.000000)

        ;; D: front line to back line, in metres
        (= (line-gap) 20.000000)

        ;; W: left lane to right lane, in metres
        (= (lane-gap) 20.000000)

        ;; B: a passing pair steps this far apart sideways, in metres
        (= (side-step) 6.000000)

        ;; Z0: height between swaps, in metres
        (= (mid-altitude) 55.000000)

        ;; A: a swap takes the slow drone down to Z0 - A = 50 m and the
        ;; passing drone up to Z0 + A = 60 m
        (= (altitude-swing) 5.000000)

        ;; w: radians of clock per second, = N * pi * V / L
        (= (swap-rate) 0.282743)

        ;; L: source to destination, in metres
        (= (route-length) 1000.000000)

        ;; N: swaps each pair flies (even, so drones 1 and 2 finish in front)
        (= (swaps-total) 10.000000)

        ;; The closest any two drones may ever get, in metres
        (= (min-separation) 10.000000)

        ;; ---- Drone 1: FRONT line, LEFT lane (f = +1, l = +1) ----------------
        ;;   x = V*tau + 10 cos(w tau)
        ;;   y = +(10 + 3 sin(w tau))
        ;;   z = 55 - 5 sin(w tau)      -> slows and dips to 50 m on swap 1
        (at drone1 src-1)
        (landed drone1)
        (front-line drone1)
        (left-lane drone1)
        (source-slot drone1 src-1)
        (destination-slot drone1 dst-1)
        (gps-ok drone1)
        (clock-synced drone1)
        (= (swaps-flown drone1) 0)
        (= (altitude drone1) 0)
        (= (swap-altitude drone1) 0)

        ;; ---- Drone 2: FRONT line, RIGHT lane (f = +1, l = -1) ---------------
        ;;   x = V*tau + 10 cos(w tau)
        ;;   y = -(10 + 3 sin(w tau))
        ;;   z = 55 - 5 sin(w tau)      -> slows and dips to 50 m on swap 1
        (at drone2 src-2)
        (landed drone2)
        (front-line drone2)
        (right-lane drone2)
        (source-slot drone2 src-2)
        (destination-slot drone2 dst-2)
        (gps-ok drone2)
        (clock-synced drone2)
        (= (swaps-flown drone2) 0)
        (= (altitude drone2) 0)
        (= (swap-altitude drone2) 0)

        ;; ---- Drone 3: BACK line, LEFT lane (f = -1, l = +1) -----------------
        ;;   x = V*tau - 10 cos(w tau)
        ;;   y = +(10 - 3 sin(w tau))
        ;;   z = 55 + 5 sin(w tau)      -> speeds up and climbs to 60 m on swap 1
        (at drone3 src-3)
        (landed drone3)
        (back-line drone3)
        (left-lane drone3)
        (source-slot drone3 src-3)
        (destination-slot drone3 dst-3)
        (gps-ok drone3)
        (clock-synced drone3)
        (= (swaps-flown drone3) 0)
        (= (altitude drone3) 0)
        (= (swap-altitude drone3) 0)

        ;; ---- Drone 4: BACK line, RIGHT lane (f = -1, l = -1) ----------------
        ;;   x = V*tau - 10 cos(w tau)
        ;;   y = -(10 - 3 sin(w tau))
        ;;   z = 55 + 5 sin(w tau)      -> speeds up and climbs to 60 m on swap 1
        (at drone4 src-4)
        (landed drone4)
        (back-line drone4)
        (right-lane drone4)
        (source-slot drone4 src-4)
        (destination-slot drone4 dst-4)
        (gps-ok drone4)
        (clock-synced drone4)
        (= (swaps-flown drone4) 0)
        (= (altitude drone4) 0)
        (= (swap-altitude drone4) 0)

        ;; ---- The slots: each drone's own source and destination point -------
        ;; Source slot of drone i = the formula at tau = 0 (x = 10 f_i, y = 10 l_i
        ;; around the picked source); destination slot = the formula at tau = T
        ;; (the same, around the picked destination). Height: Z0 = 55 m.

        (= (latitude src-1) 17.4000898)
        (= (longitude src-1) 78.3000000)
        (= (slot-altitude src-1) 55)
        (= (latitude src-2) 17.4000898)
        (= (longitude src-2) 78.3001883)
        (= (slot-altitude src-2) 55)
        (= (latitude src-3) 17.3999102)
        (= (longitude src-3) 78.3000000)
        (= (slot-altitude src-3) 55)
        (= (latitude src-4) 17.3999102)
        (= (longitude src-4) 78.3001883)
        (= (slot-altitude src-4) 55)

        (= (latitude dst-1) 17.4090730)
        (= (longitude dst-1) 78.3000000)
        (= (slot-altitude dst-1) 55)
        (= (latitude dst-2) 17.4090730)
        (= (longitude dst-2) 78.3001883)
        (= (slot-altitude dst-2) 55)
        (= (latitude dst-3) 17.4088934)
        (= (longitude dst-3) 78.3000000)
        (= (slot-altitude dst-3) 55)
        (= (latitude dst-4) 17.4088934)
        (= (longitude dst-4) 78.3001883)
        (= (slot-altitude dst-4) 55)
    )

    ;; Facts that must be true when the mission finishes
    (:goal
        (and
            ;; Every drone has landed on its own destination slot
            (mission-completed drone1)
            (mission-completed drone2)
            (mission-completed drone3)
            (mission-completed drone4)
            ;; An even number of swaps: drones 1 and 2 are in front again
            (front-line drone1)
            (front-line drone2)
            (back-line drone3)
            (back-line drone4)
        )
    )
)
