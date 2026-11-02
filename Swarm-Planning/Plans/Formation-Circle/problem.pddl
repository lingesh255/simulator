;; Formation-Circle: four drones on a turning vertical circle fly a spiral
;; from the source to the destination, each following its own formula, with
;; no communication between them. The mission program fills in this example
;; before planning: every pad's position from the picked source and
;; destination, the turn rate and the number of quarter turns for the
;; route's length. (See domain.pddl for the formula.)

(define (problem circle-formation-mission-001)
    (:domain circle-formation-drone-mission)

    ;; Drones and other objects in this example
    (:objects
        drone1 drone2 drone3 drone4 - drone
        ;; Each drone's own take-off pad (the formula at tau = 0)
        pad-src-1 pad-src-2 pad-src-3 pad-src-4 - pad
        ;; Each drone's own landing pad (the formula at tau = T)
        pad-dst-1 pad-dst-2 pad-dst-3 pad-dst-4 - pad
        ;; The four places on the circle
        bottom left top right - circle-place
    )

    ;; Starting facts and values
    (:init

        ;; ---- The formula's numbers ----------------------------------------

        ;; V: the circle moves 6 m along the route per second of the clock
        (= (formation-speed) 6.000000)

        ;; R: radius of the circle, in metres
        (= (radius) 10.000000)

        ;; Z0: height of the circle's centre, in metres
        (= (centre-altitude) 50.000000)

        ;; E: bottom drone's pad this far ahead, top drone's this far behind
        (= (pad-offset) 10.000000)

        ;; w: radians per second the circle turns
        ;;    (= 2 pi * full turns / spiral time)
        (= (turn-rate) 0.471239)

        ;; L: source to destination, in metres
        (= (route-length) 1000.000000)

        ;; Quarter turns the circle makes: 4 per full turn
        (= (quarter-turns-total) 44.000000)

        ;; The closest any two drones may ever get, in metres
        (= (min-separation) 10.000000)

        ;; The fastest a drone may be asked to fly, per second of the clock
        (= (max-speed) 10.000000)

        ;; The bottom of the circle must stay at least this high, in metres
        (= (lowest-altitude) 30.000000)

        ;; ---- The circle's places: angle a from the bottom, and height ------
        ;;   z = Z0 - R cos(a)

        ;; bottom: a = 0,      z = 50 - 10 cos(0)      = 40 m
        (= (place-angle bottom) 0)
        (= (place-altitude bottom) 40)
        ;; left:   a = 90 deg, z = 50 - 10 cos(90 deg)  = 50 m
        (= (place-angle left) 90)
        (= (place-altitude left) 50)
        ;; top:    a = 180 deg, z = 50 - 10 cos(180 deg) = 60 m
        (= (place-angle top) 180)
        (= (place-altitude top) 60)
        ;; right:  a = 270 deg, z = 50 - 10 cos(270 deg) = 50 m
        (= (place-angle right) 270)
        (= (place-altitude right) 50)

        ;; A quarter turn moves a drone on one place
        (next-place bottom left)
        (next-place left top)
        (next-place top right)
        (next-place right bottom)

        ;; ---- Drone 1: starts at the BOTTOM (a0 = 0) -------------------------
        ;;   y = 10 sin(w t),  z = 50 - 10 cos(w t)    -> 40 m, then left, top, right, ...
        ;;   pad: 10 m AHEAD of the circle (E cos 0 = +10)
        (at-pad drone1 pad-src-1)
        (landed drone1)
        (source-pad drone1 pad-src-1)
        (destination-pad drone1 pad-dst-1)
        (start-place drone1 bottom)
        (gps-ok drone1)
        (clock-synced drone1)
        (= (quarter-turns-flown drone1) 0)
        (= (altitude drone1) 0)
        (= (start-altitude drone1) 40)

        ;; ---- Drone 2: starts on the LEFT (a0 = pi/2) ------------------------
        ;;   y = 10 sin(w t + pi/2),  z = 50 - 10 cos(w t + pi/2)  -> 50 m, then top, right, ...
        ;;   pad: 10 m to the left of the route (E cos(pi/2) = 0 along it)
        (at-pad drone2 pad-src-2)
        (landed drone2)
        (source-pad drone2 pad-src-2)
        (destination-pad drone2 pad-dst-2)
        (start-place drone2 left)
        (gps-ok drone2)
        (clock-synced drone2)
        (= (quarter-turns-flown drone2) 0)
        (= (altitude drone2) 0)
        (= (start-altitude drone2) 50)

        ;; ---- Drone 3: starts at the TOP (a0 = pi) ---------------------------
        ;;   y = 10 sin(w t + pi),  z = 50 - 10 cos(w t + pi)    -> 60 m, then right, bottom, ...
        ;;   pad: 10 m BEHIND the circle (E cos(pi) = -10)
        (at-pad drone3 pad-src-3)
        (landed drone3)
        (source-pad drone3 pad-src-3)
        (destination-pad drone3 pad-dst-3)
        (start-place drone3 top)
        (gps-ok drone3)
        (clock-synced drone3)
        (= (quarter-turns-flown drone3) 0)
        (= (altitude drone3) 0)
        (= (start-altitude drone3) 60)

        ;; ---- Drone 4: starts on the RIGHT (a0 = 3pi/2) ----------------------
        ;;   y = 10 sin(w t + 3pi/2),  z = 50 - 10 cos(w t + 3pi/2) -> 50 m, then bottom, left, ...
        ;;   pad: 10 m to the right of the route
        (at-pad drone4 pad-src-4)
        (landed drone4)
        (source-pad drone4 pad-src-4)
        (destination-pad drone4 pad-dst-4)
        (start-place drone4 right)
        (gps-ok drone4)
        (clock-synced drone4)
        (= (quarter-turns-flown drone4) 0)
        (= (altitude drone4) 0)
        (= (start-altitude drone4) 50)

        ;; ---- The pads: each drone's own take-off and landing point ----------
        ;; Pad of drone i = (x = E cos(a_i0), y = R sin(a_i0)) around the picked
        ;; source / destination - a diamond, every two pads >= 14.14 m apart.

        (= (latitude pad-src-1) 17.4000898)
        (= (longitude pad-src-1) 78.3000000)
        (= (latitude pad-src-2) 17.4000000)
        (= (longitude pad-src-2) 78.2999059)
        (= (latitude pad-src-3) 17.3999102)
        (= (longitude pad-src-3) 78.3000000)
        (= (latitude pad-src-4) 17.4000000)
        (= (longitude pad-src-4) 78.3000941)

        (= (latitude pad-dst-1) 17.4090730)
        (= (longitude pad-dst-1) 78.3000000)
        (= (latitude pad-dst-2) 17.4089832)
        (= (longitude pad-dst-2) 78.2999059)
        (= (latitude pad-dst-3) 17.4088934)
        (= (longitude pad-dst-3) 78.3000000)
        (= (latitude pad-dst-4) 17.4089832)
        (= (longitude pad-dst-4) 78.3000941)
    )

    ;; Facts that must be true when the mission finishes
    (:goal
        (and
            ;; Every drone has landed on its own destination pad
            (mission-completed drone1)
            (mission-completed drone2)
            (mission-completed drone3)
            (mission-completed drone4)
        )
    )
)
