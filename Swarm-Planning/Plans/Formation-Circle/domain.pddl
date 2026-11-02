;; Formation-Circle: four drones fly from the source to the destination on a
;; vertical circle that keeps TURNING as it goes, so every drone flies a
;; spiral (a helix) round the route - WITHOUT the drones talking to each
;; other. Each drone follows its own mathematical formula of one shared
;; clock (GPS time), and the formula itself keeps every two drones more
;; than 10 m apart.
;;
;; ---------------------------------------------------------------------------
;; THE FORMATION (seen from behind - the drones fly into the page)
;;
;;                       top: drone 3 (60 m)
;;                             .  -  .
;;                          '           '
;;     left: drone 2 (50 m) .     + (50 m)  . right: drone 4 (50 m)
;;                          .           .
;;                             '  -  '
;;                      bottom: drone 1 (40 m)
;;
;;   A circle of radius R = 10 m, centre Z0 = 50 m above the ground, standing
;;   straight up across the route. The drones are a quarter of the circle
;;   (90 degrees) apart. As the circle flies along it turns:
;;
;;       bottom -> left -> top -> right -> bottom -> ...
;;
;;   every quarter turn, every drone moves on one place: drone 1 from the
;;   bottom to the left, drone 2 from the left to the top, drone 3 from the
;;   top to the right, drone 4 from the right to the bottom.
;;
;; ---------------------------------------------------------------------------
;; THE FORMULA - one for every drone, only its starting angle differs
;;
;;   Coordinates in metres: x = along the route (0 at the source),
;;   y = across the route (left is +), z = height above the ground.
;;   tau = the shared clock (seconds of the formation's own time).
;;   a = the drone's angle round the circle, measured from the bottom:
;;       a = 0 bottom, a = pi/2 left, a = pi top, a = 3pi/2 right.
;;
;;     a_i(tau) = a_i0 + w * (tau - Tj)
;;     x_i(tau) = V * tau
;;     y_i(tau) = R * sin(a_i(tau))
;;     z_i(tau) = Z0 - R * cos(a_i(tau))
;;
;;   V  = formation-speed   how fast the circle moves along the route (6 m/s)
;;   R  = radius            of the circle                          (10 m)
;;   Z0 = centre-altitude   height of the circle's centre           (50 m)
;;   w  = turn-rate         radians per second the circle turns; it turns a
;;                          whole number of full turns, about one every 80 m
;;   Tj = the time the circle takes to form after take-off (see THE ENDS)
;;
;;   Written out for each drone:
;;     drone 1 (a0 = 0):     y = 10 sin(w t),         z = 50 - 10 cos(w t)       starts bottom, 40 m
;;     drone 2 (a0 = pi/2):  y = 10 sin(w t + pi/2),  z = 50 - 10 cos(w t + pi/2) starts left,   50 m
;;     drone 3 (a0 = pi):    y = 10 sin(w t + pi),    z = 50 - 10 cos(w t + pi)   starts top,    60 m
;;     drone 4 (a0 = 3pi/2): y = 10 sin(w t + 3pi/2), z = 50 - 10 cos(w t + 3pi/2) starts right, 50 m
;;     (t = tau - Tj, and x = V tau for all four)
;;
;;   Check drone 1: t = 0 -> y = 0, z = 40 (bottom). A quarter turn later
;;   (w t = pi/2) -> y = +10, z = 50 (left). Half a turn (w t = pi) -> y = 0,
;;   z = 60 (top). Three quarters -> y = -10, z = 50 (right). Then bottom.
;;   Because x grows all the time, that path round the circle is stretched
;;   along the route into a SPIRAL.
;;
;;   Speed of a drone: sqrt(V^2 + (R w)^2) - the same for all four, always.
;;
;; WHY THEY NEVER COLLIDE (at every moment, straight from the formula)
;;
;;   All four have the same x, and the same w: the circle turns rigidly, so
;;   the angle between two drones never changes.
;;     neighbours (1 & 2, 2 & 3, 3 & 4, 4 & 1): 90 degrees apart
;;         distance = 2 R sin(45 deg) = R * sqrt(2) = 14.14 m
;;     opposite (1 & 3, 2 & 4): 180 degrees apart
;;         distance = 2 R = 20 m
;;   So every two drones are always at least 14.14 m apart (> 10 m).
;;   The check-formula action below proves this with the real numbers
;;   before anything takes off.
;;
;; WHY NO COMMUNICATION IS NEEDED
;;
;;   The formula only needs tau and the drone's own starting angle a_i0. No
;;   drone ever needs to know where another drone is - so every action below
;;   (except the one-off check-formula) takes exactly ONE drone, and only
;;   reads and changes that drone's own facts.
;;
;; THE ENDS (take-off and landing)
;;
;;   Drones 1 and 3 (bottom and top) are above the same ground point, so they
;;   can't take off there. Each drone's PAD is E * cos(a_i0) along the route
;;   from its place on the circle (E = pad-offset = 10 m): drone 1's pad 10 m
;;   ahead, drone 3's 10 m behind, drones 2 and 4 on their own side - a
;;   diamond, every two pads >= 14.14 m apart. Each drone climbs straight up
;;   on its pad to its own height (40 / 50 / 60 / 50 m), then the diamond
;;   closes into the circle while flying (no turning yet), with
;;       x_i(tau) = V tau + E cos(a_i0) (1 - b(tau / Tj)),  b(u) = (1 - cos(pi u)) / 2
;;   and at the destination it opens out the same way and every drone lands
;;   straight down on its own destination pad. Meanwhile drones 1 and 3 are
;;   20 m apart in height, and every other pair >= sqrt(R^2 + R^2) = 14.14 m.
;;   The circle makes WHOLE turns, so every drone is back at its own starting
;;   place on the circle when it leaves it.
;; ---------------------------------------------------------------------------

(define (domain circle-formation-drone-mission)

    ;; Planning features used by these actions
    (:requirements
        :strips ;; Use actions with conditions and effects
        :typing ;; Give objects types such as drone, pad and circle place
        :numeric-fluents ;; Use numbers such as speed, radius and height
        :negative-preconditions ;; Allow a condition to require that a fact is false
    )

    ;; Types of objects used in this mission
    (:types
        drone
        pad
        circle-place
    )

    ;; Facts that can be true or false
    (:predicates
        (at-pad ?d - drone ?p - pad) ;; The drone is on / straight above this pad
        (source-pad ?d - drone ?p - pad) ;; Where this drone takes off: the formula at tau = 0
        (destination-pad ?d - drone ?p - pad) ;; Where this drone lands: the formula at tau = T
        (start-place ?d - drone ?c - circle-place) ;; The drone's place on the circle at the start (its a_i0)
        (in-place ?d - drone ?c - circle-place) ;; The drone's place on the circle now
        (next-place ?from - circle-place ?to - circle-place) ;; A quarter turn moves a drone from ?from to ?to
        (landed ?d - drone) ;; The drone is on the ground
        (airborne ?d - drone) ;; The drone has taken off
        (in-circle ?d - drone) ;; The drone is flying its spiral on the circle
        (gps-ok ?d - drone) ;; The drone has a GPS fix - its position, and the shared clock
        (clock-synced ?d - drone) ;; The drone's own clock is set to GPS time (tau)
        (formula-safe) ;; The formula's numbers keep every two drones >= min-separation apart
        (mission-completed ?d - drone) ;; The drone has landed on its destination pad
    )

    ;; Numbers used by the planner
    (:functions
        (formation-speed) ;; V
        (radius) ;; R
        (centre-altitude) ;; Z0
        (pad-offset) ;; E
        (turn-rate) ;; w
        (route-length) ;; L, source to destination
        (quarter-turns-total) ;; Quarter turns the circle makes (4 per full turn)
        (min-separation) ;; Closest any two drones may ever get
        (max-speed) ;; Fastest a drone may be asked to fly, per second of the clock
        (lowest-altitude) ;; The bottom of the circle must stay at least this high
        (quarter-turns-flown ?d - drone) ;; Quarter turns this drone has flown so far
        (altitude ?d - drone) ;; The drone's height now
        (start-altitude ?d - drone) ;; Its height at its starting place: Z0 - R cos(a_i0)
        (place-angle ?c - circle-place) ;; a for this place, in degrees from the bottom
        (place-altitude ?c - circle-place) ;; Z0 - R cos(a) for this place
        (latitude ?p - pad) ;; North-south map coordinate
        (longitude ?p - pad) ;; East-west map coordinate
    )

    ;; Prove from the formula's numbers that no two drones can ever be
    ;; closer than min-separation (see WHY THEY NEVER COLLIDE above), that
    ;; the spiral is not too fast to fly, and that the circle clears the
    ;; ground. Done once, on the ground.
    (:action check-formula
        ;; Objects used by this action
        :parameters ()
        ;; These conditions must hold before the action
        :precondition
            (and
                (not (formula-safe))
                ;; neighbours on the circle: (R sqrt 2)^2 = 2 R^2 >= min-separation^2
                (>= (* 2 (* (radius) (radius))) (* (min-separation) (min-separation)))
                ;; opposite on the circle: 2R >= min-separation
                (>= (* 2 (radius)) (min-separation))
                ;; neighbouring pads / joining: E^2 + R^2 >= min-separation^2
                (>= (+ (* (pad-offset) (pad-offset)) (* (radius) (radius)))
                    (* (min-separation) (min-separation)))
                ;; bottom and top pads: 2E >= min-separation
                (>= (* 2 (pad-offset)) (min-separation))
                ;; spiral speed: V^2 + (R w)^2 <= max-speed^2
                (<= (+ (* (formation-speed) (formation-speed))
                       (* (* (radius) (turn-rate)) (* (radius) (turn-rate))))
                    (* (max-speed) (max-speed)))
                ;; the bottom of the circle clears the ground: Z0 - R >= lowest-altitude
                (>= (- (centre-altitude) (radius)) (lowest-altitude))
                ;; whole turns only, so every drone ends where it started
                (> (quarter-turns-total) 0)
            )
        ;; These facts and values change after the action
        :effect
            (formula-safe)
    )

    ;; Climb straight up on the drone's own pad to its own starting height
    ;; (the formula at tau = 0: z = Z0 - R cos(a_i0))
    (:action take-off
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?p - pad)
        ;; These conditions must hold before the action
        :precondition
            (and
                (formula-safe)
                (source-pad ?d ?p)
                (at-pad ?d ?p)
                (landed ?d)
                (gps-ok ?d)
                (clock-synced ?d)
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (landed ?d))
                (airborne ?d)
                (not (at-pad ?d ?p))
                (assign (altitude ?d) (start-altitude ?d))
            )
    )

    ;; Close in from the pad to the drone's starting place on the circle:
    ;; x = V tau + E cos(a_i0) (1 - b(tau / Tj)) goes to x = V tau
    (:action join-circle
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?c - circle-place)
        ;; These conditions must hold before the action
        :precondition
            (and
                (airborne ?d)
                (not (in-circle ?d))
                (start-place ?d ?c)
                (< (quarter-turns-flown ?d) (quarter-turns-total))
                (gps-ok ?d)
            )
        ;; These facts and values change after the action
        :effect
            (and
                (in-circle ?d)
                (in-place ?d ?c)
                (assign (altitude ?d) (place-altitude ?c))
            )
    )

    ;; One quarter turn of the spiral: w (tau - Tj) grows by pi/2, so a_i
    ;; does too and the drone moves on one place - bottom -> left -> top ->
    ;; right -> bottom - its height following z = Z0 - R cos(a_i).
    (:action turn-quarter
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?from - circle-place
             ?to - circle-place)
        ;; These conditions must hold before the action
        :precondition
            (and
                (in-circle ?d)
                (in-place ?d ?from)
                (next-place ?from ?to)
                (< (quarter-turns-flown ?d) (quarter-turns-total))
                (gps-ok ?d)
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (in-place ?d ?from))
                (in-place ?d ?to)
                (increase (quarter-turns-flown ?d) 1)
                (assign (altitude ?d) (place-altitude ?to))
            )
    )

    ;; All the turns flown - back at its starting place - open out of the
    ;; circle to straight above its own destination pad
    (:action leave-circle
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?c - circle-place
             ?p - pad)
        ;; These conditions must hold before the action
        :precondition
            (and
                (in-circle ?d)
                (in-place ?d ?c)
                (start-place ?d ?c)
                (destination-pad ?d ?p)
                (>= (quarter-turns-flown ?d) (quarter-turns-total))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (in-circle ?d))
                (not (in-place ?d ?c))
                (at-pad ?d ?p)
            )
    )

    ;; Land straight down on the destination pad
    (:action land
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?p - pad)
        ;; These conditions must hold before the action
        :precondition
            (and
                (destination-pad ?d ?p)
                (at-pad ?d ?p)
                (airborne ?d)
                (not (in-circle ?d))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (airborne ?d))
                (landed ?d)
                (mission-completed ?d)
                (assign (altitude ?d) 0)
            )
    )
)
