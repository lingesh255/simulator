;; Formula-4: four drones fly from the source to the destination, swapping
;; places all the way, WITHOUT talking to each other. Each drone follows its
;; own mathematical formula of one shared clock (GPS time), and the formula
;; itself guarantees they never come within 10 m of each other.
;;
;; ---------------------------------------------------------------------------
;; THE FORMATION (seen from above, flying up the page)
;;
;;        left lane          right lane
;;        drone 1  <-- W -->  drone 2        front line
;;           |                   |
;;           D                   D
;;           |                   |
;;        drone 3  <-- W -->  drone 4        back line
;;
;;   Drone 1 and drone 3 keep swapping places in the left lane, and drone 2
;;   and drone 4 in the right lane: the FRONT drone SLOWS DOWN and DIPS to
;;   50 m, while the BACK drone SPEEDS UP, CLIMBS to 60 m and PASSES OVER it.
;;   Then the other way round - and again, until the destination.
;;
;; ---------------------------------------------------------------------------
;; THE FORMULA - one for every drone, only the signs differ
;;
;;   Coordinates in metres: x = along the route (0 at the source),
;;   y = across the route (left is +), z = height above the ground.
;;   tau = the shared clock (seconds of the formation's own time).
;;
;;     x_i(tau) = V*tau  +  f_i * (D/2) * cos(w*tau)
;;     y_i(tau) = l_i * ( W/2  +  f_i * (B/2) * sin(w*tau) )
;;     z_i(tau) = Z0  -  f_i * A * sin(w*tau)
;;
;;   f_i = +1 for the front drones (1, 2),   -1 for the back drones (3, 4)
;;   l_i = +1 for the left lane    (1, 3),   -1 for the right lane  (2, 4)
;;
;;   V  = formation-speed   how fast the formation as a whole moves (6 m/s)
;;   D  = line-gap          front line to back line              (20 m)
;;   W  = lane-gap          left lane to right lane              (20 m)
;;   B  = side-step         how far a passing pair steps apart    (6 m)
;;   Z0 = mid-altitude      height between swaps                 (55 m)
;;   A  = altitude-swing    how far up / down a swap goes         (5 m)
;;   w  = swap-rate         radians of clock per second; every half turn
;;                          (w*tau grows by pi) is ONE swap
;;
;;   Written out for each drone:
;;     drone 1: x = V*tau + 10 cos(w tau),  y = +(10 + 3 sin(w tau)),  z = 55 - 5 sin(w tau)
;;     drone 2: x = V*tau + 10 cos(w tau),  y = -(10 + 3 sin(w tau)),  z = 55 - 5 sin(w tau)
;;     drone 3: x = V*tau - 10 cos(w tau),  y = +(10 - 3 sin(w tau)),  z = 55 + 5 sin(w tau)
;;     drone 4: x = V*tau - 10 cos(w tau),  y = -(10 - 3 sin(w tau)),  z = 55 + 5 sin(w tau)
;;
;; HOW THE SWAP COMES OUT OF IT
;;
;;   Speed along the route:  dx/dtau = V - f_i * (D/2) * w * sin(w*tau)
;;
;;   First swap, 0 < w*tau < pi (sin > 0):
;;     drone 1 (f=+1): speed V - 10w*sin  -> SLOWER,  z = 55 - 5 sin -> down to 50 m
;;     drone 3 (f=-1): speed V + 10w*sin  -> FASTER,  z = 55 + 5 sin -> up to 60 m
;;     at w*tau = pi/2 they are level along the route (cos = 0): drone 3 is
;;     right above drone 1 - 10 m higher and 6 m to the side - and passes it.
;;     at w*tau = pi drone 3 is in front, 20 m ahead, both back at 55 m.
;;   Second swap, pi < w*tau < 2 pi (sin < 0): now drone 3 slows and dips to
;;   50 m and drone 1 speeds up, climbs to 60 m and passes it. And so on.
;;   (10w is kept below V - at most V/2 - so no drone ever stops or reverses.)
;;
;; WHY THEY NEVER COLLIDE (at every moment, straight from the formula)
;;
;;   drone 1 vs drone 3 (and 2 vs 4), same lane:
;;       dx = D cos,  dy = B sin,  dz = 2A sin
;;       distance^2 = D^2 cos^2 + (B^2 + (2A)^2) sin^2
;;                 >= the smaller of D^2 and B^2 + (2A)^2
;;       distance  >= min(20, sqrt(36 + 100)) = 11.66 m
;;   drone 1 vs drone 2 (and 3 vs 4), same line:
;;       dx = 0, dz = 0, dy = W + B sin   >= W - B = 14 m
;;   drone 1 vs drone 4 (and 2 vs 3), diagonal:
;;       dy = W exactly                    = 20 m
;;   So every two drones always stay at least 11.66 m apart (> 10 m).
;;   The check-formula action below proves this with the real numbers
;;   before anything takes off.
;;
;; WHY NO COMMUNICATION IS NEEDED
;;
;;   The formula only needs tau and the drone's own f_i and l_i. No drone
;;   ever needs to know where another drone is - so every action below
;;   (except the one-off check-formula) takes exactly ONE drone, and only
;;   reads and changes that drone's own facts.
;;
;; THE ENDS
;;
;;   w is chosen so the route holds an EVEN number of swaps N:
;;   w * T = N * pi, where T = L / V is the flight time. At tau = 0 and
;;   tau = T: sin = 0, cos = +1, so every drone is at 55 m above its own
;;   slot - its own source point (x = +-10, y = +-10 around the picked
;;   source) and its own destination point. It takes off and lands
;;   straight up and down on those slots, at least 20 m from any other.
;; ---------------------------------------------------------------------------

(define (domain formula4-drone-mission)

    ;; Planning features used by these actions
    (:requirements
        :strips ;; Use actions with conditions and effects
        :typing ;; Give objects types such as drone and slot
        :numeric-fluents ;; Use numbers such as speed, gaps and height
        :negative-preconditions ;; Allow a condition to require that a fact is false
    )

    ;; Types of objects used in this mission
    (:types
        drone
        slot
    )

    ;; Facts that can be true or false
    (:predicates
        (at ?d - drone ?s - slot) ;; The drone is on / above this slot
        (source-slot ?d - drone ?s - slot) ;; Where this drone takes off: the formula at tau = 0
        (destination-slot ?d - drone ?s - slot) ;; Where this drone lands: the formula at tau = T
        (landed ?d - drone) ;; The drone is on the ground
        (airborne ?d - drone) ;; The drone has taken off
        (following-formula ?d - drone) ;; The drone is flying its own formula
        (front-line ?d - drone) ;; In the front line now (f = +1 on this half turn)
        (back-line ?d - drone) ;; In the back line now (f = -1 on this half turn)
        (left-lane ?d - drone) ;; l = +1
        (right-lane ?d - drone) ;; l = -1
        (gps-ok ?d - drone) ;; The drone has a GPS fix - its position, and the shared clock
        (clock-synced ?d - drone) ;; The drone's own clock is set to GPS time (tau)
        (formula-safe) ;; The formula's numbers keep every two drones >= min-separation apart
        (mission-completed ?d - drone) ;; The drone has landed on its destination slot
    )

    ;; Numbers used by the planner
    (:functions
        (formation-speed) ;; V
        (line-gap) ;; D
        (lane-gap) ;; W
        (side-step) ;; B
        (mid-altitude) ;; Z0
        (altitude-swing) ;; A
        (swap-rate) ;; w
        (route-length) ;; L, source to destination
        (swaps-total) ;; N, swaps each pair flies (even)
        (min-separation) ;; Closest any two drones may ever get
        (swaps-flown ?d - drone) ;; Swaps this drone has flown so far
        (altitude ?d - drone) ;; The drone's height now
        (swap-altitude ?d - drone) ;; Its height in the middle of its latest swap: 50 (slowed) or 60 (passed)
        (latitude ?s - slot) ;; North-south map coordinate
        (longitude ?s - slot) ;; East-west map coordinate
        (slot-altitude ?s - slot) ;; Height the formula puts the drone at above this slot (Z0)
    )

    ;; Prove from the formula's numbers that no two drones can ever be
    ;; closer than min-separation (see WHY THEY NEVER COLLIDE above), and that
    ;; no drone ever has to stop or fly backwards. Done once, on the ground.
    (:action check-formula
        ;; Objects used by this action
        :parameters ()
        ;; These conditions must hold before the action
        :precondition
            (and
                (not (formula-safe))
                ;; drone 1 vs 3 when level heights (cos = 1):  D >= min-separation
                (>= (line-gap) (min-separation))
                ;; drone 1 vs 3 when level along the route (sin = 1):
                ;; B^2 + (2A)^2 >= min-separation^2
                (>= (+ (* (side-step) (side-step))
                       (* (* 2 (altitude-swing)) (* 2 (altitude-swing))))
                    (* (min-separation) (min-separation)))
                ;; drone 1 vs 2 when stepped towards each other:  W - B >= min-separation
                (>= (- (lane-gap) (side-step)) (min-separation))
                ;; the slowest drone still flies forwards: (D/2) * w < V
                (< (* (/ (line-gap) 2) (swap-rate)) (formation-speed))
                ;; the swaps fill the route exactly: w * (L / V) = N * pi
                (> (swaps-total) 0)
            )
        ;; These facts and values change after the action
        :effect
            (formula-safe)
    )

    ;; Take off straight up from the drone's own source slot to Z0
    ;; (the formula at tau = 0: sin = 0 -> z = Z0)
    (:action take-off
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?s - slot)
        ;; These conditions must hold before the action
        :precondition
            (and
                (formula-safe)
                (source-slot ?d ?s)
                (at ?d ?s)
                (landed ?d)
                (gps-ok ?d)
                (clock-synced ?d)
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (landed ?d))
                (airborne ?d)
                (following-formula ?d)
                (not (at ?d ?s))
                (assign (altitude ?d) (mid-altitude))
            )
    )

    ;; One swap flown as the FRONT drone: half a turn of w*tau with this
    ;; drone's sin term pulling it back. Its speed drops to V - (D/2) w sin,
    ;; it dips to Z0 - A = 50 m while the other drone passes overhead, and it
    ;; ends the half turn D behind, in the back line, back at Z0.
    (:action slow-and-dip
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (following-formula ?d)
                (front-line ?d)
                (gps-ok ?d)
                (< (swaps-flown ?d) (swaps-total))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (front-line ?d))
                (back-line ?d)
                (increase (swaps-flown ?d) 1)
                (assign (swap-altitude ?d) (- (mid-altitude) (altitude-swing)))
                (assign (altitude ?d) (mid-altitude))
            )
    )

    ;; One swap flown as the BACK drone: its speed rises to V + (D/2) w sin,
    ;; it climbs to Z0 + A = 60 m, passes over the front drone, and ends the
    ;; half turn D in front, in the front line, back at Z0.
    (:action speed-up-and-climb
        ;; Objects used by this action
        :parameters
            (?d - drone)
        ;; These conditions must hold before the action
        :precondition
            (and
                (following-formula ?d)
                (back-line ?d)
                (gps-ok ?d)
                (< (swaps-flown ?d) (swaps-total))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (back-line ?d))
                (front-line ?d)
                (increase (swaps-flown ?d) 1)
                (assign (swap-altitude ?d) (+ (mid-altitude) (altitude-swing)))
                (assign (altitude ?d) (mid-altitude))
            )
    )

    ;; All N swaps flown - the clock reads T, so the formula has put the
    ;; drone at Z0 right above its own destination slot
    (:action arrive
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?s - slot)
        ;; These conditions must hold before the action
        :precondition
            (and
                (following-formula ?d)
                (destination-slot ?d ?s)
                (>= (swaps-flown ?d) (swaps-total))
            )
        ;; These facts and values change after the action
        :effect
            (and
                (not (following-formula ?d))
                (at ?d ?s)
            )
    )

    ;; Land straight down on the destination slot
    (:action land
        ;; Objects used by this action
        :parameters
            (?d - drone
             ?s - slot)
        ;; These conditions must hold before the action
        :precondition
            (and
                (destination-slot ?d ?s)
                (at ?d ?s)
                (airborne ?d)
                (not (following-formula ?d))
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
