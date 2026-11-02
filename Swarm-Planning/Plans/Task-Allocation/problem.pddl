(define (problem task-allocation-001)
    (:domain task-allocation)
    (:objects
        drone1 drone2 drone3 drone4 - drone
        source destination - location
    )
    (:init
        (source source)
        (destination destination)
        (connected source destination)
        (= (latitude source) 18.3901547)
        (= (longitude source) 79.0495640)
        (= (latitude destination) 18.3876026)
        (= (longitude destination) 79.1440000)

        (at drone1 source) (standby drone1) (capable drone1)
        (at drone2 source) (standby drone2) (capable drone2)
        (at drone3 source) (standby drone3) (capable drone3)
        (at drone4 source) (standby drone4) (capable drone4)
        (communication-ok drone1)
        (communication-ok drone2)
        (communication-ok drone3)
        (communication-ok drone4)

        ;; Default: no failures, so only one drone leaves the source.
        ;; To exercise handoffs, add (will-fail drone1), etc. below.
        ;; A planner may choose any capable first drone. To force the
        ;; initial drone1 flight, use the committed state in README.md.
    )
    (:goal (mission-finished))
)
