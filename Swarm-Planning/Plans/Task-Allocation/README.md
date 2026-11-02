# Task-Allocation

Four drones start at `source`. One capable drone takes responsibility and
flies toward `destination`. Other drones remain at the source until the
responsible drone reports a problem. It reserves one capable standby,
which launches from the source; both drones must be airborne and have
working radios to share mission data and transfer responsibility.
The replacement can repeat this process if it also fails.

`travel-to-destination` lands the responsible drone, marks it complete,
and closes the mission in one action. Every action requires the mission
to remain unfinished, so no drone can be called after arrival.

## Inputs and failure examples

Edit `problem.pddl` to set source/destination location objects and their
`connected` relation. `capable` means a drone meets the operation's
payload, battery, navigation, and health requirements; omit it for drones
that cannot perform the operation. Coordinates and physical flight
control are outside this symbolic PDDL model.

The default problem has no failures: one drone flies, three stay at source.
Add `(will-fail drone1)` to model a recoverable in-flight failure of drone1.
Classical planning knows these facts and may start a different drone.
To exercise a committed chain starting with drone1, replace its
`(at drone1 source) (standby drone1)` facts with:

```lisp
(mission-started)
(responsible drone1)
(mission-data drone1)
(airborne drone1)
(en-route drone1 source destination)
```

Keep `(capable drone1)` and its radio fact. Add `will-fail` facts for
drone1, drone2, and drone3 to make drone4 the only possible winner.
The planner can select drone4 immediately after drone1 fails, since the
scenario's failures are known in advance. Repeated handoffs are supported
when successive active drones develop faults during execution and the
current state is replanned. This is a
snapshot after the initial launch; the unmodified problem starts all
four drones on the ground. If every capable drone fails, no solution exists.

## Execution limits

This is a classical PDDL domain/problem, not a live fault monitor or a
contingent policy. Unexpected faults require the executor to update the
current state and replan. A failed drone must retain flight control and
communication long enough for the airborne handoff; total radio loss or
a crash requires an external copy of the mission data. `retired` removes
the old drone from task allocation; its emergency landing is handled
outside this domain.

The existing simulator needs execution support for these new handoff
actions before it can animate or fly this behavior. Adding these files
alone does not implement live simulator failover.
