"""Build the project report from authored explanations and the current PDDL sources."""
from pathlib import Path
import re
import sys
from html import escape

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.tmp_doc_tools'))
OUT = ROOT / 'docs'
parts = []
def add(text):
    parts.append(text.strip() + '\n\n')

add('''# PDDL-Based Mission Planning for a Drone Swarm Simulator

MTP Project Report â€” Planning Component

Submitted by: [Student name]\nRoll number: [Roll number]\nDepartment: [Department]\nInstitution: [Institution]\nSupervisor: [Supervisor name]\nDate: 23 October 2026

This report documents the PDDL planning component in the supplied simulator workspace. Personal details must be completed before submission. The structure is adapted from Reference Doc.pdf; its sample PhD title, institution, certificate, publications and personal statements have not been treated as project facts.

## Abstract

This project provides a planning layer for a drone swarm simulator. Planning Domain Definition Language (PDDL) describes the conditions under which a drone may move, join a formation, search an area, perform a show, or complete a mission. A domain file defines the available actions. A problem file defines the drones, locations, initial conditions and mission goals. The simulator prepares a problem from map inputs and sends it to the Expressive Numeric Heuristic Search Planner (ENHSP). The planner returns an ordered sequence of actions. Python services convert relevant actions into geographic routes and, where required, separate routes and timing information for each drone.

The current workspace contains ten mission types: point-to-point travel, forest search, V formation, two-dimensional grid formation, three-dimensional grid formation, parabola formation, figure-eight formation, star formation, planetary orbits and road inspection. This report explains their models, action sequences, completion conditions, communication relationships and integration with execution. It distinguishes symbolic planning conditions from actual network communication and flight control. The report describes the implementation and its limits; it does not claim measured performance or flight validation that was not performed during this documentation task.

Keywords: PDDL, drone swarm, automated planning, ENHSP, formation control, mission planning, simulation.

## Contents

1. Introduction and scope
2. Planning concepts and system architecture
3. Mission models
4. Communication and coordination
5. Safety, failures and model limitations
6. Execution workflow and evaluation method
7. Conclusions and future work
References
Appendix A. Complete PDDL source listings

## List of tables and figures

Table 3.1: Mission inventory. Table 4.1: Neighbour relationships. Table 6.1: Proposed validation cases. Figure 2.1: Planning and execution flow. Figure 4.1: Communication paths.

## Glossary, abbreviations and notation

PDDL: Planning Domain Definition Language. ENHSP: Expressive Numeric Heuristic Search Planner. GCS: Ground Control Station. GUI: Graphical User Interface. UDP: User Datagram Protocol. MAVLink: the flight communication protocol used by the mission execution components. SITL: Software in the Loop. SYSID: drone system identifier.

Predicate: a true or false statement about the planning state. Numeric fluent: a numeric value that may change after an action. Precondition: a condition required before an action. Effect: a state change caused by an action. Ground action: an action whose parameters have been replaced by actual object names. Corridor: the leader's geographic route. Slot: a drone's assigned place relative to a formation or path. Neighbour: a drone connected by a defined logical relationship.

N denotes drone count. B denotes battery level and E denotes action energy cost. R and C denote grid rows and columns. L denotes the number of vertical layers. Distances and offsets in the geometry code are in metres. Battery values are model units unless explicitly calibrated; a value of 100 does not establish a physical capacity in watt-hours.

# 1. Introduction and scope

The planning problem is to decide which actions can achieve a drone mission while respecting the conditions represented in the model. A route alone is insufficient: a drone may also need adequate battery, healthy GPS, a working communication link and the correct formation state. PDDL makes these requirements explicit and allows a planner to search for an action sequence that satisfies them.

The planning contribution consists of the domain and problem models, their mission rules, numeric resource conditions, formation readiness conditions and completion goals. The surrounding simulator prepares geographic inputs, requests a plan, extracts routes and executes flight commands. These supporting components are described where they explain how a PDDL plan becomes a simulated mission. Their presence does not imply that every component was authored as part of the student's individual contribution.

The report covers the current files in Swarm-Planning/Plans/ and the integration code in services/ and engine/. It uses the reference PDF as a structural example, rather than as a technical source for drone behaviour. The existing four mission PDFs in docs/ are earlier mission-specific documents; current source files are the authority for the ten-mission inventory. Archived run summaries are historical evidence and are not presented as new experimental results.

# 2. Planning concepts and system architecture

## 2.1 Domain and problem files

A domain file defines object types, predicates, numeric functions and actions. A problem file refers to that domain and provides concrete objects, initial facts, numeric values and goals. The planner searches from the initial state to a state satisfying every goal condition. A goal is a logical condition, not an instruction sent directly to an aircraft.

For example, (at drone1 source) states that drone1 is at the source. (communication-ok drone1) represents a working communication condition. (= (battery drone1) 85) sets an initial battery value. A movement action can require B > E, remove the old location, add the new location and reduce battery to B - E. A strict greater-than condition rejects a move when battery exactly equals the required energy.

The models use typed objects and numeric fluents. Formation models also use equality, negative and disjunctive conditions, universal conditions and conditional effects. A universal condition can require every formation member to be ready. A conditional effect can update all objects having the required role. These are discrete state transitions. The files do not use durative actions; a numeric launch counter does not by itself measure wall-clock seconds.

## 2.2 From map selection to flight

Figure 2.1. Planning and execution flow

```text
Operator map inputs and selected drones
                 |
                 v
PlanService -> retarget/expand problem -> generated problem.pddl
                                            |
Domain file --------------------------------+-> ENHSP (Java)
                                                   |
                                                   v
                                    ordered actions and plan.txt
                                                   |
                                                   v
                           route extraction and geometric expansion
                                                   |
                                                   v
                             per-drone flight routes / timed tracks
                                                   |
                                                   v
                               execution backend -> flight controller
                                                   |
                                      telemetry -> GUI / coordination
```

services/plan_service.py discovers folders containing a domain and a problem file. It identifies the mission kind using the domain name inside the file. Search uses SearchProblem.pddl; the other current templates use problem.pddl. Problem preparation can change coordinates, add drones, rebuild neighbour facts and recompute route energy. The generated problem is stored in Swarm-Planning/Results/ENHSP/<mission>/runs/<timestamp>/problem.pddl. Planner output is stored beside it as plan.txt.

engine/pddl_planner.py locates Java and the ENHSP JAR, starts the planner as a subprocess and parses action lines into PlanStep objects. A step contains an index, action name and arguments. The default timeout is 30 seconds, with some mission-specific calls allowing longer. ENHSP_JAR can override the default tools/enhsp/enhsp-dist/enhsp.jar location. The worker runs outside the GUI thread and reports completion or failure through Qt signals.

Route extraction depends on the domain. Travel uses travel actions. Search uses move-search and return-to-base. Formation corridors use formation-cruise. The geometric code derives offset member routes, curved paths or timed show tracks. PDDL therefore plans mission logic and selected movement structure; Python supplies detailed geometric information that is not fully represented by symbolic waypoint names.

## 2.3 Meaning of formation locations

Several models assign the leader and members to the same symbolic location after a coordinated move. This means they share a formation reference waypoint. It does not mean that aircraft should occupy identical physical coordinates. Along-track, cross-track, arc-length and altitude offsets separate their actual routes. The geometric conversion is essential to understanding these models.

# 3. Mission models

Each mission section explains its purpose, its normal action flow, the result required for completion and its communication assumptions. The action descriptions are extracted from the current domain comments. Appendix A contains every domain and problem file verbatim, including numeric values, preconditions and effects.

Table 3.1. Mission inventory

| Folder | Mission | Main operator inputs |
| --- | --- | --- |
| travell | Point-to-point travel | Source, destination, optional restricted area |
| Search | Forest or area search | Base, search polygon, selected drones |
| vformation | V formation | Source, destination, selected drones |
| gridformation | Horizontal grid | Source, destination, selected drones |
| 3dgridformation | Layered grid or cube | Source, destination, selected drones |
| parabolaformation | Three-drone parabola | Source and destination |
| figure8formation | Figure-eight train | Base, left end, crossing, right end, laps |
| starformation | Circle and star show | Show centre, selected drones, laps |
| planets | Planetary orbit show | Sun centre, selected drones, laps |
| Inspect | Road inspection | Source and destination on a road |
''')

missions = [
('travell','Point-to-point travel', '''The swarm travels from a chosen source to a destination. The reference drone is expanded for the selected fleet. Routes may include intermediate waypoints to avoid a marked restricted area. The model uses connected and safe-route facts, distance checks, battery checks and mission-completed conditions.

The normal sequence is check-distance, check-battery, plan-safe-route, travel and reach-destination. Movement consumes the energy for that edge and clears route-planned, so another planning action is required for the next movement. The template starts drone1 at battery 85, with minimum-return-battery 30 and health 95. These are sample instance values, not fixed rules for every mission.

The goal requires drone1 at the destination with mission-completed. The template does not require a final landed state. Failure actions represent warnings, controller notification, return, hover and a sacrifice state. The sacrifice-drone action is an abstract model outcome, not evidence of a physical destruction procedure. No fixed drone-to-drone neighbour mesh is defined for this mission.'''),
('Search','Forest and area coverage', '''The search mission divides work among drones rather than giving the fleet one identical route. The operator marks a base and search polygon. engine/search_problem.py prepares the geographic search instance and drone assignments. The service requires at least two drones.

The normal flow checks battery and separation, moves to assigned locations, starts searching, marks areas covered, completes the forest search and returns drones to base. move-search requires a safe route, healthy systems and separation-ok. Its effects clear battery-checked and separation-ok so those conditions must be established again. The sample problem specifies a 10 m minimum separation between the two reference drones and a 50 m coverage width.

The final goal requires forest-search-completed, completed drone missions and both drones at base. The completion action in the domain explicitly accepts two distinct covered areas. It should not be interpreted as a universal proof that every arbitrary polygon cell has been visited: the generated problem, assignments and extracted routes also determine coverage. Separation predicates describe state checks and do not implement a peer radio protocol. No fixed communication direction count is specified.'''),
('vformation','V formation', '''A leader occupies the apex and equal ranks extend along the left and right wings. The expansion code requires an odd fleet count of at least three, giving N = 1 + 2r for r ranks per wing. The geometry uses approximately 17.32 m behind and 10 m sideways per rank, as defined in the service constants and template.

Drones complete pre-flight checks, take off, climb and occupy assigned slots. The leader can hold while the launch counter advances. establish-v-formation requires the members to be ready. Each formation-cruise moves the leader and opens the formation; close-up-wings updates followers and restores the closed condition before the next leader leg.

The goal requires the established V, members in formation at cruise altitude, the destination reference location and completed missions. Neighbour relationships form two wing chains joined at the leader. The leader has two wing neighbours; an internal wing drone has inward and outward neighbours; a wing tip has one. Compass labels describe slot geometry and do not mean that every drone communicates in all eight compass directions.'''),
('gridformation','Two-dimensional grid formation', '''Drones occupy a horizontal rectangular grid with 10 m spacing. A perfect-square drone count gives a square. Other counts use an exact rectangular factorisation chosen by grid_dimensions. The GUI/service uses at least four drones for a visible grid; the lower-level expansion function supports smaller reference cases.

After pre-flight, takeoff and climb, each drone takes a grid slot. establish-grid-formation confirms readiness. The leader advances using formation-cruise, followed by close-up-grid for the members. The closed-state gate prevents another symbolic leader leg before the members have caught up.

The goal requires the established grid, member readiness, arrival and completed missions. A drone has up to four neighbours: front, back, left and right in the grid frame. Diagonals are excluded. Corner drones in a full rectangle normally have two neighbours, non-corner edge drones three and interior drones four. The generated facts are symmetric. Actual neighbour health is represented by neighbor-comm-ok rather than by a packet exchange inside PDDL.'''),
('3dgridformation','Three-dimensional grid or cube formation', '''This mission extends a grid into stacked horizontal layers. Eight drones form a 2 by 2 by 2 cube. Larger counts use the block dimensions calculated by cube_dimensions, with a partial layer where needed. The GUI/service requires at least eight drones, although the static PDDL template contains only a leader and one reference member for expansion.

The model adds layer objects, in-layer, layer-below, layer-launched and cleared-to-launch. The bottom layer starts cleared. open-next-layer releases a higher layer only after every drone in the lower layer has reached cruise altitude. Each drone has a layer-dependent altitude and vertical offset. After all layers are available, drones take cube slots, establish the cube, cruise and close up before the next leg.

The communication model permits up to 24 neighbour offsets: eight in the same layer, eight in the layer above and eight in the layer below. The two positions directly overhead and underneath are excluded. This is the actual repository rule, rather than the conventional 26-neighbour cubic mesh. Boundary and unfilled cells reduce the number of real neighbours. Layered launch is a symbolic ordering rule; physical clearance still depends on the generated trajectories and execution.'''),
('parabolaformation','Parabola formation', '''Exactly three drones form a vertical cross-section: a central vertex and two outer arms. The vertex cruises at 50 m and the arms at 60 m in the template. The service places each arm 10 m sideways from the vertex. The higher arms create an upward-opening three-point parabola when viewed across the flight direction.

The mission adds explicit establish-comm-link and confirm-swarm-comm actions. Directed link-up facts represent peer-link availability. Drones take their slots and establish the parabola. formation-cruise advances the vertex and hold-parabola-shape brings the arms into the corresponding state while checking the required links and readiness.

Each drone must maintain links with both of the other drones. There are three unordered pairs, or six directed relationships when both directions are listed. This is two peers per drone, not a general two-compass-direction radio model. The goal requires all three swarm-comm-ready conditions, their target altitudes, arrival in formation and completed missions. Neighbour-link failure can clear link-up in both directions for the affected relationships.'''),
('figure8formation','Figure-eight formation', '''The operator selects the base, left lobe end, middle crossing and right lobe end, then a lap count. engine/figure8_problem.py builds a curved figure-eight and base transit paths. Drones hold ordered slots along a shared path. The current geometry spreads them around the full loop at equal slot distances; a blanket statement that every slot is exactly 10 m apart would be inaccurate. Ten metres is the spatial clearance target used by the geometry checks.

The symbolic sequence performs pre-flight checks, takeoff, climb, slot assignment and formation establishment. fly-to-pattern enters the loop. formation-cruise and follow-leader advance the leader and followers. start-next-lap counts intermediate laps; finish-laps counts the last lap. return-to-base and a final follower update precede complete-figure8-mission.

The geometry checks the crossing and the shared route. It uses a crossing path-gap threshold of 10 times sqrt(2) metres, approximately 14.14 m, for a right-angle crossing, corresponding to a 10 m straight-line clearance under that construction. Additional empty slots may be introduced when a full set of occupied equal slots would cause a crossing conflict. The route generator includes an extra flown lap to fill and empty the spread while providing the requested fully spread laps. These details belong to geometric execution and should be distinguished from the symbolic lap counter.

Communication is with the previous and next drone in the train order, up to two distinct peers. The PDDL goal completes the mission back at base and at cruise altitude; it does not require landed. Landing is handled by the generated execution path. The runtime uses shared pace messages and local speed control rather than a demonstrated direct peer-to-peer packet network.'''),
('starformation','Star and circle drone show', '''This mission performs a show around one selected centre. Drones take off from separate pads and form a circle. During each lap they form a star, return to a circle, spin the circle, form the star again, spin the star and return to the circle. land-show completes the show on the original pads.

The domain records shape and phase progress. engine/star_problem.py generates the separate timed tracks. Drones retain their order around the centre during shape changes. The geometry adapts the number of star points to fleet size and scales the show to meet its spacing checks. Its documented transition criterion is at least 90 percent of the nominal spacing, so it should not be reported as an unconditional 10 m minimum throughout every transition.

ring-neighbor links each drone to the previous and next member around the ordered ring. A normal ring of at least three drones gives two distinct peers per drone. Small reference templates may have fewer distinct peers. Timed schedules use a shared pace clock so shape changes and rotations occur together. The final goal requires the requested laps, completed missions, return to source and landed states.'''),
('planets','Planetary orbit formation', '''Drones represent planets around a selected sun centre. The implementation supports two to eight drones. Orbit k has radius 50 times (k + 1) metres, from Mercury at 50 m to Neptune at 400 m. Neighbouring orbits turn in opposite directions.

Pre-flight, takeoff and climb precede enter-orbits. fly-lap advances the show counter and land-planets finishes it. A lap is one revolution of the outermost drone. Timed geometry coordinates the tracks so the drones line up again at lap boundaries. These are designed show trajectories rather than a physical model of planetary gravity or true orbital periods.

orbit-neighbor forms an open chain by orbit index. The innermost and outermost drones have one adjacent-orbit peer, and intermediate drones have two. Inward and outward describe neighbouring orbit assignments, not constant geographic bearings between moving drones. The final goal requires the requested laps, completed missions, drones back at source and landed.'''),
('Inspect','Road inspection', '''A single drone follows a road from a selected source to destination, inspects its segments, returns along the road and lands at source. engine/road_inspection.py obtains the road shape from an OSRM routing service. Source and destination must lie on usable connected roads. Failure to obtain a route is reported before execution.

The generated road segments replace the template's single reference segment. The normal sequence is pre-flight-check, takeoff, climb-to-cruise-altitude, inspect-segment for the road chain, finish-inspection, return-along-road and land-at-source. Segment counters represent inspection progress.

The goal requires road-inspected, drone1 at source, landed and mission-completed. There is no inter-drone neighbour network in this single-drone mission. GCS commands and returned status are the relevant communication paths. A symbolic inspected segment records task completion in the model; it does not establish that a real camera detected a road defect.'''),
]

for i, (folder, title, prose) in enumerate(missions, 1):
    domain = ROOT / 'Swarm-Planning' / 'Plans' / folder / 'domain.pddl'
    text = domain.read_text(encoding='utf-8')
    name = re.search(r'\(domain\s+([^\s)]+)', text).group(1)
    add(f'## 3.{i} {title}\n\nSource folder: Swarm-Planning/Plans/{folder}/. Domain name: {name}.\n\n{prose}')
    add('Action catalogue (source order; this is not a newly solved plan):')
    # Capture the comments directly preceding each action, preserving the author's explanation.
    matches = list(re.finditer(r'\(:action\s+([^\s)]+)', text))
    for j, m in enumerate(matches):
        prefix = text[:m.start()].rstrip()
        comments = []
        for line in reversed(prefix.splitlines()):
            if not line.strip():
                if comments: break
                continue
            if line.lstrip().startswith(';'):
                comments.insert(0, line.strip().lstrip(';').strip())
            else: break
        explanation = ' '.join(comments) or ('Represents ' + m.group(1).replace('-', ' ') + ' in the planning state.')
        add(f'- **{m.group(1)}**: {explanation}')

add('''# 4. Communication and coordination

## 4.1 Two different meanings of direction

Communication direction can mean data flow or the position of a neighbour. Data flow between the GCS and a drone is bidirectional: commands go down to the drone, and telemetry, acknowledgements and heartbeats return to the GCS. A formation direction means an adjacent slot, such as front or left. It does not specify a radio antenna beam, a frequency, or the number of independent physical communication channels.

Table 4.1. Modelled neighbour relationships

| Mission | Relationship | Maximum distinct peers per drone |
| --- | --- | --- |
| Travel | No fixed peer topology | Not specified |
| Search | Pairwise separation checks | No fixed communication degree |
| V | Adjacent drones along the wing chains | 2; wing tips have 1 |
| Horizontal grid | Front, back, left, right | 4 |
| Cube | 8 same layer + 8 above + 8 below | 24 |
| Parabola | Each drone linked to both others | 2 |
| Figure-eight | Previous and next in train order | 2 |
| Star | Previous and next around ring order | 2 |
| Planets | Adjacent inner and outer orbits | 2; endpoints have 1 |
| Inspection | Single drone | 0 inter-drone peers |

These are maximum logical degrees, not the number of peers guaranteed in every instance. A two-drone ring cannot provide two different peers to either drone. Edges, corners, incomplete layers and fleet size change the actual counts. The GCS link is a separate relationship and is not included in the peer counts.

For a complete R by C horizontal grid, the number of unordered neighbour pairs is R(C - 1) + C(R - 1). Symmetric PDDL facts produce twice that number of directed facts. A V wing tree and an open orbit chain each have N - 1 unordered links. A normal ring with N at least three has N links. The three-drone parabola has three pairs. The cube count must be computed from occupied cells using the implemented offsets; multiplying N by 24 would overcount boundary links and count each pair twice.

## 4.2 Symbolic link conditions

communication-ok represents general communication readiness, while neighbor-comm-ok represents a drone's neighbour communication condition in formation models. Facts such as grid-neighbor, wing-neighbor, train-neighbor, ring-neighbor and orbit-neighbor describe the intended topology. Most movement checks use the aggregate neighbor-comm-ok predicate rather than checking every pair individually. Merely listing neighbours therefore does not establish that a live network has verified those links.

The parabola model is more explicit: establish-comm-link creates link-up relationships, and confirmation checks required links. Even here, the facts are planning state. They are not UDP or MAVLink messages. notify-ground-controller records controller-notified symbolically and does not itself send a network packet. Runtime code must observe real state and implement the corresponding response.

## 4.3 Runtime data flow

Figure 4.1. Communication paths

```text
Planning worker -> Java planner: domain/problem file arguments
Java planner -> planning worker: console plan output

GCS -------- COMMAND / TRAIN_PACE --------> Drone endpoint
GCS <---- TELEMETRY / HEARTBEAT / ACK ----- Drone endpoint
                                               |
                                      local formation keeping
                                               |
                                          flight controller
```

engine/protocol.py defines a JSON-over-UDP transport with one datagram per message. The default GCS port is 14690, and a drone port is 14700 + SYSID; SYSID 1 therefore uses 14701. A command carries a drone identifier, command name, sequence and timestamp, with optional source, destination and parameters. Commands include NAVIGATE, ARM, TAKEOFF, HOLD, RESUME, RETURN_TO_LAUNCH, LAND and TRAIN_PACE. Telemetry carries position, velocity, battery, GPS, health and mission state. ACK reports acceptance or refusal; it does not prove that a flight operation has finished.

engine/drone_link.py separates socket handling from drone-thread logic. Its declared telemetry interval is 0.5 seconds and heartbeat interval is 1 second. These are configured intervals, not measured end-to-end delivery guarantees. UDP has no inherent reliable-delivery guarantee; protocol sequence fields and acknowledgements should not be described as proof of full reliable transport without examining the relevant retry and timeout behaviour.

The execution code also supports MAVLink mission construction and upload. engine/mavlink_mission.py, engine/mavlink_interpreter.py and services/mavlink_flight_service.py belong to that path. scripts/pddl_to_mavlink.py converts an already solved travel route into takeoff, waypoint and landing mission items and can validate them using a local flight-controller model or connect to a configured endpoint. The JSON protocol's ports should not be assumed to apply to every MAVLink or Renode path.

For trains and timed shows, services/thread_backend.py distributes a shared pace. engine/train_keeping.py compares each drone's own route progress with its required progress and calculates a local speed correction. The GCS can include the front drone's last known position and slow the common pace for a lagging drone. This is centrally distributed coordination with local control. The inspected code does not justify describing the PDDL neighbour graph as a fully implemented decentralised peer-to-peer radio network.

# 5. Safety, failures and model limitations

## 5.1 Energy and route safety

Movement usually requires adequate battery and healthy GPS, communication and drone state. Climb actions consume climb-energy, while follower updates often use cruise-leg-energy. The leader can instead consume edge-specific energy-required. These are intentionally different abstractions. Their values need calibration before they can support claims about real endurance.

The presence of minimum-return-battery does not mean every move reserves enough energy for a complete return. Several actions only compare battery with the next action cost. Similarly, the presence of max-battery does not impose a bound unless an action actually checks it. Route-level feasibility must be assessed from the complete action sequence and the required recovery route.

Problem retargeting can rotate, scale and translate the reference route. It recalculates edge distances and energy, preserving edge-specific reference cost rates. Restricted-area handling uses geometric checks and buffered detours. Formation services increase the corridor buffer to account for lateral formation reach. A safe leader corridor alone is insufficient evidence that every member trajectory is clear under every shape or turn.

## 5.2 Failure conditions and responses

The models include detection actions for GPS loss, communication loss and low health. Formation domains generally add neighbour-link loss. Effects can set failure flags and remove flying or in-formation. Travel includes controller orders, hovering, return and sacrifice state. Search includes an emergency low-battery return. The exact actions available in each mission are listed in Chapter 3 and defined in Appendix A.

These actions do not occur automatically in response to real time. A PDDL planner operates on a supplied state. An external event must first be reflected in that state before a detection action can apply. In particular, healthy initial facts will not become false merely because a runtime sensor later fails. A monitoring and replanning integration is needed to connect changing telemetry with updated planning facts.

A failure flag may also make the original success goal unreachable. A robust operational design should distinguish normal completion from safe recovery goals. A symbolic controller notification during communication loss is a model abstraction; actual delivery cannot be assumed when the link is unavailable.

## 5.3 Limits of the current representation

The models use instantaneous actions and discrete numeric updates, so they do not prove continuous collision avoidance, realistic acceleration limits or timing guarantees. A launch counter increment models ordering, not elapsed physical time. Separation checks at symbolic states are not continuous trajectory certificates. Detailed geometry and runtime control add necessary behaviour outside PDDL.

The report records source behaviour without silently changing it. Two points deserve further model review: the travel domain uses disjunction in some actions without listing disjunctive-preconditions, and the search domain uses equality without listing the corresponding requirement declaration. Planner acceptance of a file does not establish strict portability across PDDL tools. These observations do not constitute a newly executed planner failure.

Static templates are intentionally smaller than runtime missions in several folders. A two-drone cube template is an expansion seed, not the operational minimum cube size. Some template values and module comments describe earlier configurations. Current function behaviour and generated problem files should be checked when reporting exact fleet sizes or geometry.

# 6. Execution workflow and evaluation method

## 6.1 Reproducing a planning run

Install the project dependencies from requirements.txt and provide a usable Java installation and ENHSP JAR. Start the application using python main.py in the project environment. Select a mission and the required number of drones. Choose the appropriate map inputs: two endpoints for transit formations, a polygon for search, four points for the figure-eight, a centre for a show, or road endpoints for inspection.

Request planning and inspect the resulting route before execution. Preserve the generated problem.pddl and plan.txt together because the template alone does not describe the selected coordinates or expanded fleet. On success, inspect extracted routes, member offsets and timing schedules. On failure, retain the planner output and distinguish a timeout or missing dependency from a genuine infeasible problem. The wrapper's PlanNotFound exception is raised when no action sequence is parsed, so that exception name alone is not a complete mathematical diagnosis of unsolvability.

A direct template-solving command has this shape:

```text
java -jar tools/enhsp/enhsp-dist/enhsp.jar -o Swarm-Planning/Plans/travell/domain.pddl -f Swarm-Planning/Plans/travell/problem.pddl
```

This solves the stored sample instance. To reproduce a GUI run, substitute the generated problem file under that run's timestamp directory. Some larger formation calls use mission-specific planner options, including naive grounding, so the exact service invocation matters when reproducing performance.

## 6.2 Suggested evaluation cases

Table 6.1. Validation cases to execute and record

| Case | Expected property | Evidence to preserve |
| --- | --- | --- |
| Normal travel | Goal reached through allowed edges | Generated problem and action sequence |
| Restricted-area transit | Member routes clear buffered region | Route geometry and clearance calculation |
| Insufficient battery | Unsafe movement cannot satisfy its gate | Initial values, failed preconditions or planner output |
| Search coverage | Required assignments covered and drones return | Coverage facts and full extracted routes |
| V and horizontal grid | Every member closes up before next leg | Ordered actions and neighbour facts |
| Cube ascent | Lower layer ready before next layer release | open-next-layer preconditions and action order |
| Parabola links | Every required peer link confirmed | link-up and swarm-comm-ready transitions |
| Figure-eight capacity | Unsafe layout rejected or safely adjusted | Slot count, crossing and shared-path clearance checks |
| Star and planets | Shared phase timing and final landing | Timed tracks, lap counters and final state |
| Road inspection | All generated segments processed, return and landing | Road geometry, segment counters and final goal |
| Runtime fault | Monitoring detects fault and chooses recovery | Telemetry, commands and any replanning problem |

These are proposed evaluation cases, not claims that the checks passed. For formal evaluation, record drone count, waypoint count, action count, planner options, elapsed planning time, energy values, minimum geometric separation and final goal status. Use an independent plan validator where supported and replay the plan against the generated initial state. Integration testing must additionally confirm that the executed route matches the planned route and that expected final flight states are reached.

## 6.3 Evidence available for this report

This documentation task inspected all ten current domain/problem pairs, the reference template and the relevant planning, geometry, protocol and execution source files. It did not run a new ten-mission planner benchmark or a flight experiment. Historical figures in Swarm-Planning/Plans/PLANS_STATUS.txt are dated and were not reused as current success rates. The complete source appendix provides an auditable snapshot of the planning models used for this report.

# 7. Conclusions and future work

The planning layer provides a common method for describing ten different drone missions. Domain actions state when movement, formation changes and task completion are permitted. Problem generation connects these rules to selected map locations and fleet sizes. ENHSP produces discrete plans, while the simulator converts them into routes and coordinated execution.

The communication design has two levels. PDDL describes readiness and intended neighbour relationships. Runtime code carries commands, telemetry and shared pace information. Clear separation of these levels makes the planning contribution easier to explain and prevents unsupported claims about decentralised networking or flight safety.

Further work can connect live telemetry to replanning states, express explicit recovery goals, validate return-energy reserves, improve PDDL requirement declarations, calibrate energy and timing models, and test member trajectories under faults and turns. These are extensions for evaluation and implementation, rather than results already established by the current report.

# References

[R1] Reference Doc.pdf, supplied thesis-format example in the repository root. Used for report structure and front-matter organisation.

[R2] Swarm-Planning/Plans/travell/, Swarm-Planning/Plans/Search/, Swarm-Planning/Plans/vformation/, Swarm-Planning/Plans/gridformation/, Swarm-Planning/Plans/3dgridformation/, Swarm-Planning/Plans/parabolaformation/, Swarm-Planning/Plans/figure8formation/, Swarm-Planning/Plans/starformation/, Swarm-Planning/Plans/planets/ and Swarm-Planning/Plans/Inspect/: current domain and problem files, reproduced in Appendix A.

[R3] services/plan_service.py and engine/pddl_planner.py: plan discovery, instance preparation, planner invocation, output parsing and saved runs.

[R4] engine/pddl_problem.py, engine/search_problem.py, engine/figure8_problem.py, engine/star_problem.py, engine/planets_problem.py and engine/road_inspection.py: geographic preparation and formation geometry.

[R5] engine/protocol.py, engine/drone_link.py, services/thread_backend.py and engine/train_keeping.py: transport, messages and coordination.

[R6] engine/mavlink_mission.py, engine/mavlink_interpreter.py, services/mavlink_flight_service.py and scripts/pddl_to_mavlink.py: flight mission construction and execution integration.

[R7] README.md and Swarm-Planning/Plans/PLANS_STATUS.txt: repository overview and historical status context. Historical counts are not new measurements.

# Appendix A. Complete PDDL source listings

The following listings reproduce all twenty current template files verbatim. They are domain and problem definitions, not newly solved action traces. Generated run instances are excluded because they depend on individual map selections and are stored separately. Each mission's domain contains the exact action parameters, preconditions and effects; its problem contains the initial values and goal. The explanations in Chapter 3 should be read together with these definitions.
''')
for folder, title, _ in missions:
    add(f'## A.{missions.index((folder,title,_))+1} {title}')
    paths = [ROOT/'Swarm-Planning'/'Plans'/folder/'domain.pddl']
    paths += sorted(p for p in (ROOT/'Swarm-Planning'/'Plans'/folder).glob('*.pddl') if p.name != 'domain.pddl')
    for path in paths:
        add(f'### {path.relative_to(ROOT).as_posix()}\n\n```pddl\n{path.read_text(encoding="utf-8").rstrip()}\n```')

markdown = ''.join(parts)
(OUT/'PDDL_Planning_Report.md').write_text(markdown, encoding='utf-8')

from docx import Document
from docx.shared import Inches, Pt
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
doc = Document()
section = doc.sections[0]
section.page_height, section.page_width = Inches(11.69), Inches(8.27)
section.top_margin = section.bottom_margin = Inches(.8)
section.left_margin, section.right_margin = Inches(1), Inches(.8)
normal = doc.styles['Normal']
normal.font.name, normal.font.size = 'Times New Roman', Pt(11)
normal.paragraph_format.line_spacing = 1.15
normal.paragraph_format.space_after = Pt(6)
code_style = doc.styles.add_style('PDDL Code', 1)
code_style.font.name, code_style.font.size = 'Consolas', Pt(7)
code_style.paragraph_format.line_spacing = 1
code_style.paragraph_format.space_after = Pt(0)
footer = section.footer.paragraphs[0]
footer.alignment = 1
field = OxmlElement('w:fldSimple'); field.set(qn('w:instr'), 'PAGE'); footer._p.append(field)
lines = markdown.splitlines()
in_code = False
i = 0
while i < len(lines):
    line = lines[i]
    if line.startswith('```'):
        in_code = not in_code; i += 1; continue
    if in_code:
        doc.add_paragraph(line, 'PDDL Code'); i += 1; continue
    if line.startswith('|'):
        rows = []
        while i < len(lines) and lines[i].startswith('|'):
            cols = [v.strip() for v in lines[i].strip('|').split('|')]
            if not all(re.fullmatch(r'[-: ]+', v) for v in cols): rows.append(cols)
            i += 1
        table = doc.add_table(rows=0, cols=len(rows[0])); table.style = 'Light Shading Accent 1'
        for row in rows:
            for cell, value in zip(table.add_row().cells, row): cell.text = value
        continue
    if line.startswith('# '):
        if doc.paragraphs: doc.add_page_break()
        doc.add_heading(line[2:], 0 if line[2:].startswith('PDDL-Based') else 1)
    elif line.startswith('## '): doc.add_heading(line[3:], 2)
    elif line.startswith('### '): doc.add_heading(line[4:], 3)
    elif line.strip():
        doc.add_paragraph(line.replace('**',''), 'List Bullet' if line.startswith('- ') else 'Normal')
    i += 1
doc.save(OUT/'PDDL_Planning_Report.docx')

# PDF is produced from the same content, avoiding a dependency on installed Word.
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Preformatted, Table, TableStyle, PageBreak
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.enums import TA_JUSTIFY
styles = getSampleStyleSheet()
styles.add(ParagraphStyle(name='BodyReport', fontName='Times-Roman', fontSize=10, leading=13, spaceAfter=6, alignment=TA_JUSTIFY))
styles.add(ParagraphStyle(name='Source', fontName='Courier', fontSize=6.1, leading=7.2, spaceAfter=0))
story=[]; in_code=False; i=0
while i<len(lines):
    line=lines[i]
    if line.startswith('```'): in_code=not in_code; i+=1; continue
    if in_code:
        # Keep verbatim text in Word/Markdown; wrap long PDF lines for page fit.
        import textwrap
        wrapped='\n'.join(textwrap.wrap(line, width=112, replace_whitespace=False, drop_whitespace=False) or [' '])
        story.append(Preformatted(wrapped,styles['Source'])); i+=1; continue
    if line.startswith('|'):
        rows=[]
        while i<len(lines) and lines[i].startswith('|'):
            cols=[v.strip() for v in lines[i].strip('|').split('|')]
            if not all(re.fullmatch(r'[-: ]+', v) for v in cols): rows.append([Paragraph(escape(v),styles['BodyReport']) for v in cols])
            i+=1
        table=Table(rows,colWidths=[480/len(rows[0])]*len(rows[0]),repeatRows=1,hAlign='LEFT')
        table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e4edf7')),('GRID',(0,0),(-1,-1),.25,colors.lightgrey),('VALIGN',(0,0),(-1,-1),'TOP')]))
        story.extend([table,Spacer(1,9)]); continue
    if line.startswith('# '):
        if story: story.append(PageBreak())
        story.append(Paragraph(escape(line[2:]),styles['Title']))
    elif line.startswith('## '): story.append(Paragraph(escape(line[3:]),styles['Heading2']))
    elif line.startswith('### '): story.append(Paragraph(escape(line[4:]),styles['Heading3']))
    elif line.strip(): story.append(Paragraph(escape(line.replace('**','')),styles['BodyReport']))
    i+=1
def page_number(canvas, document):
    canvas.setFont('Times-Roman',9); canvas.drawCentredString(297,25,str(document.page))
SimpleDocTemplate(str(OUT/'PDDL_Planning_Report.pdf'), pagesize=(595.28,841.89),rightMargin=48,leftMargin=66,topMargin=48,bottomMargin=44).build(story,onFirstPage=page_number,onLaterPages=page_number)
print('Created Markdown, Word and PDF reports.')

