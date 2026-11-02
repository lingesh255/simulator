from pathlib import Path
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'docs/MTP_Report_CS25M053'
OUT = BASE / 'Report_V4'
OUT.mkdir(exist_ok=True)
shutil.copytree(BASE / 'Report_V3/figures', OUT / 'figures', dirs_exist_ok=True)
s = (BASE / 'Report_V3/main.tex').read_text(encoding='utf-8')

def replace(old, new):
    global s
    if old not in s:
        raise ValueError('Missing text: ' + old[:100])
    s = s.replace(old, new)

def between(start, end, content):
    global s
    a = s.index(start)
    b = s.index(end, a)
    s = s[:a] + content + '\n\n' + s[b:]

replace('Report V3.', 'Report V4. Updated against the local plan files on 30 October 2026.')
replace('Every plan works with a dynamic number of drones: the operator chooses how many drones to use,\nand the template is expanded to exactly that number before planning.', 'The original swarm templates support a selected number of drones within their limits.\nFormula-4 and Formation-Circle use four drones, battery-aware travel uses one, and the separate\ntask-allocation problem supplies four candidates.')
replace('verification of every plan through the real planning pipeline', 'verification of the simulator plans through the real planning pipeline')
replace('Ten mission plans were developed', 'Thirteen simulator mission plans and one separate task-allocation model were developed')
replace('planetary orbit show and road inspection. Every plan shares one vocabulary of drone states,',
        'planetary orbit show, road inspection, battery-aware travel, Formula-4 and a rotating circle.\nThe separate task-allocation model describes how a standby drone can take over a task.\nThe original plans share a vocabulary of drone states,')
replace('  \\item ten mission plans (Table~\\ref{tab:inventory}), about 6\\,700 lines of PDDL in total,\n        written on one shared vocabulary of predicates and numeric fluents, each of which adapts',
        '  \\item thirteen simulator mission plans (Table~\\ref{tab:inventory}) and a separate\n        task-allocation model, using shared names where the same idea applies; the original\n        swarm templates adapt')
for old,new in [('all ten mission plans','the mission plans'), ('all ten plans','the thirteen simulator plans'),
                ('Inventory of the ten mission plans','Inventory of the thirteen simulator mission plans'),
                ('To make the ten plans easy to read together','To make the original plans easy to read together'),
                ('each of the ten plans in the same way','the plans using the following points'),
                ('compares the ten plans side by side','compares the thirteen simulator plans side by side')]:
    replace(old,new)

# Keep the existing table style while giving the extra rows room to wrap.
replace(r'\begin{tabularx}{\textwidth}{@{}l>{\ttfamily\footnotesize}P{0.36\textwidth}lY@{}}',
        r'\begin{tabularx}{\textwidth}{@{}P{0.22\textwidth}>{\ttfamily\scriptsize}P{0.34\textwidth}P{0.23\textwidth}Y@{}}')
row = r'\code{Inspect}           & road-inspection-drone-mission     & road\_inspection    & 1 \\'
replace(row, row + r'''
\code{BatteryAware} & battery-aware-drone-mission & battery\_aware & 1 \\
\code{Formula-4} & formula4-drone-mission & formula4\_formation & 4 \\
\code{Formation-Circle} & circle-formation-drone-mission & circle\_formation & 4 \\''')
replace('\\section{Run Records}', r'''The folder \code{Task-Allocation} contains an additional PDDL model. Its domain,
\code{task-allocation}, is not mapped to a simulator plan kind. It is described separately in
Section~\ref{sec:plan-task}; it does not yet run as a live simulator mission.

\section{Run Records}''')

between('A stationary light show around one picked point.', '\\glancebox{At a glance: star drone show}', r'''The operator picks a source for take-off and a destination for the show. Drones leave their
source pads one by one, with a 30\,m gap along the shared corridor, and take their places on a
circle around the destination. For each lap they form a star, return to the circle, spin the
circle once, form the star again, spin the star and return to the circle. They then return one
by one and land on their own source pads (Figure~\ref{fig:shows}a). Six or seven drones make a
3-pointed star, eight or nine a 4-pointed star, and ten or more a 5-pointed star. The destination
must be far enough from the source to leave room for the pads, corridor and show.''')
replace('Map input & one centre point; number of laps and show speed',
        'Map input & source and show destination; number of laps and show speed')
replace('ENHSP plans & the show phases, in order', 'ENHSP plans & departure order, show phases, return order and landing')
replace('form-circle-in-lap \\then spin-circle \\then form-star-again \\then spin-star \\then close-lap] for\nevery lap \\then land-show}',
        'form-circle-in-lap \\then spin-circle \\then form-star-again \\then spin-star \\then close-lap] for\nevery lap \\then end-show \\then return-to-source and land (in order) \\then complete-show}')
replace('climb (each drone) \\then form-circle', 'climb (each drone) \\then fly-to-formation (in order) \\then form-circle')

between('A solar-system show round one picked point', '\\glancebox{At a glance: planetary orbit show}', r'''The operator picks a source for take-off and a destination as the sun. Drones leave the source
one by one, 30\,m apart on the corridor, and join their orbits from the innermost outward.
For drone index $k=0,1,\ldots,N-1$, the nominal orbit radius is $50+10k$\,m: 50 to 120\,m for
eight drones. Neighbouring orbits turn in opposite directions (Figure~\ref{fig:shows}b).
Every drone uses the same angular speed, so the outer drones fly faster. The drones turning
each way stay on one straight line out from the sun. One lap is one full turn for every drone;
both lines meet on the starting line at the end of each lap. Drones return one by one, outermost
first, and land on their own source pads. Moves start and stop smoothly over five clock seconds.
The destination must be at least the outermost orbit radius plus 50\,m from the source.''')
replace('Map input & one centre point (the sun); number of laps',
        'Map input & source and destination (the sun); laps and show speed')
replace('ENHSP plans & entering the orbits and the laps',
        'ENHSP plans & departure order, entering orbits, laps, return and landing')
replace('and then \\code{land-planets}:', 'and then \\code{end-show}, ordered returns, \\code{land} and \\code{complete-planets}:')
replace('climb (each drone) \\then enter-orbits \\then fly-lap (once\nper lap) \\then land-planets}',
        'climb (each drone) \\then fly-to-orbit (in order) \\then enter-orbits \\then fly-lap (once\nper lap) \\then end-show \\then return-to-source and land (in order) \\then complete-planets}')

# Replace displayed action examples with the current domain actions, without comments.
def action(folder, name):
    text = (ROOT / 'Swarm-Planning/Plans' / folder / 'domain.pddl').read_text(encoding='utf-8')
    text = re.sub(r';[^\n]*', '', text)
    start = text.index('(:action ' + name + '\n')
    depth = 0
    for i in range(start, len(text)):
        depth += (text[i] == '(') - (text[i] == ')')
        if depth == 0:
            return '\n'.join(line.rstrip() for line in text[start:i+1].splitlines() if line.strip())
    raise ValueError(name)

for folder, name in [('starformation','form-circle'), ('planets','enter-orbits')]:
    start = s.index('(:action ' + name + '\n')
    end = s.index(r'\end{lstlisting}', start)
    s = s[:start] + action(folder,name) + '\n' + s[end:]

new = r'''
\section{Battery-Aware Travel}\label{sec:plan-battery}
\paragraph{What the mission does}
One drone flies from the source to the destination. The operator also marks five charging
stations before the route is chosen. If the current battery can cover the direct trip and its
margin, the drone flies straight to the destination. Otherwise, the planner chooses a route
through reachable stations. Other selected drones stay on the ground.

\glancebox{At a glance: battery-aware travel}{
Map input & source, destination and five charging stations \\
Drones & 1, the first selected drone \\
Neighbours per drone & none; ground-station link only \\
ENHSP plans & reachable hops and charging stops, using optimal search \\
Special feature & charge only for the next hop plus a 10\,\% margin \\}

\paragraph{Communication}
The drone sends its position and battery state to the ground station. After each station
landing, the ground station sends a charge command. It sends the next flight only after the
required charge is reached. No drone-to-drone link is needed.

\paragraph{Important PDDL}
Each hop includes take-off, climb, flight and descent. Its energy estimate uses the simulated
current draw, battery capacity, flight speed and height, with ten extra seconds for transitions.
At a station, \code{charge-battery} sets the battery to the next hop's energy plus ten percentage
points. The \code{charged-for} fact ties that charge to that specific hop. Take-off is blocked
until charging is complete. A target above 100\,\% is not allowed. The route metric is metres
flown plus one per station landing; an independent shortest-path check checks the chosen route.
The old rule of charging to full with a 25\,\% reserve is no longer used.

\begin{lstlisting}
'''+action('BatteryAware','charge-battery')+r'''
\end{lstlisting}

\paragraph{Action order}
\actionflow{pre-flight-check \then take-off \then fly-hop \then [land-at-station \then
charge-battery \then take-off \then fly-hop] as needed \then land-at-destination \then complete-mission}

\paragraph{Output}
The map shows the chosen route and the stations it uses. The drone lands and charges at each
chosen station, then continues. A planned charging hop can land near 10\,\% battery, so the
ordinary 20\,\% warning is skipped for this mission; the critical 5\,\% safeguard still applies.
If no chain of stations is reachable, the operator must place the stations again.

\section{Formula-4: Dynamic Position Swaps}\label{sec:plan-formula4}
\paragraph{What the mission does}
Four drones fly in two lanes from source to destination. Each lane has a front and a back drone.
The front drone slows and dips to 50\,m. The back drone speeds up, climbs to 60\,m and passes it.
They repeat these swaps along the route and land in their own destination slots.

\glancebox{At a glance: Formula-4}{
Map input & source and destination \\
Drones & 4, the first four selected drones \\
Neighbours per drone & no drone-to-drone messages \\
ENHSP plans & formula checks and each drone's swaps, arrival and landing \\
Special feature & safe passing from a position formula and one shared clock \\}

\paragraph{Communication}
Each drone follows its own timed track. It needs its starting slot and the shared clock from
the ground station. It does not need the current positions of the other drones.

\paragraph{Important PDDL}
Let $x$ be distance along the route, $y$ distance across it and $z$ height. The front/back sign
is $f=+1/-1$ and the left/right sign is $l=+1/-1$. At shared clock time $\tau$,
\[
 x=V\tau+f\frac D2\cos(\omega\tau),\qquad
 y=l\left(\frac W2+f\frac B2\sin(\omega\tau)\right),\qquad
 z=Z_0-fA\sin(\omega\tau).
\]
Here $D=W=20$\,m, $B=6$\,m, $Z_0=55$\,m and $A=5$\,m. The speed $V$ is 6 metres per clock
second. The turn rate gives an even number of swaps, so drones finish in their starting slot
order. In the same lane, the distance is at least
$\min(D,\sqrt{B^2+4A^2})=11.66$\,m; across a line it is at least $W-B=14$\,m.
The \code{check-formula} action checks these bounds and the speed limits before take-off.
These are bounds for the planned formula; actual flight also depends on tracking that formula.

\paragraph{Action order}
\actionflow{check-formula \then take-off (each drone) \then slow-and-dip / speed-up-and-climb
(alternating for each drone) \then arrive \then land}

\paragraph{Output}
The simulator shows repeated passes, with the passing drone above the slower drone. Heights
change throughout the flight. The worker checks each drone's action sequence before sending
the timed tracks. Show speed changes how fast the shared clock runs.

\section{Formation-Circle: A Rotating Vertical Circle}\label{sec:plan-circle}
\paragraph{What the mission does}
Four drones form a vertical circle across the route. They start at the bottom, left, top and
right, at heights of 40, 50, 60 and 50\,m. The circle turns while moving toward the destination.
Each drone moves bottom to left to top to right, so its flight path is a spiral.

\glancebox{At a glance: Formation-Circle}{
Map input & source and destination \\
Drones & 4, the first four selected drones \\
Neighbours per drone & no drone-to-drone messages \\
ENHSP plans & formula checks, joining, quarter turns, leaving and landing \\
Special feature & a rotating vertical circle with separate take-off and landing pads \\}

\paragraph{Communication}
As in Formula-4, each drone uses its own starting place and the shared ground-station clock.
No drone needs to ask a neighbour for its position.

\paragraph{Important PDDL}
With radius $R=10$\,m, centre height $Z_0=50$\,m and starting angle $a_{i0}$, the main flight is
\[
 a_i=a_{i0}+\omega(\tau-T_j),\qquad
 x_i=V\tau,\qquad y_i=R\sin(a_i),\qquad z_i=Z_0-R\cos(a_i).
\]
The starting angles differ by a quarter turn. Neighbours stay $R\sqrt{2}=14.14$\,m apart in
the formula, and opposite drones stay $2R=20$\,m apart. The \code{check-formula} action checks
these gaps, the pad spacing, speed and lowest height. The circle makes a whole number of turns.

The top and bottom drones cannot take off from the same ground point. They therefore start on
separate pads in a diamond. The diamond closes into the circle over the first 40\,m and opens
again over the last 40\,m. Each drone then lands on its own destination pad. These joins also
keep the planned separation; actual tracking can give slightly different measured gaps.

\paragraph{Action order}
\actionflow{check-formula \then take-off (each drone) \then join-circle \then turn-quarter
(four per full turn, for each drone) \then leave-circle \then land}

\paragraph{Output}
The simulator shows a circle that moves forward while turning, with each drone changing height
and side. Long timed tracks are split into smaller UDP messages and joined again at the drone.
Both formula plans have a 9000-point track limit, so their maximum route length depends on how
many track points the movement needs.

\section{Task Allocation with a Standby Drone}\label{sec:plan-task}
\paragraph{What the model does}
Four capable drones start at the source. One drone takes responsibility and starts the task.
The others wait. If the active drone develops a recoverable problem, it reserves a capable
standby drone. The replacement takes off; while both drones are airborne and their radios work,
the old drone shares the mission data and transfers the task. The replacement can repeat this
process if it later develops a problem.

\glancebox{At a glance: task allocation model}{
Input & source, destination, capable drones and any known failure facts \\
Drones & 4 in the supplied problem; one active owner at a time \\
Communication & working radios on both drones during the airborne handoff \\
Planner models & task ownership, standby launch, data transfer and completion \\
Current status & separate PDDL model; simulator execution support is still needed \\}

\paragraph{Important PDDL}
The \code{responsible} and \code{mission-data} facts identify the active owner. Reserving a
standby removes it from the available pool. The handoff removes the old drone's mission data
and marks it retired. Arrival completes the task and sets \code{mission-finished}; every action
requires the mission to remain unfinished, so no replacement can launch after completion.

\begin{lstlisting}
'''+action('Task-Allocation','share-data-and-transfer-task')+r'''
\end{lstlisting}

\paragraph{Action order}
\actionflow{start-mission \then [detect-problem \then notify-capable-drone \then
launch-replacement \then share-data-and-transfer-task] if needed \then travel-to-destination}

\paragraph{Limits}
This is a classical planning model. Known \code{will-fail} facts can make the planner choose a
different initial drone or skip directly to a healthy replacement. Unexpected failures need a
new state and replanning. A crash or total radio loss cannot use this handoff, because the old
drone must remain airborne and able to transmit. Its emergency landing is outside the model.
The simulator still needs handlers for these actions before it can fly this behaviour.
'''
replace('\\section{Summary of the Plans}', new+'\n\\section{Summary of the Plans}')
replace('Inspect    & 1 & none & road segments & OSRM road geometry \\\\', r'''Inspect    & 1 & none & road segments & OSRM road geometry \\
Battery-aware & 1 & none & hops and charges & next-hop charge plus margin \\
Formula-4 & 4 & none & swaps per drone & shared-clock passing \\
Circle & 4 & none & quarter turns & vertical rotating circle \\''')
replace('Communication appears at two levels in the simulator.',
        'Formula-4 and Formation-Circle use a shared clock without drone-to-drone position messages.\nBattery-aware travel uses only a ground-station link. The separate task-allocation model needs\na working radio on both drones for a handoff. In the other plans, communication appears at two levels in the simulator.')
replace('Every drone in every plan also has a ground-station link',
        'Every drone in the simulator plans also has a ground-station link')

# Mark older measurements honestly and replace results invalidated by the changed shows.
replace('The planning layer was verified in two ways.', 'The planning layer was verified in two ways.') if 'The planning layer was verified in two ways.' in s else None
replace('\\section{In-App Planning Requests}', r'''The earlier measurements below are kept as results for the earlier plan versions. They are
not new measurements of the updated plans. Results for the changed shows and new simulator
plans are taken from the local plan status notes dated 29--30 October 2026. This report update
checks the model and source files; it does not repeat those flight experiments.

\section{In-App Planning Requests}''')
replace('Star show & 6 drones $\\times$ 1 lap 0.3\\,s; 40 drones $\\times$ 5 laps 0.6\\,s. In flight (10\ndrones, 2 laps), worst drone 0.7--3.2\\,m off its slot while shapes were held, closest pair\n9.3\\,m, every drone landed on its own pad.',
        'Star show & Updated source-to-show model: 6 drones 0.9\\,s; 40 drones with 5 laps 2.3\\,s.\nSampled timed tracks for 6--40 drones: closest pair 9.1\\,m; all tracks end on their source pads.')
replace('Planets & 2--8 drones $\\times$ 3 laps, 0.2--0.3\\,s. With 8 drones every drone stayed within 0.4\\,m\nof its orbit and turned the correct direction and number of turns.',
        'Planets & Updated 2--8 drone model, 1--2 laps: 0.5--0.7\\,s. Sampled timed tracks: about\n10.0\\,m minimum gap and aligned spokes. The status notes do not record a completed drone-thread\nflight after this change; the attempted run received no UDP heartbeats.')
replace('\\section{A Complete Sample Plan}', r'''\section{Results for the New Simulator Plans}
The local status notes record the following checks. Planning results, sampled tracks and
drone-thread flight results are different checks and are reported separately.
\begin{table}[H]
\caption{Recorded checks for the new simulator plans}\label{tab:newresults}
\small
\begin{tabularx}{\textwidth}{@{}P{0.2\textwidth}Y@{}}
\toprule
\textbf{Plan} & \textbf{Recorded result} \\
\midrule
Battery-aware & Latest partial-charge example: 2.4\,km, 1000\,mAh drone; source to C1 to
destination. At C1 the charge rose from 39\,\% to 94\,\%; arrival was at 11.4\,\%, with no
ordinary low-battery prompt. An 80\,km route agreed with the independent shortest-path check. \\
Formula-4 & Recorded ENHSP checks: 0.5--0.7\,s up to 8\,km. A 500\,m flight made six swaps per
lane with a closest pair of 11.6\,m and heights of 50--60\,m. A 1\,km flight with different
drone speed limits also landed every drone on its own slot. \\
Formation-Circle & Recorded ENHSP checks: 0.4--0.5\,s up to 6\,km. Repeated 500\,m flights
measured 13.1--13.2\,m closest separation; a 1\,km flight measured 13.5\,m. Every drone landed
on its own pad. One earlier flight stalled; later successful flights do not remove that open concern. \\
\bottomrule
\end{tabularx}
\end{table}
Negative formula checks rejected unsafe gaps, excessive turning rates and an unsafe circle
height. After message splitting was added, the notes record completed 3 and 10\,km circle
flights and a 6\,km Formula-4 flight. These recorded examples do not prove safety for all inputs.
Task allocation has no recorded live simulator flight, so it is not counted in this table.

\section{A Complete Sample Plan}''')
between('  \\item \\textbf{Flying at a constant height.}', '  \\item \\textbf{No replanning when a plan fails.}', r'''  \item \textbf{Height and terrain have limits.} Formula-4 and Formation-Circle change height
        during flight. A separate route helper also samples artificial hills and can raise a
        route or add a bypass. This is computed terrain, not measured ground data. It does not
        prove that every formation member clears real hills or buildings.''')
replace('The plan is made once, before take-off.',
        'The simulator plan is made once, before take-off. The separate task-allocation model\n        describes handoffs, but it is not yet connected to live simulator replanning.')
replace('Battery and energy in PDDL use a 0--100 scale, not real',
        'Most earlier plans use a 0--100 scale, not real')
replace('therefore two separate safeguards rather than one consistent model.',
        'therefore separate safeguards in those plans. Battery-aware travel instead estimates\n        each hop from the simulated current draw and battery capacity, with a 10\\,\\% margin.\n        This still needs calibration before use on physical drones.')
replace('This thesis presented the PDDL planning layer of a drone swarm simulator. Ten mission plans were\nbuilt on one shared vocabulary, covering travel, area search, four rigid formations, a\nfigure-eight train, two shape-changing shows and road inspection, and every plan adapts to the\nnumber of drones the operator selects.',
        'This thesis presented the PDDL planning layer of a drone swarm simulator. Thirteen simulator\nmission plans cover travel, area search, rigid formations, a figure-eight train, two shows,\nroad inspection, battery-aware travel and two moving four-drone formations. The swarm templates\nadapt to selected counts within their limits; the new formula formations use four drones and\nbattery-aware travel uses one. A separate task-allocation model describes airborne task transfer\nto a standby drone and still needs simulator execution support.')
replace('  \\item \\textbf{Online replanning.}',
        '  \\item \\textbf{Connect task allocation to the simulator.} Add execution support for standby\n        launch and airborne data transfer, update the state after an unexpected fault, and test\n        repeated handoffs and cases where no healthy standby remains.\n  \\item \\textbf{Online replanning.}')

# Summary of new scope in the abstract, without changing the original report styling.
replace('Inter-drone communication is modelled in two layers.',
        'Battery-aware travel adds charging stops with a next-hop charge target. Formula-4 and the\nrotating circle change positions and heights using one shared clock, without exchanging\nneighbour positions. Task allocation is a separate model and is not yet a live simulator feature.\n\nInter-drone communication in the original formations is modelled in two layers.')
# The current code includes Task-Allocation integration; its README is older.
s = s.replace('Thirteen simulator mission plans and one separate task-allocation model', 'Fourteen simulator mission plans')
s = s.replace('and the separate\ntask-allocation plan uses', 'and the\ntask-allocation plan uses')
s = s.replace('thirteen simulator mission plans', 'fourteen simulator mission plans')
s = s.replace('thirteen simulator plans', 'fourteen simulator plans')
s = s.replace('Thirteen simulator\nmission plans', 'Fourteen simulator\nmission plans')
s = s.replace('and a separate\n        task-allocation model, using', 'including task allocation, using')
s = s.replace('The separate task-allocation model describes how a standby drone can take over a task.', 'Task allocation lets a standby drone take over after an active drone reports a problem.')
s = s.replace('task-allocation problem supplies four candidates.', 'task-allocation plan uses four candidates.')
s = s.replace('Task allocation is a separate model and is not yet a live simulator feature.', 'Task allocation also has a runtime controller for launching a standby and transferring task data.')
between('The folder \\code{Task-Allocation} contains an additional PDDL model.', '\\section{Run Records}', r'''The folder \code{Task-Allocation} uses domain \code{task-allocation} and kind
\code{task\_allocation}. It needs exactly four selected drones and uses the drone-thread backend.
Its current code is described in Section~\ref{sec:plan-task}; its older README does not yet
describe this simulator support.''')
s = s.replace(r'\code{Formation-Circle} & circle-formation-drone-mission & circle\_formation & 4 \\', r'''\code{Formation-Circle} & circle-formation-drone-mission & circle\_formation & 4 \\
\code{Task-Allocation} & task-allocation & task\_allocation & 4 \\''')
s = s.replace('\\section{Task Allocation with a Standby Drone}', '\\section{Task Allocation with a Standby Drone}')
s = s.replace('Drones & 4 in the supplied problem; one active owner at a time', 'Drones & exactly 4 selected; one active owner at a time')
s = s.replace('Input & source, destination, capable drones and any known failure facts', 'Map input & source and destination, at least 30\\,m apart')
s = s.replace('Current status & separate PDDL model; simulator execution support is still needed', 'Current status & PDDL plan plus a runtime controller in the drone-thread backend')
between('\\paragraph{Limits}\nThis is a classical planning model.', '\\section{Summary of the Plans}', r'''\paragraph{Simulator execution}
The plan service checks the four drone profiles, removes capability facts for drones that
cannot fly the route and asks ENHSP to choose an initial owner. All four drone threads start
at the source, but only the owner receives a flight command. The runtime controller watches
faults, GPS state and low battery. It picks an available standby with enough charge to fly
the whole route from the source. The standby climbs above 1\,m, receives the task data through
the ground station and reports that data back in its state. Only then does it become the owner;
the old drone receives a landing command. A failed or timed-out standby is retired and another
candidate is tried. Arrival closes allocation, so no further drone launches.

\paragraph{Limits}
The PDDL file describes a handoff with two airborne drones and working radios. Known
\code{will-fail} facts can let classical planning choose a healthy drone directly. Unexpected
faults are handled by the runtime controller; it does not rerun ENHSP for every handoff.
The controller relays stored task data rather than using a direct drone-to-drone transfer.
For communication loss or an unavailable drone, it can use its cached copy. This is a wider
runtime behaviour than the PDDL handoff alone describes. Emergency landing and actual fault
recovery still depend on the flight backend. No successful flight measurements for this new
task-allocation code were found in the checked status notes.
''')
s = s.replace('Circle & 4 & none & quarter turns & vertical rotating circle \\\\', r'''Circle & 4 & none & quarter turns & vertical rotating circle \\
Task allocation & 4 & handoff pair & initial owner and task completion & runtime standby replacement \\''')
s = s.replace('The separate task-allocation model needs\na working radio on both drones for a handoff.', 'The task-allocation PDDL model needs\nworking radios on both drones for a handoff; runtime data is relayed through the ground station.')
s = s.replace('Task allocation has no recorded live simulator flight, so it is not counted in this table.', 'Task allocation is connected in the source code, but has no recorded successful flight result\nin the checked notes, so no flight measurement is given for it.')
s = s.replace('The simulator plan is made once, before take-off. The separate task-allocation model\n        describes handoffs, but it is not yet connected to live simulator replanning.', 'The symbolic plan is made before take-off. Task allocation can replace a failed owner\n        through its runtime controller, but that controller does not run ENHSP again.')
s = s.replace('A separate task-allocation model describes airborne task transfer\nto a standby drone and still needs simulator execution support.', 'Task allocation adds a PDDL ownership model and a runtime controller that launches a standby\nand relays mission data after a fault. Its runtime handling is separate from symbolic replanning.')
s = s.replace('\\textbf{Connect task allocation to the simulator.} Add execution support for standby\n        launch and airborne data transfer, update the state after an unexpected fault, and test\n        repeated handoffs and cases where no healthy standby remains.', '\\textbf{Verify task allocation end to end.} Test repeated handoffs, failed replacements,\n        missing radio messages and cases where no healthy standby remains. Check that the\n        runtime state follows the PDDL ownership rules.')
s = s.replace('\\paragraph{Action order}\n\\actionflow', '\\paragraph{Action order} The actions follow this order:\n\\actionflow')
(OUT / 'main.tex').write_text(s, encoding='utf-8')
(OUT / 'CHANGES.md').write_text('''# Report V4

Updated from Report_V3/main.tex against the local plans and simulator source on 30 October 2026.
The original document class, fonts, margins, colours, front matter and chapter order are retained.

- Added Battery-Aware, Formula-4 and Formation-Circle mission sections.
- Added Task-Allocation, including current runtime support and the differences from its PDDL model.
- Updated star and planetary map inputs, departure/return order and planetary motion.
- Updated counts, inventory, summary, abstract, conclusion and affected limitations.
- Kept earlier measurements as historical results; used dated local notes for newer results.
- Preserved existing screenshot placeholders; no simulator screenshots or flight results were invented.

Sources: Swarm-Planning/Plans/*/{domain.pddl,problem.pddl}, Task-Allocation/README.md,
PLANS_STATUS.txt, services/plan_service.py, engine/battery_mission.py and engine/terrain.py.
''', encoding='utf-8')
print(OUT)
