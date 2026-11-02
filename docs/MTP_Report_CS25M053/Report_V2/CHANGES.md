# Changes against the original report

1. Updated the title throughout, including PDF metadata.
2. Replaced the cover emblem with a PDDL inputs → ENHSP → mission routes → swarm diagram.
3. Set report/submission dates to December 2026. Removed dated run identifiers from the narrative; retained historical publication years in References.
4. Rewrote acknowledgements in a formal first-person style tailored to this project, retaining the existing advisor names. No additional acknowledgement reference image was attached.
5. Replaced the abstract's solved/request ratios with dynamic swarm sizing, mission-specific ranges and demonstrated capacity limits. Clearly distinguished fixed-size parabola/inspection implementations and demonstrated sizes from general upper bounds.
6. Reworked contents indentation and spacing.
7. Aligned table-list entries and page numbers.
8. Added short one-line figure-list titles and fitted the figure list to one page.
9. Removed inline bracketed citation markers and optional firmware/emulation claims. Kept an unnumbered reference list.
10. Replaced third-person authorship descriptions with this work/I, and clarified the student's GUI contribution.
11. Removed the blanket attribution of the GUI to other team members. Focused the scope on PDDL and its integration.
12. Removed the PDDL line-count claim from the contribution summary; kept numerical technical evidence where it describes actual model constraints or results.
13. Improved paragraph spacing and figure/table sizing.
14. Rewrote section 2.3 as cohesive paragraphs and prevented short orphaned endings.
15. Explained section 3.2 as ordered steps, retaining the pipeline diagram.
16. Explained the shared PDDL vocabulary in plain language and made the symbol table readable; kept command tokens intact in the run-time command table.
17. Simplified the modelling design patterns and common mission life cycle.
18. Added an ENHSP result interpretation section and a consistent, explicitly illustrative travel output. Explained action order, solver statistics and no-plan versus timeout.
19. Retained existing mission diagrams and explanations; added mission progression, per-mission communication arrows, neighbour counts, selected PDDL facts and completion conditions for all ten missions.
20. Retained Chapter 6's communication material and cross-connected it with Chapter 5's mission-specific material.
21. Rewrote section 6.4 with simple ordered explanations and a clearer local-control diagram.
22. Interpreted the unspecified “this part” near requests 21–23 as the communication/control and fault-handling material; simplified those diagrams and the battery diagram.
23. Added simple GPS, missing-message, missing-neighbour and low-battery fault examples without claiming new implemented fault mechanisms.
24. Added fixed cruise-height and no automatic replacement-plan limitations, plus clear swarm-size, communication, energy and simulation limits.
25. Replaced the third future-work point with a local planner for every drone, distinguishing it from the current local speed controller.
26. Removed the entire full-source appendix and long full-plan transcript. Placed selected PDDL explanations beside their missions, with ten sample output PDFs for replacement in Overleaf.

## Validation

Compiled from the revised LaTeX using Tectonic's XeTeX engine. Checked the cover, front matter and representative mission/communication figures visually. Checked for missing references, overfull text boxes, missing image inputs and unintended appendix/citation remnants. Selected new predicates and mission-size constraints were compared against the local PDDL domains and Python services. Existing measurements in the results chapter were retained from the original report; the simulator experiments were not rerun for this editorial revision.
