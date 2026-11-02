# Report V2 — CS25M053

Title: PDDL-Based Mission Planning for a Drone Swarm Simulator  
Submission: December 2026

- `main.pdf`: revised compiled report.
- `main.tex`: editable LaTeX source.
- `figures/`: institute logo and ten replaceable sample mission-output PDFs.
- `Report_V2_Overleaf.zip`: source package ready to upload to Overleaf.
- `CHANGES.md`: changes and editorial assumptions.

## Overleaf

Upload `Report_V2_Overleaf.zip`, set the main document to `main.tex`, and choose **XeLaTeX**. Recompile until the contents and figure/table lists update. The report is self-contained; it does not require PDDL source files or an appendix.

The planning and communication diagrams are editable TikZ drawings in `main.tex`.
The ten output figures are clearly labelled **illustrations, not recorded simulator screenshots**. Replace the corresponding PDF under `figures/` with your screenshot converted to PDF, retaining its filename, or change its `\includegraphics` path to a PNG/JPG. After replacement, update its caption to describe the actual result and remove the illustration-only wording.

| Mission | Replaceable file |
| --- | --- |
| Point-to-point | `figures/travel_sample.pdf` |
| Search | `figures/search_sample.pdf` |
| V formation | `figures/v_sample.pdf` |
| Grid | `figures/grid_sample.pdf` |
| Cube | `figures/cube_sample.pdf` |
| Parabola | `figures/parabola_sample.pdf` |
| Figure-eight | `figures/figure8_sample.pdf` |
| Star show | `figures/star_sample.pdf` |
| Planetary show | `figures/planets_sample.pdf` |
| Road inspection | `figures/inspect_sample.pdf` |

The original report in `docs/MTP_Report_CS25M053` is preserved.
