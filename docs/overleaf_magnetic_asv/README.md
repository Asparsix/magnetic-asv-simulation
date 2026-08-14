# Overleaf book: Magnetic ASV + full ROS 2 architecture

## Upload
1. Create a new Overleaf project.
2. Upload **this entire folder** (`main.tex`, `chapters/`, …).
3. Set main document to `main.tex`.
4. Compiler: **pdfLaTeX**.

## Companion Markdown (long-form narrative)
For a detailed ROS-developer write-up (strategy, step-by-step workflow, math,
topic list, FSM, failure modes) open:

`docs/MAGNETIC_ASV_ROS2_DEVELOPER_MANUAL.md`

That file is meant to read as a 15–20+ page engineering manual in GitHub /
VS Code / Cursor. This Overleaf project is the printable chapter book that
embeds the exhaustive node catalog.

## What you get (Overleaf PDF)
A **book-format** manual:

| Chapter | Content |
|---------|---------|
| Intro | Document map, packages, namespace rules (3 ASV) |
| Pipelines | Control / sensing / belief / mission / viz data-flows |
| Algorithms | Dipole, Bayes, MI, inland-safe spiral, LOS |
| Mission FSM | Modes, transitions, verify handshake, HOLD semantics |
| **ROS node catalog** | **Every message, bridge, and node** |
| Launches & ops | Phase launches, YAML, inspect commands |
| Acceptance | CONFIRMED vs offline scoring |

The catalog chapter is generated from
`version_3/docs/ros2_node_catalog.md`.

## Regenerate the catalog chapter (optional)
```bash
python3 docs/overleaf_magnetic_asv/md_to_tex.py
```

## Expected length
With the full node catalog, this typically compiles to **25–40+ pages**.
