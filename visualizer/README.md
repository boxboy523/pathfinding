# Path-quality visualizer

```sh
nix develop --command cargo run --release -p visualizer
```

A Wayland desktop is required by the current window configuration. Use release
mode for the 256x256 preview, since topology construction is intentionally not
optimized by the visualizer.

## Controls

- **1**: simple staircase / L / edge-wall room map.
- **2**: complex connected map with walls and edge-only barriers.
- **3**: three disconnected staircase regions.
- **4** (default): 256x256 RTS preview, 16 rooms with obstacle islands. A spiral
  route and two shortcuts connect every room, but several adjacent room pairs
  have no direct doorway and require a detour.
- **Shift + left click**: select a walkable start cell (cyan).
- **Right click**: select a walkable goal cell (green).
- **Left click**: place a one-cell building, except on start/goal.
- **Middle click**: delete a building placed in this session, not base terrain.
- **R**: toggle wall repulsion (0 / 0.5) for both flow implementations.
- **A**: toggle the optional magenta A* reference (initially hidden).
- **B**: toggle Block outlines.
- **F**: toggle Block/Portal flow direction vectors.

Each map is fitted to the window. Cell arrows/labels are hidden when cells are
too small to read. Start/goal/terrain/repulsion changes recompute the displayed
paths; other frames reuse them. Switching maps resets buildings and endpoints.

## Paths and status

- **Cyan**: point-agent trajectory following the actual Block/Portal flow directions.
- **Yellow**: point-agent trajectory following a non-hierarchical flow field built by
  full-grid Dijkstra integration and downhill neighbor selection.
- **Magenta** (A): optional eight-neighbor A* reference path.
- **Cyan/yellow moving dots**: playback along the two flow trajectories, not
  separate steering simulations.
- **Red endpoint ring**: that flow trajectory did not reach the goal.

Grid flow and A* use 1000/1414 neighbor costs, blocked-edge checks, and no diagonal
corner cutting. Grid flow reuses the same repulsion function and options as
Block/Portal flow. Both fields are followed by the exact same 0.08-cell tracer.

The header displays status and traveled length in cell units. Block/Grid length
ratio is shown only when both reach the goal and Grid length is nonzero. Grid
directions are discrete and flow trajectories are not guaranteed to equal A*
paths. A* is grid-optimal, not continuous-space optimal. If a flow fails, its
reported length is only the partial trajectory.

Flow following uses piecewise-constant sampling of the current cell vector and
0.08-cell Euler steps. Entering the goal cell finishes at the goal center.
Crossings into walls or blocked edges stop with `BLOCKED`; motion is not silently
corrected with A* or wall sliding. Quantized revisits report `LOOP`; zero/invalid
vectors report a stall. A step cap prevents an unbounded trace. Loop detection
is diagnostic and quantized, not a formal proof of a dynamical cycle.

This is a zero-radius point-agent diagnostic, not a finite-radius unit collision
or crowd/formation simulation. Different sampling, interpolation, steering, or
agent radius can change trajectories. Compare repulsion on/off and multiple
endpoints rather than interpreting one successful trace as a general guarantee.
