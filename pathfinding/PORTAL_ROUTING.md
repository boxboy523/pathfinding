# Block / Portal routing

`build_flow_field` uses goal-specific reverse Dijkstra over shared boundary
Portals, then selects one shared exit per rectangular Block. There is no Chunk
layer, Block merging, convex hull closure, persistent weighted graph, or
pairwise-distance cache.

## Topology and ownership

- `Map::blocks` and `Map::portals` are SlotMaps with distinct generational
  `BlockId` and `PortalId` keys, exported at crate root.
- Cells carry `BlockId`. Each Block holds incident `PortalId`s; the Portal objects
  themselves remain exclusively Map-owned.
- A Portal stores its two Block IDs and a canonical boundary cell interval on the
  left/top Block's side. `side_from(block)` derives the appropriate cell-side
  interval and crossing direction without duplicate opposite-side objects.
- All sides of new Blocks are scanned. When both incident Blocks are new, only
  the canonical left/top Block creates the opening. No coordinate matching table
  or separate Node object is needed.
- `MAX_ASPECT_RATIO` in `block.rs` limits the longer Block side to twice the
  shorter side. Expansion suppresses growth that would exceed this bound. If two
  opposite steps would overshoot but one is legal, one step is retained.

## Terrain changes

Building addition/removal compares old and new obstacle masks to identify affected
Blocks. Changed edges invalidate both incident Blocks. Only those Blocks and their
incident Portals are removed, including references on surviving neighboring Blocks.
The invalidated cells are repartitioned, then new boundary Portals are registered.
Unaffected Block geometry, keys, and Portal objects remain unchanged.

Removing a building still recomposes obstacle masks from base terrain and surviving
buildings, but applies only the difference to topology. Newly opened cells receive
new Blocks; surrounding existing Blocks are not coalesced. Local edits need not
reproduce a fresh full-map partition.

Removed keys remain invalid even after slot reuse. Use `.get(id)` to check retained
keys: stale keys return `None`; indexing a stale key panics. Keys belong to their
originating Map, not another Map. Successful terrain changes increment generation
and invalidate the whole goal-specific FlowField, even if most topology keys survive.

## Goal-specific routes

Portal IDs are graph node IDs directly. Each Block implicitly connects all its
Portals. Edge weights are distances between fixed Portal representatives, computed
on demand during relaxation. Representatives use the integer interval midpoint,
shifted half a cell onto the actual boundary.

The goal is a virtual source: its Block's Portals are seeded with goal-to-Portal
costs, without constructing a physical goal node. Dijkstra supports cheaper later
relaxations and rejects stale heap entries. Disconnected Portals retain infinite
cost. Costs and the incident Block containing the next segment toward the goal
are held in goal-local SecondaryMaps.

Each reachable non-goal Block selects one exit, minimizing distance from the Block's
geometric center in cell-center coordinates to the Portal representative plus its
remaining goal cost. Portals whose next segment returns into that Block are excluded.
The selected exit is stored in `SecondaryMap<BlockId, PortalId>`.

`PortalRoutes<'a>` borrows Map and returns the original `&Portal`. All cells in a
Block use the same exit through constant-time lookup; there is no per-cell candidate
scan. The goal Block steers directly to the goal. Disconnected Blocks have no exit.
Routes are not synchronized back into Portal objects and do not survive the field
construction call. The returned direction array contains no topology references;
`FlowField` retains its existing generation-based whole-field reuse.

## Steering and limits

Per-cell steering targets the nearest point on the selected Portal's cell-side
interval, excluding one cell at each end for intervals of at least three cells.
At that target cell it points across the boundary. Normalization and optional wall
repulsion follow. The visualizer samples cell vectors without interpolation or
hidden collision correction; an in-game sampler can provide continuous steering.

Portal-midpoint costs, one exit per Block, nearest-point steering, repulsion, and
piecewise cell sampling remain approximations. Rectangular Blocks avoid the former
non-convex merged-region issue, but do not guarantee a continuous-space shortest
path or collision-free traced motion under every steering configuration.

For m_b Portals per Block, graph expansion considers roughly sum(m_b^2) internal
links. Exit selection evaluates sum(m_b) candidates once per field. Per-cell
nearest-point steering, normalization, and repulsion remain.

```sh
nix develop --command cargo test --workspace
nix develop --command cargo bench -p chunk-flow-field --bench rts_flow
nix develop --command cargo run --release -p visualizer
```
