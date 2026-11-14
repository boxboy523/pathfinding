# RTS flow-field benchmark

Run from the workspace root:

```sh
nix develop --command cargo bench -p chunk-flow-field --bench rts_flow
# Or, with a Rust toolchain already available:
cargo bench -p chunk-flow-field --bench rts_flow

# More samples (default: 5):
FLOW_BENCH_SAMPLES=15 nix develop --command cargo bench -p chunk-flow-field --bench rts_flow
```

No extra dependencies. The benchmark requires the optimized bench profile and
prints a timing table for each deterministic 256x256 map:

1. 16 large rooms, 8-cell doorways.
2. 16 large rooms, 4-cell doorways, one 6x6 obstacle island per room.
3. 64 rooms, 4-cell doorways, one 6x6 obstacle island per room.

Walls are two cells thick, with a two-cell outer border. All walkable cells
must be connected. Map generation and converting terrain to `Obstacle` are
outside timed sections. Scenarios contain blocked cells, not edge-only walls.

## Implementations

- **Block:** the actual `Map::new`, `add_building`, `remove_building`, and
  `build_flow_field` APIs. Wall repulsion is disabled.
- **Grid Dijkstra:** a conventional full-cell integration field followed by
  downhill direction extraction. Eight neighbors, costs 1000/1414, no diagonal
  corner cutting. Fresh integration/direction arrays are allocated for every
  goal, as the Block/Portal implementation also produces a fresh direction array.
- **A* x100:** one independent eight-neighbor A* query per unit, using the same
  costs/corner rules as Grid Dijkstra and an admissible octile heuristic.
  Equal `f` favors smaller `h`. Cost/parent buffers and heap capacity are reused
  across queries; cost reset, search, path allocation, and full path reconstruction
  are timed. No path sharing, hierarchy, or prior-query result cache is used.

All implementations operate on exactly the same blocked cells and goals. These
are straightforward implementations, not claims about the fastest possible
algorithms. A four-neighbor uniform-cost BFS would have a different cost and
movement model. SIMD and incremental algorithms are not included. Flow-field
arrays are freshly allocated; A* reuses scratch buffers as described above.

## Measurements

All values are median milliseconds. Implementations alternate order between
samples. Each scenario has an untimed correctness/warm-up pass.

- `topology setup`: rectangular Block/Portal construction versus copying the baseline blocked
  grid. Not equivalent work: baseline needs no region graph. This row exposes
  preprocessing cost rather than claiming an algorithmic speedup ratio.
- `setup + first flow`: sum of setup and first field generation in each sample.
- `new goal / flow`: three goals per sample, with existing topology. This
  intentionally calls field generation, not a valid generation-cache lookup.
- `add 3x3 building`: actual building API versus setting nine baseline cells.
  Constructing the full-map building input is excluded. Each sample starts with
  no buildings, so this does not measure many-building scaling.
  Block/Portal topology updates are local; unaffected object keys are retained.
- `add + fresh flow`: building addition plus rebuilding one goal's field.
- `remove building`: actual removal API versus clearing those nine baseline
  cells. Their original state is known to be empty.
- `remove + fresh flow`: removal plus rebuilding one goal's field.

Ratios greater than 1 mean Block is slower. For sub-microsecond baseline
mutations, ratios are omitted because single-shot timer overhead dominates.
Prefer the end-to-end `... + fresh flow` rows for dynamic-terrain decisions.
For many independent goals, factor in the number of fields regenerated after
each topology change, rather than comparing only a single flow request.

### 100-unit comparison

A second table compares **one shared flow field** (Block or Grid Dijkstra) with
**100 independent A* paths to the same goal**. The 100 distinct start cells are
deterministically spread across the walkable map, excluding goals and the test
building's footprint. This is a distributed-army workload, not a clustered squad.
Each of the three goals uses the same start positions.

- `setup + ready`: topology/grid setup plus one shared field or 100 A* paths.
  A* scratch allocation is included here.
- `new goal / ready`: one field versus 100 paths on unchanged terrain.
- `add + ready`, `remove + ready`: terrain update plus replanning for all units.

These are planning costs only: movement, 100 flow lookups, following A* waypoints,
collision avoidance, formations, and rendering are excluded. A* returns full
paths; flow fields provide directions, not 100 explicitly materialized paths.
After terrain changes all 100 units are replanned; an actual game could retain
unaffected A* paths and replan fewer units. Likewise, a valid cached flow requires
no regeneration when neither goal nor terrain changes. Separate unit goals would
reduce sharing and require a different experiment.

## Correctness and limits

Before timing, every goal is checked for matching per-cell reachability,
finite normalized Block/Portal flow vectors, and zero goal direction. Every A* path is
checked for endpoints, legal neighbor/corner steps, and shortest cost equal to
the Dijkstra integration distance. Addition/removal are also checked, including
A* after the terrain changes; removal must restore the original wall layout.
Failures abort the benchmark rather than producing timings for invalid data.

This does **not** establish equivalent path quality: Block/Portal routing is
approximate and continuous, whereas the baseline follows discrete octile
shortest paths. No moving-agent simulation, trajectory/collision check, or
path-length comparison is included. Printed counts are Blocks and shared Portals;
there is no Chunk layer or hull-based merging.

Machine load, CPU, compiler version, and sample count affect results. Use
release measurements on the consuming project's machine before choosing an
architecture. The benchmark currently measures public operations, not internal
Block/Portal phase timings.
