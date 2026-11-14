//! Run with: cargo bench -p chunk-flow-field --bench rts_flow
use std::{
    cmp::Reverse,
    collections::BinaryHeap,
    hint::black_box,
    time::{Duration, Instant},
};

use chunk_flow_field::{
    map::{Map, Obstacle},
    types::{Cell, FlowFieldOptions, Pos},
};

const SIZE: usize = 256;
const UNITS: usize = 100;
const UNREACHABLE: u32 = u32::MAX;
const STEPS: [(isize, isize, u32); 8] = [
    (0, -1, 1000), (1, 0, 1000), (0, 1, 1000), (-1, 0, 1000),
    (-1, -1, 1414), (1, -1, 1414), (-1, 1, 1414), (1, 1, 1414),
];
type Directions = Vec<Option<(f32, f32)>>;

/// Conventional cell integration field: 8-way Dijkstra, then a downhill direction
/// per reachable cell. Diagonals cannot cut blocked corners. No wall repulsion.
struct Grid {
    blocked: Vec<bool>,
}

impl Grid {
    fn neighbor(&self, index: usize, dx: isize, dy: isize) -> Option<usize> {
        let x = index % SIZE;
        let y = index / SIZE;
        let nx = x.checked_add_signed(dx)?;
        let ny = y.checked_add_signed(dy)?;
        if nx >= SIZE || ny >= SIZE || self.blocked[ny * SIZE + nx] {
            return None;
        }
        if dx != 0 && dy != 0
            && (self.blocked[y * SIZE + nx] || self.blocked[ny * SIZE + x])
        {
            return None;
        }
        Some(ny * SIZE + nx)
    }

    fn distances(&self, goal: Pos) -> Vec<u32> {
        let goal_index = goal.y * SIZE + goal.x;
        assert!(!self.blocked[goal_index]);
        let mut distance = vec![UNREACHABLE; SIZE * SIZE];
        let mut queue = BinaryHeap::new();
        distance[goal_index] = 0;
        queue.push(Reverse((0_u32, goal_index)));
        while let Some(Reverse((cost, index))) = queue.pop() {
            if distance[index] != cost {
                continue;
            }
            for (dx, dy, step_cost) in STEPS {
                if let Some(next) = self.neighbor(index, dx, dy) {
                    let next_cost = cost + step_cost;
                    if next_cost < distance[next] {
                        distance[next] = next_cost;
                        queue.push(Reverse((next_cost, next)));
                    }
                }
            }
        }
        distance
    }

    fn flow(&self, goal: Pos) -> Directions {
        let goal_index = goal.y * SIZE + goal.x;
        let distance = self.distances(goal);
        let mut flow = vec![None; SIZE * SIZE];
        for index in 0..flow.len() {
            if distance[index] == UNREACHABLE {
                continue;
            }
            if index == goal_index {
                flow[index] = Some((0.0, 0.0));
                continue;
            }
            let mut best_cost = UNREACHABLE;
            let mut direction = None;
            for (dx, dy, step_cost) in STEPS {
                if let Some(next) = self.neighbor(index, dx, dy) {
                    if distance[next] < distance[index] {
                        let cost = distance[next] + step_cost;
                        if cost < best_cost {
                            best_cost = cost;
                            let length = (dx as f32).hypot(dy as f32);
                            direction = Some((dx as f32 / length, dy as f32 / length));
                        }
                    }
                }
            }
            flow[index] = direction;
        }
        flow
    }
}

/// Independent A* searches; buffers are reused, but no paths/search results are shared.
struct AStar {
    cost: Vec<u32>,
    parent: Vec<usize>,
    open: BinaryHeap<Reverse<(u32, u32, usize)>>,
}

fn heuristic(from: usize, goal: usize) -> u32 {
    let dx = (from % SIZE).abs_diff(goal % SIZE) as u32;
    let dy = (from / SIZE).abs_diff(goal / SIZE) as u32;
    1414 * dx.min(dy) + 1000 * dx.abs_diff(dy)
}

impl AStar {
    fn new() -> Self {
        Self {
            cost: vec![UNREACHABLE; SIZE * SIZE],
            parent: vec![usize::MAX; SIZE * SIZE],
            open: BinaryHeap::new(),
        }
    }

    fn path(&mut self, grid: &Grid, start: usize, goal: Pos) -> Vec<usize> {
        let goal = goal.y * SIZE + goal.x;
        assert!(!grid.blocked[start] && !grid.blocked[goal]);
        self.cost.fill(UNREACHABLE);
        // Parent entries are read only for nodes reached in this search.
        self.open.clear();
        self.cost[start] = 0;
        self.parent[start] = start;
        let h = heuristic(start, goal);
        self.open.push(Reverse((h, h, start)));
        while let Some(Reverse((f, h, index))) = self.open.pop() {
            if f - h != self.cost[index] {
                continue;
            }
            if index == goal {
                let mut path = vec![goal];
                let mut current = goal;
                while current != start {
                    current = self.parent[current];
                    path.push(current);
                }
                path.reverse();
                return path;
            }
            for (dx, dy, step_cost) in STEPS {
                if let Some(next) = grid.neighbor(index, dx, dy) {
                    let cost = self.cost[index] + step_cost;
                    if cost < self.cost[next] {
                        self.cost[next] = cost;
                        self.parent[next] = index;
                        let h = heuristic(next, goal);
                        // Equal f prefers lower h (closer to goal).
                        self.open.push(Reverse((cost + h, h, next)));
                    }
                }
            }
        }
        Vec::new()
    }

    fn paths(&mut self, grid: &Grid, starts: &[usize], goal: Pos) -> Vec<Vec<usize>> {
        starts.iter().map(|&start| self.path(grid, start, goal)).collect()
    }
}

fn verify_astar(grid: &Grid, starts: &[usize], goal: Pos) {
    let reference = grid.distances(goal);
    let mut astar = AStar::new();
    for &start in starts {
        let path = astar.path(grid, start, goal);
        assert_eq!(path.first(), Some(&start));
        assert_eq!(path.last(), Some(&(goal.y * SIZE + goal.x)));
        let mut cost = 0;
        for pair in path.windows(2) {
            let dx = (pair[1] % SIZE) as isize - (pair[0] % SIZE) as isize;
            let dy = (pair[1] / SIZE) as isize - (pair[0] / SIZE) as isize;
            assert!(dx.abs() <= 1 && dy.abs() <= 1 && (dx != 0 || dy != 0));
            assert_eq!(grid.neighbor(pair[0], dx, dy), Some(pair[1]));
            cost += if dx != 0 && dy != 0 { 1414 } else { 1000 };
        }
        assert_eq!(cost, reference[start], "A* must match Dijkstra shortest cost");
    }
}

struct Scenario {
    name: &'static str,
    blocked: Vec<bool>,
    obstacle: Obstacle,
    goals: Vec<Pos>,
    building: Vec<usize>,
}

impl Scenario {
    fn rooms(name: &'static str, room_size: usize, door_width: usize, clutter: bool) -> Self {
        let rooms = SIZE / room_size;
        let mut blocked = vec![false; SIZE * SIZE];
        for y in 0..SIZE {
            for x in 0..SIZE {
                let border = x < 2 || y < 2 || x >= SIZE - 2 || y >= SIZE - 2;
                let vertical = x >= room_size && x % room_size < 2;
                let horizontal = y >= room_size && y % room_size < 2;
                let door_y = y % room_size >= room_size / 2 - door_width / 2
                    && y % room_size < room_size / 2 + door_width / 2;
                let door_x = x % room_size >= room_size / 2 - door_width / 2
                    && x % room_size < room_size / 2 + door_width / 2;
                let island = clutter
                    && (room_size / 3..room_size / 3 + 6).contains(&(x % room_size))
                    && (room_size / 3..room_size / 3 + 6).contains(&(y % room_size));
                blocked[y * SIZE + x] = border
                    || (vertical && !door_y)
                    || (horizontal && !door_x)
                    || island;
            }
        }
        let mut obstacle = Obstacle::new(SIZE, SIZE);
        for (index, &wall) in blocked.iter().enumerate() {
            if wall {
                obstacle.set_cell_blocked(index % SIZE, index / SIZE);
            }
        }
        let goals = vec![
            Pos { x: 8, y: 8 },
            Pos { x: SIZE - 8, y: SIZE - 8 },
            Pos { x: (rooms / 2) * room_size + 8, y: 8 },
        ];
        let building = (4..7)
            .flat_map(|y| (4..7).map(move |x| y * SIZE + x))
            .collect::<Vec<_>>();
        assert!(building.iter().all(|&i| !blocked[i]));
        assert!(goals.iter().all(|p| !blocked[p.y * SIZE + p.x]));
        Self { name, blocked, obstacle, goals, building }
    }

    fn building_obstacle(&self) -> Obstacle {
        let mut obstacle = Obstacle::new(SIZE, SIZE);
        for &index in &self.building {
            obstacle.set_cell_blocked(index % SIZE, index / SIZE);
        }
        obstacle
    }

    fn unit_starts(&self) -> Vec<usize> {
        let eligible = self.blocked.iter().enumerate()
            .filter(|(i, blocked)| !**blocked && !self.building.contains(i)
                && !self.goals.iter().any(|p| p.y * SIZE + p.x == *i))
            .map(|(i, _)| i).collect::<Vec<_>>();
        assert!(eligible.len() >= UNITS);
        // Deterministic, distinct starts spread across the map; no RNG dependency.
        (0..UNITS).map(|unit| eligible[(2 * unit + 1) * eligible.len() / (2 * UNITS)]).collect()
    }
}

fn options() -> FlowFieldOptions {
    FlowFieldOptions { wall_repulsion_strength: 0.0, ..Default::default() }
}

/// Checks reachability, finite unit directions and goal, not path optimality.
/// Block/Portal fields are continuous/approximate; baseline paths are discrete octile paths.
fn verify(map: &Map, grid: &Grid, goal: Pos) {
    let chunk = map.build_flow_field(goal, &options()).unwrap();
    let simple = grid.flow(goal);
    assert_eq!(chunk.len(), simple.len());
    assert_eq!(simple.iter().filter(|v| v.is_some()).count(),
        grid.blocked.iter().filter(|&&wall| !wall).count(),
        "scenario must have one connected walkable region");
    for (index, (a, b)) in chunk.iter().zip(&simple).enumerate() {
        assert_eq!(a.is_some(), b.is_some(), "reachability mismatch at {index}");
        if let Some((dx, dy)) = a {
            assert!(dx.is_finite() && dy.is_finite());
            let length = dx.hypot(*dy);
            assert!(length == 0.0 || (length - 1.0).abs() < 1e-5);
        }
    }
    assert_eq!(chunk[goal.y * SIZE + goal.x], Some((0.0, 0.0)));
}

fn measure<T>(operation: impl FnOnce() -> T) -> (Duration, T) {
    let start = Instant::now();
    let value = operation();
    (start.elapsed(), value)
}

fn median(samples: &[Duration]) -> f64 {
    let mut sorted = samples.to_vec();
    sorted.sort_unstable();
    let middle = sorted.len() / 2;
    if sorted.len() % 2 == 0 {
        (sorted[middle - 1].as_secs_f64() + sorted[middle].as_secs_f64()) * 500.0
    } else {
        sorted[middle].as_secs_f64() * 1000.0
    }
}

fn row(label: &str, chunk: &[Duration], simple: &[Duration]) {
    let a = median(chunk);
    let b = median(simple);
    if b < 0.001 {
        // Sub-microsecond mutations are too short for a useful single-shot ratio.
        println!("{label:<27} {a:>12.3} {b:>14.6} {:>14}", "--");
    } else {
        println!("{label:<27} {a:>12.3} {b:>14.3} {:>13.2}x", a / b);
    }
}

fn run(scenario: Scenario, samples: usize) {
    println!("\n{}: {}x{}, {} walkable cells", scenario.name, SIZE, SIZE,
        scenario.blocked.iter().filter(|&&wall| !wall).count());
    let warm_map = Map::new(&scenario.obstacle);
    let warm_grid = Grid { blocked: scenario.blocked.clone() };
    let starts = scenario.unit_starts();
    println!("{} blocks, {} portals; checking all goals and add/remove...",
        warm_map.blocks.len(), warm_map.portals.len());
    for &goal in &scenario.goals {
        verify(&warm_map, &warm_grid, goal);
        verify_astar(&warm_grid, &starts, goal);
    }
    let mut check_map = Map::new(&scenario.obstacle);
    let mut check_grid = Grid { blocked: scenario.blocked.clone() };
    let id = check_map.add_building(scenario.building_obstacle()).unwrap();
    for &index in &scenario.building { check_grid.blocked[index] = true; }
    verify(&check_map, &check_grid, scenario.goals[0]);
    verify_astar(&check_grid, &starts, scenario.goals[0]);
    check_map.remove_building(id).unwrap();
    verify(&check_map, &warm_grid, scenario.goals[0]);
    verify_astar(&warm_grid, &starts, scenario.goals[0]);
    for y in 0..SIZE {
        for x in 0..SIZE {
            assert_eq!(check_map.get_cell(x, y) == Cell::Wall, scenario.blocked[y * SIZE + x]);
        }
    }

    let mut setup = [Vec::new(), Vec::new()];
    let mut initial = [Vec::new(), Vec::new()];
    let mut goals = [Vec::new(), Vec::new()];
    let mut add = [Vec::new(), Vec::new()];
    let mut add_ready = [Vec::new(), Vec::new()];
    let mut remove = [Vec::new(), Vec::new()];
    let mut remove_ready = [Vec::new(), Vec::new()];
    let mut astar_initial = Vec::new();
    let mut astar_goals = Vec::new();
    let mut astar_add = Vec::new();
    let mut astar_remove = Vec::new();
    for sample in 0..samples {
        // Rotate all three implementations to reduce systematic ordering bias.
        for offset in 0..3 {
            let implementation = (sample + offset) % 3;
            if implementation == 0 {
                let (elapsed, mut map) = measure(|| Map::new(black_box(&scenario.obstacle)));
                setup[0].push(elapsed);
                let (flow_time, flow) = measure(|| map.build_flow_field(scenario.goals[0], &options()).unwrap());
                black_box(&flow);
                initial[0].push(elapsed + flow_time);
                for &goal in &scenario.goals {
                    let (elapsed, flow) = measure(|| map.build_flow_field(black_box(goal), &options()).unwrap());
                    black_box(&flow);
                    goals[0].push(elapsed);
                }
                // Full-map input allocation is excluded: this measures the API itself.
                let building = scenario.building_obstacle();
                let (elapsed, id) = measure(|| map.add_building(black_box(building)).unwrap());
                add[0].push(elapsed);
                let (flow_time, flow) = measure(|| map.build_flow_field(scenario.goals[0], &options()).unwrap());
                black_box(&flow);
                add_ready[0].push(elapsed + flow_time);
                let (elapsed, ()) = measure(|| map.remove_building(black_box(id)).unwrap());
                remove[0].push(elapsed);
                let (flow_time, flow) = measure(|| map.build_flow_field(scenario.goals[0], &options()).unwrap());
                black_box(&flow);
                remove_ready[0].push(elapsed + flow_time);
                black_box(&map);
            } else if implementation == 1 {
                let (elapsed, mut grid) = measure(|| Grid { blocked: black_box(&scenario.blocked).clone() });
                setup[1].push(elapsed);
                let (flow_time, flow) = measure(|| grid.flow(scenario.goals[0]));
                black_box(&flow);
                initial[1].push(elapsed + flow_time);
                for &goal in &scenario.goals {
                    let (elapsed, flow) = measure(|| grid.flow(black_box(goal)));
                    black_box(&flow);
                    goals[1].push(elapsed);
                }
                let (elapsed, ()) = measure(|| {
                    for &index in black_box(&scenario.building) { grid.blocked[index] = true; }
                });
                add[1].push(elapsed);
                let (flow_time, flow) = measure(|| grid.flow(scenario.goals[0]));
                black_box(&flow);
                add_ready[1].push(elapsed + flow_time);
                let (elapsed, ()) = measure(|| {
                    for &index in black_box(&scenario.building) { grid.blocked[index] = false; }
                });
                remove[1].push(elapsed);
                let (flow_time, flow) = measure(|| grid.flow(scenario.goals[0]));
                black_box(&flow);
                remove_ready[1].push(elapsed + flow_time);
                black_box(&grid);
            } else {
                let (elapsed, (mut grid, mut astar)) = measure(|| (
                    Grid { blocked: black_box(&scenario.blocked).clone() },
                    AStar::new(),
                ));
                let (path_time, paths) = measure(|| astar.paths(&grid, black_box(&starts), scenario.goals[0]));
                black_box(&paths);
                astar_initial.push(elapsed + path_time);
                for &goal in &scenario.goals {
                    let (elapsed, paths) = measure(|| astar.paths(&grid, black_box(&starts), black_box(goal)));
                    black_box(&paths);
                    astar_goals.push(elapsed);
                }
                let (elapsed, ()) = measure(|| {
                    for &index in black_box(&scenario.building) { grid.blocked[index] = true; }
                });
                let (path_time, paths) = measure(|| astar.paths(&grid, black_box(&starts), scenario.goals[0]));
                black_box(&paths);
                astar_add.push(elapsed + path_time);
                let (elapsed, ()) = measure(|| {
                    for &index in black_box(&scenario.building) { grid.blocked[index] = false; }
                });
                let (path_time, paths) = measure(|| astar.paths(&grid, black_box(&starts), scenario.goals[0]));
                black_box(&paths);
                astar_remove.push(elapsed + path_time);
                black_box((&grid, &astar));
            }
        }
    }
    println!("{:<27} {:>12} {:>14} {:>14}", "median milliseconds", "Block", "Grid Dijkstra", "Block / Grid");
    row("topology setup", &setup[0], &setup[1]);
    row("setup + first flow", &initial[0], &initial[1]);
    row("new goal / flow", &goals[0], &goals[1]);
    row("add 3x3 building", &add[0], &add[1]);
    row("add + fresh flow", &add_ready[0], &add_ready[1]);
    row("remove building", &remove[0], &remove[1]);
    row("remove + fresh flow", &remove_ready[0], &remove_ready[1]);
    println!("\n100 units, one shared goal (planning only; median ms):");
    println!("{:<27} {:>12} {:>14} {:>14}", "workload", "Block x1", "Grid flow x1", "A* x100");
    for (label, block, grid, astar) in [
        ("setup + ready", &initial[0], &initial[1], &astar_initial),
        ("new goal / ready", &goals[0], &goals[1], &astar_goals),
        ("add + ready", &add_ready[0], &add_ready[1], &astar_add),
        ("remove + ready", &remove_ready[0], &remove_ready[1], &astar_remove),
    ] {
        println!("{label:<27} {:>12.3} {:>14.3} {:>14.3}", median(block), median(grid), median(astar));
    }
}

fn main() {
    assert!(!cfg!(debug_assertions), "Use cargo bench (optimized build)");
    let samples = std::env::var("FLOW_BENCH_SAMPLES")
        .map(|s| s.parse::<usize>().expect("FLOW_BENCH_SAMPLES must be a positive integer"))
        .unwrap_or(5);
    assert!(samples > 0);
    println!("RTS flow benchmark: {samples} samples, 3 goals per sample; no repulsion.");
    println!("Baseline: 8-way Dijkstra (1000/1414), no corner cutting.");
    println!("Block: rectangular partition + approximate portal routing, continuous directions.");
    println!("A*: 100 distributed starts, one shared goal, octile heuristic, path reconstruction included.");
    println!("Ratios >1 mean Block is slower. Reachability checked; path quality is NOT equivalent.");
    for scenario in [
        Scenario::rooms("16 large rooms / 8-cell doors", 64, 8, false),
        Scenario::rooms("16 large rooms / 4-cell doors + islands", 64, 4, true),
        Scenario::rooms("64 rooms / 4-cell doors + islands", 32, 4, true),
    ] {
        run(scenario, samples);
    }
}
