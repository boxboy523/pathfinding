//! Point-agent trajectory diagnostics; no sliding/steering or collision correction.
use std::{cmp::Reverse, collections::{BinaryHeap, HashSet}};

use chunk_flow_field::{
    map::{Map, EDGE_BOTTOM, EDGE_LEFT, EDGE_RIGHT, EDGE_TOP},
    types::{Cell, FlowFieldOptions, Pos},
};
use macroquad::prelude::*;

const STEPS: [(isize, isize, u32); 8] = [
    (0, -1, 1000), (1, 0, 1000), (0, 1, 1000), (-1, 0, 1000),
    (-1, -1, 1414), (1, -1, 1414), (-1, 1, 1414), (1, 1, 1414),
];

fn walkable(map: &Map, p: Pos) -> bool {
    matches!(map.get_cell(p.x, p.y), Cell::Block(_))
}

pub fn center(p: Pos) -> Vec2 {
    vec2(p.x as f32 + 0.5, p.y as f32 + 0.5)
}

fn cell(p: Vec2, map: &Map) -> Option<Pos> {
    if !p.is_finite() || p.x < 0.0 || p.y < 0.0
        || p.x >= map.width as f32 || p.y >= map.height as f32
    {
        return None;
    }
    Some(Pos { x: p.x.floor() as usize, y: p.y.floor() as usize })
}

fn cardinal(map: &Map, a: Pos, b: Pos) -> bool {
    if !walkable(map, a) || !walkable(map, b) {
        return false;
    }
    let (edge, opposite) = match (b.x as isize - a.x as isize, b.y as isize - a.y as isize) {
        (1, 0) => (EDGE_RIGHT, EDGE_LEFT),
        (-1, 0) => (EDGE_LEFT, EDGE_RIGHT),
        (0, 1) => (EDGE_BOTTOM, EDGE_TOP),
        (0, -1) => (EDGE_TOP, EDGE_BOTTOM),
        _ => return false,
    };
    map.can_pass(a.x, a.y, edge) && map.can_pass(b.x, b.y, opposite)
}

fn can_step(map: &Map, a: Pos, b: Pos) -> bool {
    if a == b {
        return walkable(map, a);
    }
    if a.x.abs_diff(b.x) > 1 || a.y.abs_diff(b.y) > 1 {
        return false;
    }
    if a.x == b.x || a.y == b.y {
        return cardinal(map, a, b);
    }
    // Conservative corner rule: both orthogonal routes must be clear.
    let horizontal = Pos { x: b.x, y: a.y };
    let vertical = Pos { x: a.x, y: b.y };
    cardinal(map, a, horizontal) && cardinal(map, horizontal, b)
        && cardinal(map, a, vertical) && cardinal(map, vertical, b)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status { Reached, Unreachable, Blocked, Loop, StepLimit, Stalled }

impl Status {
    pub fn label(self) -> &'static str {
        match self {
            Self::Reached => "reached",
            Self::Unreachable => "unreachable",
            Self::Blocked => "BLOCKED",
            Self::Loop => "LOOP",
            Self::StepLimit => "step limit",
            Self::Stalled => "stalled/invalid vector",
        }
    }
}

pub struct Trace {
    pub points: Vec<Vec2>,
    pub status: Status,
}

impl Trace {
    pub fn length(&self) -> f32 {
        self.points.windows(2).map(|p| p[0].distance(p[1])).sum()
    }
}

pub fn trace_flow(map: &Map, flow: &[Option<(f32, f32)>], start: Pos, goal: Pos) -> Trace {
    let mut trace = Trace { points: vec![center(start)], status: Status::StepLimit };
    if !walkable(map, start) || !walkable(map, goal) {
        trace.status = Status::Unreachable;
        return trace;
    }
    let mut visited = HashSet::new();
    let mut last_key = None;
    let max_steps = (map.width * map.height * 32).min(500_000);
    for _ in 0..max_steps {
        let current = *trace.points.last().unwrap();
        let pos = cell(current, map).unwrap();
        if pos == goal {
            trace.points.push(center(goal));
            trace.status = Status::Reached;
            return trace;
        }
        let Some(Some((dx, dy))) = flow.get(pos.y * map.width + pos.x) else {
            trace.status = Status::Unreachable;
            return trace;
        };
        let direction = vec2(*dx, *dy);
        if !direction.is_finite() || direction.length_squared() < 1e-10 {
            trace.status = Status::Stalled;
            return trace;
        }
        // Piecewise-constant cell sampling with 0.08-cell Euler steps.
        let next = current + direction.normalize() * 0.08;
        let Some(next_cell) = cell(next, map) else {
            trace.status = Status::Blocked;
            return trace;
        };
        if !can_step(map, pos, next_cell) {
            trace.status = Status::Blocked;
            return trace;
        }
        let key = ((next.x * 16.0).floor() as i32, (next.y * 16.0).floor() as i32);
        if last_key != Some(key) && !visited.insert(key) {
            trace.status = Status::Loop;
            return trace;
        }
        last_key = Some(key);
        trace.points.push(next);
    }
    trace
}

fn heuristic(a: Pos, b: Pos) -> u32 {
    let dx = a.x.abs_diff(b.x) as u32;
    let dy = a.y.abs_diff(b.y) as u32;
    1414 * dx.min(dy) + 1000 * dx.abs_diff(dy)
}

/// Full-cell integration field with the same 8-way costs as A*. No Portal routing.
/// Repulsion is applied using the same Map helper as the Block/Portal field.
pub fn grid_flow(map: &Map, goal: Pos, options: &FlowFieldOptions) -> Vec<Option<(f32, f32)>> {
    let mut flow = vec![None; map.width * map.height];
    if !walkable(map, goal) { return flow; }
    let goal_index = goal.y * map.width + goal.x;
    let mut distance = vec![u32::MAX; flow.len()];
    let mut queue = BinaryHeap::new();
    distance[goal_index] = 0;
    queue.push(Reverse((0_u32, goal_index)));
    while let Some(Reverse((cost, index))) = queue.pop() {
        if cost != distance[index] { continue; }
        let pos = Pos { x: index % map.width, y: index / map.width };
        for (dx, dy, step_cost) in STEPS {
            let (Some(x), Some(y)) = (pos.x.checked_add_signed(dx), pos.y.checked_add_signed(dy)) else { continue; };
            let next = Pos { x, y };
            if !can_step(map, pos, next) { continue; }
            let next_index = y * map.width + x;
            if cost + step_cost < distance[next_index] {
                distance[next_index] = cost + step_cost;
                queue.push(Reverse((cost + step_cost, next_index)));
            }
        }
    }
    for index in 0..flow.len() {
        if distance[index] == u32::MAX { continue; }
        if index == goal_index {
            flow[index] = Some((0.0, 0.0));
            continue;
        }
        let pos = Pos { x: index % map.width, y: index / map.width };
        let mut best_cost = u32::MAX;
        let mut target = None;
        for (dx, dy, step_cost) in STEPS {
            let (Some(x), Some(y)) = (pos.x.checked_add_signed(dx), pos.y.checked_add_signed(dy)) else { continue; };
            let next = Pos { x, y };
            if !can_step(map, pos, next) { continue; }
            let next_distance = distance[y * map.width + x];
            if next_distance < distance[index] && next_distance + step_cost < best_cost {
                best_cost = next_distance + step_cost;
                target = Some(next);
            }
        }
        if let Some(target) = target {
            flow[index] = Some(map.get_norm_with_repulsion(pos, target, options));
        }
    }
    flow
}

pub fn astar(map: &Map, start: Pos, goal: Pos) -> Trace {
    let unreachable = || Trace { points: vec![center(start)], status: Status::Unreachable };
    if !walkable(map, start) || !walkable(map, goal) { return unreachable(); }
    let start_index = start.y * map.width + start.x;
    let goal_index = goal.y * map.width + goal.x;
    let mut cost = vec![u32::MAX; map.width * map.height];
    let mut parent = vec![usize::MAX; cost.len()];
    let mut queue = BinaryHeap::new();
    cost[start_index] = 0;
    let h = heuristic(start, goal);
    queue.push(Reverse((h, h, start_index)));
    while let Some(Reverse((f, h, index))) = queue.pop() {
        if f - h != cost[index] { continue; }
        if index == goal_index {
            let mut points = vec![center(goal)];
            let mut current = index;
            while current != start_index {
                current = parent[current];
                points.push(center(Pos { x: current % map.width, y: current / map.width }));
            }
            points.reverse();
            return Trace { points, status: Status::Reached };
        }
        let pos = Pos { x: index % map.width, y: index / map.width };
        for (dx, dy, step_cost) in STEPS {
            let (Some(x), Some(y)) = (pos.x.checked_add_signed(dx), pos.y.checked_add_signed(dy)) else { continue; };
            let next = Pos { x, y };
            if !can_step(map, pos, next) { continue; }
            let next_index = y * map.width + x;
            let next_cost = cost[index] + step_cost;
            if next_cost < cost[next_index] {
                cost[next_index] = next_cost;
                parent[next_index] = index;
                let h = heuristic(next, goal);
                queue.push(Reverse((next_cost + h, h, next_index)));
            }
        }
    }
    unreachable()
}

#[cfg(test)]
mod tests {
    use super::*;
    use chunk_flow_field::{map::{Obstacle, Side}, types::FlowFieldOptions};

    #[test]
    fn grid_flow_reaches_open_goal_and_marks_walls_and_unreachable_cells() {
        let map = Map::new(&Obstacle::new(8, 8));
        let start = Pos { x: 1, y: 1 };
        let goal = Pos { x: 6, y: 6 };
        for strength in [0.0, 0.5] {
            let options = FlowFieldOptions { wall_repulsion_strength: strength, ..Default::default() };
            let flow = grid_flow(&map, goal, &options);
            let trace = trace_flow(&map, &flow, start, goal);
            assert_eq!(trace.status, Status::Reached);
            assert!((trace.length() - astar(&map, start, goal).length()).abs() < 0.01);
            assert_eq!(flow[goal.y * map.width + goal.x], Some((0.0, 0.0)));
        }
        let mut obstacle = Obstacle::new(5, 3);
        for y in 0..3 { obstacle.set_cell_blocked(2, y); }
        let map = Map::new(&obstacle);
        let flow = grid_flow(&map, Pos { x: 4, y: 1 }, &FlowFieldOptions::default());
        assert!(flow[5].is_none());
        assert!(flow[7].is_none());
        assert!(flow[8].is_some());
    }

    #[test]
    fn grid_flow_respects_edge_only_barriers() {
        let mut obstacle = Obstacle::new(4, 3);
        for y in 0..3 { obstacle.set_edge(1, y, Side::Right); }
        let map = Map::new(&obstacle);
        let flow = grid_flow(&map, Pos { x: 3, y: 1 }, &FlowFieldOptions::default());
        for y in 0..3 {
            assert!(flow[y * 4 + 1].is_none());
            assert!(flow[y * 4 + 2].is_some());
        }
    }

    #[test]
    fn open_map_paths_reach_goal() {
        let map = Map::new(&Obstacle::new(8, 8));
        let start = Pos { x: 1, y: 1 };
        let goal = Pos { x: 6, y: 6 };
        let flow = map.build_flow_field(goal, &FlowFieldOptions::default()).unwrap();
        let trace = trace_flow(&map, &flow, start, goal);
        assert_eq!(trace.status, Status::Reached);
        let reference = astar(&map, start, goal);
        assert_eq!(reference.status, Status::Reached);
        assert!((reference.length() - 5.0 * 2.0_f32.sqrt()).abs() < 1e-4);
        assert!((trace.length() - reference.length()).abs() < 0.01);
        assert_eq!(trace_flow(&map, &flow, goal, goal).length(), 0.0);
    }

    #[test]
    fn trace_does_not_cross_a_blocked_edge() {
        let mut obstacle = Obstacle::new(4, 3);
        obstacle.set_edge(1, 1, Side::Right);
        let map = Map::new(&obstacle);
        let flow = vec![Some((1.0, 0.0)); 12];
        let start = Pos { x: 1, y: 1 };
        let goal = Pos { x: 2, y: 1 };
        assert_eq!(trace_flow(&map, &flow, start, goal).status, Status::Blocked);
        let reference = astar(&map, start, goal);
        assert_eq!(reference.status, Status::Reached);
        assert!(reference.length() > 1.0);
    }

    #[test]
    fn disconnected_and_looping_fields_are_reported() {
        let mut obstacle = Obstacle::new(5, 3);
        for y in 0..3 { obstacle.set_cell_blocked(2, y); }
        let map = Map::new(&obstacle);
        let start = Pos { x: 0, y: 1 };
        let goal = Pos { x: 4, y: 1 };
        let flow = map.build_flow_field(goal, &FlowFieldOptions::default()).unwrap();
        assert_eq!(trace_flow(&map, &flow, start, goal).status, Status::Unreachable);
        assert_eq!(astar(&map, start, goal).status, Status::Unreachable);
        let map = Map::new(&Obstacle::new(4, 3));
        let mut flow = vec![Some((1.0, 0.0)); 12];
        for y in 0..3 { flow[y * 4 + 1] = Some((-1.0, 0.0)); }
        assert_eq!(trace_flow(&map, &flow, start, Pos { x: 3, y: 1 }).status, Status::Loop);
    }
}
