mod path_debug;

use chunk_flow_field::{
    map::{EDGE_BOTTOM, EDGE_RIGHT, Map, Obstacle, Side},
    types::{Cell, FlowField, FlowFieldOptions, Pos},
    BlockId,
};
use macroquad::prelude::*;
use std::hash::{DefaultHasher, Hash, Hasher};

const COLS: usize = 24;
const ROWS: usize = 22;
const SIZE: f32 = 32.0;
const TOP: f32 = 136.0;

fn window_conf() -> Conf {
    Conf {
        window_title: "Block Portal Flow Path Quality".into(),
        window_width: (COLS as f32 * SIZE) as i32,
        window_height: (ROWS as f32 * SIZE + TOP) as i32,
        platform: miniquad::conf::Platform {
            linux_backend: miniquad::conf::LinuxBackend::WaylandOnly,
            ..Default::default()
        },
        ..Default::default()
    }
}

fn demo_map() -> Map {
    let mut obstacle = Obstacle::new(COLS, ROWS);
    for y in 0..ROWS {
        for x in 0..COLS {
            // 계단형 삼각형은 병합되고, 오목한 L자는 분리되어야 한다.
            let staircase = (1..=9).contains(&x) && (1..=9).contains(&y) && x + y <= 10;
            let concave = (14..=22).contains(&x) && (1..=9).contains(&y) && (x <= 16 || y >= 7);
            let room = (1..=22).contains(&x) && (12..=20).contains(&y);
            if !staircase && !concave && !room {
                obstacle.set_cell_blocked(x, y);
            }
        }
    }
    // 아래 방: 막힌 경계 중 한 칸만 통행 가능.
    for y in 12..=20 {
        if y != 16 {
            obstacle.set_edge(11, y, Side::Right);
        }
    }
    Map::new(&obstacle)
}

// 서로 떨어진 세 공간: 기본 계단, 반대 방향 계단, 가로로 긴 2:1 계단.
fn staircase_region(x: usize, y: usize) -> Option<usize> {
    if (1..=9).contains(&x) && (1..=9).contains(&y) && x + y <= 10 {
        Some(0)
    } else if (14..=22).contains(&x) && (1..=9).contains(&y) && x - 14 >= y - 1 {
        Some(1)
    } else if (2..=20).contains(&x) && (12..=20).contains(&y) && x - 2 + 2 * (y - 12) <= 18 {
        Some(2)
    } else {
        None
    }
}

fn staircase_map() -> Map {
    let mut obstacle = Obstacle::new(COLS, ROWS);
    for y in 0..ROWS {
        for x in 0..COLS {
            if staircase_region(x, y).is_none() {
                obstacle.set_cell_blocked(x, y);
            }
        }
    }
    Map::new(&obstacle)
}

fn complex_map() -> Map {
    let mut obstacle = Obstacle::new(COLS, ROWS);
    let mut blocked = vec![false; COLS * ROWS];
    for y in 0..ROWS {
        for x in 0..COLS {
            let border = x == 0 || y == 0 || x + 1 == COLS || y + 1 == ROWS;
            let left_wall = x == 7 && (3..=17).contains(&y) && y != 6 && y != 15;
            let right_wall = x == 16 && (2..=18).contains(&y) && y != 10 && y != 17;
            let cross_wall = y == 10 && (2..=20).contains(&x) && ![5, 12, 18].contains(&x);
            let island = (10..=12).contains(&x) && (4..=6).contains(&y);
            let u_shape = ((x == 10 || x == 14) && (14..=18).contains(&y))
                || (y == 18 && (10..=14).contains(&x));
            let stair_obstacle =
                (19..=22).contains(&x) && (2..=6).contains(&y) && x - 19 + y - 2 <= 3;
            blocked[y * COLS + x] = border
                || left_wall
                || right_wall
                || cross_wall
                || island
                || u_shape
                || stair_obstacle;
            if blocked[y * COLS + x] {
                obstacle.set_cell_blocked(x, y);
            }
        }
    }
    // 실제 벽과 별도로, 엇갈린 통로가 있는 세로/가로 edge를 배치한다.
    for y in 2..=8 {
        if y != 5 && !blocked[y * COLS + 4] && !blocked[y * COLS + 5] {
            obstacle.set_edge(4, y, Side::Right);
        }
    }
    for y in 12..=19 {
        if y != 16 && !blocked[y * COLS + 19] && !blocked[y * COLS + 20] {
            obstacle.set_edge(19, y, Side::Right);
        }
    }
    for x in 9..=14 {
        if x != 12 && !blocked[7 * COLS + x] && !blocked[8 * COLS + x] {
            obstacle.set_edge(x, 7, Side::Bottom);
        }
    }
    Map::new(&obstacle)
}

// 방 ID는 row-major 0..15. 나선형 연결로 전 방 도달을 보장하고 지름길 두 개만 추가한다.
fn rts_rooms_connected(a: usize, b: usize) -> bool {
    const SPIRAL: [usize; 16] = [0, 1, 2, 3, 7, 11, 15, 14, 13, 12, 8, 4, 5, 6, 10, 9];
    SPIRAL.windows(2).any(|pair| (pair[0] == a && pair[1] == b) || (pair[0] == b && pair[1] == a))
        || [(1, 5), (10, 14)].iter().any(|&(x, y)| (a == x && b == y) || (a == y && b == x))
}

fn rts_map() -> Map {
    let mut obstacle = Obstacle::new(256, 256);
    for y in 0..256 {
        for x in 0..256 {
            let door_y = x >= 64 && (30..38).contains(&(y % 64))
                && rts_rooms_connected((y / 64) * 4 + x / 64 - 1, (y / 64) * 4 + x / 64);
            let door_x = y >= 64 && (30..38).contains(&(x % 64))
                && rts_rooms_connected((y / 64 - 1) * 4 + x / 64, (y / 64) * 4 + x / 64);
            let wall = x < 2 || y < 2 || x >= 254 || y >= 254
                || (x >= 64 && x % 64 < 2 && !door_y)
                || (y >= 64 && y % 64 < 2 && !door_x)
                || ((21..27).contains(&(x % 64)) && (21..27).contains(&(y % 64)));
            if wall { obstacle.set_cell_blocked(x, y); }
        }
    }
    Map::new(&obstacle)
}

fn draw_blocked_edges(map: &Map, tile_size: f32) {
    let scale = (tile_size / SIZE).min(1.0);
    for y in 0..map.height {
        for x in 0..map.width {
            if !matches!(map.get_cell(x, y), Cell::Block(_)) {
                continue;
            }
            let px = x as f32 * tile_size;
            let py = TOP + y as f32 * tile_size;
            let edges = [
                (EDGE_RIGHT, px + tile_size, py, px + tile_size, py + tile_size),
                (EDGE_BOTTOM, px, py + tile_size, px + tile_size, py + tile_size),
            ];
            for (mask, x1, y1, x2, y2) in edges {
                if map.can_pass(x, y, mask) {
                    continue;
                }
                draw_line(x1, y1, x2, y2, (10.0 * scale).max(1.0), BLACK);
                draw_line(x1, y1, x2, y2, (6.0 * scale).max(0.7), ORANGE);
                if tile_size >= 12.0 {
                    draw_line(x1, y1, x2, y2, 1.5, WHITE);
                    draw_circle(x1, y1, 3.0, ORANGE);
                    draw_circle(x2, y2, 3.0, ORANGE);
                }
            }
        }
    }
}

fn block_at(map: &Map, x: usize, y: usize) -> Option<BlockId> {
    match map.get_cell(x, y) {
        Cell::Block(id) => Some(id),
        _ => None,
    }
}

fn block_color(id: BlockId) -> Color {
    const COLORS: [Color; 6] = [
        Color::new(0.15, 0.40, 0.48, 1.0),
        Color::new(0.46, 0.28, 0.18, 1.0),
        Color::new(0.30, 0.23, 0.48, 1.0),
        Color::new(0.22, 0.42, 0.23, 1.0),
        Color::new(0.48, 0.22, 0.32, 1.0),
        Color::new(0.40, 0.39, 0.16, 1.0),
    ];
    let mut hash = DefaultHasher::new();
    id.hash(&mut hash);
    COLORS[hash.finish() as usize % COLORS.len()]
}

fn draw_trace(trace: &path_debug::Trace, tile_size: f32, color: Color, width: f32) {
    let screen = |p: Vec2| vec2(p.x * tile_size, TOP + p.y * tile_size);
    for segment in trace.points.windows(2) {
        let a = screen(segment[0]);
        let b = screen(segment[1]);
        draw_line(a.x, a.y, b.x, b.y, width, color);
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut map = rts_map();
    let mut map_kind = 4;
    let mut options = FlowFieldOptions::default();
    let mut start = Pos { x: 248, y: 248 };
    let mut route_key = None;
    let mut routes: Option<(path_debug::Trace, path_debug::Trace, path_debug::Trace)> = None;
    let mut goal = Pos { x: 8, y: 8 };
    let mut field = FlowField::new(goal, &options, &map).unwrap();
    let mut buildings = std::collections::HashMap::new();
    let mut show_blocks = true;
    let mut show_flow = true;
    let mut show_astar = false;
    loop {
        let selected = if is_key_pressed(KeyCode::Key1) {
            Some(1)
        } else if is_key_pressed(KeyCode::Key2) {
            Some(2)
        } else if is_key_pressed(KeyCode::Key3) {
            Some(3)
        } else if is_key_pressed(KeyCode::Key4) {
            Some(4)
        } else {
            None
        };
        if let Some(selected) = selected {
            map_kind = selected;
            map = match map_kind {
                1 => demo_map(),
                2 => complex_map(),
                4 => rts_map(),
                _ => staircase_map(),
            };
            goal = if map_kind == 4 { Pos { x: 8, y: 8 } } else { Pos { x: 2, y: 2 } };
            start = if map_kind == 4 { Pos { x: 248, y: 248 } } else { Pos { x: 7, y: 2 } };
            field = FlowField::new(goal, &options, &map).unwrap();
            buildings.clear();
        }
        if is_key_pressed(KeyCode::B) {
            show_blocks = !show_blocks;
        }
        if is_key_pressed(KeyCode::F) {
            show_flow = !show_flow;
        }
        if is_key_pressed(KeyCode::A) {
            show_astar = !show_astar;
        }
        if is_key_pressed(KeyCode::R) {
            options.wall_repulsion_strength = if options.wall_repulsion_strength == 0.0 { 0.5 } else { 0.0 };
            field = FlowField::new(goal, &options, &map).unwrap();
            route_key = None;
        }
        let tile_size = (screen_width() / map.width as f32)
            .min((screen_height() - TOP) / map.height as f32);
        let add_wall = is_mouse_button_pressed(MouseButton::Left);
        let change_start = add_wall && (is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift));
        let change_goal = is_mouse_button_pressed(MouseButton::Right);
        let remove_wall = is_mouse_button_pressed(MouseButton::Middle);
        if add_wall || change_goal || remove_wall {
            let (mx, my) = mouse_position();
            if mx >= 0.0 && my >= TOP {
                let pos = Pos {
                    x: (mx / tile_size) as usize,
                    y: ((my - TOP) / tile_size) as usize,
                };
                if remove_wall {
                    // 기본 지형은 유지하고 이 화면에서 추가한 건물만 삭제한다.
                    if let Some(&id) = buildings.get(&pos) {
                        map.remove_building(id).unwrap();
                        buildings.remove(&pos);
                    }
                } else if matches!(map.get_cell(pos.x, pos.y), Cell::Block(_)) {
                    if change_start {
                        start = pos;
                    } else if add_wall {
                        // 출발지와 목표는 유지한다.
                        if pos != goal && pos != start {
                            let mut obstacle = Obstacle::new(map.width, map.height);
                            obstacle.set_cell_blocked(pos.x, pos.y);
                            match map.add_building(obstacle) {
                                Ok(id) => { buildings.insert(pos, id); }
                                Err(error) => eprintln!("Cannot place building: {error}"),
                            }
                        }
                    } else if change_goal {
                        goal = pos;
                        field = FlowField::new(goal, &options, &map).unwrap();
                    }
                }
            }
        }
        let flow = field.get_valid_field(&map).unwrap();
        let key = (start, goal, map.get_generation(), map_kind);
        if route_key != Some(key) {
            let grid_flow = path_debug::grid_flow(&map, goal, &options);
            routes = Some((
                path_debug::trace_flow(&map, flow, start, goal),
                path_debug::trace_flow(&map, &grid_flow, start, goal),
                path_debug::astar(&map, start, goal),
            ));
            route_key = Some(key);
        }
        clear_background(Color::new(0.06, 0.07, 0.09, 1.0));
        draw_text(&format!("Map {} | {} blocks / {} portals | gen {} | B: blocks | F: vectors",
            map_kind, map.blocks.len(), map.portals.len(), map.get_generation()), 10.0, 20.0, 17.0, WHITE);
        draw_text("1/2/3/4: maps (4: RTS 256) | Shift+Left: start | Right: goal",
            10.0, 41.0, 17.0, WHITE);
        draw_text(&format!("Left: wall | Middle: remove | R: repulsion ({}) | A: A* ({})",
            if options.wall_repulsion_strength == 0.0 { "off" } else { "on" },
            if show_astar { "on" } else { "off" }),
            10.0, 62.0, 16.0, LIGHTGRAY);
        if let Some((trace, grid, reference)) = &routes {
            let ratio = if trace.status == path_debug::Status::Reached
                && grid.status == path_debug::Status::Reached && grid.length() > 0.0 {
                format!(" | Block/Grid {:.3}", trace.length() / grid.length())
            } else { String::new() };
            draw_text(&format!("Cyan Block: {} {:.2} | Yellow Grid: {} {:.2}{}",
                trace.status.label(), trace.length(), grid.status.label(), grid.length(), ratio),
                10.0, 84.0, 16.0, WHITE);
            draw_text(&format!("Magenta A*: {} {:.2} cells | Both flows use the same repulsion and tracer",
                reference.status.label(), reference.length()), 10.0, 105.0, 15.0, LIGHTGRAY);
        }
        draw_text("Point agent, 0.08-cell steps; lengths in cells. A* is grid-optimal, not continuous-optimal.",
            10.0, 126.0, 14.0, LIGHTGRAY);
        for y in 0..map.height {
            for x in 0..map.width {
                let px = x as f32 * tile_size;
                let py = TOP + y as f32 * tile_size;
                let Some(id) = block_at(&map, x, y) else {
                    draw_rectangle(px, py, tile_size, tile_size, Color::new(0.19, 0.20, 0.22, 1.0));
                    continue;
                };
                draw_rectangle(px, py, tile_size, tile_size, block_color(id));
                if tile_size >= 10.0 {
                    draw_rectangle_lines(px, py, tile_size, tile_size, 0.5, Color::new(1.0, 1.0, 1.0, 0.12));
                }
                // 직사각형 Block의 실제 타일 경계를 표시한다.
                let borders = [
                    (
                        y == 0 || block_at(&map, x, y - 1) != Some(id),
                        px,
                        py,
                        px + tile_size,
                        py,
                    ),
                    (
                        block_at(&map, x, y + 1) != Some(id),
                        px,
                        py + tile_size,
                        px + tile_size,
                        py + tile_size,
                    ),
                    (
                        x == 0 || block_at(&map, x - 1, y) != Some(id),
                        px,
                        py,
                        px,
                        py + tile_size,
                    ),
                    (
                        block_at(&map, x + 1, y) != Some(id),
                        px + tile_size,
                        py,
                        px + tile_size,
                        py + tile_size,
                    ),
                ];
                for (visible, x1, y1, x2, y2) in borders {
                    if visible {
                        draw_line(x1, y1, x2, y2, 2.0, WHITE);
                    }
                }
                if show_flow && tile_size >= 10.0 {
                    if let Some((dx, dy)) = flow[y * map.width + x] {
                        let cx = px + tile_size / 2.0;
                        let cy = py + tile_size / 2.0;
                        draw_line(cx, cy, cx + dx * tile_size * 0.4, cy + dy * tile_size * 0.4, 2.0, RED);
                    }
                }
            }
        }
        for block in map.blocks.values() {
            let px = block.x as f32 * tile_size;
            let py = TOP + block.y as f32 * tile_size;
            if show_blocks && tile_size >= 10.0 {
                draw_rectangle_lines(
                    px + 2.0,
                    py + 2.0,
                    block.width as f32 * tile_size - 4.0,
                    block.height as f32 * tile_size - 4.0,
                    1.0,
                    YELLOW,
                );
            }
            if tile_size >= 12.0 {
            draw_text(
                &format!("{:?}", block.id),
                px + 5.0,
                py + 18.0,
                16.0,
                WHITE,
            );
            }
        }
        draw_blocked_edges(&map, tile_size);
        if let Some((trace, grid, reference)) = &routes {
            if show_astar { draw_trace(reference, tile_size, MAGENTA, 5.0); }
            draw_trace(grid, tile_size, YELLOW, 3.5);
            draw_trace(trace, tile_size, SKYBLUE, 2.0);
            for (route, color) in [(trace, SKYBLUE), (grid, YELLOW)] {
                if route.status != path_debug::Status::Reached {
                    let end = *route.points.last().unwrap();
                    draw_circle_lines(end.x * tile_size, TOP + end.y * tile_size, 6.0, 2.0, RED);
                }
                if !route.points.is_empty() {
                    let i = ((get_time() * 35.0) as usize) % route.points.len();
                    let p = route.points[i];
                    draw_circle(p.x * tile_size, TOP + p.y * tile_size, 3.0, color);
                }
            }
        }
        let s = path_debug::center(start);
        let g = path_debug::center(goal);
        draw_circle(s.x * tile_size, TOP + s.y * tile_size, 5.0, SKYBLUE);
        draw_circle(g.x * tile_size, TOP + g.y * tile_size, 5.0, LIME);
        next_frame().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rts_rooms_remain_connected_but_some_neighbors_require_detours() {
        let map = rts_map();
        // 가로 인접 방 6-7과 세로 인접 방 2-6 사이에는 직접 연결이 없다.
        assert_eq!(map.get_cell(192, 98), Cell::Wall);
        assert_eq!(map.get_cell(162, 64), Cell::Wall);
        assert!(matches!(map.get_cell(64, 34), Cell::Block(_)));

        let start = Pos { x: 190, y: 98 };
        let goal = Pos { x: 194, y: 98 };
        let route = path_debug::astar(&map, start, goal);
        assert_eq!(route.status, path_debug::Status::Reached);
        assert!(route.length() > 64.0, "blocked neighbors must require a detour");

        // 실제 셀을 4방향 BFS로 탐색해 모든 통행 가능 셀의 연결을 검증한다.
        let mut visited = vec![false; map.width * map.height];
        let first = Pos { x: 8, y: 8 };
        let mut queue = std::collections::VecDeque::from([first]);
        visited[first.y * map.width + first.x] = true;
        while let Some(pos) = queue.pop_front() {
            for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                let (Some(x), Some(y)) = (pos.x.checked_add_signed(dx), pos.y.checked_add_signed(dy)) else { continue; };
                if !matches!(map.get_cell(x, y), Cell::Block(_)) { continue; }
                let index = y * map.width + x;
                if !visited[index] {
                    visited[index] = true;
                    queue.push_back(Pos { x, y });
                }
            }
        }
        for y in 0..map.height {
            for x in 0..map.width {
                if matches!(map.get_cell(x, y), Cell::Block(_)) {
                    assert!(visited[y * map.width + x], "unreachable cell ({x}, {y})");
                }
            }
        }
    }

    #[test]
    fn rts_preview_paths_are_computed_without_a_window() {
        let map = rts_map();
        let start = Pos { x: 248, y: 248 };
        let goal = Pos { x: 8, y: 8 };
        let reference = path_debug::astar(&map, start, goal);
        assert_eq!(reference.status, path_debug::Status::Reached);
        for strength in [0.0, 0.5] {
            let options = FlowFieldOptions { wall_repulsion_strength: strength, ..Default::default() };
            let flow = map.build_flow_field(goal, &options).unwrap();
            let trace = path_debug::trace_flow(&map, &flow, start, goal);
            assert!(trace.length().is_finite());
            assert_eq!(trace.points.first(), Some(&path_debug::center(start)));
            assert!(trace.points.iter().all(|p| matches!(
                map.get_cell(p.x.floor() as usize, p.y.floor() as usize), Cell::Block(_))));
            // Reaching is deliberately not required: this UI must expose failures too.
            eprintln!("RTS repulsion={strength}: flow={} length={:.2}, A* length={:.2}",
                trace.status.label(), trace.length(), reference.length());
            let grid_flow = path_debug::grid_flow(&map, goal, &options);
            assert_eq!(grid_flow.len(), flow.len());
            for (chunk, grid) in flow.iter().zip(&grid_flow) {
                assert_eq!(chunk.is_some(), grid.is_some());
            }
            let grid_trace = path_debug::trace_flow(&map, &grid_flow, start, goal);
            assert!(grid_trace.length().is_finite());
            eprintln!("RTS grid repulsion={strength}: {} length={:.2}",
                grid_trace.status.label(), grid_trace.length());
        }
    }

    #[test]
    fn adding_wall_invalidates_and_refreshes_flow() {
        let mut map = staircase_map();
        let goal = Pos { x: 2, y: 2 };
        let wall = Pos { x: 3, y: 2 };
        let mut field = FlowField::new(goal, &FlowFieldOptions::default(), &map).unwrap();
        let index = wall.y * map.width + wall.x;
        assert!(field.get_valid_field(&map).unwrap()[index].is_some());
        let generation = map.get_generation();

        let mut obstacle = Obstacle::new(map.width, map.height);
        obstacle.set_cell_blocked(wall.x, wall.y);
        map.add_building(obstacle).unwrap();

        assert_eq!(map.get_generation(), generation + 1);
        assert_eq!(map.get_cell(wall.x, wall.y), Cell::Wall);
        assert!(!field.is_valid(&map));
        let flow = field.get_valid_field(&map).unwrap();
        assert_eq!(flow[index], None);
        assert_eq!(flow[goal.y * map.width + goal.x], Some((0.0, 0.0)));
        assert!(field.is_valid(&map));
    }

    #[test]
    fn staircase_regions_keep_rectangular_blocks_and_goal_block_steers_directly() {
        let map = staircase_map();
        let options = FlowFieldOptions {
            wall_repulsion_strength: 0.0,
            ..Default::default()
        };
        assert!(map.blocks.len() > 3);
        for region in 0..3 {
            let mut blocks = std::collections::BTreeSet::new();
            let mut tiles = Vec::new();
            for y in 0..ROWS {
                for x in 0..COLS {
                    if staircase_region(x, y) == Some(region) {
                        if let Cell::Block(id) = map.get_cell(x, y) {
                            blocks.insert(id);
                        }
                        tiles.push(Pos { x, y });
                    }
                }
            }
            assert!(blocks.len() > 1);
            let goal = tiles[0];
            let field = map.build_flow_field(goal, &options).unwrap();
            for pos in tiles {
                let actual = field[pos.y * COLS + pos.x].unwrap();
                if block_at(&map, pos.x, pos.y) != block_at(&map, goal.x, goal.y) { continue; }
                let dx = goal.x as f32 - pos.x as f32;
                let dy = goal.y as f32 - pos.y as f32;
                let len = dx.hypot(dy);
                let expected = if len == 0.0 {
                    (0.0, 0.0)
                } else {
                    (dx / len, dy / len)
                };
                assert!((actual.0 - expected.0).abs() < 1e-6);
                assert!((actual.1 - expected.1).abs() < 1e-6);
            }
        }
    }
    #[test]
    fn complex_map_has_connected_walkable_tiles_and_edge_gaps() {
        let map = complex_map();
        let field = map
            .build_flow_field(Pos { x: 2, y: 2 }, &FlowFieldOptions::default())
            .unwrap();
        for y in 0..ROWS {
            for x in 0..COLS {
                if matches!(map.get_cell(x, y), Cell::Block(_)) {
                    assert!(
                        field[y * COLS + x].is_some(),
                        "unreachable tile ({x}, {y})"
                    );
                }
            }
        }
        assert!(!map.can_pass(4, 2, EDGE_RIGHT));
        assert!(map.can_pass(4, 5, EDGE_RIGHT));
        assert!(!map.can_pass(9, 7, EDGE_BOTTOM));
        assert!(map.can_pass(12, 7, EDGE_BOTTOM));
        assert!(!map.can_pass(19, 12, EDGE_RIGHT));
        assert!(map.can_pass(19, 16, EDGE_RIGHT));
    }
    #[test]
    fn demo_covers_rectangular_partition_and_edge_cases() {
        let map = demo_map();
        let mut staircase_blocks = std::collections::BTreeSet::new();
        for y in 1..=9 {
            for x in 1..=9 {
                if x + y <= 10 {
                    if let Cell::Block(id) = map.get_cell(x, y) {
                        staircase_blocks.insert(id);
                    }
                }
            }
        }
        assert!(staircase_blocks.len() > 1);
        assert_ne!(block_at(&map, 14, 1), block_at(&map, 22, 9));
        assert_ne!(block_at(&map, 11, 12), block_at(&map, 12, 12));
        assert!(!map.can_pass(11, 12, EDGE_RIGHT));
        assert!(map.can_pass(11, 16, EDGE_RIGHT));
    }
}
