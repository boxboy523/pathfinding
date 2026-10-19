use chunk_flow_field::{
    map::{EDGE_BOTTOM, EDGE_RIGHT, Map, Obstacle, Side},
    types::{Cell, FlowFieldOptions, Pos},
};
use macroquad::prelude::*;

const COLS: usize = 24;
const ROWS: usize = 22;
const SIZE: f32 = 32.0;
const TOP: f32 = 64.0;

fn window_conf() -> Conf {
    Conf {
        window_title: "Chunk Merge Debug".into(),
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

fn draw_blocked_edges(map: &Map) {
    // 모든 타일/Block/라벨을 그린 뒤 별도 패스로 그려 덮이지 않게 한다.
    for y in 0..ROWS {
        for x in 0..COLS {
            if !matches!(map.get_cell(x, y), Cell::Block(_)) {
                continue;
            }
            let px = x as f32 * SIZE;
            let py = TOP + y as f32 * SIZE;
            let edges = [
                (EDGE_RIGHT, px + SIZE, py, px + SIZE, py + SIZE),
                (EDGE_BOTTOM, px, py + SIZE, px + SIZE, py + SIZE),
            ];
            for (mask, x1, y1, x2, y2) in edges {
                if map.can_pass(x, y, mask) {
                    continue;
                }
                draw_line(x1, y1, x2, y2, 10.0, BLACK);
                draw_line(x1, y1, x2, y2, 6.0, ORANGE);
                draw_line(x1, y1, x2, y2, 1.5, WHITE);
                draw_circle(x1, y1, 3.0, ORANGE);
                draw_circle(x2, y2, 3.0, ORANGE);
            }
        }
    }
}

fn chunk_at(map: &Map, x: usize, y: usize) -> Option<usize> {
    match map.get_cell(x, y) {
        Cell::Block(id) => map.blocks[id].chunk_id,
        _ => None,
    }
}

fn chunk_color(id: usize) -> Color {
    const COLORS: [Color; 6] = [
        Color::new(0.15, 0.40, 0.48, 1.0),
        Color::new(0.46, 0.28, 0.18, 1.0),
        Color::new(0.30, 0.23, 0.48, 1.0),
        Color::new(0.22, 0.42, 0.23, 1.0),
        Color::new(0.48, 0.22, 0.32, 1.0),
        Color::new(0.40, 0.39, 0.16, 1.0),
    ];
    COLORS[id % COLORS.len()]
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut map = staircase_map();
    let mut map_kind = 3;
    let options = FlowFieldOptions::default();
    let mut goal = Pos { x: 2, y: 2 };
    let mut field = map.build_flow_field(goal, &options).unwrap();
    let mut show_blocks = true;
    let mut show_flow = true;
    loop {
        let selected = if is_key_pressed(KeyCode::Key1) {
            Some(1)
        } else if is_key_pressed(KeyCode::Key2) {
            Some(2)
        } else if is_key_pressed(KeyCode::Key3) {
            Some(3)
        } else {
            None
        };
        if let Some(selected) = selected {
            map_kind = selected;
            map = match map_kind {
                1 => demo_map(),
                2 => complex_map(),
                _ => staircase_map(),
            };
            goal = Pos { x: 2, y: 2 };
            field = map.build_flow_field(goal, &options).unwrap();
        }
        if is_key_pressed(KeyCode::B) {
            show_blocks = !show_blocks;
        }
        if is_key_pressed(KeyCode::F) {
            show_flow = !show_flow;
        }
        if is_mouse_button_pressed(MouseButton::Left) {
            let (mx, my) = mouse_position();
            if mx >= 0.0 && my >= TOP {
                let pos = Pos {
                    x: (mx / SIZE) as usize,
                    y: ((my - TOP) / SIZE) as usize,
                };
                if matches!(map.get_cell(pos.x, pos.y), Cell::Block(_)) {
                    goal = pos;
                    field = map.build_flow_field(goal, &options).unwrap();
                }
            }
        }
        clear_background(Color::new(0.06, 0.07, 0.09, 1.0));
        draw_text(
            &format!(
                "{} blocks / {} chunks | B: blocks | F: flow | Click: goal",
                map.blocks.len(),
                map.chunks.len()
            ),
            10.0,
            23.0,
            19.0,
            WHITE,
        );
        draw_text(
            match map_kind {
                1 => "1: Simple / 2: Complex / 3: Stairs | Staircase / L / edge room",
                2 => "1: Simple / 2: Complex / 3: Stairs | Orange: blocked edge",
                _ => "1: Simple / 2: Complex / 3: Stairs | Each stair region should be ONE chunk",
            },
            10.0,
            47.0,
            16.0,
            LIGHTGRAY,
        );
        for y in 0..ROWS {
            for x in 0..COLS {
                let px = x as f32 * SIZE;
                let py = TOP + y as f32 * SIZE;
                let Some(id) = chunk_at(&map, x, y) else {
                    draw_rectangle(px, py, SIZE, SIZE, Color::new(0.19, 0.20, 0.22, 1.0));
                    continue;
                };
                draw_rectangle(px, py, SIZE, SIZE, chunk_color(id));
                draw_rectangle_lines(px, py, SIZE, SIZE, 0.5, Color::new(1.0, 1.0, 1.0, 0.12));
                // Chunk의 실제 타일 경계만 표시한다.
                let borders = [
                    (
                        y == 0 || chunk_at(&map, x, y - 1) != Some(id),
                        px,
                        py,
                        px + SIZE,
                        py,
                    ),
                    (
                        chunk_at(&map, x, y + 1) != Some(id),
                        px,
                        py + SIZE,
                        px + SIZE,
                        py + SIZE,
                    ),
                    (
                        x == 0 || chunk_at(&map, x - 1, y) != Some(id),
                        px,
                        py,
                        px,
                        py + SIZE,
                    ),
                    (
                        chunk_at(&map, x + 1, y) != Some(id),
                        px + SIZE,
                        py,
                        px + SIZE,
                        py + SIZE,
                    ),
                ];
                for (visible, x1, y1, x2, y2) in borders {
                    if visible {
                        draw_line(x1, y1, x2, y2, 2.0, WHITE);
                    }
                }
                if show_flow {
                    if let Some((dx, dy)) = field.flow[y * COLS + x] {
                        let cx = px + SIZE / 2.0;
                        let cy = py + SIZE / 2.0;
                        draw_line(cx, cy, cx + dx * SIZE * 0.4, cy + dy * SIZE * 0.4, 2.0, RED);
                    }
                }
            }
        }
        for block in &map.blocks {
            let px = block.x as f32 * SIZE;
            let py = TOP + block.y as f32 * SIZE;
            if show_blocks {
                draw_rectangle_lines(
                    px + 2.0,
                    py + 2.0,
                    block.width as f32 * SIZE - 4.0,
                    block.height as f32 * SIZE - 4.0,
                    1.0,
                    YELLOW,
                );
            }
            draw_text(
                &format!("C{}", block.chunk_id.unwrap()),
                px + 5.0,
                py + 18.0,
                16.0,
                WHITE,
            );
        }
        draw_blocked_edges(&map);
        draw_circle(
            (goal.x as f32 + 0.5) * SIZE,
            TOP + (goal.y as f32 + 0.5) * SIZE,
            6.0,
            LIME,
        );
        next_frame().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn staircase_regions_merge_and_share_direct_goal_flow() {
        let map = staircase_map();
        let options = FlowFieldOptions {
            wall_repulsion_strength: 0.0,
            ..Default::default()
        };
        assert_eq!(map.chunks.len(), 3);
        for region in 0..3 {
            let mut chunks = std::collections::BTreeSet::new();
            let mut blocks = std::collections::BTreeSet::new();
            let mut tiles = Vec::new();
            for y in 0..ROWS {
                for x in 0..COLS {
                    if staircase_region(x, y) == Some(region) {
                        chunks.insert(chunk_at(&map, x, y).unwrap());
                        if let Cell::Block(id) = map.get_cell(x, y) {
                            blocks.insert(id);
                        }
                        tiles.push(Pos { x, y });
                    }
                }
            }
            assert_eq!(chunks.len(), 1, "region {region} split across chunks");
            assert!(blocks.len() > 1);
            let goal = tiles[0];
            let field = map.build_flow_field(goal, &options).unwrap();
            for pos in tiles {
                let actual = field.flow[pos.y * COLS + pos.x].unwrap();
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
                        field.flow[y * COLS + x].is_some(),
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
    fn demo_covers_merge_rejection_and_edge_cases() {
        let map = demo_map();
        let mut staircase_chunks = std::collections::BTreeSet::new();
        let mut staircase_blocks = std::collections::BTreeSet::new();
        for y in 1..=9 {
            for x in 1..=9 {
                if x + y <= 10 {
                    staircase_chunks.insert(chunk_at(&map, x, y).unwrap());
                    if let Cell::Block(id) = map.get_cell(x, y) {
                        staircase_blocks.insert(id);
                    }
                }
            }
        }
        // 중간 모양의 빈틈을 다른 Block으로 채워 계단 전체를 병합한다.
        assert_eq!(staircase_chunks.len(), 1);
        assert!(staircase_chunks.len() < staircase_blocks.len());
        assert_ne!(chunk_at(&map, 14, 1), chunk_at(&map, 22, 9));
        assert_ne!(chunk_at(&map, 11, 12), chunk_at(&map, 12, 12));
        assert!(!map.can_pass(11, 12, EDGE_RIGHT));
        assert!(map.can_pass(11, 16, EDGE_RIGHT));
    }
}
