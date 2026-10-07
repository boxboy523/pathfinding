use chunk_flow_field::{
    map::{EDGE_BOTTOM, EDGE_RIGHT, Map, Obstacle, Side}, types::{FlowFieldOptions, Pos},
};
use macroquad::prelude::*;

fn window_conf() -> Conf {
    Conf {
        window_title: "Flow Field Debug".into(),
        platform: miniquad::conf::Platform {
            linux_backend: miniquad::conf::LinuxBackend::WaylandOnly,
            ..Default::default()
        },
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let wall = vec![
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1,
        1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1,
        1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
        1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
        1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
    ]
    .into_iter()
    .map(|x| x == 1)
    .collect::<Vec<bool>>();

    let mut obstacle = Obstacle::new(20, 20);
    for (index, &blocked) in wall.iter().enumerate() {
        if blocked {
            obstacle.set_cell_blocked(index % 20, index / 20);
        }
    }

    // 높이 차이로 생긴 절벽을 가정한 edge다. 중간의 빈 구간은 slope 또는 통로로 사용한다.
    for y in 1..9 {
        if y != 3 && !wall[y * 20 + 8] && !wall[y * 20 + 9] {
            obstacle.set_edge(8, y, Side::Right);
        }
    }
    for x in 1..9 {
        if x != 6 && !wall[9 * 20 + x] && !wall[10 * 20 + x] {
            obstacle.set_edge(x, 9, Side::Bottom);
        }
    }
    let map = Map::new(&obstacle);
    println!("intersect test: {:?}", map.get_neighbors(1));
    // for i in 0..1000 {
    //     let new_map = Map::new(20, 20, wall.clone());
    //     if new_map.blocks.len() < map.blocks.len() {
    //         map = new_map;
    //     }
    // }
    let option = FlowFieldOptions {
        wall_margin: 1.5,
        wall_repulsion_strength: 0.4,
    };
    let field = map.build_flow_field(Pos { x: 3, y: 3 }, &option).unwrap();
    loop {
        clear_background(BLACK);

        let cell_size = 40.0;
        let cols = 20;
        let rows = 20;

        for y in 0..rows {
            for x in 0..cols {
                if wall[y * cols + x] {
                    draw_rectangle(
                        x as f32 * cell_size,
                        y as f32 * cell_size,
                        cell_size,
                        cell_size,
                        GRAY,
                    );
                    continue;
                }

                let px = x as f32 * cell_size;
                let py = y as f32 * cell_size;

                draw_rectangle_lines(px, py, cell_size, cell_size, 1.0, DARKGRAY);

                if !map.can_pass(x, y, EDGE_RIGHT) {
                    draw_line(
                        px + cell_size,
                        py,
                        px + cell_size,
                        py + cell_size,
                        4.0,
                        BLUE,
                    );
                }
                if !map.can_pass(x, y, EDGE_BOTTOM) {
                    draw_line(
                        px,
                        py + cell_size,
                        px + cell_size,
                        py + cell_size,
                        4.0,
                        BLUE,
                    );
                }

                let dir = field.flow[y * cols + x].unwrap_or((0.0, 0.0));
                let len = cell_size / 2.0;
                let center = Vec2::new(px + cell_size / 2.0, py + cell_size / 2.0);
                draw_line(
                    center.x,
                    center.y,
                    center.x + dir.0 * len,
                    center.y + dir.1 * len,
                    2.0,
                    RED,
                );
            }
        }

        for block in &map.blocks {
            let px = block.x as f32 * cell_size;
            let py = block.y as f32 * cell_size;
            let pw = block.width as f32 * cell_size;
            let ph = block.height as f32 * cell_size;

            draw_rectangle_lines(px, py, pw, ph, 2.0, YELLOW);
        }

        next_frame().await
    }
}
