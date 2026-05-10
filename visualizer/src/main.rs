use chunk_flow_field::{map::Map, types::Pos};
use macroquad::prelude::*;

#[macroquad::main("Flow Field Debug")]
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

    let map = Map::new(20, 20, &wall);
    println!("intersect test: {:?}", map.get_neighbors(1));
    // for i in 0..1000 {
    //     let new_map = Map::new(20, 20, wall.clone());
    //     if new_map.blocks.len() < map.blocks.len() {
    //         map = new_map;
    //     }
    // }
    let field = map.build_flow_field(Pos { x: 3, y: 3 }).unwrap();
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
