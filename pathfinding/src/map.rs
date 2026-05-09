use std::collections::{BinaryHeap, HashSet};

use crate::{
    block::Block,
    types::{Cell, FlowField, OrthogonalLine, Pos, VisitNode},
};

#[derive(Debug)]
pub struct Map {
    pub width: usize,
    pub height: usize,
    block_map: Vec<Cell>,
    pub blocks: Vec<Block>,
}

impl Map {
    pub fn new(width: usize, height: usize, wall: Vec<bool>) -> Self {
        let mut block_map = vec![Cell::Empty; width * height];
        for i in 0..width * height {
            if wall[i] {
                block_map[i] = Cell::Wall;
            }
        }
        let blocks = vec![];
        let mut map = Map {
            width,
            height,
            block_map,
            blocks,
        };
        map.random_expand();
        map
    }

    pub fn get_cell(&self, x: usize, y: usize) -> Cell {
        self.block_map[y * self.width + x]
    }

    pub fn get_neighbors(&self, block_id: usize) -> HashSet<(usize, OrthogonalLine)> {
        let mut neighbors = HashSet::new();
        let block = &self.blocks[block_id];
        if block.y > 0 {
            let mut x = block.x;
            while x < block.x + block.width {
                if let Cell::Block(neighbor_id) = self.get_cell(x, block.y - 1) {
                    let neighbor_block = &self.blocks[neighbor_id];
                    let from = block.x.max(neighbor_block.x);
                    let to = (block.x + block.width).min(neighbor_block.x + neighbor_block.width);
                    if from < to {
                        let overlap = OrthogonalLine {
                            is_vertical: false,
                            fixed: block.y,
                            from,
                            to,
                        };
                        neighbors.insert((neighbor_id, overlap));
                        x += overlap.len().max(1);
                    } else {
                        x += 1;
                    }
                } else {
                    x += 1;
                }
            }
        }
        if block.y + block.height < self.height {
            let mut x = block.x;
            while x < block.x + block.width {
                if let Cell::Block(neighbor_id) = self.get_cell(x, block.y + block.height) {
                    let neighbor_block = &self.blocks[neighbor_id];
                    let from = block.x.max(neighbor_block.x);
                    let to = (block.x + block.width).min(neighbor_block.x + neighbor_block.width);
                    if from < to {
                        let overlap = OrthogonalLine {
                            is_vertical: false,
                            fixed: block.y + block.height - 1,
                            from,
                            to,
                        };
                        neighbors.insert((neighbor_id, overlap));
                        x += overlap.len().max(1);
                    } else {
                        x += 1;
                    }
                } else {
                    x += 1;
                }
            }
        }
        if block.x > 0 {
            let mut y = block.y;
            while y < block.y + block.height {
                if let Cell::Block(neighbor_id) = self.get_cell(block.x - 1, y) {
                    let neighbor_block = &self.blocks[neighbor_id];
                    let from = block.y.max(neighbor_block.y);
                    let to = (block.y + block.height).min(neighbor_block.y + neighbor_block.height);
                    if from < to {
                        let overlap = OrthogonalLine {
                            is_vertical: true,
                            fixed: block.x,
                            from,
                            to,
                        };
                        neighbors.insert((neighbor_id, overlap));
                        y += overlap.len().max(1);
                    } else {
                        y += 1;
                    }
                } else {
                    y += 1;
                }
            }
        }
        if block.x + block.width < self.width {
            let mut y = block.y;
            while y < block.y + block.height {
                if let Cell::Block(neighbor_id) = self.get_cell(block.x + block.width, y) {
                    let neighbor_block = &self.blocks[neighbor_id];
                    let from = block.y.max(neighbor_block.y);
                    let to = (block.y + block.height).min(neighbor_block.y + neighbor_block.height);
                    if from < to {
                        let overlap = OrthogonalLine {
                            is_vertical: true,
                            fixed: block.x + block.width - 1,
                            from,
                            to,
                        };
                        neighbors.insert((neighbor_id, overlap));
                        y += overlap.len().max(1);
                    } else {
                        y += 1;
                    }
                } else {
                    y += 1;
                }
            }
        }
        neighbors
    }

    fn random_expand(&mut self) {
        let mut empty_cells = HashSet::new();
        for y in 0..self.height {
            for x in 0..self.width {
                if self.get_cell(x, y) == Cell::Empty {
                    empty_cells.insert((x, y));
                }
            }
        }
        while let Some(&(x, y)) = empty_cells.iter().next() {
            let new_block_id = self.blocks.len();
            let mut new_block = Block {
                x,
                y,
                width: 1,
                height: 1,
            };
            while new_block.expand(self).is_some() {}
            for yy in new_block.y..new_block.y + new_block.height {
                for xx in new_block.x..new_block.x + new_block.width {
                    self.block_map[yy * self.width + xx] = Cell::Block(new_block_id);
                    empty_cells.remove(&(xx, yy));
                }
            }
            self.blocks.push(new_block);
        }
    }

    fn block_route(
        &self,
        goal: Pos,
    ) -> anyhow::Result<Vec<Option<(f32, OrthogonalLine, Option<usize>)>>> {
        let mut pq = BinaryHeap::new();
        let mut visited = vec![None; self.blocks.len()]; // block_id -> (cost, entry, from)

        let goal_block_id = match self.get_cell(goal.x, goal.y) {
            Cell::Block(id) => id,
            _ => anyhow::bail!("Goal is not on a block"),
        };

        pq.push(VisitNode {
            block_id: goal_block_id,
            entry: OrthogonalLine {
                is_vertical: false,
                fixed: goal.y,
                from: goal.x,
                to: goal.x + 1,
            },
            cost: 0.0,
            from: None,
        });

        while let Some(node) = pq.pop() {
            if visited[node.block_id].is_some() {
                continue;
            }
            visited[node.block_id] = Some((node.cost, node.entry, node.from));

            for (neighbor_id, line) in self.get_neighbors(node.block_id) {
                if visited[neighbor_id].is_some() {
                    continue;
                }
                let boundary_mid = line.mid_point();
                let entry_mid = node.entry.mid_point();
                let diff = (
                    (boundary_mid.x as isize - entry_mid.x as isize).abs() as usize,
                    (boundary_mid.y as isize - entry_mid.y as isize).abs() as usize,
                );
                let dist = ((diff.0 * diff.0 + diff.1 * diff.1) as f32).sqrt();
                pq.push(VisitNode {
                    block_id: neighbor_id,
                    entry: line,
                    cost: node.cost + dist,
                    from: Some(node.block_id),
                });
            }
        }
        Ok(visited)
    }

    pub fn build_flow_field(&self, goal: Pos) -> anyhow::Result<FlowField> {
        let block_routes = self.block_route(goal)?;
        let mut flow = vec![None; self.width * self.height];
        for x in 0..self.width {
            for y in 0..self.height {
                let cell = self.get_cell(x, y);
                let cell_pos = Pos { x, y };
                if let Cell::Block(block_id) = cell {
                    if let Some((_, line, from_block)) = block_routes[block_id] {
                        let nearest = line.nearest_point(cell_pos); // nearest point on the entry line to the cell
                        if nearest == cell_pos {
                            // cell is on the entry line
                            if let Some(from_block_id) = from_block {
                                flow[y * self.width + x] = Some(
                                    self.get_norm(cell_pos, self.blocks[from_block_id].center()), // direction towards the center of the next block
                                );
                            } else {
                                flow[y * self.width + x] = Some(self.get_norm(cell_pos, goal));
                                // this is the goal block, direction towards the goal
                            }
                        } else {
                            flow[y * self.width + x] = Some(self.get_norm(cell_pos, nearest));
                            // direction towards the nearest point on the entry line
                        }
                    } else {
                        flow[y * self.width + x] = None; // unreachable block
                    }
                }
            }
        }
        Ok(FlowField { goal, flow })
    }

    fn get_norm(&self, from: Pos, to: Pos) -> (f32, f32) {
        if from == to {
            return (0.0, 0.0);
        }
        let dx = (to.x as isize - from.x as isize).abs() as usize;
        let dy = (to.y as isize - from.y as isize).abs() as usize;
        let dist = ((dx * dx + dy * dy) as f32).sqrt();
        (
            (to.x as f32 - from.x as f32) / dist,
            (to.y as f32 - from.y as f32) / dist,
        )
    }
}
