use std::collections::{BinaryHeap, HashSet};

use crate::{
    block::Block,
    types::{BlockRoute, Cell, FlowField, OrthogonalLine, Pos, VisitNode},
};

pub const EDGE_TOP: u8 = 0b0001;
pub const EDGE_RIGHT: u8 = 0b0010;
pub const EDGE_BOTTOM: u8 = 0b0100;
pub const EDGE_LEFT: u8 = 0b1000;
pub const CELL_BLOCKED: u8 = 0b10000;

pub enum Side {
    Top,
    Right,
    Bottom,
    Left,
}

impl Side {
    pub fn to_mask(&self) -> u8 {
        match self {
            Side::Top => EDGE_TOP,
            Side::Right => EDGE_RIGHT,
            Side::Bottom => EDGE_BOTTOM,
            Side::Left => EDGE_LEFT,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Obstacle {
    width: usize,
    height: usize,
    masks: Vec<u8>,
}

impl Obstacle {
    pub fn new(width: usize, height: usize) -> Self {
        let masks = vec![0; width * height];
        Self {
            width,
            height,
            masks,
        }
    }

    pub fn set_edge(&mut self, x: usize, y: usize, side: Side) {
        let index = y * self.width + x;
        let edge = side.to_mask();
        self.masks[index] |= edge;
        match side {
            Side::Top if y > 0 => {
                self.masks[(y - 1) * self.width + x] |= EDGE_BOTTOM;
            }
            Side::Right if x + 1 < self.width => {
                self.masks[y * self.width + (x + 1)] |= EDGE_LEFT;
            }
            Side::Bottom if y + 1 < self.height => {
                self.masks[(y + 1) * self.width + x] |= EDGE_TOP;
            }
            Side::Left if x > 0 => {
                self.masks[y * self.width + (x - 1)] |= EDGE_RIGHT;
            }
            _ => {}
        }
    }

    pub fn set_cell_blocked(&mut self, x: usize, y: usize) {
        let index = y * self.width + x;
        self.masks[index] |= CELL_BLOCKED;
    }
}

#[derive(Debug)]
/// wall grid, walkable block 분할, block adjacency를 보관하는 map이다.
pub struct Map {
    pub width: usize,
    pub height: usize,
    block_map: Vec<Cell>,
    obstacle_masks: Vec<u8>,
    pub blocks: Vec<Block>,
}

impl Map {
    /// obstacle mask를 받아 walkable 영역을 block으로 분할한 map을 만든다.
    pub fn new(obstacle: &Obstacle) -> Self {
        let width = obstacle.width;
        let height = obstacle.height;
        let masks = obstacle.masks.clone();
        let mut block_map = vec![Cell::Empty; width * height];
        for (cell, &mask) in block_map.iter_mut().zip(&masks) {
            if mask & CELL_BLOCKED != 0 {
                *cell = Cell::Wall;
            }
        }
        let blocks = vec![];
        let mut map = Map {
            width,
            height,
            obstacle_masks: masks,
            block_map,
            blocks,
        };
        map.random_expand();
        map
    }

    /// 좌표에 해당하는 cell 상태를 반환한다.
    pub fn get_cell(&self, x: usize, y: usize) -> Cell {
        self.block_map[y * self.width + x]
    }

    /// 주어진 block과 변을 공유하는 block과 그 접촉 구간(portal)을 반환한다.
    pub fn get_neighbors(&self, block_id: usize) -> HashSet<(usize, OrthogonalLine)> {
        let mut neighbors = HashSet::new();
        let block = &self.blocks[block_id];
        if block.y > 0 {
            let mut x = block.x;
            while x < block.x + block.width {
                if !(self.can_pass(x, block.y, EDGE_TOP)) {
                    x += 1;
                    continue;
                }
                let Cell::Block(neighbor_id) = self.get_cell(x, block.y - 1) else {
                    x += 1;
                    continue;
                };
                let start_x = x;
                while x < block.x + block.width
                    && self.get_cell(x, block.y - 1) == Cell::Block(neighbor_id)
                    && self.can_pass(x, block.y, EDGE_TOP)
                {
                    x += 1;
                }
                if start_x < x {
                    let overlap = OrthogonalLine {
                        is_vertical: false,
                        fixed: block.y,
                        from: start_x,
                        to: x,
                    };
                    neighbors.insert((neighbor_id, overlap));
                }
            }
        }
        if block.y + block.height < self.height {
            let mut x = block.x;
            while x < block.x + block.width {
                if !(self.can_pass(x, block.y + block.height - 1, EDGE_BOTTOM)) {
                    x += 1;
                    continue;
                }
                let Cell::Block(neighbor_id) = self.get_cell(x, block.y + block.height) else {
                    x += 1;
                    continue;
                };
                let start_x = x;
                while x < block.x + block.width
                    && self.get_cell(x, block.y + block.height) == Cell::Block(neighbor_id)
                    && self.can_pass(x, block.y + block.height - 1, EDGE_BOTTOM)
                {
                    x += 1;
                }
                if start_x < x {
                    let overlap = OrthogonalLine {
                        is_vertical: false,
                        fixed: block.y + block.height - 1,
                        from: start_x,
                        to: x,
                    };
                    neighbors.insert((neighbor_id, overlap));
                }
            }
        }
        if block.x > 0 {
            let mut y = block.y;
            while y < block.y + block.height {
                if !(self.can_pass(block.x, y, EDGE_LEFT)) {
                    y += 1;
                    continue;
                }
                let Cell::Block(neighbor_id) = self.get_cell(block.x - 1, y) else {
                    y += 1;
                    continue;
                };
                let start_y = y;
                while y < block.y + block.height
                    && self.get_cell(block.x - 1, y) == Cell::Block(neighbor_id)
                    && self.can_pass(block.x, y, EDGE_LEFT)
                {
                    y += 1;
                }
                if start_y < y {
                    let overlap = OrthogonalLine {
                        is_vertical: true,
                        fixed: block.x,
                        from: start_y,
                        to: y,
                    };
                    neighbors.insert((neighbor_id, overlap));
                }
            }
        }
        if block.x + block.width < self.width {
            let mut y = block.y;
            while y < block.y + block.height {
                if !(self.can_pass(block.x + block.width - 1, y, EDGE_RIGHT)) {
                    y += 1;
                    continue;
                }
                let Cell::Block(neighbor_id) = self.get_cell(block.x + block.width, y) else {
                    y += 1;
                    continue;
                };
                let start_y = y;
                while y < block.y + block.height
                    && self.get_cell(block.x + block.width, y) == Cell::Block(neighbor_id)
                    && self.can_pass(block.x + block.width - 1, y, EDGE_RIGHT)
                {
                    y += 1;
                }
                if start_y < y {
                    let overlap = OrthogonalLine {
                        is_vertical: true,
                        fixed: block.x + block.width - 1,
                        from: start_y,
                        to: y,
                    };
                    neighbors.insert((neighbor_id, overlap));
                }
            }
        }
        neighbors
    }

    /// 빈 cell을 seed로 삼아 확장 가능한 직사각형 block으로 map을 채운다.
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

    /// goal block에서 역방향으로 탐색해 block별 추정 cost, exit portal, 다음 block을 구한다.
    fn get_block_route(&self, goal: Pos) -> anyhow::Result<Vec<Option<BlockRoute>>> {
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
            visited[node.block_id] = Some(BlockRoute {
                cost: node.cost,
                exit_portal: node.entry,
                next_block: node.from,
            });

            for (neighbor_id, line) in self.get_neighbors(node.block_id) {
                if visited[neighbor_id].is_some() {
                    continue;
                }
                let boundary_mid = line.mid_point();
                let entry_mid = node.entry.mid_point();
                let diff = (
                    boundary_mid.x.abs_diff(entry_mid.x),
                    boundary_mid.y.abs_diff(entry_mid.y),
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

    /// block route를 기준으로 각 walkable cell의 goal 방향 단위 vector를 생성한다.
    pub fn build_flow_field(&self, goal: Pos) -> anyhow::Result<FlowField> {
        let block_routes = self.get_block_route(goal)?;
        let mut flow = vec![None; self.width * self.height];
        for x in 0..self.width {
            for y in 0..self.height {
                let cell = self.get_cell(x, y);
                let cell_pos = Pos { x, y };
                if let Cell::Block(block_id) = cell {
                    if let Some(BlockRoute {
                        cost: _,
                        exit_portal,
                        next_block,
                    }) = block_routes[block_id]
                    {
                        let nearest = exit_portal.nearest_point(cell_pos); // nearest point on the entry line to the cell
                        if nearest == cell_pos {
                            // cell is on the entry line
                            if let Some(from_block_id) = next_block {
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

    /// =from=에서 =to=로 향하는 2D 단위 vector를 반환한다.
    fn get_norm(&self, from: Pos, to: Pos) -> (f32, f32) {
        if from == to {
            return (0.0, 0.0);
        }
        let dx = to.x.abs_diff(from.x);
        let dy = to.y.abs_diff(from.y);
        let dist = ((dx * dx + dy * dy) as f32).sqrt();
        (
            (to.x as f32 - from.x as f32) / dist,
            (to.y as f32 - from.y as f32) / dist,
        )
    }

    pub fn can_pass(&self, x: usize, y: usize, edge: u8) -> bool {
        let index = y * self.width + x;
        (self.obstacle_masks[index] & edge) == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_edge_blocks_both_sides() {
        let mut obstacle = Obstacle::new(2, 1);
        obstacle.set_edge(0, 0, Side::Right);
        let map = Map::new(&obstacle);

        assert!(!map.can_pass(0, 0, EDGE_RIGHT));
        assert!(!map.can_pass(1, 0, EDGE_LEFT));
    }

    #[test]
    fn blocked_cell_is_not_assigned_to_a_block() {
        let mut obstacle = Obstacle::new(2, 1);
        obstacle.set_cell_blocked(1, 0);
        let map = Map::new(&obstacle);

        assert_eq!(map.get_cell(1, 0), Cell::Wall);
    }

    #[test]
    fn generated_blocks_do_not_contain_blocked_edges() {
        let mut obstacle = Obstacle::new(4, 4);
        obstacle.set_edge(0, 0, Side::Right);
        obstacle.set_edge(2, 1, Side::Bottom);
        obstacle.set_edge(1, 2, Side::Right);
        let map = Map::new(&obstacle);

        for block in &map.blocks {
            for y in block.y..block.y + block.height {
                for x in block.x..block.x + block.width.saturating_sub(1) {
                    assert!(map.can_pass(x, y, EDGE_RIGHT));
                }
            }
            for y in block.y..block.y + block.height.saturating_sub(1) {
                for x in block.x..block.x + block.width {
                    assert!(map.can_pass(x, y, EDGE_BOTTOM));
                }
            }
        }
    }

    #[test]
    fn blocked_edge_splits_neighbor_portals() {
        let mut obstacle = Obstacle::new(5, 2);
        obstacle.set_edge(2, 0, Side::Bottom);
        let map = map_with_two_row_blocks(obstacle);
        let neighbors = map.get_neighbors(0);

        assert!(neighbors.contains(&(
            1,
            OrthogonalLine {
                is_vertical: false,
                fixed: 0,
                from: 0,
                to: 2,
            }
        )));
        assert!(neighbors.contains(&(
            1,
            OrthogonalLine {
                is_vertical: false,
                fixed: 0,
                from: 3,
                to: 5,
            }
        )));
        assert_eq!(neighbors.len(), 2);
    }

    #[test]
    fn fully_blocked_boundary_has_no_neighbor() {
        let mut obstacle = Obstacle::new(3, 2);
        for x in 0..3 {
            obstacle.set_edge(x, 0, Side::Bottom);
        }
        let map = map_with_two_row_blocks(obstacle);

        assert!(map.get_neighbors(0).is_empty());
    }

    fn map_with_two_row_blocks(obstacle: Obstacle) -> Map {
        let Obstacle {
            width,
            height,
            masks,
        } = obstacle;
        let mut block_map = vec![Cell::Block(0); width * height];
        for cell in &mut block_map[width..] {
            *cell = Cell::Block(1);
        }

        Map {
            width,
            height,
            block_map,
            obstacle_masks: masks,
            blocks: vec![
                Block {
                    x: 0,
                    y: 0,
                    width,
                    height: 1,
                },
                Block {
                    x: 0,
                    y: 1,
                    width,
                    height: 1,
                },
            ],
        }
    }
}
