use std::collections::{BinaryHeap, BTreeSet};

use crate::{
    convex_hull::{convex_hull, tiles_inside_hull},
    block::{Block, Portal}, types::{ChunkRoute, Cell, FlowField, FlowFieldOptions, OrthogonalLine, Pos, VisitNode},
};

pub const EDGE_TOP: u8 = 0b0001;
pub const EDGE_RIGHT: u8 = 0b0010;
pub const EDGE_BOTTOM: u8 = 0b0100;
pub const EDGE_LEFT: u8 = 0b1000;
pub const CELL_BLOCKED: u8 = 0b10000;

// 타일 면적의 50% 이상이 hull 안에 있으면 병합 검사 대상으로 포함한다.
const HULL_TILE_MIN_COVERAGE: f64 = 0.5;

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
        if self.masks[index] & CELL_BLOCKED != 0 {
            return; // cell is blocked, do not set edge
        }
        match side {
            Side::Top if y > 0 => {
                if self.masks[(y - 1) * self.width + x] & CELL_BLOCKED == 0 {
                    self.masks[(y - 1) * self.width + x] |= EDGE_BOTTOM;
                } else { return; }
            }
            Side::Right if x + 1 < self.width => {
                if self.masks[y * self.width + (x + 1)] & CELL_BLOCKED == 0 {
                    self.masks[y * self.width + (x + 1)] |= EDGE_LEFT;
                } else { return; }
            }
            Side::Bottom if y + 1 < self.height => {
                if self.masks[(y + 1) * self.width + x] & CELL_BLOCKED == 0 {
                    self.masks[(y + 1) * self.width + x] |= EDGE_TOP;
                } else { return; }
            }
            Side::Left if x > 0 => {
                if self.masks[y * self.width + (x - 1)] & CELL_BLOCKED == 0 {
                    self.masks[y * self.width + (x - 1)] |= EDGE_RIGHT;
                } else { return; }
            }
            _ => {}
        }
        self.masks[index] |= edge;
    }

    pub fn set_cell_blocked(&mut self, x: usize, y: usize) {
        let index = y * self.width + x;
        self.masks[index] = CELL_BLOCKED;
        if x > 0 {
            self.masks[y * self.width + (x - 1)] &= !EDGE_RIGHT;
        }
        if x + 1 < self.width {
            self.masks[y * self.width + (x + 1)] &= !EDGE_LEFT;
        }
        if y > 0 {
            self.masks[(y - 1) * self.width + x] &= !EDGE_BOTTOM;
        }
        if y + 1 < self.height {
            self.masks[(y + 1) * self.width + x] &= !EDGE_TOP;
        }
    }
}

#[derive(Debug)]
pub struct Chunk {
    pub block_ids: Vec<usize>,
    pub portals: Vec<Portal>,
}

#[derive(Debug)]
/// wall grid, walkable block 분할, block adjacency를 보관하는 map이다.
pub struct Map {
    pub width: usize,
    pub height: usize,
    block_map: Vec<Cell>,
    obstacle_masks: Vec<u8>,
    pub blocks: Vec<Block>,
    pub chunks: Vec<Chunk>,
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
        let mut map = Map {
            width,
            height,
            obstacle_masks: masks,
            block_map,
            blocks: vec![],
            chunks: vec![],
        };
        map.bulid_map();
        map
    }

    /// 좌표에 해당하는 cell 상태를 반환한다.
    pub fn get_cell(&self, x: usize, y: usize) -> Cell {
        if x >= self.width || y >= self.height {
            return Cell::Wall;
        }
        self.block_map[y * self.width + x]
    }

    /// 주어진 block과 변을 공유하는 block과 그 접촉 구간(portal)을 반환한다.
    pub fn get_neighbors(&self, block_id: usize) -> Vec<Portal> {
        let mut neighbors = Vec::new();
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
                    neighbors.push( Portal {
                        neighbor: neighbor_id,
                        line: overlap
                    });
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
                    neighbors.push( Portal {
                        neighbor: neighbor_id,
                        line: overlap
                    });
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
                    neighbors.push( Portal {
                        neighbor: neighbor_id,
                        line: overlap
                    });
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
                    neighbors.push( Portal {
                        neighbor: neighbor_id,
                        line: overlap
                    });
                }
            }
        }
        neighbors
    }

    fn has_edge_between(&self, block1: usize, block2: usize) -> bool {
        let b1 = &self.blocks[block1];
        let b2 = &self.blocks[block2];
        if b1.y + b1.height == b2.y || b2.y + b2.height == b1.y {
            let (top, bottom) = if b1.y < b2.y { (b1, b2) } else { (b2, b1) };
            let start_x = b1.x.max(b2.x);
            let end_x = (b1.x + b1.width).min(b2.x + b2.width);
            for x in start_x..end_x {
                if !self.can_pass(x, top.y + top.height - 1, EDGE_BOTTOM) || !self.can_pass(x, bottom.y, EDGE_TOP) {
                    return true;
                }
            }
        } else if b1.x + b1.width == b2.x || b2.x + b2.width == b1.x {
            let (left, right) = if b1.x < b2.x { (b1, b2) } else { (b2, b1) };
            let start_y = b1.y.max(b2.y);
            let end_y = (b1.y + b1.height).min(b2.y + b2.height);
            for y in start_y..end_y {
                if !self.can_pass(left.x + left.width - 1, y, EDGE_RIGHT) || !self.can_pass(right.x, y, EDGE_LEFT) {
                    return true;
                }
            }
        }
        false
    }

    /// hull을 채우는 데 필요한 미할당 Block들을 고정점까지 모은다.
    /// 검증이 끝나기 전에는 Chunk/Block의 할당 상태를 변경하지 않는다.
    fn collect_merge_blocks(
        &self,
        chunk: &Chunk,
        candidate_id: usize,
        unchunked: &BTreeSet<usize>,
    ) -> Option<Vec<usize>> {
        let mut block_ids = chunk.block_ids.iter().copied().collect::<BTreeSet<_>>();
        if !unchunked.contains(&candidate_id) {
            return None;
        }
        block_ids.insert(candidate_id);
        loop {
            let ids = block_ids.iter().copied().collect::<Vec<_>>();
            // 자동으로 추가된 Block 사이의 접촉면도 모두 검사한다.
            for (i, &a) in ids.iter().enumerate() {
                for &b in &ids[i + 1..] {
                    if self.has_edge_between(a, b) {
                        return None;
                    }
                }
            }
            let points = ids.iter()
                .flat_map(|&id| self.blocks[id].get_vertices())
                .collect::<Vec<_>>();
            let hull = convex_hull(&points);
            let mut additions = BTreeSet::new();
            for pos in tiles_inside_hull(&hull, HULL_TILE_MIN_COVERAGE) {
                match self.get_cell(pos.x, pos.y) {
                    Cell::Block(id) if block_ids.contains(&id) => {}
                    Cell::Block(id) if unchunked.contains(&id) => {
                        additions.insert(id);
                    }
                    _ => return None,
                }
            }
            if additions.is_empty() {
                return Some(ids.into_iter()
                    .filter(|id| !chunk.block_ids.contains(id))
                    .collect());
            }
            block_ids.extend(additions);
        }
    }

    /// 빈 cell을 seed로 삼아 확장 가능한 직사각형 block으로 map을 채운다.
    fn bulid_map(&mut self) {
        for y in 0..self.height {
            for x in 0..self.width {
                if self.block_map[y * self.width + x] != Cell::Empty {
                    continue;
                }
                let new_block_id = self.blocks.len();
                let mut new_block = Block {
                    x,
                    y,
                    width: 1,
                    height: 1,
                    id: new_block_id,
                    chunk_id: None,
                    portals: vec![],
                };
                while new_block.expand(self).is_some() {}
                for yy in new_block.y..new_block.y + new_block.height {
                    for xx in new_block.x..new_block.x + new_block.width {
                        self.block_map[yy * self.width + xx] = Cell::Block(new_block_id);
                    }
                }
                self.blocks.push(new_block);
            }
        }
        for i in 0..self.blocks.len() {
            self.blocks[i].portals = self.get_neighbors(i);
        }
        let mut unchunked_blocks = self.blocks.iter().map(|b| b.id).collect::<BTreeSet<_>>();
        while !unchunked_blocks.is_empty() {
            let mut chunk = Chunk {
                block_ids: vec![],
                portals: vec![],
            };
            let mut stack = vec![*unchunked_blocks.first().unwrap()];
            while let Some(block_id) = stack.pop() {
                let Some(additions) = self.collect_merge_blocks(&chunk, block_id, &unchunked_blocks) else {
                    continue;
                };
                for id in additions {
                    unchunked_blocks.remove(&id);
                    self.blocks[id].chunk_id = Some(self.chunks.len());
                    chunk.block_ids.push(id);
                    chunk.portals.extend(self.blocks[id].portals.iter().cloned());
                }
                // 묶음이 커지면 앞서 실패한 경계 후보도 다시 검토한다.
                let mut frontier = BTreeSet::new();
                for portal in &chunk.portals {
                    if unchunked_blocks.contains(&portal.neighbor) {
                        frontier.insert(portal.neighbor);
                    }
                }
                stack = frontier.into_iter().rev().collect();
            }
            chunk.portals.retain(|portal| !chunk.block_ids.contains(&portal.neighbor));
            self.chunks.push(chunk);
        }
    }

    /// Chunk 외부 portal만 사용하여 목표에서 역방향 탐색한다.
    fn get_chunk_route(&self, goal: Pos) -> anyhow::Result<Vec<Option<ChunkRoute>>> {
        let goal_chunk = match self.get_cell(goal.x, goal.y) {
            Cell::Block(id) => self.blocks[id].chunk_id
                .ok_or_else(|| anyhow::anyhow!("Goal block has no chunk"))?,
            _ => anyhow::bail!("Goal is not on a block"),
        };
        let mut pq = BinaryHeap::new();
        let mut visited = vec![None; self.chunks.len()];
        pq.push(VisitNode {
            chunk_id: goal_chunk,
            entry: OrthogonalLine {
                is_vertical: false, fixed: goal.y, from: goal.x, to: goal.x + 1,
            },
            exit_step: (0, 0),
            cost: 0.0,
            from: None,
        });
        while let Some(node) = pq.pop() {
            if visited[node.chunk_id].is_some() { continue; }
            visited[node.chunk_id] = Some(ChunkRoute {
                cost: node.cost,
                exit_portal: node.entry,
                next_chunk: node.from,
                exit_step: node.exit_step,
            });
            for portal in &self.chunks[node.chunk_id].portals {
                let neighbor = &self.blocks[portal.neighbor];
                let neighbor_chunk = neighbor.chunk_id
                    .ok_or_else(|| anyhow::anyhow!("Neighbor block has no chunk"))?;
                if neighbor_chunk == node.chunk_id || visited[neighbor_chunk].is_some() {
                    continue;
                }
                let boundary = portal.line.mid_point();
                let entry = node.entry.mid_point();
                let distance = (boundary.x.abs_diff(entry.x) as f32)
                    .hypot(boundary.y.abs_diff(entry.y) as f32);

                // 저장된 portal은 현재 쪽 셀 좌표이므로 이웃 Chunk 쪽으로 한 셀 이동한다.
                let mut line = portal.line;
                let exit_step = if line.is_vertical {
                    if line.fixed < neighbor.x {
                        line.fixed += 1;
                        (-1, 0)
                    } else {
                        line.fixed -= 1;
                        (1, 0)
                    }
                } else if line.fixed < neighbor.y {
                    line.fixed += 1;
                    (0, -1)
                } else {
                    line.fixed -= 1;
                    (0, 1)
                };
                pq.push(VisitNode {
                    chunk_id: neighbor_chunk,
                    entry: line,
                    exit_step,
                    cost: node.cost + distance + 1.0,
                    from: Some(node.chunk_id),
                });
            }
        }
        Ok(visited)
    }

    /// Chunk별 출구를 공유하고 내부 Block 사이의 portal은 무시한다.
    pub fn build_flow_field(&self, goal: Pos, options: &FlowFieldOptions) -> anyhow::Result<FlowField> {
        if options.wall_margin < 0.0 {
            anyhow::bail!("wall_margin must be non-negative");
        }
        if options.wall_repulsion_strength < 0.0 {
            anyhow::bail!("wall_repulsion_strength must be non-negative");
        }
        if options.max_repulsion_ratio < 0.0 {
            anyhow::bail!("max_repulsion_ratio must be non-negative");
        }
        let routes = self.get_chunk_route(goal)?;
        let mut flow = vec![None; self.width * self.height];
        for y in 0..self.height {
            for x in 0..self.width {
                let Cell::Block(block_id) = self.get_cell(x, y) else { continue; };
                let chunk_id = self.blocks[block_id].chunk_id
                    .ok_or_else(|| anyhow::anyhow!("Block has no chunk"))?;
                let Some(route) = routes[chunk_id] else { continue; };
                let pos = Pos { x, y };
                let target = if route.next_chunk.is_none() {
                    goal
                } else {
                    let nearest = route.exit_portal.nearest_point_with_margin(pos);
                    if nearest == pos {
                        // 다음 Block 중심 대신 선택한 portal을 수직으로 건넌다.
                        Pos {
                            x: pos.x.checked_add_signed(route.exit_step.0)
                                .ok_or_else(|| anyhow::anyhow!("Invalid portal x"))?,
                            y: pos.y.checked_add_signed(route.exit_step.1)
                                .ok_or_else(|| anyhow::anyhow!("Invalid portal y"))?,
                        }
                    } else {
                        nearest
                    }
                };
                flow[y * self.width + x] = Some(self.get_norm_with_repulsion(pos, target, options));
            }
        }
        Ok(FlowField { goal, flow, options: *options })
    }

    /// =from=에서 =to=로 향하는 2D 단위 vector를 반환한다.
    fn get_norm(&self, from: Pos, to: Pos) -> (f32, f32) {
        if from == to {
            return (0.0, 0.0);
        }
        let dx = to.x.abs_diff(from.x);
        let dy = to.y.abs_diff(from.y);
        let dist = (dx as f32).hypot(dy as f32);
        (
            (to.x as f32 - from.x as f32) / dist,
            (to.y as f32 - from.y as f32) / dist,
        )
    }

    fn get_repulsion(&self, pos: Pos, options: &FlowFieldOptions) -> (f32, f32) {
        if options.wall_margin <= 0.0 || options.wall_repulsion_strength <= 0.0 {
            return (0.0, 0.0);
        }
        let mut repulsion = (0.0, 0.0);
        let px = pos.x as f32 + 0.5;
        let py = pos.y as f32 + 0.5;

        let radius = options.wall_margin.ceil() as isize + 1;
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                let nx = pos.x as isize + dx;
                let ny = pos.y as isize + dy;

                if nx < 0 || ny < 0 || nx >= self.width as isize || ny >= self.height as isize {
                    continue;
                }

                let nx = nx as usize;
                let ny = ny as usize;

                match self.get_cell(nx, ny) {
                    Cell::Wall => {
                        let qx = px.clamp(nx as f32, nx as f32 + 1.0);
                        let qy = py.clamp(ny as f32, ny as f32 + 1.0);
                        let diff_x = px - qx;
                        let diff_y = py - qy;

                        let distance = diff_x.hypot(diff_y);
                        let weight =  (1.0 - distance / options.wall_margin).clamp(0.0, 1.0) * options.wall_repulsion_strength;
                        repulsion.0 += (diff_x / distance) * weight;
                        repulsion.1 += (diff_y / distance) * weight;
                    },
                    _ => {
                        let edge_mask = self.obstacle_masks[ny * self.width + nx];
                        if edge_mask & EDGE_TOP != 0 {
                            let qx = px.clamp(nx as f32, nx as f32 + 1.0);
                            let qy = ny as f32;
                            let diff_x = px - qx;
                            let diff_y = py - qy;
                            let distance = diff_x.hypot(diff_y);
                            let weight = (1.0 - distance / options.wall_margin).clamp(0.0, 1.0) * options.wall_repulsion_strength;
                            repulsion.0 += (diff_x / distance) * weight;
                            repulsion.1 += (diff_y / distance) * weight;
                        }
                        if edge_mask & EDGE_LEFT != 0 {
                            let qx = nx as f32;
                            let qy = py.clamp(ny as f32, ny as f32 + 1.0);
                            let diff_x = px - qx;
                            let diff_y = py - qy;
                            let distance = diff_x.hypot(diff_y);
                            let weight = (1.0 - distance / options.wall_margin).clamp(0.0, 1.0) * options.wall_repulsion_strength;
                            repulsion.0 += (diff_x / distance) * weight;
                            repulsion.1 += (diff_y / distance) * weight;
                        }
                    }
                }
            }
        }
        repulsion
    }

    pub fn get_norm_with_repulsion(&self, from: Pos, to: Pos, options: &FlowFieldOptions) -> (f32, f32) {
        if from == to {
            return (0.0, 0.0);
        }
        let (flow_x, flow_y) = self.get_norm(from, to);
        let (mut repulsion_x, mut repulsion_y) = self.get_repulsion(from, options);
        let repulsion_magnitude = repulsion_x.hypot(repulsion_y);
        if repulsion_magnitude > options.max_repulsion_ratio {
            let scale = options.max_repulsion_ratio / repulsion_magnitude;
            repulsion_x = repulsion_x * scale;
            repulsion_y = repulsion_y * scale;
        }
        let combined_x = flow_x + repulsion_x;
        let combined_y = flow_y + repulsion_y;
        let length = combined_x.hypot(combined_y);
        if length < 1e-6 {
            (flow_x, flow_y)
        } else {
            (combined_x / length, combined_y / length)
        }
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
    fn goal_chunk_ignores_all_internal_block_portals() {
        let mut obstacle = Obstacle::new(5, 5);
        for y in 0..5 {
            for x in 0..5 {
                if x + y > 4 { obstacle.set_cell_blocked(x, y); }
            }
        }
        let map = Map::new(&obstacle);
        assert!(map.blocks.len() > 1);
        assert_eq!(map.chunks.len(), 1);
        let goal = Pos { x: 0, y: 0 };
        let options = FlowFieldOptions { wall_repulsion_strength: 0.0, ..Default::default() };
        let field = map.build_flow_field(goal, &options).unwrap();
        for y in 0..5 {
            for x in 0..5 {
                if matches!(map.get_cell(x, y), Cell::Block(_)) {
                    let actual = field.flow[y * 5 + x].unwrap();
                    let expected = map.get_norm(Pos { x, y }, goal);
                    assert!((actual.0 - expected.0).abs() < 1e-6);
                    assert!((actual.1 - expected.1).abs() < 1e-6);
                }
            }
        }
    }

    #[test]
    fn chunk_portals_cross_in_all_four_directions() {
        for vertical in [false, true] {
            let mut map = Map::new(&Obstacle::new(6, 4));
            map.blocks = (0..2).map(|id| Block {
                x: if vertical { id * 3 } else { 0 },
                y: if vertical { 0 } else { id * 2 },
                width: if vertical { 3 } else { 6 },
                height: if vertical { 4 } else { 2 },
                id, chunk_id: Some(id), portals: vec![],
            }).collect();
            for y in 0..4 {
                for x in 0..6 {
                    map.block_map[y * 6 + x] = Cell::Block(if vertical { x / 3 } else { y / 2 });
                }
            }
            for id in 0..2 { map.blocks[id].portals = map.get_neighbors(id); }
            map.chunks = (0..2).map(|id| Chunk {
                block_ids: vec![id], portals: map.blocks[id].portals.clone(),
            }).collect();
            for goal_id in 0..2 {
                let goal = map.blocks[goal_id].center();
                let source_id = 1 - goal_id;
                let routes = map.get_chunk_route(goal).unwrap();
                let route = routes[source_id].unwrap();
                assert_eq!(route.next_chunk, Some(goal_id));
                let direction = if goal_id == 1 { 1 } else { -1 };
                let expected = if vertical { (direction, 0) } else { (0, direction) };
                assert_eq!(route.exit_step, expected);
                let options = FlowFieldOptions { wall_repulsion_strength: 0.0, ..Default::default() };
                let field = map.build_flow_field(goal, &options).unwrap();
                let p = route.exit_portal.mid_point();
                assert_eq!(map.get_cell(p.x, p.y), Cell::Block(source_id));
                assert_eq!(field.flow[p.y * 6 + p.x], Some((expected.0 as f32, expected.1 as f32)));
                assert_eq!(field.flow[goal.y * 6 + goal.x], Some((0.0, 0.0)));
            }
        }
    }

    // 왼쪽 A와 오른쪽 B의 hull에 가운데 C 전체가 들어간다.
    fn closure_test_map() -> Map {
        let mut map = Map::new(&Obstacle::new(3, 3));
        map.blocks = vec![
            Block { x: 0, y: 0, width: 1, height: 3, id: 0, chunk_id: Some(0), portals: vec![] },
            Block { x: 2, y: 0, width: 1, height: 3, id: 1, chunk_id: None, portals: vec![] },
            Block { x: 1, y: 0, width: 1, height: 3, id: 2, chunk_id: None, portals: vec![] },
        ];
        map.block_map = vec![
            Cell::Block(0), Cell::Block(2), Cell::Block(1),
            Cell::Block(0), Cell::Block(2), Cell::Block(1),
            Cell::Block(0), Cell::Block(2), Cell::Block(1),
        ];
        map.chunks = vec![Chunk { block_ids: vec![0], portals: vec![] }];
        map
    }

    #[test]
    fn hull_closure_includes_block_with_half_covered_tile() {
        let mut map = closure_test_map();
        map.blocks[1].x = 1;
        map.blocks[1].width = 2;
        map.blocks[1].height = 1;
        map.blocks[2].y = 1;
        map.blocks[2].width = 2;
        map.blocks[2].height = 2;
        map.block_map = vec![
            Cell::Block(0), Cell::Block(1), Cell::Block(1),
            Cell::Block(0), Cell::Block(2), Cell::Block(2),
            Cell::Block(0), Cell::Block(2), Cell::Block(2),
        ];
        // C 타일 하나의 면적 절반이 hull 안에 있으므로 C도 함께 병합한다.
        assert_eq!(map.collect_merge_blocks(&map.chunks[0], 1, &BTreeSet::from([1, 2])), Some(vec![1, 2]));
        assert_eq!(map.blocks[2].chunk_id, None);
    }

    #[test]
    fn hull_closure_collects_required_blocks_without_assigning_them() {
        let map = closure_test_map();
        let pending = BTreeSet::from([1, 2]);
        assert_eq!(map.collect_merge_blocks(&map.chunks[0], 1, &pending), Some(vec![1, 2]));
        assert_eq!(map.blocks[1].chunk_id, None);
        assert_eq!(map.blocks[2].chunk_id, None);
        assert_eq!(map.chunks[0].block_ids, vec![0]);
    }

    #[test]
    fn hull_closure_rejects_required_block_owned_by_another_chunk() {
        let mut map = closure_test_map();
        map.blocks[2].chunk_id = Some(1);
        assert!(map.collect_merge_blocks(&map.chunks[0], 1, &BTreeSet::from([1])).is_none());
    }

    #[test]
    fn hull_closure_checks_edges_between_automatically_added_blocks() {
        let mut map = closure_test_map();
        // B와 자동으로 추가될 C 사이의 edge. A와의 접촉면만 검사하면 놓친다.
        map.obstacle_masks[4] |= EDGE_RIGHT;
        map.obstacle_masks[5] |= EDGE_LEFT;
        assert!(map.collect_merge_blocks(&map.chunks[0], 1, &BTreeSet::from([1, 2])).is_none());
    }

    #[test]
    fn staircase_merges_but_concave_l_shape_does_not() {
        let mut staircase = Obstacle::new(3, 3);
        let mut l_shape = Obstacle::new(5, 5);
        for y in 0..3 {
            for x in 0..3 {
                if x + y > 2 {
                    staircase.set_cell_blocked(x, y);
                }
            }
        }
        // 작은 모서리 침범이 아닌, hull에 타일 전체가 들어가는 큰 빈틈.
        for y in 0..4 {
            for x in 1..5 {
                l_shape.set_cell_blocked(x, y);
            }
        }
        let map = Map::new(&staircase);
        assert!(map.blocks.len() > 1);
        assert_eq!(map.chunks.len(), 1);
        assert!(map.chunks[0].portals.is_empty());
        assert_eq!(map.chunks[0].block_ids.len(), map.blocks.len());
        assert!(map.blocks.iter().all(|block| block.chunk_id == Some(0)));

        let map = Map::new(&l_shape);
        assert!(map.chunks.len() > 1);
        for (id, chunk) in map.chunks.iter().enumerate() {
            for portal in &chunk.portals {
                assert_ne!(map.blocks[portal.neighbor].chunk_id, Some(id));
            }
        }
    }

    #[test]
    fn contact_edge_check_is_symmetric_and_prevents_merge() {
        let mut obstacle = Obstacle::new(5, 2);
        obstacle.set_edge(2, 0, Side::Bottom);
        let map = map_with_two_row_blocks(obstacle);
        assert!(map.has_edge_between(0, 1));
        assert!(map.has_edge_between(1, 0));
        let chunk = Chunk { block_ids: vec![1], portals: vec![] };
        assert!(map.collect_merge_blocks(&chunk, 0, &BTreeSet::from([0])).is_none());
    }

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
        let neighbors = &map.blocks[0].portals;

        assert!(neighbors.contains(&Portal {
            neighbor: 1,
            line: OrthogonalLine {
                is_vertical: false,
                fixed: 0,
                from: 0,
                to: 2,
            },
        }));
        assert!(neighbors.contains(&Portal {
            neighbor: 1,
            line: OrthogonalLine {
                is_vertical: false,
                fixed: 0,
                from: 3,
                to: 5,
            },
        }));
        assert_eq!(neighbors.len(), 2);
    }

    #[test]
    fn fully_blocked_boundary_has_no_neighbor() {
        let mut obstacle = Obstacle::new(3, 2);
        for x in 0..3 {
            obstacle.set_edge(x, 0, Side::Bottom);
        }
        let map = map_with_two_row_blocks(obstacle);

        assert!(map.blocks[0].portals.is_empty());
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

        let mut map = Map {
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
                    id: 0,
                    chunk_id: None,
                    portals: vec![],
                },
                Block {
                    x: 0,
                    y: 1,
                    width,
                    height: 1,
                    id: 1,
                    chunk_id: None,
                    portals: vec![],
                },
            ],
            chunks: vec![],
        };
        for block_id in 0..map.blocks.len() {
            map.blocks[block_id].portals = map.get_neighbors(block_id);
        }
        map
    }
}
