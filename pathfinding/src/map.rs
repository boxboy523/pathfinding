use std::collections::{BTreeMap, BTreeSet};
use slotmap::SlotMap;

use crate::{
    block::Block,
    keys::BlockId,
    portal::{Portal, PortalId},
    portal_route::PortalRoutes,
    types::{Cell, FlowFieldOptions, OrthogonalLine, Pos},
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

    pub fn merge(&mut self, other: &Obstacle) -> anyhow::Result<()> {
        if self.width != other.width || self.height != other.height {
            anyhow::bail!("Obstacle sizes do not match for merging");
        }
        for (i, &mask) in other.masks.iter().enumerate() {
            let x = i % self.width;
            let y = i / self.width;
            if mask & CELL_BLOCKED != 0 {
                self.set_cell_blocked(x, y);
            } else {
                if mask & EDGE_TOP != 0 {
                    self.set_edge(x, y, Side::Top);
                }
                if mask & EDGE_RIGHT != 0 {
                    self.set_edge(x, y, Side::Right);
                }
                if mask & EDGE_BOTTOM != 0 {
                    self.set_edge(x, y, Side::Bottom);
                }
                if mask & EDGE_LEFT != 0 {
                    self.set_edge(x, y, Side::Left);
                }
            }
        }
        Ok(())
    }
}


/// Map 내에서 생성된 건물의 식별자. 삭제 후에도 재사용하지 않는다.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BuildingId(usize);

#[derive(Debug)]
/// wall grid, walkable block 분할, block adjacency를 보관하는 map이다.
pub struct Map {
    pub width: usize,
    pub height: usize,
    block_map: Vec<Cell>,
    obstacle: Obstacle,
    base_obstacle: Obstacle,
    buildings: BTreeMap<BuildingId, Obstacle>,
    next_building_id: usize,
    pub blocks: SlotMap<BlockId, Block>,
    /// Shared openings, each stored exactly once.
    pub portals: SlotMap<PortalId, Portal>,
    generation: usize,
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
            obstacle: Obstacle {
                width,
                height,
                masks,
            },
            base_obstacle: obstacle.clone(),
            buildings: BTreeMap::new(),
            next_building_id: 0,
            block_map,
            blocks: SlotMap::with_key(),
            portals: SlotMap::with_key(),
            generation: 0,
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

    pub fn get_generation(&self) -> usize {
        self.generation
    }

    /// IDs of shared openings incident to a Block; Portal data is owned by Map.
    pub fn get_neighbors(&self, block_id: BlockId) -> &[PortalId] {
        &self.blocks[block_id].portals
    }

    fn boundary_blocks(&self, vertical: bool, fixed: usize, offset: usize) -> Option<[BlockId; 2]> {
        let (x, y, nx, ny, edge, opposite) = if vertical {
            (fixed, offset, fixed + 1, offset, EDGE_RIGHT, EDGE_LEFT)
        } else {
            (offset, fixed, offset, fixed + 1, EDGE_BOTTOM, EDGE_TOP)
        };
        let Cell::Block(a) = self.get_cell(x, y) else { return None; };
        let Cell::Block(b) = self.get_cell(nx, ny) else { return None; };
        (a != b && self.can_pass(x, y, edge) && self.can_pass(nx, ny, opposite)).then_some([a, b])
    }

    /// Discover openings from cells, independently of existing Portal objects.
    fn block_openings(&self, id: BlockId) -> Vec<Portal> {
        let block = &self.blocks[id];
        let mut sides = vec![
            (true, block.x + block.width - 1, block.y, block.y + block.height),
            (false, block.y + block.height - 1, block.x, block.x + block.width),
        ];
        if let Some(x) = block.x.checked_sub(1) {
            sides.push((true, x, block.y, block.y + block.height));
        }
        if let Some(y) = block.y.checked_sub(1) {
            sides.push((false, y, block.x, block.x + block.width));
        }
        let mut openings = Vec::new();
        for (vertical, fixed, from, to) in sides {
            let mut offset = from;
            while offset < to {
                let Some(blocks) = self.boundary_blocks(vertical, fixed, offset) else {
                    offset += 1;
                    continue;
                };
                let start = offset;
                while offset < to && self.boundary_blocks(vertical, fixed, offset) == Some(blocks) {
                    offset += 1;
                }
                openings.push(Portal {
                    blocks,
                    line: OrthogonalLine { is_vertical: vertical, fixed, from: start, to: offset },
                });
            }
        }
        openings
    }

    /// Only boundaries incident to new Blocks need Portal objects. Preserve every other key.
    fn rebuild_portals(&mut self, new_blocks: &BTreeSet<BlockId>) {
        for &id in new_blocks {
            for portal in self.block_openings(id) {
                // If both Blocks are new, only the canonical left/top Block creates it.
                if new_blocks.contains(&portal.blocks[0]) && id != portal.blocks[0] { continue; }
                let blocks = portal.blocks;
                let portal_id = self.portals.insert(portal);
                for block in blocks { self.blocks[block].portals.push(portal_id); }
            }
        }
    }

    #[cfg(test)]
    fn build_portals(&mut self) {
        self.portals.clear();
        for block in self.blocks.values_mut() { block.portals.clear(); }
        self.rebuild_portals(&self.blocks.keys().collect());
    }


    fn bulid_map(&mut self) {
        let blocks = self.build_blocks(0..self.block_map.len());
        self.rebuild_portals(&blocks);
    }

    /// Partition only Empty cells reachable from the supplied seeds.
    fn build_blocks(&mut self, seeds: impl IntoIterator<Item = usize>) -> BTreeSet<BlockId> {
        let mut blocks = BTreeSet::new();
        for index in seeds {
            if self.block_map[index] != Cell::Empty { continue; }
            let mut block = Block {
                x: index % self.width, y: index / self.width, width: 1, height: 1,
                id: BlockId::default(), portals: vec![],
            };
            while block.expand(self).is_some() {}
            let id = self.blocks.insert_with_key(|id| Block { id, ..block });
            let block = &self.blocks[id];
            for y in block.y..block.y + block.height {
                for x in block.x..block.x + block.width {
                    self.block_map[y * self.width + x] = Cell::Block(id);
                }
            }
            blocks.insert(id);
        }
        blocks
    }


    /// 벽이나 경계가 있는 셀이 기본 지형 또는 기존 건물과 겹치면 거절한다.
    /// obstacle은 Map과 같은 크기여야 한다.
    pub fn add_building(&mut self, obstacle: Obstacle) -> anyhow::Result<BuildingId> {
        if self.width != obstacle.width || self.height != obstacle.height {
            anyhow::bail!("Building obstacle size does not match map");
        }
        if obstacle.masks.iter().all(|&mask| mask == 0) {
            anyhow::bail!("Building obstacle must not be empty");
        }
        if self.obstacle.masks.iter().zip(&obstacle.masks)
            .any(|(&existing, &added)| existing != 0 && added != 0)
        {
            anyhow::bail!("Building overlaps existing terrain or building");
        }
        let next_id = self.next_building_id.checked_add(1)
            .ok_or_else(|| anyhow::anyhow!("Building ID exhausted"))?;
        let generation = self.generation.checked_add(1)
            .ok_or_else(|| anyhow::anyhow!("Map generation exhausted"))?;
        let mut combined = self.obstacle.clone();
        combined.merge(&obstacle)?;

        let id = BuildingId(self.next_building_id);
        self.buildings.insert(id, obstacle);
        self.next_building_id = next_id;
        self.rebuild_obstacle(combined);
        self.generation = generation;
        Ok(id)
    }

    /// 건물을 제거하고 기본 지형과 나머지 건물로 맵을 재구성한다.
    /// 없는 ID라면 Map과 generation을 변경하지 않는다.
    pub fn remove_building(&mut self, id: BuildingId) -> anyhow::Result<()> {
        if !self.buildings.contains_key(&id) {
            anyhow::bail!("Unknown building ID");
        }
        let generation = self.generation.checked_add(1)
            .ok_or_else(|| anyhow::anyhow!("Map generation exhausted"))?;
        let mut combined = self.base_obstacle.clone();
        for (&building_id, obstacle) in &self.buildings {
            if building_id != id {
                combined.merge(obstacle)?;
            }
        }
        self.buildings.remove(&id);
        self.rebuild_obstacle(combined);
        self.generation = generation;
        Ok(())
    }

    /// Replace only topology invalidated by changed masks (addition or removal).
    fn rebuild_obstacle(&mut self, obstacle: Obstacle) {
        let mut cells = self.obstacle.masks.iter().zip(&obstacle.masks).enumerate()
            .filter_map(|(i, (old, new))| (old != new).then_some(i)).collect::<Vec<_>>();
        let mut blocks = BTreeSet::new();
        for &index in &cells {
            if let Cell::Block(id) = self.block_map[index] { blocks.insert(id); }
            // A changed edge invalidates both incident Blocks, even for one-sided masks.
            let changed = self.obstacle.masks[index] ^ obstacle.masks[index];
            let (x, y) = (index % self.width, index / self.width);
            for (edge, dx, dy) in [(EDGE_TOP, 0, -1), (EDGE_RIGHT, 1, 0),
                (EDGE_BOTTOM, 0, 1), (EDGE_LEFT, -1, 0)] {
                if changed & edge == 0 { continue; }
                if let (Some(nx), Some(ny)) = (x.checked_add_signed(dx), y.checked_add_signed(dy)) {
                    if let Cell::Block(id) = self.get_cell(nx, ny) { blocks.insert(id); }
                }
            }
        }
        let portals = blocks.iter().flat_map(|&id| self.blocks[id].portals.iter().copied())
            .collect::<BTreeSet<_>>();
        for id in blocks {
            let block = self.blocks.remove(id).expect("affected Block");
            for y in block.y..block.y + block.height {
                cells.extend((block.x..block.x + block.width).map(|x| y * self.width + x));
            }
        }
        for id in portals {
            let portal = self.portals.remove(id).expect("incident Portal");
            for block_id in portal.blocks {
                if let Some(block) = self.blocks.get_mut(block_id) {
                    block.portals.retain(|&p| p != id);
                }
            }
        }
        self.obstacle = obstacle;
        cells.sort_unstable();
        cells.dedup();
        for &index in &cells {
            self.block_map[index] = if self.obstacle.masks[index] & CELL_BLOCKED != 0 {
                Cell::Wall
            } else {
                Cell::Empty
            };
        }
        let new_blocks = self.build_blocks(cells);
        self.rebuild_portals(&new_blocks);
    }

    /// 공유 Portal을 노드로 역방향 Dijkstra를 수행하고 Block마다 출구 하나를 선택한다.
    pub fn build_flow_field(&self, goal: Pos, options: &FlowFieldOptions) -> anyhow::Result<Vec<Option<(f32, f32)>>> {
        if !options.wall_margin.is_finite() || options.wall_margin < 0.0 {
            anyhow::bail!("wall_margin must be non-negative");
        }
        if !options.wall_repulsion_strength.is_finite() || options.wall_repulsion_strength < 0.0 {
            anyhow::bail!("wall_repulsion_strength must be non-negative");
        }
        if !options.max_repulsion_ratio.is_finite() || options.max_repulsion_ratio < 0.0 {
            anyhow::bail!("max_repulsion_ratio must be non-negative");
        }
        let routes = PortalRoutes::build(self, goal)?;
        let mut flow = vec![None; self.width * self.height];
        for y in 0..self.height {
            for x in 0..self.width {
                let Cell::Block(block_id) = self.get_cell(x, y) else { continue; };
                let pos = Pos { x, y };
                let target = if block_id == routes.goal_block {
                    goal
                } else {
                    let Some(route) = routes.exit_for(block_id) else { continue; };
                    // Actual steering retains the existing nearest-point policy.
                    // Its approximation can differ from the midpoint search model.
                    let (line, exit_step) = route.side_from(block_id)
                        .ok_or_else(|| anyhow::anyhow!("Portal is not incident to Block"))?;
                    let nearest = line.nearest_point_with_margin(pos);
                    if nearest == pos {
                        Pos {
                            x: pos.x.checked_add_signed(exit_step.0)
                                .ok_or_else(|| anyhow::anyhow!("Invalid portal x"))?,
                            y: pos.y.checked_add_signed(exit_step.1)
                                .ok_or_else(|| anyhow::anyhow!("Invalid portal y"))?,
                        }
                    } else {
                        nearest
                    }
                };
                flow[y * self.width + x] = Some(self.get_norm_with_repulsion(pos, target, options));
            }
        }
        Ok(flow)
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
                        let edge_mask = self.obstacle.masks[ny * self.width + nx];
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
        (self.obstacle.masks[index] & edge) == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::MAX_ASPECT_RATIO;

    fn clear_topology(map: &mut Map) {
        map.blocks.clear();
        map.portals.clear();
        map.block_map.fill(Cell::Empty);
    }

    fn block(map: &mut Map, x: usize, y: usize, width: usize, height: usize) -> BlockId {
        let id = map.blocks.insert_with_key(|id| Block {
            x, y, width, height, id, portals: vec![],
        });
        for yy in y..y + height {
            for xx in x..x + width { map.block_map[yy * map.width + xx] = Cell::Block(id); }
        }
        id
    }

    #[test]
    fn goal_block_steers_directly_to_goal() {
        let map = Map::new(&Obstacle::new(6, 4));
        assert_eq!(map.blocks.len(), 1);
        let goal = Pos { x: 1, y: 1 };
        let options = FlowFieldOptions { wall_repulsion_strength: 0.0, ..Default::default() };
        let field = map.build_flow_field(goal, &options).unwrap();
        for y in 0..4 {
            for x in 0..6 {
                let actual = field[y * 6 + x].unwrap();
                let expected = map.get_norm(Pos { x, y }, goal);
                assert!((actual.0 - expected.0).abs() < 1e-6);
                assert!((actual.1 - expected.1).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn block_portals_cross_in_all_four_directions() {
        for vertical in [false, true] {
            let mut map = Map::new(&Obstacle::new(6, 4));
            clear_topology(&mut map);
            let ids = if vertical {
                [block(&mut map, 0, 0, 3, 4), block(&mut map, 3, 0, 3, 4)]
            } else {
                [block(&mut map, 0, 0, 6, 2), block(&mut map, 0, 2, 6, 2)]
            };
            map.build_portals();
            for goal_index in 0..2 {
                let source_index = 1 - goal_index;
                let goal = map.blocks[ids[goal_index]].center();
                let routes = PortalRoutes::build(&map, goal).unwrap();
                let route = routes.exit_for(ids[source_index]).unwrap();
                let direction = if goal_index == 1 { 1 } else { -1 };
                let expected = if vertical { (direction, 0) } else { (0, direction) };
                let (line, exit_step) = route.side_from(ids[source_index]).unwrap();
                assert_eq!(exit_step, expected);
                let options = FlowFieldOptions { wall_repulsion_strength: 0.0, ..Default::default() };
                let field = map.build_flow_field(goal, &options).unwrap();
                let p = line.mid_point();
                assert_eq!(map.get_cell(p.x, p.y), Cell::Block(ids[source_index]));
                assert_eq!(field[p.y * 6 + p.x], Some((expected.0 as f32, expected.1 as f32)));
                assert_eq!(field[goal.y * 6 + goal.x], Some((0.0, 0.0)));
            }
        }
    }

    fn map_with_two_row_blocks(obstacle: Obstacle) -> (Map, [BlockId; 2]) {
        let width = obstacle.width;
        let mut map = Map::new(&obstacle);
        clear_topology(&mut map);
        let ids = [block(&mut map, 0, 0, width, 1), block(&mut map, 0, 1, width, 1)];
        map.build_portals();
        (map, ids)
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
        assert_eq!(Map::new(&obstacle).get_cell(1, 0), Cell::Wall);
    }

    #[test]
    fn generated_blocks_do_not_contain_blocked_edges() {
        let mut obstacle = Obstacle::new(4, 4);
        obstacle.set_edge(0, 0, Side::Right);
        obstacle.set_edge(2, 1, Side::Bottom);
        obstacle.set_edge(1, 2, Side::Right);
        let map = Map::new(&obstacle);
        for block in map.blocks.values() {
            for y in block.y..block.y + block.height {
                for x in block.x..block.x + block.width.saturating_sub(1) {
                    assert!(map.can_pass(x, y, EDGE_RIGHT));
                }
            }
            for y in block.y..block.y + block.height.saturating_sub(1) {
                for x in block.x..block.x + block.width { assert!(map.can_pass(x, y, EDGE_BOTTOM)); }
            }
        }
    }

    #[test]
    fn blocked_edge_splits_neighbor_portals() {
        let mut obstacle = Obstacle::new(5, 2);
        obstacle.set_edge(2, 0, Side::Bottom);
        let (map, [a, b]) = map_with_two_row_blocks(obstacle);
        let neighbors = map.get_neighbors(a).iter().map(|&id| &map.portals[id]).collect::<Vec<_>>();
        for (from, to) in [(0, 2), (3, 5)] {
            assert!(neighbors.contains(&&Portal {
                blocks: [a, b],
                line: OrthogonalLine { is_vertical: false, fixed: 0, from, to },
            }));
        }
        assert_eq!(neighbors.len(), 2);
        assert_eq!(map.get_neighbors(a), map.get_neighbors(b));
        assert_eq!(map.portals.len(), 2);
    }

    #[test]
    fn fully_blocked_boundary_has_no_neighbor() {
        let mut obstacle = Obstacle::new(3, 2);
        for x in 0..3 { obstacle.set_edge(x, 0, Side::Bottom); }
        let (map, [a, b]) = map_with_two_row_blocks(obstacle);
        assert!(map.get_neighbors(a).is_empty());
        assert!(map.get_neighbors(b).is_empty());
        assert!(map.portals.is_empty());
    }

    #[test]
    fn aspect_ratio_is_bounded_for_blocks() {
        for (width, height) in [(33, 1), (1, 33), (31, 7), (7, 31), (32, 32)] {
            let map = Map::new(&Obstacle::new(width, height));
            for block in map.blocks.values() {
                assert!(block.width.max(block.height) <= block.width.min(block.height) * MAX_ASPECT_RATIO);
            }
            for y in 0..height {
                for x in 0..width { assert!(matches!(map.get_cell(x, y), Cell::Block(_))); }
            }
            if width.max(height) > width.min(height) * MAX_ASPECT_RATIO {
                assert!(map.blocks.len() > 1);
            }
        }
    }

    #[test]
    fn expansion_keeps_one_legal_direction_when_two_steps_would_exceed_ratio() {
        for vertical in [false, true] {
            let (width, height, x, y) = if vertical { (1, 9, 0, 4) } else { (9, 1, 4, 0) };
            let mut map = Map::new(&Obstacle::new(width, height));
            clear_topology(&mut map);
            let mut candidate = Block {
                x, y, width: 1, height: 1, id: BlockId::default(), portals: vec![],
            };
            assert!(candidate.expand(&map).is_some());
            assert_eq!(candidate.width.max(candidate.height), MAX_ASPECT_RATIO);
            assert!(candidate.expand(&map).is_none());
        }
    }
}
