//! Goal-specific reverse Dijkstra over shared Portal IDs. Each rectangular Block
//! implicitly connects its incident Portals; distances are computed on demand.
use std::{cmp::Ordering, collections::BinaryHeap};

use crate::{keys::BlockId, map::Map, portal::{Portal, PortalId}, types::{Cell, Pos}};
use slotmap::SecondaryMap;

#[derive(Debug)]
pub(crate) struct PortalRoutes<'a> {
    pub goal_block: BlockId,
    map: &'a Map,
    costs: SecondaryMap<PortalId, f32>,
    // Incident Block containing the next segment towards the goal.
    toward_block: SecondaryMap<PortalId, Option<BlockId>>,
    // One selected Portal per reachable non-goal Block.
    exits: SecondaryMap<BlockId, PortalId>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Visit {
    cost: f32,
    node: PortalId,
}
impl Eq for Visit {}
impl Ord for Visit {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.total_cmp(&self.cost).then_with(|| other.node.cmp(&self.node))
    }
}
impl PartialOrd for Visit {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}

fn distance(a: (f32, f32), b: (f32, f32)) -> f32 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

impl<'a> PortalRoutes<'a> {
    pub fn build(map: &'a Map, goal: Pos) -> anyhow::Result<Self> {
        let goal_block = match map.get_cell(goal.x, goal.y) {
            Cell::Block(id) => id,
            _ => anyhow::bail!("Goal is not on a block"),
        };
        let mut routes = Self {
            goal_block, map,
            costs: map.portals.keys().map(|id| (id, f32::INFINITY)).collect(),
            toward_block: map.portals.keys().map(|id| (id, None)).collect(),
            exits: SecondaryMap::new(),
        };
        routes.search((goal.x as f32, goal.y as f32));
        routes.select_exits();
        Ok(routes)
    }

    fn search(&mut self, goal: (f32, f32)) {
        let mut queue = BinaryHeap::new();
        // Virtual goal node: seed each Portal in the goal Block.
        for &node in &self.map.blocks[self.goal_block].portals {
            let cost = distance(goal, self.map.portals[node].midpoint());
            self.costs[node] = cost;
            self.toward_block[node] = Some(self.goal_block);
            queue.push(Visit { cost, node });
        }
        while let Some(Visit { cost, node }) = queue.pop() {
            if cost != self.costs[node] { continue; }
            let portal = &self.map.portals[node];
            for block in portal.blocks {
                for &next_node in &self.map.blocks[block].portals {
                    if next_node == node { continue; }
                    let candidate = cost + distance(portal.midpoint(), self.map.portals[next_node].midpoint());
                    if candidate < self.costs[next_node] {
                        self.costs[next_node] = candidate;
                        self.toward_block[next_node] = Some(block);
                        queue.push(Visit { cost: candidate, node: next_node });
                    }
                }
            }
        }
    }

    fn select_exits(&mut self) {
        for (block_id, block) in &self.map.blocks {
            if block_id == self.goal_block { continue; }
            let point = (
                block.x as f32 + (block.width - 1) as f32 * 0.5,
                block.y as f32 + (block.height - 1) as f32 * 0.5,
            );
            let mut best_cost = f32::INFINITY;
            for &node in &block.portals {
                if self.toward_block[node].is_none() || self.toward_block[node] == Some(block_id) {
                    continue;
                }
                let cost = distance(point, self.map.portals[node].midpoint()) + self.costs[node];
                if cost < best_cost {
                    best_cost = cost;
                    self.exits.insert(block_id, node);
                }
            }
        }
    }

    /// Constant-time lookup: all cells in a Block share this Map-owned Portal.
    pub fn exit_for(&self, block: BlockId) -> Option<&'a Portal> {
        self.exits.get(block).map(|&id| &self.map.portals[id])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{block::Block, map::Obstacle, types::OrthogonalLine};

    // Synthetic graph fixtures; real cell-side crossing is tested in map.rs/portal.rs.
    fn graph(openings: Vec<(usize, usize, [usize; 2])>, count: usize) -> (Map, Vec<BlockId>, Vec<PortalId>) {
        let mut map = Map::new(&Obstacle::new(16, 16));
        let first = map.blocks.keys().next().unwrap();
        map.portals.clear();
        map.blocks[first].portals.clear();
        let mut blocks = vec![first];
        for _ in 1..count {
            blocks.push(map.blocks.insert_with_key(|id| Block {
                x: 0, y: 0, width: 16, height: 16, id, portals: vec![],
            }));
        }
        let portals = openings.into_iter().map(|(x, y, indices)| {
            let id = map.portals.insert(Portal {
                blocks: indices.map(|i| blocks[i]),
                line: OrthogonalLine { is_vertical: false, fixed: y, from: x, to: x + 1 },
            });
            for index in indices { map.blocks[blocks[index]].portals.push(id); }
            id
        }).collect();
        (map, blocks, portals)
    }

    #[test]
    fn all_cells_in_one_block_share_one_exit() {
        let (map, blocks, portals) = graph(vec![(0, 0, [0, 1]), (10, 0, [0, 1])], 2);
        let routes = PortalRoutes::build(&map, Pos { x: 1, y: 0 }).unwrap();
        assert!(routes.costs[portals[0]] < routes.costs[portals[1]]);
        assert!(std::ptr::eq(routes.exit_for(blocks[1]).unwrap(), &map.portals[portals[0]]));
        assert!(routes.exit_for(blocks[0]).is_none());
        assert_eq!(routes.exits.len(), 1);
    }

    #[test]
    fn cheaper_later_relaxation_replaces_an_already_discovered_portal() {
        let (map, blocks, portals) = graph(vec![
            (1, 0, [0, 1]),
            (0, 2, [0, 2]),
            (0, 10, [1, 2]),
            (9, 9, [3, 4]), // disconnected
        ], 5);
        let routes = PortalRoutes::build(&map, Pos { x: 0, y: 0 }).unwrap();
        assert!((routes.costs[portals[2]] - 10.5).abs() < 1e-6);
        assert_eq!(routes.toward_block[portals[2]], Some(blocks[2]));
        assert_eq!(routes.costs[portals[3]], f32::INFINITY);
        assert!(routes.exit_for(blocks[3]).is_none());
    }

    #[test]
    fn block_center_is_used_to_choose_the_exit() {
        let (map, blocks, portals) = graph(vec![(0, 0, [0, 1]), (10, 0, [0, 1])], 2);
        let routes = PortalRoutes::build(&map, Pos { x: 5, y: 0 }).unwrap();
        assert_eq!(routes.costs[portals[0]], routes.costs[portals[1]]);
        assert!(std::ptr::eq(routes.exit_for(blocks[1]).unwrap(), &map.portals[portals[1]]));
    }

    #[test]
    fn selected_block_routes_do_not_cycle() {
        use std::collections::BTreeSet;
        for seed in 0..64_u32 {
            let mut obstacle = Obstacle::new(8, 8);
            let mut state = seed + 1;
            for y in 0..8 {
                for x in 0..8 {
                    state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                    if state % 7 == 0 { obstacle.set_cell_blocked(x, y); }
                }
            }
            let map = Map::new(&obstacle);
            for goal_block in map.blocks.values() {
                let routes = PortalRoutes::build(&map, goal_block.center()).unwrap();
                for start in map.blocks.keys() {
                    let mut block = start;
                    let mut seen = BTreeSet::new();
                    while let Some(portal) = routes.exit_for(block) {
                        assert!(seen.insert(block), "Block cycle: seed={seed}, start={start:?}, goal={:?}", routes.goal_block);
                        block = portal.other_block(block).unwrap();
                    }
                    if !seen.is_empty() { assert_eq!(block, routes.goal_block); }
                }
            }
        }
    }
}
