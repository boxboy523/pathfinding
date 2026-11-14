use std::collections::BTreeSet;

use chunk_flow_field::{
    map::{Map, Obstacle, Side, EDGE_BOTTOM, EDGE_RIGHT},
    types::{Cell, FlowFieldOptions, Pos},
    BlockId, PortalId,
};

fn assert_shared_topology(map: &Map) {
    let mut unique = BTreeSet::new();
    for (id, portal) in &map.portals {
        let [a, b] = portal.blocks;
        assert_ne!(a, b);
        assert!(!portal.line.is_empty());
        assert!(unique.insert((portal.line.is_vertical, portal.line.fixed, portal.line.from, portal.line.to)));
        for block in [a, b] {
            assert_eq!(map.get_neighbors(block).iter().filter(|&&p| p == id).count(), 1);
            let (line, step) = portal.side_from(block).unwrap();
            for offset in line.from..line.to {
                let pos = if line.is_vertical {
                    Pos { x: line.fixed, y: offset }
                } else {
                    Pos { x: offset, y: line.fixed }
                };
                let other = Pos {
                    x: pos.x.checked_add_signed(step.0).unwrap(),
                    y: pos.y.checked_add_signed(step.1).unwrap(),
                };
                assert_eq!(map.get_cell(pos.x, pos.y), Cell::Block(block));
                assert_eq!(map.get_cell(other.x, other.y), Cell::Block(portal.other_block(block).unwrap()));
            }
        }
    }
    // Every passable inter-Block cell edge must belong to exactly one Portal.
    for y in 0..map.height {
        for x in 0..map.width {
            let Cell::Block(block) = map.get_cell(x, y) else { continue; };
            for (nx, ny, edge) in [(x + 1, y, EDGE_RIGHT), (x, y + 1, EDGE_BOTTOM)] {
                let Cell::Block(other) = map.get_cell(nx, ny) else { continue; };
                if other == block || !map.can_pass(x, y, edge) { continue; }
                let count = map.get_neighbors(block).iter().filter(|&&id| {
                    let portal = &map.portals[id];
                    portal.other_block(block) == Some(other)
                        && portal.side_from(block).unwrap().0.contains(Pos { x, y })
                        && portal.line.is_vertical == (edge == EDGE_RIGHT)
                }).count();
                assert_eq!(count, 1, "cell edge ({x},{y}) -> ({nx},{ny})");
            }
        }
    }
}

#[test]
fn shared_portals_cover_split_block_boundaries() {
    for seed in 0..12_u32 {
        let mut obstacle = Obstacle::new(8, 8);
        let mut state = seed + 1;
        for y in 0..8 {
            for x in 0..8 {
                state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                if state % 7 == 0 { obstacle.set_cell_blocked(x, y); }
                if state % 11 == 0 { obstacle.set_edge(x, y, Side::Right); }
                if state % 13 == 0 { obstacle.set_edge(x, y, Side::Bottom); }
            }
        }
        let map = Map::new(&obstacle);
        assert_shared_topology(&map);
        if let Some(goal) = (0..8).flat_map(|y| (0..8).map(move |x| Pos { x, y }))
            .find(|p| matches!(map.get_cell(p.x, p.y), Cell::Block(_)))
        {
            assert_eq!(map.build_flow_field(goal, &FlowFieldOptions::default()).unwrap().len(), 64);
        }
    }
}

#[test]
fn topology_rebuild_refreshes_shared_portal_ids_and_adjacency() {
    let mut map = Map::new(&Obstacle::new(12, 8));
    let mut building = Obstacle::new(12, 8);
    for y in 1..7 { building.set_cell_blocked(6, y); }
    let id = map.add_building(building).unwrap();
    assert!(!map.portals.is_empty());
    assert_shared_topology(&map);
    map.remove_building(id).unwrap();
    assert_shared_topology(&map);
    // Removal fills the reopened cells without coalescing unaffected Blocks.
    for y in 0..8 {
        for x in 0..12 { assert!(matches!(map.get_cell(x, y), Cell::Block(_))); }
    }
}

#[test]
fn topology_keys_stay_invalid_after_repeated_slot_reuse() {
    let mut base = Obstacle::new(12, 8);
    for y in 1..7 { base.set_cell_blocked(6, y); }
    let mut map = Map::new(&base);
    let mut expired = (Vec::<BlockId>::new(), Vec::<PortalId>::new());
    for _ in 0..3 {
        let Cell::Block(block) = map.get_cell(2, 3) else { panic!("open cell"); };
        let portals = map.blocks[block].portals.clone();
        let mut building = Obstacle::new(12, 8);
        building.set_cell_blocked(2, 3);
        let id = map.add_building(building).unwrap();
        expired.0.push(block);
        expired.1.extend(portals);
        for &old in &expired.0 { assert!(!map.blocks.contains_key(old)); }
        for &old in &expired.1 { assert!(!map.portals.contains_key(old)); }
        assert_shared_topology(&map);
        // Opening a formerly blocked cell does not invalidate any existing Block.
        let kept = map.blocks.keys().collect::<Vec<_>>();
        map.remove_building(id).unwrap();
        for old in kept { assert!(map.blocks.contains_key(old)); }
        for &old in &expired.0 { assert!(!map.blocks.contains_key(old)); }
        for &old in &expired.1 { assert!(!map.portals.contains_key(old)); }
        assert_shared_topology(&map);
    }
}

#[test]
fn local_updates_preserve_unaffected_objects_and_keys() {
    let mut base = Obstacle::new(64, 8);
    for y in 0..8 { base.set_cell_blocked(31, y); }
    base.set_cell_blocked(50, 3);
    let mut map = Map::new(&base);
    let blocks = map.blocks.iter().filter(|(_, b)| b.x > 31)
        .map(|(id, b)| (id, (b.x, b.y, b.width, b.height))).collect::<Vec<_>>();
    let portals = map.portals.iter().filter(|(_, p)| p.blocks.iter().all(|&b| map.blocks[b].x > 31))
        .map(|(id, p)| (id, p.clone())).collect::<Vec<_>>();
    assert!(!blocks.is_empty() && !portals.is_empty());
    for _ in 0..4 {
        let mut building = Obstacle::new(64, 8);
        building.set_cell_blocked(3, 3);
        let id = map.add_building(building).unwrap();
        for remove in [false, true] {
            if remove { map.remove_building(id).unwrap(); }
            assert_shared_topology(&map);
            for &(id, geometry) in &blocks {
                let b = &map.blocks[id];
                assert_eq!((b.x, b.y, b.width, b.height), geometry);
            }
            for (id, portal) in &portals { assert_eq!(&map.portals[*id], portal); }
        }
    }
}

#[test]
fn mixed_local_edits_match_full_rebuild_walkability_and_reachability() {
    use std::collections::VecDeque;
    for seed in 0..6_u32 {
        let mut base = Obstacle::new(16, 12);
        for y in 2..10 { base.set_cell_blocked(8, y); }
        let mut map = Map::new(&base);
        let mut buildings = Vec::new();
        let mut state = seed + 1;
        for step in 0..24 {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            let (x, y) = ((state as usize >> 8) % 16, (state as usize >> 16) % 12);
            let mut addition = Obstacle::new(16, 12);
            if step % 3 == 0 { addition.set_edge(x, y, Side::Right); }
            else if step % 3 == 1 { addition.set_edge(x, y, Side::Bottom); }
            else { addition.set_cell_blocked(x, y); }
            if let Ok(id) = map.add_building(addition.clone()) { buildings.push((id, addition)); }
            if step % 4 == 3 && !buildings.is_empty() {
                let (id, _) = buildings.remove(0);
                map.remove_building(id).unwrap();
            }
            assert_shared_topology(&map);
            let mut combined = base.clone();
            for (_, obstacle) in &buildings { combined.merge(obstacle).unwrap(); }
            let reference = Map::new(&combined);
            for y in 0..12 {
                for x in 0..16 {
                    assert_eq!(map.get_cell(x, y) == Cell::Wall, reference.get_cell(x, y) == Cell::Wall);
                    for edge in [1, 2, 4, 8] { assert_eq!(map.can_pass(x, y, edge), reference.can_pass(x, y, edge)); }
                    if let Cell::Block(id) = map.get_cell(x, y) {
                        let b = &map.blocks[id];
                        assert!(x >= b.x && x < b.x + b.width && y >= b.y && y < b.y + b.height);
                        assert_eq!(b.id, id);
                    }
                }
            }
            let goal = (0..12).flat_map(|y| (0..16).map(move |x| Pos { x, y }))
                .find(|p| reference.get_cell(p.x, p.y) != Cell::Wall).unwrap();
            let mut reachable = vec![false; 16 * 12];
            let mut queue = VecDeque::from([goal]);
            reachable[goal.y * 16 + goal.x] = true;
            while let Some(p) = queue.pop_front() {
                for (dx, dy, edge) in [(0, -1, 1), (1, 0, 2), (0, 1, 4), (-1, 0, 8)] {
                    let (Some(x), Some(y)) = (p.x.checked_add_signed(dx), p.y.checked_add_signed(dy)) else { continue; };
                    if x >= 16 || y >= 12 || !reference.can_pass(p.x, p.y, edge) || reference.get_cell(x, y) == Cell::Wall { continue; }
                    if !reachable[y * 16 + x] { reachable[y * 16 + x] = true; queue.push_back(Pos { x, y }); }
                }
            }
            let flow = map.build_flow_field(goal, &FlowFieldOptions::default()).unwrap();
            assert_eq!(flow.iter().map(Option::is_some).collect::<Vec<_>>(), reachable);
        }
    }
}

#[test]
fn nearby_surviving_blocks_and_portals_keep_their_keys() {
    let mut base = Obstacle::new(9, 9);
    for y in 0..9 {
        for x in 0..9 { if x + y > 8 { base.set_cell_blocked(x, y); } }
    }
    let mut map = Map::new(&base);
    assert!(map.blocks.len() > 2);
    let victim = map.blocks.keys().next().unwrap();
    let pos = map.blocks[victim].center();
    let survivors = map.blocks.iter().filter(|(id, _)| *id != victim)
        .map(|(id, b)| (id, (b.x, b.y, b.width, b.height))).collect::<Vec<_>>();
    let kept_portals = map.portals.iter().filter(|(_, p)| !p.blocks.contains(&victim))
        .map(|(id, p)| (id, p.clone())).collect::<Vec<_>>();
    assert!(!kept_portals.is_empty());
    let mut building = Obstacle::new(9, 9);
    building.set_cell_blocked(pos.x, pos.y);
    map.add_building(building).unwrap();
    assert!(!map.blocks.contains_key(victim));
    for (id, geometry) in survivors {
        let b = &map.blocks[id];
        assert_eq!((b.x, b.y, b.width, b.height), geometry);
    }
    for (id, portal) in kept_portals { assert_eq!(map.portals[id], portal); }
    assert_shared_topology(&map);
}
