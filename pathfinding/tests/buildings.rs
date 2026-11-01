use chunk_flow_field::{
    map::{Map, Obstacle, Side, EDGE_RIGHT},
    types::{Cell, FlowField, FlowFieldOptions, Pos},
};

fn wall(width: usize, height: usize, x: usize, y: usize) -> Obstacle {
    let mut obstacle = Obstacle::new(width, height);
    obstacle.set_cell_blocked(x, y);
    obstacle
}

#[test]
fn removal_restores_flow_and_preserves_base_and_other_buildings() {
    let mut map = Map::new(&wall(5, 3, 4, 2));
    let mut field = FlowField::new(Pos { x: 0, y: 0 }, &FlowFieldOptions::default(), &map).unwrap();
    let first = map.add_building(wall(5, 3, 1, 1)).unwrap();
    let second = map.add_building(wall(5, 3, 2, 1)).unwrap();
    assert_eq!(map.get_generation(), 2);
    assert!(!field.is_valid(&map));
    assert_eq!(field.get_valid_field(&map).unwrap()[6], None);
    map.remove_building(first).unwrap();
    assert_eq!(map.get_generation(), 3);
    assert!(!field.is_valid(&map));
    assert!(matches!(map.get_cell(1, 1), Cell::Block(_)));
    assert_eq!(map.get_cell(2, 1), Cell::Wall);
    assert_eq!(map.get_cell(4, 2), Cell::Wall);
    assert!(field.get_valid_field(&map).unwrap()[6].is_some());
    assert!(field.is_valid(&map));
    map.remove_building(second).unwrap();
    assert!(matches!(map.get_cell(2, 1), Cell::Block(_)));
    assert_eq!(map.get_cell(4, 2), Cell::Wall);
}

#[test]
fn rejected_placement_is_atomic_and_ids_are_not_reused() {
    let mut map = Map::new(&wall(4, 3, 0, 0));
    let id = map.add_building(wall(4, 3, 1, 1)).unwrap();
    let generation = map.get_generation();
    let mut overlapping = wall(4, 3, 1, 1);
    overlapping.set_cell_blocked(2, 1);
    assert!(map.add_building(overlapping).is_err());
    assert!(map.add_building(wall(4, 3, 0, 0)).is_err());
    assert!(map.add_building(Obstacle::new(4, 3)).is_err());
    assert!(map.add_building(wall(3, 3, 1, 1)).is_err());
    assert_eq!(map.get_generation(), generation);
    assert!(matches!(map.get_cell(2, 1), Cell::Block(_)));
    map.remove_building(id).unwrap();
    let next = map.add_building(wall(4, 3, 1, 1)).unwrap();
    assert_ne!(id, next);
    let generation = map.get_generation();
    assert!(map.remove_building(id).is_err());
    assert_eq!(map.get_generation(), generation);
    assert_eq!(map.get_cell(1, 1), Cell::Wall);
}

#[test]
fn edge_cells_are_reserved_and_removal_restores_base_edges() {
    let mut base = Obstacle::new(5, 3);
    base.set_edge(0, 0, Side::Right);
    let mut map = Map::new(&base);
    assert!(map.add_building(wall(5, 3, 1, 0)).is_err());
    let mut edge = Obstacle::new(5, 3);
    edge.set_edge(2, 1, Side::Right);
    let id = map.add_building(edge.clone()).unwrap();
    assert!(!map.can_pass(2, 1, EDGE_RIGHT));
    assert!(map.add_building(edge).is_err());
    assert!(map.add_building(wall(5, 3, 3, 1)).is_err());
    map.remove_building(id).unwrap();
    assert!(map.can_pass(2, 1, EDGE_RIGHT));
    assert!(!map.can_pass(0, 0, EDGE_RIGHT));
}
