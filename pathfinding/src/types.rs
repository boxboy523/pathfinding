use std::ops::{Add, Sub};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Pos {
    pub x: usize,
    pub y: usize,
}

impl Add for Pos {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Pos {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct OrthogonalLine {
    pub is_vertical: bool,
    pub fixed: usize,
    pub from: usize, // contained
    pub to: usize,   // not contained
}

impl OrthogonalLine {
    pub fn from_points(a: Pos, b: Pos) -> Option<Self> {
        if a.x == b.x {
            Some(OrthogonalLine {
                is_vertical: true,
                fixed: a.x,
                from: a.y.min(b.y),
                to: a.y.max(b.y),
            })
        } else if a.y == b.y {
            Some(OrthogonalLine {
                is_vertical: false,
                fixed: a.y,
                from: a.x.min(b.x),
                to: a.x.max(b.x),
            })
        } else {
            None
        }
    }

    pub fn contains(&self, pos: Pos) -> bool {
        if self.is_vertical {
            pos.x == self.fixed && pos.y >= self.from && pos.y < self.to
        } else {
            pos.y == self.fixed && pos.x >= self.from && pos.x < self.to
        }
    }

    pub fn len(&self) -> usize {
        self.to - self.from
    }

    pub fn mid_point(&self) -> Pos {
        if self.is_vertical {
            Pos {
                x: self.fixed,
                y: (self.from + self.to) / 2,
            }
        } else {
            Pos {
                x: (self.from + self.to) / 2,
                y: self.fixed,
            }
        }
    }

    pub fn nearest_point(&self, pos: Pos) -> Pos {
        if self.is_vertical {
            Pos {
                x: self.fixed,
                y: pos.y.clamp(self.from, self.to - 1),
            }
        } else {
            Pos {
                x: pos.x.clamp(self.from, self.to - 1),
                y: self.fixed,
            }
        }
    }
}

pub enum Side {
    Top,
    Bottom,
    Left,
    Right,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cell {
    Wall,
    Block(usize),
    Empty,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VisitNode {
    pub block_id: usize,
    pub entry: OrthogonalLine,
    pub cost: f32,
    pub from: Option<usize>,
}

impl Ord for VisitNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}
impl PartialOrd for VisitNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for VisitNode {}

pub struct FlowField {
    pub goal: Pos,
    pub flow: Vec<Option<(f32, f32)>>, // grid index -> flow norm vector (dx, dy)
}
