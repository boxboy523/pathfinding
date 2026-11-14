use std::ops::{Add, Sub};

use crate::{keys::BlockId, map::Map};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
/// grid 내 cell 좌표이다.
pub struct Pos {
    /// 왼쪽에서부터의 column index.
    pub x: usize,
    /// 위쪽에서부터의 row index.
    pub y: usize,
}

impl Add for Pos {
    type Output = Self;

    /// 두 좌표의 각 성분을 더한다.
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Pos {
    type Output = Self;

    /// 두 좌표의 각 성분을 뺀다.
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
/// grid 위의 가로 또는 세로 선분이다. =from=은 포함하고 =to=는 포함하지 않는다.
pub struct OrthogonalLine {
    /// =true=면 x가 고정된 세로선, =false=면 y가 고정된 가로선이다.
    pub is_vertical: bool,
    /// 세로선의 x 또는 가로선의 y 좌표이다.
    pub fixed: usize,
    /// 가변 축 구간의 포함되는 시작점이다.
    pub from: usize,
    /// 가변 축 구간의 포함되지 않는 끝점이다.
    pub to: usize,
}

impl OrthogonalLine {
    /// 두 점이 같은 x 또는 y에 있으면 두 점 사이의 직교 선분을 만든다.
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

    /// 선분의 half-open 구간에 주어진 cell이 포함되는지 확인한다.
    pub fn contains(&self, pos: Pos) -> bool {
        if self.is_vertical {
            pos.x == self.fixed && pos.y >= self.from && pos.y < self.to
        } else {
            pos.y == self.fixed && pos.x >= self.from && pos.x < self.to
        }
    }

    /// 선분이 포함하는 grid cell 개수를 반환한다.
    pub fn len(&self) -> usize {
        self.to - self.from
    }

    /// 선분이 cell을 하나도 포함하지 않는지 확인한다.
    pub fn is_empty(&self) -> bool {
        self.from == self.to
    }

    /// 선분의 정수 grid 중간점을 반환한다.
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

    /// 주어진 cell에서 선분 위의 가장 가까운 cell을 반환한다.
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

    /// 3 cell 이상인 선분은 양 끝 1 cell을 제외한 구간에서 가장 가까운 cell을 반환한다.
    /// 1~2 cell인 선분은 기존 구간을 그대로 사용한다.
    pub fn nearest_point_with_margin(&self, pos: Pos) -> Pos {
        if self.len() < 3 {
            return self.nearest_point(pos);
        }
        Self {
            from: self.from + 1,
            to: self.to - 1,
            ..*self
        }
        .nearest_point(pos)
    }
}

/// 직사각형 block의 경계 방향이다.
pub enum Side {
    Top,
    Bottom,
    Left,
    Right,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// map의 cell이 wall인지, block에 속하는지, 아직 할당되지 않았는지를 나타낸다.
pub enum Cell {
    Wall,
    Block(BlockId),
    Empty,
}


#[derive(Debug, Clone, Copy)]
pub struct FlowFieldOptions {
    pub wall_margin: f32,
    pub wall_repulsion_strength: f32,
    pub max_repulsion_ratio: f32,
}

impl Default for FlowFieldOptions {
    fn default() -> Self {
        Self {
            wall_margin: 1.0,
            wall_repulsion_strength: 0.5,
            max_repulsion_ratio: 0.8,
        }
    }
}

/// goal을 향한 cell별 단위 방향 vector를 보관한다.
#[derive(Debug, Clone)]
pub struct FlowField {
    goal: Pos,
    /// grid index별 =(dx, dy)= 단위 vector이며, wall 또는 도달 불가 cell은 =None=이다.
    flow: Vec<Option<(f32, f32)>>,
    options: FlowFieldOptions,
    generation: usize,
}

impl FlowField {
    pub fn new(goal: Pos, options: &FlowFieldOptions, map: &Map) -> anyhow::Result<Self> {
        let flow = map.build_flow_field(goal, options)?;
        Ok(Self {
            goal,
            flow,
            options: *options,
            generation: map.get_generation(),
        })
    }

    pub fn empty(goal: Pos, options: &FlowFieldOptions) -> Self {
        Self {
            goal,
            flow: Vec::new(),
            options: *options,
            generation: 0,
        }
    }

    pub fn is_valid(&self, map: &Map) -> bool {
        self.generation == map.get_generation()
    }

    pub fn recompute(&mut self, map: &Map) -> anyhow::Result<()> {
        self.flow = map.build_flow_field(self.goal, &self.options)?;
        self.generation = map.get_generation();
        Ok(())
    }

    pub fn get_valid_field(&mut self, map: &Map) -> anyhow::Result<&[Option<(f32, f32)>]> {
        if !self.is_valid(map) {
            self.recompute(map)?;
        }
        Ok(&self.flow)
    }

    pub fn get_goal(&self) -> Pos {
        self.goal
    }
}
