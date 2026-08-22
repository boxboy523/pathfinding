use std::ops::{Add, Sub};

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
    Block(usize),
    Empty,
}

#[derive(Debug, Clone, Copy, PartialEq)]
/// goal에서 역방향으로 block route를 탐색할 때 priority queue에 저장하는 node이다.
pub struct VisitNode {
    /// 탐색 대상 block index.
    pub block_id: usize,
    /// 해당 block이 goal 방향으로 향할 경계 구간이다.
    pub entry: OrthogonalLine,
    /// goal에서 현재 node까지의 누적 추정 비용이다.
    pub cost: f32,
    /// 역방향 탐색의 부모이며, 실제 이동에서는 goal 쪽 다음 block이다.
    pub from: Option<usize>,
}

impl Ord for VisitNode {
    /// =BinaryHeap=이 낮은 cost를 먼저 꺼내도록 역순으로 비교한다.
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}
impl PartialOrd for VisitNode {
    /// =Ord=와 동일한 순서를 사용한다.
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for VisitNode {}

/// goal을 향한 cell별 단위 방향 vector를 보관한다.
pub struct FlowField {
    pub goal: Pos,
    /// grid index별 =(dx, dy)= 단위 vector이며, wall 또는 도달 불가 cell은 =None=이다.
    pub flow: Vec<Option<(f32, f32)>>,
}

/// goal에서 역방향으로 block route를 탐색한 결과를 저장한다.
#[derive(Debug, Clone, Copy)]
pub struct BlockRoute {
    pub cost: f32,
    pub exit_portal: OrthogonalLine,
    pub next_block: Option<usize>,
}
