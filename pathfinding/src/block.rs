use crate::{
    map::{EDGE_BOTTOM, EDGE_LEFT, EDGE_RIGHT, EDGE_TOP, Map},
    types::{Cell, OrthogonalLine, Pos, Side},
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Portal {
    pub neighbor: usize,
    pub line: OrthogonalLine,
}

#[derive(Debug)]
pub struct Block {
    pub x: usize,
    pub y: usize,
    pub width: usize,
    pub height: usize,
    pub id: usize,
    pub chunk_id: Option<usize>,
    pub portals: Vec<Portal>,
}

impl Block {
    pub fn expand(&mut self, map: &Map) -> Option<()> {
        let mut expand_y_up = true;
        let mut expand_y_down = true;
        for x in self.x..self.x + self.width {
            if self.y == 0
                || matches!(map.get_cell(x, self.y - 1), Cell::Wall | Cell::Block(_))
                || !(map.can_pass(x, self.y, EDGE_TOP))
            {
                expand_y_up = false;
            }
            if (self.y + self.height == map.height)
                || matches!(
                    map.get_cell(x, self.y + self.height),
                    Cell::Wall | Cell::Block(_)
                )
                || !(map.can_pass(x, self.y + self.height - 1, EDGE_BOTTOM))
            {
                expand_y_down = false;
            }
        }
        if expand_y_up {
            let new_y = self.y - 1;
            let end_x = self.x + self.width.saturating_sub(1);
            expand_y_up = (self.x..end_x).all(|x| map.can_pass(x, new_y, EDGE_RIGHT));
        }
        if expand_y_down {
            let new_y = self.y + self.height;
            let end_x = self.x + self.width.saturating_sub(1);
            expand_y_down = (self.x..end_x).all(|x| map.can_pass(x, new_y, EDGE_RIGHT));
        }
        let mut expand_x_left = true;
        let mut expand_x_right = true;
        for y in self.y..self.y + self.height {
            if self.x == 0
                || matches!(map.get_cell(self.x - 1, y), Cell::Wall | Cell::Block(_))
                || !(map.can_pass(self.x, y, EDGE_LEFT))
            {
                expand_x_left = false;
            }
            if (self.x + self.width == map.width)
                || matches!(
                    map.get_cell(self.x + self.width, y),
                    Cell::Wall | Cell::Block(_)
                )
                || !(map.can_pass(self.x + self.width - 1, y, EDGE_RIGHT))
            {
                expand_x_right = false;
            }
        }
        if expand_x_left {
            let new_x = self.x - 1;
            let end_y = self.y + self.height.saturating_sub(1);
            expand_x_left = (self.y..end_y).all(|y| map.can_pass(new_x, y, EDGE_BOTTOM));
        }
        if expand_x_right {
            let new_x = self.x + self.width;
            let end_y = self.y + self.height.saturating_sub(1);
            expand_x_right = (self.y..end_y).all(|y| map.can_pass(new_x, y, EDGE_BOTTOM));
        }

        if self.width >= self.height {
            if expand_y_up || expand_y_down {
                if expand_y_up {
                    self.y -= 1;
                    self.height += 1;
                }
                if expand_y_down {
                    self.height += 1;
                }
                return Some(());
            } else if expand_x_left || expand_x_right {
                if expand_x_left {
                    self.x -= 1;
                    self.width += 1;
                }
                if expand_x_right {
                    self.width += 1;
                }
                return Some(());
            }
            None
        } else {
            if expand_x_left || expand_x_right {
                if expand_x_left {
                    self.x -= 1;
                    self.width += 1;
                }
                if expand_x_right {
                    self.width += 1;
                }
                return Some(());
            } else if expand_y_up || expand_y_down {
                if expand_y_up {
                    self.y -= 1;
                    self.height += 1;
                }
                if expand_y_down {
                    self.height += 1;
                }
                return Some(());
            }
            None
        }
    }

    /// block의 정수 grid 중심 cell을 반환한다.
    pub fn center(&self) -> Pos {
        Pos {
            x: self.x + self.width / 2,
            y: self.y + self.height / 2,
        }
    }

    /// 주어진 방향에 해당하는 block 경계 선분을 반환한다.
    pub fn side_line(&self, side: Side) -> OrthogonalLine {
        match side {
            Side::Top => OrthogonalLine {
                is_vertical: false,
                fixed: self.y,
                from: self.x,
                to: self.x + self.width,
            },
            Side::Bottom => OrthogonalLine {
                is_vertical: false,
                fixed: self.y + self.height - 1,
                from: self.x,
                to: self.x + self.width,
            },
            Side::Left => OrthogonalLine {
                is_vertical: true,
                fixed: self.x,
                from: self.y,
                to: self.y + self.height,
            },
            Side::Right => OrthogonalLine {
                is_vertical: true,
                fixed: self.x + self.width - 1,
                from: self.y,
                to: self.y + self.height,
            },
        }
    }

    pub fn get_vertices(&self) -> Vec<Pos> {
        vec![
            Pos {
                x: self.x,
                y: self.y,
            },
            Pos {
                x: self.x + self.width - 1,
                y: self.y,
            },
            Pos {
                x: self.x,
                y: self.y + self.height - 1,
            },
            Pos {
                x: self.x + self.width - 1,
                y: self.y + self.height - 1,
            },
        ]
    }
}
