use crate::{
    map::Map,
    types::{Cell, OrthogonalLine, Pos, Side},
};

#[derive(Debug)]
pub struct Block {
    pub x: usize,
    pub y: usize,
    pub width: usize,
    pub height: usize,
}

impl Block {
    pub fn expand(&mut self, map: &Map) -> Option<()> {
        let mut expand_y_up = true;
        let mut expand_y_down = true;
        for x in self.x..self.x + self.width {
            if self.y == 0 || matches!(map.get_cell(x, self.y - 1), Cell::Wall | Cell::Block(_)) {
                expand_y_up = false;
            }
            if (self.y + self.height == map.height)
                || matches!(
                    map.get_cell(x, self.y + self.height),
                    Cell::Wall | Cell::Block(_)
                )
            {
                expand_y_down = false;
            }
        }
        let mut expand_x_left = true;
        let mut expand_x_right = true;
        for y in self.y..self.y + self.height {
            if self.x == 0 || matches!(map.get_cell(self.x - 1, y), Cell::Wall | Cell::Block(_)) {
                expand_x_left = false;
            }
            if (self.x + self.width == map.width)
                || matches!(
                    map.get_cell(self.x + self.width, y),
                    Cell::Wall | Cell::Block(_)
                )
            {
                expand_x_right = false;
            }
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
            return None;
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
            return None;
        }
    }

    pub fn center(&self) -> Pos {
        Pos {
            x: self.x + self.width / 2,
            y: self.y + self.height / 2,
        }
    }

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
}
