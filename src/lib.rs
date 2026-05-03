pub struct Block {
    pub x: usize,
    pub y: usize,
    pub width: usize,
    pub height: usize,
}

impl Block {
    fn split(&mut self, map: &Map) -> Option<(Block, Block)> {
        let cx = self.x + self.width / 2;
        let cy = self.y + self.height / 2;

        let max_r = self.width.max(self.height);

        let width_longer = self.width >= self.height;

        for r in 0..max_r {
            for dy in -(r as isize)..=(r as isize) {
                for dx in -(r as isize)..=(r as isize) {
                    if dx.abs() != r as isize && dy.abs() != r as isize {
                        continue;
                    }

                    let nx = cx as isize + dx;
                    let ny = cy as isize + dy;

                    if nx < self.x as isize
                        || ny < self.y as isize
                        || nx >= (self.width + self.x) as isize
                        || ny >= (self.height + self.y) as isize
                    {
                        continue;
                    }
                    let nx = nx as usize;
                    let ny = ny as usize;

                    if map.is_wall(nx as usize, ny as usize) {
                        if width_longer {
                            let block1 = Block {
                                x: self.x,
                                y: self.y,
                                width: nx - self.x,
                                height: self.height,
                            };
                            let block2 = Block {
                                x: nx + 1,
                                y: self.y,
                                width: self.x + self.width - (nx + 1),
                                height: self.height,
                            };
                            return Some((block1, block2));
                        } else {
                            let block1 = Block {
                                x: self.x,
                                y: self.y,
                                width: self.width,
                                height: ny - self.y,
                            };
                            let block2 = Block {
                                x: self.x,
                                y: ny + 1,
                                width: self.width,
                                height: self.y + self.height - (ny + 1),
                            };
                            return Some((block1, block2));
                        }
                    }
                }
            }
        }
        None
    }

    pub fn expend(&mut self, map: &Map) {

    }
}

enum Cell {
    Wall,
    Block(usize),
    Empty
}

pub struct Map {
    pub width: usize,
    pub height: usize,
    pub block_map: Vec<Cell>,
    pub blocks: Vec<Block>,
}

impl Map {
    pub fn new(width: usize, height: usize, wall: Vec<bool>) -> Self {
        let block_map = vec![None; width * height];
        for w in
        let blocks = vec![];
        let mut map = Map {
            width,
            height,
            wall,
            block_map,
            blocks,
        };
        map.split_blocks();
        map
    }

    fn is_wall(&self, x: usize, y: usize) -> bool {
        self.wall[y * self.width + x]
    }

    fn split_blocks(&mut self) {
        let mut to_visit = vec![Block {
            x: 0,
            y: 0,
            width: self.width,
            height: self.height,
        }];

        while let Some(mut block) = to_visit.pop() {
            if block.width == 0 || block.height == 0 {
                continue;
            }
            if let Some((block1, block2)) = block.split(self) {
                to_visit.push(block1);
                to_visit.push(block2);
            } else {
                let block_id = self.blocks.len();
                for y in block.y..block.y + block.height {
                    for x in block.x..block.x + block.width {
                        self.block_map[y * self.width + x] = Some(block_id);
                    }
                }
                self.blocks.push(block);
            }
        }
    }
}
