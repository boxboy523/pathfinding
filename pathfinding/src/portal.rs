use crate::{keys::BlockId, types::OrthogonalLine};
pub use crate::keys::PortalId;

/// One shared opening between two Blocks, owned once by the Map.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Portal {
    /// First Block is left (vertical opening) or above (horizontal opening).
    pub blocks: [BlockId; 2],
    /// Boundary cell interval on the first Block's side, not the physical edge.
    pub line: OrthogonalLine,
}

impl Portal {
    pub fn other_block(&self, block: BlockId) -> Option<BlockId> {
        if self.blocks[0] == block {
            Some(self.blocks[1])
        } else if self.blocks[1] == block {
            Some(self.blocks[0])
        } else {
            None
        }
    }

    /// Cell-side interval and crossing direction seen from either incident Block.
    pub fn side_from(&self, block: BlockId) -> Option<(OrthogonalLine, (isize, isize))> {
        let direction = if block == self.blocks[0] {
            1
        } else if block == self.blocks[1] {
            -1
        } else {
            return None;
        };
        let mut line = self.line;
        if direction == -1 { line.fixed += 1; }
        let step = if line.is_vertical { (direction, 0) } else { (0, direction) };
        Some((line, step))
    }

    /// Shared representative in cell-center coordinates (physical boundary).
    pub fn midpoint(&self) -> (f32, f32) {
        let p = self.line.mid_point();
        if self.line.is_vertical {
            (p.x as f32 + 0.5, p.y as f32)
        } else {
            (p.x as f32, p.y as f32 + 0.5)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn either_side_refers_to_the_same_boundary() {
        let mut blocks = slotmap::SlotMap::<BlockId, ()>::with_key();
        let first = blocks.insert(());
        let second = blocks.insert(());
        let unrelated = blocks.insert(());
        for vertical in [false, true] {
            let portal = Portal {
                blocks: [first, second],
                line: OrthogonalLine { is_vertical: vertical, fixed: 3, from: 1, to: 4 },
            };
            let (a, step_a) = portal.side_from(first).unwrap();
            let (b, step_b) = portal.side_from(second).unwrap();
            assert_eq!(b.fixed, a.fixed + 1);
            assert_eq!(step_a, (-step_b.0, -step_b.1));
            let a = a.mid_point();
            let b = b.mid_point();
            assert_eq!(portal.midpoint(), ((a.x + b.x) as f32 / 2.0, (a.y + b.y) as f32 / 2.0));
            assert_eq!(portal.other_block(first), Some(second));
            assert_eq!(portal.other_block(second), Some(first));
            assert_eq!(portal.side_from(unrelated), None);
            assert_eq!(portal.other_block(unrelated), None);
        }
    }
}
