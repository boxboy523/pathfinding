use crate::types::Pos;

pub fn convex_hull(points: &[Pos]) -> Vec<Pos> {
    let mut points = points.to_vec();
    points.sort_by(|a, b| a.x.cmp(&b.x).then(a.y.cmp(&b.y)));
    points.dedup();

    if points.len() <= 2 {
        return points;
    }

    fn cross(a: Pos, b: Pos, c: Pos) -> i128 {
        let abx = b.x as i128 - a.x as i128;
        let aby = b.y as i128 - a.y as i128;
        let acx = c.x as i128 - a.x as i128;
        let acy = c.y as i128 - a.y as i128;
        abx * acy - aby * acx
    }

    fn half(points: impl Iterator<Item = Pos>) -> Vec<Pos> {
        let mut hull = Vec::new();

        for p in points {
            while hull.len() >= 2 {
                let n = hull.len();
                if cross(hull[n - 2], hull[n - 1], p) > 0 {
                    break;
                }
                hull.pop();
            }
            hull.push(p);
        }

        hull
    }

    let mut lower = half(points.iter().copied());
    let mut upper = half(points.iter().rev().copied());

    // 양 끝점이 양쪽 껍질에 중복되어 있으므로 제거.
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// hull과 겹치는 면적이 타일 면적의 min_coverage 이상인 타일을 반환한다.
/// Pos는 타일 중심이며, 타일 영역은 (x ± 0.5, y ± 0.5)다.
/// 점/선분 hull은 면적이 없으므로 검사 대상 타일도 없다.
pub fn tiles_inside_hull(hull: &[Pos], min_coverage: f64) -> Vec<Pos> {
    assert!(min_coverage.is_finite() && min_coverage > 0.0 && min_coverage <= 1.0);
    if hull.len() < 3 {
        return vec![];
    }
    let min_x = hull.iter().map(|p| p.x).min().unwrap();
    let max_x = hull.iter().map(|p| p.x).max().unwrap();
    let min_y = hull.iter().map(|p| p.y).min().unwrap();
    let max_y = hull.iter().map(|p| p.y).max().unwrap();
    let mut tiles = Vec::new();
    for y in min_y..=max_y {
        for x in min_x..=max_x {
            if tile_overlap_area(hull, Pos { x, y }) + 1e-10 >= min_coverage {
                tiles.push(Pos { x, y });
            }
        }
    }
    tiles
}

/// 타일을 hull의 각 반평면으로 잘라 남은 다각형의 면적을 계산한다.
fn tile_overlap_area(hull: &[Pos], tile: Pos) -> f64 {
    // 타일 중심을 원점으로 사용해 면적 계산의 정밀도 손실을 줄인다.
    let mut polygon = vec![(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)];
    for i in 0..hull.len() {
        let a = hull[i];
        let b = hull[(i + 1) % hull.len()];
        let ax = (a.x as i128 - tile.x as i128) as f64;
        let ay = (a.y as i128 - tile.y as i128) as f64;
        let dx = (b.x as i128 - a.x as i128) as f64;
        let dy = (b.y as i128 - a.y as i128) as f64;
        let side = |p: (f64, f64)| dx * (p.1 - ay) - dy * (p.0 - ax);
        if polygon.is_empty() {
            return 0.0;
        }
        let mut clipped = Vec::new();
        let mut previous = *polygon.last().unwrap();
        let mut previous_side = side(previous);
        for current in polygon {
            let current_side = side(current);
            if (previous_side >= 0.0) != (current_side >= 0.0) {
                let t = previous_side / (previous_side - current_side);
                clipped.push((
                    previous.0 + t * (current.0 - previous.0),
                    previous.1 + t * (current.1 - previous.1),
                ));
            }
            if current_side >= 0.0 {
                clipped.push(current);
            }
            previous = current;
            previous_side = current_side;
        }
        polygon = clipped;
    }
    let mut twice_area = 0.0;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        twice_area += a.0 * b.1 - a.1 * b.0;
    }
    (twice_area.abs() * 0.5).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hull_removes_duplicates_and_interior_points_without_truncating_cross() {
        let points = [
            Pos { x: 0, y: 0 },
            Pos { x: 100_000, y: 0 },
            Pos {
                x: 100_000,
                y: 100_000,
            },
            Pos { x: 0, y: 100_000 },
            Pos { x: 1, y: 1 },
            Pos { x: 0, y: 0 },
        ];
        assert_eq!(convex_hull(&points), points[..4]);
    }

    #[test]
    fn coverage_threshold_includes_half_tiles_but_excludes_slivers() {
        let hull = convex_hull(&[Pos { x: 0, y: 0 }, Pos { x: 4, y: 0 }, Pos { x: 0, y: 4 }]);
        let tiles = tiles_inside_hull(&hull, 0.5);
        assert!(tiles.contains(&Pos { x: 1, y: 1 }));
        // (1.5, 2.5)는 hull 경계 위: 네 꼭짓점이 모두 포함된다.
        assert!(tiles.contains(&Pos { x: 1, y: 2 }));
        // 중심을 지나는 직선은 타일 면적의 정확히 절반을 덮는다.
        assert!(tiles.contains(&Pos { x: 2, y: 2 }));
        assert!(tiles.contains(&Pos { x: 0, y: 1 }));
        assert!(!tiles.contains(&Pos { x: 2, y: 3 }));
        assert!(!tiles_inside_hull(&hull, 0.6).contains(&Pos { x: 2, y: 2 }));
        assert!((tile_overlap_area(&hull, Pos { x: 2, y: 2 }) - 0.5).abs() < 1e-10);
        assert!(tile_overlap_area(&hull, Pos { x: 1, y: 1 }) > 0.99);
    }

    #[test]
    fn degenerate_hulls_contain_no_complete_tiles() {
        assert!(tiles_inside_hull(&[], 0.5).is_empty());
        assert!(tiles_inside_hull(&[Pos { x: 2, y: 3 }], 0.5).is_empty());
        let hull = convex_hull(&[Pos { x: 0, y: 0 }, Pos { x: 2, y: 2 }]);
        assert!(tiles_inside_hull(&hull, 0.5).is_empty());
    }
}
