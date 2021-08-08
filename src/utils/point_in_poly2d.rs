use na::{Point2, RealField};

// FIXME: move this on ncollide_geometry.
/// Tests if the given point is inside of a polygon with arbitrary orientation.
pub fn point_in_poly2d<N: RealField + Copy>(pt: &Point2<N>, poly: &[Point2<N>]) -> bool {
    if poly.len() == 0 {
        false
    } else {
        let mut sign = N::zero();

        for i1 in 0..poly.len() {
            let i2 = (i1 + 1) % poly.len();
            let seg_dir = poly[i2] - poly[i1];
            let dpt = pt - poly[i1];
            let perp = dpt.perp(&seg_dir);

            if sign.is_zero() {
                sign = perp;
            } else if sign * perp < N::zero() {
                return false;
            }
        }

        return true;
    }
}
