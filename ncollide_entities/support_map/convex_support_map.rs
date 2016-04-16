use na::{Transform, Rotate};
use support_map::SupportMap;
use support_map;
use shape::ConvexHull;
use math::Point;


impl<P, M> SupportMap<P, M> for ConvexHull<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inv_rotate(dir);

        let best_pt = support_map::point_cloud_support_point(&local_dir, self.points());

        m.transform(&best_pt)
    }
}
