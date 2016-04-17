use na;
use na::{Translate, Translation};
use math::{Point, Vector, Isometry};
use entities::inspection;
use entities::inspection::Repr;
use entities::shape::{Ball, Plane};
use entities::bounding_volume::{HasBoundingVolume, AABB};
use geometry::Proximity;
use geometry::proximity_internal;

/// Tests whether two shapes are in intersecting or separated by a distance smaller than `margin`.
pub fn any_against_any<P, M, G1: ?Sized, G2: ?Sized>(m1: &M, g1: &G1,
                                                     m2: &M, g2: &G2,
                                                     margin: <P::Vect as Vector>::Scalar)
                                                     -> Proximity
    where P:  Point,
          P::Vect: Translate<P>,
          M:  Isometry<P, P::Vect> + Translation<P::Vect>,
          G1: Repr<P, M> + HasBoundingVolume<M, AABB<P>>,
          G2: Repr<P, M> + HasBoundingVolume<M, AABB<P>> {
    let r1 = g1.repr();
    let r2 = g2.repr();

    if let (Some(b1), Some(b2)) = (r1.downcast_ref::<Ball<<P::Vect as Vector>::Scalar>>(),
                                   r2.downcast_ref::<Ball<<P::Vect as Vector>::Scalar>>()) {
        let p1 = m1.translate(&na::origin());
        let p2 = m2.translate(&na::origin());

        proximity_internal::ball_against_ball(&p1, b1, &p2, b2, margin)
    }
    else if let (Some(p1), Some(s2)) =
            (r1.downcast_ref::<Plane<P::Vect>>(), inspection::maybe_repr_desc_as_support_map::<P, M>(r2)) {
        proximity_internal::plane_against_support_map(m1, p1, m2, s2, margin)
    }
    else if let (Some(s1), Some(p2)) =
            (inspection::maybe_repr_desc_as_support_map::<P, M>(r1), r2.downcast_ref::<Plane<P::Vect>>()) {
        proximity_internal::support_map_against_plane(m1, s1, m2, p2, margin)
    }
    else if let (Some(s1), Some(s2)) =
            (inspection::maybe_repr_desc_as_support_map::<P, M>(r1), inspection::maybe_repr_desc_as_support_map::<P, M>(r2)) {
        proximity_internal::support_map_against_support_map::<P, _, _, _>(m1, s1, m2, s2, margin)
    }
    else if let Some(c1) = inspection::maybe_repr_desc_as_composite_shape::<P, M>(r1) {
        proximity_internal::composite_shape_against_any(m1, c1, m2, g2, margin)
    }
    else if let Some(c2) = inspection::maybe_repr_desc_as_composite_shape::<P, M>(r2) {
        proximity_internal::any_against_composite_shape(m1, g1, m2, c2, margin)
    }
    else {
        panic!("No algorithm known to compute proximity between the given pair of shapes.")
    }
}
