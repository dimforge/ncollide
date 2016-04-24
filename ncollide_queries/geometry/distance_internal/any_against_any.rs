use na;
use na::{Translate, Translation};
use math::{Point, Vector, Isometry};
use entities::inspection::Shape;
use entities::shape::{Ball, Plane};
use entities::bounding_volume::{HasBoundingVolume, AABB};
use geometry::distance_internal;

/// Computes the minimum distance separating two shapes.
///
/// Returns `0.0` if the objects are touching or penetrating.
pub fn any_against_any<P, M, G1: ?Sized, G2: ?Sized>(m1: &M, g1: &G1, m2: &M, g2: &G2) -> <P::Vect as Vector>::Scalar
    where P:  Point,
          P::Vect: Translate<P>,
          M:  Isometry<P, P::Vect> + Translation<P::Vect>,
          G1: Shape<P, M> + HasBoundingVolume<M, AABB<P>>,
          G2: Shape<P, M> + HasBoundingVolume<M, AABB<P>> {
    let r1 = g1.desc();
    let r2 = g2.desc();

    if let (Some(b1), Some(b2)) = (r1.as_shape::<Ball<<P::Vect as Vector>::Scalar>>(),
                                   r2.as_shape::<Ball<<P::Vect as Vector>::Scalar>>()) {
        let p1 = m1.translate(&na::origin());
        let p2 = m2.translate(&na::origin());

        distance_internal::ball_against_ball(&p1, b1, &p2, b2)
    }
    else if let (Some(p1), Some(s2)) = (r1.as_shape::<Plane<P::Vect>>(), r2.as_support_map()) {
        distance_internal::plane_against_support_map(m1, p1, m2, s2)
    }
    else if let (Some(s1), Some(p2)) = (r1.as_support_map(), r2.as_shape::<Plane<P::Vect>>()) {
        distance_internal::support_map_against_plane(m1, s1, m2, p2)
    }
    else if let (Some(s1), Some(s2)) = (r1.as_support_map(), r2.as_support_map()) {
        distance_internal::support_map_against_support_map::<P, _, _, _>(m1, s1, m2, s2)
    }
    else if let Some(c1) = r1.as_composite_shape() {
        distance_internal::composite_shape_against_any(m1, c1, m2, g2)
    }
    else if let Some(c2) = r2.as_composite_shape() {
        distance_internal::any_against_composite_shape(m1, g1, m2, c2)
    }
    else {
        panic!("No algorithm known to compute a contact point between the given pair of shapes.")
    }
}
