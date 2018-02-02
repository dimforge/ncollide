use alga::linear::Translation;
use na::{self, Unit};
use query::Contact;
use shape::{Plane, SupportMap};
use math::{Isometry, Point};

/// Contact between a plane and a support-mapped shape (Cuboid, ConvexHull, etc.)
pub fn plane_against_support_map<P, M, G: ?Sized>(
    mplane: &M,
    plane: &Plane<P::Vector>,
    mother: &M,
    other: &G,
    prediction: P::Real,
) -> Option<Contact<P>>
where
    P: Point,
    M: Isometry<P>,
    G: SupportMap<P, M>,
{
    let plane_normal = mplane.rotate_vector(&*plane.normal());
    let plane_center = P::from_coordinates(mplane.translation().to_vector());
    let deepest = other.support_point(mother, &-plane_normal);

    let distance = na::dot(&plane_normal, &(plane_center - deepest));

    if distance > -prediction {
        let c1 = deepest + plane_normal * distance;

        Some(Contact::new(c1, deepest, Unit::new_unchecked(plane_normal), distance))
    } else {
        None
    }
}

/// Contact between a support-mapped shape (Cuboid, ConvexHull, etc.) and a plane.
pub fn support_map_against_plane<P, M, G: ?Sized>(
    mother: &M,
    other: &G,
    mplane: &M,
    plane: &Plane<P::Vector>,
    prediction: P::Real,
) -> Option<Contact<P>>
where
    P: Point,
    M: Isometry<P>,
    G: SupportMap<P, M>,
{
    plane_against_support_map(mplane, plane, mother, other, prediction).map(|mut c| {
        c.flip();
        c
    })
}
