use crate::bounding_volume::BoundingVolume;
use crate::math::Isometry;
use na::{self, RealField};
use crate::query::contacts_internal;
use crate::query::visitors::BoundingVolumeInterferencesCollector;
use crate::query::Contact;
use crate::shape::{CompositeShape, Shape};

/// Best contact between a composite shape (`Mesh`, `Compound`) and any other shape.
pub fn composite_shape_against_shape<N: RealField, G1: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &Shape<N>,
    prediction: N,
) -> Option<Contact<N>>
where
    G1: CompositeShape<N>,
{
    // Find new collisions
    let ls_m2 = m1.inverse() * m2.clone();
    let ls_aabb2 = g2.aabb(&ls_m2).loosened(prediction);

    let mut interferences = Vec::new();

    {
        let mut visitor = BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut interferences);
        g1.bvh().visit(&mut visitor);
    }

    let mut res = None::<Contact<N>>;

    for i in interferences.into_iter() {
        g1.map_part_at(i, m1, &mut |m, part| {
            match contacts_internal::contact_internal(m, part, m2, g2, prediction) {
                Some(c) => {
                    let replace = match res {
                        Some(ref cbest) => c.depth > cbest.depth,
                        None => true,
                    };

                    if replace {
                        res = Some(c)
                    }
                }
                None => {}
            }
        });
    }

    res
}

/// Best contact between a shape and a composite (`Mesh`, `Compound`) shape.
pub fn shape_against_composite_shape<N: RealField, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    g2: &G2,
    prediction: N,
) -> Option<Contact<N>>
where
    G2: CompositeShape<N>,
{
    let mut res = composite_shape_against_shape(m2, g2, m1, g1, prediction);

    for c in res.iter_mut() {
        c.flip()
    }

    res
}
