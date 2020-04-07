use crate::bounding_volume::BoundingVolume;
use crate::math::Isometry;
use crate::partitioning::VisitStatus;
use crate::query::visitors::BoundingVolumeInterferencesVisitor;
use crate::query::{self, Contact};
use crate::shape::{CompositeShape, Shape};
use na::{self, RealField};

/// Best contact between a composite shape (`Mesh`, `Compound`) and any other shape.
pub fn contact_composite_shape_shape<N: RealField, G1: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &dyn Shape<N>,
    prediction: N,
) -> Option<Contact<N>>
where
    G1: CompositeShape<N>,
{
    // Find new collisions
    let ls_m2 = m1.inverse() * m2.clone();
    let ls_aabb2 = g2.aabb(&ls_m2).loosened(prediction);

    let mut res = None::<Contact<N>>;

    let mut visitor = BoundingVolumeInterferencesVisitor::new(&ls_aabb2, |&i| {
        g1.map_part_at(i, m1, &mut |m, part| {
            if let Some(c) = query::contact(m, part, m2, g2, prediction) {
                let replace = res.map_or(true, |cbest| c.depth > cbest.depth);

                if replace {
                    res = Some(c)
                }
            }
        });

        VisitStatus::Continue
    });
    g1.bvh().visit(&mut visitor);

    res
}

/// Best contact between a shape and a composite (`Mesh`, `Compound`) shape.
pub fn contact_shape_composite_shape<N: RealField, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &dyn Shape<N>,
    m2: &Isometry<N>,
    g2: &G2,
    prediction: N,
) -> Option<Contact<N>>
where
    G2: CompositeShape<N>,
{
    let mut res = contact_composite_shape_shape(m2, g2, m1, g1, prediction);

    for c in res.iter_mut() {
        c.flip()
    }

    res
}
