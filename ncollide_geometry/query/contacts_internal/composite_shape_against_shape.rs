use na;
use partitioning::BoundingVolumeInterferencesCollector;
use bounding_volume::BoundingVolume;
use shape::{Shape, CompositeShape};
use query::Contact;
use query::contacts_internal;
use math::{Point, Isometry};

/*
/// Contacts between a composite shape (`Mesh`, `Compound`) and any other shape.
pub fn manifold_composite_shape_against_shape<N, P, V, AV, M, G1, G2>(
                                            m1: &M, g1: &G1,
                                            m2: &M, g2: &G2,
                                            prediction: N,
                                            contacts: &mut Vec<Contact<N, P, V>>)
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vector<N> + Translate<P> + Cross<AV>,
          AV: Vector<N>,
          M:  Isometry<N, P, V> + Rotation<AV>,
          G1: CompositeShape<N, P, V, M>,
          G2: Shape<N, P, V, M> {
    // Find new collisions
    let ls_m2    = na::inverse(m1) * *m2;
    let ls_aabb2 = g2.aabb(&ls_m2).loosened(prediction);
    let g2       = g2 as &Shape<N, P, V, M>;

    let mut interferences = Vec::new();

    {
        let mut visitor = BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut interferences);
        g1.bvt().visit(&mut visitor);
    }

    for i in interferences.into_iter() {
        g1.map_part_at(i, |_, part| {
            contacts_internal::manifold_shape_against_shape(m1, part, m2, g2, prediction, contacts)
        });
    }
}

/// Contacts between a shape and a composite (`Mesh`, `Compound`) shape.
pub fn manifold_shape_against_composite_shape<N, P, V, AV, M, G1, G2>(
                                            m1: &M, g1: &G1,
                                            m2: &M, g2: &G2,
                                            prediction: N,
                                            contacts: &mut Vec<Contact<N, P, V>>)
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vector<N> + Translate<P> + Cross<AV>,
          AV: Vector<N>,
          M:  Isometry<N, P, V> + Rotation<AV>,
          G1: Shape<N, P, V, M>,
          G2: CompositeShape<N, P, V, M> {
    let curr_len = contacts.len();

    manifold_composite_shape_against_shape(m2, g2, m1, g1, prediction, contacts);

    for c in contacts.slice_from_mut(curr_len).iter_mut() {
        c.flip();
    }
}
*/

/// Best contact between a composite shape (`Mesh`, `Compound`) and any other shape.
pub fn composite_shape_against_shape<P, M, G1: ?Sized>(m1: &M, g1: &G1,
                                                       m2: &M, g2: &Shape<P, M>,
                                                       prediction: P::Real)
                                                       -> Option<Contact<P>>
    where P:  Point,
          M:  Isometry<P>,
          G1: CompositeShape<P, M> {
    // Find new collisions
    let ls_m2    = na::inverse(m1) * m2.clone();
    let ls_aabb2 = g2.aabb(&ls_m2).loosened(prediction);

    let mut interferences = Vec::new();

    {
        let mut visitor = BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut interferences);
        g1.bvt().visit(&mut visitor);
    }

    let mut res = None::<Contact<P>>;

    for i in interferences.into_iter() {
        g1.map_part_at(i, &mut |_, part| {
            match contacts_internal::contact_internal(m1, part, m2, g2, prediction) {
                Some(c) => {
                    let replace = match res {
                        Some(ref cbest) => {
                            c.depth > cbest.depth
                        },
                        None => true
                    };

                    if replace {
                        res = Some(c)
                    }
                },
                None => { }
            }
        });
    }

    res
}

/// Best contact between a shape and a composite (`Mesh`, `Compound`) shape.
pub fn shape_against_composite_shape<P, M, G2: ?Sized>(m1: &M, g1: &Shape<P, M>,
                                                       m2: &M, g2: &G2,
                                                       prediction: P::Real)
                                                       -> Option<Contact<P>>
    where P:  Point,
          M:  Isometry<P>,
          G2: CompositeShape<P, M> {
    let mut res = composite_shape_against_shape(m2, g2, m1, g1, prediction);

    for c in res.iter_mut() {
        c.flip()
    }

    res
}
