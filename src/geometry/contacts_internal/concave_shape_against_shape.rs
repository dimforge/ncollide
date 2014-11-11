use na::{Translate, Cross, Rotation};
use na;
use shape::{Shape, ConcaveShape};
use partitioning::BoundingVolumeInterferencesCollector;
use bounding_volume::BoundingVolume;
use geometry::Contact;
use geometry::contacts_internal;
use math::{Scalar, Point, Vect, Isometry};

/// Contacts between a concave shape (`Mesh`, `Compound`) and any other shape.
pub fn manifold_concave_shape_against_shape<N, P, V, AV, M, G1, G2>(
                                            m1: &M, g1: &G1,
                                            m2: &M, g2: &G2,
                                            prediction: N,
                                            contacts: &mut Vec<Contact<N, P, V>>)
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + Cross<AV>,
          AV: Vect<N>,
          M:  Isometry<N, P, V> + Rotation<AV>,
          G1: ConcaveShape<N, P, V, M>,
          G2: Shape<N, P, V, M> {
    // Find new collisions
    let ls_m2    = na::inv(m1).expect("The transformation `m1` must be inversible.") * *m2;
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

/// Contacts between a shape and a concave (`Mesh`, `Compound`) shape.
pub fn manifold_shape_against_concave_shape<N, P, V, AV, M, G1, G2>(
                                            m1: &M, g1: &G1,
                                            m2: &M, g2: &G2,
                                            prediction: N,
                                            contacts: &mut Vec<Contact<N, P, V>>)
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + Cross<AV>,
          AV: Vect<N>,
          M:  Isometry<N, P, V> + Rotation<AV>,
          G1: Shape<N, P, V, M>,
          G2: ConcaveShape<N, P, V, M> {
    let curr_len = contacts.len();

    manifold_concave_shape_against_shape(m2, g2, m1, g1, prediction, contacts);

    for c in contacts.slice_from_mut(curr_len).iter_mut() {
        c.flip();
    }
}

/// Best contact between a concave shape (`Mesh`, `Compound`) and any other shape.
pub fn concave_shape_against_shape<N, P, V, AV, M, G1, G2>(
                                   m1: &M, g1: &G1,
                                   m2: &M, g2: &G2,
                                   prediction: N)
                                   -> Option<Contact<N, P, V>>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + Cross<AV>,
          AV: Vect<N>,
          M:  Isometry<N, P, V> + Rotation<AV>,
          G1: ConcaveShape<N, P, V, M>,
          G2: Shape<N, P, V, M> {
    // Find new collisions
    let ls_m2    = na::inv(m1).expect("The transformation `m1` must be inversible.") * *m2;
    let ls_aabb2 = g2.aabb(&ls_m2).loosened(prediction);
    let g2       = g2 as &Shape<N, P, V, M>;

    let mut interferences = Vec::new();

    {
        let mut visitor = BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut interferences);
        g1.bvt().visit(&mut visitor);
    }

    let mut res = None::<Contact<N, P, V>>;

    for i in interferences.into_iter() {
        g1.map_part_at(i, |_, part| {
            match contacts_internal::shape_against_shape(m1, part, m2, g2, prediction) {
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

/// Best contact between a shape and a concave (`Mesh`, `Compound`) shape.
pub fn shape_against_concave_shape<N, P, V, AV, M, G1, G2>(
                                   m1: &M, g1: &G1,
                                   m2: &M, g2: &G2,
                                   prediction: N)
                                   -> Option<Contact<N, P, V>>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + Cross<AV>,
          AV: Vect<N>,
          M:  Isometry<N, P, V> + Rotation<AV>,
          G1: Shape<N, P, V, M>,
          G2: ConcaveShape<N, P, V, M> {
    let mut res = concave_shape_against_shape(m2, g2, m1, g1, prediction);

    for c in res.iter_mut() {
        c.flip()
    }

    res
}
