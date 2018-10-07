use bounding_volume::{AABB, BoundingVolume};
use math::Isometry;
use na::{self, Real, Unit};
use pipeline::narrow_phase::{ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator};
use query::{Contact, ContactManifold, ContactPrediction, visitors::AABBSetsInterferencesCollector};
use query::closest_points_internal;
use shape::{CompositeShape, Segment, Shape, Triangle};
use std::collections::{hash_map::Entry, HashMap};
use utils::DeterministicState;
use utils::IdAllocator;


/// Collision detector between a concave shape and another shape.
pub struct TriMeshTriMeshManifoldGenerator<N: Real> {
    manifold: ContactManifold<N>,
    interferences: Vec<(usize, usize)>,
}

impl<N: Real> TriMeshTriMeshManifoldGenerator<N> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new() -> TriMeshTriMeshManifoldGenerator<N> {
        TriMeshTriMeshManifoldGenerator {
            manifold: ContactManifold::new(),
            interferences: Vec::new(),
        }
    }
}


impl<N: Real> TriMeshTriMeshManifoldGenerator<N> {
    fn faces_closest_points(t1: &Triangle<N>, t2: &Triangle<N>, prediction: &ContactPrediction<N>) {
        if let (Some(n1), Some(n2)) = (t1.normal(), t2.normal()) {
            /*
             * Start with the SAT.
             */
            let mut sep_axis = None;

            #[inline(always)]
            fn intersection<N: Real>(a: (N, N), b: (N, N)) -> Option<N> {
                let min = a.0.max(b.0);
                let max = a.1.min(b.1);

                if min <= max {
                    Some(max - min)
                } else {
                    None
                }
            }

            // This loop is a trick to be able to easily stop the search for a separating axis as
            // as we find one using `break 'search` (without having to do all this on a separate function
            // and do a return instead of breaks).
            'search: loop {
                // First, test normals.
                let interval1 = t1.a().coords.dot(&n1);
                let interval2 = t2.extents_on_dir(&n1);

                if let Some(overlap) = intersection((interval1, interval1), interval2) {
                    unimplemented!()
                } else {
                    // The triangles are disjoint.
                    sep_axis = Some(n1);
                    break;
                }

                let interval2 = t2.a().coords.dot(&n2);
                let interval1 = t1.extents_on_dir(&n2);
                if let Some(overlap) = intersection(interval1, (interval2, interval2)) {
                    unimplemented!()
                } else {
                    // The triangles are disjoint.
                    sep_axis = Some(n2);
                    break;
                }

                let edge_dirs_a = t1.edges_scaled_directions();
                let edge_dirs_b = t2.edges_scaled_directions();

                // Second, test edges cross products.
                for e1 in &edge_dirs_a {
                    for e2 in &edge_dirs_b {
                        if let Some(dir) = Unit::try_new(e1.cross(e2), N::default_epsilon()) {
                            let interval1 = t1.extents_on_dir(&dir);
                            let interval2 = t2.extents_on_dir(&dir);

                            if let Some(overlap) = intersection(interval1, interval2) {
                                unimplemented!()
                            } else {
                                // Triangles are disjoint.
                                sep_axis = Some(dir);
                                break 'search;
                            }
                        }
                    }
                }

                // If we reached this point, no separating axis was found.

                break;
            }

            /*
                        let sides1 = [
                            edges_a[0].scaled_direction().cross(&n1),
                            edges_a[1].scaled_direction().cross(&n1),
                            edges_a[2].scaled_direction().cross(&n1),
                        ];

                        let sides2 = [
                            edges_b[0].scaled_direction().cross(&n2),
                            edges_b[1].scaled_direction().cross(&n2),
                            edges_b[2].scaled_direction().cross(&n2),
                        ];

                        for e1 in &edges_a {
                            for e2 in &edges_b {
                                let dir1 = e1.scaled_direction();
                                let dir2 = e2.scaled_direction();

                                match closest_points_internal::segment_against_segment_with_locations_nD((e1.a(), e1.b()), (e2.a(), e2.b())) {
                                    _ => {
                                        unimplemented!()
                                        /*
                                    let world1 = e1.a() + dir1 * s;
                                    let world2 = e2.a() + dir2 * t;
                                    out.push(Contact::new(world1, world2));
                                    */
                                    }
                                }
                            }
                        }

                        'vloop1: for v in t1.vertices() {
                            let dpt = v - t2.a();

                            for side2 in &sides2 {
                                if dpt.dot(side2) >= N::zero() {
                                    continue 'vloop1;
                                }
                            }

                            let dist = dpt.dot(&n2);
                            let proj = v + *n2 * -dist;
            //                out.push(Contact::new(*v, proj, n2, dist));
                        }

                        'vloop2: for v in t2.vertices() {
                            let dpt = v - t2.b();

                            for side1 in &sides1 {
                                if dpt.dot(side1) >= N::zero() {
                                    continue 'vloop2;
                                }
                            }

                            let dist = dpt.dot(&n1);
                            let proj = v + *n1 * -dist;
            //                out.push(Contact::new(proj, *v, n1, dist));
                        }*/
        }
    }

    fn do_update(
        &mut self,
        _: &ContactDispatcher<N>,
        id1: usize,
        m1: &Isometry<N>,
        g1: &CompositeShape<N>,
        id2: usize,
        m2: &Isometry<N>,
        g2: &CompositeShape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
    ) {
        self.manifold.save_cache_and_clear(id_alloc);

        // Find new collisions
        let ls_m2 = m1.inverse() * m2;
        // For transforming AABBs from g2 in the local space of g1.
        let ls_m2_abs_rot = ls_m2.rotation.to_rotation_matrix().matrix().abs();

        {
            let mut visitor =
                AABBSetsInterferencesCollector::new(&ls_m2, &ls_m2_abs_rot, &mut self.interferences);
            g1.bvh().visit_bvtt(g2.bvh(), &mut visitor);
        }

        for id in self.interferences.drain(..) {}
    }
}

impl<N: Real> ContactManifoldGenerator<N>
for TriMeshTriMeshManifoldGenerator<N> {
    fn update(
        &mut self,
        d: &ContactDispatcher<N>,
        ida: usize,
        ma: &Isometry<N>,
        a: &Shape<N>,
        idb: usize,
        mb: &Isometry<N>,
        b: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
    ) -> bool {
        if let (Some(csa), Some(csb)) = (a.as_composite_shape(), b.as_composite_shape()) {
            self.do_update(d, ida, ma, csa, idb, mb, csb, prediction, id_alloc);
            true
        } else {
            false
        }
    }

    fn num_contacts(&self) -> usize {
        self.manifold.len()
    }

    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<N>>) {
        out.push(&self.manifold)
    }
}
