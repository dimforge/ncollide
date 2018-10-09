use bounding_volume::{self, BoundingVolume};
use math::Isometry;
use na::{self, Real};
use pipeline::narrow_phase::{ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator};
use query::{visitors::BoundingVolumeInterferencesCollector, ContactManifold, ContactPrediction};
use shape::{CompositeShape, Shape};
use std::collections::{hash_map::Entry, HashMap};
use utils::DeterministicState;
use utils::IdAllocator;

/// Collision detector between a concave shape and another shape.
pub struct CompositeShapeShapeManifoldGenerator<N: Real> {
    sub_detectors: HashMap<usize, (ContactAlgorithm<N>, ContactManifold<N>), DeterministicState>,
    interferences: Vec<usize>,
    flip: bool,
}

impl<N: Real> CompositeShapeShapeManifoldGenerator<N> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new(flip: bool) -> CompositeShapeShapeManifoldGenerator<N> {
        CompositeShapeShapeManifoldGenerator {
            sub_detectors: HashMap::with_hasher(DeterministicState),
            interferences: Vec::new(),
            flip,
        }
    }

    fn do_update(
        &mut self,
        dispatcher: &ContactDispatcher<N>,
        id1: usize,
        m1: &Isometry<N>,
        g1: &CompositeShape<N>,
        id2: usize,
        m2: &Isometry<N>,
        g2: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        flip: bool,
    ) {
        // Find new collisions
        let ls_m2 = na::inverse(m1) * m2.clone();
        let ls_aabb2 = bounding_volume::aabb(g2, &ls_m2).loosened(prediction.linear());

        {
            let mut visitor =
                BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut self.interferences);
            g1.bvh().visit(&mut visitor);
        }

        for i in self.interferences.drain(..) {
            match self.sub_detectors.entry(i) {
                Entry::Occupied(_) => {}
                Entry::Vacant(entry) => {
                    let mut new_detector = None;

                    g1.map_part_at(i, &mut |_, _, g1| {
                        if flip {
                            new_detector = dispatcher.get_contact_algorithm(g2, g1)
                        } else {
                            new_detector = dispatcher.get_contact_algorithm(g1, g2)
                        }
                    });

                    if let Some(new_detector) = new_detector {
                        let _ = entry.insert((new_detector, ContactManifold::new()));
                    }
                }
            }
        }

        // Update all collisions
        self.sub_detectors.retain(|key, detector| {
            if ls_aabb2.intersects(&g1.aabb_at(*key)) {
                g1.map_transformed_part_at(*key, m1, &mut |sub_id1, m1, g1| {
                    detector.1.save_cache_and_clear(id_alloc);

                    if flip {
                        assert!(
                            detector.0.update_to(
                                dispatcher,
                                id2,
                                m2,
                                g2,
                                id1 + sub_id1,
                                m1,
                                g1,
                                prediction,
                                id_alloc,
                                &mut detector.1
                            ),
                            "Internal error: the shape was no longer valid."
                        );
                    } else {
                        assert!(
                            detector.0.update_to(
                                dispatcher,
                                id1 + sub_id1,
                                m1,
                                g1,
                                id2,
                                m2,
                                g2,
                                prediction,
                                id_alloc,
                                &mut detector.1
                            ),
                            "Internal error: the shape was no longer valid."
                        );
                    }
                });

                true
            } else {
                // FIXME: ask the detector if it wants to be removed or not
                false
            }
        });
    }
}

impl<N: Real> ContactManifoldGenerator<N> for CompositeShapeShapeManifoldGenerator<N> {
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
        if !self.flip {
            if let Some(cs) = a.as_composite_shape() {
                self.do_update(d, ida, ma, cs, idb, mb, b, prediction, id_alloc, false);
                return true;
            }
        } else {
            if let Some(cs) = b.as_composite_shape() {
                self.do_update(d, idb, mb, cs, ida, ma, a, prediction, id_alloc, true);
                return true;
            }
        }

        return false;
    }

    fn update_to(
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
        manifold: &mut ContactManifold<N>,
    ) -> bool {
        unimplemented!()
    }

    fn num_contacts(&self) -> usize {
        let mut res = 0;

        for detector in &self.sub_detectors {
            res = res + (detector.1).1.len()
        }

        res
    }

    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<N>>) {
        for detector in &self.sub_detectors {
            out.push(&(detector.1).1);
        }
    }
}
