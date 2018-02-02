use na;
use math::{Isometry, Point};
use utils::data::hash_map::HashMap;
use utils::data::hash::UintTWHash;
use geometry::bounding_volume::{self, BoundingVolume};
use geometry::partitioning::BoundingVolumeInterferencesCollector;
use geometry::shape::{CompositeShape, Shape};
use geometry::query::Contact;
use narrow_phase::{ContactAlgorithm, ContactDispatcher, ContactGenerator};

/// Collision detector between a concave shape and another shape.
pub struct CompositeShapeShapeContactGenerator<P: Point, M> {
    sub_detectors: HashMap<usize, ContactAlgorithm<P, M>, UintTWHash>,
    to_delete: Vec<usize>,
    interferences: Vec<usize>,
}

impl<P: Point, M> CompositeShapeShapeContactGenerator<P, M> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new() -> CompositeShapeShapeContactGenerator<P, M> {
        CompositeShapeShapeContactGenerator {
            sub_detectors: HashMap::new_with_capacity(5, UintTWHash::new()),
            to_delete: Vec::new(),
            interferences: Vec::new(),
        }
    }
}

impl<P: Point, M: Isometry<P>> CompositeShapeShapeContactGenerator<P, M> {
    fn do_update(
        &mut self,
        dispatcher: &ContactDispatcher<P, M>,
        m1: &M,
        g1: &CompositeShape<P, M>,
        m2: &M,
        g2: &Shape<P, M>,
        prediction: P::Real,
        swap: bool,
    ) {
        // Find new collisions
        let ls_m2 = na::inverse(m1) * m2.clone();
        let ls_aabb2 = bounding_volume::aabb(g2, &ls_m2).loosened(prediction);

        {
            let mut visitor =
                BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut self.interferences);
            g1.bvt().visit(&mut visitor);
        }

        for i in self.interferences.iter() {
            let _ = self.sub_detectors.find_or_insert_lazy(*i, || {
                let mut new_detector = None;

                g1.map_part_at(*i, &mut |_, g1| {
                    if swap {
                        new_detector = dispatcher.get_contact_algorithm(g2, g1)
                    } else {
                        new_detector = dispatcher.get_contact_algorithm(g1, g2)
                    }
                });

                new_detector
            });
        }

        self.interferences.clear();

        // Update all collisions
        for detector in self.sub_detectors.elements_mut().iter_mut() {
            let key = detector.key;
            if ls_aabb2.intersects(&g1.aabb_at(key)) {
                g1.map_transformed_part_at(key, m1, &mut |m1, g1| {
                    if swap {
                        assert!(
                            detector
                                .value
                                .update(dispatcher, m2, g2, m1, g1, prediction),
                            "Internal error: the shape was no longer valid."
                        );
                    } else {
                        assert!(
                            detector
                                .value
                                .update(dispatcher, m1, g1, m2, g2, prediction),
                            "Internal error: the shape was no longer valid."
                        );
                    }
                });
            } else {
                // FIXME: ask the detector if it wants to be removed or not
                self.to_delete.push(key);
            }
        }

        // Remove outdated sub detectors
        for i in self.to_delete.iter() {
            let _ = self.sub_detectors.remove(i);
        }

        self.to_delete.clear();
    }
}

/// Collision detector between a shape and a concave shape.
pub struct ShapeCompositeShapeContactGenerator<P: Point, M> {
    sub_detector: CompositeShapeShapeContactGenerator<P, M>,
}

impl<P: Point, M> ShapeCompositeShapeContactGenerator<P, M> {
    /// Creates a new collision detector between a shape and a concave shape.
    pub fn new() -> ShapeCompositeShapeContactGenerator<P, M> {
        ShapeCompositeShapeContactGenerator {
            sub_detector: CompositeShapeShapeContactGenerator::new(),
        }
    }
}

impl<P: Point, M: Isometry<P>> ContactGenerator<P, M>
    for CompositeShapeShapeContactGenerator<P, M> {
    fn update(
        &mut self,
        d: &ContactDispatcher<P, M>,
        ma: &M,
        a: &Shape<P, M>,
        mb: &M,
        b: &Shape<P, M>,
        prediction: P::Real,
    ) -> bool {
        if let Some(cs) = a.as_composite_shape() {
            self.do_update(d, ma, cs, mb, b, prediction, false);

            true
        } else {
            false
        }
    }

    fn num_contacts(&self) -> usize {
        let mut res = 0;

        for detector in self.sub_detectors.elements().iter() {
            res = res + detector.value.num_contacts()
        }

        res
    }

    fn contacts(&self, out: &mut Vec<Contact<P>>) {
        for detector in self.sub_detectors.elements().iter() {
            detector.value.contacts(out);
        }
    }
}

impl<P: Point, M: Isometry<P>> ContactGenerator<P, M>
    for ShapeCompositeShapeContactGenerator<P, M> {
    fn update(
        &mut self,
        d: &ContactDispatcher<P, M>,
        ma: &M,
        a: &Shape<P, M>,
        mb: &M,
        b: &Shape<P, M>,
        prediction: P::Real,
    ) -> bool {
        if let Some(cs) = b.as_composite_shape() {
            self.sub_detector
                .do_update(d, mb, cs, ma, a, prediction, true);

            true
        } else {
            false
        }
    }

    fn num_contacts(&self) -> usize {
        self.sub_detector.num_contacts()
    }

    fn contacts(&self, out: &mut Vec<Contact<P>>) {
        self.sub_detector.contacts(out)
    }
}
