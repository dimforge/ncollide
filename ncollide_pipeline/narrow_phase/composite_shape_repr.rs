use na;
use na::Translate;
use math::{Scalar, Point, Vect, Isometry};
use utils::data::hash_map::HashMap;
use utils::data::hash::UintTWHash;
use entities::bounding_volume::{HasAABB, BoundingVolume};
use entities::partitioning::BoundingVolumeInterferencesCollector;
use entities::shape::CompositeShape;
use entities::inspection::Repr;
use entities::inspection;
use queries::geometry::Contact;
use narrow_phase::{CollisionDetector, CollisionDispatcher, CollisionAlgorithm};


/// Collision detector between a concave shape and another shape.
pub struct CompositeShapeRepr<N, P, V, M> {
    prediction:    N,
    sub_detectors: HashMap<usize, CollisionAlgorithm<N, P, V, M>, UintTWHash>,
    to_delete:     Vec<usize>,
    interferences: Vec<usize>
}

impl<N, P, V, M> CompositeShapeRepr<N, P, V, M> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new(prediction: N) -> CompositeShapeRepr<N, P, V, M> {
        CompositeShapeRepr {
            prediction:    prediction,
            sub_detectors: HashMap::new_with_capacity(5, UintTWHash::new()),
            to_delete:     Vec::new(),
            interferences: Vec::new()
        }
    }
}

impl<N, P, V, M> CompositeShapeRepr<N, P, V, M>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M:  Isometry<N, P, V> {
    fn do_update(&mut self,
                 dispatcher: &CollisionDispatcher<N, P, V, M>,
                 m1:         &M,
                 g1:         &CompositeShape<N, P, V, M>,
                 m2:         &M,
                 g2:         &Repr<N, P, V, M>,
                 swap:       bool) {
        // Find new collisions
        let ls_m2    = na::inv(m1).expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = g2.aabb(&ls_m2).loosened(self.prediction);

        {
            let mut visitor = BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut self.interferences);
            g1.bvt().visit(&mut visitor);
        }

        for i in self.interferences.iter() {
            let mut detector = None;

            g1.map_part_at(*i, &mut |_, g1| {
                let r1 = g1.repr();
                let r2 = g2.repr();

                if swap {
                    detector = dispatcher.get_collision_algorithm(&r2, &r1)
                }
                else {
                    detector = dispatcher.get_collision_algorithm(&r1, &r2)
                }
            });

            match detector {
                Some(detector) => {
                    let _ = self.sub_detectors.insert_or_replace(*i, detector, false);
                },
                None => { }
            }
        }

        self.interferences.clear();

        // Update all collisions
        for detector in self.sub_detectors.elements_mut().iter_mut() {
            let key = detector.key;
            if ls_aabb2.intersects(g1.aabb_at(key)) {
                g1.map_transformed_part_at(m1, key, &mut |m1, g1| {
                    if swap {
                        assert!(detector.value.update(dispatcher, m2, g2, m1, g1), "The shape was no longer valid.");
                    }
                    else {
                        assert!(detector.value.update(dispatcher, m1, g1, m2, g2), "The shape was nolonger valid.");
                    }
                });
            }
            else {
                // FIXME: ask the detector if it wants to be removed or not
                self.to_delete.push(key);
            }
        }

        // Remove outdated sub detectors
        for i in self.to_delete.iter() {
            self.sub_detectors.remove(i);
        }

        self.to_delete.clear();
    }
}

/// Collision detector between a shape and a concave shape.
pub struct ReprCompositeShape<N, P, V, M> {
    sub_detector: CompositeShapeRepr<N, P, V, M>
}

impl<N, P, V, M> ReprCompositeShape<N, P, V, M> {
    /// Creates a new collision detector between a shape and a concave shape.
    pub fn new(prediction: N) -> ReprCompositeShape<N, P, V, M> {
        ReprCompositeShape {
            sub_detector: CompositeShapeRepr::new(prediction)
        }
    }
}

impl<N, P, V, M> CollisionDetector<N, P, V, M> for CompositeShapeRepr<N, P, V, M>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M:  Isometry<N, P, V> {
    fn update(&mut self,
              d:  &CollisionDispatcher<N, P, V, M>,
              ma: &M,
              a:  &Repr<N, P, V, M>,
              mb: &M,
              b:  &Repr<N, P, V, M>)
              -> bool {
        if let Some(cs) = inspection::maybe_as_composite_shape(a) {
            self.do_update(d, ma, cs, mb, b, false);

            true
        }
        else {
            false
        }
    }

    fn num_colls(&self) -> usize {
        let mut res = 0;

        for detector in self.sub_detectors.elements().iter() {
            res = res + detector.value.num_colls()
        }

        res
    }

    fn colls(&self, out: &mut Vec<Contact<N, P, V>>) {
        for detector in self.sub_detectors.elements().iter() {
            detector.value.colls(out);
        }
    }
}

impl<N, P, V, M> CollisionDetector<N, P, V, M> for ReprCompositeShape<N, P, V, M>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M:  Isometry<N, P, V> {
    fn update(&mut self,
              d:  &CollisionDispatcher<N, P, V, M>,
              ma: &M,
              a:  &Repr<N, P, V, M>,
              mb: &M,
              b:  &Repr<N, P, V, M>)
              -> bool {
        if let Some(cs) = inspection::maybe_as_composite_shape(b) {
            self.sub_detector.do_update(d, mb, cs, ma, a, true);

            true
        }
        else {
            false
        }
    }

    fn num_colls(&self) -> usize {
        self.sub_detector.num_colls()
    }

    fn colls(&self, out: &mut Vec<Contact<N, P, V>>) {
        self.sub_detector.colls(out)
    }
}
