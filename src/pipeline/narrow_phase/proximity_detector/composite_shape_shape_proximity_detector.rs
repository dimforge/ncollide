use crate::bounding_volume::{self, BoundingVolume};
use crate::math::{Isometry, Vector};
use na::{self, Real};
use crate::pipeline::narrow_phase::{ProximityAlgorithm, ProximityDetector, ProximityDispatcher};
use crate::query::{visitors::BoundingVolumeInterferencesCollector, Proximity, Ray, RayCast};
use crate::shape::{CompositeShape, Shape, Polyline, FeatureId};
#[cfg(feature = "dim3")]
use crate::shape::TriMesh;
use std::collections::{hash_map::Entry, HashMap};
use crate::utils::DeterministicState;

/// Proximity detector between a concave shape and another shape.
pub struct CompositeShapeShapeProximityDetector<N> {
    proximity: Proximity,
    sub_detectors: HashMap<usize, ProximityAlgorithm<N>, DeterministicState>,
    to_delete: Vec<usize>,
    interferences: Vec<usize>,
    intersecting_key: usize,
    flip: bool,
}

impl<N> CompositeShapeShapeProximityDetector<N> {
    /// Creates a new proximity detector between a concave shape and another shape.
    pub fn new(flip: bool) -> CompositeShapeShapeProximityDetector<N> {
        CompositeShapeShapeProximityDetector {
            proximity: Proximity::Disjoint,
            sub_detectors: HashMap::with_hasher(DeterministicState),
            to_delete: Vec::new(),
            interferences: Vec::new(),
            intersecting_key: usize::max_value(),
            flip,
        }
    }
}

impl<N: Real> CompositeShapeShapeProximityDetector<N> {
    fn do_update(
        &mut self,
        dispatcher: &ProximityDispatcher<N>,
        m1: &Isometry<N>,
        g1: &CompositeShape<N>,
        m2: &Isometry<N>,
        g2: &Shape<N>,
        margin: N,
        flip: bool,
    )
    {
        // Remove outdated sub detectors.
        for key in self.to_delete.iter() {
            let _ = self.sub_detectors.remove(key);
        }

        self.to_delete.clear();
        self.interferences.clear();

        // First, test if the previously intersecting shapes are still intersecting.
        if self.proximity == Proximity::Intersecting {
            if let Some(detector) = self.sub_detectors.get_mut(&self.intersecting_key) {
                g1.map_part_at(self.intersecting_key, m1, &mut |m1, g1| {
                    assert!(
                        detector.update(dispatcher, m1, g1, m2, g2, margin),
                        "The shape was no longer valid."
                    );
                });

                match detector.proximity() {
                    Proximity::Intersecting => return, // Early return.
                    Proximity::WithinMargin => self.proximity = Proximity::WithinMargin,
                    Proximity::Disjoint => {}
                }
            }
        }

        self.proximity = Proximity::Disjoint;

        let m12 = na::inverse(m1) * m2.clone();
        let ls_aabb2 = bounding_volume::aabb(g2, &m12).loosened(margin);

        // Update all collisions
        for detector in &mut self.sub_detectors {
            let key = *detector.0;

            if key == self.intersecting_key {
                // We already dealt with that one.
                continue;
            }

            if ls_aabb2.intersects(&g1.aabb_at(key)) {
                g1.map_part_at(key, m1, &mut |m1, g1| {
                    assert!(
                        detector.1.update(dispatcher, m1, g1, m2, g2, margin),
                        "The shape was no longer valid."
                    );
                });

                match detector.1.proximity() {
                    Proximity::Intersecting => {
                        self.proximity = Proximity::Intersecting;
                        self.intersecting_key = *detector.0;
                        return; // No need to search any further.
                    }
                    Proximity::WithinMargin => self.proximity = Proximity::WithinMargin,
                    Proximity::Disjoint => {}
                }
            } else {
                // FIXME: ask the detector if it wants to be removed or not
                self.to_delete.push(key);
            }
        }

        // Find new proximities.
        {
            let mut visitor =
                BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut self.interferences);
            g1.bvh().visit(&mut visitor);
        }

        for key in &self.interferences {
            let entry = self.sub_detectors.entry(*key);
            let detector = match entry {
                Entry::Occupied(entry) => Some(entry.into_mut()),
                Entry::Vacant(entry) => {
                    let mut new_detector = None;

                    g1.map_part_at(*key, &Isometry::identity(), &mut |_, g1| {
                        if flip {
                            new_detector = dispatcher.get_proximity_algorithm(g2, g1)
                        } else {
                            new_detector = dispatcher.get_proximity_algorithm(g1, g2)
                        }
                    });

                    if let Some(new_detector) = new_detector {
                        Some(entry.insert(new_detector))
                    } else {
                        None
                    }
                }
            };

            if let Some(sub_detector) = detector {
                g1.map_part_at(*key, m1, &mut |m1, g1| {
                    if flip {
                        let _ = sub_detector.update(dispatcher, m2, g2, m1, g1, margin);
                    } else {
                        let _ = sub_detector.update(dispatcher, m1, g1, m2, g2, margin);
                    }
                });

                match sub_detector.proximity() {
                    Proximity::Intersecting => {
                        self.proximity = Proximity::Intersecting;
                        self.intersecting_key = *key;
                        return; // No need to search further.
                    }
                    Proximity::WithinMargin => self.proximity = Proximity::WithinMargin,
                    Proximity::Disjoint => {}
                }
            }
        }

        // Totally disjoints.
        self.intersecting_key = usize::max_value()
    }


    fn handle_interior(
        &mut self,
        m1: &Isometry<N>,
        g1: &Shape<N>,
        m2: &Isometry<N>,
        g2: &Shape<N>,
    )
    {
        #[cfg(feature = "dim3")]
            let mesh1 = g1.as_shape::<TriMesh<N>>();
        #[cfg(feature = "dim2")] // There is no TriMesh in 2D.
            let mesh1 = None::<Polyline<N>>;
        let poly1 = g1.as_shape::<Polyline<N>>();

        let oriented_a = mesh1.as_ref().filter(|m| m.oriented()).is_some();
        let oriented_b = poly1.as_ref().filter(|p| p.oriented()).is_some();

        if oriented_a || oriented_b {
            match self.proximity {
                Proximity::Disjoint | Proximity::WithinMargin => {
                    // Check if the second shape is completely inside of `g1`.
                    let pt = m2 * g2.any_local_boundary_point();

                    // Handle potential inaccuracies by concluding only if the normal
                    // is not nearly orthogonal to the ray. If none of the 6 directions
                    // result in a satisfactory conclusion, then we peak the one normal
                    // which is the less orthogonal to the ray.

                    #[cfg(feature = "dim2")]
                        let dirs = [
                        Vector::x(),
                        Vector::y(),
                        -Vector::x(),
                        -Vector::y(),
                    ];

                    #[cfg(feature = "dim3")]
                        let dirs = [
                        Vector::x(),
                        Vector::y(),
                        Vector::z(),
                        -Vector::x(),
                        -Vector::y(),
                        -Vector::z(),
                    ];

                    let mut largest_dot = N::zero();
                    let mut best_feature = FeatureId::Unknown;

                    for dir in &dirs {
                        let ray = Ray::new(pt, *dir);
                        let cast_a = mesh1.as_ref().and_then(|m| m.toi_and_normal_with_ray(m1, &ray, false));
                        let cast_b = poly1.as_ref().and_then(|p| p.toi_and_normal_with_ray(m1, &ray, false));

                        if let Some(inter) = cast_a.or(cast_b) {
                            let dot = inter.normal.dot(&dir).abs();
                            if dot > na::convert(1.0e-3) {
                                best_feature = inter.feature;
                                break;
                            } else {
                                if dot > largest_dot {
                                    largest_dot = dot;
                                    best_feature = inter.feature;
                                }
                            }
                        } else {
                            return;
                        }
                    }

                    let is_backface_a = mesh1.filter(|m| m.is_backface(best_feature)).is_some();
                    let is_backface_b = poly1.filter(|p| p.is_backface(best_feature)).is_some();

                    if is_backface_a || is_backface_b {
                        self.proximity = Proximity::Intersecting;
                    }
                }
                Proximity::Intersecting => {}
            }
        }
    }
}

impl<N: Real> ProximityDetector<N> for CompositeShapeShapeProximityDetector<N> {
    fn update(
        &mut self,
        dispatcher: &ProximityDispatcher<N>,
        m1: &Isometry<N>,
        g1: &Shape<N>,
        m2: &Isometry<N>,
        g2: &Shape<N>,
        margin: N,
    ) -> bool {
        if !self.flip {
            if let Some(cs) = g1.as_composite_shape() {
                self.do_update(dispatcher, m1, cs, m2, g2, margin, false);
                self.handle_interior(m1, g1, m2, g2);
                return true;
            }
        } else {
            if let Some(cs) = g2.as_composite_shape() {
                self.do_update(dispatcher, m2, cs, m1, g1, margin, true);
                self.handle_interior(m2, g2, m1, g1);
                return true;
            }
        }

        return false;
    }

    fn proximity(&self) -> Proximity {
        self.proximity
    }
}
