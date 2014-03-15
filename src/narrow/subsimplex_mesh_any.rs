use std::vec;
use std::vec_ng::Vec;
use nalgebra::na::{Translation, Inv, AlgebraicVecExt};
use nalgebra::na;
use util::hash_map::HashMap;
use util::hash::UintTWHash;
use geom::subsimplex_mesh::{Subsimplex, SubsimplexMesh};
use bounding_volume::{BoundingVolume, HasAABB};
use broad::Dispatcher;
use narrow::CollisionDetector;
use contact::Contact;
use partitioning::bvt_visitor::BoundingVolumeInterferencesCollector;

pub struct SubsimplexMeshAny<Scalar, Vector, Matrix, G, D, SD> {
    priv dispatcher:    D,
    priv sub_detectors: HashMap<uint, SD, UintTWHash>,
    priv to_delete:     Vec<uint>,
    priv interferences: Vec<uint>
}

pub struct AnySubsimplexMesh<Scalar, Vector, Matrix, G, D, SD> {
    priv sub_detector: SubsimplexMeshAny<Scalar, Vector, Matrix, G, D, SD>
}

impl<'a,
     Scalar:  Algebraic + Primitive + Orderable,
     Vector:  AlgebraicVecExt<Scalar> + Clone,
     Matrix:  Inv + Mul<Matrix, Matrix> + Translation<Vector>,
     G:  HasAABB<Scalar, Vector, Matrix>,
     D:  Dispatcher<Subsimplex<'a, Scalar, Vector>, G, SD>,
     SD: CollisionDetector<Scalar, Vector, Matrix, Subsimplex<'a, Scalar, Vector>, G>>
SubsimplexMeshAny<Scalar, Vector, Matrix, G, D, SD> {
    fn do_update(&mut self, m1: &Matrix, g1: &SubsimplexMesh<Scalar, Vector>, m2: &Matrix, g2: &G) {
        // Find new collisions
        let ls_m2    = na::inv(m1).expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = g2.aabb(&ls_m2);

        {
            let mut visitor = BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut self.interferences);
            g1.bvt().visit(&mut visitor);
        }

        for i in self.interferences.iter() {
            let g1 = g1.subsimplex_at(*i);

            if self.dispatcher.is_valid(&g1, g2) {
                self.sub_detectors.insert_or_replace(*i, self.dispatcher.dispatch(&g1, g2), false);
            }
        }

        self.interferences.clear();

        // Update all collisions
        for detector in self.sub_detectors.elements_mut().iter() {
            let key = detector.key;
            if ls_aabb2.intersects(&g1.bounding_volumes()[key]) {
                let g1 = g1.subsimplex_at(key);

                detector.value.update(m1, &g1, m2, g2);
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

// impl<Scalar
//      Vector,
//      Matrix,
//      G,
//      D,
//      SD>
// CollisionDetector<Scalar, Vector, Matrix, SubsimplexMesh<Scalar, Vector>, G>
// for  SubsimplexMeshAny<Scalar, Vector, Matrix, G, D, SD> {
// }
