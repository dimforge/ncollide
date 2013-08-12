use std::vec;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::vector_space::VectorSpace;
use bounding_volume::aabb::HasAABB;
use broad::dispatcher::Dispatcher;
use narrow::collision_detector::CollisionDetector;
use narrow::compound_any::{CompoundAABBAny, AnyCompoundAABB};
use contact::Contact;
use geom::compound::CompoundAABB;

/// Collision detector between two `CompoundAABB` geometries.
pub struct CompoundAABBCompoundAABB<N, V, M, S, D, SD> {
    priv sub_detectors: Either<~[CompoundAABBAny<N, V, M, S, D, SD>],
                               ~[AnyCompoundAABB<N, V, M, S, D, SD>]>
}

impl<N, V, M, S, D: Clone, SD> CompoundAABBCompoundAABB<N, V, M, S, D, SD> {
    /// Creates a new `CompoundAABBCompoundAABB` collision detector.
    ///
    /// # Arguments:
    ///     * `dispatcher` - the collision dispatcher to build the collision detectors between the
    ///     compound geometries sub-shapes.
    ///     * `g1` - the first compound geometry to be handled by the detector.
    ///     * `g2` - the second compound geometry to be handled by the detector.
    pub fn new(dispatcher: D,
               g1:         &CompoundAABB<N, V, M, S>,
               g2:         &CompoundAABB<N, V, M, S>)
               -> CompoundAABBCompoundAABB<N, V, M, S, D, SD> {
        let sub_detectors;

        if g1.shapes().len() > g2.shapes().len() {
            // g2 leaves vs g1
            // FIXME: it could be great to avoid the dispatcher.clone()
            sub_detectors = Left(vec::from_fn(g2.shapes().len(),
                                 |_| CompoundAABBAny::new(dispatcher.clone(), g1)))
        }
        else {
            // g1 leaves vs g2
            // FIXME: it could be great to avoid the dispatcher.clone()
            sub_detectors = Right(vec::from_fn(g1.shapes().len(),
                                  |_| AnyCompoundAABB::new(dispatcher.clone(), g2)))
        }

        CompoundAABBCompoundAABB {
            sub_detectors: sub_detectors
        }
    }
}

impl<N:  NumCast + Ord,
     V:  'static + VectorSpace<N> + Norm<N> + Ord + Orderable + Clone,
     M:  Inv + Mul<M, M>,
     S:  HasAABB<N, V, M>,
     D:  Dispatcher<S, SD>,
     SD: CollisionDetector<N, V, M, S, S>>
CollisionDetector<N, V, M, CompoundAABB<N, V, M, S>, CompoundAABB<N, V, M, S>>
for CompoundAABBCompoundAABB<N, V, M, S, D, SD> {
    #[inline]
    fn update(&mut self,
              m1: &M,
              g1: &CompoundAABB<N, V, M, S>,
              m2: &M,
              g2: &CompoundAABB<N, V, M, S>) {
        match self.sub_detectors {
            Left(ref mut ds) => {
                for (i, d) in ds.mut_iter().enumerate() {
                    let child_transform = m2 * *g2.shapes()[i].first_ref();
                    d.update(m1, g1, &child_transform, g2.shapes()[i].second_ref())
                }
            },
            Right(ref mut ds) => {
                for (i, d) in ds.mut_iter().enumerate() {
                    let child_transform = m1 * *g1.shapes()[i].first_ref();
                    d.update(&child_transform, g1.shapes()[i].second_ref(), m2, g2)
                }
            },
        }
    }

    #[inline]
    fn num_coll(&self) -> uint {
        let mut res = 0;

        match self.sub_detectors {
            Left(ref ds) => {
                for d in ds.iter() {
                    res = res + d.num_coll()
                }
            },
            Right(ref ds) => {
                for d in ds.iter() {
                    res = res + d.num_coll()
                }
            }

        }

        res 
    }

    #[inline]
    fn colls(&self, out: &mut ~[Contact<N, V>]) {
        match self.sub_detectors {
            Left(ref ds) => {
                for d in ds.iter() {
                    d.colls(out)
                }
            },
            Right(ref ds) => {
                for d in ds.iter() {
                    d.colls(out)
                }
            }

        }
    }

}
