use std::vec;
use nalgebra::mat::{Translation, Inv};
use nalgebra::vec::AlgebraicVecExt;
use bounding_volume::HasAABB;
use broad::Dispatcher;
use narrow::{CollisionDetector, CompoundAABBAny, AnyCompoundAABB};
use contact::Contact;
use geom::CompoundAABB;

#[deriving(Encodable, Decodable)]
enum SubDetectors<N, V, M, S, D, SD> {
    CA(~[CompoundAABBAny<N, V, M, S, D, SD>]),
    AC(~[AnyCompoundAABB<N, V, M, S, D, SD>])
}
/// Collision detector between two `CompoundAABB` geometries.
#[deriving(Encodable, Decodable)]
pub struct CompoundAABBCompoundAABB<N, V, M, S, D, SD> {
    priv sub_detectors: SubDetectors<N, V, M, S, D, SD>
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
            sub_detectors = CA(vec::from_fn(g2.shapes().len(),
                               |_| CompoundAABBAny::new(dispatcher.clone(), g1)))
        }
        else {
            // g1 leaves vs g2
            // FIXME: it could be great to avoid the dispatcher.clone()
            sub_detectors = AC(vec::from_fn(g1.shapes().len(),
                               |_| AnyCompoundAABB::new(dispatcher.clone(), g2)))
        }

        CompoundAABBCompoundAABB {
            sub_detectors: sub_detectors
        }
    }
}

impl<N:  Algebraic + Primitive + Orderable + ToStr,
     V:  'static + Clone + AlgebraicVecExt<N> + ToStr,
     M:  Inv + Mul<M, M> + Translation<V>,
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
            CA(ref mut ds) => {
                for (i, d) in ds.mut_iter().enumerate() {
                    let child_transform = m2 * *g2.shapes()[i].first_ref();
                    d.update(m1, g1, &child_transform, g2.shapes()[i].second_ref())
                }
            },
            AC(ref mut ds) => {
                for (i, d) in ds.mut_iter().enumerate() {
                    let child_transform = m1 * *g1.shapes()[i].first_ref();
                    d.update(&child_transform, g1.shapes()[i].second_ref(), m2, g2)
                }
            },
        }
    }

    #[inline]
    fn num_colls(&self) -> uint {
        let mut res = 0;

        match self.sub_detectors {
            CA(ref ds) => {
                for d in ds.iter() {
                    res = res + d.num_colls()
                }
            },
            AC(ref ds) => {
                for d in ds.iter() {
                    res = res + d.num_colls()
                }
            }

        }

        res 
    }

    #[inline]
    fn colls(&self, out: &mut ~[Contact<N, V>]) {
        match self.sub_detectors {
            CA(ref ds) => {
                for d in ds.iter() {
                    d.colls(out)
                }
            },
            AC(ref ds) => {
                for d in ds.iter() {
                    d.colls(out)
                }
            }

        }
    }

    #[inline]
    fn toi(_: Option<CompoundAABBCompoundAABB<N, V, M, S, D, SD>>,
           _: &M,
           _: &V,
           _: &N,
           _: &CompoundAABB<N, V, M, S>,
           _: &M,
           _: &CompoundAABB<N, V, M, S>)
           -> Option<N> {
        fail!("TOI for compound_compound is not yet implemented.")
    }
}
