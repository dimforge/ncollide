use std::num::{Zero, One};
use nalgebra::traits::basis::Basis;
use nalgebra::traits::cross::Cross;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::rotation;
use nalgebra::traits::rotation::{Rotate, Rotation};
use nalgebra::traits::scalar_op::ScalarMul;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::translation::{Translation, Translatable};
use nalgebra::traits::vector_space::VectorSpace;
use narrow::collision_detector::CollisionDetector;
use narrow::incremental_contact_manifold_generator::IncrementalContactManifoldGenerator;
use contact::Contact;
use geom::implicit::Implicit;

/// This is an hybrid contact manifold genarator. Whenever a new contact is detected (i.e. when the
/// current manifold is empty) a full manifold is generated. Then, the manifold is incrementally
/// updated by the `IncrementalContactManifoldGenerator`.
pub struct OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
    priv sub_detector: IncrementalContactManifoldGenerator<CD, N, LV>
}

impl<CD, N, LV, AV, M> OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
    /// Creates a new one shot contact manifold generator.
    pub fn new(cd: CD) -> OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
        OneShotContactManifoldGenerator {
            sub_detector: IncrementalContactManifoldGenerator::new(cd)
        }
    }
}

impl<CD: CollisionDetector<N, LV, UnsafeTransformedRef<N, M, G1>, G2>,
     G1: Transform<LV> + Translation<LV>,
     G2: Transform<LV>,
     N:  Clone + DivisionRing + Ord + NumCast,
     LV: Clone + VectorSpace<N> + Cross<AV> + Dot<N> + Norm<N> + ApproxEq<N> + Dim + Basis,
     AV: ScalarMul<N> + Neg<AV>,
     M:  Rotation<AV> + Transform<LV> + Translation<LV> + Translatable<LV, M> + One>
CollisionDetector<N, LV, G1, G2> for OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
    fn update(&mut self, g1: &G1, g2: &G2) {
        let wrapped_g1 = UnsafeTransformedRef::new(g1);

        if self.sub_detector.num_coll() == 0 {
            // do the one-shot manifold generation
            let pos_g1 = g1.translation();

            match self.sub_detector.get_sub_collision(&wrapped_g1, g2) {
                Some(coll) => {
                    do coll.normal.orthonormal_subspace_basis |b| {
                        let mut rot_axis: AV = coll.normal.cross(&b);

                        // first perturbation
                        rot_axis.scalar_mul_inplace(&NumCast::from(0.01));

                        let mut rot_mat: M =
                            rotation::rotated_wrt_point(&One::one::<M>(), &rot_axis, &pos_g1);

                        let rotated_g1 = UnsafeTransformedRef::new_with_transform(rot_mat, g1);

                        self.sub_detector.add_new_contacts(&rotated_g1, g2);

                        // second perturbation (opposite direction)
                        rot_axis = -rot_axis;

                        rot_mat = rotation::rotated_wrt_point(&One::one::<M>(), &rot_axis, &pos_g1);

                        let rotated_g1 = UnsafeTransformedRef::new_with_transform(rot_mat, g1);

                        self.sub_detector.add_new_contacts(&rotated_g1, g2);
                    }

                    self.sub_detector.update_contacts(&wrapped_g1, g2);
                },
                None => { } // no collision
            }
        }
        else {
            // otherwise, let the incremental manifold do its job
            self.sub_detector.update(&wrapped_g1, g2)
        }
    }

    #[inline]
    fn num_coll(&self) -> uint {
        self.sub_detector.num_coll()
    }

    #[inline]
    fn colls(&mut self, out_colls: &mut ~[Contact<N, LV>]) {
        self.sub_detector.colls(out_colls)
    }
}

/// Dont use this type. It is full of magic, dragons and apples.
/// It is like the `Transformed` type but with its lifetime parameter. To remove the lifetime
/// parameter, it simply store an unsafe pointer, which is very bad. However, discarding this
/// lifetime makes it easier to use by the `OneShotContactManifoldGenerator`. This avoids having a
/// weird lifetime on the sub-detector type bound.
///
/// The point is that, if you use this type, make sure any instance of it doesnt outlives the
/// stored geometry. In other words, dont use that.
///
/// Maybe at some point there will be an obvious solution. Maybe when lifetimes other than 'self
/// will be allowed on trait bounds? We’ll see.
///
/// In the mean time I just mark it as: FIXME.
pub struct UnsafeTransformedRef<N, M, G> {
    priv t: Option<M>,
    priv g: *G
}

// implementations for TransformedRef
impl<N, M, G> UnsafeTransformedRef<N, M, G> {
    /// Creates a transformed geometry with its transform set to identity.
    #[inline]
    pub fn new(geometry: &G) -> UnsafeTransformedRef<N, M, G> {
        UnsafeTransformedRef { t: None, g: geometry }
    }

    /// Creates a transformed geometry with at custom transform.
    #[inline]
    pub fn new_with_transform(transform: M, geometry: &G) -> UnsafeTransformedRef<N, M, G> {
        UnsafeTransformedRef { t: Some(transform), g: geometry }
    }

    /// Resets this geometry transform to the identity.
    #[inline]
    pub fn reset_to_identity(&mut self) {
        self.t = None;
    }

    /// Changes this geometry transform. Note that the case of the new transform being the identity
    /// is not detected. Use `reset_to_identity` to trigger the identity related optimization.
    #[inline]
    pub fn set_transform(&mut self, t: M) {
        self.t = Some(t)
    }
}

impl<G: Implicit<V>, M: Rotate<V> + Transform<V>, V: Clone, N>
Implicit<V> for UnsafeTransformedRef<N, M, G> { // FIXME: rename this UnsafeRotatedRef
    #[inline]
    fn support_point(&self, dir: &V) -> V {
        unsafe {
            match self.t {
                None        => (*self.g).support_point(dir),
                Some(ref t) => t.transform_vec(&(*self.g).support_point(&t.inv_rotate(dir)))
            }
        }
    }
}

impl<G: Translation<V>, M: Translation<V> + One, N, V: Zero>
Translation<V> for UnsafeTransformedRef<N, M, G> {
    // FIXME take the sub geometry in account?
    #[inline]
    fn translation(&self) -> V {
        unsafe {
            match self.t {
                None    => Zero::zero(),
                Some(_) => (*self.g).translation()
            }
        }
    }

    #[inline]
    fn inv_translation(&self) -> V {
        unsafe {
            match self.t {
                None    => Zero::zero(),
                Some(_) => (*self.g).inv_translation()
            }
        }
    }

    #[inline]
    fn translate_by(&mut self, v: &V) {
        match self.t {
            None => {
                let mut newt: M = One::one();

                newt.translate_by(v);
                self.t = Some(newt);
            },
            Some(ref mut t) => t.translate_by(v)
        }
    }
}

impl<G: Transform<V>, M: Transform<V>, N, V: Clone> Transform<V> for UnsafeTransformedRef<N, M, G> {
    #[inline]
    fn transform_vec(&self, v: &V) -> V {
        unsafe {
            match self.t {
                None        => (*self.g).transform_vec(v),
                Some(ref t) => t.transform_vec(&(*self.g).transform_vec(v))
            }
        }
    }

    #[inline]
    fn inv_transform(&self, v: &V) -> V {
        unsafe {
            match self.t {
                None        => (*self.g).inv_transform(v),
                Some(ref t) => (*self.g).inv_transform(&t.inv_transform(v))
            }
        }
    }
}
