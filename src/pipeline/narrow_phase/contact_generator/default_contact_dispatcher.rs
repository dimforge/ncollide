#[cfg(feature = "dim3")]
use crate::pipeline::narrow_phase::TriMeshTriMeshManifoldGenerator;
use crate::pipeline::{
    BallBallManifoldGenerator, BallConvexPolyhedronManifoldGenerator,
    CapsuleCapsuleManifoldGenerator, CapsuleShapeManifoldGenerator,
    CompositeShapeCompositeShapeManifoldGenerator, CompositeShapeShapeManifoldGenerator,
    ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator,
    ConvexPolyhedronConvexPolyhedronManifoldGenerator, HeightFieldShapeManifoldGenerator,
    PlaneBallManifoldGenerator, PlaneConvexPolyhedronManifoldGenerator,
};
#[cfg(feature = "dim3")]
use crate::shape::TriMesh;
use crate::shape::{Ball, Capsule, HeightField, Plane, Shape};
use na::RealField;

/// Collision dispatcher for shapes defined by `ncollide_entities`.
pub struct DefaultContactDispatcher {}

impl DefaultContactDispatcher {
    /// Creates a new basic collision dispatcher.
    pub fn new() -> DefaultContactDispatcher {
        DefaultContactDispatcher {}
    }
}

impl<N: RealField> ContactDispatcher<N> for DefaultContactDispatcher {
    fn get_flipped_contact_algorithm(
        &self,
        flip: bool,
        a: &dyn Shape<N>,
        b: &dyn Shape<N>,
    ) -> Option<ContactAlgorithm<N>> {
        let a_is_ball = a.is_shape::<Ball<N>>();
        let b_is_ball = b.is_shape::<Ball<N>>();
        let a_is_plane = a.is_shape::<Plane<N>>();
        let b_is_plane = b.is_shape::<Plane<N>>();
        let a_is_capsule = a.is_shape::<Capsule<N>>();
        let b_is_capsule = b.is_shape::<Capsule<N>>();
        let a_is_heightfield = a.is_shape::<HeightField<N>>();
        let b_is_heightfield = b.is_shape::<HeightField<N>>();

        #[cfg(feature = "dim3")]
        {
            let a_is_trimesh = a.is_shape::<TriMesh<N>>();
            let b_is_trimesh = b.is_shape::<TriMesh<N>>();

            if a_is_trimesh && b_is_trimesh {
                return Some(wrap(flip, TriMeshTriMeshManifoldGenerator::<N>::new()));
            }
        }

        if a_is_heightfield || b_is_heightfield {
            return Some(wrap(
                flip ^ b_is_heightfield,
                HeightFieldShapeManifoldGenerator::<N>::new(),
            ));
        } else if a_is_capsule && b_is_capsule {
            Some(wrap(flip, CapsuleCapsuleManifoldGenerator::<N>::new()))
        } else if a_is_capsule || b_is_capsule {
            Some(wrap(
                flip,
                CapsuleShapeManifoldGenerator::<N>::new(b_is_capsule),
            ))
        } else if a_is_ball && b_is_ball {
            Some(wrap(flip, BallBallManifoldGenerator::<N>::new()))
        } else if a_is_plane && b_is_ball {
            Some(wrap(flip, PlaneBallManifoldGenerator::<N>::new()))
        } else if a_is_ball && b_is_plane {
            Some(wrap(!flip, PlaneBallManifoldGenerator::<N>::new()))
        } else if a_is_plane && b.is_support_map() {
            let gen = PlaneConvexPolyhedronManifoldGenerator::<N>::new(false);
            Some(wrap(flip, gen))
        } else if b_is_plane && a.is_support_map() {
            let gen = PlaneConvexPolyhedronManifoldGenerator::<N>::new(true);
            Some(wrap(flip, gen))
        } else if a_is_ball && b.is_convex_polyhedron() {
            let gen = BallConvexPolyhedronManifoldGenerator::<N>::new(false);
            Some(wrap(flip, gen))
        } else if b_is_ball && a.is_convex_polyhedron() {
            let gen = BallConvexPolyhedronManifoldGenerator::<N>::new(true);
            Some(wrap(flip, gen))
        } else if a.is_convex_polyhedron() && b.is_convex_polyhedron() {
            let gen = ConvexPolyhedronConvexPolyhedronManifoldGenerator::new();
            Some(wrap(flip, gen))
        } else if a.is_composite_shape() && b.is_composite_shape() {
            Some(wrap(
                flip,
                CompositeShapeCompositeShapeManifoldGenerator::<N>::new(),
            ))
        } else if a.is_composite_shape() {
            Some(wrap(
                flip,
                CompositeShapeShapeManifoldGenerator::<N>::new(false),
            ))
        } else if b.is_composite_shape() {
            Some(wrap(
                flip,
                CompositeShapeShapeManifoldGenerator::<N>::new(true),
            ))
        } else {
            None
        }
    }
}

fn wrap<N: RealField, T: ContactManifoldGenerator<N>>(flip: bool, x: T) -> ContactAlgorithm<N> {
    if flip {
        Box::new(x.flip())
    } else {
        Box::new(x)
    }
}
