use na::RealField;
use crate::pipeline::{
    BallBallManifoldGenerator, BallConvexPolyhedronManifoldGenerator,
    CompositeShapeCompositeShapeManifoldGenerator, CompositeShapeShapeManifoldGenerator,
    ContactAlgorithm, ContactDispatcher, ConvexPolyhedronConvexPolyhedronManifoldGenerator,
    PlaneBallManifoldGenerator, PlaneConvexPolyhedronManifoldGenerator, CapsuleShapeManifoldGenerator,
    CapsuleCapsuleManifoldGenerator, HeightFieldShapeManifoldGenerator
};
#[cfg(feature = "dim3")]
use crate::pipeline::narrow_phase::TriMeshTriMeshManifoldGenerator;
#[cfg(feature = "dim3")]
use crate::shape::TriMesh;
use crate::shape::{Ball, Plane, Shape, Capsule, HeightField};

/// Collision dispatcher for shapes defined by `ncollide_entities`.
pub struct DefaultContactDispatcher {}

impl DefaultContactDispatcher {
    /// Creates a new basic collision dispatcher.
    pub fn new() -> DefaultContactDispatcher {
        DefaultContactDispatcher {}
    }
}

impl<N: RealField> ContactDispatcher<N> for DefaultContactDispatcher {
    fn get_contact_algorithm(&self, a: &dyn Shape<N>, b: &dyn Shape<N>) -> Option<ContactAlgorithm<N>> {
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
                return Some(Box::new(TriMeshTriMeshManifoldGenerator::<N>::new()));
            }
        }

        if a_is_heightfield || b_is_heightfield {
            return Some(Box::new(HeightFieldShapeManifoldGenerator::<N>::new(b_is_heightfield)));
        } else if a_is_capsule && b_is_capsule {
            Some(Box::new(CapsuleCapsuleManifoldGenerator::<N>::new()))
        } else if a_is_capsule || b_is_capsule {
            Some(Box::new(CapsuleShapeManifoldGenerator::<N>::new(b_is_capsule)))
        } else if a_is_ball && b_is_ball {
            Some(Box::new(BallBallManifoldGenerator::<N>::new()))
        } else if a_is_plane && b_is_ball {
            Some(Box::new(PlaneBallManifoldGenerator::<N>::new(false)))
        } else if a_is_ball && b_is_plane {
            Some(Box::new(PlaneBallManifoldGenerator::<N>::new(true)))
        } else if a_is_plane && b.is_support_map() {
            let gen = PlaneConvexPolyhedronManifoldGenerator::<N>::new(false);
            Some(Box::new(gen))
        } else if b_is_plane && a.is_support_map() {
            let gen = PlaneConvexPolyhedronManifoldGenerator::<N>::new(true);
            Some(Box::new(gen))
        } else if a_is_ball && b.is_convex_polyhedron() {
            let gen = BallConvexPolyhedronManifoldGenerator::<N>::new(false);
            Some(Box::new(gen))
        } else if b_is_ball && a.is_convex_polyhedron() {
            let gen = BallConvexPolyhedronManifoldGenerator::<N>::new(true);
            Some(Box::new(gen))
        } else if a.is_convex_polyhedron() && b.is_convex_polyhedron() {
            let gen = ConvexPolyhedronConvexPolyhedronManifoldGenerator::new();
            Some(Box::new(gen))
        } else if a.is_composite_shape() && b.is_composite_shape() {
            Some(Box::new(
                CompositeShapeCompositeShapeManifoldGenerator::<N>::new(),
            ))
        } else if a.is_composite_shape() {
            Some(Box::new(CompositeShapeShapeManifoldGenerator::<N>::new(
                false,
            )))
        } else if b.is_composite_shape() {
            Some(Box::new(CompositeShapeShapeManifoldGenerator::<N>::new(
                true,
            )))
        } else {
            None
        }
    }
}
