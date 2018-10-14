use na::Real;
use pipeline::narrow_phase::{
    BallBallManifoldGenerator, BallConvexPolyhedronManifoldGenerator,
    CompositeShapeCompositeShapeManifoldGenerator, CompositeShapeShapeManifoldGenerator,
    ContactAlgorithm, ContactDispatcher, ConvexPolyhedronConvexPolyhedronManifoldGenerator,
    PlaneBallManifoldGenerator, PlaneConvexPolyhedronManifoldGenerator,
};
#[cfg(feature = "dim3")]
use pipeline::narrow_phase::{TriMeshShapeManifoldGenerator, TriMeshTriMeshManifoldGenerator};
#[cfg(feature = "dim3")]
use shape::TriMesh;
use shape::{Ball, Plane, Shape};

/// Collision dispatcher for shapes defined by `ncollide_entities`.
pub struct DefaultContactDispatcher {}

impl DefaultContactDispatcher {
    /// Creates a new basic collision dispatcher.
    pub fn new() -> DefaultContactDispatcher {
        DefaultContactDispatcher {}
    }
}

impl<N: Real> ContactDispatcher<N> for DefaultContactDispatcher {
    fn get_contact_algorithm(&self, a: &Shape<N>, b: &Shape<N>) -> Option<ContactAlgorithm<N>> {
        let a_is_ball = a.is_shape::<Ball<N>>();
        let b_is_ball = b.is_shape::<Ball<N>>();
        let a_is_plane = a.is_shape::<Plane<N>>();
        let b_is_plane = b.is_shape::<Plane<N>>();

        #[cfg(feature = "dim3")]
        {
            let a_is_trimesh = a.is_shape::<TriMesh<N>>();
            let b_is_trimesh = b.is_shape::<TriMesh<N>>();

            if a_is_trimesh && b_is_trimesh {
                return Some(Box::new(TriMeshTriMeshManifoldGenerator::<N>::new()));
            } else if a_is_trimesh {
                return Some(Box::new(TriMeshShapeManifoldGenerator::<N>::new(false)));
            } else if b_is_trimesh {
                return Some(Box::new(TriMeshShapeManifoldGenerator::<N>::new(true)));
            }
        }

        if a_is_ball && b_is_ball {
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
