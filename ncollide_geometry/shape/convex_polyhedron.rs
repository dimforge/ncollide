use na::Unit;
use bounding_volume::PolyhedralCone;
use shape::{ConvexPolyface, FeatureId, SupportMap};
use math::Point;

pub trait ConvexPolyhedron<P: Point, M>: SupportMap<P, M> {
    fn vertex(&self, id: FeatureId) -> P;
    fn edge(&self, id: FeatureId) -> (P, P, FeatureId, FeatureId);
    fn face(&self, id: FeatureId, face: &mut ConvexPolyface<P>);

    fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<P>;

    fn support_face_toward(
        &self,
        transform: &M,
        dir: &Unit<P::Vector>,
        out: &mut ConvexPolyface<P>,
    );

    fn support_feature_toward(
        &self,
        transform: &M,
        dir: &Unit<P::Vector>,
        _angle: P::Real,
        out: &mut ConvexPolyface<P>,
    );
}
